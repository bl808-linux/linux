// SPDX-License-Identifier: GPL-2.0-only
/* BL808 UART clock path. Shared PLLs and mux selections are firmware-owned. */
#include <linux/bitfield.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/spinlock.h>

#include <dt-bindings/clock/bflb,bl808.h>

#define MM_CPU_CTRL	0x00
#define MM_CPU_DIV	0x04
#define MM_PERI_CTRL	0x10
#define MM_PERI_RESET	0x44
#define MM_UART_GATE	BIT(16)
#define MM_UART_RESET	BIT(2)
#define MM_DMA_RESET	BIT(1)

enum bl808_kind {
	BL808_CPU_PLL,
	BL808_WIFI_PLL,
	BL808_AUDIO_PLL,
	BL808_PLL_MUX,
	BL808_MM,
};

struct bl808_controller;

struct bl808_clock {
	struct clk_hw hw;
	struct bl808_controller *controller;
	unsigned int index;
};

struct bl808_controller {
	void __iomem *base;
	enum bl808_kind kind;
	/* Serialize UART gate and reset register updates. */
	spinlock_t lock;
	struct reset_controller_dev reset;
	struct bl808_clock clocks[3];
};

#define to_bl808_clock(_hw) container_of(_hw, struct bl808_clock, hw)

static u8 bl808_pll_parent(struct clk_hw *hw)
{
	struct bl808_controller *c = to_bl808_clock(hw)->controller;

	return FIELD_GET(GENMASK(17, 16), readl(c->base + 4)) == 3;
}

static unsigned long bl808_pll_rate(struct clk_hw *hw, unsigned long parent)
{
	struct bl808_controller *c = to_bl808_clock(hw)->controller;
	bool wifi = c->kind == BL808_WIFI_PLL;
	u32 cfg = readl(c->base + 4), sdm = readl(c->base + 24);
	u32 ref = FIELD_GET(GENMASK(17, 16), cfg);
	u32 div = FIELD_GET(GENMASK(11, 8), cfg);
	u32 shift = wifi ? 19 : 11;
	u32 value = sdm & (wifi ? GENMASK(25, 0) : GENMASK(18, 0));
	u32 bypass = wifi ? BIT(28) : BIT(24);

	if (!div || !parent || (ref != (wifi ? 1 : 0) && ref != 3))
		return 0;
	if ((sdm & bypass) && (value & ((1U << shift) - 1)))
		return 0;
	return div_u64((u64)parent * value, div << shift);
}

static int bl808_pll_enable(struct clk_hw *hw)
{
	struct bl808_controller *c = to_bl808_clock(hw)->controller;

	/* Never power-cycle a root shared with the other cores. */
	return readl(c->base) & BIT(10) ? 0 : -EIO;
}

static const struct clk_ops bl808_pll_ops = {
	.get_parent = bl808_pll_parent,
	.recalc_rate = bl808_pll_rate,
	.enable = bl808_pll_enable,
};

static u8 bl808_mux_parent(struct clk_hw *hw)
{
	struct bl808_clock *clk = to_bl808_clock(hw);

	return !!(readl(clk->controller->base) & BIT(clk->index));
}

static unsigned long bl808_mux_rate(struct clk_hw *hw, unsigned long parent)
{
	struct bl808_clock *clk = to_bl808_clock(hw);
	bool alternate = bl808_mux_parent(hw);

	if (clk->index == BL808_MM_PLL_160M)
		return alternate ? div_u64((u64)parent * 2, 5) : parent / 6;
	return parent / (alternate ? 2 : 4);
}

static const struct clk_ops bl808_mux_ops = {
	.get_parent = bl808_mux_parent,
	.recalc_rate = bl808_mux_rate,
};

static u8 bl808_mm_parent(struct clk_hw *hw)
{
	struct bl808_clock *clk = to_bl808_clock(hw);
	u32 value = readl(clk->controller->base + MM_CPU_CTRL);

	switch (clk->index) {
	case BL808_MM_XCLK:
		return !!(value & BIT(10));
	case BL808_MM_PBCLK:
		return FIELD_GET(GENMASK(14, 13), value);
	default:
		return FIELD_GET(GENMASK(5, 4), value);
	}
}

static unsigned long bl808_mm_rate(struct clk_hw *hw, unsigned long parent)
{
	struct bl808_clock *clk = to_bl808_clock(hw);
	void __iomem *base = clk->controller->base;
	u32 div;

	switch (clk->index) {
	case BL808_MM_XCLK:
		return parent;
	case BL808_MM_PBCLK:
		div = FIELD_GET(GENMASK(31, 24), readl(base + MM_CPU_DIV));
		break;
	default:
		div = FIELD_GET(GENMASK(19, 17), readl(base + MM_PERI_CTRL));
		break;
	}
	return parent / (div + 1);
}

static void bl808_uart_gate(struct clk_hw *hw, bool enabled)
{
	struct bl808_controller *c = to_bl808_clock(hw)->controller;
	unsigned long flags;
	u32 value;

	spin_lock_irqsave(&c->lock, flags);
	value = readl(c->base + MM_PERI_CTRL) & ~MM_UART_GATE;
	writel(value | (enabled ? MM_UART_GATE : 0), c->base + MM_PERI_CTRL);
	spin_unlock_irqrestore(&c->lock, flags);
}

static int bl808_uart_enable(struct clk_hw *hw)
{
	bl808_uart_gate(hw, true);
	return 0;
}

static void bl808_uart_disable(struct clk_hw *hw)
{
	bl808_uart_gate(hw, false);
}

static int bl808_uart_is_enabled(struct clk_hw *hw)
{
	struct bl808_controller *c = to_bl808_clock(hw)->controller;

	return !!(readl(c->base + MM_PERI_CTRL) & MM_UART_GATE);
}

static const struct clk_ops bl808_mm_ops = {
	.get_parent = bl808_mm_parent,
	.recalc_rate = bl808_mm_rate,
};

static const struct clk_ops bl808_uart_ops = {
	.get_parent = bl808_mm_parent,
	.recalc_rate = bl808_mm_rate,
	.enable = bl808_uart_enable,
	.disable = bl808_uart_disable,
	.is_enabled = bl808_uart_is_enabled,
};

static int bl808_reset_update(struct reset_controller_dev *rc,
			      unsigned long id, bool asserted)
{
	struct bl808_controller *c = container_of(rc, struct bl808_controller, reset);
	unsigned long flags;
	u32 value, mask;

	if (id > BL808_MM_RST_DMA2)
		return -EINVAL;
	mask = id == BL808_MM_RST_UART3 ? MM_UART_RESET : MM_DMA_RESET;
	spin_lock_irqsave(&c->lock, flags);
	value = readl(c->base + MM_PERI_RESET) & ~mask;
	writel(value | (asserted ? mask : 0), c->base + MM_PERI_RESET);
	readl(c->base + MM_PERI_RESET);
	spin_unlock_irqrestore(&c->lock, flags);
	return 0;
}

static int bl808_reset_assert(struct reset_controller_dev *rc, unsigned long id)
{
	return bl808_reset_update(rc, id, true);
}

static int bl808_reset_deassert(struct reset_controller_dev *rc, unsigned long id)
{
	return bl808_reset_update(rc, id, false);
}

static int bl808_reset_status(struct reset_controller_dev *rc, unsigned long id)
{
	struct bl808_controller *c = container_of(rc, struct bl808_controller, reset);

	if (id > BL808_MM_RST_DMA2)
		return -EINVAL;
	return !!(readl(c->base + MM_PERI_RESET) &
		  (id == BL808_MM_RST_UART3 ? MM_UART_RESET : MM_DMA_RESET));
}

static const struct reset_control_ops bl808_reset_ops = {
	.assert = bl808_reset_assert,
	.deassert = bl808_reset_deassert,
	.status = bl808_reset_status,
};

static int bl808_register_clock(struct device *dev, struct bl808_controller *c,
				unsigned int index, const char *suffix,
				const struct clk_ops *ops,
				const struct clk_parent_data *parents,
				unsigned int num_parents)
{
	struct clk_init_data init = {
		.ops = ops,
		.parent_data = parents,
		.num_parents = num_parents,
		.flags = CLK_GET_RATE_NOCACHE,
	};
	struct bl808_clock *clk = &c->clocks[index];

	init.name = devm_kasprintf(dev, GFP_KERNEL, "%s-%s", dev_name(dev), suffix);
	if (!init.name)
		return -ENOMEM;
	clk->controller = c;
	clk->index = index;
	clk->hw.init = &init;
	return devm_clk_hw_register(dev, &clk->hw);
}

static int bl808_clock_probe(struct platform_device *pdev)
{
	static const struct clk_parent_data pll_parents[] = {
		{ .fw_name = "xtal" }, { .fw_name = "rc32m" },
	};
	static const struct clk_parent_data mux160_parents[] = {
		{ .fw_name = "wifi" }, { .fw_name = "cpu" },
	};
	static const struct clk_parent_data mux240_parents[] = {
		{ .fw_name = "wifi" }, { .fw_name = "audio" },
	};
	struct clk_parent_data bus_parents[4] = {};
	struct clk_parent_data uart_parents[4] = {};
	struct clk_parent_data xclk_parents[] = {
		{ .fw_name = "rc32m" }, { .fw_name = "xtal" },
	};
	struct device *dev = &pdev->dev;
	struct clk_hw_onecell_data *data;
	struct bl808_controller *c;
	unsigned int count, i;
	int ret;

	c = devm_kzalloc(dev, sizeof(*c), GFP_KERNEL);
	if (!c)
		return -ENOMEM;
	c->kind = (unsigned long)of_device_get_match_data(dev);
	c->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(c->base))
		return PTR_ERR(c->base);
	spin_lock_init(&c->lock);

	if (c->kind <= BL808_AUDIO_PLL) {
		ret = bl808_register_clock(dev, c, 0, "pll", &bl808_pll_ops,
					   pll_parents, ARRAY_SIZE(pll_parents));
		if (ret)
			return ret;
		return devm_of_clk_add_hw_provider(dev, of_clk_hw_simple_get,
						 &c->clocks[0].hw);
	}

	count = c->kind == BL808_PLL_MUX ? 2 : 3;
	data = devm_kzalloc(dev, struct_size(data, hws, count), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	data->num = count;
	if (c->kind == BL808_PLL_MUX) {
		ret = bl808_register_clock(dev, c, BL808_MM_PLL_160M, "mux160",
					   &bl808_mux_ops, mux160_parents, 2);
		if (ret)
			return ret;
		ret = bl808_register_clock(dev, c, BL808_MM_PLL_240M, "mux240",
					   &bl808_mux_ops, mux240_parents, 2);
	} else {
		ret = bl808_register_clock(dev, c, BL808_MM_XCLK, "xclk",
					   &bl808_mm_ops, xclk_parents, 2);
		if (ret)
			return ret;
		bus_parents[0].hw = &c->clocks[BL808_MM_XCLK].hw;
		bus_parents[1].hw = &c->clocks[BL808_MM_XCLK].hw;
		bus_parents[2].fw_name = "pll160";
		bus_parents[3].fw_name = "pll240";
		ret = bl808_register_clock(dev, c, BL808_MM_PBCLK, "pbclk",
					   &bl808_mm_ops, bus_parents, 4);
		if (ret)
			return ret;
		uart_parents[0].hw = &c->clocks[BL808_MM_PBCLK].hw;
		uart_parents[1].fw_name = "pll160";
		uart_parents[2].hw = &c->clocks[BL808_MM_XCLK].hw;
		/* Encoding 3 is reserved and deliberately has no parent. */
		uart_parents[3].index = -1;
		ret = bl808_register_clock(dev, c, BL808_MM_UART3, "uart3",
					   &bl808_uart_ops, uart_parents, 4);
		if (ret)
			return ret;
		c->reset.ops = &bl808_reset_ops;
		c->reset.owner = THIS_MODULE;
		c->reset.of_node = dev->of_node;
		c->reset.nr_resets = 2;
		ret = devm_reset_controller_register(dev, &c->reset);
	}
	if (ret)
		return ret;
	for (i = 0; i < count; i++)
		data->hws[i] = &c->clocks[i].hw;
	return devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get, data);
}

static const struct of_device_id bl808_clock_match[] = {
	{ .compatible = "bflb,bl808-cpu-pll", .data = (void *)BL808_CPU_PLL },
	{ .compatible = "bflb,bl808-wifi-pll", .data = (void *)BL808_WIFI_PLL },
	{ .compatible = "bflb,bl808-audio-pll", .data = (void *)BL808_AUDIO_PLL },
	{ .compatible = "bflb,bl808-mm-pll-mux", .data = (void *)BL808_PLL_MUX },
	{ .compatible = "bflb,bl808-mm-clock", .data = (void *)BL808_MM },
	{ }
};

static struct platform_driver bl808_clock_driver = {
	.probe = bl808_clock_probe,
	.driver = {
		.name = "bl808-clock",
		.of_match_table = bl808_clock_match,
	},
};
builtin_platform_driver(bl808_clock_driver);
