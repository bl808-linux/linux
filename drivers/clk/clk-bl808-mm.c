// SPDX-License-Identifier: GPL-2.0-only
/* SPI1 leaf clock/reset only. Shared clock roots remain firmware-owned. */
#include <linux/bitfield.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>

#define MM_CPU_CTRL 0x00
#define MM_PERI_CTRL 0x10
#define MM_RESET 0x44
#define SPI_XCLK BIT(7)
#define XCLK_XTAL BIT(10)
#define SPI_GATE BIT(23)
#define SPI_DIV GENMASK(31, 24)
#define SPI_RESET BIT(8)
struct bl808_mm {
	void __iomem *base;
	/* Serialize shared leaf clock and reset register updates. */
	spinlock_t lock;
	struct clk_hw hw;
	struct reset_controller_dev reset;
};
#define to_bl808_mm(h) container_of(h, struct bl808_mm, hw)
static unsigned long bl808_spi_rate(struct clk_hw *hw, unsigned long parent)
{
	struct bl808_mm *m = to_bl808_mm(hw);
	u32 source = readl(m->base + MM_CPU_CTRL);

	if ((source & (SPI_XCLK | XCLK_XTAL)) != (SPI_XCLK | XCLK_XTAL))
		return 0;
	return parent / (FIELD_GET(SPI_DIV, readl(m->base + MM_PERI_CTRL)) + 1);
}

static int bl808_spi_enable(struct clk_hw *hw)
{
	struct bl808_mm *m = to_bl808_mm(hw);
	unsigned long flags;
	u32 value;

	/* Changing the common XCLK would also change the live UART3 clock. */
	if (!(readl(m->base + MM_CPU_CTRL) & XCLK_XTAL))
		return -EINVAL;
	spin_lock_irqsave(&m->lock, flags);
	value = readl(m->base + MM_CPU_CTRL);
	writel(value | SPI_XCLK, m->base + MM_CPU_CTRL);
	value = readl(m->base + MM_PERI_CTRL);
	writel((value & ~SPI_DIV) | SPI_GATE, m->base + MM_PERI_CTRL);
	spin_unlock_irqrestore(&m->lock, flags);
	return 0;
}

static void bl808_spi_disable(struct clk_hw *hw)
{
	struct bl808_mm *m = to_bl808_mm(hw);
	unsigned long flags;

	spin_lock_irqsave(&m->lock, flags);
	writel(readl(m->base + MM_PERI_CTRL) & ~SPI_GATE,
	       m->base + MM_PERI_CTRL);
	spin_unlock_irqrestore(&m->lock, flags);
}

static int bl808_spi_enabled(struct clk_hw *hw)
{
	return !!(readl(to_bl808_mm(hw)->base + MM_PERI_CTRL) & SPI_GATE);
}

static const struct clk_ops bl808_spi_clock_ops = {
	.enable = bl808_spi_enable,
	.disable = bl808_spi_disable,
	.is_enabled = bl808_spi_enabled,
	.recalc_rate = bl808_spi_rate,
};

static int bl808_spi_reset_set(struct reset_controller_dev *rc,
			       unsigned long id, bool asserted)
{
	struct bl808_mm *m = container_of(rc, struct bl808_mm, reset);
	unsigned long flags;
	u32 value;

	spin_lock_irqsave(&m->lock, flags);
	value = readl(m->base + MM_RESET) & ~SPI_RESET;
	writel(value | (asserted ? SPI_RESET : 0), m->base + MM_RESET);
	readl(m->base + MM_RESET);
	spin_unlock_irqrestore(&m->lock, flags);
	return 0;
}

static int bl808_spi_assert(struct reset_controller_dev *rc, unsigned long id)
{
	return bl808_spi_reset_set(rc, id, true);
}

static int bl808_spi_deassert(struct reset_controller_dev *rc, unsigned long id)
{
	return bl808_spi_reset_set(rc, id, false);
}

static int bl808_spi_reset(struct reset_controller_dev *rc, unsigned long id)
{
	bl808_spi_assert(rc, id);
	udelay(2);
	return bl808_spi_deassert(rc, id);
}

static int bl808_spi_reset_status(struct reset_controller_dev *rc,
				  unsigned long id)
{
	struct bl808_mm *m = container_of(rc, struct bl808_mm, reset);

	return !!(readl(m->base + MM_RESET) & SPI_RESET);
}

static const struct reset_control_ops bl808_spi_reset_ops = {
	.assert = bl808_spi_assert,
	.deassert = bl808_spi_deassert,
	.reset = bl808_spi_reset,
	.status = bl808_spi_reset_status,
};

static int bl808_mm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct clk_parent_data parent = { .fw_name = "xtal" };
	struct clk_init_data init = {
		.name = "bl808-spi1",
		.ops = &bl808_spi_clock_ops,
		.parent_data = &parent,
		.num_parents = 1,
		.flags = CLK_GET_RATE_NOCACHE,
	};
	struct bl808_mm *m;
	int ret;

	m = devm_kzalloc(dev, sizeof(*m), GFP_KERNEL);
	if (!m)
		return -ENOMEM;
	m->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(m->base))
		return PTR_ERR(m->base);
	spin_lock_init(&m->lock);
	m->hw.init = &init;
	ret = devm_clk_hw_register(dev, &m->hw);
	if (ret)
		return ret;
	m->reset = (struct reset_controller_dev){
		.owner = THIS_MODULE,
		.ops = &bl808_spi_reset_ops,
		.of_node = dev->of_node,
		.nr_resets = 1,
	};
	ret = devm_reset_controller_register(dev, &m->reset);
	if (ret)
		return ret;
	return devm_of_clk_add_hw_provider(dev, of_clk_hw_simple_get, &m->hw);
}

static const struct of_device_id bl808_mm_match[] = {
	{ .compatible = "bflb,bl808-mm-spi-clock" },
	{}
};
MODULE_DEVICE_TABLE(of, bl808_mm_match);
static struct platform_driver bl808_mm_driver = {
	.probe = bl808_mm_probe,
	.driver = { .name = "bl808-mm-spi-clock",
		    .of_match_table = bl808_mm_match },
};
module_platform_driver(bl808_mm_driver);
MODULE_DESCRIPTION("Bouffalo Lab BL808 SPI1 clock and reset");
MODULE_LICENSE("GPL");
