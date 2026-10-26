// SPDX-License-Identifier: GPL-2.0-only
/* BL808 per-pad pinmux and GPIO. Probe never changes inherited pad state. */
#include <linux/bitfield.h>
#include <linux/gpio/driver.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include "../pinctrl-utils.h"

#define PAD_IE BIT(0)
#define PAD_SMT BIT(1)
#define PAD_DRV GENMASK(3, 2)
#define PAD_PU BIT(4)
#define PAD_PD BIT(5)
#define PAD_OE BIT(6)
#define PAD_FUNC GENMASK(12, 8)
#define PAD_OUT BIT(24)
#define PAD_INPUT BIT(28)
#define PAD_STROBES (BIT(20) | BIT(25) | BIT(26))
#define PAD_MODE GENMASK(31, 30)

struct bl808_pins {
	void __iomem *base;
	raw_spinlock_t lock;
	struct gpio_chip gpio;
	struct pinctrl_desc desc;
};

static const struct pinctrl_pin_desc bl808_pads[] = {
	PINCTRL_PIN(0, "GPIO0"),   PINCTRL_PIN(1, "GPIO1"),
	PINCTRL_PIN(2, "GPIO2"),   PINCTRL_PIN(3, "GPIO3"),
	PINCTRL_PIN(4, "GPIO4"),   PINCTRL_PIN(5, "GPIO5"),
	PINCTRL_PIN(6, "GPIO6"),   PINCTRL_PIN(7, "GPIO7"),
	PINCTRL_PIN(8, "GPIO8"),   PINCTRL_PIN(9, "GPIO9"),
	PINCTRL_PIN(10, "GPIO10"), PINCTRL_PIN(11, "GPIO11"),
	PINCTRL_PIN(12, "GPIO12"), PINCTRL_PIN(13, "GPIO13"),
	PINCTRL_PIN(14, "GPIO14"), PINCTRL_PIN(15, "GPIO15"),
	PINCTRL_PIN(16, "GPIO16"), PINCTRL_PIN(17, "GPIO17"),
	PINCTRL_PIN(18, "GPIO18"), PINCTRL_PIN(19, "GPIO19"),
	PINCTRL_PIN(20, "GPIO20"), PINCTRL_PIN(21, "GPIO21"),
	PINCTRL_PIN(22, "GPIO22"), PINCTRL_PIN(23, "GPIO23"),
	PINCTRL_PIN(24, "GPIO24"), PINCTRL_PIN(25, "GPIO25"),
	PINCTRL_PIN(26, "GPIO26"), PINCTRL_PIN(27, "GPIO27"),
	PINCTRL_PIN(28, "GPIO28"), PINCTRL_PIN(29, "GPIO29"),
	PINCTRL_PIN(30, "GPIO30"), PINCTRL_PIN(31, "GPIO31"),
	PINCTRL_PIN(32, "GPIO32"), PINCTRL_PIN(33, "GPIO33"),
	PINCTRL_PIN(34, "GPIO34"), PINCTRL_PIN(35, "GPIO35"),
	PINCTRL_PIN(36, "GPIO36"), PINCTRL_PIN(37, "GPIO37"),
	PINCTRL_PIN(38, "GPIO38"), PINCTRL_PIN(39, "GPIO39"),
	PINCTRL_PIN(40, "GPIO40"), PINCTRL_PIN(41, "GPIO41"),
	PINCTRL_PIN(42, "GPIO42"), PINCTRL_PIN(43, "GPIO43"),
	PINCTRL_PIN(44, "GPIO44"), PINCTRL_PIN(45, "GPIO45")
};

static const char *const all_groups[] = {
	"GPIO0",  "GPIO1",  "GPIO2",  "GPIO3",	"GPIO4",  "GPIO5",  "GPIO6",
	"GPIO7",  "GPIO8",  "GPIO9",  "GPIO10", "GPIO11", "GPIO12", "GPIO13",
	"GPIO14", "GPIO15", "GPIO16", "GPIO17", "GPIO18", "GPIO19", "GPIO20",
	"GPIO21", "GPIO22", "GPIO23", "GPIO24", "GPIO25", "GPIO26", "GPIO27",
	"GPIO28", "GPIO29", "GPIO30", "GPIO31", "GPIO32", "GPIO33", "GPIO34",
	"GPIO35", "GPIO36", "GPIO37", "GPIO38", "GPIO39", "GPIO40", "GPIO41",
	"GPIO42", "GPIO43", "GPIO44", "GPIO45"
};

static const char *const spi_groups[] = { "GPIO19", "GPIO25" };

static void bl808_pad_update(struct bl808_pins *p, unsigned int pin, u32 mask,
			     u32 value)
{
	unsigned long flags;
	u32 old;

	raw_spin_lock_irqsave(&p->lock, flags);
	old = readl(p->base + pin * 4) & ~PAD_STROBES;
	writel((old & ~mask) | value, p->base + pin * 4);
	raw_spin_unlock_irqrestore(&p->lock, flags);
}

static int bl808_group_count(struct pinctrl_dev *pc)
{
	return ARRAY_SIZE(bl808_pads);
}

static const char *bl808_group_name(struct pinctrl_dev *pc, unsigned int group)
{
	return bl808_pads[group].name;
}

static int bl808_group_pins(struct pinctrl_dev *pc, unsigned int group,
			    const unsigned int **pins, unsigned int *num)
{
	*pins = &bl808_pads[group].number;
	*num = 1;
	return 0;
}

static const struct pinctrl_ops bl808_pctl_ops = {
	.get_groups_count = bl808_group_count,
	.get_group_name = bl808_group_name,
	.get_group_pins = bl808_group_pins,
	.dt_node_to_map = pinconf_generic_dt_node_to_map_pin,
	.dt_free_map = pinctrl_utils_free_map,
};

static int bl808_function_count(struct pinctrl_dev *pc)
{
	return 2;
}

static const char *bl808_function_name(struct pinctrl_dev *pc, unsigned int fn)
{
	return fn ? "spi1" : "gpio";
}

static int bl808_function_groups(struct pinctrl_dev *pc, unsigned int fn,
				 const char *const **groups, unsigned int *num)
{
	*groups = fn ? spi_groups : all_groups;
	*num = fn ? ARRAY_SIZE(spi_groups) : ARRAY_SIZE(all_groups);
	return 0;
}

static int bl808_mux(struct pinctrl_dev *pc, unsigned int fn, unsigned int pin)
{
	struct bl808_pins *p = pinctrl_dev_get_drvdata(pc);

	if (fn && pin != 19 && pin != 25)
		return -EINVAL;
	bl808_pad_update(p, pin, PAD_FUNC | PAD_MODE | PAD_OE,
			 FIELD_PREP(PAD_FUNC, fn ? 18 : 11) | PAD_IE);
	return 0;
}

static int bl808_gpio_request(struct pinctrl_dev *pc,
			      struct pinctrl_gpio_range *range,
			      unsigned int pin)
{
	return bl808_mux(pc, 0, pin);
}

static const struct pinmux_ops bl808_mux_ops = {
	.get_functions_count = bl808_function_count,
	.get_function_name = bl808_function_name,
	.get_function_groups = bl808_function_groups,
	.set_mux = bl808_mux,
	.gpio_request_enable = bl808_gpio_request,
	.strict = true,
};

static const struct pinconf_generic_params bl808_custom_params[] = {
	{ "bflb,drive-strength", PIN_CONFIG_END + 1, 0 },
};

static int bl808_conf_set(struct pinctrl_dev *pc, unsigned int pin,
			  unsigned long *configs, unsigned int count)
{
	struct bl808_pins *p = pinctrl_dev_get_drvdata(pc);
	u32 mask = 0, value = 0;
	unsigned int i, arg;

	for (i = 0; i < count; i++) {
		arg = pinconf_to_config_argument(configs[i]);
		switch ((unsigned int)pinconf_to_config_param(configs[i])) {
		case PIN_CONFIG_BIAS_DISABLE:
			mask |= PAD_PU | PAD_PD;
			value &= ~(PAD_PU | PAD_PD);
			break;
		case PIN_CONFIG_BIAS_PULL_UP:
		case PIN_CONFIG_BIAS_PULL_DOWN:
			mask |= PAD_PU | PAD_PD;
			value &= ~(PAD_PU | PAD_PD);
			value |= pinconf_to_config_param(configs[i]) ==
						 PIN_CONFIG_BIAS_PULL_UP ?
					 PAD_PU :
					 PAD_PD;
			break;
		case PIN_CONFIG_INPUT_SCHMITT_ENABLE:
			mask |= PAD_SMT;
			value = (value & ~PAD_SMT) | (arg ? PAD_SMT : 0);
			break;
		case PIN_CONFIG_END + 1:
			/* Hardware drive encoding; no unverified conversion to mA. */
			if (arg > 3)
				return -EINVAL;
			mask |= PAD_DRV;
			value = (value & ~PAD_DRV) | FIELD_PREP(PAD_DRV, arg);
			break;
		case PIN_CONFIG_INPUT_ENABLE:
			mask |= PAD_IE;
			value = (value & ~PAD_IE) | (arg ? PAD_IE : 0);
			break;
		default:
			return -ENOTSUPP;
		}
	}
	bl808_pad_update(p, pin, mask, value);
	return 0;
}

static const struct pinconf_ops bl808_conf_ops = {
	.is_generic = true,
	.pin_config_set = bl808_conf_set,
};

static int bl808_get(struct gpio_chip *gc, unsigned int pin)
{
	struct bl808_pins *p = gpiochip_get_data(gc);

	return !!(readl(p->base + pin * 4) & PAD_INPUT);
}

static int bl808_set(struct gpio_chip *gc, unsigned int pin, int value)
{
	bl808_pad_update(gpiochip_get_data(gc), pin, PAD_OUT,
			 value ? PAD_OUT : 0);
	return 0;
}

static int bl808_input(struct gpio_chip *gc, unsigned int pin)
{
	bl808_pad_update(gpiochip_get_data(gc), pin, PAD_OE | PAD_IE, PAD_IE);
	return 0;
}

static int bl808_output(struct gpio_chip *gc, unsigned int pin, int value)
{
	struct bl808_pins *p = gpiochip_get_data(gc);

	bl808_set(gc, pin, value);
	bl808_pad_update(p, pin, PAD_OE | PAD_IE, PAD_OE | PAD_IE);
	return 0;
}

static int bl808_direction(struct gpio_chip *gc, unsigned int pin)
{
	struct bl808_pins *p = gpiochip_get_data(gc);

	return (readl(p->base + pin * 4) & PAD_OE) ? GPIO_LINE_DIRECTION_OUT :
						     GPIO_LINE_DIRECTION_IN;
}

static int bl808_pin_ranges(struct gpio_chip *gc)
{
	return gpiochip_add_pin_range(gc, dev_name(gc->parent), 0, 0,
				      gc->ngpio);
}

static int bl808_pins_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pinctrl_dev *pc;
	struct bl808_pins *p;
	int ret;

	p = devm_kzalloc(dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;
	p->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(p->base))
		return PTR_ERR(p->base);
	raw_spin_lock_init(&p->lock);
	p->desc = (struct pinctrl_desc){
		.name = dev_name(dev),
		.owner = THIS_MODULE,
		.pins = bl808_pads,
		.npins = ARRAY_SIZE(bl808_pads),
		.pctlops = &bl808_pctl_ops,
		.pmxops = &bl808_mux_ops,
		.confops = &bl808_conf_ops,
		.custom_params = bl808_custom_params,
		.num_custom_params = ARRAY_SIZE(bl808_custom_params),
	};
	ret = devm_pinctrl_register_and_init(dev, &p->desc, p, &pc);
	if (ret)
		return ret;
	ret = pinctrl_enable(pc);
	if (ret)
		return ret;
	p->gpio = (struct gpio_chip){
		.label = dev_name(dev),
		.parent = dev,
		.owner = THIS_MODULE,
		.base = -1,
		.ngpio = ARRAY_SIZE(bl808_pads),
		.request = gpiochip_generic_request,
		.free = gpiochip_generic_free,
		.get = bl808_get,
		.set = bl808_set,
		.set_config = gpiochip_generic_config,
		.get_direction = bl808_direction,
		.direction_input = bl808_input,
		.direction_output = bl808_output,
		.add_pin_ranges = bl808_pin_ranges,
	};
	return devm_gpiochip_add_data(dev, &p->gpio, p);
}

static const struct of_device_id bl808_pins_match[] = {
	{ .compatible = "bflb,bl808-pinctrl" },
	{}
};
MODULE_DEVICE_TABLE(of, bl808_pins_match);
static struct platform_driver bl808_pins_driver = {
	.probe = bl808_pins_probe,
	.driver = { .name = "bl808-pinctrl",
		    .of_match_table = bl808_pins_match },
};
module_platform_driver(bl808_pins_driver);
MODULE_DESCRIPTION("Bouffalo Lab BL808 pinctrl and GPIO");
MODULE_LICENSE("GPL");
