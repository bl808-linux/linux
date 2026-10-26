// SPDX-License-Identifier: GPL-2.0
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>

__rust_helper struct regmap *rust_helper_syscon_lookup(struct device *dev,
						       const char *property)
{
	return syscon_regmap_lookup_by_phandle(dev->of_node, property);
}

__rust_helper int rust_helper_regmap_update_bits(struct regmap *map,
						 unsigned int reg,
						 unsigned int mask,
						 unsigned int value)
{
	return regmap_update_bits(map, reg, mask, value);
}
