// SPDX-License-Identifier: GPL-2.0

#include <linux/reset.h>

__rust_helper
struct reset_control *
rust_helper_reset_control_get_optional_exclusive(struct device *dev, const char *id)
{
	return reset_control_get_optional_exclusive(dev, id);
}

__rust_helper
int rust_helper_reset_control_deassert(struct reset_control *reset)
{
	return reset_control_deassert(reset);
}

__rust_helper
void rust_helper_reset_control_put(struct reset_control *reset)
{
	reset_control_put(reset);
}
