// SPDX-License-Identifier: GPL-2.0
#include <linux/reset.h>

__rust_helper struct reset_control *
rust_helper_reset_control_get_exclusive(struct device *dev)
{
	return reset_control_get_exclusive(dev, NULL);
}

