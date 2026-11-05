// SPDX-License-Identifier: GPL-2.0
/* Validation-only extraction of the UART3 reset operations from 62fe251e78c0. */
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/reset-controller.h>
#include <linux/spinlock.h>

#define RESET_OFFSET 0x44
#define UART_RESET BIT(2)

struct reset_validation {
	void __iomem *base;
	spinlock_t lock;
	struct reset_controller_dev rc;
	char result[384];
};

static int reset_update(struct reset_controller_dev *rc, unsigned long id, bool assert)
{
	struct reset_validation *v = container_of(rc, struct reset_validation, rc);
	unsigned long flags;
	u32 word;

	if (id)
		return -EINVAL;
	spin_lock_irqsave(&v->lock, flags);
	word = readl(v->base + RESET_OFFSET) & ~UART_RESET;
	writel(word | (assert ? UART_RESET : 0), v->base + RESET_OFFSET);
	readl(v->base + RESET_OFFSET);
	spin_unlock_irqrestore(&v->lock, flags);
	return 0;
}

static int reset_assert(struct reset_controller_dev *rc, unsigned long id)
{
	return reset_update(rc, id, true);
}

static int reset_deassert(struct reset_controller_dev *rc, unsigned long id)
{
	return reset_update(rc, id, false);
}

static int reset_status(struct reset_controller_dev *rc, unsigned long id)
{
	struct reset_validation *v = container_of(rc, struct reset_validation, rc);

	return id ? -EINVAL : !!(readl(v->base + RESET_OFFSET) & UART_RESET);
}

static const struct reset_control_ops reset_ops = {
	.assert = reset_assert,
	.deassert = reset_deassert,
	.status = reset_status,
};

static ssize_t test_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct reset_validation *v = dev_get_drvdata(dev);

	ssize_t length;

	device_lock(dev);
	length = sysfs_emit(buf, "%s\n", v->result);
	device_unlock(dev);
	return length;
}

static ssize_t test_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct reset_validation *v = dev_get_drvdata(dev);
	struct device_node *node;
	struct platform_device *uart;
	struct reset_control *rst;
	struct resource resource;
	void __iomem *io;
	u32 before = 0, after = 0, cpu = 0, peri = 0, tx, rx, period, sw, fifo;
	u32 observed = 0, status, cpu_after = 0, peri_after = 0;
	u32 busy_before = 0, busy_held = 1, busy_after = 1;
	unsigned long flags;
	int asserted = -1, released = -1;
	int ret;

	if (!sysfs_streq(buf, "1"))
		return -EINVAL;
	/* Serialize test results and reject tests while the UART is bound. */
	device_lock(dev);
	node = of_parse_phandle(dev->of_node, "validation-uart", 0);
	if (!node) {
		ret = -ENODEV;
		goto out;
	}
	uart = of_find_device_by_node(node);
	ret = of_address_to_resource(node, 0, &resource);
	of_node_put(node);
	if (!uart) {
		ret = -ENODEV;
		goto out;
	}
	if (ret)
		goto put_device;
	device_lock(&uart->dev);
	if (uart->dev.driver) {
		ret = -EBUSY;
		goto unlock_uart;
	}
	rst = reset_control_get_exclusive(&uart->dev, NULL);
	if (IS_ERR(rst)) {
		ret = PTR_ERR(rst);
		goto unlock_uart;
	}
	io = ioremap(resource.start, 0x90);
	if (!io) {
		ret = -ENOMEM;
		goto put_reset;
	}
	before = readl(v->base + RESET_OFFSET);
	cpu = readl(v->base);
	peri = readl(v->base + 0x10);
	if (before & UART_RESET) {
		ret = -EBUSY;
		goto unmap;
	}
	ret = readl_poll_timeout(io + 0x30, status, !(status & BIT(0)), 10, 100000);
	if (ret)
		goto unmap;
	tx = readl(io);
	rx = readl(io + 4);
	period = readl(io + 8);
	sw = readl(io + 0x1c);
	fifo = readl(io + 0x80) & 3;
	writel(tx & ~BIT(0), io);
	writel(rx & ~BIT(0), io + 4);
	/* Keep the external TX pad idle while a slow internal byte is active. */
	writel(sw | 3, io + 0x1c);
	writel(12, io + 0x80);
	writel(0xffffffff, io + 8);
	if (readl(io + 8) != 0xffffffff) {
		ret = -EIO;
		goto restore;
	}
	/* At the firmware's 40 MHz UART clock the byte takes about 16 ms. */
	local_irq_save(flags);
	writel(0x55, io + 0x88);
	writel(0xf05, io);
	ret = readl_poll_timeout_atomic(io + 0x30, status, status & BIT(0), 1, 1000);
	udelay(100);
	busy_before = readl(io + 0x30) & BIT(0);
	if (!busy_before)
		ret = -EIO;
	if (reset_control_assert(rst) && !ret)
		ret = -EIO;
	asserted = reset_control_status(rst);
	if (!ret && asserted != 1)
		ret = -EIO;
	if (readl_poll_timeout_atomic(io + 0x30, status, !(status & BIT(0)), 1, 1000))
		ret = -EIO;
	busy_held = readl(io + 0x30) & BIT(0);
	observed = readl(io + 8);
	/* Registers and FIFO survive reset; prevent the canary restarting. */
	writel(tx & ~BIT(0), io);
	writel(12, io + 0x80);
	if (reset_control_deassert(rst) && !ret)
		ret = -EIO;
	released = reset_control_status(rst);
	busy_after = readl(io + 0x30) & BIT(0);
	local_irq_restore(flags);
	if (released != 0) {
		ret = -EIO;
		goto unmap;
	}
	after = readl(v->base + RESET_OFFSET);
	cpu_after = readl(v->base);
	peri_after = readl(v->base + 0x10);
	if (busy_before != 1 || busy_held || busy_after || before != after ||
	    cpu != cpu_after || peri != peri_after)
		ret = -EIO;
restore:
	writel(period, io + 8);
	writel(fifo, io + 0x80);
	writel(sw, io + 0x1c);
	writel(rx, io + 4);
	writel(tx, io);
unmap:
	iounmap(io);
put_reset:
	reset_control_put(rst);
unlock_uart:
	device_unlock(&uart->dev);
put_device:
	put_device(&uart->dev);
out:
	scnprintf(v->result, sizeof(v->result),
		  "ret=%d reset_period=%08x reset_after=%08x asserted=%d released=%d busy_before=%u busy_held=%u busy_after=%u reset_before=%08x cpu=%08x/%08x peri=%08x/%08x",
		  ret, observed, after, asserted, released, busy_before, busy_held,
		  busy_after, before, cpu, cpu_after,
		  peri, peri_after);
	device_unlock(dev);
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(test);

static struct attribute *test_attrs[] = { &dev_attr_test.attr, NULL };
static const struct attribute_group test_group = { .attrs = test_attrs };

static int validation_probe(struct platform_device *pdev)
{
	struct reset_validation *v;
	int ret;

	v = devm_kzalloc(&pdev->dev, sizeof(*v), GFP_KERNEL);
	if (!v)
		return -ENOMEM;
	v->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(v->base))
		return PTR_ERR(v->base);
	spin_lock_init(&v->lock);
	v->rc.owner = THIS_MODULE;
	v->rc.ops = &reset_ops;
	v->rc.of_node = pdev->dev.of_node;
	v->rc.nr_resets = 1;
	v->rc.dev = &pdev->dev;
	ret = devm_reset_controller_register(&pdev->dev, &v->rc);
	if (ret)
		return ret;
	strscpy(v->result, "not-run");
	platform_set_drvdata(pdev, v);
	return devm_device_add_group(&pdev->dev, &test_group);
}

static const struct of_device_id validation_match[] = {
	{ .compatible = "bouffalolab,bl808-reset-validation" },
	{ }
};

static struct platform_driver validation_driver = {
	.probe = validation_probe,
	.driver = {
		.name = "bl808_reset_validation",
		.of_match_table = validation_match,
	},
};
module_platform_driver(validation_driver);
MODULE_LICENSE("GPL");
