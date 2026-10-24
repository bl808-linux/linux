// SPDX-License-Identifier: GPL-2.0-only
/* BL808 multimedia DMA2. Initial slave support uses byte-wide single requests. */
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of_dma.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/slab.h>

#include "virt-dma.h"

#define BL808_CHANNELS 8
#define BL808_LLI_MAX 512
#define BL808_TRANSFER_MAX 4095
#define DMA_TC_STATUS 0x04
#define DMA_TC_CLEAR 0x08
#define DMA_ERR_STATUS 0x0c
#define DMA_ERR_CLEAR 0x10
#define DMA_ENABLED 0x1c
#define DMA_CONFIG 0x30
#define CH_SRC 0x00
#define CH_DST 0x04
#define CH_LLI 0x08
#define CH_CONTROL 0x0c
#define CH_CONFIG 0x10
#define CONTROL_SI BIT(26)
#define CONTROL_DI BIT(27)
#define CONTROL_IRQ BIT(31)
#define CONFIG_ENABLE BIT(0)
#define CONFIG_ACTIVE BIT(17)
#define CONFIG_HALT BIT(18)
/* Unlike PL080, IE/ITC are active-high interrupt masks on BL808. */
#define CONFIG_IRQ_MASK (BIT(14) | BIT(15))
#define CONFIG_LLI_COUNT GENMASK(29, 20)

struct bl808_lli {
	__le32 src, dst, next, control;
};

struct bl808_dma;

struct bl808_desc {
	struct virt_dma_desc vd;
	struct bl808_lli *lli;
	dma_addr_t lli_dma;
	dma_addr_t buffer;
	size_t total;
	unsigned int count;
	enum dma_transfer_direction direction;
	bool cyclic;
};

struct bl808_channel {
	struct virt_dma_chan vc;
	struct bl808_dma *parent;
	void __iomem *base;
	struct dma_slave_config config;
	struct bl808_desc *active;
	dma_cookie_t error_cookie;
	unsigned int id, request, lli_count;
};

struct bl808_dma {
	struct dma_device dma;
	void __iomem *base;
	struct dma_pool *pool;
	struct reset_control *reset;
	bool faulted;
	struct bl808_channel channels[BL808_CHANNELS];
};

static struct bl808_channel *to_bl808_channel(struct dma_chan *chan)
{
	return container_of(chan, struct bl808_channel, vc.chan);
}

static struct bl808_desc *to_bl808_desc(struct virt_dma_desc *vd)
{
	return container_of(vd, struct bl808_desc, vd);
}

static void bl808_desc_free(struct virt_dma_desc *vd)
{
	struct bl808_desc *d = to_bl808_desc(vd);
	struct bl808_channel *c = to_bl808_channel(vd->tx.chan);

	dma_pool_free(c->parent->pool, d->lli, d->lli_dma);
	kfree(d);
}

static struct bl808_desc *bl808_alloc_desc(struct bl808_channel *c)
{
	struct bl808_desc *d = kzalloc_obj(*d, GFP_NOWAIT);

	if (!d)
		return NULL;
	d->lli = dma_pool_alloc(c->parent->pool, GFP_NOWAIT, &d->lli_dma);
	if (!d->lli) {
		kfree(d);
		return NULL;
	}
	return d;
}

static bool bl808_valid_range(dma_addr_t address, size_t length)
{
	return length && address <= U32_MAX && length - 1 <= U32_MAX - address;
}

static bool bl808_valid_slave(struct bl808_channel *c, enum dma_transfer_direction dir)
{
	if (dir == DMA_DEV_TO_MEM)
		return c->config.src_addr && bl808_valid_range(c->config.src_addr, 1);
	if (dir == DMA_MEM_TO_DEV)
		return c->config.dst_addr && bl808_valid_range(c->config.dst_addr, 1);
	return false;
}

static void bl808_fill_lli(struct bl808_channel *c, struct bl808_desc *d,
			   unsigned int index, dma_addr_t memory, size_t length,
			   bool interrupt)
{
	u32 control = length | (interrupt ? CONTROL_IRQ : 0);
	u32 source, destination;

	if (d->direction == DMA_DEV_TO_MEM) {
		source = c->config.src_addr;
		destination = memory;
		control |= CONTROL_DI;
	} else {
		source = memory;
		destination = c->config.dst_addr;
		control |= CONTROL_SI;
	}
	d->lli[index].src = cpu_to_le32(source);
	d->lli[index].dst = cpu_to_le32(destination);
	d->lli[index].control = cpu_to_le32(control);
	d->lli[index].next = cpu_to_le32(d->lli_dma + (index + 1) * sizeof(*d->lli));
}

static struct dma_async_tx_descriptor *
bl808_prep_cyclic(struct dma_chan *chan, dma_addr_t buffer, size_t length,
		  size_t period, enum dma_transfer_direction direction, unsigned long flags)
{
	struct bl808_channel *c = to_bl808_channel(chan);
	struct bl808_desc *d;
	unsigned int count, i;

	if (READ_ONCE(c->parent->faulted) || !period || period > BL808_TRANSFER_MAX ||
	    length % period ||
	    !bl808_valid_range(buffer, length) || !bl808_valid_slave(c, direction))
		return NULL;
	count = length / period;
	if (!count || count > BL808_LLI_MAX)
		return NULL;
	d = bl808_alloc_desc(c);
	if (!d)
		return NULL;
	d->direction = direction;
	d->buffer = buffer;
	d->total = length;
	d->count = count;
	d->cyclic = true;
	for (i = 0; i < count; i++)
		bl808_fill_lli(c, d, i, buffer + i * period, period, true);
	d->lli[count - 1].next = cpu_to_le32(d->lli_dma);
	return vchan_tx_prep(&c->vc, &d->vd, flags);
}

static struct dma_async_tx_descriptor *
bl808_prep_slave_sg(struct dma_chan *chan, struct scatterlist *sgl, unsigned int sg_len,
		    enum dma_transfer_direction direction, unsigned long flags, void *context)
{
	struct bl808_channel *c = to_bl808_channel(chan);
	struct scatterlist *sg;
	struct bl808_desc *d;
	unsigned int i, count = 0, index = 0;

	if (READ_ONCE(c->parent->faulted) || !sg_len || !bl808_valid_slave(c, direction))
		return NULL;
	for_each_sg(sgl, sg, sg_len, i) {
		if (!bl808_valid_range(sg_dma_address(sg), sg_dma_len(sg)))
			return NULL;
		count += DIV_ROUND_UP(sg_dma_len(sg), BL808_TRANSFER_MAX);
		if (count > BL808_LLI_MAX)
			return NULL;
	}
	d = bl808_alloc_desc(c);
	if (!d)
		return NULL;
	d->direction = direction;
	d->count = count;
	for_each_sg(sgl, sg, sg_len, i) {
		dma_addr_t address = sg_dma_address(sg);
		size_t remaining = sg_dma_len(sg);

		while (remaining) {
			size_t bytes = min_t(size_t, remaining, BL808_TRANSFER_MAX);

			bl808_fill_lli(c, d, index, address, bytes, index == count - 1);
			d->total += bytes;
			index++;
			address += bytes;
			remaining -= bytes;
		}
	}
	d->lli[count - 1].next = 0;
	return vchan_tx_prep(&c->vc, &d->vd, flags);
}

static void bl808_start(struct bl808_channel *c)
{
	struct virt_dma_desc *vd = vchan_next_desc(&c->vc);
	struct bl808_desc *d;
	u32 config = CONFIG_ENABLE; /* IE/ITC = 0 enables error/completion IRQs. */

	if (!vd || READ_ONCE(c->parent->faulted))
		return;
	list_del(&vd->node);
	d = to_bl808_desc(vd);
	c->active = d;
	if (d->direction == DMA_DEV_TO_MEM)
		config |= (c->request << 1) | (2 << 11);
	else
		config |= (c->request << 6) | (1 << 11);
	writel(BIT(c->id), c->parent->base + DMA_TC_CLEAR);
	writel(BIT(c->id), c->parent->base + DMA_ERR_CLEAR);
	writel(le32_to_cpu(d->lli[0].src), c->base + CH_SRC);
	writel(le32_to_cpu(d->lli[0].dst), c->base + CH_DST);
	writel(le32_to_cpu(d->lli[0].next), c->base + CH_LLI);
	writel(le32_to_cpu(d->lli[0].control), c->base + CH_CONTROL);
	/* Publish every descriptor before enabling its fetch engine. */
	dma_wmb();
	writel(config, c->base + CH_CONFIG);
	c->lli_count = FIELD_GET(CONFIG_LLI_COUNT, readl(c->base + CH_CONFIG));
}

static void bl808_issue_pending(struct dma_chan *chan)
{
	struct bl808_channel *c = to_bl808_channel(chan);
	unsigned long flags;

	spin_lock_irqsave(&c->vc.lock, flags);
	if (vchan_issue_pending(&c->vc) && !c->active)
		bl808_start(c);
	spin_unlock_irqrestore(&c->vc.lock, flags);
}

static int bl808_slave_config(struct dma_chan *chan, struct dma_slave_config *config)
{
	struct bl808_channel *c = to_bl808_channel(chan);

	/* Single-byte requests also allow an RX timeout with a partial period. */
	if ((config->src_addr_width && config->src_addr_width != DMA_SLAVE_BUSWIDTH_1_BYTE) ||
	    (config->dst_addr_width && config->dst_addr_width != DMA_SLAVE_BUSWIDTH_1_BYTE) ||
	    config->src_maxburst > 1 || config->dst_maxburst > 1)
		return -EINVAL;
	c->config = *config;
	return 0;
}

static int bl808_halt(struct bl808_channel *c)
{
	u32 value;
	int ret;

	writel(readl(c->base + CH_CONFIG) | CONFIG_HALT, c->base + CH_CONFIG);
	ret = readl_poll_timeout_atomic(c->base + CH_CONFIG, value,
					!(value & CONFIG_ACTIVE), 1, 1000);
	if (ret) {
		/* The dedicated MMIO multimedia reset stops every bus master beat.
		 * Keep the controller faulted and in reset until a full rebind;
		 * no client memory may be freed while the engine can still write.
		 */
		WRITE_ONCE(c->parent->faulted, true);
		return reset_control_assert(c->parent->reset);
	}
	writel(0, c->base + CH_CONFIG);
	readl(c->base + CH_CONFIG);
	writel(BIT(c->id), c->parent->base + DMA_TC_CLEAR);
	writel(BIT(c->id), c->parent->base + DMA_ERR_CLEAR);
	return 0;
}

static int bl808_terminate_all(struct dma_chan *chan)
{
	struct bl808_channel *c = to_bl808_channel(chan);
	unsigned long flags;
	LIST_HEAD(head);
	int ret;

	spin_lock_irqsave(&c->vc.lock, flags);
	ret = bl808_halt(c);
	if (!ret) {
		if (c->active) {
			vchan_terminate_vdesc(&c->active->vd);
			c->active = NULL;
		}
		list_splice_tail_init(&c->vc.desc_allocated, &head);
		list_splice_tail_init(&c->vc.desc_submitted, &head);
		list_splice_tail_init(&c->vc.desc_issued, &head);
		list_splice_tail_init(&c->vc.desc_completed, &head);
		/* desc_terminated is retained until vchan_synchronize kills callbacks. */
	}
	spin_unlock_irqrestore(&c->vc.lock, flags);
	vchan_dma_desc_free_list(&c->vc, &head);
	return ret;
}

static void bl808_synchronize(struct dma_chan *chan)
{
	vchan_synchronize(&to_bl808_channel(chan)->vc);
}

static void bl808_free_resources(struct dma_chan *chan)
{
	bl808_terminate_all(chan);
	bl808_synchronize(chan);
	vchan_free_chan_resources(&to_bl808_channel(chan)->vc);
}

static enum dma_status bl808_tx_status(struct dma_chan *chan, dma_cookie_t cookie,
				       struct dma_tx_state *state)
{
	struct bl808_channel *c = to_bl808_channel(chan);
	struct bl808_desc *d;
	struct virt_dma_desc *vd;
	enum dma_status status;
	unsigned long flags;
	u32 address;
	size_t residue = 0;
	unsigned int i;

	spin_lock_irqsave(&c->vc.lock, flags);
	status = READ_ONCE(c->parent->faulted) || cookie == c->error_cookie ?
		 DMA_ERROR : dma_cookie_status(chan, cookie, state);
	if (status == DMA_COMPLETE || status == DMA_ERROR || !state)
		goto out;
	d = c->active;
	if (d && d->vd.tx.cookie == cookie) {
		address = readl(c->base + (d->direction == DMA_DEV_TO_MEM ? CH_DST : CH_SRC));
		if (d->cyclic) {
			if (address >= d->buffer && address <= d->buffer + d->total)
				residue = d->total - (address - d->buffer) % d->total;
			else
				status = DMA_ERROR;
		} else {
			for (i = d->count; i-- > 0;) {
				u32 base = le32_to_cpu(d->direction == DMA_DEV_TO_MEM ?
						      d->lli[i].dst : d->lli[i].src);
				u32 bytes = le32_to_cpu(d->lli[i].control) & BL808_TRANSFER_MAX;

				if (address >= base && address <= base + bytes) {
					residue += base + bytes - address;
					break;
				}
				residue += bytes;
			}
		}
	} else {
		vd = vchan_find_desc(&c->vc, cookie);
		if (vd)
			residue = to_bl808_desc(vd)->total;
	}
	dma_set_residue(state, residue);
out:
	spin_unlock_irqrestore(&c->vc.lock, flags);
	return status;
}

static irqreturn_t bl808_interrupt(int irq, void *data)
{
	struct bl808_channel *c = data;
	struct bl808_desc *d;
	unsigned long flags;
	u32 bit = BIT(c->id), done, error, count, delta;

	done = readl(c->parent->base + DMA_TC_STATUS) & bit;
	error = readl(c->parent->base + DMA_ERR_STATUS) & bit;
	if (!done && !error)
		return IRQ_NONE;
	writel(bit, c->parent->base + DMA_TC_CLEAR);
	writel(bit, c->parent->base + DMA_ERR_CLEAR);
	spin_lock_irqsave(&c->vc.lock, flags);
	d = c->active;
	if (!d)
		goto out;
	count = FIELD_GET(CONFIG_LLI_COUNT, readl(c->base + CH_CONFIG));
	delta = (count - c->lli_count) & 1023;
	c->lli_count = count;
	/* A whole missed ring cannot be reconstructed from a modulo residue. */
	if (d->cyclic && d->direction == DMA_DEV_TO_MEM && delta >= d->count)
		error = bit;
	if (error) {
		c->error_cookie = d->vd.tx.cookie;
		if (bl808_halt(c))
			goto out;
		d->vd.tx_result.result = DMA_TRANS_ABORTED;
		c->active = NULL;
		if (c->vc.cyclic == &d->vd)
			c->vc.cyclic = NULL;
		vchan_cookie_complete(&d->vd);
		bl808_start(c);
	} else if (d->cyclic) {
		vchan_cyclic_callback(&d->vd);
	} else {
		c->active = NULL;
		vchan_cookie_complete(&d->vd);
		bl808_start(c);
	}
out:
	spin_unlock_irqrestore(&c->vc.lock, flags);
	return IRQ_HANDLED;
}

static struct dma_chan *bl808_of_xlate(struct of_phandle_args *spec, struct of_dma *ofdma)
{
	struct bl808_dma *b = ofdma->of_dma_data;
	struct dma_chan *chan;

	if (spec->args_count != 1 ||
	    !(spec->args[0] <= 3 || (spec->args[0] >= 6 && spec->args[0] <= 11) ||
	      spec->args[0] == 22))
		return NULL;
	chan = dma_get_any_slave_channel(&b->dma);
	if (chan)
		to_bl808_channel(chan)->request = spec->args[0];
	return chan;
}

static int bl808_dma_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct reset_control *reset;
	struct bl808_dma *b;
	struct clk *clock;
	int ret, i, irq;

	b = devm_kzalloc(dev, sizeof(*b), GFP_KERNEL);
	if (!b)
		return -ENOMEM;
	b->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(b->base))
		return PTR_ERR(b->base);
	clock = devm_clk_get_enabled(dev, "bus");
	if (IS_ERR(clock))
		return PTR_ERR(clock);
	reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(reset))
		return PTR_ERR(reset);
	b->reset = reset;
	ret = reset_control_deassert(reset);
	if (ret)
		return ret;
	/* Do not take over any channel still running on behalf of firmware. */
	if (readl(b->base + DMA_ENABLED))
		return dev_err_probe(dev, -EBUSY, "firmware DMA channel still active\n");
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	b->pool = dmam_pool_create("bl808-lli", dev,
				   BL808_LLI_MAX * sizeof(struct bl808_lli), 16, 0);
	if (!b->pool)
		return -ENOMEM;
	INIT_LIST_HEAD(&b->dma.channels);
	dma_cap_set(DMA_SLAVE, b->dma.cap_mask);
	dma_cap_set(DMA_CYCLIC, b->dma.cap_mask);
	b->dma.dev = dev;
	b->dma.device_prep_slave_sg = bl808_prep_slave_sg;
	b->dma.device_prep_dma_cyclic = bl808_prep_cyclic;
	b->dma.device_config = bl808_slave_config;
	b->dma.device_issue_pending = bl808_issue_pending;
	b->dma.device_tx_status = bl808_tx_status;
	b->dma.device_terminate_all = bl808_terminate_all;
	b->dma.device_synchronize = bl808_synchronize;
	b->dma.device_free_chan_resources = bl808_free_resources;
	b->dma.src_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_1_BYTE);
	b->dma.dst_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_1_BYTE);
	b->dma.directions = BIT(DMA_MEM_TO_DEV) | BIT(DMA_DEV_TO_MEM);
	b->dma.residue_granularity = DMA_RESIDUE_GRANULARITY_BURST;
	writel(0xff, b->base + DMA_TC_CLEAR);
	writel(0xff, b->base + DMA_ERR_CLEAR);
	for (i = 0; i < BL808_CHANNELS; i++) {
		struct bl808_channel *c = &b->channels[i];

		c->parent = b;
		c->id = i;
		c->base = b->base + 0x100 + i * 0x100;
		c->vc.desc_free = bl808_desc_free;
		vchan_init(&c->vc, &b->dma);
		irq = platform_get_irq(pdev, i);
		if (irq < 0)
			return irq;
		ret = devm_request_irq(dev, irq, bl808_interrupt, 0, "bl808-dma2", c);
		if (ret)
			return ret;
	}
	writel(readl(b->base + DMA_CONFIG) | BIT(0), b->base + DMA_CONFIG);
	ret = dma_async_device_register(&b->dma);
	if (ret)
		return ret;
	ret = of_dma_controller_register(dev->of_node, bl808_of_xlate, b);
	if (ret) {
		dma_async_device_unregister(&b->dma);
		return ret;
	}
	platform_set_drvdata(pdev, b);
	return 0;
}

static void bl808_dma_remove(struct platform_device *pdev)
{
	struct bl808_dma *b = platform_get_drvdata(pdev);
	unsigned int i;

	of_dma_controller_free(pdev->dev.of_node);
	dma_async_device_unregister(&b->dma);
	for (i = 0; i < BL808_CHANNELS; i++)
		bl808_free_resources(&b->channels[i].vc.chan);
	writel(readl(b->base + DMA_CONFIG) & ~BIT(0), b->base + DMA_CONFIG);
}

static const struct of_device_id bl808_dma_match[] = {
	{ .compatible = "bflb,bl808-dma2" },
	{ }
};
MODULE_DEVICE_TABLE(of, bl808_dma_match);

static struct platform_driver bl808_dma_driver = {
	.probe = bl808_dma_probe,
	.remove = bl808_dma_remove,
	.driver = {
		.name = "bl808-dma2",
		.of_match_table = bl808_dma_match,
	},
};
module_platform_driver(bl808_dma_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Bouffalo Lab BL808 multimedia DMA2");
