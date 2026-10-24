// SPDX-License-Identifier: GPL-2.0
#include <linux/dmaengine.h>

__rust_helper int rust_helper_dmaengine_slave_config(struct dma_chan *chan,
						    struct dma_slave_config *config)
{
	return dmaengine_slave_config(chan, config);
}

__rust_helper struct dma_async_tx_descriptor *
rust_helper_dmaengine_prep_dma_cyclic(struct dma_chan *chan, dma_addr_t buffer,
				     size_t length, size_t period,
				     enum dma_transfer_direction direction,
				     unsigned long flags)
{
	return dmaengine_prep_dma_cyclic(chan, buffer, length, period, direction, flags);
}

__rust_helper dma_cookie_t rust_helper_dmaengine_submit(struct dma_async_tx_descriptor *desc)
{
	return dmaengine_submit(desc);
}

__rust_helper int rust_helper_dmaengine_desc_free(struct dma_async_tx_descriptor *desc)
{
	return dmaengine_desc_free(desc);
}

__rust_helper void rust_helper_dma_async_issue_pending(struct dma_chan *chan)
{
	dma_async_issue_pending(chan);
}

__rust_helper int rust_helper_dmaengine_terminate_sync(struct dma_chan *chan)
{
	return dmaengine_terminate_sync(chan);
}

__rust_helper enum dma_status rust_helper_dmaengine_tx_status(struct dma_chan *chan,
							     dma_cookie_t cookie,
							     struct dma_tx_state *state)
{
	return dmaengine_tx_status(chan, cookie, state);
}

__rust_helper struct device *rust_helper_dmaengine_get_dma_device(struct dma_chan *chan)
{
	return dmaengine_get_dma_device(chan);
}


__rust_helper struct device_link *
rust_helper_device_link_add_consumer(struct device *consumer, struct device *supplier)
{
	return device_link_add(consumer, supplier, DL_FLAG_AUTOREMOVE_CONSUMER);
}
