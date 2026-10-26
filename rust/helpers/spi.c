// SPDX-License-Identifier: GPL-2.0
#include <linux/spi/spi.h>

__rust_helper struct spi_controller *
rust_helper_spi_alloc_host(struct device *dev)
{
	return spi_alloc_host(dev, 0);
}

__rust_helper void rust_helper_spi_controller_put(struct spi_controller *ctlr)
{
	spi_controller_put(ctlr);
}

__rust_helper void *
rust_helper_spi_controller_get_devdata(struct spi_controller *ctlr)
{
	return spi_controller_get_devdata(ctlr);
}

__rust_helper void rust_helper_spi_controller_init(
	struct spi_controller *ctlr, void *data,
	int (*transfer)(struct spi_controller *, struct spi_device *,
			struct spi_transfer *),
	u32 min_hz, u32 max_hz)
{
	spi_controller_set_devdata(ctlr, data);
	ctlr->transfer_one = transfer;
	ctlr->mode_bits = 0;
	ctlr->bits_per_word_mask = SPI_BPW_MASK(8);
	ctlr->flags = SPI_CONTROLLER_NO_RX;
	ctlr->num_chipselect = 1;
	ctlr->use_gpio_descriptors = true;
	ctlr->min_speed_hz = min_hz;
	ctlr->max_speed_hz = max_hz;
}


#ifdef CONFIG_RUST_SPI_KUNIT_TEST
__rust_helper int rust_helper_spi_sync_transfer(struct spi_device *spi,
                                               struct spi_transfer *xfers,
                                               unsigned int count)
{
    return spi_sync_transfer(spi, xfers, count);
}
#endif
