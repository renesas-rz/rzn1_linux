/*
 * Memory-mapped interface driver for DW SPI Core
 *
 * Copyright (c) 2010, Octasic semiconductor.
 *
 * DMA parts copied from spi-dw-mid.c:
 * Copyright (c) 2009, 2014 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/scatterlist.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/property.h>
#include <linux/platform_data/dma-dw.h>

#include "spi-dw.h"

#define DRIVER_NAME "dw_spi_mmio"

struct dw_spi_mmio {
	struct dw_spi  dws;
	struct clk     *clk;
	struct device  *dw_spi_dev;
	struct delayed_work	work;
	struct workqueue_struct *work_queue;
};

#define RX_BUSY		0
#define TX_BUSY		1

static int mmio_spi_dma_init(struct dw_spi *dws)
{
	struct dw_spi_mmio *dwsmmio = dws->priv;
	struct device *dev = dwsmmio->dw_spi_dev;
	int ret;

	/* RX */
	dws->rxchan = dma_request_slave_channel_reason(dev, "rx");
	if (IS_ERR(dws->rxchan))
		return PTR_ERR(dws->rxchan);

	dev_info(dev, "DMA channel RX %s\n", dma_chan_name(dws->rxchan));
	dws->master->dma_rx = dws->rxchan;

	/* TX */
	dws->txchan = dma_request_slave_channel_reason(dev, "tx");
	if (IS_ERR(dws->txchan)) {
		ret = PTR_ERR(dws->txchan);
		goto err_rx;
	}

	dev_info(dev, "DMA channel TX %s\n", dma_chan_name(dws->txchan));
	dws->master->dma_tx = dws->txchan;

	dws->dma_inited = 1;
	return 0;

err_rx:
	dma_release_channel(dws->rxchan);
	return ret;
}

static void mmio_spi_dma_exit(struct dw_spi *dws)
{
	if (!dws->dma_inited)
		return;

	dmaengine_terminate_all(dws->txchan);
	dma_release_channel(dws->txchan);

	dmaengine_terminate_all(dws->rxchan);
	dma_release_channel(dws->rxchan);
}

static irqreturn_t dma_transfer(struct dw_spi *dws)
{
	u16 irq_status = dw_readl(dws, DW_SPI_ISR);

	if (!irq_status)
		return IRQ_NONE;

	dw_readl(dws, DW_SPI_ICR);
	spi_reset_chip(dws);

	dev_err(&dws->master->dev, "%s: FIFO overrun/underrun\n", __func__);
	if (irq_status & SPI_INT_TXEI)
		dev_err(&dws->master->dev, "%s: Tx FIFO underrun\n", __func__);
	if (irq_status & SPI_INT_TXOI)
		dev_err(&dws->master->dev, "%s: Tx FIFO overrun\n", __func__);
	if (irq_status & SPI_INT_RXUI)
		dev_err(&dws->master->dev, "%s: Rx FIFO underrun\n", __func__);
	if (irq_status & SPI_INT_RXOI)
		dev_err(&dws->master->dev, "%s: Rx FIFO overrun\n", __func__);
	if (irq_status & SPI_INT_RXFI)
		dev_err(&dws->master->dev, "%s: Rx FIFO full\n", __func__);
	dws->master->cur_msg->status = -EIO;
	spi_finalize_current_transfer(dws->master);
	return IRQ_HANDLED;
}

static bool mmio_spi_can_dma(struct spi_master *master, struct spi_device *spi,
		struct spi_transfer *xfer)
{
	struct dw_spi *dws = spi_master_get_devdata(master);

	if (!dws->dma_inited)
		return false;

	return xfer->len > dws->fifo_len;
}

static enum dma_slave_buswidth convert_dma_width(u32 dma_width)
{
	if (dma_width == 1)
		return DMA_SLAVE_BUSWIDTH_1_BYTE;
	else if (dma_width == 2)
		return DMA_SLAVE_BUSWIDTH_2_BYTES;

	return DMA_SLAVE_BUSWIDTH_UNDEFINED;
}

static void dma_tx_complete_work(struct work_struct *work)
{
	struct dw_spi_mmio *dwsmmio =
		container_of(work, struct dw_spi_mmio, work.work);
	struct dw_spi *dws = &dwsmmio->dws;

	spi_finalize_current_transfer(dws->master);
}

/*
 * dws->dma_chan_busy is set before the dma transfer starts, callback for tx
 * channel will clear a corresponding bit.
 */
static void dw_spi_dma_tx_done(void *arg)
{
	struct dw_spi *dws = arg;
	struct dw_spi_mmio *dwsmmio = dws->priv;
	u32 delay;
	u32 words_to_tx;

	clear_bit(TX_BUSY, &dws->dma_chan_busy);
	if (test_bit(RX_BUSY, &dws->dma_chan_busy))
		return;

	/*
	 * Although the DMA is complete, the SPI transaction on the wire is not!
	 * DMA writes to the SPI FIFO and so we need to wait until the FIFO has
	 * been drained and shifted out.
	 */
	words_to_tx = dw_readl(dws, DW_SPI_TXFLR);

	/* Calc time until the data has been put on the wire (in us) */
	delay = 1000000 / (dws->max_freq / dw_readl(dws, DW_SPI_BAUDR));
	delay *= words_to_tx;
	delay *= (8 * dws->dma_width) + 2; /* 8 bits per byte + 2 for framing */

	/*
	 * At 500KHz SPI clock, this delay is 320us and we are in atomic
	 * context. Therefore, we defer this to a workqueue.
	 * Arbitary threshold where it is better to just delay.
	 */
	if (delay <= 20) {
		udelay(delay);
		spi_finalize_current_transfer(dws->master);
	} else {
		queue_delayed_work(dwsmmio->work_queue,
				&dwsmmio->work, usecs_to_jiffies(delay) + 1);
	}
}

static struct dma_async_tx_descriptor *dw_spi_dma_prepare_tx(struct dw_spi *dws,
		struct spi_transfer *xfer)
{
	struct dma_slave_config txconf = {
		.direction = DMA_MEM_TO_DEV,
		.dst_addr = dws->dma_addr,
		.dst_maxburst = dws->fifo_len / 2,
		.dst_addr_width = convert_dma_width(dws->dma_width),
		.device_fc = false,
	};
	struct dma_async_tx_descriptor *txdesc;
	u32 val = 0;

	if (!xfer->tx_buf)
		return NULL;

#ifdef CONFIG_SPI_DW_RZN1
	txconf.device_fc = true;

	/* The DMAC uses the maxburst size if the transfer is bigger than it */
	if (txconf.dst_maxburst == 1)
		val |= SPI_xDMACR_1_WORD_BURST;
	else if (txconf.dst_maxburst == 4)
		val |= SPI_xDMACR_4_WORD_BURST;
	else if (txconf.dst_maxburst == 8)
		val |= SPI_xDMACR_8_WORD_BURST;
	dw_writel(dws, DW_SPI_TDMACR, val);

	/* Block size */
	val |= (xfer->len / dws->dma_width) << SPI_xDMACR_BLK_SIZE_OFFSET;

	val |= SPI_xDMACR_DMA_EN;
	dw_writel(dws, DW_SPI_TDMACR, val);
#endif

	dmaengine_slave_config(dws->txchan, &txconf);

	txdesc = dmaengine_prep_slave_sg(dws->txchan,
				xfer->tx_sg.sgl,
				xfer->tx_sg.nents,
				DMA_MEM_TO_DEV,
				DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!txdesc)
		return NULL;

	txdesc->callback = dw_spi_dma_tx_done;
	txdesc->callback_param = dws;

	return txdesc;
}

/*
 * dws->dma_chan_busy is set before the dma transfer starts, callback for rx
 * channel will clear a corresponding bit.
 */
static void dw_spi_dma_rx_done(void *arg)
{
	struct dw_spi *dws = arg;

	clear_bit(RX_BUSY, &dws->dma_chan_busy);
	if (test_bit(TX_BUSY, &dws->dma_chan_busy))
		return;
	spi_finalize_current_transfer(dws->master);
}

static struct dma_async_tx_descriptor *dw_spi_dma_prepare_rx(struct dw_spi *dws,
		struct spi_transfer *xfer)
{
	struct dma_slave_config rxconf = {
		.direction = DMA_DEV_TO_MEM,
		.src_addr = dws->dma_addr,
		.src_maxburst = dws->fifo_len / 2,
		.src_addr_width = convert_dma_width(dws->dma_width),
		.device_fc = false,
	};
	struct dma_async_tx_descriptor *rxdesc;
	u32 val = 0;

	if (!xfer->rx_buf)
		return NULL;

#ifdef CONFIG_SPI_DW_RZN1
	rxconf.device_fc = true;

	/* The DMAC uses the maxburst size if the transfer is bigger than it */
	if (rxconf.src_maxburst == 1)
		val |= SPI_xDMACR_1_WORD_BURST;
	else if (rxconf.src_maxburst == 4)
		val |= SPI_xDMACR_4_WORD_BURST;
	else if (rxconf.src_maxburst == 8)
		val |= SPI_xDMACR_8_WORD_BURST;
	dw_writel(dws, DW_SPI_RDMACR, val);

	/* Block size */
	val |= (xfer->len / dws->dma_width) << SPI_xDMACR_BLK_SIZE_OFFSET;

	val |= SPI_xDMACR_DMA_EN;
	dw_writel(dws, DW_SPI_RDMACR, val);
#endif

	dmaengine_slave_config(dws->rxchan, &rxconf);

	rxdesc = dmaengine_prep_slave_sg(dws->rxchan,
				xfer->rx_sg.sgl,
				xfer->rx_sg.nents,
				DMA_DEV_TO_MEM,
				DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!rxdesc)
		return NULL;

	rxdesc->callback = dw_spi_dma_rx_done;
	rxdesc->callback_param = dws;

	return rxdesc;
}

static int mmio_spi_dma_setup(struct dw_spi *dws, struct spi_transfer *xfer)
{
	u16 dma_ctrl = 0;
	u32 cr0;

	/* watermarks for initiating DMA requests, both set to half the FIFO */
	dw_writel(dws, DW_SPI_DMARDLR, (dws->fifo_len / 2) - 1);
	dw_writel(dws, DW_SPI_DMATDLR, dws->fifo_len / 2);

	/* Always enable tx */
	dma_ctrl |= SPI_DMA_TDMAE;
	if (xfer->rx_buf)
		dma_ctrl |= SPI_DMA_RDMAE;
	dw_writel(dws, DW_SPI_DMACR, dma_ctrl);

	/* If tx only, set transfer mode to avoid data in the rx fifo */
	cr0 = dw_readl(dws, DW_SPI_CTRL0);
	cr0 &= ~SPI_TMOD_MASK;
	if (!xfer->rx_buf)
		cr0 |= (SPI_TMOD_TO << SPI_TMOD_OFFSET);
	else
		cr0 |= (SPI_TMOD_TR << SPI_TMOD_OFFSET);
	dw_writel(dws, DW_SPI_CTRL0, cr0);

	/* Set the interrupt mask */
	spi_umask_intr(dws, SPI_INT_TXOI | SPI_INT_RXUI | SPI_INT_RXOI);

	dws->transfer_handler = dma_transfer;

	return 0;
}

static int mmio_spi_dma_transfer(struct dw_spi *dws, struct spi_transfer *xfer)
{
	struct dma_async_tx_descriptor *txdesc, *rxdesc;

	/* Prepare the TX dma transfer */
	txdesc = dw_spi_dma_prepare_tx(dws, xfer);

	/* Prepare the RX dma transfer */
	rxdesc = dw_spi_dma_prepare_rx(dws, xfer);

	/* rx must be started before tx due to spi instinct */
	if (rxdesc) {
		set_bit(RX_BUSY, &dws->dma_chan_busy);
		dmaengine_submit(rxdesc);
		dma_async_issue_pending(dws->rxchan);
	}

	if (txdesc) {
		set_bit(TX_BUSY, &dws->dma_chan_busy);
		dmaengine_submit(txdesc);
		dma_async_issue_pending(dws->txchan);
	}

	return 0;
}

static void mmio_spi_dma_stop(struct dw_spi *dws)
{
	if (test_bit(TX_BUSY, &dws->dma_chan_busy)) {
		dmaengine_terminate_all(dws->txchan);
		clear_bit(TX_BUSY, &dws->dma_chan_busy);
	}
	if (test_bit(RX_BUSY, &dws->dma_chan_busy)) {
		dmaengine_terminate_all(dws->rxchan);
		clear_bit(RX_BUSY, &dws->dma_chan_busy);
	}
}

static struct dw_spi_dma_ops mmio_dma_ops = {
	.dma_init	= mmio_spi_dma_init,
	.dma_exit	= mmio_spi_dma_exit,
	.dma_setup	= mmio_spi_dma_setup,
	.can_dma	= mmio_spi_can_dma,
	.dma_transfer	= mmio_spi_dma_transfer,
	.dma_stop	= mmio_spi_dma_stop,
};

static int dw_spi_mmio_probe(struct platform_device *pdev)
{
	struct dw_spi_mmio *dwsmmio;
	struct dw_spi *dws;
	struct resource *mem;
	int ret;
	int num_cs;

	dwsmmio = devm_kzalloc(&pdev->dev, sizeof(struct dw_spi_mmio),
			GFP_KERNEL);
	if (!dwsmmio)
		return -ENOMEM;

	dws = &dwsmmio->dws;

	/* Get basic io resource and map it */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dws->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(dws->regs)) {
		dev_err(&pdev->dev, "SPI region map failed\n");
		return PTR_ERR(dws->regs);
	}

	dws->irq = platform_get_irq(pdev, 0);
	if (dws->irq < 0) {
		dev_err(&pdev->dev, "no irq resource?\n");
		return dws->irq; /* -ENXIO */
	}

	dwsmmio->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(dwsmmio->clk))
		return PTR_ERR(dwsmmio->clk);
	ret = clk_prepare_enable(dwsmmio->clk);
	if (ret)
		return ret;

	dws->bus_num = pdev->id;

	dws->max_freq = clk_get_rate(dwsmmio->clk);

	device_property_read_u32(&pdev->dev, "reg-io-width", &dws->reg_io_width);

	num_cs = 4;

	device_property_read_u32(&pdev->dev, "num-cs", &num_cs);

	dws->num_cs = num_cs;

#ifdef CONFIG_SPI_DW_RZN1
	/* default to sw mode on that platform, the HW mode is next to useless
	 * for most purposes */
	dws->mode = (1 << dws->num_cs) -1;
	if (pdev->dev.of_node) {
		struct device_node *child = NULL;

		of_property_read_u32(pdev->dev.of_node,
			"renesas,rzn1-cs-mode", &dws->mode);
		/* Also check the mode in each of the separate child nodes */
		for_each_available_child_of_node(pdev->dev.of_node, child) {
			u32 addr;
			int ret = of_property_read_u32(child, "reg", &addr);

			if (ret || addr >= dws->num_cs) {
				dev_warn(&pdev->dev, "invalid slave %s",
					child->name);
				continue;
			}
			if (of_property_read_bool(child, "renesas,rzn1-cs-sw"))
				dws->mode |= (1 << addr);
			if (of_property_read_bool(child, "renesas,rzn1-cs-hw"))
				dws->mode &= ~(1 << addr);
		}
	}
#endif

	if (pdev->dev.of_node) {
		int i;

		for (i = 0; i < dws->num_cs; i++) {
			int cs_gpio = of_get_named_gpio(pdev->dev.of_node,
					"cs-gpios", i);

			if (cs_gpio == -EPROBE_DEFER) {
				ret = cs_gpio;
				goto out;
			}

			if (gpio_is_valid(cs_gpio)) {
				ret = devm_gpio_request(&pdev->dev, cs_gpio,
						dev_name(&pdev->dev));
				if (ret)
					goto out;
			}
		}
	}

	dwsmmio->dw_spi_dev = &pdev->dev;
	dws->priv = dwsmmio;
	dws->paddr = (unsigned long)(mem->start);
	dws->dma_ops = &mmio_dma_ops;

	INIT_DELAYED_WORK(&dwsmmio->work, dma_tx_complete_work);
	dwsmmio->work_queue = create_singlethread_workqueue(dev_name(&pdev->dev));
	if (!dwsmmio->work_queue) {
		dev_err(&pdev->dev, "failed to create workqueue\n");
		ret = -ENOMEM;
		goto out;
	}

	ret = dw_spi_add_host(&pdev->dev, dws);
	if (ret)
		goto out;

	/* DMA requires tx whilst doing an rx */
	dws->master->flags = SPI_MASTER_MUST_TX;

	platform_set_drvdata(pdev, dwsmmio);
	return 0;

out:
	clk_disable_unprepare(dwsmmio->clk);
	return ret;
}

static int dw_spi_mmio_remove(struct platform_device *pdev)
{
	struct dw_spi_mmio *dwsmmio = platform_get_drvdata(pdev);

	dw_spi_remove_host(&dwsmmio->dws);
	clk_disable_unprepare(dwsmmio->clk);

	return 0;
}

static const struct of_device_id dw_spi_mmio_of_match[] = {
	{ .compatible = "snps,dw-apb-ssi", },
	{ /* end of table */}
};
MODULE_DEVICE_TABLE(of, dw_spi_mmio_of_match);

static struct platform_driver dw_spi_mmio_driver = {
	.probe		= dw_spi_mmio_probe,
	.remove		= dw_spi_mmio_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.of_match_table = dw_spi_mmio_of_match,
	},
};
module_platform_driver(dw_spi_mmio_driver);

MODULE_AUTHOR("Jean-Hugues Deschenes <jean-hugues.deschenes@octasic.com>");
MODULE_DESCRIPTION("Memory-mapped I/O interface driver for DW SPI Core");
MODULE_LICENSE("GPL v2");
