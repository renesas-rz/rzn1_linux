// SPDX-License-Identifier: GPL-2.0-only
/*
 * Memory-mapped interface driver for DW SPI Core
 *
 * Copyright (c) 2010, Octasic semiconductor.
 */

#include <linux/clk.h>
#include <linux/dmaengine.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/scatterlist.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/acpi.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/platform_data/dma-dw.h>

#include "spi-dw.h"

#define DRIVER_NAME "dw_spi_mmio"

struct dw_spi_mmio {
	struct dw_spi  dws;
	struct clk     *clk;
	struct clk     *pclk;
	void           *priv;
	struct reset_control *rstc;
	struct device  *dw_spi_dev;
	struct delayed_work	work;
	struct workqueue_struct *work_queue;
};

#define RX_BUSY		0
#define TX_BUSY		1

static int mmio_spi_dma_init(struct device *dev, struct dw_spi *dws)
{
	int ret;

	/* RX */
	dws->rxchan = dma_request_slave_channel(dev, "rx");
	if (IS_ERR(dws->rxchan))
		return PTR_ERR(dws->rxchan);

	dev_info(dev, "DMA channel RX %s\n", dma_chan_name(dws->rxchan));
	dws->master->dma_rx = dws->rxchan;

	/* TX */
	dws->txchan = dma_request_slave_channel(dev, "tx");
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

	dmaengine_terminate_sync(dws->txchan);
	dma_release_channel(dws->txchan);

	dmaengine_terminate_sync(dws->rxchan);
	dma_release_channel(dws->rxchan);
}

static bool mmio_spi_can_dma(struct spi_controller *master,
			     struct spi_device *spi, struct spi_transfer *xfer)
{
	struct dw_spi *dws = spi_controller_get_devdata(master);

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
	u32 words_to_tx;
	u32 delay;

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
	 * Arbitrary threshold where it is better to just delay.
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
	cr0 = dw_readl(dws, DW_SPI_CTRLR0);
	cr0 &= ~SPI_TMOD_MASK;
	if (!xfer->rx_buf)
		cr0 |= (SPI_TMOD_TO << SPI_TMOD_OFFSET);
	else
		cr0 |= (SPI_TMOD_TR << SPI_TMOD_OFFSET);
	dw_writel(dws, DW_SPI_CTRLR0, cr0);

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

	return 1;
}

static void mmio_spi_dma_stop(struct dw_spi *dws)
{
	if (test_bit(TX_BUSY, &dws->dma_chan_busy)) {
		dmaengine_terminate_sync(dws->txchan);
		clear_bit(TX_BUSY, &dws->dma_chan_busy);
	}

	if (test_bit(RX_BUSY, &dws->dma_chan_busy)) {
		dmaengine_terminate_sync(dws->rxchan);
		clear_bit(RX_BUSY, &dws->dma_chan_busy);
	}
}

static const struct dw_spi_dma_ops mmio_dma_ops = {
	.dma_init	= mmio_spi_dma_init,
	.dma_exit	= mmio_spi_dma_exit,
	.dma_setup	= mmio_spi_dma_setup,
	.can_dma	= mmio_spi_can_dma,
	.dma_transfer	= mmio_spi_dma_transfer,
	.dma_stop	= mmio_spi_dma_stop,
};

#define MSCC_CPU_SYSTEM_CTRL_GENERAL_CTRL	0x24
#define OCELOT_IF_SI_OWNER_OFFSET		4
#define JAGUAR2_IF_SI_OWNER_OFFSET		6
#define MSCC_IF_SI_OWNER_MASK			GENMASK(1, 0)
#define MSCC_IF_SI_OWNER_SISL			0
#define MSCC_IF_SI_OWNER_SIBM			1
#define MSCC_IF_SI_OWNER_SIMC			2

#define MSCC_SPI_MST_SW_MODE			0x14
#define MSCC_SPI_MST_SW_MODE_SW_PIN_CTRL_MODE	BIT(13)
#define MSCC_SPI_MST_SW_MODE_SW_SPI_CS(x)	(x << 5)

#define SPARX5_FORCE_ENA			0xa4
#define SPARX5_FORCE_VAL			0xa8

struct dw_spi_mscc {
	struct regmap       *syscon;
	void __iomem        *spi_mst; /* Not sparx5 */
};

/*
 * The Designware SPI controller (referred to as master in the documentation)
 * automatically deasserts chip select when the tx fifo is empty. The chip
 * selects then needs to be either driven as GPIOs or, for the first 4 using
 * the SPI boot controller registers. the final chip select is an OR gate
 * between the Designware SPI controller and the SPI boot controller.
 */
static void dw_spi_mscc_set_cs(struct spi_device *spi, bool enable)
{
	struct dw_spi *dws = spi_master_get_devdata(spi->master);
	struct dw_spi_mmio *dwsmmio = container_of(dws, struct dw_spi_mmio, dws);
	struct dw_spi_mscc *dwsmscc = dwsmmio->priv;
	u32 cs = spi->chip_select;

	if (cs < 4) {
		u32 sw_mode = MSCC_SPI_MST_SW_MODE_SW_PIN_CTRL_MODE;

		if (!enable)
			sw_mode |= MSCC_SPI_MST_SW_MODE_SW_SPI_CS(BIT(cs));

		writel(sw_mode, dwsmscc->spi_mst + MSCC_SPI_MST_SW_MODE);
	}

	dw_spi_set_cs(spi, enable);
}

static int dw_spi_mscc_init(struct platform_device *pdev,
			    struct dw_spi_mmio *dwsmmio,
			    const char *cpu_syscon, u32 if_si_owner_offset)
{
	struct dw_spi_mscc *dwsmscc;

	dwsmscc = devm_kzalloc(&pdev->dev, sizeof(*dwsmscc), GFP_KERNEL);
	if (!dwsmscc)
		return -ENOMEM;

	dwsmscc->spi_mst = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(dwsmscc->spi_mst)) {
		dev_err(&pdev->dev, "SPI_MST region map failed\n");
		return PTR_ERR(dwsmscc->spi_mst);
	}

	dwsmscc->syscon = syscon_regmap_lookup_by_compatible(cpu_syscon);
	if (IS_ERR(dwsmscc->syscon))
		return PTR_ERR(dwsmscc->syscon);

	/* Deassert all CS */
	writel(0, dwsmscc->spi_mst + MSCC_SPI_MST_SW_MODE);

	/* Select the owner of the SI interface */
	regmap_update_bits(dwsmscc->syscon, MSCC_CPU_SYSTEM_CTRL_GENERAL_CTRL,
			   MSCC_IF_SI_OWNER_MASK << if_si_owner_offset,
			   MSCC_IF_SI_OWNER_SIMC << if_si_owner_offset);

	dwsmmio->dws.set_cs = dw_spi_mscc_set_cs;
	dwsmmio->priv = dwsmscc;

	return 0;
}

static int dw_spi_mscc_ocelot_init(struct platform_device *pdev,
				   struct dw_spi_mmio *dwsmmio)
{
	return dw_spi_mscc_init(pdev, dwsmmio, "mscc,ocelot-cpu-syscon",
				OCELOT_IF_SI_OWNER_OFFSET);
}

static int dw_spi_mscc_jaguar2_init(struct platform_device *pdev,
				    struct dw_spi_mmio *dwsmmio)
{
	return dw_spi_mscc_init(pdev, dwsmmio, "mscc,jaguar2-cpu-syscon",
				JAGUAR2_IF_SI_OWNER_OFFSET);
}

/*
 * The Designware SPI controller (referred to as master in the
 * documentation) automatically deasserts chip select when the tx fifo
 * is empty. The chip selects then needs to be driven by a CS override
 * register. enable is an active low signal.
 */
static void dw_spi_sparx5_set_cs(struct spi_device *spi, bool enable)
{
	struct dw_spi *dws = spi_master_get_devdata(spi->master);
	struct dw_spi_mmio *dwsmmio = container_of(dws, struct dw_spi_mmio, dws);
	struct dw_spi_mscc *dwsmscc = dwsmmio->priv;
	u8 cs = spi->chip_select;

	if (!enable) {
		/* CS override drive enable */
		regmap_write(dwsmscc->syscon, SPARX5_FORCE_ENA, 1);
		/* Now set CSx enabled */
		regmap_write(dwsmscc->syscon, SPARX5_FORCE_VAL, ~BIT(cs));
		/* Allow settle */
		usleep_range(1, 5);
	} else {
		/* CS value */
		regmap_write(dwsmscc->syscon, SPARX5_FORCE_VAL, ~0);
		/* Allow settle */
		usleep_range(1, 5);
		/* CS override drive disable */
		regmap_write(dwsmscc->syscon, SPARX5_FORCE_ENA, 0);
	}

	dw_spi_set_cs(spi, enable);
}

static int dw_spi_mscc_sparx5_init(struct platform_device *pdev,
				   struct dw_spi_mmio *dwsmmio)
{
	const char *syscon_name = "microchip,sparx5-cpu-syscon";
	struct device *dev = &pdev->dev;
	struct dw_spi_mscc *dwsmscc;

	if (!IS_ENABLED(CONFIG_SPI_MUX)) {
		dev_err(dev, "This driver needs CONFIG_SPI_MUX\n");
		return -EOPNOTSUPP;
	}

	dwsmscc = devm_kzalloc(dev, sizeof(*dwsmscc), GFP_KERNEL);
	if (!dwsmscc)
		return -ENOMEM;

	dwsmscc->syscon =
		syscon_regmap_lookup_by_compatible(syscon_name);
	if (IS_ERR(dwsmscc->syscon)) {
		dev_err(dev, "No syscon map %s\n", syscon_name);
		return PTR_ERR(dwsmscc->syscon);
	}

	dwsmmio->dws.set_cs = dw_spi_sparx5_set_cs;
	dwsmmio->priv = dwsmscc;

	return 0;
}

static int dw_spi_alpine_init(struct platform_device *pdev,
			      struct dw_spi_mmio *dwsmmio)
{
	dwsmmio->dws.caps = DW_SPI_CAP_CS_OVERRIDE;

	return 0;
}

static int dw_spi_dw_apb_init(struct platform_device *pdev,
			      struct dw_spi_mmio *dwsmmio)
{
	dw_spi_dma_setup_generic(&dwsmmio->dws);

	return 0;
}

static int dw_spi_dwc_ssi_init(struct platform_device *pdev,
			       struct dw_spi_mmio *dwsmmio)
{
	dwsmmio->dws.caps = DW_SPI_CAP_DWC_SSI;

	dw_spi_dma_setup_generic(&dwsmmio->dws);

	return 0;
}

static int dw_spi_keembay_init(struct platform_device *pdev,
			       struct dw_spi_mmio *dwsmmio)
{
	dwsmmio->dws.caps = DW_SPI_CAP_KEEMBAY_MST | DW_SPI_CAP_DWC_SSI;

	return 0;
}

static int dw_spi_canaan_k210_init(struct platform_device *pdev,
				   struct dw_spi_mmio *dwsmmio)
{
	/*
	 * The Canaan Kendryte K210 SoC DW apb_ssi v4 spi controller is
	 * documented to have a 32 word deep TX and RX FIFO, which
	 * spi_hw_init() detects. However, when the RX FIFO is filled up to
	 * 32 entries (RXFLR = 32), an RX FIFO overrun error occurs. Avoid this
	 * problem by force setting fifo_len to 31.
	 */
	dwsmmio->dws.fifo_len = 31;

	return 0;
}

static int dw_spi_mmio_probe(struct platform_device *pdev)
{
	int (*init_func)(struct platform_device *pdev,
			 struct dw_spi_mmio *dwsmmio);
	struct dw_spi_mmio *dwsmmio;
	struct resource *mem;
	struct dw_spi *dws;
	int ret;
	int num_cs;

	dwsmmio = devm_kzalloc(&pdev->dev, sizeof(struct dw_spi_mmio),
			GFP_KERNEL);
	if (!dwsmmio)
		return -ENOMEM;

	dws = &dwsmmio->dws;

	/* Get basic io resource and map it */
	dws->regs = devm_platform_get_and_ioremap_resource(pdev, 0, &mem);
	if (IS_ERR(dws->regs))
		return PTR_ERR(dws->regs);

	dws->paddr = mem->start;

	dws->irq = platform_get_irq(pdev, 0);
	if (dws->irq < 0)
		return dws->irq; /* -ENXIO */

	dwsmmio->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(dwsmmio->clk))
		return PTR_ERR(dwsmmio->clk);
	ret = clk_prepare_enable(dwsmmio->clk);
	if (ret)
		return ret;

	/* Optional clock needed to access the registers */
	dwsmmio->pclk = devm_clk_get_optional(&pdev->dev, "pclk");
	if (IS_ERR(dwsmmio->pclk)) {
		ret = PTR_ERR(dwsmmio->pclk);
		goto out_clk;
	}
	ret = clk_prepare_enable(dwsmmio->pclk);
	if (ret)
		goto out_clk;

	/* find an optional reset controller */
	dwsmmio->rstc = devm_reset_control_get_optional_exclusive(&pdev->dev, "spi");
	if (IS_ERR(dwsmmio->rstc)) {
		ret = PTR_ERR(dwsmmio->rstc);
		goto out_clk;
	}
	reset_control_deassert(dwsmmio->rstc);

	dws->bus_num = pdev->id;

	dws->max_freq = clk_get_rate(dwsmmio->clk);

	device_property_read_u32(&pdev->dev, "reg-io-width", &dws->reg_io_width);

	num_cs = 4;

	device_property_read_u32(&pdev->dev, "num-cs", &num_cs);

	dws->num_cs = num_cs;

	init_func = device_get_match_data(&pdev->dev);
	if (init_func) {
		ret = init_func(pdev, dwsmmio);
		if (ret)
			goto out;
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

	pm_runtime_enable(&pdev->dev);

	ret = dw_spi_add_host(&pdev->dev, dws);
	if (ret)
		goto out;

	/* DMA requires tx whilst doing an rx */
	dws->master->flags |= SPI_MASTER_MUST_TX;

	platform_set_drvdata(pdev, dwsmmio);
	return 0;

out:
	pm_runtime_disable(&pdev->dev);
	clk_disable_unprepare(dwsmmio->pclk);
out_clk:
	clk_disable_unprepare(dwsmmio->clk);
	reset_control_assert(dwsmmio->rstc);

	return ret;
}

static int dw_spi_mmio_remove(struct platform_device *pdev)
{
	struct dw_spi_mmio *dwsmmio = platform_get_drvdata(pdev);

	dw_spi_remove_host(&dwsmmio->dws);
	pm_runtime_disable(&pdev->dev);
	clk_disable_unprepare(dwsmmio->pclk);
	clk_disable_unprepare(dwsmmio->clk);
	reset_control_assert(dwsmmio->rstc);

	return 0;
}

static const struct of_device_id dw_spi_mmio_of_match[] = {
	{ .compatible = "snps,dw-apb-ssi", .data = dw_spi_dw_apb_init},
	{ .compatible = "mscc,ocelot-spi", .data = dw_spi_mscc_ocelot_init},
	{ .compatible = "mscc,jaguar2-spi", .data = dw_spi_mscc_jaguar2_init},
	{ .compatible = "amazon,alpine-dw-apb-ssi", .data = dw_spi_alpine_init},
	{ .compatible = "renesas,rzn1-spi", .data = dw_spi_dw_apb_init},
	{ .compatible = "snps,dwc-ssi-1.01a", .data = dw_spi_dwc_ssi_init},
	{ .compatible = "intel,keembay-ssi", .data = dw_spi_keembay_init},
	{ .compatible = "microchip,sparx5-spi", dw_spi_mscc_sparx5_init},
	{ .compatible = "canaan,k210-spi", dw_spi_canaan_k210_init},
	{ /* end of table */}
};
MODULE_DEVICE_TABLE(of, dw_spi_mmio_of_match);

#ifdef CONFIG_ACPI
static const struct acpi_device_id dw_spi_mmio_acpi_match[] = {
	{"HISI0173", (kernel_ulong_t)dw_spi_dw_apb_init},
	{},
};
MODULE_DEVICE_TABLE(acpi, dw_spi_mmio_acpi_match);
#endif

static struct platform_driver dw_spi_mmio_driver = {
	.probe		= dw_spi_mmio_probe,
	.remove		= dw_spi_mmio_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.of_match_table = dw_spi_mmio_of_match,
		.acpi_match_table = ACPI_PTR(dw_spi_mmio_acpi_match),
	},
};
module_platform_driver(dw_spi_mmio_driver);

MODULE_AUTHOR("Jean-Hugues Deschenes <jean-hugues.deschenes@octasic.com>");
MODULE_DESCRIPTION("Memory-mapped I/O interface driver for DW SPI Core");
MODULE_LICENSE("GPL v2");
