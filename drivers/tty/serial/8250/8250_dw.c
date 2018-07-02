/*
 * Synopsys DesignWare 8250 driver.
 *
 * Copyright 2011 Picochip, Jamie Iles.
 * Copyright 2013 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * The Synopsys DesignWare 8250 has an extra feature whereby it detects if the
 * LCR is written whilst busy.  If it is, then a busy detect interrupt is
 * raised, the LCR needs to be rewritten and the uart status register read.
 */
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/serial_8250.h>
#include <linux/serial_reg.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/acpi.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/pm_runtime.h>

#include <asm/byteorder.h>

#include "8250.h"

/* Offsets for the DesignWare specific registers */
#define DW_UART_USR	0x1f /* UART Status Register */
#define DW_UART_CPR	0xf4 /* Component Parameter Register */
#define DW_UART_UCV	0xf8 /* UART Component Version */

/* Component Parameter Register bits */
#define DW_UART_CPR_ABP_DATA_WIDTH	(3 << 0)
#define DW_UART_CPR_AFCE_MODE		(1 << 4)
#define DW_UART_CPR_THRE_MODE		(1 << 5)
#define DW_UART_CPR_SIR_MODE		(1 << 6)
#define DW_UART_CPR_SIR_LP_MODE		(1 << 7)
#define DW_UART_CPR_ADDITIONAL_FEATURES	(1 << 8)
#define DW_UART_CPR_FIFO_ACCESS		(1 << 9)
#define DW_UART_CPR_FIFO_STAT		(1 << 10)
#define DW_UART_CPR_SHADOW		(1 << 11)
#define DW_UART_CPR_ENCODED_PARMS	(1 << 12)
#define DW_UART_CPR_DMA_EXTRA		(1 << 13)
#define DW_UART_CPR_FIFO_MODE		(0xff << 16)
/* Helper for fifo size calculation */
#define DW_UART_CPR_FIFO_SIZE(a)	(((a >> 16) & 0xff) * 16)

/* Offsets for the Renesas RZ/N1 DesignWare specific registers */
#define RZN1_UART_DMASA		0xa8	/* DMA Software Ack */
#define RZN1_UART_TDMACR	0x10c	/* DMA Control Register Transmit Mode */
#define RZN1_UART_RDMACR	0x110	/* DMA Control Register Receive Mode */
#define RZN1_UART_xDMACR_DMA_EN		(1 << 0)
#define RZN1_UART_xDMACR_1_WORD_BURST	(0 << 1)
#define RZN1_UART_xDMACR_4_WORD_BURST	(1 << 1)
#define RZN1_UART_xDMACR_8_WORD_BURST	(2 << 1)
#define RZN1_UART_xDMACR_BLK_SZ_OFFSET	3

struct dw8250_data {
	u8			usr_reg;
	int			line;
	int			msr_mask_on;
	int			msr_mask_off;
	struct clk		*clk;
	struct clk		*pclk;
	struct reset_control	*rst;
	struct uart_8250_dma	dma;

	unsigned int		skip_autocfg:1;
	unsigned int		uart_16550_compatible:1;
	unsigned int		dma_capable:1;
	bool			is_rzn1:1;
	u32			cpr_val;
};

static inline int dw8250_modify_msr(struct uart_port *p, int offset, int value)
{
	struct dw8250_data *d = p->private_data;

	/* Override any modem control signals if needed */
	if (offset == UART_MSR) {
		value |= d->msr_mask_on;
		value &= ~d->msr_mask_off;
	}

	return value;
}

static void dw8250_force_idle(struct uart_port *p)
{
	struct uart_8250_port *up = up_to_u8250p(p);

	serial8250_clear_and_reinit_fifos(up);
	(void)p->serial_in(p, UART_RX);
}

static void dw8250_check_lcr(struct uart_port *p, int value)
{
	void __iomem *offset = p->membase + (UART_LCR << p->regshift);
	int tries = 1000;

	/* Make sure LCR write wasn't ignored */
	while (tries--) {
		unsigned int lcr = p->serial_in(p, UART_LCR);

		if ((value & ~UART_LCR_SPAR) == (lcr & ~UART_LCR_SPAR))
			return;

		dw8250_force_idle(p);

#ifdef CONFIG_64BIT
		if (p->type == PORT_OCTEON)
			__raw_writeq(value & 0xff, offset);
		else
#endif
		if (p->iotype == UPIO_MEM32)
			writel(value, offset);
		else if (p->iotype == UPIO_MEM32BE)
			iowrite32be(value, offset);
		else
			writeb(value, offset);
	}
	/*
	 * FIXME: this deadlocks if port->lock is already held
	 * dev_err(p->dev, "Couldn't set LCR to %d\n", value);
	 */
}

static void dw8250_serial_out(struct uart_port *p, int offset, int value)
{
	struct dw8250_data *d = p->private_data;

	writeb(value, p->membase + (offset << p->regshift));

	if (offset == UART_LCR && !d->uart_16550_compatible)
		dw8250_check_lcr(p, value);
}

static unsigned int dw8250_serial_in(struct uart_port *p, int offset)
{
	unsigned int value = readb(p->membase + (offset << p->regshift));

	return dw8250_modify_msr(p, offset, value);
}

#ifdef CONFIG_64BIT
static unsigned int dw8250_serial_inq(struct uart_port *p, int offset)
{
	unsigned int value;

	value = (u8)__raw_readq(p->membase + (offset << p->regshift));

	return dw8250_modify_msr(p, offset, value);
}

static void dw8250_serial_outq(struct uart_port *p, int offset, int value)
{
	struct dw8250_data *d = p->private_data;

	value &= 0xff;
	__raw_writeq(value, p->membase + (offset << p->regshift));
	/* Read back to ensure register write ordering. */
	__raw_readq(p->membase + (UART_LCR << p->regshift));

	if (offset == UART_LCR && !d->uart_16550_compatible)
		dw8250_check_lcr(p, value);
}
#endif /* CONFIG_64BIT */

static void dw8250_serial_out32(struct uart_port *p, int offset, int value)
{
	struct dw8250_data *d = p->private_data;

	writel(value, p->membase + (offset << p->regshift));

	if (offset == UART_LCR && !d->uart_16550_compatible)
		dw8250_check_lcr(p, value);
}

static unsigned int dw8250_serial_in32(struct uart_port *p, int offset)
{
	unsigned int value = readl(p->membase + (offset << p->regshift));

	return dw8250_modify_msr(p, offset, value);
}

static void dw8250_serial_out32be(struct uart_port *p, int offset, int value)
{
	struct dw8250_data *d = p->private_data;

	iowrite32be(value, p->membase + (offset << p->regshift));

	if (offset == UART_LCR && !d->uart_16550_compatible)
		dw8250_check_lcr(p, value);
}

static unsigned int dw8250_serial_in32be(struct uart_port *p, int offset)
{
       unsigned int value = ioread32be(p->membase + (offset << p->regshift));

       return dw8250_modify_msr(p, offset, value);
}


static void rzn1_8250_handle_irq(struct uart_port *port, unsigned int iir)
{
	struct uart_8250_port *up = up_to_u8250p(port);
	struct uart_8250_dma *dma = up->dma;
	unsigned char status;

	if (up->dma && dma->rx_running) {
		status = port->serial_in(port, UART_LSR);
		if (status & (UART_LSR_DR | UART_LSR_BI)) {
			/* Stop the DMA transfer */
			writel(0, port->membase + RZN1_UART_RDMACR);
			writel(1, port->membase + RZN1_UART_DMASA);
		}
	}
}

static int dw8250_handle_irq(struct uart_port *p)
{
	struct dw8250_data *d = p->private_data;
	unsigned int iir = p->serial_in(p, UART_IIR);

	if (d->is_rzn1 && ((iir & 0x3f) == UART_IIR_RX_TIMEOUT))
		rzn1_8250_handle_irq(p, iir);

	if (serial8250_handle_irq(p, iir))
		return 1;

	if ((iir & UART_IIR_BUSY) == UART_IIR_BUSY) {
		/* Clear the USR */
		(void)p->serial_in(p, d->usr_reg);

		return 1;
	}

	return 0;
}

static void
dw8250_do_pm(struct uart_port *port, unsigned int state, unsigned int old)
{
	if (!state)
		pm_runtime_get_sync(port->dev);

	serial8250_do_pm(port, state, old);

	if (state)
		pm_runtime_put_sync_suspend(port->dev);
}

static void dw8250_set_termios(struct uart_port *p, struct ktermios *termios,
			       struct ktermios *old)
{
	unsigned int baud = tty_termios_baud_rate(termios);
	struct dw8250_data *d = p->private_data;
	unsigned int rate;
	int ret;

	if (IS_ERR(d->clk) || !old)
		goto out;

	clk_disable_unprepare(d->clk);
	rate = clk_round_rate(d->clk, baud * 16);
	ret = clk_set_rate(d->clk, rate);
	clk_prepare_enable(d->clk);

	if (!ret)
		p->uartclk = rate;

	p->status &= ~UPSTAT_AUTOCTS;
	if (termios->c_cflag & CRTSCTS)
		p->status |= UPSTAT_AUTOCTS;

out:
	serial8250_do_set_termios(p, termios, old);
}

/*
 * dw8250_fallback_dma_filter will prevent the UART from getting just any free
 * channel on platforms that have DMA engines, but don't have any channels
 * assigned to the UART.
 *
 * REVISIT: This is a work around for limitation in the DMA Engine API. Once the
 * core problem is fixed, this function is no longer needed.
 */
static bool dw8250_fallback_dma_filter(struct dma_chan *chan, void *param)
{
	return false;
}

static bool dw8250_idma_filter(struct dma_chan *chan, void *param)
{
	return param == chan->device->dev->parent;
}

static u32 rzn1_get_dmacr_burst(int max_burst)
{
	u32 val = 0;

	if (max_burst >= 8)
		val = RZN1_UART_xDMACR_8_WORD_BURST;
	else if (max_burst >= 4)
		val = RZN1_UART_xDMACR_4_WORD_BURST;
	else
		val = RZN1_UART_xDMACR_1_WORD_BURST;

	return val;
}

static int rzn1_dw8250_tx_dma(struct uart_8250_port *p)
{
	struct uart_port		*up = &p->port;
	struct uart_8250_dma		*dma = p->dma;
	struct circ_buf			*xmit = &p->port.state->xmit;
	int tx_size;
	u32 val;

	if (uart_tx_stopped(&p->port) || dma->tx_running ||
	    uart_circ_empty(xmit))
		return 0;

	tx_size = CIRC_CNT_TO_END(xmit->head, xmit->tail, UART_XMIT_SIZE);

	writel(0, up->membase + RZN1_UART_TDMACR);
	val = rzn1_get_dmacr_burst(dma->txconf.dst_maxburst);
	val |= tx_size << RZN1_UART_xDMACR_BLK_SZ_OFFSET;
	val |= RZN1_UART_xDMACR_DMA_EN;
	writel(val, up->membase + RZN1_UART_TDMACR);

	return serial8250_tx_dma(p);
}

static int rzn1_dw8250_rx_dma(struct uart_8250_port *p)
{
	struct uart_port		*up = &p->port;
	struct uart_8250_dma		*dma = p->dma;
	u32 val;

	if (dma->rx_running)
		return 0;

	writel(0, up->membase + RZN1_UART_RDMACR);
	val = rzn1_get_dmacr_burst(dma->rxconf.src_maxburst);
	val |= dma->rx_size << RZN1_UART_xDMACR_BLK_SZ_OFFSET;
	val |= RZN1_UART_xDMACR_DMA_EN;
	writel(val, up->membase + RZN1_UART_RDMACR);

	return serial8250_rx_dma(p);
}

static void dw8250_quirks(struct uart_port *p, struct dw8250_data *data)
{
	if (p->dev->of_node) {
		struct device_node *np = p->dev->of_node;
		int id;

		/* get index of serial line, if found in DT aliases */
		id = of_alias_get_id(np, "serial");
		if (id >= 0)
			p->line = id;
#ifdef CONFIG_64BIT
		if (of_device_is_compatible(np, "cavium,octeon-3860-uart")) {
			p->serial_in = dw8250_serial_inq;
			p->serial_out = dw8250_serial_outq;
			p->flags = UPF_SKIP_TEST | UPF_SHARE_IRQ | UPF_FIXED_TYPE;
			p->type = PORT_OCTEON;
			data->usr_reg = 0x27;
			data->skip_autocfg = true;
		}
#endif
		if (of_device_is_big_endian(p->dev->of_node)) {
			p->iotype = UPIO_MEM32BE;
			p->serial_in = dw8250_serial_in32be;
			p->serial_out = dw8250_serial_out32be;
		}
	} else if (has_acpi_companion(p->dev)) {
		const struct acpi_device_id *id;

		id = acpi_match_device(p->dev->driver->acpi_match_table,
				       p->dev);
		if (id && !strcmp(id->id, "APMC0D08")) {
			p->iotype = UPIO_MEM32;
			p->regshift = 2;
			p->serial_in = dw8250_serial_in32;
			data->uart_16550_compatible = true;
		}
		p->set_termios = dw8250_set_termios;
	}

	/* Platforms with iDMA */
	if (platform_get_resource_byname(to_platform_device(p->dev),
					 IORESOURCE_MEM, "lpss_priv")) {
		p->set_termios = dw8250_set_termios;
		data->dma.rx_param = p->dev->parent;
		data->dma.tx_param = p->dev->parent;
		data->dma.fn = dw8250_idma_filter;
	}
}

static void dw8250_setup_port(struct uart_port *p, struct dw8250_data *data)
{
	struct uart_8250_port *up = up_to_u8250p(p);
	u32 reg;

	/*
	 * If the Component Version Register returns zero, we know that
	 * ADDITIONAL_FEATURES are not enabled. No need to go any further.
	 */
	if (p->iotype == UPIO_MEM32BE)
		reg = ioread32be(p->membase + DW_UART_UCV);
	else
		reg = readl(p->membase + DW_UART_UCV);
	if (!reg)
		return;

	dev_dbg(p->dev, "Designware UART version %c.%c%c\n",
		(reg >> 24) & 0xff, (reg >> 16) & 0xff, (reg >> 8) & 0xff);

	if (p->iotype == UPIO_MEM32BE)
		reg = ioread32be(p->membase + DW_UART_CPR);
	else
		reg = readl(p->membase + DW_UART_CPR);
	/* If the optional CPR register is not there, use value from DT */
	if (!reg)
		reg = data->cpr_val;

	/* Select the type based on fifo */
	if (reg & DW_UART_CPR_FIFO_MODE)
		p->fifosize = DW_UART_CPR_FIFO_SIZE(reg);

	if (p->fifosize) {
		p->type = PORT_16550A;
		p->flags |= UPF_FIXED_TYPE;
		up->capabilities = UART_CAP_FIFO;
	}

	if (reg & DW_UART_CPR_AFCE_MODE)
		up->capabilities |= UART_CAP_AFE;

	if (reg & DW_UART_CPR_DMA_EXTRA)
		data->dma_capable = 1;
}

static int dw8250_probe(struct platform_device *pdev)
{
	struct uart_8250_port uart = {};
	struct resource *regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	int irq = platform_get_irq(pdev, 0);
	struct uart_port *p = &uart.port;
	struct device *dev = &pdev->dev;
	struct dw8250_data *data;
	int err;
	u32 val;

	if (!regs) {
		dev_err(dev, "no registers defined\n");
		return -EINVAL;
	}

	if (irq < 0) {
		if (irq != -EPROBE_DEFER)
			dev_err(dev, "cannot get irq\n");
		return irq;
	}

	spin_lock_init(&p->lock);
	p->mapbase	= regs->start;
	p->irq		= irq;
	p->handle_irq	= dw8250_handle_irq;
	p->pm		= dw8250_do_pm;
	p->type		= PORT_8250;
	p->flags	= UPF_SHARE_IRQ | UPF_FIXED_PORT;
	p->dev		= dev;
	p->iotype	= UPIO_MEM;
	p->serial_in	= dw8250_serial_in;
	p->serial_out	= dw8250_serial_out;

	p->membase = devm_ioremap(dev, regs->start, resource_size(regs));
	if (!p->membase)
		return -ENOMEM;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dma.fn = dw8250_fallback_dma_filter;
	data->usr_reg = DW_UART_USR;
	p->private_data = data;

	data->uart_16550_compatible = device_property_read_bool(dev,
						"snps,uart-16550-compatible");

	err = device_property_read_u32(dev, "reg-shift", &val);
	if (!err)
		p->regshift = val;

	err = device_property_read_u32(dev, "reg-io-width", &val);
	if (!err && val == 4) {
		p->iotype = UPIO_MEM32;
		p->serial_in = dw8250_serial_in32;
		p->serial_out = dw8250_serial_out32;
	}

	if (device_property_read_bool(dev, "dcd-override")) {
		/* Always report DCD as active */
		data->msr_mask_on |= UART_MSR_DCD;
		data->msr_mask_off |= UART_MSR_DDCD;
	}

	if (device_property_read_bool(dev, "dsr-override")) {
		/* Always report DSR as active */
		data->msr_mask_on |= UART_MSR_DSR;
		data->msr_mask_off |= UART_MSR_DDSR;
	}

	if (device_property_read_bool(dev, "cts-override")) {
		/* Always report CTS as active */
		data->msr_mask_on |= UART_MSR_CTS;
		data->msr_mask_off |= UART_MSR_DCTS;
	}

	if (device_property_read_bool(dev, "ri-override")) {
		/* Always report Ring indicator as inactive */
		data->msr_mask_off |= UART_MSR_RI;
		data->msr_mask_off |= UART_MSR_TERI;
	}

	/* Optional CPR value in case it was omitted from the IP */
	err = device_property_read_u32(dev, "cpr-value", &val);
	if (!err)
		data->cpr_val = val;

	data->is_rzn1 = device_property_read_bool(dev, "snps,rzn1-uart");

	/* Always ask for fixed clock rate from a property. */
	device_property_read_u32(dev, "clock-frequency", &p->uartclk);

	/* If there is separate baudclk, get the rate from it. */
	data->clk = devm_clk_get(dev, "baudclk");
	if (IS_ERR(data->clk) && PTR_ERR(data->clk) != -EPROBE_DEFER)
		data->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(data->clk) && PTR_ERR(data->clk) == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	if (!IS_ERR_OR_NULL(data->clk)) {
		err = clk_prepare_enable(data->clk);
		if (err)
			dev_warn(dev, "could not enable optional baudclk: %d\n",
				 err);
		else
			p->uartclk = clk_get_rate(data->clk);
	}

	/* If no clock rate is defined, fail. */
	if (!p->uartclk) {
		dev_err(dev, "clock rate not defined\n");
		return -EINVAL;
	}

	data->pclk = devm_clk_get(dev, "apb_pclk");
	if (IS_ERR(data->pclk) && PTR_ERR(data->pclk) == -EPROBE_DEFER) {
		err = -EPROBE_DEFER;
		goto err_clk;
	}
	if (!IS_ERR(data->pclk)) {
		err = clk_prepare_enable(data->pclk);
		if (err) {
			dev_err(dev, "could not enable apb_pclk\n");
			goto err_clk;
		}
	}

	data->rst = devm_reset_control_get_optional(dev, NULL);
	if (IS_ERR(data->rst) && PTR_ERR(data->rst) == -EPROBE_DEFER) {
		err = -EPROBE_DEFER;
		goto err_pclk;
	}
	if (!IS_ERR(data->rst))
		reset_control_deassert(data->rst);

	dw8250_quirks(p, data);

	/* If the Busy Functionality is not implemented, don't handle it */
	if (data->uart_16550_compatible)
		p->handle_irq = NULL;

	if (!data->skip_autocfg)
		dw8250_setup_port(p, data);

#ifdef CONFIG_PM
	uart.capabilities |= UART_CAP_RPM;
#endif

	/* If we have a valid fifosize, and DMA signals, try hooking up DMA */
	if (p->fifosize && data->dma_capable) {
		data->dma.rxconf.src_maxburst = p->fifosize / 4;
		data->dma.txconf.dst_maxburst = p->fifosize / 4;
		uart.dma = &data->dma;
	}

	if (data->dma_capable && data->is_rzn1) {
		/*
		 * When the 'char timeout' irq fires because no more data has
		 * been received in some time, the 8250 driver stops the DMA.
		 * However, if the DMAC has been setup to write more data to mem
		 * than is read from the UART FIFO, the data will *not* be
		 * written to memory.
		 * Therefore, we limit the width of writes to mem so that it is
		 * the same amount of data as read from the FIFO. You can use
		 * anything less than or equal, but same size is optimal
		 */
		data->dma.rxconf.dst_addr_width = p->fifosize / 4;

		/*
		 * Unless you set the maxburst to 1, if you send only 1 char, it
		 * doesn't get transmitted
		 */
		data->dma.txconf.dst_maxburst = 1;

		data->dma.txconf.device_fc = 1;
		data->dma.rxconf.device_fc = 1;

		uart.dma->tx_dma = rzn1_dw8250_tx_dma;
		uart.dma->rx_dma = rzn1_dw8250_rx_dma;
	}

	data->line = serial8250_register_8250_port(&uart);
	if (data->line < 0) {
		err = data->line;
		goto err_reset;
	}

	platform_set_drvdata(pdev, data);

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	return 0;

err_reset:
	if (!IS_ERR(data->rst))
		reset_control_assert(data->rst);

err_pclk:
	if (!IS_ERR(data->pclk))
		clk_disable_unprepare(data->pclk);

err_clk:
	if (!IS_ERR(data->clk))
		clk_disable_unprepare(data->clk);

	return err;
}

static int dw8250_remove(struct platform_device *pdev)
{
	struct dw8250_data *data = platform_get_drvdata(pdev);

	pm_runtime_get_sync(&pdev->dev);

	serial8250_unregister_port(data->line);

	if (!IS_ERR(data->rst))
		reset_control_assert(data->rst);

	if (!IS_ERR(data->pclk))
		clk_disable_unprepare(data->pclk);

	if (!IS_ERR(data->clk))
		clk_disable_unprepare(data->clk);

	pm_runtime_disable(&pdev->dev);
	pm_runtime_put_noidle(&pdev->dev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int dw8250_suspend(struct device *dev)
{
	struct dw8250_data *data = dev_get_drvdata(dev);

	serial8250_suspend_port(data->line);

	return 0;
}

static int dw8250_resume(struct device *dev)
{
	struct dw8250_data *data = dev_get_drvdata(dev);

	serial8250_resume_port(data->line);

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

#ifdef CONFIG_PM
static int dw8250_runtime_suspend(struct device *dev)
{
	struct dw8250_data *data = dev_get_drvdata(dev);

	if (!IS_ERR(data->clk))
		clk_disable_unprepare(data->clk);

	if (!IS_ERR(data->pclk))
		clk_disable_unprepare(data->pclk);

	return 0;
}

static int dw8250_runtime_resume(struct device *dev)
{
	struct dw8250_data *data = dev_get_drvdata(dev);

	if (!IS_ERR(data->pclk))
		clk_prepare_enable(data->pclk);

	if (!IS_ERR(data->clk))
		clk_prepare_enable(data->clk);

	return 0;
}
#endif

static const struct dev_pm_ops dw8250_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(dw8250_suspend, dw8250_resume)
	SET_RUNTIME_PM_OPS(dw8250_runtime_suspend, dw8250_runtime_resume, NULL)
};

static const struct of_device_id dw8250_of_match[] = {
	{ .compatible = "snps,dw-apb-uart" },
	{ .compatible = "cavium,octeon-3860-uart" },
	{ /* Sentinel */ }
};
MODULE_DEVICE_TABLE(of, dw8250_of_match);

static const struct acpi_device_id dw8250_acpi_match[] = {
	{ "INT33C4", 0 },
	{ "INT33C5", 0 },
	{ "INT3434", 0 },
	{ "INT3435", 0 },
	{ "80860F0A", 0 },
	{ "8086228A", 0 },
	{ "APMC0D08", 0},
	{ "AMD0020", 0 },
	{ "AMDI0020", 0 },
	{ "HISI0031", 0 },
	{ },
};
MODULE_DEVICE_TABLE(acpi, dw8250_acpi_match);

static struct platform_driver dw8250_platform_driver = {
	.driver = {
		.name		= "dw-apb-uart",
		.pm		= &dw8250_pm_ops,
		.of_match_table	= dw8250_of_match,
		.acpi_match_table = ACPI_PTR(dw8250_acpi_match),
	},
	.probe			= dw8250_probe,
	.remove			= dw8250_remove,
};

module_platform_driver(dw8250_platform_driver);

MODULE_AUTHOR("Jamie Iles");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Synopsys DesignWare 8250 serial port driver");
MODULE_ALIAS("platform:dw-apb-uart");
