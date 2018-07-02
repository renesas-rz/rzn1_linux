/*
 * (C) Copyright 2015 Renesas Electronics Europe Ltd.
 *
 * Using some material derived from:
 * (C) 2014 Altera Corporation <www.altera.com>
 * (C) 2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/completion.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/spi-nor.h>

#include "rzn1-qspi.h"

#define CQSPI_CAL_DELAY(tdelay_ns, tref_ns, tsclk_ns)	\
	(((tdelay_ns) > (tsclk_ns)) ? \
		((((tdelay_ns) - (tsclk_ns)) / (tref_ns))) : 0)

/* these are default */
#define CONFIG_CQSPI_TSHSL_NS	200
#define CONFIG_CQSPI_TSD2D_NS	255
#define CONFIG_CQSPI_TCHSH_NS	20
#define CONFIG_CQSPI_TSLCH_NS	20


struct rzn1_qspi_slave {
	u32	cs;
	u32	clk_rate;	/* target rate */
	unsigned int pha: 1, pol: 1,
		read_cmd: 8, dummy_clk: 4,
		mux_address: 1, cap_delay: 4;

	/* these are cached register values for this slave, contains
	 * clock speed, mode, CS etc */
	struct {
		u32 cfg;
		u32 delay;
		u32 devsz;
		u32 rddatacap;
	} reg;
	/* corresponds to the cfg register value, clock polarity/rate etc */
	u32	cfg_base;
	struct rzn1_qspi *host;
	struct mtd_info *mtd;
	struct spi_nor nor;
};

struct rzn1_qspi {
	/* keep it const prevents stupidly writing to reg */
	const struct cadence_qspi __iomem *reg;
	u32 slave_count;
	struct rzn1_qspi_slave slave[CONFIG_RZN1_QSPI_NR_CS];
	void __iomem *ahb_base; /* Used when read from AHB bus */
	phys_addr_t ahb_phys;
	struct device *dev;
	int use_cs_mux;		/* renesas,rzn1-cs_mux */
	struct clk *clk, *clk_fw;
	int disable_count;
};

/* made to bypass the const used in the rzn1_qspi->reg structure, and
 * therefore make sure a writel is done instead of a direct assign */
#define qspi_writel(_v, _a) writel((_v), (void *)(_a))

/* Do not re-enable controller while multiple function might have disabled it */
static void _rzn1_qspi_enable(struct rzn1_qspi *q)
{
	if (q->disable_count)
		q->disable_count--;
	if (q->disable_count)
		return;
	qspi_writel(readl(&q->reg->cfg) | CQSPI_REG_CONFIG_ENABLE_MASK,
		&q->reg->cfg);
}

static void _rzn1_qspi_disable(struct rzn1_qspi *q)
{
	q->disable_count++;
	qspi_writel(readl(&q->reg->cfg) & ~CQSPI_REG_CONFIG_ENABLE_MASK,
		&q->reg->cfg);
}

/* Return 1 if idle, otherwise return 0 (busy). */
static u32 _rzn1_qspi_wait_idle(struct rzn1_qspi *q)
{
	u32 count = 0;
	u32 timeout = 50;

	while (count < timeout) {
		if ((readl(&q->reg->cfg) >> CQSPI_REG_CONFIG_IDLE_LSB) & 1) {
			count++;
			schedule_timeout_interruptible(HZ / 10000);
		} else {
			count = 0;
		}
		/*
		 * Ensure the QSPI controller is in true idle state after
		 * reading back the same idle status consecutively
		 */
		if (count >= CQSPI_POLL_IDLE_RETRY)
			return 1;
	}

	/* Timeout, still in busy mode. */
	dev_warn(q->dev, "still busy after poll for %d times.\n",
	       CQSPI_REG_RETRY);
	return 0;
}

static u32 _rzn1_qspi_set_baudrate_div(
	struct rzn1_qspi_slave *s,
	u32 ref_clk_hz,
	u32 sclk_hz)
{
	u32 reg;
	u32 div;
	u32 actual_div;

	_rzn1_qspi_disable(s->host);
	reg = readl(&s->host->reg->cfg);
	reg &= ~(CQSPI_REG_CONFIG_BAUD_MASK << CQSPI_REG_CONFIG_BAUD_LSB);

	/* Recalculate the baudrate divisor based on QSPI specification. */
	div = DIV_ROUND_UP(ref_clk_hz, 2 * sclk_hz) - 1;
	if (div > 15)
		div = 15;

	actual_div = (div + 1) << 1;

	dev_dbg(s->host->dev, "%s: ref_clk %dHz sclk %dHz Div 0x%x\n", __func__,
	      ref_clk_hz, sclk_hz, div);
	dev_info(s->host->dev, "%s: output clock is %dHz (div = %d)\n", __func__,
	      ref_clk_hz / actual_div, actual_div);

	div = (div & CQSPI_REG_CONFIG_BAUD_MASK) << CQSPI_REG_CONFIG_BAUD_LSB;
	reg |= div;
	qspi_writel(reg, &s->host->reg->cfg);

	_rzn1_qspi_enable(s->host);

	return ref_clk_hz / actual_div;
}

static void _rzn1_qspi_set_clk_mode(
	struct rzn1_qspi_slave *s)
{
	u32 reg;

	_rzn1_qspi_disable(s->host);
	reg = readl(&s->host->reg->cfg);
	reg &= ~((1 << CQSPI_REG_CONFIG_CLK_POL_LSB) |
		(1 << CQSPI_REG_CONFIG_CLK_PHA_LSB));

	reg |= s->pol << CQSPI_REG_CONFIG_CLK_POL_LSB;
	reg |= s->pha << CQSPI_REG_CONFIG_CLK_PHA_LSB;

	qspi_writel(reg, &s->host->reg->cfg);

	_rzn1_qspi_enable(s->host);
}

static void _rzn1_qspi_set_cs(
	struct rzn1_qspi_slave *s)
{
	struct rzn1_qspi *q = s->host;
	u32 reg;
	int chip_select = s->cs;

	_rzn1_qspi_disable(s->host);

	dev_dbg(s->host->dev, "set CS:%d decode %d\n",
		chip_select, q->use_cs_mux);

	reg = readl(&s->host->reg->cfg);
	/* decoder */
	if (q->use_cs_mux) {
		reg |= CQSPI_REG_CONFIG_DECODE_MASK;
	} else {
		reg &= ~CQSPI_REG_CONFIG_DECODE_MASK;
		/* Convert CS if without decoder.
		 * CS0 to 4b'1110
		 * CS1 to 4b'1101
		 * CS2 to 4b'1011
		 * CS3 to 4b'0111
		 */
		chip_select = 0xF & ~(1 << chip_select);
	}

	reg &= ~(CQSPI_REG_CONFIG_CHIPSELECT_MASK
			<< CQSPI_REG_CONFIG_CHIPSELECT_LSB);
	reg |= (chip_select & CQSPI_REG_CONFIG_CHIPSELECT_MASK)
			<< CQSPI_REG_CONFIG_CHIPSELECT_LSB;
	qspi_writel(reg, &s->host->reg->cfg);

	_rzn1_qspi_enable(s->host);
}

static void _rzn1_qspi_set_device_size(
	struct rzn1_qspi_slave *s)
{
	struct spi_nor *nor = &(s->nor);
	u32 reg;

	_rzn1_qspi_disable(s->host);

	/* Configure the device size and address bytes */
	reg = readl(&s->host->reg->devsz);
	/* Clear the previous value */
	reg &= ~(CQSPI_REG_SIZE_PAGE_MASK << CQSPI_REG_SIZE_PAGE_LSB);
	reg &= ~(CQSPI_REG_SIZE_BLOCK_MASK << CQSPI_REG_SIZE_BLOCK_LSB);
	reg |= (nor->page_size << CQSPI_REG_SIZE_PAGE_LSB);
	reg |= (ilog2(nor->mtd.erasesize) << CQSPI_REG_SIZE_BLOCK_LSB);
	reg |= (nor->addr_width - 1);
	qspi_writel(reg, &s->host->reg->devsz);

	_rzn1_qspi_enable(s->host);
}

static void _rzn1_qspi_set_readdelay(
	struct rzn1_qspi_slave *s,
	unsigned int delay)
{
	unsigned int reg;

	_rzn1_qspi_disable(s->host);

	reg = readl(&s->host->reg->rddatacap);

	reg |= (1 << CQSPI_REG_RD_DATA_CAPTURE_BYPASS_LSB) |
		(1 << CQSPI_REG_RD_DATA_CAPTURE_EDGE_LSB);

	reg &= ~(CQSPI_REG_RD_DATA_CAPTURE_DELAY_MASK
		<< CQSPI_REG_RD_DATA_CAPTURE_DELAY_LSB);

	reg |= ((delay & CQSPI_REG_RD_DATA_CAPTURE_DELAY_MASK)
		<< CQSPI_REG_RD_DATA_CAPTURE_DELAY_LSB);

	qspi_writel(reg, &s->host->reg->rddatacap);

	_rzn1_qspi_enable(s->host);
}

static void _rzn1_qspi_set_delay(
	struct rzn1_qspi_slave *s,
	u32 ref_clk, u32 sclk_hz,
	u32 tshsl_ns, u32 tsd2d_ns,
	u32 tchsh_ns, u32 tslch_ns)
{
	u32 ref_clk_ns;
	u32 sclk_ns;
	u32 tshsl, tchsh, tslch, tsd2d;
	u32 reg;

	_rzn1_qspi_disable(s->host);

	/* Convert to ns. */
	ref_clk_ns = 1000000000 / ref_clk;

	/* Convert to ns. */
	sclk_ns = 1000000000 / sclk_hz;

	tshsl = CQSPI_CAL_DELAY(tshsl_ns, ref_clk_ns, sclk_ns);
	tchsh = CQSPI_CAL_DELAY(tchsh_ns, ref_clk_ns, sclk_ns);
	tslch = CQSPI_CAL_DELAY(tslch_ns, ref_clk_ns, sclk_ns);
	tsd2d = CQSPI_CAL_DELAY(tsd2d_ns, ref_clk_ns, sclk_ns);

	reg = (tshsl & CQSPI_REG_DELAY_TSHSL_MASK)
			<< CQSPI_REG_DELAY_TSHSL_LSB;
	reg |= (tchsh & CQSPI_REG_DELAY_TCHSH_MASK)
			<< CQSPI_REG_DELAY_TCHSH_LSB;
	reg |= (tslch & CQSPI_REG_DELAY_TSLCH_MASK)
			<< CQSPI_REG_DELAY_TSLCH_LSB;
	reg |= (tsd2d & CQSPI_REG_DELAY_TSD2D_MASK)
			<< CQSPI_REG_DELAY_TSD2D_LSB;
	dev_dbg(s->host->dev, "%s: delay reg = 0x%X\n", __func__, reg);
	qspi_writel(reg, &s->host->reg->delay);

	_rzn1_qspi_enable(s->host);
}

static void rzn1_qspi_set_speed(struct rzn1_qspi_slave *s)
{
	unsigned long rate = clk_get_rate(s->host->clk);
	unsigned long hz = _rzn1_qspi_set_baudrate_div(
					s, rate, s->clk_rate);

	/* Reconfigure delay timing if speed is changed. */
	_rzn1_qspi_set_delay(s,
		rate, hz,
		CONFIG_CQSPI_TSHSL_NS, CONFIG_CQSPI_TSD2D_NS,
		CONFIG_CQSPI_TCHSH_NS, CONFIG_CQSPI_TSLCH_NS);
}

static u32
_rzn1_qspi_set_address(
	struct rzn1_qspi_slave *s,
	u32 addr,
	u32 reg)
{
	/* Command with address */
	reg |= 1 << CQSPI_REG_CMDCTRL_ADDR_EN_LSB;
	/* Number of bytes to write. */
	reg |= ((s->nor.addr_width - 1) & CQSPI_REG_CMDCTRL_ADD_BYTES_MASK)
		<< CQSPI_REG_CMDCTRL_ADD_BYTES_LSB;

	qspi_writel(addr, &s->host->reg->flashcmdaddr);

	return reg;
}

static int _rzn1_qspi_exec_cmd(
	struct rzn1_qspi_slave *s,
	u32 reg)
{
	u32 retry = CQSPI_REG_RETRY;

	/* Write the CMDCTRL without start execution. */
	qspi_writel(reg, &s->host->reg->flashcmd);
	/* Start execute */
	reg |= CQSPI_REG_CMDCTRL_EXECUTE_MASK;
	qspi_writel(reg, &s->host->reg->flashcmd);

	while (retry--) {
		reg = readl(&s->host->reg->flashcmd);
		if ((reg & CQSPI_REG_CMDCTRL_INPROGRESS_MASK) == 0)
			break;
		schedule_timeout_interruptible(HZ / 10000);
	}

	if (!retry) {
		dev_err(s->host->dev, "flash command execution timeout\n");
		return -EIO;
	}

	/* Polling QSPI idle status. */
	if (!_rzn1_qspi_wait_idle(s->host))
		return -EIO;

	return 0;
}

/* We use this function to do some basic init for spi_nor_scan(). */
static int rzn1_qspi_nor_setup(struct rzn1_qspi *q)
{
	u32 reg = 0;

	_rzn1_qspi_disable(q);

	/* Configure the remap address register, no remap */
	qspi_writel(0, &q->reg->remapaddr);

	/* Enable Protected Area Write Attempt interrupts */
	qspi_writel(CQSPI_REG_IRQMASK_WP, &q->reg->irqmask);

	/* this is the defaul SPI mode for most SPI NOR flashes */
	reg |= (1 << CQSPI_REG_CONFIG_CLK_POL_LSB);
	reg |= (1 << CQSPI_REG_CONFIG_CLK_PHA_LSB);
	/* This is the only setup we need; enable memory mapped range */
	reg |= CQSPI_REG_CONFIG_DIRECT_MASK;
	/* Enable AHB write protection to prevent random write access
	 * that could nuke the flash memory. Instead you have to explicitly
	 * use the write command.
	 */
	qspi_writel(0, &q->reg->lowwrprot);
	qspi_writel(0, &q->reg->uppwrprot);
	qspi_writel(CQSPI_REG_WRPROT_ENABLE_MASK | CQSPI_REG_WRPROT_INVERT,
			&q->reg->wrprot);

	qspi_writel(reg, &q->reg->cfg);

	_rzn1_qspi_enable(q);
	return 0;
}

static int rzn1_qspi_read_reg(
	struct spi_nor *nor, u8 opcode, u8 *buf, int len)
{
	int ret;
	struct rzn1_qspi_slave *s = nor->priv;
	struct rzn1_qspi *q = s->host;
	u32 reg;
	uint32_t rx[2];

	if (len > CQSPI_STIG_DATA_LEN_MAX || buf == NULL) {
		dev_err(q->dev, "%s CS:%d 0x%2x Invalid size len:%d\n",
			__func__, s->cs, opcode, len);
		return -EINVAL;
	}

	reg = opcode << CQSPI_REG_CMDCTRL_OPCODE_LSB;
	reg |= 1 << CQSPI_REG_CMDCTRL_RD_EN_LSB;

	/* 0 means 1 byte. */
	reg |= ((len - 1) & CQSPI_REG_CMDCTRL_RD_BYTES_MASK)
		<< CQSPI_REG_CMDCTRL_RD_BYTES_LSB;
	ret = _rzn1_qspi_exec_cmd(s, reg);
	if (ret != 0)
		return ret;
	/* copy the IO registers into a non io buffer, then into the dest one */
	rx[0] = readl(&q->reg->flashcmdrddatalo);
	if (len > 4)
		rx[1] = readl(&q->reg->flashcmdrddataup);
	memcpy(buf, rx, len);

	return 0;
}

static int rzn1_qspi_write_reg(struct spi_nor *nor, u8 opcode, u8 *buf, int len)
{
	struct rzn1_qspi_slave *s = nor->priv;
	struct rzn1_qspi *q = s->host;
	int ret;
	u32 reg;

	if (len > CQSPI_STIG_DATA_LEN_MAX) {
		dev_err(q->dev, "%s CS:%d 0x%2x Invalid size len:%d\n",
			__func__, s->cs, opcode, len);
		return -EINVAL;
	}

	reg = opcode << CQSPI_REG_CMDCTRL_OPCODE_LSB;
	if (buf && len) {
		const uint32_t *src = (uint32_t *)buf;
		const uint32_t *d = &q->reg->flashcmdwrdatalo;
		int i;

		for (i = 0; i < 1 + (len / 4); i++, src++, d++)
			qspi_writel(*src, d);
		reg |= 1 << CQSPI_REG_CMDCTRL_WR_EN_LSB;
		/* Number of bytes to write. */
		reg |= ((len - 1) & CQSPI_REG_CMDCTRL_WR_BYTES_MASK)
			<< CQSPI_REG_CMDCTRL_WR_BYTES_LSB;
	}

	ret = _rzn1_qspi_exec_cmd(s, reg);

	return ret;
}

static ssize_t rzn1_qspi_write(struct spi_nor *nor, loff_t to,
		size_t len, const u_char *buf)
{
	struct rzn1_qspi_slave *s = nor->priv;
	struct rzn1_qspi *q = s->host;
	u32 low = ((u32)q->ahb_phys + (u32)to) / nor->mtd.erasesize;
	u32 upp = low + len / nor->mtd.erasesize;

	qspi_writel(low, &q->reg->lowwrprot);
	qspi_writel(upp, &q->reg->uppwrprot);

	memcpy(q->ahb_base + to, buf, len);

	qspi_writel(0, &q->reg->uppwrprot);
	qspi_writel(0, &q->reg->lowwrprot);

	return len;
}

static ssize_t rzn1_qspi_read(struct spi_nor *nor, loff_t from,
		size_t len, u_char *buf)
{
	struct rzn1_qspi_slave *s = nor->priv;
	struct rzn1_qspi *q = s->host;

	/* Read out the data directly from the AHB buffer.*/
	memcpy(buf, q->ahb_base + from, len);

	return len;
}

static int rzn1_qspi_erase(struct spi_nor *nor, loff_t offs)
{
	struct rzn1_qspi_slave *s = nor->priv;
	struct rzn1_qspi *q = s->host;
	int ret;
	u32 reg;

	dev_dbg(nor->dev, "Erase %dKiB at 0x%08x [o:%02x a:%d]\n",
		nor->mtd.erasesize / 1024, (u32)offs,
		s->nor.erase_opcode, s->nor.addr_width);

	/* Send write enable, then erase commands. */
	ret = nor->write_reg(nor, SPINOR_OP_WREN, NULL, 0);
	if (ret)
		return ret;

	reg = s->nor.erase_opcode << CQSPI_REG_CMDCTRL_OPCODE_LSB;
	reg = _rzn1_qspi_set_address(s, offs, reg);
	/*
	 * Here we do not use the synchronous function, as the
	 * erase is so slow we want to let the mtd pool the status until
	 * it's done instead of locking the driver in that opperation
	 */
	/* Write the CMDCTRL without start execution. */
	qspi_writel(reg, &q->reg->flashcmd);
	/* Start execute */
	reg |= CQSPI_REG_CMDCTRL_EXECUTE_MASK;
	qspi_writel(reg, &q->reg->flashcmd);

	return ret;
}

static int rzn1_qspi_prep(struct spi_nor *nor, enum spi_nor_ops ops)
{
	struct rzn1_qspi_slave *s = nor->priv;
	struct rzn1_qspi *q = s->host;
	u32 rd_reg = s->read_cmd << CQSPI_REG_RD_INSTR_OPCODE_LSB;
	u32 wr_reg = nor->program_opcode << CQSPI_REG_WR_INSTR_OPCODE_LSB;

	_rzn1_qspi_disable(q);

	switch (nor->flash_read) {
	case SPI_NOR_DUAL:
		rd_reg |= (CQSPI_INST_TYPE_DUAL <<
				CQSPI_REG_RD_INSTR_TYPE_DATA_LSB);
		if (s->mux_address)
			rd_reg |= (CQSPI_INST_TYPE_DUAL <<
				CQSPI_REG_RD_INSTR_TYPE_ADDR_LSB);
		break;
	case SPI_NOR_QUAD:
		rd_reg |= (CQSPI_INST_TYPE_QUAD <<
				CQSPI_REG_RD_INSTR_TYPE_DATA_LSB);
		if (s->mux_address)
			rd_reg |= (CQSPI_INST_TYPE_QUAD <<
				CQSPI_REG_RD_INSTR_TYPE_ADDR_LSB);
		break;
	default: /* nothing to do for the other modes */
		break;
	}

	if (s->dummy_clk) {
		rd_reg |= (s->dummy_clk & CQSPI_REG_RD_INSTR_DUMMY_MASK) <<
				CQSPI_REG_RD_INSTR_DUMMY_LSB;
	}
	qspi_writel(rd_reg, &q->reg->devrd);
	qspi_writel(wr_reg, &q->reg->devwr);

	qspi_writel(s->reg.devsz, &q->reg->devsz);
	qspi_writel(s->reg.delay, &q->reg->delay);
	qspi_writel(s->reg.rddatacap, &q->reg->rddatacap);
	qspi_writel(0xff, &q->reg->modebit);

	/* this also technically enable the controller, but doesn't matter */
	qspi_writel(s->reg.cfg, &q->reg->cfg);

	_rzn1_qspi_enable(q);

	return 0;
}

static void rzn1_qspi_unprep(struct spi_nor *nor, enum spi_nor_ops ops)
{

}

static irqreturn_t rzn1_qspi_irq_handler(int irq, void *dev_id)
{
	struct rzn1_qspi *q = dev_id;
	u32 reg = readl(&q->reg->irqstat);

	dev_warn(q->dev, "Detected invalid attempt to write to QSPI!\n");

	reg = 0;
	qspi_writel(reg, &q->reg->irqstat);

	return IRQ_HANDLED;
}

static const struct spi_nor nor_callbacks = {
	.read_reg = rzn1_qspi_read_reg,
	.write_reg = rzn1_qspi_write_reg,
	.read = rzn1_qspi_read,
	.write = rzn1_qspi_write,
	.write_mmap = rzn1_qspi_write,
	.erase = rzn1_qspi_erase,
	.prepare = rzn1_qspi_prep,
	.unprepare = rzn1_qspi_unprep,
};

/*
 * Handles properties that are listed either as a bool, or as an integer. Like
 *
 *  {
 *     renesas,rzn1-phase;
 *  .. or
 *     renesas,rzn1-phase = <0>;
 *  }
 */
static int _of_property_bool_or_int(
		struct device_node *np,
		const char * name,
		int default_value)
{
	struct property *prop;
	int res = default_value;

	prop = of_find_property(np, name, NULL);
	if (prop && prop->length) {
		u32 reg = default_value;

		of_property_read_u32(np, name, &reg);
		res = !!reg;
	}
	return res;
}

static int rzn1_get_slave_config(
		struct device_node *np,
		struct rzn1_qspi_slave *s)
{
	u32 reg = 0;
	unsigned long clk = clk_get_rate(s->host->clk);
	int mode = SPI_NOR_NORMAL;
	u32 value;
	const __be32 *read_mode;
	int rm_len;

	s->pha = s->pol = 0;

	of_property_read_u32_index(np, "reg", 0, &s->cs);

	read_mode = of_get_property(np, "renesas,rzn1-read-cmd", &rm_len);
	if (read_mode) {
		rm_len /= sizeof(read_mode[0]);
		if (rm_len > 0)
			s->read_cmd = be32_to_cpu(read_mode[0]);
		if (rm_len > 1)
			s->dummy_clk = be32_to_cpu(read_mode[1]);
		if (rm_len > 2)
			s->mux_address = read_mode[2] != 0;
	}

	if (of_find_property(np, "spi-cpha", NULL))
		s->pha = 1;
	if (of_find_property(np, "spi-cpol", NULL))
		s->pol = 1;

	if (!of_property_read_u32(np, "spi-rx-bus-width", &value)) {
		switch (value) {
		case 1:
			break;
		case 2:
			mode |= SPI_NOR_DUAL;
			break;
		case 4:
			mode |= SPI_NOR_QUAD;
			break;
		default:
			dev_warn(s->host->dev,
				"spi-rx-bus-width %d not supported\n",
				value);
			break;
		}
	}

	of_property_read_u32(np, "spi-max-frequency", &s->clk_rate);
	if (s->clk_rate > (clk / 2))
		s->clk_rate = clk / 2;
	if (s->clk_rate < (clk/32)) {
		dev_warn(s->host->dev,
			 "%s CS:%d Invalid rate %d, should be %ul<>%d\n",
			__func__, s->cs, s->clk_rate,
			(unsigned)(clk/32), (unsigned)(clk/2));
	}
	reg = 2;	/* default delay for full reads */
	of_property_read_u32(np, "renesas,rzn1-readcap-delay", &reg);
	s->cap_delay = reg;

	return mode;
}

static int rzn1_qspi_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct rzn1_qspi *q;
	struct resource *res;
	int ret, i = 0;

	q = devm_kzalloc(dev, sizeof(*q), GFP_KERNEL);
	if (!q)
		return -ENOMEM;

	q->slave_count = of_get_child_count(dev->of_node);
	if (!q->slave_count || q->slave_count > CONFIG_RZN1_QSPI_NR_CS) {
		dev_err(dev, "Invalid number of flash slave chips\n");
		return -ENODEV;
	}

	/* find the resources */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "qspi");
	q->reg = devm_ioremap_resource(dev, res);
	if (IS_ERR(q->reg)) {
		ret = PTR_ERR(q->reg);
		goto map_failed;
	}
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					"qspi-mapping");
	/* Make sure the IO mapping range is cacheable */
	if (res) {
		res->flags |= IORESOURCE_CACHEABLE;
		q->ahb_phys = (phys_addr_t)res->start;
	}
	q->ahb_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(q->ahb_base)) {
		ret = PTR_ERR(q->ahb_base);
		goto map_failed;
	}

	/* find the clocks */
	q->clk_fw = devm_clk_get(dev, "flexway");
	if (IS_ERR(q->clk_fw)) {
		ret = PTR_ERR(q->clk_fw);
		goto clk_failed;
	}
	q->clk = devm_clk_get(dev, "pclk");
	if (IS_ERR(q->clk)) {
		ret = PTR_ERR(q->clk);
		goto clk_failed;
	}
	ret = clk_prepare_enable(q->clk_fw);
	if (ret) {
		dev_err(dev, "can not enable the qspi_fw clock\n");
		goto clk_failed;
	}
	ret = clk_prepare_enable(q->clk);
	if (ret) {
		clk_disable_unprepare(q->clk_fw);
		dev_err(dev, "can not enable the qspi clock\n");
		goto clk_failed;
	}

	/* find the irq */
	ret = platform_get_irq(pdev, 0);
	if (ret < 0) {
		dev_err(dev, "failed to get the irq\n");
		goto irq_failed;
	}

	ret = devm_request_irq(dev, ret,
			rzn1_qspi_irq_handler, 0, pdev->name, q);
	if (ret) {
		dev_err(dev, "failed to request irq.\n");
		goto irq_failed;
	}
	/* handle either a boolean prop, or an explicit integer one */
	q->use_cs_mux = _of_property_bool_or_int(np, "renesas,rzn1-cs_mux", 0);

	q->dev = dev;
	platform_set_drvdata(pdev, q);

	ret = rzn1_qspi_nor_setup(q);
	if (ret)
		goto irq_failed;

	/* iterate the subnodes. */
	for_each_available_child_of_node(dev->of_node, np) {
		struct rzn1_qspi_slave *s = &q->slave[i];
		int mode = SPI_NOR_NORMAL;

		q->slave[i].host = q;
		/* Start by setting callbacks */
		q->slave[i].nor = nor_callbacks;

		s->mtd = &s->nor.mtd;
		s->nor.dev = dev;
		spi_nor_set_flash_node(&s->nor, np);
		s->nor.priv = s;

		/* get default properties from the node */
		mode = rzn1_get_slave_config(np, s);

		/* need to at least set the CS line properly here
		 * as 'prepare' is not called */
		/* For the clock, we use a slow one, as the 'probe' get ID
		 * command sometime doesn't like the faster speed on some
		 * devices */
		_rzn1_qspi_set_baudrate_div(s,
				clk_get_rate(q->clk), 1000000);
		qspi_writel(0, &s->host->reg->delay);
		/* Set clock phase/polarity */
		_rzn1_qspi_set_clk_mode(s);
		_rzn1_qspi_set_cs(s);
		_rzn1_qspi_set_readdelay(s, 0);

		if (spi_nor_scan(&s->nor, NULL, mode))
			goto scan_failed;

		if (s->read_cmd == 0)
			s->read_cmd = s->nor.read_opcode;
		/* Note: this relies on patch
		 * 0b78a2cf2a73b5f32ffda8cde7866ca61ce0e0b7
		 * That converts 'read_dummy' to 'bits' instead of bytes
		 */
		if (s->dummy_clk == 0)
			s->dummy_clk = s->nor.read_dummy;

		dev_dbg(dev, "CS:%d r:%02x p:%02x e:%02x addr:%d dummy:%d:%d\n",
			s->cs, s->nor.read_opcode, s->nor.program_opcode,
			s->nor.erase_opcode, s->nor.addr_width,
			s->nor.read_dummy, s->dummy_clk);

		rzn1_qspi_set_speed(s);
		_rzn1_qspi_set_readdelay(s, s->cap_delay);

		ret = mtd_device_register(s->mtd, NULL, 0);
		if (ret)
			goto scan_failed;
		_rzn1_qspi_set_device_size(s);
		dev_dbg(dev, "CS:%d size:%d width:%d devsz:%08x\n",
			s->cs, (int)s->mtd->size/1024*1024,
			s->nor.addr_width, readl(&q->reg->devsz));
		/* cache the IO register values with our configutation */
		s->reg.cfg = readl(&q->reg->cfg);
		s->reg.delay = readl(&q->reg->delay);
		s->reg.devsz = readl(&q->reg->devsz);
		s->reg.rddatacap = readl(&q->reg->rddatacap);

		/* force 4 bytes mode for device, even if it has specific
		 * opcodes to handle 4 bytes addresses */
		/* Normally spi_nor_scan() will switch into 4-byte address mode
		 * if supported.
		 * However, it will not switch Spansion (aka AMD) devices into
		 * 4-byte mode as it selects dedicated 4-byte commands whilst
		 * remaining in 3-byte mode. Since we allow users to specify
		 * the commands, we switch into 4-byte mode here. */
		if (s->mtd->size > 0x1000000 && s->nor.addr_width == 4) {
			/* For spansion, it's special... grrrr */
			u8 extadd = 0x80; /* Bank Register bit 7 */

			/* Send BRWR command */
			s->nor.write_reg(&s->nor, 0x17 , &extadd, 1);

			/* for everyone but spansion, send 0xb7 command */
			s->nor.write_reg(&s->nor, SPINOR_OP_EN4B, NULL, 0);

			dev_dbg(dev, "CS:%d configured 4 bytes addressing\n",
				s->cs);
		}

		i++;
		continue;
scan_failed:
		dev_err(dev, "probe failed for CS:%d\n", s->cs);
	}

	dev_info(dev, "probed\n");
	return 0;

irq_failed:
	clk_disable_unprepare(q->clk);
	clk_disable_unprepare(q->clk_fw);
clk_failed:
map_failed:
	dev_err(dev, "RZN1 QSPI probe failed\n");
	return ret;
}

static int rzn1_qspi_remove(struct platform_device *pdev)
{
	struct rzn1_qspi *q = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < q->slave_count; i++)
		mtd_device_unregister(q->slave[i].mtd);
	clk_disable_unprepare(q->clk);
	clk_disable_unprepare(q->clk_fw);

	return 0;
}


static struct of_device_id rzn1_qspi_dt_ids[] = {
	{ .compatible = "renesas,rzn1-qspi" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rzn1_qspi_dt_ids);

static struct platform_driver rzn1_qspi_driver = {
	.driver = {
		.name	= "rzn1-qspi",
		.bus	= &platform_bus_type,
		.of_match_table = rzn1_qspi_dt_ids,
	},
	.probe          = rzn1_qspi_probe,
	.remove		= rzn1_qspi_remove,
};
module_platform_driver(rzn1_qspi_driver);

MODULE_DESCRIPTION("RZN1 QSPI Controller Driver");
MODULE_AUTHOR("Michel Pollet <michel.pollet@bp.renesas.com>,<buserror@gmail.com>");
MODULE_LICENSE("GPL v2");
