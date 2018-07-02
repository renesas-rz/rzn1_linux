/*
 * (C) Copyright 2014 Altera Corporation <www.altera.com>
 * (C) Copyright 2015 Renesas Electronics Europe Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __CADENCE_QSPI_H__
#define __CADENCE_QSPI_H__

struct cadence_qspi {
	u32	cfg;
	u32	devrd;
	u32	devwr;
	u32	delay;
	u32	rddatacap; /* 0x10 */
	u32	devsz;
	u32	srampart;
	u32	indaddrtrig;
	u32	dmaper; /* 0x20 */
	u32	remapaddr;
	u32	modebit;
	u32	sramfill;
	u32	txthresh; /* 0x30 */
	u32	rxthresh;
	u32	_pad_0x38_0x3f[2];
	u32	irqstat;
	u32	irqmask;
	u32	_pad_0x48_0x4f[2];
	u32	lowwrprot;
	u32	uppwrprot;
	u32	wrprot;
	u32	_pad_0x5c_0x5f;
	u32	indrd;
	u32	indrdwater;
	u32	indrdstaddr;
	u32	indrdcnt;
	u32	indwr; /* 0x70 */
	u32	indwrwater;
	u32	indwrstaddr;
	u32	indwrcnt;
	u32	_pad_0x80_0x8f[4];
	u32	flashcmd;
	u32	flashcmdaddr;
	u32	_pad_0x98_0x9f[2];
	u32	flashcmdrddatalo;
	u32	flashcmdrddataup;
	u32	flashcmdwrdatalo;
	u32	flashcmdwrdataup;
	u32	_pad_0xb0_0xfb[19];
	u32	moduleid;
};

/* Controller's configuration and status register */
#define	CQSPI_REG_CONFIG_CLK_POL_LSB		1
#define	CQSPI_REG_CONFIG_CLK_PHA_LSB		2
#define	CQSPI_REG_CONFIG_ENABLE_MASK		(1 << 0)
#define	CQSPI_REG_CONFIG_DIRECT_MASK		(1 << 7)
#define	CQSPI_REG_CONFIG_DECODE_MASK		(1 << 9)
#define	CQSPI_REG_CONFIG_AHB_ENABLE_MASK	(1 << 16)
#define	CQSPI_REG_CONFIG_XIP_IMM_MASK		(1 << 18)
#define	CQSPI_REG_CONFIG_CHIPSELECT_LSB		10
#define	CQSPI_REG_CONFIG_BAUD_LSB		19
#define	CQSPI_REG_CONFIG_IDLE_LSB		31
#define	CQSPI_REG_CONFIG_CHIPSELECT_MASK	0xF
#define	CQSPI_REG_CONFIG_BAUD_MASK		0xF
#define	CQSPI_REG_RD_INSTR_OPCODE_LSB		0
#define	CQSPI_REG_RD_INSTR_TYPE_INSTR_LSB	8
#define	CQSPI_REG_RD_INSTR_TYPE_ADDR_LSB	12
#define	CQSPI_REG_RD_INSTR_TYPE_DATA_LSB	16
#define	CQSPI_REG_RD_INSTR_MODE_EN_LSB		20
#define	CQSPI_REG_RD_INSTR_DUMMY_LSB		24
#define	CQSPI_REG_RD_INSTR_TYPE_INSTR_MASK	0x3
#define	CQSPI_REG_RD_INSTR_TYPE_ADDR_MASK	0x3
#define	CQSPI_REG_RD_INSTR_TYPE_DATA_MASK	0x3
#define	CQSPI_REG_RD_INSTR_DUMMY_MASK		0x1F
#define	CQSPI_REG_WR_INSTR_OPCODE_LSB		0
#define	CQSPI_REG_DELAY_TSLCH_LSB		0
#define	CQSPI_REG_DELAY_TCHSH_LSB		8
#define	CQSPI_REG_DELAY_TSD2D_LSB		16
#define	CQSPI_REG_DELAY_TSHSL_LSB		24
#define	CQSPI_REG_DELAY_TSLCH_MASK		0xFF
#define	CQSPI_REG_DELAY_TCHSH_MASK		0xFF
#define	CQSPI_REG_DELAY_TSD2D_MASK		0xFF
#define	CQSPI_REG_DELAY_TSHSL_MASK		0xFF
#define	CQSPI_REG_RD_DATA_CAPTURE_BYPASS_LSB	0
#define	CQSPI_REG_RD_DATA_CAPTURE_DELAY_LSB	1
#define	CQSPI_REG_RD_DATA_CAPTURE_DELAY_MASK	0xF
#define CQSPI_REG_RD_DATA_CAPTURE_EDGE_LSB	5
#define	CQSPI_REG_SIZE_ADDRESS_LSB		0
#define	CQSPI_REG_SIZE_PAGE_LSB			4
#define	CQSPI_REG_SIZE_BLOCK_LSB		16
#define	CQSPI_REG_SIZE_ADDRESS_MASK		0xF
#define	CQSPI_REG_SIZE_PAGE_MASK		0xFFF
#define	CQSPI_REG_SIZE_BLOCK_MASK		0x3F
#define	CQSPI_REG_SRAMLEVEL_RD_LSB		0
#define	CQSPI_REG_SRAMLEVEL_WR_LSB		16
#define	CQSPI_REG_SRAMLEVEL_RD_MASK		0xFFFF
#define	CQSPI_REG_SRAMLEVEL_WR_MASK		0xFFFF
#define CQSPI_REG_WRPROT_INVERT			(1 << 0)
#define CQSPI_REG_IRQMASK_WP			(1 << 4)
#define CQSPI_REG_WRPROT_INVERT			(1 << 0)
#define CQSPI_REG_WRPROT_ENABLE_MASK		(1 << 1)
#define	CQSPI_REG_INDIRECTRD_START_MASK		(1 << 0)
#define	CQSPI_REG_INDIRECTRD_CANCEL_MASK	(1 << 1)
#define	CQSPI_REG_INDIRECTRD_INPROGRESS_MASK	(1 << 2)
#define	CQSPI_REG_INDIRECTRD_DONE_MASK		(1 << 5)
#define	CQSPI_REG_CMDCTRL_EXECUTE_MASK		(1 << 0)
#define	CQSPI_REG_CMDCTRL_INPROGRESS_MASK	(1 << 1)
#define	CQSPI_REG_CMDCTRL_DUMMY_LSB		7
#define	CQSPI_REG_CMDCTRL_WR_BYTES_LSB		12
#define	CQSPI_REG_CMDCTRL_WR_EN_LSB		15
#define	CQSPI_REG_CMDCTRL_ADD_BYTES_LSB		16
#define	CQSPI_REG_CMDCTRL_ADDR_EN_LSB		19
#define	CQSPI_REG_CMDCTRL_RD_BYTES_LSB		20
#define	CQSPI_REG_CMDCTRL_RD_EN_LSB		23
#define	CQSPI_REG_CMDCTRL_OPCODE_LSB		24
#define	CQSPI_REG_CMDCTRL_DUMMY_MASK		0x1F
#define	CQSPI_REG_CMDCTRL_WR_BYTES_MASK		0x7
#define	CQSPI_REG_CMDCTRL_ADD_BYTES_MASK	0x3
#define	CQSPI_REG_CMDCTRL_RD_BYTES_MASK		0x7
#define	CQSPI_REG_CMDCTRL_OPCODE_MASK		0xFF
#define	CQSPI_REG_INDIRECTWR_START_MASK		(1 << 0)
#define	CQSPI_REG_INDIRECTWR_CANCEL_MASK	(1 << 1)
#define	CQSPI_REG_INDIRECTWR_INPROGRESS_MASK	(1 << 2)
#define	CQSPI_REG_INDIRECTWR_DONE_MASK		(1 << 5)

/* Transfer mode */
#define CQSPI_INST_TYPE_SINGLE			0
#define CQSPI_INST_TYPE_DUAL			1
#define CQSPI_INST_TYPE_QUAD			2

/* controller operation setting */
#define CQSPI_READ_CAPTURE_MAX_DELAY		16
#define CQSPI_REG_POLL_US			1
#define CQSPI_REG_RETRY				10000
#define CQSPI_POLL_IDLE_RETRY			3
#define CQSPI_STIG_DATA_LEN_MAX			8

#endif /* __CADENCE_QSPI_H__ */
