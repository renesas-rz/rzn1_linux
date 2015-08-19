/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * linux/drivers/video/db9000fb.h
 *    -- Digital Blocks DB9000 LCD Controller Frame Buffer Device
 * Copyright (C) 2016 Renesas Electronics Europe Ltd.
 * Copyright (C) 2010 Digital Blocks, Inc.
 *
 * Based on pxafb.h
 * Copyright (C) 1999 Eric A. Thomas.
 * Copyright (C) 2004 Jean-Frederic Clere.
 * Copyright (C) 2004 Ian Campbell.
 * Copyright (C) 2004 Jeff Lackey.
 * Based on sa1100fb.c Copyright (C) 1999 Eric A. Thomas
 * which in turn is
 * Based on acornfb.c Copyright (C) Russell King.
 *
 *  2001-08-03: Cliff Brake <cbrake@acclent.com>
 *	 - ported SA1100 code to PXA
 *  2010-05-01: Guy Winter <gwinter@digitalblocks.com>
 *  - ported pxafb code to DB9000
 */

#ifndef __DB9000FB_H__
#define __DB9000FB_H__

/* LCD Controller Control Register 1 */
#define DB9000_CR1	(0x000)
/* Horizontal Timing Register */
#define DB9000_HTR	(0x008)
/* Vertical Timing Register 1 */
#define DB9000_VTR1	(0x00C)
/* Vertical Timing Register 2 */
#define DB9000_VTR2	(0x010)
/* Pixel Clock Timing Register */
#define DB9000_PCTR	(0x014)
/* Interrupt Status Register */
#define DB9000_ISR	(0x018)
/* Interrupt Mask Register */
#define DB9000_IMR	(0x01C)
/* Interrupt Vector Register */
#define DB9000_IVR	(0x020)
/* Interrupt Scan Compare Register */
#define DB9000_ISCR	(0x024)
/* DMA Base Address Register */
#define DB9000_DBAR	(0x028)
/* DMA Current Address Register */
#define DB9000_DCAR	(0x02C)
/* DMA End Address Register */
#define DB9000_DEAR	(0x030)
/* PWM Frequency Register */
#if defined(CONFIG_FB_DB9000_BLINK)
#define DB9000_PWMFR	(0x04c)
#define DB9000_PWMFR_BLINK	(0x034)
#else
#define DB9000_PWMFR	(0x034)
#endif
/* PWM Duty Cycle Register */
#if defined(CONFIG_FB_DB9000_BLINK)
#define DB9000_PWMDCR	(0x050)
#define DB9000_PWMDCR_BLINK	(0x038)
#else
#define DB9000_PWMDCR	(0x038)
#endif
/* DMA Frame Descriptor Branch Address Register */
#define DB9000_DFBAR	(0x03C)
/* DMA Frame Descriptor Last Address Register */
#define DB9000_DFLAR	(0x040)
/* DMA Horizontal and Vertical Timing Extension Register */
#define DB9000_HVTER	(0x044)
/* GPIO Register */
#define DB9000_GPIOR	(0x1F8)
/* Core Identification Register */
#define DB9000_CIR	(0x1FC)
/* Palette Data Words */
#define DB9000_PALT	(0x200)

/* Multiple Memory Reads Request Register */
#define DB9000_MRR	(0xFFC)

/* Control Register 1, Offset 0x000 */
/* LCD Controller Enable */
#define DB9000_CR1_ENB		BIT(0)
/* LCD Power Enable */
#define DB9000_CR1_LPE		BIT(1)
/* LCD Bits per Pixel */
#define DB9000_CR1_BPP(x)	(((x) & 0x7) << 2)
/* RGB or BGR Format */
#define DB9000_CR1_RGB		BIT(5)
/* Big or Little Endian Pixel Ordering */
#define DB9000_CR1_EPO		BIT(6)
/* Big or Little Endian Byte Ordering  */
#define DB9000_CR1_EBO		BIT(7)
/* Data Enable Polarity */
#define DB9000_CR1_DEP		BIT(8)
/* Pixel Clock Polarity */
#define DB9000_CR1_PCP		BIT(9)
/* Horizontal Sync Polarity */
#define DB9000_CR1_HSP		BIT(10)
/* Vertical Sync Polarity */
#define DB9000_CR1_VSP		BIT(11)
/* Output Pixel Select */
#define DB9000_CR1_OPS(x)	(((x) & 0x7) << 12)
/* FIFO DMA Request Words */
#define DB9000_CR1_FDW(x)	(((x) & 0x3) << 16)
/* LCD 1 or Port Select */
#define DB9000_CR1_LPS		BIT(18)
/* Frame Buffer 24bpp Packed Word */
#define DB9000_CR1_FBP		BIT(19)
/* DMA End Address Enable */
#define DB9000_CR1_DEE		BIT(20)
/* 1 bit per pixel */
#define DB9000_CR1_BPP_1bpp	(0)
/* 2 bits per pixel */
#define DB9000_CR1_BPP_2bpp	(1)
/* 4 bits per pixel */
#define DB9000_CR1_BPP_4bpp	(2)
/* 8 bits per pixel */
#define DB9000_CR1_BPP_8bpp	(3)
/* 16 bits per pixel */
#define DB9000_CR1_BPP_16bpp	(4)
/* 18 bits per pixel */
#define DB9000_CR1_BPP_18bpp	(5)
/* 24 bits per pixel */
#define DB9000_CR1_BPP_24bpp	(6)
/*  Pixel clock Rising-Edge */
#define DB9000_CR1_PixRsEdg	(DB9000_CR1_PCP * 0)
/*  Pixel clock Falling-Edge */
#define DB9000_CR1_PixFlEdg	(DB9000_CR1_PCP * 1)

/* Horizontal Timing Register, Offset 0x008 */
/* Horizontal Front Porch */
#define DB9000_HTR_HFP(x)	(((x) & 0xff) << 0)
/* Pixels per Line */
#define DB9000_HTR_PPL(x)	(((x) & 0xff) << 8)
/* Horizontal Back Porch */
#define DB9000_HTR_HBP(x)	(((x) & 0xff) << 16)
/* Horizontal Sync Width */
#define DB9000_HTR_HSW(x)	(((x) & 0xff) << 24)

/* Vertical Timing Register 1, Offset 0x00C */
/* Vertical Sync Width */
#define DB9000_VTR1_VSW(x)	(((x) & 0xff) << 0)
/* Vertical Front Porch */
#define DB9000_VTR1_VFP(x)	(((x) & 0xff) << 8)
/* Vertical Back Porch */
#define DB9000_VTR1_VBP(x)	(((x) & 0xff) << 16)

/* Vertical and Horizontal Timing Extension Register, Offset 0x044 */
/* Horizontal Front Porch Extension */
#define DB9000_HVTER_HFPE(x)	((((x) >> 8) & 0x3) << 0)
/* Horizontal Back Porch Extension */
#define DB9000_HVTER_HBPE(x)	((((x) >> 8) & 0x3) << 4)
/* Vertical Front Porch Extension */
#define DB9000_HVTER_VFPE(x)	((((x) >> 8) & 0x3) << 8)
/* Vertical Back Porch Extension */
#define DB9000_HVTER_VBPE(x)	((((x) >> 8) & 0x3) << 12)

/* DB9000 Revisions */
#define DB9000_REVISION_1_14	(0x0E)

/* Vertical Timing Register 2, Offset 0x010 */
/* Lines Per Panel */
#define DB9000_VTR2_LPP(x)	(((x) & 0xfff) << 0)

/* Pixel Clock Timing Register, Offset 0x014 */
/* Pixel Clock Divider */
#define DB9000_PCTR_PCD(x)	(((x) & 0xff) << 0)
/* Pixel Clock Divider Bypass */
#define DB9000_PCTR_PCB		BIT(8)
/* Pixel Clock Input Select */
#define DB9000_PCTR_PCI		BIT(9)
/* clock reset select */
#define DB9000_PCTR_PCR		BIT(10)

/* Interrupt Status Register, Offset 0x018 */
#define DB9000_ISR_OFU	BIT(0) /* Output FIFO Underrun */
#define DB9000_ISR_OFO	BIT(1) /* Output FIFO Overrun */
#define DB9000_ISR_IFU	BIT(2) /* Input FIFO Underrun */
#define DB9000_ISR_IFO	BIT(3) /* Input FIFO Overrun */
#define DB9000_ISR_FER	BIT(4) /* OR of OFU, OFO, IFU, IFO */
#define DB9000_ISR_MBE	BIT(5) /* Master Bus Error */
#define DB9000_ISR_VCT	BIT(6) /* Vertical Compare Triggered */
#define DB9000_ISR_BAU	BIT(7) /* DMA Base Address Register Update to CAR */
#define DB9000_ISR_LDD	BIT(8) /* LCD Controller Disable Done */

#define DB9000_ISR_ABL	BIT(9) /* AXI Master - Read Burst Length Error */
#define DB9000_ISR_ARI	BIT(10) /* AXI Master - Return ID Error */
#define DB9000_ISR_ARS	BIT(11) /* AXI Master - Response Signal Error */

#define DB9000_ISR_FBE	BIT(12) /* Frame Descriptor - Bus Error */
#define DB9000_ISR_FNC	BIT(13) /* Frame Descriptor - Node Complete */
#define DB9000_ISR_FLC	BIT(14) /* Frame Descriptor - List Complete */

/* Interrupt Mask Register, Offset 0x01C */
#define DB9000_ISR_OFUM	BIT(0)  /* Output FIFO Underrun - Mask */
#define DB9000_ISR_OFOM	BIT(1)  /* Output FIFO Overrun - Mask */
#define DB9000_ISR_IFUM	BIT(2)  /* Input FIFO Underrun - Mask */
#define DB9000_ISR_IFOM	BIT(3)  /* Input FIFO Overrun - Mask */
#define DB9000_ISR_FERM	BIT(4)  /* OR of OFU, OFO, IFU, IFO - Mask */
#define DB9000_ISR_MBEM	BIT(5)  /* Master Bus Error - Mask */
#define DB9000_ISR_VCTM	BIT(6)  /* Vertical Compare Triggered - Mask */
/* DMA Base Address Register Update to CAR - Mask */
#define DB9000_ISR_BAUM	BIT(7)
#define DB9000_ISR_LDDM	BIT(8)  /* LCD Controller Disable Done - Mask */

/* AXI Master - Read Burst Length Error - Mask */
#define DB9000_ISR_ABLM	BIT(9)
/* AXI Master - Return ID Error - Mask */
#define DB9000_ISR_ARIM	BIT(10)
/* AXI Master - Response Signal Error - Mask */

#define DB9000_ISR_ARSM	BIT(11)
#define DB9000_ISR_FBEM	BIT(12) /* Frame Descriptor - Bus Error - Mask */
#define DB9000_ISR_FNCM	BIT(13) /* Frame Descriptor - Node Complete - Mask */
#define DB9000_ISR_FLCM	BIT(14) /* Frame Descriptor - List Complete - Mask */

/* PWM Frequency Registers, Offset 0x034 and 0x04c */
#define DB9000_PWMFR_PWM_FCD(x)	(((x) & 0x3fffff) << 0)
#define DB9000_PWMFR_PWM_FCE	BIT(22)

/* PWM Duty Cycle Registers, Offset 0x038 and 0x050 */
#define DB9000_PWMDCR_DCR(x)	((x) & 0xff)

/* Multiple Memory Reads Request Register, offset 0xFFC */
#define DB9000_MRR_MRR(x)	(((x) & 0x3) << 0)
#define DB9000_MRR_DEAR_MRR(x)	((x) & 0xFFFFFFFC)
#define DB9000_MRR_OUTST_0	0x0
#define DB9000_MRR_OUTST_2	0x1
#define DB9000_MRR_OUTST_4	0x2

#endif /* __DB9000FB_H__ */
