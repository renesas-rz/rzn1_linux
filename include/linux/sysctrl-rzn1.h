/* SPDX-License-Identifier: GPL-2.0 */
/*
 * RZ/N1 sysctrl access API
 *
 * Copyright (C) 2014-2022 Renesas Electronics Europe Limited
 *
 * Michel Pollet <michel.pollet@bp.renesas.com>, <buserror@gmail.com>
 */

#ifndef __SYSCTRL_RZN1__
#define __SYSCTRL_RZN1__

#include <linux/io.h>
#include <dt-bindings/soc/rzn1-sysctrl.h>

#define RZN1_SYSTEM_CTRL_BASE           0x4000C000
#define RZN1_SYSTEM_CTRL_SIZE           0x1000          /* 4 KB */

/* Good policy for drivers to call this, even tho it's only needed once */
void __init rzn1_sysctrl_init(void);
/* Get the base address for the sysctrl block. Use sparingly (clock drivers) */
void __iomem *rzn1_sysctrl_base(void);

static inline u32 rzn1_sysctrl_readl(u32 reg)
{
	BUG_ON(reg >= RZN1_SYSTEM_CTRL_SIZE);
	return readl(rzn1_sysctrl_base() + reg);
}

static inline void rzn1_sysctrl_writel(u32 value, u32 reg)
{
	BUG_ON(reg >= RZN1_SYSTEM_CTRL_SIZE);
	writel(value, rzn1_sysctrl_base() + reg);
}

#endif /* __SYSCTRL_RZN1__ */
