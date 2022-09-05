// SPDX-License-Identifier: GPL-2.0
/*
 * RZ/N1 processor support file
 *
 * Copyright (C) 2014 Renesas Electronics Europe Limited
 *
 * Michel Pollet <michel.pollet@bp.renesas.com>, <buserror@gmail.com>
 */

#include <asm/mach/arch.h>
#include <linux/kernel.h>
#include <linux/of_platform.h>
#include <linux/sysctrl-rzn1.h>

static void rzn1_restart(enum reboot_mode mode, const char *cmd)
{
	u32 val;

	val = rzn1_sysctrl_readl(RZN1_SYSCTRL_REG_RSTEN);
	val |= (1 << RZN1_SYSCTRL_REG_RSTEN_SWRST_EN);
	val |= (1 << RZN1_SYSCTRL_REG_RSTEN_MRESET_EN);
	rzn1_sysctrl_writel(val, RZN1_SYSCTRL_REG_RSTEN);

	val = rzn1_sysctrl_readl(RZN1_SYSCTRL_REG_RSTCTRL);
	val |= (1 << RZN1_SYSCTRL_REG_RSTCTRL_SWRST_REQ);
	rzn1_sysctrl_writel(val, RZN1_SYSCTRL_REG_RSTCTRL);
}

static const char *rzn1_boards_compat_dt[] __initconst = {
	"renesas,r9a06g032",
	NULL,
};

DT_MACHINE_START(rzn1_DT, "Renesas RZ/N1 (DT)")
	.dt_compat	= rzn1_boards_compat_dt,
	.restart	= rzn1_restart,
MACHINE_END
