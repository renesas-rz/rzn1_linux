// SPDX-License-Identifier: GPL-2.0
/*
 * RZ/N1 sysctrl access API
 *
 * Copyright (C) 2014-2022 Renesas Electronics Europe Limited
 *
 * Michel Pollet <michel.pollet@bp.renesas.com>, <buserror@gmail.com>
 */

#include <linux/kernel.h>
#include <linux/sysctrl-rzn1.h>
#include <linux/rzn1-a5psw-workaround.h>

static void __iomem *sysctrl_base_addr;
static void __iomem *watchdog_base_addr;

#define RZN1_WATCHDOG0_BASE             0x40008000
#define RZN1_WATCHDOG0_SIZE             0x1000          /* 4 KB */

void __init rzn1_sysctrl_init(void)
{
	if (sysctrl_base_addr)
		return;
	sysctrl_base_addr = ioremap(RZN1_SYSTEM_CTRL_BASE,
				    RZN1_SYSTEM_CTRL_SIZE);
	BUG_ON(!sysctrl_base_addr);

	watchdog_base_addr = ioremap(RZN1_WATCHDOG0_BASE,
				     3 * RZN1_WATCHDOG0_SIZE);
	BUG_ON(!watchdog_base_addr);

	rzn1_a5psw_workaround_init();
}

void __iomem *rzn1_sysctrl_base(void)
{
	BUG_ON(!sysctrl_base_addr);
	return sysctrl_base_addr;
}
EXPORT_SYMBOL(rzn1_sysctrl_base);
