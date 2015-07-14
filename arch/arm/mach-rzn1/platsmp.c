// SPDX-License-Identifier: GPL-2.0
/*
 * SMP support for Renesas RZ/N1
 *
 * Copyright (C) 2014 Renesas Electronics Europe Limited
 *
 * Michel Pollet <michel.pollet@bp.renesas.com>, <buserror@gmail.com>
 *
 * Based on code
 *  Copyright (C) 2012-2013 Allwinner Ltd.
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/memory.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/smp.h>
#include <asm/system_misc.h>

#include <linux/smp.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/sysctrl-rzn1.h>

#include <linux/reset.h>
#include <linux/cpu.h>
#include <asm/cacheflush.h>
#include <asm/cp15.h>
#include <asm/smp_scu.h>
#include <asm/smp_plat.h>
#include <asm/mach/map.h>

#define BOOTADDR2_CANARY	0x525a4e31

static void __iomem *pen2_base;

static DEFINE_SPINLOCK(cpu_lock);

/* The alternate boot address for the second core can be overridden in the DT,
 * typically this will happen if the bootloader decides to park the second
 * core somewhere else than the fixed ALT_BOOTADDR address
 */
static void __init rzn1_smp_prepare_cpus(unsigned int max_cpus)
{
	u32 bootaddr = 0;
	struct device_node *np = of_find_node_by_path("/chosen");

	if (np)
		of_property_read_u32(np, "rzn1,bootaddr", &bootaddr);

	if (bootaddr &&
	    bootaddr != RZN1_SYSCTRL_REG_BOOTADDR &&
		bootaddr != (RZN1_SYSTEM_CTRL_BASE + RZN1_SYSCTRL_REG_BOOTADDR)) {
		pr_info("RZ/N1 CPU#2 boot address %08x\n", bootaddr);
		pen2_base = ioremap(bootaddr, 8);

		if (!pen2_base) {
			pr_warn("Couldn't map RZ/N1 CPU#2 PEN2\n");
			return;
		}
	} else {
		pr_info("RZ/N1 CPU#2 boot address not specified - using SYSCTRL reg\n");
	}
}

static int rzn1_smp_boot_secondary(unsigned int cpu,
				   struct task_struct *idle)
{
	u32 t = (u32)virt_to_phys(secondary_startup);

	/* Set CPU boot address */
	if (pen2_base && (readl(pen2_base + 4) == BOOTADDR2_CANARY))
		pr_info("RZ/N1 CPU#%d writing %08x to boot address\n", cpu, t);
	else
		pr_info("RZ/N1 CPU#%d writing %08x to SYSCTRL reg\n", cpu, t);

	spin_lock(&cpu_lock);

	/* Set CPU boot address */
	if (pen2_base && (readl(pen2_base + 4) == BOOTADDR2_CANARY))
		writel(virt_to_phys(secondary_startup), pen2_base);
	else
		rzn1_sysctrl_writel(virt_to_phys(secondary_startup),
				    RZN1_SYSCTRL_REG_BOOTADDR);

	arch_send_wakeup_ipi_mask(cpumask_of(cpu));

	/*
	 * now the secondary core is starting up let it run its
	 * calibrations, then wait for it to finish
	 */
	spin_unlock(&cpu_lock);

	return 0;
}

struct smp_operations rzn1_smp_ops __initdata = {
	.smp_prepare_cpus	= rzn1_smp_prepare_cpus,
	.smp_boot_secondary	= rzn1_smp_boot_secondary,
};

CPU_METHOD_OF_DECLARE(rzn1_smp, "renesas,rzn1", &rzn1_smp_ops);
