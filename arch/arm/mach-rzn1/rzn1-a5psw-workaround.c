/*
 * Copyright (C) 2022 Renesas Electronics Europe Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/sysctrl-rzn1.h>
#include <dt-bindings/soc/rzn1-memory-map.h>

/*
 * Only one semaphore user currently, we use the last one.
 * CPU1 is for Cortex-M3
 * CPU2 is for Cortex-A7 core 0
 * CPU3 is for Cortex-A7 core 1
 */
#define SEMAPHORE_LOCK_CPU2_ETH (0x1000 + (0x10 * 63))
#define SEMAPHORE_LOCK_CPU3_ETH (0x2000 + (0x10 * 63))

/*
 * Implementation #1: using h/w semaphore
 */
#ifdef CONFIG_RZN1_A5PSW_WORKAROUND_SEMAPHORE

static void __iomem *rzn1_semaphore_addr[2];

void __init rzn1_a5psw_workaround_init(void)
{
	void __iomem *semaphore_base_addr;
	u32 val;

	semaphore_base_addr = ioremap(RZN1_SEMAPHORE_BASE, RZN1_SEMAPHORE_SIZE);
	BUG_ON(!semaphore_base_addr);

	/* The two semaphore registers provide per-cpu access to the underlying semaphore */
	rzn1_semaphore_addr[0] = semaphore_base_addr + SEMAPHORE_LOCK_CPU2_ETH;
	rzn1_semaphore_addr[1] = semaphore_base_addr + SEMAPHORE_LOCK_CPU3_ETH;

	/* Activate semaphore */
	val = rzn1_sysctrl_readl(RZN1_SYSCTRL_REG_PWRCTRL_PG4_FW);
	val &= ~BIT(RZN1_SYSCTRL_REG_PWRCTRL_PG4_FW_MIREQ_UI);
	val |= BIT(RZN1_SYSCTRL_REG_PWRCTRL_PG4_FW_SLVRDY_AD);
	rzn1_sysctrl_writel(val, RZN1_SYSCTRL_REG_PWRCTRL_PG4_FW);

	pr_info("rzn1-a5psw-workaround: using semaphore method\n");
}
EXPORT_SYMBOL(rzn1_a5psw_workaround_init);

void rzn1_a5psw_workaround_lock(unsigned long *flags)
{
	void __iomem *semaphore;

	/* Disable task switching on local CPU */
	preempt_disable();
	local_irq_save(*flags);

	/* Use hardware semaphore to ensure only one CPU runs.
	 * Notably this includes the Cortex-M3 secondary CPU.
	 */
	semaphore = rzn1_semaphore_addr[smp_processor_id()];
	if (readl(semaphore)) {
		while (readl(semaphore))
			;
	}
}
EXPORT_SYMBOL(rzn1_a5psw_workaround_lock);

void rzn1_a5psw_workaround_unlock(unsigned long *flags)
{
	void __iomem *semaphore;

	semaphore = rzn1_semaphore_addr[smp_processor_id()];

	/* Release hardware sem */
	writel(0, semaphore);

	/* Restore local task switching */
	local_irq_restore(*flags);
	preempt_enable();
}
EXPORT_SYMBOL(rzn1_a5psw_workaround_unlock);
#endif

/*
 * Implementation #2: using spinlocks
 */
#ifdef CONFIG_RZN1_A5PSW_WORKAROUND_SPINLOCK

static spinlock_t spinlock;

void __init rzn1_a5psw_workaround_init(void)
{
	spin_lock_init(&spinlock);

	pr_info("rzn1-a5psw-workaround: using spinlock method\n");
}
EXPORT_SYMBOL(rzn1_a5psw_workaround_init);

void rzn1_a5psw_workaround_lock(unsigned long *flags)
{
	spin_lock_irqsave(&spinlock, *flags);
}
EXPORT_SYMBOL(rzn1_a5psw_workaround_lock);

void rzn1_a5psw_workaround_unlock(unsigned long *flags)
{
	spin_unlock_irqrestore(&spinlock, *flags);
}
EXPORT_SYMBOL(rzn1_a5psw_workaround_unlock);

#endif

/* Normal writel() with semaphore held */
void rzn1_a5psw_workaround_writel(unsigned int value, volatile void __iomem *addr)
{
	unsigned long flags;

	rzn1_a5psw_workaround_lock(&flags);
	writel(value, addr);
	rzn1_a5psw_workaround_unlock(&flags);
}
EXPORT_SYMBOL(rzn1_a5psw_workaround_writel);
