/*
 * Copyright (C) 2022 Renesas Electronics Europe Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __RZN1_A5PSW_WORKAROUND__
#define __RZN1_A5PSW_WORKAROUND__

#include <linux/io.h>

#ifdef CONFIG_RZN1_A5PSW_WORKAROUND
void __init rzn1_a5psw_workaround_init(void);
void rzn1_a5psw_workaround_lock(unsigned long *flags);
void rzn1_a5psw_workaround_unlock(unsigned long *flags);
void rzn1_a5psw_workaround_writel(unsigned int value, volatile void __iomem *addr);
#else
static inline void __init rzn1_a5psw_workaround_init(void)			{ return; }
static inline void rzn1_a5psw_workaround_lock(unsigned long *flags)		{ return; }
static inline void rzn1_a5psw_workaround_unlock(unsigned long *flags)	{ return; }
static inline void rzn1_a5psw_workaround_writel( unsigned int value,
	volatile void __iomem *addr)			{ writel(value, addr); }
#endif

#endif /* __RZN1_A5PSW_WORKAROUND__ */
