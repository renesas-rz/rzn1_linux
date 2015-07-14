/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2014 Renesas Electronics Europe Limited
 *
 * Michel Pollet <michel.pollet@bp.renesas.com>, <buserror@gmail.com>
 */

#ifndef __RZN1_CLOCK_H
#define __RZN1_CLOCK_H

#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/sysctrl-rzn1.h>

/* Generic register/bit group descriptor */
struct rzn1_onereg {
	uint16_t reg : 7,	/* Register number (word) */
		pos : 5,	/* Bit number */
		size : 4;	/* Optional: size in bits */
};

struct rzn1_clkdesc {
	const char *name;
	struct rzn1_onereg clock, reset, ready, masteridle;
};

/*
 * These accessors allows control of the clock gates as
 * defined in rzn1-clocks.h
 */
const struct rzn1_clkdesc *rzn1_get_clk_desc(int clkdesc_id);
void rzn1_clk_set_gate(int clkdesc_id, int on);
int rzn1_clk_is_gate_enabled(int clkdesc_id);

/*
 * This allow setting overrride and/or supplemental functions to specific
 * clocks; the current case is for the USB clock that needs a special PLL
 * register to be set.
 */
enum {
	RZN1_CLK_HOOK_GATE_PRE = 0,
	RZN1_CLK_HOOK_GATE_SET,
	RZN1_CLK_HOOK_GATE_POST,
};

typedef int (*rzn1_clk_hook)(int clkdesc_id, int operation, u32 value);

void rzn1_clk_set_hook(int clkdesc_id, rzn1_clk_hook clk_hook);

#endif /* __RZN1_CLOCK_H */

