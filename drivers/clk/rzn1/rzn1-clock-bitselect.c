// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2014 Renesas Electronics Europe Limited
 *
 * Michel Pollet <michel.pollet@bp.renesas.com>, <buserror@gmail.com>
 *
 * Clock selector driven from a register bit
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/of.h>

#include "rzn1-clock.h"

/*
 * This clock provider handles the case of the RZN1 where you have peripherals
 * that have two potential clock source and two gates, one for each of the
 * clock source - the used clock source (for all sub clocks) is selected by a
 * single bit.
 * That single bit affects all sub-clocks, and therefore needs to change the
 * active gate (and turn the others off) and force a recalculation of the rates.
 *
 * Therefore this file implements two clock providers, one 'bitselect' that
 * handles the switch between both parents, and another 'dualgate'
 * that knows which gate to poke at, depending on the parent's bit position.
 */

struct rzn1_bitselect {
	struct clk_hw	hw;
	u32 __iomem *reg;
	u8 bit;			/* bit for the selector */
	u8 gate[2][8];		/* bit index for each gate, indexed by 'bit' */
};

#define to_clk_bitselect(_hw) container_of(_hw, struct rzn1_bitselect, hw)

static u8 rzn1_bitselect_get_value(struct rzn1_bitselect *set)
{
	return clk_readl(set->reg) & (1 << set->bit) ? 1 : 0;
}

static u8 clk_mux_get_parent(struct clk_hw *hw)
{
	struct rzn1_bitselect *set = to_clk_bitselect(hw);

	return rzn1_bitselect_get_value(set);
}

static int clk_mux_set_parent(struct clk_hw *hw, u8 index)
{
	struct rzn1_bitselect *set = to_clk_bitselect(hw);

	/* a single bit in the register selects one of two parent clocks */
	if (index)
		clk_writel(clk_readl(set->reg) | (1 << set->bit), set->reg);
	else
		clk_writel(clk_readl(set->reg) & ~(1 << set->bit), set->reg);

	return 0;
}

static const struct clk_ops clk_bitselect_ops = {
	.get_parent = clk_mux_get_parent,
	.set_parent = clk_mux_set_parent,
};

/*
 * This parses the renesas,rzn1-gate property, which consists of a list of
 * 'gate' numbers for when the selector bit is 'off' (0) followed by a list of
 * gate numbers for when the selector is 'on' (1).
 *
 * renesas,rzn1-gates =
 *      <RZN1_CLK_SCLK_UART0 RZN1_CLK_SCLK_UART1
 *	RZN1_CLK_SCLK_UART2>,
 *      <RZN1_CLK_USB_SCLK_UART0 RZN1_CLK_USB_SCLK_UART1
 *	RZN1_CLK_USB_SCLK_UART2>;
 */
static int __init rzn1_read_bitselect_table(
	struct rzn1_bitselect *sel,
	struct device_node *node)
{
	u32 table_size;
	const __be32 *tablespec;
	int i;

	tablespec = of_get_property(node, "renesas,rzn1-gates", &table_size);
	if (!tablespec)
		return -EINVAL;
	table_size /= sizeof(u32);
	if (table_size & 1) {
		pr_err("%s: %s renesas,rzn1-gates needs an even # of gates\n",
				__func__, node->name);
		return -EINVAL;
	}
	table_size /= 2;
	if (table_size > sizeof(sel->gate[0])) {
		pr_err("%s: %s renesas,rzn1-gates overflow\n",
				__func__, node->name);
		return -EINVAL;
	}
	pr_devel("%s count = %d\n", __func__, table_size);
	for (i = 0; i < table_size && i < sizeof(sel->gate[0]); i++) {
		u32 g1, g2;

		of_property_read_u32_index(node, "renesas,rzn1-gates",
			i, &g1);
		of_property_read_u32_index(node, "renesas,rzn1-gates",
			table_size + i, &g2);
		pr_devel("  [%d] = %s, %s\n", i,
			rzn1_get_clk_desc(g1)->name,
			rzn1_get_clk_desc(g2)->name);
		sel->gate[0][i] = g1;
		sel->gate[1][i] = g2;
	}
	return 0;
}

static void __init rzn1_clock_bitselect_init(struct device_node *node)
{
	struct rzn1_bitselect *sel;
	struct clk *clk;
	const char *clk_name = node->name;
	const char *parent_name[2];
	u32 reg_idx;
	u32 bit_idx;
	struct clk_init_data init;

	of_property_read_string(node, "clock-output-names", &clk_name);

	parent_name[0] = of_clk_get_parent_name(node, 0);
	parent_name[1] = of_clk_get_parent_name(node, 1);

	if (!parent_name[0] || !parent_name[1]) {
		pr_err("%s: invalid parent clocks for %s\n",
				__func__, node->name);
		return;
	}
	if (of_property_read_u32(node, "reg", &reg_idx)) {
		pr_err("%s: missing reg property for %s\n",
				__func__, node->name);
		return;
	}
	if (of_property_read_u32(node, "renesas,rzn1-sel-bit", &bit_idx)) {
		pr_err("%s: missing renesas,rzn1-sel-bit property for %s\n",
				__func__, node->name);
		return;
	}

	/* allocate the gate */
	sel = kzalloc(sizeof(struct rzn1_bitselect), GFP_KERNEL);
	if (!sel)
		return;

	init.name = clk_name;
	init.ops = &clk_bitselect_ops;
	init.flags = CLK_IS_BASIC | CLK_SET_RATE_PARENT;
	init.parent_names = parent_name;
	init.num_parents = 2;

	if (rzn1_read_bitselect_table(sel, node)) {
		pr_err("%s: could not read renesas,rzn1-gates property\n",
			__func__);
		kfree(sel);
		return;
	}
	/* struct clk_gate assignments */
	sel->reg = (u32 *)(((u8 *)rzn1_sysctrl_base()) + reg_idx);
	sel->bit = bit_idx;
	sel->hw.init = &init;

	clk = clk_register(NULL, &sel->hw);

	if (!IS_ERR(clk))
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
}

CLK_OF_DECLARE(rzn1_bitselect_clk,
		"renesas,rzn1-clock-bitselect", rzn1_clock_bitselect_init);

struct rzn1_dualgate {
	struct clk_hw	hw;
	u8 gate;
};
#define to_clk_dualgate(_hw) container_of(_hw, struct rzn1_dualgate, hw)


static int clk_dualgate_setenable(struct rzn1_dualgate *gate, int enable)
{
	struct clk *parent = clk_get_parent(gate->hw.clk);
	struct rzn1_bitselect *sel = to_clk_bitselect(__clk_get_hw(parent));
	uint8_t sel_bit = rzn1_bitselect_get_value(sel);

	/* we always turn off the 'other' gate, regardless */
	rzn1_clk_set_gate(sel->gate[!sel_bit][gate->gate], 0);
	rzn1_clk_set_gate(sel->gate[sel_bit][gate->gate], enable);

	return 0;
}

static int clk_gate_enable(struct clk_hw *hw)
{
	struct rzn1_dualgate *gate = to_clk_dualgate(hw);

	clk_dualgate_setenable(gate, 1);

	return 0;
}

static void clk_gate_disable(struct clk_hw *hw)
{
	struct rzn1_dualgate *gate = to_clk_dualgate(hw);

	clk_dualgate_setenable(gate, 0);
}

static int _clk_gate_is_enabled(struct clk_hw *hw)
{
	struct rzn1_dualgate *gate = to_clk_dualgate(hw);
	struct clk *parent = clk_get_parent(gate->hw.clk);
	struct rzn1_bitselect *sel = to_clk_bitselect(__clk_get_hw(parent));
	uint8_t sel_bit = rzn1_bitselect_get_value(sel);

	return rzn1_clk_is_gate_enabled(sel->gate[sel_bit][gate->gate]);
}

static const struct clk_ops clk_dualgate_ops = {
	.enable = clk_gate_enable,
	.disable = clk_gate_disable,
	.is_enabled = _clk_gate_is_enabled,
};

static void __init rzn1_clock_dualgate_init(struct device_node *node)
{
	struct rzn1_dualgate *gate;
	struct clk *clk;
	const char *clk_name = node->name;
	const char *parent_name;
	u32 gate_idx;
	struct clk_init_data init;

	of_property_read_string(node, "clock-output-names", &clk_name);

	parent_name = of_clk_get_parent_name(node, 0);

	if (of_property_read_u32(node, "reg", &gate_idx)) {
		pr_err("%s: missing gate index property for %s\n",
				__func__, node->name);
		return;
	}

	/* allocate the gate */
	gate = kzalloc(sizeof(struct rzn1_dualgate), GFP_KERNEL);
	if (!gate)
		return;

	init.name = clk_name;
	init.ops = &clk_dualgate_ops;
	init.flags = CLK_IS_BASIC | CLK_SET_RATE_PARENT;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	/* struct clk_gate assignments */
	gate->gate = gate_idx;
	gate->hw.init = &init;

	clk = clk_register(NULL, &gate->hw);

	if (!IS_ERR(clk))
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
}

CLK_OF_DECLARE(rzn1_dualgate_clk,
		"renesas,rzn1-clock-dualgate", rzn1_clock_dualgate_init);

