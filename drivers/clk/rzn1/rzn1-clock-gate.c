// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2014 Renesas Electronics Europe Limited
 *
 * Michel Pollet <michel.pollet@bp.renesas.com>, <buserror@gmail.com>
 */

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/regmap.h>

#include "rzn1-clock.h"

/*
 * This implements the RZ/N1 clock gate 'driver'. We cannot use the system's
 * clock gate framework as the gates on the RZ/N1 have a special enabling
 * sequence, therefore we use this little proxy to call into the general
 * clock gate API in rznr-clock.c that implements what is needed.
 */
struct rzn1_gate {
	struct clk_hw	hw;
	u32 gate_idx;
	unsigned no_disable : 1;
};

#define to_rzn1_gate(_hw) container_of(_hw, struct rzn1_gate, hw)

static int clk_gate_enable(struct clk_hw *hw)
{
	struct rzn1_gate *g = to_rzn1_gate(hw);

	rzn1_clk_set_gate(g->gate_idx, 1);
	return 0;
}

static void clk_gate_disable(struct clk_hw *hw)
{
	struct rzn1_gate *g = to_rzn1_gate(hw);

	if (!g->no_disable)
		rzn1_clk_set_gate(g->gate_idx, 0);
	else
		pr_info("%s %s: disallowed\n", __func__,
			__clk_get_name(hw->clk));
}

static int _clk_gate_is_enabled(struct clk_hw *hw)
{
	struct rzn1_gate *g = to_rzn1_gate(hw);

	return rzn1_clk_is_gate_enabled(g->gate_idx);
}

static const struct clk_ops _clk_gate_ops = {
	.enable = clk_gate_enable,
	.disable = clk_gate_disable,
	.is_enabled = _clk_gate_is_enabled,
};

static void __init rzn1_clock_gate_init(struct device_node *node)
{
	struct clk *clk;
	const char *clk_name = node->name;
	const char *parent_name;
	u32 gate_idx = 0;
	struct rzn1_gate *g;
	struct clk_init_data init;

	of_property_read_string(node, "clock-output-names", &clk_name);

	parent_name = of_clk_get_parent_name(node, 0);

	/* Some clock in the clock tree do not have a declared gate register */
	if (of_property_read_u32(node, "reg", &gate_idx))
		return;

	/* allocate the gate */
	g = kzalloc(sizeof(struct rzn1_gate), GFP_KERNEL);
	if (!g)
		return;

	init.name = clk_name;
	init.ops = &_clk_gate_ops;
	init.flags = CLK_IS_BASIC | CLK_SET_RATE_PARENT;
	init.parent_names = parent_name ? &parent_name : NULL;
	init.num_parents = parent_name ? 1 : 0;

	/* struct clk_gate assignments */
	g->gate_idx = gate_idx;
	g->hw.init = &init;
	g->no_disable = of_property_read_bool(node, "renesas,no-disable");

	clk = clk_register(NULL, &g->hw);

	if (!IS_ERR(clk))
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
	else
		kfree(g);
}

CLK_OF_DECLARE(rzn1_gate_clk, "renesas,rzn1-clock-gate", rzn1_clock_gate_init);
