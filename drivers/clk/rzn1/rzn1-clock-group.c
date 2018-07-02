/*
 * Copyright (C) 2014 Renesas Electronics Europe Limited
 *
 * Michel Pollet <michel.pollet@bp.renesas.com>, <buserror@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/*
 * This driver is needed to allow clocks to be 'grouped' together, with
 * just one enable/disable. It is needed because the RZN1 has sometime
 * several clocks that need to be enabled for a peripheral to work, while
 * most of the time the drivers has no nothing of more than one clock (at
 * best).
 * So this driver allow the grouping of these multiple sclk, pclk and so
 * forth into one 'umbrella' clock that will enable (and claim) all the
 * specified parent clocks and enable/disable then all when necessary
 */
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/of_address.h>

#include "rzn1-clock.h"

struct rzn1_group {
	struct clk_hw	hw;
	int	used;
	int	nparents;
	struct {
		const char *name;
		struct clk *clk;
	} parent[0];
};

#define to_clk_group(_hw) container_of(_hw, struct rzn1_group, hw)

static int clk_group_enable(struct clk_hw *hw)
{
	struct rzn1_group *group = to_clk_group(hw);
	int i;

	group->used = 1;

	for (i = 1; i < group->nparents; i++) {
		if (!group->parent[i].clk) {
			group->parent[i].clk =
				__clk_lookup(group->parent[i].name);
			if (group->parent[i].clk)
				clk_prepare(group->parent[i].clk);
		}
		if (group->parent[i].clk)
			clk_enable(group->parent[i].clk);
	}

	return 0;
}

static void clk_group_disable(struct clk_hw *hw)
{
	struct rzn1_group *group = to_clk_group(hw);
	int i;

	/* Do not disable clock groups that have not been initialised. This is
	 * because we will end up decrementing the use count for clocks that
	 * are shared by multiple IPs */
	if (!group->used)
		return;

	for (i = 1; i < group->nparents; i++) {
		if (!group->parent[i].clk) {
			group->parent[i].clk =
				__clk_lookup(group->parent[i].name);
			if (group->parent[i].clk)
				clk_prepare(group->parent[i].clk);
		}
		/* Do not clk_disable() unless the clock has been
		 * enabled, otherwise the clk subsystem will BUG() */
		if (group->parent[i].clk)
			if (__clk_get_enable_count(group->parent[i].clk) > 0)
				clk_disable(group->parent[i].clk);
	}
}

static int clk_group_is_enabled(struct clk_hw *hw)
{
	struct rzn1_group *group = to_clk_group(hw);
	struct clk *parent = clk_get_parent(group->hw.clk);

	return __clk_is_enabled(parent);
}

static const struct clk_ops clk_group_ops = {
	.enable = clk_group_enable,
	.disable = clk_group_disable,
	.is_enabled = clk_group_is_enabled,
};

static void __init rzn1_clock_group_init(struct device_node *node)
{
	const char *clk_name = node->name;
	struct clk *clk;
	int nparents  = 0;
	int i;
	struct rzn1_group *group = NULL;
	struct clk_init_data init;

	nparents = of_count_phandle_with_args(node, "clocks", "#clock-cells");
	pr_devel("%s %s %d parents\n", __func__, node->name, nparents);

	group = kzalloc(sizeof(struct rzn1_group) +
			(sizeof(group->parent[0]) * nparents),
			GFP_KERNEL);
	if (!group) {
		pr_err("%s failed to allocate %s(%d)\n", __func__,
			node->name, nparents);
		return;
	}
	for (i = 0; i < nparents; i++)
		group->parent[i].name = of_clk_get_parent_name(node, i);
	group->nparents = nparents;

	init.name = clk_name;
	init.ops = &clk_group_ops;
	init.flags = CLK_IS_BASIC | CLK_SET_RATE_PARENT;
	/* There is only own 'true' parent here, the other ones are ghosts */
	init.parent_names = &group->parent[0].name;
	init.num_parents = 1;
	group->hw.init = &init;

	clk = clk_register(NULL, &group->hw);

	if (!IS_ERR(clk))
		of_clk_add_provider(node, of_clk_src_simple_get, clk);

}

CLK_OF_DECLARE(rzn1_group_clk,
		"renesas,rzn1-clock-group", rzn1_clock_group_init);
