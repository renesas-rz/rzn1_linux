/*
 * Copyright (C) 2014 Renesas Electronics Europe Limited
 *
 * Michel Pollet <michel.pollet@bp.renesas.com>, <buserror@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/of_address.h>

#include "rzn1-clock.h"

static uint32_t ioreg[32];
static int ioregcnt;

struct rzn1_divider {
	struct clk_hw	hw;
	u32 __iomem *reg;
	u16 mask;
	u16 flags, min, max;
	uint8_t table_size;
	u16 table[8];	/* we know there are no more than 8 */
};

#define to_rzn1_divider(_hw) container_of(_hw, struct rzn1_divider, hw)


static unsigned long rzn1_divider_recalc_rate(
	struct clk_hw *hw,
	unsigned long parent_rate)
{
	struct rzn1_divider *clk = to_rzn1_divider(hw);
	long div = readl(clk->reg) & clk->mask;

	pr_devel("%s %ld/%ld (mask %x)\n", __func__,
		parent_rate, div, clk->mask);
	if (div < clk->min)
		div = clk->min;
	else if (div > clk->max)
		div = clk->max;
	return DIV_ROUND_UP(parent_rate, div);
}

/* Attempts to find a value that is in range of min,max,
 * and if a table of set dividers was specified for this
 * register, try to find the fixed divider that is the closest
 * to the target frequency */
static long rzn1_divider_clamp_div(
	struct rzn1_divider *clk,
	unsigned long rate, unsigned long prate)
{
	/* + 1 to cope with rates that have the remainder dropped */
	long div = DIV_ROUND_UP(prate, rate + 1);
	int i;

	if (div <= clk->min)
		return clk->min;
	if (div >= clk->max)
		return clk->max;

	for (i = 0; clk->table_size && i < clk->table_size - 1; i++) {
		if (div >= clk->table[i] && div <= clk->table[i+1]) {
			unsigned long m = rate -
				DIV_ROUND_UP(prate, clk->table[i]);
			unsigned long p =
				DIV_ROUND_UP(prate, clk->table[i + 1]) -
				rate;
			/* select the divider that generates
			 * the value closest to the ideal frequency */
			div = p >= m ? clk->table[i] : clk->table[i + 1];
			return div;
		}
	}
	return div;
}

static long rzn1_divider_round_rate(
	struct clk_hw *hw, unsigned long rate,
	unsigned long *prate)
{
	struct rzn1_divider *clk = to_rzn1_divider(hw);
	long div = DIV_ROUND_UP(*prate, rate);

	pr_devel("%s %s %ld (prate %ld) (wanted div %ld)\n", __func__,
		__clk_get_name(hw->clk), rate, *prate, div);
	pr_devel("   min %d (%ld) max %d (%ld)\n",
		clk->min, DIV_ROUND_UP(*prate, clk->min),
		clk->max, DIV_ROUND_UP(*prate, clk->max));

	div = rzn1_divider_clamp_div(clk, rate, *prate);
	pr_devel("%s %s %ld / %ld = %ld\n", __func__,
		__clk_get_name(hw->clk),
		*prate, div, DIV_ROUND_UP(*prate, div));
	return DIV_ROUND_UP(*prate, div);
}

static int rzn1_divider_set_rate(
	struct clk_hw *hw, unsigned long rate,
	unsigned long parent_rate)
{
	struct rzn1_divider *clk = to_rzn1_divider(hw);
	/* + 1 to cope with rates that have the remainder dropped */
	u32 div = DIV_ROUND_UP(parent_rate, rate + 1);

	pr_devel("%s rate %ld parent %ld div %d\n", __func__,
		rate, parent_rate, div);

	/* Need to write the bit 31 with the divider value to
	 * latch it. Technically we should wait until it has been
	 * cleared too.
	 * TODO: Find wether this callback is sleepable, in case
	 * the hardware /does/ require some sort of spinloop here. */
	writel(div | (1 << 31), clk->reg);

	return 0;
}

static const struct clk_ops rzn1_clk_divider_ops = {
	.recalc_rate = rzn1_divider_recalc_rate,
	.round_rate = rzn1_divider_round_rate,
	.set_rate = rzn1_divider_set_rate,
};

/*
 * This reads an optional table for dividers, the values have to be
 * sorted, and have to be within min and max
 */
static int __init rzn1_read_divider_table(
	struct rzn1_divider *div,
	struct device_node *node)
{
	u32 table_size;
	const __be32 *tablespec;
	int i;

	tablespec = of_get_property(node, "renesas,rzn1-div-table",
					&table_size);
	if (!tablespec)
		return -1;
	table_size /= sizeof(u32);
	if (table_size > ARRAY_SIZE(div->table)) {
		pr_err("%s: %s renesas,rzn1-div-table overflow\n",
				__func__, node->name);
		table_size = ARRAY_SIZE(div->table);
	}
	pr_devel("%s %s renesas,rzn1-div-table size = %d (min %d, max %d)\n",
		__func__, node->name, table_size, div->min, div->max);
	for (i = 0; i < table_size; i++) {
		u32 d;

		of_property_read_u32_index(node, "renesas,rzn1-div-table",
						i, &d);
		pr_devel("  [%d] = %d\n", i, d);

		if ((div->table_size && d <= div->table[div->table_size-1]) ||
			d < div->min || d > div->max) {
			pr_err("%s: %s renesas,rzn1-div-table invalid: %d\n",
					__func__, node->name, d);
			return -EOVERFLOW;
		}
		div->table[div->table_size++] = d;
	}
	return 0;
}

static struct rzn1_divider *_register_divider(
		struct device *dev, const char *name,
		const char *parent_name, unsigned long flags,
		u32 __iomem *reg, u16 mask,
		u16 min, u16 max,
		u8 clk_divider_flags)
{
	struct rzn1_divider *div;
	struct clk *clk;
	struct clk_init_data init;

	/* allocate the divider */
	div = kzalloc(sizeof(struct rzn1_divider), GFP_KERNEL);
	if (!div)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	init.ops = &rzn1_clk_divider_ops;
	init.flags = flags | CLK_IS_BASIC | CLK_SET_RATE_PARENT;
	init.parent_names = parent_name ? &parent_name : NULL;
	init.num_parents = parent_name ? 1 : 0;

	/* struct rzn1_divider assignments */
	div->reg = reg;
	div->mask = mask;
	div->flags = clk_divider_flags;
	div->hw.init = &init;
	div->min = min;
	div->max = max;

	/* register the clock */
	clk = clk_register(dev, &div->hw);

	if (IS_ERR(clk)) {
		kfree(div);
		return ERR_PTR(-ENOMEM);
	}
	return div;
}


void __init rzn1_clock_divider_init(struct device_node *node)
{
	struct rzn1_divider *div;
	const char *clk_name = node->name;
	void __iomem *reg;
	const char *parent_name;
	u8 clk_divider_flags = 0;
	u32 mask = 0, div_offset = 0, min = 1, max = 1;

	parent_name = of_clk_get_parent_name(node, 0);

	if (of_property_read_u32(node, "renesas,rzn1-div-min", &min)) {
		min = 1;
		pr_err("%s: missing renesas,rzn1-div-min property for %s\n",
			__func__, node->name);
	}
	if (of_property_read_u32(node, "renesas,rzn1-div-max", &max)) {
		max = min;
		pr_err("%s: missing renesas,rzn1-div-max property for %s\n",
			__func__, node->name);
	}

	if (of_property_read_u32(node, "reg", &div_offset)) {
		reg = &ioreg[ioregcnt++];
		*((u32 *)reg) = min;

		pr_debug("%s: %s MISSING REG property, faking it\n",
			__func__, node->name);
	} else {
		reg = ((u8 *)rzn1_sysctrl_base()) + div_offset;
	}

	if (of_property_read_u32(node, "renesas,rzn1-bit-mask", &mask)) {
		mask = 0xff;
		pr_err("%s: missing bit-mask property for %s\n", __func__,
			node->name);
	}

	if (of_property_read_bool(node, "renesas,rzn1-index-power-of-two"))
		clk_divider_flags |= CLK_DIVIDER_POWER_OF_TWO;

	pr_devel("%s %s mask %x, val %x\n", __func__,
		node->name, mask, *((u32 *)reg));

	div = _register_divider(NULL, clk_name,
			parent_name, 0,
			reg, mask, min, max,
			clk_divider_flags);

	if (!IS_ERR(div)) {
		if (rzn1_read_divider_table(div, node) == 0)
			of_clk_add_provider(node, of_clk_src_simple_get,
					div->hw.clk);
	}
}

CLK_OF_DECLARE(rzn1_divider_clk,
	"renesas,rzn1-clock-divider", rzn1_clock_divider_init);

