// SPDX-License-Identifier: GPL-2.0+
/*
 * MSEBI driver for Renesas RZ/N1 chips
 * Copyright (C) 2015 Renesas Electronics Europe Ltd.
 *
 * Based on imx-weim driver
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 */
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/of_device.h>

#define MSEBI_MAX_CS 4
#define MSEBI_NR_TIMING_REGS 3
#define MSEBI_CS_STRIDE 0x100
#define MSEBI_CS_CYCLESIZE_REG	0x00
#define MSEBI_CS_SETUPHOLD_REG	0x04
#define MSEBI_CS_CONFIG_REG	0x60
#define MSEBI_CONFIG_REG	0x800
#define MSEBI_CONFIG_DIRBUF_REG	0x804

static const struct of_device_id msebi_id_table[] = {
	{ .compatible = "renesas,rzn1-msebi", },
	{ }
};
MODULE_DEVICE_TABLE(of, msebi_id_table);

/* Parse and set the timing for this device. */
static int __init msebi_timing_setup(struct device_node *np, void __iomem *base)
{
	u32 cs_idx, value[MSEBI_NR_TIMING_REGS];
	int ret;

	/* get the CS index from this child node's "reg" property. */
	ret = of_property_read_u32(np, "reg", &cs_idx);
	if (ret)
		return ret;

	if (cs_idx >= MSEBI_MAX_CS)
		return -EINVAL;

	ret = of_property_read_u32_array(np, "renesas,msebi-cs-timing",
					 value, MSEBI_NR_TIMING_REGS);
	if (ret)
		return ret;

	/* set the timing */
	writel(value[0], base + cs_idx * MSEBI_CS_STRIDE + MSEBI_CS_CYCLESIZE_REG);
	writel(value[1], base + cs_idx * MSEBI_CS_STRIDE + MSEBI_CS_SETUPHOLD_REG);
	writel(value[2], base + cs_idx * MSEBI_CS_STRIDE + MSEBI_CS_CONFIG_REG);

	return 0;
}

static int __init msebi_parse_dt(struct platform_device *pdev,
				 void __iomem *base)
{
	struct device_node *child;
	int ret, have_child = 0;
	u32 cfg;

	ret = of_property_read_u32(pdev->dev.of_node, "renesas,msebi-config", &cfg);
	if (ret)
		return ret;
	writel(cfg, base + MSEBI_CONFIG_REG);

	ret = of_property_read_u32(pdev->dev.of_node, "renesas,msebi-config-dirbuf", &cfg);
	if (ret)
		return ret;
	writel(cfg, base + MSEBI_CONFIG_DIRBUF_REG);

	for_each_child_of_node(pdev->dev.of_node, child) {
		if (!child->name)
			continue;

		ret = msebi_timing_setup(child, base);
		if (ret)
			dev_warn(&pdev->dev, "%s set timing failed.\n",
				 child->full_name);
		else
			have_child = 1;
	}

	if (have_child)
		ret = of_platform_populate(pdev->dev.of_node,
					   of_default_bus_match_table,
				   NULL, &pdev->dev);
	if (ret)
		dev_err(&pdev->dev, "%s fail to create devices.\n",
			pdev->dev.of_node->full_name);
	return ret;
}

static int __init msebi_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct clk *clk;
	void __iomem *base;
	int ret;

	/* get the resource */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	/* get the clock */
	clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	ret = clk_prepare_enable(clk);
	if (ret)
		return ret;

	/* parse the device node */
	ret = msebi_parse_dt(pdev, base);
	if (ret)
		clk_disable_unprepare(clk);
	else
		dev_info(&pdev->dev, "Driver registered.\n");

	return ret;
}

static struct platform_driver msebi_driver = {
	.driver = {
		.name		= "renesas-msebi",
		.of_match_table	= msebi_id_table,
	},
};
module_platform_driver_probe(msebi_driver, msebi_probe);

MODULE_AUTHOR("Renesas Electronics Europe Ltd");
MODULE_DESCRIPTION("RZ/N1 MSEBI Driver");
MODULE_LICENSE("GPL");
