/*
 * Renesas RZ/N1 RGMII/GMII Converter (MIIC) driver.
 * Copyright (C) 2016-2017 Renesas Electronics Europe Ltd
 *
 * Each ethernet PHY interface goes through a RGMII/GMII Converter. This piece
 * of hardware needs to know about the interface type, speed and duplex.
 * The hardware documentation for the RGMII/GMII Converters numbers them 1 to 5,
 * however all RZ/N1 software uses 0 based indexes. Therefore the phy_nr field
 * is numbered 0 to 4.
 *
 * This driver also sets the SwitchCore registers with the correct speed and
 * duplex since these are also part of the RIN registers.
 *
 * Since all of these registers are part of the RIN SwitchCore, this driver also
 * sets the RIN operation mode, if it is specified in the dtb, i.e. which IPs
 * are multiplexed to the PHYs.
 *
 * WARNING:
 * The code running on the Cortex M3 also writes to R-In Engine registers. This
 * works because Linux and the Cortex M3 are modifying *different* registers.
 * However, there is one register (CONVRST) where Linux and the Cortex M3 write
 * to the same register, but different bits in it. This works only because we
 * assume that the Cortex M3 writes to CONVRST during start up, before Linux has
 * been started.
 *
 * However, there is a complication. There are regs that are write protected,
 * and unprotected by writing a specific sequence to the PRCMD register. Since
 * the Cortex M3 also needs to use this feature, we have to assume that writing
 * to write protected registers may fail. Therefore, we re-try until the bits we
 * are writing stick.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/etherdevice.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/of_platform.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

/* RIN Ether Accessory (Switch Control) regs */
#define PRCMD			0x0		/* Ethernet Protect */
#define IDCODE			0x4		/* EtherSwitch IDCODE */
#define MODCTRL			0x8		/* Mode Control */
#define PTP_MODE_CTRL		0xc		/* PTP Mode Control */
#define PHY_LINK_MODE		0x14		/* Ethernet PHY Link Mode */

/* RIN RGMII/RMII Converter regs */
#define CONVCTRL(x)		(0x100 + (x) * 4) /* RGMII/RMII Converter */
#define  CONVCTRL_10_MBPS		0
#define  CONVCTRL_100_MBPS		BIT(0)
#define  CONVCTRL_1000_MBPS		BIT(1)
#define  CONVCTRL_MII			0
#define  CONVCTRL_RMII			BIT(2)
#define  CONVCTRL_RGMII			BIT(3)
#define  CONVCTRL_REF_CLK_OUT		BIT(4)
#define  CONVCTRL_HALF_DUPLEX		0
#define  CONVCTRL_FULL_DUPLEX		BIT(8)
#define CONVRST			0x114		/* RGMII/RMII Converter RST */
#define  PHYIF_RST(x)			BIT(x)

/* RIN SwitchCore regs */
#define RIN_SWCTRL		0x304		/* SwitchCore Control */
#define  RIN_SWCTRL_MPBS_10(x)		(((0 << 4) | (1 << 0)) << (x))
#define  RIN_SWCTRL_MPBS_100(x)		(((0 << 4) | (0 << 0)) << (x))
#define  RIN_SWCTRL_MPBS_1000(x)	(((1 << 4) | (0 << 0)) << (x))
#define  RIN_SWCTRL_MPBS_MASK(x)	(((1 << 4) | (1 << 0)) << (x))
#define RIN_SWDUPC		0x308		/* SwitchCore Duplex Mode */
#define  RIN_SWDUPC_DUPLEX_MASK(x)	BIT(x)
#define  RIN_SWDUPC_DUPLEX_FULL(x)	BIT(x)

#define MIIC_MAX_NR_PORTS 5

struct rzn1_miic_port {
	struct rzn1_miic *miic;
	int phy_nr;
	phy_interface_t phy_if;
	struct phy_device *phy_dev;
	int speed;
	int duplex;
};

struct rzn1_miic {
	struct device *dev;
	void __iomem *regs;
	struct clk *clk;
	struct rzn1_miic_port *ports[MIIC_MAX_NR_PORTS];
};

static void rin_writel_to_prot_bits(struct rzn1_miic *miic, u32 reg, u32 new_bits, u32 mask)
{
	int tries = 5;
	u32 val;
	u8 rand;

	do {
		/* RIN: Enable protection */
		writel(0x0000, miic->regs + PRCMD);

		/* RIN: Unprotect register writes */
		writel(0x00a5, miic->regs + PRCMD);
		writel(0x0001, miic->regs + PRCMD);
		writel(0xfffe, miic->regs + PRCMD);
		writel(0x0001, miic->regs + PRCMD);

		/* Read, modify, write */
		val = readl(miic->regs + reg);
		val &= ~mask;
		val |= (new_bits & mask);
		writel(val, miic->regs + reg);

		/* Enable protection */
		writel(0x0000, miic->regs + PRCMD);

		/* Has the write taken place? */
		val = readl(miic->regs + reg);
		if ((val & mask) == (new_bits & mask))
			break;

		/* Try to give the Cortex M3 a chance to do it */
		get_random_bytes(&rand, 1);
		udelay(rand & 0xf);

	} while (tries--);

	if (!tries)
		dev_warn(miic->dev, "Failed write to protected R-In register!\n");
}

void rzn1_switchcore_adjust(struct rzn1_miic_port *port, int duplex, int speed)
{
	struct rzn1_miic *miic = port->miic;
	/* This reg use switch port numbers, not phy interface numbers! */
	int port_nr = 4 - port->phy_nr;
	u32 val;

	/* We only handle speed/duplex changes on the switch ports */
	if (port_nr > 3)
		return;

	/* speed */
	val = readl(miic->regs + RIN_SWCTRL);
	val &= ~RIN_SWCTRL_MPBS_MASK(port_nr);

	if (speed == SPEED_1000)
		val |= RIN_SWCTRL_MPBS_1000(port_nr);
	else if (speed == SPEED_100)
		val |= RIN_SWCTRL_MPBS_100(port_nr);
	else
		val |= RIN_SWCTRL_MPBS_10(port_nr);


	rin_writel_to_prot_bits(miic, RIN_SWCTRL, val, RIN_SWCTRL_MPBS_MASK(port_nr));

	/* duplex */
	val = 0;
	if (duplex == DUPLEX_FULL)
		val = RIN_SWDUPC_DUPLEX_FULL(port_nr);

	rin_writel_to_prot_bits(miic, RIN_SWDUPC, val, RIN_SWDUPC_DUPLEX_MASK(port_nr));
}

static void rzn1_miic_adjust(struct rzn1_miic_port *port, int duplex, int speed)
{
	struct rzn1_miic *miic = port->miic;
	u32 val = 0;

	val = readl(miic->regs + CONVCTRL(port->phy_nr));

	/* Keep the MII interface type bits */
	val &= CONVCTRL_MII | CONVCTRL_RMII | CONVCTRL_RGMII |
		CONVCTRL_REF_CLK_OUT;

	/* The interface type and speed bits are somewhat intertwined */
	if (val != CONVCTRL_MII) {
		if (speed == SPEED_1000)
			val |= CONVCTRL_1000_MBPS;
		else if (speed == SPEED_100)
			val |= CONVCTRL_100_MBPS;
		else if (speed == SPEED_10)
			val |= CONVCTRL_10_MBPS;
	}

	if (duplex == DUPLEX_FULL)
		val |= CONVCTRL_FULL_DUPLEX;

	writel(val, miic->regs + CONVCTRL(port->phy_nr));

	port->speed = speed;
	port->duplex = duplex;
}

static void rzn1_miic_link_adjust(void *priv)
{
	struct rzn1_miic_port *port = priv;
	struct rzn1_miic *miic = port->miic;
	struct phy_device *phydev = port->phy_dev;

	if (phydev->link &&
	    ((port->speed != phydev->speed) ||
	    (port->duplex != phydev->duplex))) {
		rzn1_miic_adjust(port, phydev->duplex, phydev->speed);
		rzn1_switchcore_adjust(port, phydev->duplex, phydev->speed);
		dev_info(miic->dev, "RGMII/GMII Converter %d link up, %dMbps\n",
			 port->phy_nr, phydev->speed);
	}
}

static void rzn1_miic_setup(struct rzn1_miic_port *port, int duplex, int speed,
			    int rmii_ref_clk_out)
{
	struct rzn1_miic *miic = port->miic;
	u32 val = 0;

	dev_dbg(miic->dev, "Setup RGMII/GMII Converter\n");

	switch (port->phy_if) {
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_RGMII_ID:
	case PHY_INTERFACE_MODE_RGMII_RXID:
	case PHY_INTERFACE_MODE_RGMII_TXID:
		val |= CONVCTRL_RGMII;
		break;
	case PHY_INTERFACE_MODE_RMII:
		val |= CONVCTRL_RMII;
		break;
	case PHY_INTERFACE_MODE_MII:
		val |= CONVCTRL_MII;
		break;
	default:
		dev_err(miic->dev, "Unsupported PHY %d, type %d\n",
			port->phy_nr, port->phy_if);
		return;
	}

	if (port->phy_if == PHY_INTERFACE_MODE_RMII && rmii_ref_clk_out)
		val |= CONVCTRL_REF_CLK_OUT;

	writel(val, miic->regs + CONVCTRL(port->phy_nr));

	rzn1_miic_adjust(port, duplex, speed);

	/* reset */
	rin_writel_to_prot_bits(miic, CONVRST, 0, PHYIF_RST(port->phy_nr));
	usleep_range(1000, 2000);
	val = PHYIF_RST(port->phy_nr);
	rin_writel_to_prot_bits(miic, CONVRST, val, PHYIF_RST(port->phy_nr));
}

/*
 * Find a parent node that will have a driver attached to it.
 * This has to cope with "snps,dwmac-mdio" nodes that do not have a driver but
 * do have a compatible prop but without a status prop. Unfortunately, the
 * of_device_is_available() func assumes that a missing status prop is the same
 * as status=okay.
 */
static struct device_node *get_parent_node(struct device_node *np)
{
	struct property *prop, *stat;

	while (np) {
		prop = of_find_property(np, "compatible", NULL);
		stat = of_find_property(np, "status", NULL);

		if (prop && stat && of_device_is_available(np))
			return np;

		np = of_get_next_parent(np);
	}

	return NULL;
}

static int rzn1_miic_setup_ports(struct rzn1_miic *miic, struct device *dev)
{
	struct device_node *node = dev->of_node;
	struct device_node *port_node;
	int nr_ports, i;
	int phy_nr = 0;
	u32 prop;

	if (!node)
		return -EINVAL;

	nr_ports = of_get_child_count(node);
	if (nr_ports > MIIC_MAX_NR_PORTS) {
		dev_err(dev, "%s has too many PHY ports\n", node->full_name);
		return -EINVAL;
	}
	dev_info(dev, "nr phy ports %d\n", nr_ports);

	if (of_property_read_u32(node, "mode_control", &prop)) {
		dev_warn(dev, "%s missing mode_control property\n",
			node->full_name);
	} else {
		/* RIN: Mode Control */
		rin_writel_to_prot_bits(miic, MODCTRL, prop, ~0);
	}


	/* Each port node in DT directly corresponds to the PHY MIIs, i.e. the
	 * order of the nodes must match the MIIs and if you are not using one,
	 * an empty node must still be present.
	 */
	for_each_child_of_node(node, port_node) {
		struct rzn1_miic_port *port;
		struct device_node *phy_node;
		struct phy_device *phy_dev;
		struct platform_device *parent_dev;
		int mode;

		phy_nr++;

		/* If there is a phandle for the phy, go look there */
		phy_node = of_parse_phandle(port_node, "phy-handle", 0);
		if (!phy_node)
			phy_node = port_node;

		/* Do not directly assign to something that is of type
		 * phy_interface_t as that is unsigned
		 */
		mode = of_get_phy_mode(phy_node);
		if (mode < 0) {
			dev_err(dev, "%s phy-mode is not in your DT\n",
				phy_node->full_name);
			continue;
		}

		phy_dev = of_phy_find_device(phy_node);
		if (IS_ERR(phy_dev)) {
			dev_err(dev, "%s is not a phy device\n",
				phy_node->full_name);
			continue;
		}

		/*
		 * In a simple world, if there is no PHY device, we could return
		 * EPROBE_DEFER because we know the PHY hasn't yet been probed.
		 * However, we would like the driver to work where a dtb for a
		 * board with a plug-on board that add extra PHYs, also works
		 * without the plug-on board.
		 * Test if the PHY should have been probed by now by seeing if
		 * the PHYs parent node has a device associated with it.
		 */
		parent_dev = of_find_device_by_node(get_parent_node(phy_node));
		if (!parent_dev || !platform_get_drvdata(parent_dev)) {
			dev_info(dev, "%s phy parent has not been probed yet\n",
				 phy_node->full_name);
			return -EPROBE_DEFER;
		}

		if (!phy_dev) {
			dev_info(dev, "%s phy is not physically present\n",
				 phy_node->full_name);
			continue;
		}

		port = devm_kzalloc(dev, sizeof(*port), GFP_KERNEL);
		if (!port)
			return -ENOMEM;

		port->miic = miic;
		port->phy_nr = phy_nr - 1;
		port->phy_if = mode;
		port->phy_dev = phy_dev;
		miic->ports[port->phy_nr] = port;

		dev_info(dev, "%s %s\n", port_node->full_name, phy_modes(mode));

		/* Initialise the MIIC, the speed & duplex doesn't really
		 * matter initially as we'll get a callback when the link is up,
		 * but setting up the interface type needs to be done now
		 */
		if (of_property_read_bool(port_node, "ref-clk-out"))
			rzn1_miic_setup(port, DUPLEX_FULL, SPEED_1000, 1);
		else
			rzn1_miic_setup(port, DUPLEX_FULL, SPEED_1000, 0);
	}

	for (i = 0; i < nr_ports; i++) {
		struct rzn1_miic_port *port = miic->ports[i];

		if (!port)
			continue;

		/* hook into the phy */
		phy_pre_prepare_link(port->phy_dev, port,
				     &rzn1_miic_link_adjust);
	}

	return 0;
}

static int rzn1_miic_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rzn1_miic *miic;
	struct resource *res;

	miic = devm_kzalloc(dev, sizeof(struct rzn1_miic), GFP_KERNEL);
	miic->dev = dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	miic->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(miic->regs))
		return PTR_ERR(miic->regs);

	miic->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(miic->clk)) {
		dev_err(dev, "failed to get device clock\n");
		return PTR_ERR(miic->clk);
	}
	clk_prepare_enable(miic->clk);

	return rzn1_miic_setup_ports(miic, dev);
}

static const struct of_device_id rzn1_miic_match[] = {
	{ .compatible = "renesas,rzn1-miic" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rzn1_miic_match);

static struct platform_driver rzn1_miic_driver = {
	.probe  = rzn1_miic_probe,
	.driver = {
		.name           = "rzn1-miic",
		.of_match_table = rzn1_miic_match,
	},
};
module_platform_driver(rzn1_miic_driver);

MODULE_AUTHOR("Phil Edworthy <phil.edworthy@renesas.com>");
MODULE_DESCRIPTION("Renesas RZ/N1 RGMII/GMII Converter");
MODULE_LICENSE("GPL");
