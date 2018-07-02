/*
 * MoreThanIP 5-port switch MDIO driver
 * Copyright (C) 2016 Renesas Electronics Europe Ltd.
 *
 * Based on DaVinci MDIO Module driver
 * Copyright (C) 2010 Texas Instruments.
 *
 * Shamelessly ripped out of davinci_emac.c, original copyrights follow:
 * Copyright (C) 2009 Texas Instruments.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/io.h>
#include <linux/if_ether.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/pinctrl-rzn1.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/if_vlan.h>

#define DRV_NAME			"mt5pt_switch"
#define DRV_VERSION			"0.2"

#define MT5PT_NR_PORTS			4 /* Number of downstream ports */
#define MT5PT_UPSTREAM_PORT_NR		4

#define MDIO_TIMEOUT			100 /* in MDIO clocks */
#define FRM_LENGTH_EXTRA		34
#define MAX_MTU				9190

#define PHY_REG_MASK			0x1f
#define PHY_ID_MASK			0x1f

/* MoreThanIP 5pt Switch regs */
#define MT5PT_REVISION			0x0
#define MT5PT_SCRATCH			0x4
#define MT5PT_PORT_ENA			0x8
#define  MT5PT_PORT_ENA_RX(x)		BIT((x) + 16)
#define  MT5PT_PORT_ENA_TX(x)		BIT(x)
#define  MT5PT_PORT_ENA_TXRX(x)		(MT5PT_PORT_ENA_TX(x) | MT5PT_PORT_ENA_RX(x))
#define MT5PT_AUTH_PORT(x)		(0x240 + (x) * 4)
#define  MT5PT_AUTH_PORT_AUTHORIZED	BIT(0)
#define  MT5PT_AUTH_PORT_CONTROLLED	BIT(1)
#define  MT5PT_AUTH_PORT_EAPOL_EN	BIT(2)
#define  MT5PT_AUTH_PORT_GUEST		BIT(3)
#define  MT5PT_AUTH_PORT_EAPOL_PORT(x)	((x) << 12)
#define MT5PT_LK_CTRL			0x400
#define MT5PT_LK_STATUS			0x404
#define MT5PT_LK_ADDR_CTRL		0x408
#define  MT5PT_LK_PORT_NUM(x)		BIT(x)
#define  MT5PT_LK_PORT_NUM_MASK		0xfff
#define  MT5PT_LK_WAIT_COMPLETE		BIT(27)
#define  MT5PT_LK_DELETE_PORT		BIT(30)
#define  MT5PT_LK_BUSY			BIT(31)
#define MDIO_CFG_STATUS			0x700
#define  MDIO_CFG_STATUS_BUSY		BIT(0)
#define  MDIO_CFG_STATUS_ERR		BIT(1)
#define  MDIO_CFG_MIN_DIV		5
#define  MDIO_CFG_MAX_DIV		511
#define MDIO_COMMAND			0x704
#define  MDIO_COMMAND_READ		BIT(15)
#define MDIO_DATA			0x708

#define MT5PT_MAC_CMD_CFG(x)		(0x808 + (x) * 0x400)
#define  MT5PT_CC_TX_ENA		BIT(0)
#define  MT5PT_CC_RX_ENA		BIT(1)
#define  MT5PT_CC_MBPS_1000		BIT(3)
#define  MT5PT_CC_CNTL_FRM_ENA		BIT(23)
#define MT5PT_MAC_FRM_LENGTH(x)		(0x814 + (x) * 0x400)

#define MT5PT_HUB_CONFIG		0x3e00
#define  MT5PT_HC_HUB_ENA		BIT(0)
#define  MT5PT_HC_RETRANSMIT_ENA	BIT(1)
#define  MT5PT_HC_TRIGGER_MODE		BIT(2)
#define  MT5PT_HC_HUB_ISOLATE		BIT(3)
#define MT5PT_HUB_GROUP			0x3e04
#define  MT5PT_HUB_GROUP_PORT(x)	BIT(x)
#define  MT5PT_HUB_GROUP_MASK		0xf

struct mt5pt_switch_port {
	struct net_device *ndev;
	struct mt5pt_switch *mt5pt;
	int port_nr;
	int phy_addr;
	phy_interface_t phy_if;
	struct phy_device *phy_dev;
	int speed;
	int link;
	bool early_en;
};

struct mt5pt_switch {
	void __iomem	*regs;
	struct clk	*clk;
	struct device	*dev;
	struct mii_bus	*bus;

	unsigned long mdio_bus_freq;
	int nr_ports;
	struct mt5pt_switch_port *ports[PHY_MAX_ADDR];
};

static void mt5pt_switch_port_enable(void __iomem *regs, int port_nr)
{
	u32 val;

	val = readl(regs + MT5PT_AUTH_PORT(port_nr));
	val |= MT5PT_AUTH_PORT_AUTHORIZED;
	writel(val, regs + MT5PT_AUTH_PORT(port_nr));

	val = readl(regs + MT5PT_PORT_ENA);
	val |= MT5PT_PORT_ENA_TXRX(port_nr);
	writel(val, regs + MT5PT_PORT_ENA);
}

/*
 * Remove entries for forwarding lookup table associated to port_nr.
 * Remove only dynamic entries (no static entries), i.e. only entries added in
 * learning mode.
 */
static void mt5pt_switch_fdb_flush_port(void __iomem *regs, int port_nr)
{
	u32 val;

	val = readl(regs + MT5PT_LK_ADDR_CTRL);
	val |= MT5PT_LK_DELETE_PORT | MT5PT_LK_WAIT_COMPLETE;
	val = (val & MT5PT_LK_PORT_NUM_MASK) |
		MT5PT_LK_PORT_NUM(port_nr);
	writel(val, regs + MT5PT_LK_ADDR_CTRL);
}

static void mt5pt_switch_port_disable_rxtx(void __iomem *regs, int port_nr)
{
	u32 val;

	val = readl(regs + MT5PT_PORT_ENA);
	val &= ~MT5PT_PORT_ENA_TXRX(port_nr);
	writel(val, regs + MT5PT_PORT_ENA);
}

static void mt5pt_switch_port_disable(void __iomem *regs, int port_nr)
{
	mt5pt_switch_port_disable_rxtx(regs, port_nr);
	mt5pt_switch_fdb_flush_port(regs, port_nr);
}

static void mt5pt_switch_port_mtu(void __iomem *regs, int port_nr, int mtu)
{
	writel(mtu + FRM_LENGTH_EXTRA, regs + MT5PT_MAC_FRM_LENGTH(port_nr));
}

static void mt5pt_switch_handle_link_change(struct net_device *dev)
{
	struct mt5pt_switch_port *port = netdev_priv(dev);
	struct mt5pt_switch *mt5pt = port->mt5pt;
	struct phy_device *phydev = port->phy_dev;
	u32 val;

	if (phydev->link && (port->speed != phydev->speed)) {
		val = readl(mt5pt->regs + MT5PT_MAC_CMD_CFG(port->port_nr));
		val &= ~MT5PT_CC_MBPS_1000;
		if (phydev->speed == SPEED_1000)
			val |= MT5PT_CC_MBPS_1000;
		val |= MT5PT_CC_CNTL_FRM_ENA;
		writel(val, mt5pt->regs + MT5PT_MAC_CMD_CFG(port->port_nr));

		port->speed = phydev->speed;
	}

	if ((port->link != phydev->link) || (port->speed != phydev->speed))
		phy_print_status(phydev);

	port->link = phydev->link;
}

static void mt5pt_switch_setup_mdio(struct mt5pt_switch *mt5pt)
{
	u32 mdio_in, div, val;

	mdio_in = clk_get_rate(mt5pt->clk);
	div = (mdio_in / 2) / mt5pt->mdio_bus_freq;
	if (div > MDIO_CFG_MAX_DIV)
		div = MDIO_CFG_MAX_DIV;
	if (div < MDIO_CFG_MIN_DIV)
		div = MDIO_CFG_MIN_DIV;
	val = div << 7;

	/* Set MDIO hold time to 1 cpu cycle (min) */
	val |= 0 << 2;

	dev_info(mt5pt->dev, "MDIO clk is %d Hz\n", mdio_in / (2 * div + 1));
	writel(val, mt5pt->regs + MDIO_CFG_STATUS);
}

static int mt5pt_switch_reset(struct mii_bus *bus)
{
	struct mt5pt_switch *mt5pt = bus->priv;
	u32 ver;
	int i;

	mt5pt_switch_setup_mdio(mt5pt);

	/* dump hardware version info */
	ver = readl(mt5pt->regs + MT5PT_REVISION);
	dev_info(mt5pt->dev, "MoreThanIP 5-port switch revision %d\n",
		 ver & 0xffff);

	/* Disable all downstream ports */
	for (i = 0; i < MT5PT_NR_PORTS; i++) {
		mt5pt_switch_port_disable(mt5pt->regs, i);
		mt5pt_switch_port_mtu(mt5pt->regs, i, MAX_MTU);
	}

	/* Enable upstream port */
	mt5pt_switch_port_enable(mt5pt->regs, MT5PT_UPSTREAM_PORT_NR);
	mt5pt_switch_port_mtu(mt5pt->regs, MT5PT_UPSTREAM_PORT_NR, MAX_MTU);

	return 0;
}

/* wait until hardware is ready for another user access */
static inline int mt5pt_switch_wait_for_mdio(struct mt5pt_switch *mt5pt)
{
	unsigned long timeout;
	u32 reg;

	/* divide by 10 because we udelay for 10us */
	timeout = ((MDIO_TIMEOUT * 1000000) / mt5pt->mdio_bus_freq) / 10;

	while (timeout--) {
		reg = readl(mt5pt->regs + MDIO_CFG_STATUS);
		if ((reg & MDIO_CFG_STATUS_BUSY) == 0)
			return 0;

		udelay(10);
	}

	return -ETIMEDOUT;
}

static int mt5pt_switch_read(struct mii_bus *bus, int phy_id, int phy_reg)
{
	struct mt5pt_switch *mt5pt = bus->priv;
	u32 reg;
	int ret;

	if (phy_reg & ~PHY_REG_MASK || phy_id & ~PHY_ID_MASK)
		return -EINVAL;

	reg = MDIO_COMMAND_READ | (phy_id << 5) | phy_reg;

	ret = mt5pt_switch_wait_for_mdio(mt5pt);
	if (ret < 0)
		return ret;

	writel(reg, mt5pt->regs + MDIO_COMMAND);

	ret = mt5pt_switch_wait_for_mdio(mt5pt);
	if (ret < 0)
		return ret;

	ret = readl(mt5pt->regs + MDIO_DATA);

	reg = readl(mt5pt->regs + MDIO_CFG_STATUS);
	if (reg & MDIO_CFG_STATUS_ERR)
		return -EIO;

	dev_dbg(mt5pt->dev, "phy[%d] reg%d=0x%x\n", phy_id, phy_reg, ret);
	return ret;
}

static int mt5pt_switch_write(struct mii_bus *bus, int phy_id,
			      int phy_reg, u16 phy_data)
{
	struct mt5pt_switch *mt5pt = bus->priv;
	u32 reg;
	int ret;

	if (phy_reg & ~PHY_REG_MASK || phy_id & ~PHY_ID_MASK)
		return -EINVAL;
	dev_dbg(mt5pt->dev, "phy[%d] reg%d=0x%x\n", phy_id, phy_reg, phy_data);

	reg = (phy_id << 5) | phy_reg;

	ret = mt5pt_switch_wait_for_mdio(mt5pt);
	if (ret < 0)
		return ret;

	writel(reg, mt5pt->regs + MDIO_COMMAND);
	writel(phy_data, mt5pt->regs + MDIO_DATA);

	/* Wait for it to complete */
	ret = mt5pt_switch_wait_for_mdio(mt5pt);

	return ret;
}

static int mt5pt_switch_port_open(struct net_device *dev)
{
	struct mt5pt_switch_port *port = netdev_priv(dev);
	struct mt5pt_switch *mt5pt = port->mt5pt;
	struct phy_device *phydev = port->phy_dev;

	dev_info(mt5pt->dev, "open port %c\n", 'A' + port->port_nr);

	/* if the phy is not yet registered, retry later */
	if (!phydev)
		return -EAGAIN;

	mt5pt_switch_port_enable(mt5pt->regs, port->port_nr);
	dev->mtu = MAX_MTU;

	/* schedule a link state check */
	phy_start(phydev);

	return 0;
}

static int mt5pt_switch_port_init(struct net_device *dev)
{
	struct mt5pt_switch_port *port = netdev_priv(dev);
	int ret = 0;

	if (port->early_en) {
		netif_device_attach(dev);
		dev_open(dev);
	}

	return ret;
}

static int mt5pt_switch_port_stop(struct net_device *dev)
{
	struct mt5pt_switch_port *port = netdev_priv(dev);
	struct mt5pt_switch *mt5pt = port->mt5pt;

	dev_info(mt5pt->dev, "stop port %c\n", 'A' + port->port_nr);

	mt5pt_switch_port_disable(mt5pt->regs, port->port_nr);

	return 0;
}

static int mt5pt_switch_port_ioctl(struct net_device *dev, struct ifreq *ifrq, int cmd)
{
	struct mt5pt_switch_port *port = netdev_priv(dev);
	struct mt5pt_switch *mt5pt = port->mt5pt;
	struct phy_device *phydev = port->phy_dev;

	dev_dbg(mt5pt->dev, "ioctl port %c\n", 'A' + port->port_nr);

	return phy_mii_ioctl(phydev, ifrq, cmd);
}

static netdev_tx_t mt5pt_switch_port_start_xmit(struct sk_buff *skb,
						struct net_device *dev)
{
	struct mt5pt_switch_port *port = netdev_priv(dev);
	struct mt5pt_switch *mt5pt = port->mt5pt;

	dev_info(mt5pt->dev, "Switch devices can't send data, use the GMAC\n");

	dev->stats.tx_dropped++;
	netif_stop_queue(dev);

	return NETDEV_TX_BUSY;
}

static int mt5pt_switch_port_change_mtu(struct net_device *dev, int new_mtu)
{
	struct mt5pt_switch_port *port = netdev_priv(dev);
	struct mt5pt_switch *mt5pt = port->mt5pt;

	if (new_mtu + FRM_LENGTH_EXTRA > MAX_MTU)
		return -EINVAL;

	mt5pt_switch_port_mtu(mt5pt->regs, port->port_nr, new_mtu);
	dev->mtu = new_mtu;

	return 0;
}

static const struct net_device_ops mt5pt_switch_netdev_ops = {
	/* When using an NFS rootfs, we can only bring up the GMAC behind the
	 * upstream port. So we need ndo_init to enable the Switch's downstream
	 * ports ("swport-early-enable" DT prop) and kick the PHY.
	 */
	.ndo_init		= mt5pt_switch_port_init,
	.ndo_open		= mt5pt_switch_port_open,
	.ndo_stop		= mt5pt_switch_port_stop,
	.ndo_do_ioctl		= mt5pt_switch_port_ioctl,
	.ndo_start_xmit		= mt5pt_switch_port_start_xmit,
	.ndo_change_mtu		= mt5pt_switch_port_change_mtu,
};

static void netdev_get_drvinfo(struct net_device *dev,
			       struct ethtool_drvinfo *info)
{
	strlcpy(info->driver, DRV_NAME, sizeof(info->driver));
	strlcpy(info->version, DRV_VERSION, sizeof(info->version));
}

static int netdev_get_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct mt5pt_switch_port *port = netdev_priv(dev);

	return phy_ethtool_gset(port->phy_dev, cmd);
}

static int netdev_set_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct mt5pt_switch_port *port = netdev_priv(dev);

	return phy_ethtool_sset(port->phy_dev, cmd);
}

static const struct ethtool_ops netdev_ethtool_ops = {
	.get_drvinfo		= netdev_get_drvinfo,
	.get_settings		= netdev_get_settings,
	.set_settings		= netdev_set_settings,
	.get_link		= ethtool_op_get_link,
	.get_ts_info		= ethtool_op_get_ts_info,
};

static int mt5pt_switch_remove(struct platform_device *pdev)
{
	struct mt5pt_switch *mt5pt = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < MT5PT_NR_PORTS; i++) {
		struct mt5pt_switch_port *port = mt5pt->ports[i];

		if (port->ndev) {
			unregister_netdev(port->ndev);
			free_netdev(port->ndev);
		}
	}

	if (mt5pt->bus)
		mdiobus_unregister(mt5pt->bus);

	return 0;
}

static void mt5pt_switch_reset_phy(struct platform_device *pdev)
{
	int err, phy_reset;
	int msec = 1;
	struct device_node *np = pdev->dev.of_node;

	if (!np)
		return;

	of_property_read_u32(np, "phy-reset-duration", &msec);

	phy_reset = of_get_named_gpio(np, "phy-reset-gpios", 0);
	if (!gpio_is_valid(phy_reset))
		return;

	err = devm_gpio_request_one(&pdev->dev, phy_reset,
				    GPIOF_OUT_INIT_LOW, "phy-reset");
	if (err) {
		dev_err(&pdev->dev, "failed to get phy-reset-gpios: %d\n", err);
		return;
	}
	msleep(msec);
	gpio_set_value(phy_reset, 1);
}

static int mt5pt_switch_probe_dt(struct mt5pt_switch *mt5pt, struct device *dev)
{
	struct device_node *node = dev->of_node;
	struct device_node *port_node;
	u32 prop;
	int nr_ports;
	int port_nr = 0;

	if (!node)
		return -EINVAL;

	if (of_property_read_u32(node, "bus_freq", &prop)) {
		dev_err(dev, "Missing bus_freq property in DT\n");
		return -EINVAL;
	}
	mt5pt->mdio_bus_freq = prop;

	nr_ports = of_get_child_count(node);
	if (nr_ports != MT5PT_NR_PORTS) {
		dev_err(dev, "%s needs %d ports\n", node->full_name,
			MT5PT_NR_PORTS);
		return -EINVAL;
	}

	for_each_child_of_node(node, port_node) {
		struct net_device *ndev;
		struct mt5pt_switch_port *port;
		struct device_node *phy_node;
		int phy_mode;
		int phy_addr;

		/* If there is a phandle for the phy, go look there */
		phy_node = of_parse_phandle(port_node, "phy-handle", 0);
		if (!phy_node)
			phy_node = port_node;

		/* Do not directly assign to data->slave[i].phy_if as
		 * phy_interface_t type is unsigned
		 */
		phy_mode = of_get_phy_mode(phy_node);
		if (phy_mode < 0) {
			dev_warn(dev, "%s missing phy-mode, skipping port\n",
				 phy_node->full_name);
			port_nr++;
			continue;
		}

		phy_addr = of_mdio_parse_addr(dev, phy_node);
		if (phy_addr < 0) {
			dev_warn(dev, "%s missing phy addr, skipping port\n",
				 phy_node->full_name);
			port_nr++;
			continue;
		}

		ndev = alloc_etherdev(sizeof(struct mt5pt_switch_port));
		if (!ndev)
			return -ENOMEM;

		SET_NETDEV_DEV(ndev, dev);

		port = netdev_priv(ndev);

		mt5pt->ports[port_nr] = port;
		port->ndev = ndev;
		port->mt5pt = mt5pt;
		port->port_nr = port_nr;
		port->phy_addr = phy_addr;
		port->phy_if = phy_mode;
		port->phy_dev = NULL;
		port->early_en = of_property_read_bool(phy_node,
						       "swport-early-enable");

		port_nr++;
	}

	/* Enable the upstream port */
	mt5pt_switch_port_enable(mt5pt->regs, MT5PT_UPSTREAM_PORT_NR);

	return 0;
}

static int mt5pt_switch_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mt5pt_switch *mt5pt;
	struct resource *res;
	int ret, i;

	mt5pt = devm_kzalloc(dev, sizeof(struct mt5pt_switch), GFP_KERNEL);
	if (!mt5pt)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mt5pt->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(mt5pt->regs))
		return PTR_ERR(mt5pt->regs);

	mt5pt->clk = devm_clk_get(dev, "fck");
	if (IS_ERR(mt5pt->clk)) {
		dev_err(dev, "failed to get device clock\n");
		return PTR_ERR(mt5pt->clk);
	}
	clk_prepare_enable(mt5pt->clk);

	mt5pt->bus = devm_mdiobus_alloc(dev);
	if (!mt5pt->bus) {
		dev_err(dev, "failed to alloc mii bus\n");
		return -ENOMEM;
	}

	if (!dev->of_node) {
		dev_err(dev, "Missing DT node\n");
		return -EINVAL;
	}

	mt5pt_switch_reset_phy(pdev);

	mt5pt_switch_probe_dt(mt5pt, dev);
	snprintf(mt5pt->bus->id, MII_BUS_ID_SIZE, "%s", pdev->name);

	mt5pt->bus->name	= dev_name(dev);
	mt5pt->bus->read	= mt5pt_switch_read,
	mt5pt->bus->write	= mt5pt_switch_write,
	mt5pt->bus->reset	= mt5pt_switch_reset,
	mt5pt->bus->parent	= dev;
	mt5pt->bus->priv	= mt5pt;

	dev_set_drvdata(dev, mt5pt);
	mt5pt->dev = dev;

	/* register the mii bus */
	ret = of_mdiobus_register(mt5pt->bus, dev->of_node);
	if (ret)
		return ret;

	/* attach to the phys */
	for (i = 0; i < MT5PT_NR_PORTS; i++) {
		struct mt5pt_switch_port *port = mt5pt->ports[i];
		struct phy_device *phydev;

		if (!port)
			continue;

		phydev = mdiobus_get_phy(mt5pt->bus, port->phy_addr);
		port->phy_dev = phydev;
		if (!phydev)
			continue;

		dev_info(dev, "phy@%d: %s\n", phydev->mdio.addr,
			 phydev->drv ? phydev->drv->name : "unknown");

		/* attach to the phy */
		ret = phy_connect_direct(port->ndev, phydev,
					 &mt5pt_switch_handle_link_change,
					 port->phy_if);
		if (ret)
			netdev_err(port->ndev, "Could not attach to PHY\n");

		port->ndev->netdev_ops = &mt5pt_switch_netdev_ops;
		port->ndev->ethtool_ops = &netdev_ethtool_ops;

		/* libphy will determine the link state */
		netif_carrier_off(port->ndev);

		/*
		 * Change default net device name to match switchdev naming
		 * convention even it is not based on switchdev driver model
		 * Temporary solution. Must be done later with udev
		 * see switchdev.txt
		 */
		snprintf(port->ndev->name, IFNAMSIZ, "sw%%dp%d", i);
		dev_alloc_name(port->ndev, port->ndev->name);

		if (register_netdev(port->ndev))
			goto err;
	}

	return 0;

err:
	mt5pt_switch_remove(pdev);

	return ret;
}

static const struct of_device_id mt5pt_switch_of_mtable[] = {
	{ .compatible = "mtip,5pt_switch", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, mt5pt_switch_of_mtable);

static struct platform_driver mt5pt_switch_driver = {
	.driver = {
		.name	 = DRV_NAME,
		.of_match_table = of_match_ptr(mt5pt_switch_of_mtable),
	},
	.probe = mt5pt_switch_probe,
	.remove = mt5pt_switch_remove,
};

static int __init mt5pt_switch_init(void)
{
	return platform_driver_register(&mt5pt_switch_driver);
}
device_initcall(mt5pt_switch_init);

static void __exit mt5pt_switch_exit(void)
{
	platform_driver_unregister(&mt5pt_switch_driver);
}
module_exit(mt5pt_switch_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MoreThanIP 5-port switch driver");
