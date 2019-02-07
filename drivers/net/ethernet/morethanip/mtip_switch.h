/*
 * MoreThanIP 5-port switch MDIO driver
 * Copyright (C) 2016 Renesas Electronics Europe Ltd.
 * Copyright (C) 2018 Schneider Electric.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _MTIP_SWITCH_H_
#define _MTIP_SWITCH_H_

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
#define  MT5PT_PORT_ENA_TXRX(x)		(MT5PT_PORT_ENA_TX(x) | \
					 MT5PT_PORT_ENA_RX(x))
#define MT5PT_AUTH_PORT(x)		(0x240 + (x) * 4)
#define  MT5PT_AUTH_PORT_AUTHORIZED	BIT(0)
#define  MT5PT_AUTH_PORT_CONTROLLED	BIT(1)
#define  MT5PT_AUTH_PORT_EAPOL_EN	BIT(2)
#define  MT5PT_AUTH_PORT_GUEST		BIT(3)
#define  MT5PT_AUTH_PORT_EAPOL_PORT(x)	((x) << 12)
#define MT5PT_LK_CTRL			0x400
#define  MT5PT_LK_CLR_TBL		BIT(6)
#define MT5PT_LK_STATUS			0x404
#define MT5PT_LK_ADDR_CTRL		0x408
#define  MT5PT_LK_PORT_NUM(x)		BIT(x)
#define  MT5PT_LK_PORT_NUM_MASK		0xfff
#define  MT5PT_LK_WR			BIT(25)
#define  MT5PT_LK_RD			BIT(26)
#define  MT5PT_LK_WAIT_COMPLETE		BIT(27)
#define  MT5PT_LK_DELETE_PORT		BIT(30)
#define  MT5PT_LK_BUSY			BIT(31)
#define  MT5PT_LK_ADDR_MASK		0x1fff
#define MT5PT_LK_DATA_LO		0x40C
#define MT5PT_LK_DATA_HI		0x410
#define MT5PT_SYSTEM_TAGINFO_PN(x)	(0x200 + (4 * (x)))
#define MT5PT_VL_IN_MODE		0x28
#define  MT5PT_VL_IN_MODE_MASK_PN(x)	(0x3 << ((x) * 2))
#define  MT5PT_VL_IN_MODE_SHIFT_PN(x)	((x) * 2)
#define MT5PT_VL_OUT_MODE		0x2c
#define  MT5PT_VL_OUT_MODE_MASK_PN(x)	(0x3 << ((x) * 2))
#define  MT5PT_VL_OUT_MODE_SHIFT_PN(x)	((x) * 2)
#define MT5PT_VL_VERIFY			0x10
#define  MT5PT_VL_VERIFY_PN(x)		BIT(x)
#define  MT5PT_VL_VERIFY_DISCARD_UNK_TAG_PN(x)	BIT(16 + (x))
#define MT5PT_VL_IN_MODE_ENA		0x30
#define  MT5PT_VL_IN_MODE_ENA_PN(x)	BIT(x)
#define MT5PT_VLTBL_BASE		0x280
#define  MT5PT_VLTBL_SIZE		(32 * 4)
#define  MT5PT_VLTBL_VID_MASK		0xfff
#define  MT5PT_VLTBL_VID_SHIFT		5
#define  MT5PT_VLTBL_WR_PORTMASK	BIT(30)
#define  MT5PT_VLTBL_WR_TAGMASK		BIT(29)
#define  MT5PT_VLTBL_RD_TAGMASK		BIT(28)
#define  MT5PT_VLTBL_PORT(x)		BIT(x)
#define  MT5PT_VLTBL_PORT_MASK		0x1f
#define MT5PT_UCAST_DEFAULT_MASK	0xc
#define MT5PT_BCAST_DEFAULT_MASK	0x14
#define MT5PT_MCAST_DEFAULT_MASK	0x18
#define MT5PT_MGMT_CFG			0x20
#define  MT5PT_MGMT_CFG_PORT_MASK	0xf
#define  MT5PT_MGMT_CFG_EN_BPDU		BIT(6)
#define  MT5PT_MGMT_CFG_DIS_BPDU	BIT(7)
#define  MT5PT_MGMT_CFG_PORTMASK_PN(x)	BIT((x) + 16)
#define  MT5PT_MGMT_CFG_PRIO_MASK	0x7
#define  MT5PT_MGMT_CFG_PRIO_SHIFT	13

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

struct vids {
	int vid;
	bool egress_untagged;
	struct list_head list;
};

struct mt5pt_switch_port {
	struct net_device *ndev;
	struct mt5pt_switch *mt5pt;
	struct kobject kobj;
	int port_nr;
	int phy_addr;
	phy_interface_t phy_if;
	struct phy_device *phy_dev;
	int speed;
	int link;
	bool early_en;
	int pvid;
	struct vids vids;
	bool bpdu_port_en;
};

struct mt5pt_switch {
	void __iomem	*regs;
	struct clk	*clk;
	struct device	*dev;
	struct mii_bus	*bus;
	/* mutex for switch and port data (mainly for sysfs) */
	struct mutex     lock;

	unsigned long mdio_bus_freq;
	int nr_ports;
	struct mt5pt_switch_port *ports[MT5PT_NR_PORTS + 1];
	int bpdu_port_mgt;
};

int mt5pt_sysfs_add(struct device *dev);
void mt5pt_sysfs_remove(struct device *dev);
int mt5pt_port_sysfs_add(struct mt5pt_switch_port *port);
void mt5pt_port_sysfs_remove(struct mt5pt_switch_port *port);
int mt5pt_switch_fdb_flush(struct mt5pt_switch *mt5pt);
int mt5pt_switch_port_fdb_flush(void __iomem *regs, int port_nr);
void mt5pt_switch_bpdu_port_mgt_set(struct mt5pt_switch *mt5pt,
				    int port_nr, int prio);
void mt5pt_bpdu_port_en(struct mt5pt_switch_port *port, bool en);

#endif /* _SWITCH_H_ */
