/*
 * MoreThanIP 5-port switch MDIO driver - sysfs
 *
 * Copyright (C) 2018 Schneider Electric.
 * Author:Jean-Christophe Gillet <jean-christophe.gillet@schneider-electric.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

/* sysfs organisation
 *
 * /sys/bus/platform/devices/44050000.eth-switch
 * |- sw : directory for switch general configuration and status
 * |-   fdb_flush (WO):
 * |               flush auto-learning fdb entries relative to all ports
 * |-   fdb_show (RO)
 * |               dump fdb for all port and show it
 * |-   bpdu_port_mgt <p> (RW):
 * |               if p=[0..4] BPDU frames coming from any port
 * |               except p are forwarded to management port number <p> only
 * |               if p=-1 BPDU frames are discarded always
 * |
 * |- swp<i> : directory for switch port configuration and status
 * |               configuration and status relative to MTIP switch port <i>
 * |-    fdb_flush (WO):
 * |               flush auto-learning fdb enties relative to this switch port
 * |-    fdb_show (RO):
 * |               dump fdb for this port and show it
 * |-    vlan_show (RO):
 * |               list vlan configuration relative to this switch port
 * |-    vlan_add "tagged|untagged" <vid> (WO):
 * |               add a new vlan filter entry <vid>
 * |               the vlan specified is to be treated as tagged (keep tag)
 * |               or untagged on egress relative to this <vid>
 * |-    vlan_rm <vid> (WO):
 * |               remove vlan filter entry <vid>
 * |-    pvid <pvid> (RW):
 * |               the vlan specified is to be considered a PVID at ingress.
 * |               any untagged frames will be assigned to this VLAN.
 * |-    bpdu_port_en [0|1] (RW):
 * |               port number list. BPDU frame coming from management port
 *                 will be dispatched to this port
 */
#include <linux/err.h>
#include <linux/etherdevice.h>
#include "mtip_switch.h"

/* UNTAGGED_ADD_TAG_AND_TAGGED_PASSTROUGH
 *   insert tag for untagged frames
 *   Leave frame unmodified if tagged and VID > 0.
 *   If tagged with VID = 0 (priority tagged) then the VID will be overwritten
 *   with the VID from SYSTEM_TAGINFO and priority is kept
 * UNTAGGED_ADD_TAG_AND_TAGGED_REPLACE
 *   insert tag for untagged frames
 *   if single tagged, overwrite the tag
 * ADD_TAG_ALWAYS
 *   insert tag (single tagged frame) when an untagged is received
 *   insert tag (double tagged frame) when a single tagged frame is received
 *   insert tag (triple tagged frame) when a double tagged frame is received
 */
enum ingress_vlan_mode {
	UNTAGGED_ADD_TAG_AND_TAGGED_PASSTROUGH,
	UNTAGGED_ADD_TAG_AND_TAGGED_REPLACE,
	ADD_TAG_ALWAYS,
};

/* NO_CHANGE        = no frame manipulation
 * REMOVE_ALL_TAG   = all tags (single or double) are removed
 * REMOVE_FIRST_TAG = Always removes first tag from frame
 *                    the inner Tag is passed thru while the outer Tag is
 *                    removed for a double Tagged frame
 * NO_CHANGE_FOR_THIS_VID | REMOVE_FIRST_TAG_FOR_THIS_VID =
 *                    no frame manipulation or remove first tag under certain
 *                    condition frame unmodified in both cases
 *                    (NO_CHANGE_FOR_THIS_VID)
 *                    - frame’s VLAN id is found in the VLAN table + port_nr
 *                    is defined as tagged for the VLAN
 *                    - frame’s VLAN id is not found in the VLAN table
 *                    first tag is removed (REMOVE_FIRST_TAG_FOR_THIS_VID) if
 *                    - frame’s VLAN id is found in the VLAN table and the
 *                    port_nr is defined as untagged for the VLAN
 */
enum egress_vlan_mode {
	NO_CHANGE,
	REMOVE_ALL_TAG,
	REMOVE_FIRST_TAG,
	NO_CHANGE_FOR_THIS_VID,
	REMOVE_FIRST_TAG_FOR_THIS_VID,
};

struct mt5pt_attribute {
	struct attribute	attr;
	ssize_t (*show)(struct mt5pt_switch_port *port,
			struct mt5pt_attribute *attr,
			char *buf);
	ssize_t (*store)(struct mt5pt_switch_port *port,
			 struct mt5pt_attribute *attr,
			 const char *buf, size_t count);
};

struct vlan_res_tbl_entry {
	int found_off;
	int free_off;
};

/* scan table to find if vid already exists in table */
static void mt5pt_get_vltbl(struct mt5pt_switch_port *port, int vid,
			    struct vlan_res_tbl_entry *tbl)
{
	void __iomem *regs = port->mt5pt->regs;
	u32 val;
	int i;

	tbl->found_off = -1;
	tbl->free_off = -1;

	for (i = 0; i < MT5PT_VLTBL_SIZE; i += 4) {
		val = readl(regs + MT5PT_VLTBL_BASE + i);
		val >>= MT5PT_VLTBL_VID_SHIFT;
		val &= MT5PT_VLTBL_VID_MASK;

		if ((tbl->found_off == -1) && (val == vid))
			tbl->found_off = i;

		if ((tbl->free_off == -1) && (val == MT5PT_VLTBL_VID_MASK))
			tbl->free_off = i;
	}
}

static void mt5pt_add_port_vltbl(struct mt5pt_switch_port *port, int vid,
				 enum egress_vlan_mode em, int off)
{
	void __iomem *entry = port->mt5pt->regs + MT5PT_VLTBL_BASE + off;
	int port_nr = port->port_nr;
	u32 val_pm;
	u32 val_tm;

	/* update portmask */
	val_pm = readl(entry);
	val_pm |= MT5PT_VLTBL_WR_PORTMASK | vid << MT5PT_VLTBL_VID_SHIFT |
		  MT5PT_VLTBL_PORT(port_nr);
	writel(val_pm, entry);

	/* read for tagmask */
	writel(MT5PT_VLTBL_RD_TAGMASK, entry);
	val_tm = readl(entry);

	/* update tagmask */
	val_tm &= MT5PT_VLTBL_PORT_MASK;
	/* bit = 1: leave the fame tagged, bit = 0: untag the fame */
	if (em == REMOVE_FIRST_TAG_FOR_THIS_VID)
		val_tm &= ~(MT5PT_VLTBL_PORT(port_nr));
	else
		val_tm |= MT5PT_VLTBL_PORT(port_nr);

	val_tm |= MT5PT_VLTBL_WR_TAGMASK | vid << MT5PT_VLTBL_VID_SHIFT;

	writel(val_tm, entry);
}

/* Remove all port as member of vid = 0xfff in vlan resolution table for a
 * single entry
 */
static void mt5pt_init_vltbl(struct mt5pt_switch_port *port, int vid, int off)
{
	void __iomem *entry = port->mt5pt->regs + MT5PT_VLTBL_BASE + off;

	/* clear portmask and set tagmask */
	writel(MT5PT_VLTBL_WR_PORTMASK |
	       vid << MT5PT_VLTBL_VID_SHIFT,
	       entry);
	/* set tagmask (leave the fame tagged) */
	writel(MT5PT_VLTBL_WR_TAGMASK |
	       MT5PT_VLTBL_PORT_MASK |
	       vid << MT5PT_VLTBL_VID_SHIFT,
	       entry);
}

/* Inititalise one entry in vlan resolution table.
 * Set all ports as member of vid = 0
 */
static void mt5pt_free_vltbl(struct mt5pt_switch_port *port, int off)
{
	void __iomem *entry = port->mt5pt->regs + MT5PT_VLTBL_BASE + off;
	int vid = MT5PT_VLTBL_VID_MASK;

	/* set portmask and tagmask (leave the fame tagged) */
	writel(MT5PT_VLTBL_WR_PORTMASK |
	       MT5PT_VLTBL_WR_TAGMASK |
	       MT5PT_VLTBL_PORT_MASK |
	       vid << MT5PT_VLTBL_VID_SHIFT, entry);
}

static void mt5pt_rm_port_vltbl(struct mt5pt_switch_port *port, int off)
{
	void __iomem *entry = port->mt5pt->regs + MT5PT_VLTBL_BASE + off;
	int port_nr = port->port_nr;
	u32 val;

	val = readl(entry);
	val |= MT5PT_VLTBL_WR_PORTMASK;
	val &= ~(MT5PT_VLTBL_PORT(port_nr));
	writel(val, entry);

	if (!(val & MT5PT_VLTBL_PORT_MASK))
		mt5pt_free_vltbl(port, off);
}

static bool vid_present(struct mt5pt_switch_port *port, int vid)
{
	struct vids *v;

	list_for_each_entry(v, &port->vids.list, list)
		if (v->vid == vid)
			return true;

	return false;
}

static int mt5pt_vlan_add(struct mt5pt_switch_port *port, int vid)
{
	struct vlan_res_tbl_entry entry;
	enum egress_vlan_mode em = REMOVE_FIRST_TAG_FOR_THIS_VID;

	mt5pt_get_vltbl(port, vid, &entry);

	if (entry.found_off != -1) {
		mt5pt_add_port_vltbl(port, vid, em, entry.found_off);
	} else if (entry.free_off != -1) {
		mt5pt_init_vltbl(port, vid, entry.free_off);
		mt5pt_add_port_vltbl(port, vid, em, entry.free_off);
	} else {
		dev_err(port->mt5pt->dev, "no more space in resolution table to add a new entry\n");
		return -ENOSPC;
	}

	if (!port->pvid)
		port->pvid = vid;

	return 0;
}

static void mt5pt_vlan_upd_ingress(struct mt5pt_switch_port *port,
				   enum ingress_vlan_mode im)
{
	void __iomem *regs = port->mt5pt->regs;
	int port_nr = port->port_nr;
	u32 val;
	const u32 prio = 0;
	const u32 cfi = 0;
	int mode;

	val = prio <<  13 | cfi << 12 | port->pvid;
	writel(val, regs + MT5PT_SYSTEM_TAGINFO_PN(port_nr));

	switch (im) {
	case UNTAGGED_ADD_TAG_AND_TAGGED_PASSTROUGH:
		mode = 0;
		break;
	case UNTAGGED_ADD_TAG_AND_TAGGED_REPLACE:
		mode = 1;
		break;
	case ADD_TAG_ALWAYS:
		mode = 2;
		break;
	default:
		mode = 0;
		break;
	}

	val = readl(regs + MT5PT_VL_IN_MODE);
	val &= ~MT5PT_VL_IN_MODE_MASK_PN(port_nr);
	val |= mode << MT5PT_VL_IN_MODE_SHIFT_PN(port_nr);
	writel(val, regs + MT5PT_VL_IN_MODE);

	val = readl(regs + MT5PT_VL_IN_MODE_ENA);
	val |= MT5PT_VL_IN_MODE_ENA_PN(port_nr);
	writel(val, regs + MT5PT_VL_IN_MODE_ENA);

	// do not verify VLAN for this port
	val = readl(regs + MT5PT_VL_VERIFY);
	val &= ~MT5PT_VL_VERIFY_PN(port_nr);
	val &= ~MT5PT_VL_VERIFY_DISCARD_UNK_TAG_PN(port_nr);
	writel(val, regs + MT5PT_VL_VERIFY);
}

static void mt5pt_vlan_upd_egress(struct mt5pt_switch_port *port, int vid,
				  enum egress_vlan_mode em)
{
	void __iomem *regs = port->mt5pt->regs;
	int port_nr = port->port_nr;
	u32 val;
	int mode;

	switch (em) {
	case NO_CHANGE:
		mode = 0;
		break;
	case REMOVE_ALL_TAG:
		mode = 1;
		break;
	case REMOVE_FIRST_TAG:
		mode = 2;
		break;
	case NO_CHANGE_FOR_THIS_VID:
		mode = 3;
		break;
	case REMOVE_FIRST_TAG_FOR_THIS_VID:
		mode = 3;
		break;
	default:
		mode = 3;
		break;
	}

	val = readl(regs + MT5PT_VL_OUT_MODE);
	val &= ~MT5PT_VL_OUT_MODE_MASK_PN(port_nr);
	val |= mode << MT5PT_VL_OUT_MODE_SHIFT_PN(port_nr);
	writel(val, regs + MT5PT_VL_OUT_MODE);
}

/* Remove vid from vlan resolution table and ingress
 * If there is no more vid for this port, disable vlan for this port
 * If still vid exist for this port, arbitrary use first vid in the list of vid
 * as pvid.
 */
static void mt5pt_vlan_remove(struct mt5pt_switch_port *port, int vid)
{
	struct vlan_res_tbl_entry entry;
	void __iomem *regs = port->mt5pt->regs;
	struct vids *v;
	u32 val;

	mt5pt_get_vltbl(port, vid, &entry);

	if (entry.found_off == -1) {
		WARN_ON(1);
		return;
	}

	mt5pt_rm_port_vltbl(port, entry.found_off);

	if (list_empty(&port->vids.list)) {
		port->pvid = 0;

		val = readl(regs + MT5PT_VL_IN_MODE_ENA);
		val &= ~MT5PT_VL_IN_MODE_ENA_PN(port->port_nr);
		writel(val, regs + MT5PT_VL_IN_MODE_ENA);

	} else if (!vid_present(port, port->pvid)) {
		v = list_first_entry(&port->vids.list, struct vids, list);
		port->pvid = v->vid;
		mt5pt_vlan_upd_ingress(port,
				       UNTAGGED_ADD_TAG_AND_TAGGED_PASSTROUGH);
	}
}

static ssize_t vlan_add_store(struct mt5pt_switch_port *port,
			      struct mt5pt_attribute *attr,
			      const char *buf,
			      size_t count)
{
	struct vids *v;
	int vid;
	bool untagged;
	char *p;
	int ret;

	if (strncmp(buf, "tagged", 6) == 0) {
		untagged = false;
	} else if (strncmp(buf, "untagged", 8) == 0) {
		untagged = true;
	} else {
		dev_err(port->mt5pt->dev,
			"illegal format. Must start by \"tagged\" or \"untagged\"\n");
		return -EINVAL;
	}

	p = strpbrk(buf, " ");
	if (!p || ((p + 1) >= (buf + count))) {
		dev_err(port->mt5pt->dev,
			"illegal format\n");
		return -EINVAL;
	}

	ret = kstrtoint(p + 1, 0, &vid);
	if (ret || (vid < 1) || (vid > 4094)) {
		dev_err(port->mt5pt->dev,
			"illegal format or vid out of range [1,4094]\n");
		return -EINVAL;
	}

	mutex_lock(&port->mt5pt->lock);

	if (vid_present(port, vid)) {
		mutex_unlock(&port->mt5pt->lock);
		return count;
	}

	v = devm_kzalloc(port->mt5pt->dev, sizeof(*v), GFP_KERNEL);
	if (!v) {
		mutex_unlock(&port->mt5pt->lock);
		return -ENOMEM;
	}

	v->vid = vid;
	v->egress_untagged = untagged;
	list_add_tail(&v->list, &port->vids.list);

	ret = mt5pt_vlan_add(port, vid);
	if (ret) {
		list_del(&v->list);
		devm_kfree(port->mt5pt->dev, v);
		mutex_unlock(&port->mt5pt->lock);
		return ret;
	}

	mt5pt_vlan_upd_ingress(port, UNTAGGED_ADD_TAG_AND_TAGGED_PASSTROUGH);
	if (untagged)
		mt5pt_vlan_upd_egress(port, vid, REMOVE_FIRST_TAG_FOR_THIS_VID);
	else
		mt5pt_vlan_upd_egress(port, vid, NO_CHANGE_FOR_THIS_VID);

	mutex_unlock(&port->mt5pt->lock);
	return count;
}

static ssize_t vlan_rm_store(struct mt5pt_switch_port *port,
			     struct mt5pt_attribute *attr,
			     const char *buf,
			     size_t count)
{
	struct vids *v;
	struct list_head *iter, *tmp;
	int vid;
	int ret;
	bool found = false;

	ret = kstrtoint(buf, 0, &vid);
	if (ret || (vid < 1) || (vid > 4094)) {
		dev_err(port->mt5pt->dev,
			"vid illegal format or out of range [1,4094]\n");
		return -EINVAL;
	}

	mutex_lock(&port->mt5pt->lock);

	list_for_each_safe(iter, tmp, &port->vids.list) {
		v = list_entry(iter, struct vids, list);
		if (v->vid == vid) {
			found = true;
			break;
		}
	}

	if (!found) {
		mutex_unlock(&port->mt5pt->lock);
		dev_err(port->mt5pt->dev, "vid not present\n");
		return -EINVAL;
	}

	list_del(iter);
	devm_kfree(port->mt5pt->dev, v);
	mt5pt_vlan_remove(port, vid);
	mutex_unlock(&port->mt5pt->lock);

	return count;
}

static ssize_t vlan_show_show(struct mt5pt_switch_port *port,
			      struct mt5pt_attribute *attr,
			      char *buf)
{
	struct vids *v;
	int count = 0;

	mutex_lock(&port->mt5pt->lock);

	list_for_each_entry(v, &port->vids.list, list) {
		if (PAGE_SIZE - count <= 1)
			break;

		count += scnprintf(buf + count,
				   PAGE_SIZE - count,
				   "%-4d %s egress %s\n",
				   v->vid,
				   port->pvid == v->vid ? "pvid" : "    ",
				   v->egress_untagged ? "untagged" : "tagged");
	}

	mutex_unlock(&port->mt5pt->lock);

	return count;
}

static ssize_t pvid_store(struct mt5pt_switch_port *port,
			  struct mt5pt_attribute *attr,
			  const char *buf,
			  size_t count)
{
	int ret;
	int pvid;

	ret = kstrtoint(buf, 0, &pvid);
	if (ret || (pvid < 1) || (pvid > 4094)) {
		dev_err(port->mt5pt->dev,
			"pvid illegal format or out of range [1,4094]\n");
		return -EINVAL;
	}

	mutex_lock(&port->mt5pt->lock);

	if (!vid_present(port, pvid)) {
		mutex_unlock(&port->mt5pt->lock);
		dev_err(port->mt5pt->dev, "add vip before pvid\n");
		return -EINVAL;
	}

	port->pvid = pvid;
	mt5pt_vlan_upd_ingress(port, UNTAGGED_ADD_TAG_AND_TAGGED_PASSTROUGH);
	mutex_unlock(&port->mt5pt->lock);

	return count;
}

static ssize_t pvid_show(struct mt5pt_switch_port *port,
			 struct mt5pt_attribute *attr,
			 char *buf)
{
	return sprintf(buf, "%d", port->pvid);
}

static ssize_t fdb_flush_store(struct mt5pt_switch_port *port,
			       struct mt5pt_attribute *attr,
			       const char *buf,
			       size_t count)
{
	int ret;

	mutex_lock(&port->mt5pt->lock);
	ret = mt5pt_switch_port_fdb_flush(port->mt5pt->regs, port->port_nr);
	mutex_unlock(&port->mt5pt->lock);

	if (ret)
		return ret;

	dev_info(port->mt5pt->dev,
		 "dynamic fdb entries was flushed for port %d\n",
		 port->port_nr);
	return count;
}

static ssize_t mt5pt_fdb_show(struct mt5pt_switch *mt5pt, int port_flt,
			      char *buf)
{
	void __iomem *regs = mt5pt->regs;
	u32 val;
	u32 valh;
	int count = 0;
	int i;
	int portmask;

	for (i = 0; i < MT5PT_LK_ADDR_MASK; i++) {
		val = MT5PT_LK_RD | MT5PT_LK_WAIT_COMPLETE | i;
		writel(val, regs + MT5PT_LK_ADDR_CTRL);

		val = readl(regs + MT5PT_LK_DATA_LO);
		valh = readl(regs + MT5PT_LK_DATA_HI);

		if (!val && !valh)
			continue;

		portmask = (valh >> 21) & 0x1f;

		if ((port_flt == -1) || ((portmask >> port_flt) & 0x1)) {
			if (PAGE_SIZE - count <= 1)
				break;

			count += scnprintf(buf + count,
					   PAGE_SIZE - count,
					   "portmask=0x%x mac=%02x:%02x:%02x:%02x:%02x:%02x attr=0x%x @0x%x\n",
					   portmask,
					   val & 0xff,
					   (val >> 8) & 0xff,
					   (val >> 16) & 0xff,
					   (val >> 24) & 0xff,
					   valh & 0xff,
					   (valh >> 8) & 0xff,
					   (valh >> 16) & 0x1f,
					   i);
		}
	}

	return count;
}

static ssize_t fdb_show_show(struct mt5pt_switch_port *port,
			     struct mt5pt_attribute *attr,
			     char *buf)
{
	ssize_t ret;

	mutex_lock(&port->mt5pt->lock);
	ret = mt5pt_fdb_show(port->mt5pt, port->port_nr, buf);
	mutex_unlock(&port->mt5pt->lock);
	return ret;
}

void mt5pt_bpdu_port_en(struct mt5pt_switch_port *port, bool en)
{
	void __iomem *regs = port->mt5pt->regs;
	u32 val;

	port->bpdu_port_en = en;

	val = readl(regs + MT5PT_MGMT_CFG);
	if (en)
		val |= MT5PT_MGMT_CFG_PORTMASK_PN(port->port_nr);
	else
		val &= ~MT5PT_MGMT_CFG_PORTMASK_PN(port->port_nr);
	writel(val, regs + MT5PT_MGMT_CFG);
}

static ssize_t bpdu_port_en_store(struct mt5pt_switch_port *port,
				  struct mt5pt_attribute *attr,
				  const char *buf,
				  size_t count)
{
	int ret;
	int en;

	ret = kstrtoint(buf, 0, &en);
	if (ret || (en < 0) || (en > 1)) {
		dev_err(port->mt5pt->dev, "arg out of range [0,1]\n");
		return -EINVAL;
	}

	mutex_lock(&port->mt5pt->lock);
	mt5pt_bpdu_port_en(port, en);
	mutex_unlock(&port->mt5pt->lock);

	return count;
}

static ssize_t bpdu_port_en_show(struct mt5pt_switch_port *port,
				 struct mt5pt_attribute *attr,
				 char *buf)
{
	return sprintf(buf, "%d", port->bpdu_port_en);
}

#define MT5PT_ATTR(_name, _mode, _show, _store) \
	struct mt5pt_attribute mt5pt_attr_##_name = \
	__ATTR(_name, _mode, _show, _store)
#define MT5PT_ATTR_RW(_name) \
	struct mt5pt_attribute mt5pt_attr_##_name = __ATTR_RW(_name)
#define MT5PT_ATTR_RO(_name) \
	struct mt5pt_attribute mt5pt_attr_##_name = __ATTR_RO(_name)
#define MT5PT_ATTR_WO(_name) \
	struct mt5pt_attribute mt5pt_attr_##_name = __ATTR_WO(_name)

static MT5PT_ATTR_RW(pvid);
static MT5PT_ATTR_WO(vlan_add);
static MT5PT_ATTR_WO(vlan_rm);
static MT5PT_ATTR_RO(vlan_show);
static MT5PT_ATTR_WO(fdb_flush);
static MT5PT_ATTR_RO(fdb_show);
static MT5PT_ATTR_RW(bpdu_port_en);

static struct attribute *mt5pt_port_attrs[] = {
	&mt5pt_attr_pvid.attr,
	&mt5pt_attr_vlan_add.attr,
	&mt5pt_attr_vlan_rm.attr,
	&mt5pt_attr_vlan_show.attr,
	&mt5pt_attr_fdb_flush.attr,
	&mt5pt_attr_fdb_show.attr,
	&mt5pt_attr_bpdu_port_en.attr,
	NULL
};

static struct attribute_group mt5pt_port_attribute_group = {
	.attrs = mt5pt_port_attrs
};

#define to_mt5pt_attr(_attr) container_of(_attr, struct mt5pt_attribute, attr)
#define to_mt5pt_port(_kobj) container_of(_kobj, struct mt5pt_switch_port, kobj)

static ssize_t mt5pt_attr_show(struct kobject *kobj, struct attribute *attr,
			       char *buf)
{
	struct mt5pt_attribute *mt5pt_attr = to_mt5pt_attr(attr);
	struct mt5pt_switch_port *port = to_mt5pt_port(kobj);

	if (!mt5pt_attr->show)
		return -EIO;

	return mt5pt_attr->show(port, mt5pt_attr, buf);
}

static ssize_t mt5pt_attr_store(struct kobject *kobj, struct attribute *attr,
				const char *buf, size_t count)
{
	struct mt5pt_attribute *mt5pt_attr = to_mt5pt_attr(attr);
	struct mt5pt_switch_port *port = to_mt5pt_port(kobj);

	if (!mt5pt_attr->store)
		return -EIO;

	return mt5pt_attr->store(port, mt5pt_attr, buf, count);
}

static const struct sysfs_ops mt5pt_port_sysfs_ops = {
	.show	= mt5pt_attr_show,
	.store	= mt5pt_attr_store,
};

static struct kobj_type mt5pt_port_ktype = {
	.sysfs_ops	= &mt5pt_port_sysfs_ops,
};

static ssize_t dev_switch_fdb_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct mt5pt_switch *mt5pt = dev_get_drvdata(dev);
	ssize_t ret;

	mutex_lock(&mt5pt->lock);
	ret = mt5pt_fdb_show(mt5pt, -1, buf);
	mutex_unlock(&mt5pt->lock);
	return ret;
}

static ssize_t dev_switch_fdb_flush(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t count)
{
	struct mt5pt_switch *mt5pt = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&mt5pt->lock);
	ret = mt5pt_switch_fdb_flush(mt5pt);
	mutex_unlock(&mt5pt->lock);
	if (ret)
		return ret;

	dev_info(dev, "all dynamic fdb entries was flushed\n");
	return count;
}

void mt5pt_switch_bpdu_port_mgt_set(struct mt5pt_switch *mt5pt,
				    int port_nr, int prio)
{
	void __iomem *regs = mt5pt->regs;
	u32 val;

	mt5pt->bpdu_port_mgt = port_nr;

	val = readl(regs + MT5PT_MGMT_CFG);
	if (port_nr != -1) {
		val |= MT5PT_MGMT_CFG_EN_BPDU;
		val &= MT5PT_MGMT_CFG_PRIO_MASK;
		val |= prio << MT5PT_MGMT_CFG_PRIO_SHIFT;
		val &= ~MT5PT_MGMT_CFG_PORT_MASK;
		val |= port_nr;
	} else {
		val |= MT5PT_MGMT_CFG_DIS_BPDU;
	}
	writel(val, regs + MT5PT_MGMT_CFG);
}

static ssize_t dev_switch_bpdu_port_mgt_show(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct mt5pt_switch *mt5pt = dev_get_drvdata(dev);

	return sprintf(buf, "%d", mt5pt->bpdu_port_mgt);
}

static ssize_t dev_switch_bpdu_port_mgt_store(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf,
					      size_t count)
{
	struct mt5pt_switch *mt5pt = dev_get_drvdata(dev);
	int ret;
	int port_mgt;

	ret = kstrtoint(buf, 0, &port_mgt);
	if (ret || (port_mgt < -1) || (port_mgt > MT5PT_NR_PORTS)) {
		dev_err(mt5pt->dev,
			"arg out of range [-1,%d]\n", MT5PT_NR_PORTS);
		return -EINVAL;
	}

	if (port_mgt != 1 && !mt5pt->ports[port_mgt]) {
		dev_err(mt5pt->dev, "port doesn't exist in DT\n");
		return -EINVAL;
	}

	mutex_lock(&mt5pt->lock);
	mt5pt_switch_bpdu_port_mgt_set(mt5pt, port_mgt, 0);
	mutex_unlock(&mt5pt->lock);

	return count;
}

static DEVICE_ATTR(fdb_show, 0444, dev_switch_fdb_show, NULL);
static DEVICE_ATTR(fdb_flush, 0200, NULL, dev_switch_fdb_flush);
static DEVICE_ATTR(bpdu_port_mgt, 0644,
		   dev_switch_bpdu_port_mgt_show,
		   dev_switch_bpdu_port_mgt_store);

static struct attribute *mt5pt_attrs[] = {
	&dev_attr_fdb_show.attr,
	&dev_attr_fdb_flush.attr,
	&dev_attr_bpdu_port_mgt.attr,
	NULL
};

static struct attribute_group mt5pt_attribute_group = {
	.name = "sw",
	.attrs = mt5pt_attrs
};

int mt5pt_sysfs_add(struct device *dev)
{
	return sysfs_create_group(&dev->kobj, &mt5pt_attribute_group);
}

int mt5pt_port_sysfs_add(struct mt5pt_switch_port *port)
{
	int ret;

	ret = kobject_init_and_add(&port->kobj,
				   &mt5pt_port_ktype,
				   &port->mt5pt->dev->kobj,
				   "swp%u", port->port_nr);
	if (ret)
		return ret;

	ret = sysfs_create_group(&port->kobj, &mt5pt_port_attribute_group);
	if (ret)
		kobject_put(&port->kobj);

	return ret;
}

void mt5pt_sysfs_remove(struct device *dev)
{
	sysfs_remove_group(&dev->kobj, &mt5pt_attribute_group);
}

void mt5pt_port_sysfs_remove(struct mt5pt_switch_port *port)
{
	sysfs_remove_group(&port->kobj, &mt5pt_port_attribute_group);
}
