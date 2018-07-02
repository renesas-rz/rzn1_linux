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
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/delay.h>

#include "rzn1-clock.h"
#include <dt-bindings/clock/rzn1-clocks.h>
#include <dt-bindings/soc/rzn1-sysctrl.h>

#define _BIT(_r, _p) { .reg = _r, .pos = _p }

#define _CLK(_n, _clk, _rst, _rdy, _midle, _scon, _mirack, _mistat ) \
	{ .name = _n, .clock = _clk, .reset = _rst, \
		.ready = _rdy, .masteridle = _midle }

#include "rzn1-clkctrl-tables.h"

static rzn1_clk_hook hook[RZN1_CLK_COUNT];

void rzn1_clk_set_hook(int clkdesc_id, rzn1_clk_hook clk_hook)
{
	BUG_ON(clkdesc_id >= RZN1_CLK_COUNT);
	hook[clkdesc_id] = clk_hook;
}

const struct rzn1_clkdesc *rzn1_get_clk_desc(int clkdesc_id)
{
	static struct rzn1_clkdesc zero = _CLK("*unknown*",
				{}, {}, {}, {}, {}, {}, {});

	return clkdesc_id < RZN1_CLK_COUNT ?
			&rzn1_clock_list[clkdesc_id] :
			&zero;
}
EXPORT_SYMBOL_GPL(rzn1_get_clk_desc);

static void clk_mgr_desc_set(const struct rzn1_onereg *one, int on)
{
	u32 *reg = ((u32 *)rzn1_sysctrl_base()) + one->reg;
	u32 val = clk_readl(reg);

	val = (val & ~(1 << one->pos)) | ((!!on) << one->pos);
	clk_writel(val, reg);
}

void rzn1_clk_set_gate(int clkdesc_id, int on)
{
	const struct rzn1_clkdesc *g = rzn1_get_clk_desc(clkdesc_id);

	BUG_ON(!rzn1_sysctrl_base());
	BUG_ON(!g->clock.reg);

	if (hook[clkdesc_id])
		if (hook[clkdesc_id](clkdesc_id, RZN1_CLK_HOOK_GATE_PRE, on))
			return;
	clk_mgr_desc_set(&(g->clock), on);

	if (hook[clkdesc_id])
		if (hook[clkdesc_id](clkdesc_id, RZN1_CLK_HOOK_GATE_SET, on))
			return;

	/* De-assert reset */
	if (g->reset.reg)
		clk_mgr_desc_set(&g->reset, 1);

	/* Hardware manual recommends 5us delay after enabling clock & reset */
	udelay(5);

	/* If the peripheral is memory mapped (i.e. an AXI slave), there is an
	 * associated SLVRDY bit in the System Controller that needs to be set
	 * so that the FlexWAY bus fabric passes on the read/write requests.
	 */
	if (g->ready.reg)
		clk_mgr_desc_set(&(g->ready), on);

	/* Clear 'Master Idle Request' bit */
	if (g->masteridle.reg)
		clk_mgr_desc_set(&(g->masteridle), !on);

	if (hook[clkdesc_id])
		hook[clkdesc_id](clkdesc_id, RZN1_CLK_HOOK_GATE_POST, on);

	/* Note: We don't wait for FlexWAY Socket Connection signal */
}
EXPORT_SYMBOL_GPL(rzn1_clk_set_gate);

int rzn1_clk_is_gate_enabled(int clkdesc_id)
{
	const struct rzn1_clkdesc *g = rzn1_get_clk_desc(clkdesc_id);
	u32 *reg;

	BUG_ON(!rzn1_sysctrl_base());
	BUG_ON(!g->clock.reg);

	reg = ((u32 *)rzn1_sysctrl_base()) + g->clock.reg;
	return !!(clk_readl(reg) & (1 << g->clock.pos));
}
EXPORT_SYMBOL_GPL(rzn1_clk_is_gate_enabled);


static ssize_t sys_clk_set_rate_show(
	struct kobject *kobj,
	struct kobj_attribute *attr,
	char *buf)
{
	strcpy(buf, "Usage: echo <clock name> <rate in hz> >clk_set_rate\n");
	return strlen(buf);
}

static ssize_t sys_clk_set_rate_store(
	struct kobject *kobj,
	struct kobj_attribute *attr,
	const char *buf, size_t count)
{
	char copy[count + 1];
	char *p = copy;
	const char *trate;
	const char *name;
	long rate;
	struct clk *clk;

	/* the 'buf' is const, so we can't strsep into it */
	strncpy(copy, buf, sizeof(copy));
	name = strsep(&p, " ");
	if (name)
		trate = strsep(&p, " ");
	if (!name || !trate)
		return count;
	pr_devel("%s clock:%s rate:%s\n", __func__, name, trate);
	if (kstrtol(trate, 10, &rate) != 0) {
		pr_err("%s %s, invalid rate %s\n", __func__, name, trate);
		return count;
	}
	pr_devel("%s %s requested rate %ld\n", __func__, name, rate);

	clk = __clk_lookup(name);
	if (IS_ERR(clk)) {
		pr_err("%s %s: clock not found\n", __func__, name);
		return -EINVAL;
	}
	pr_devel("%s clk %s found\n", __func__, name);
	pr_devel("%s %s current rate %lu\n", __func__, name,
		clk_get_rate(clk));
	if (rate > 0) {
		if (clk_set_rate(clk, rate)) {
			pr_err("%s %s FAILED\n", __func__, name);
		} else {
			pr_info("%s %s set to %lu\n", __func__, name,
				clk_get_rate(clk));
		}
	}
	return count;
}

static struct kobj_attribute clk_set_rate_attribute =
	__ATTR(clk_set_rate, 0644,
	       sys_clk_set_rate_show, sys_clk_set_rate_store);

static struct attribute *attrs[] = {
	&clk_set_rate_attribute.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};

static struct kobject *grp_kobj;
static int __init rzn1_clock_sys_init(void)
{
	int retval;

	grp_kobj = kobject_create_and_add("rzn1", kernel_kobj);
	if (!grp_kobj)
		return -ENOMEM;
	retval = sysfs_create_group(grp_kobj, &attr_group);
	if (retval)
		kobject_put(grp_kobj);
	return retval;
}
postcore_initcall(rzn1_clock_sys_init);

#define USBF_EPCTR		(RZN1_USB_DEV_BASE + 0x1000 + 0x10)
#define USBF_EPCTR_EPC_RST	BIT(0)
#define USBF_EPCTR_PLL_RST	BIT(2)
#define USBF_EPCTR_DIRPD	BIT(12)

#define USBH_USBCTR		(RZN1_USB_HOST_BASE + 0x10000 + 0x834)
#define USBH_USBCTR_USBH_RST	BIT(0)
#define USBH_USBCTR_PCICLK_MASK	BIT(1)
#define USBH_USBCTR_PLL_RST	BIT(2)
#define USBH_USBCTR_DIRPD	BIT(9)

#define setbits_le32(addr, bits) \
	writel(readl((void *)(addr)) |  (bits), (void *)(addr))
#define clrbits_le32(addr, bits) \
	writel(readl((void *)(addr)) & ~(bits), (void *)(addr))

static int rzn1_usb_clock_hook(int clkdesc_id, int operation, u32 value)
{
	u32 val, h2mode = 0;
	struct device_node *np;
	void *epctr = NULL;
	void *usbctr = NULL;

	if (operation != RZN1_CLK_HOOK_GATE_POST || value != 1)
		return 0;

	np = of_find_node_by_path("/chosen");
	if (np && of_property_read_bool(np, "rzn1,h2mode"))
		h2mode = BIT(RZN1_SYSCTRL_REG_CFG_USB_H2MODE);

	/* If the PLL is already started, we don't need to do it again anyway
	 * and if the USB configuration is already in the right mode, we don't
	 * need to configure that either */
	val = rzn1_sysctrl_readl(RZN1_SYSCTRL_REG_CFG_USB);
	if (!(val & BIT(RZN1_SYSCTRL_REG_CFG_USB_DIRPD)) &&
		(val & BIT(RZN1_SYSCTRL_REG_CFG_USB_H2MODE)) == h2mode &&
		rzn1_sysctrl_readl(RZN1_SYSCTRL_REG_USBSTAT) &
			BIT(RZN1_SYSCTRL_REG_USBSTAT_PLL_LOCK)) {
		pr_info("rzn1: USB PLL already started\n");
		return 0;
	}
	pr_info("rzn1: USB PLL in %s mode\n", h2mode ? "Host" : "Func");

	/* trick here, the usb function clocks NEEDS to have been enabled
	 * otherwise this register is not available. That means the h2mode
	 * still requires the usbf clocks, at least for this part (!) */
	epctr = ioremap(USBF_EPCTR, 4);
	usbctr = ioremap(USBH_USBCTR, 4);

	/* Hold USBF and USBH in reset */
	writel(USBH_USBCTR_USBH_RST | USBH_USBCTR_PCICLK_MASK, usbctr);
	writel(USBF_EPCTR_EPC_RST, epctr);
	/* Hold USBPLL in reset */
	setbits_le32(usbctr, USBH_USBCTR_PLL_RST);
	udelay(2);

	/* Power down USB PLL, setting any DIRPD bit will do this */
	setbits_le32(usbctr, USBH_USBCTR_DIRPD);

	/* Stop USB suspend from powering down the USB PLL */
	/* Note: have to update these bits at the same time */
	val |= BIT(RZN1_SYSCTRL_REG_CFG_USB_FRCLK48MOD);
	val &= ~BIT(RZN1_SYSCTRL_REG_CFG_USB_DIRPD);
	val &= ~BIT(RZN1_SYSCTRL_REG_CFG_USB_H2MODE);
	val |= h2mode;
	rzn1_sysctrl_writel(val, RZN1_SYSCTRL_REG_CFG_USB);

	/* Power up USB PLL, all DIRPD bits need to be cleared */
	clrbits_le32(epctr, USBF_EPCTR_DIRPD);
	clrbits_le32(usbctr, USBH_USBCTR_DIRPD);
	udelay(1000);

	/* Release USBPLL reset, either PLL_RST bit will do this */
	clrbits_le32(usbctr, USBH_USBCTR_PLL_RST);

	iounmap(epctr);
	iounmap(usbctr);

	/* Wait for USB PLL lock */
	do {
		val = rzn1_sysctrl_readl(RZN1_SYSCTRL_REG_USBSTAT);
	} while (!(val & BIT(RZN1_SYSCTRL_REG_USBSTAT_PLL_LOCK)));

	return 0;
}

static void __init rzn1_clock_init(struct device_node *node)
{
	rzn1_sysctrl_init();

	/* This hook will decide of the fate of the HOST vs
	 * DEVICE for the usb port -- the board designer has to enable
	 * either of the driver in the DTS, or have the bootloader set
	 * the appropriate DT property before starting the kernel */
	rzn1_clk_set_hook(RZN1_CLK_48MHZ_PG_F_ID, rzn1_usb_clock_hook);
}


CLK_OF_DECLARE(rzn1_clock, "renesas,rzn1-clock", rzn1_clock_init);
