// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2015 Renesas Electronics Europe Ltd.
 *
 * Based on highbank edac driver:
 * Copyright 2011-2012 Calxeda, Inc.
 */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/ctype.h>
#include <linux/edac.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/uaccess.h>

#include "edac_mc.h"
#include "edac_module.h"

/* DDR Controller Error Registers */
#define CAD_DDR_DDR_CLASS		(00 * 4 + 1)	/* DENALI_CTL_00 */
#define CAD_DDR_ECC_ENABLE		(36 * 4 + 2)	/* DENALI_CTL_36 */

#define CAD_DDR_ECC_FWC			(36 * 4 + 3)	/* DENALI_CTL_36 */
#define CAD_DDR_ECC_XOR			(37 * 4)	/* DENALI_CTL_37 */
#define CAD_DDR_ECC_U_ERR_ADDR		(38 * 4)	/* DENALI_CTL_38 */
#define CAD_DDR_ECC_U_ERR_STAT		(39 * 4)	/* DENALI_CTL_39 */
#define CAD_DDR_ECC_U_ERR_DATA		(40 * 4)	/* DENALI_CTL_40 */
#define CAD_DDR_ECC_C_ERR_ADDR		(41 * 4)	/* DENALI_CTL_41 */
#define CAD_DDR_ECC_C_ERR_STAT		(42 * 4)	/* DENALI_CTL_42 */
#define CAD_DDR_ECC_C_ERR_DATA		(43 * 4)	/* DENALI_CTL_43 */
#define CAD_DDR_PORT_CMD_ERR_ADDR	(61 * 4)	/* DENALI_CTL_61 */
#define CAD_DDR_PORT_CMD_ERR_SRC_ID	(62 * 4 + 0)	/* DENALI_CTL_62 */
#define CAD_DDR_PORT_CMD_ERR_TYPE	(62 * 4 + 1)	/* DENALI_CTL_62 */

/* DDR Controller Interrupt Registers */
#define CAD_DDR_ECC_INT_STATUS		(56 * 4)	/* DENALI_CTL_56 */
#define CAD_DDR_ECC_INT_ACK		(57 * 4)	/* DENALI_CTL_57 */
#define CAD_DDR_ECC_INT_MASK		(58 * 4)	/* DENALI_CTL_58 */

#define CAD_DDR_ECC_INT_STAT_CE		BIT(3)
#define CAD_DDR_ECC_INT_STAT_UE		BIT(5)
#define CAD_DDR_ECC_INT_STAT_PORT	BIT(7)

struct cad_mc_drvdata {
	void __iomem *io_base;
};

static irqreturn_t cadence_mc_err_handler(int irq, void *dev_id)
{
	struct mem_ctl_info *mci = dev_id;
	struct cad_mc_drvdata *drvdata = mci->pvt_info;
	u32 status, err_addr;

	/* Read the interrupt status register */
	status = readl(drvdata->io_base + CAD_DDR_ECC_INT_STATUS);

	if (status & CAD_DDR_ECC_INT_STAT_UE) {
		u32 syndrome = readl(drvdata->io_base + CAD_DDR_ECC_U_ERR_STAT);

		syndrome = syndrome & 0x7f;
		err_addr = readl(drvdata->io_base + CAD_DDR_ECC_U_ERR_ADDR);
		edac_mc_handle_error(HW_EVENT_ERR_UNCORRECTED, mci, 1,
				     err_addr >> PAGE_SHIFT,
				     err_addr & ~PAGE_MASK, syndrome,
				     0, 0, -1,
				     mci->ctl_name, "");
	}
	if (status & CAD_DDR_ECC_INT_STAT_CE) {
		u32 syndrome = readl(drvdata->io_base + CAD_DDR_ECC_C_ERR_STAT);

		syndrome = syndrome & 0x7f;
		err_addr = readl(drvdata->io_base + CAD_DDR_ECC_C_ERR_ADDR);

		// TODO scrub by reading 32-bit and write back?

		edac_mc_handle_error(HW_EVENT_ERR_CORRECTED, mci, 1,
				     err_addr >> PAGE_SHIFT,
				     err_addr & ~PAGE_MASK, syndrome,
				     0, 0, -1,
				     mci->ctl_name, "");
	}

	if (status & CAD_DDR_ECC_INT_STAT_PORT) {
		u32 addr = readl(drvdata->io_base + CAD_DDR_PORT_CMD_ERR_ADDR);
		u8 type = readb(drvdata->io_base + CAD_DDR_PORT_CMD_ERR_TYPE);
		u8 id = readb(drvdata->io_base + CAD_DDR_PORT_CMD_ERR_SRC_ID);

		pr_warn("DDR protected at addr 0x%x, type 0x%x, id 0x%x\n",
			addr, type, id);
	}

	/* clear the error, clears the interrupt */
	writel(status, drvdata->io_base + CAD_DDR_ECC_INT_ACK);

	return IRQ_HANDLED;
}

static void cadence_mc_err_inject(struct mem_ctl_info *mci, u16 synd)
{
	struct cad_mc_drvdata *pdata = mci->pvt_info;

	writew(synd & 0x3fff, pdata->io_base + CAD_DDR_ECC_XOR);
	writeb(1, pdata->io_base + CAD_DDR_ECC_FWC);
}

#define to_mci(k) container_of(k, struct mem_ctl_info, dev)

static ssize_t cadence_mc_inject_ctrl(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count)
{
	struct mem_ctl_info *mci = to_mci(dev);
	u16 synd;

	if (kstrtou16(buf, 16, &synd))
		return -EINVAL;

	cadence_mc_err_inject(mci, synd);

	return count;
}

static DEVICE_ATTR(inject_ctrl, 0200, NULL, cadence_mc_inject_ctrl);

static const struct of_device_id cad_ddr_ctrl_of_match[] = {
	{ .compatible = "cadence,ddr-ctrl" },
	{},
};
MODULE_DEVICE_TABLE(of, cad_ddr_ctrl_of_match);

static int cadence_mc_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;
	struct edac_mc_layer layers[2];
	struct mem_ctl_info *mci;
	struct cad_mc_drvdata *drvdata;
	struct dimm_info *dimm;
	struct resource *r;
	struct clk *clk;
	u32 val;
	int irq;
	int res = 0;

	id = of_match_device(cad_ddr_ctrl_of_match, &pdev->dev);
	if (!id)
		return -ENODEV;

	layers[0].type = EDAC_MC_LAYER_CHIP_SELECT;
	layers[0].size = 1;
	layers[0].is_virt_csrow = true;
	layers[1].type = EDAC_MC_LAYER_CHANNEL;
	layers[1].size = 1;
	layers[1].is_virt_csrow = false;
	mci = edac_mc_alloc(0, ARRAY_SIZE(layers), layers,
			    sizeof(struct cad_mc_drvdata));
	if (!mci)
		return -ENOMEM;

	mci->pdev = &pdev->dev;
	drvdata = mci->pvt_info;
	platform_set_drvdata(pdev, mci);

	if (!devres_open_group(&pdev->dev, NULL, GFP_KERNEL))
		return -ENOMEM;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r) {
		dev_err(&pdev->dev, "Unable to get mem resource\n");
		res = -ENODEV;
		goto err;
	}

	if (!devm_request_mem_region(&pdev->dev, r->start,
				     resource_size(r), dev_name(&pdev->dev))) {
		dev_err(&pdev->dev, "Error while requesting mem region\n");
		res = -EBUSY;
		goto err;
	}

	drvdata->io_base = devm_ioremap(&pdev->dev, r->start, resource_size(r));
	if (!drvdata->io_base) {
		dev_err(&pdev->dev, "Unable to map regs\n");
		res = -ENOMEM;
		goto err;
	}

	/* If we have a clock, enable it - and keep hold on it */
	clk = devm_clk_get(&pdev->dev, NULL);
	if (!IS_ERR(clk))
		clk_prepare_enable(clk);

	val = readb(drvdata->io_base + CAD_DDR_ECC_ENABLE);
	if (!val)
		mci->edac_cap = EDAC_FLAG_NONE;
	else
		mci->edac_cap = EDAC_FLAG_SECDED;

	mci->mtype_cap = MEM_FLAG_DDR2 | MEM_FLAG_DDR3;
	mci->edac_ctl_cap = EDAC_FLAG_NONE | EDAC_FLAG_SECDED;
	mci->mod_name = pdev->dev.driver->name;
	mci->ctl_name = id->compatible;
	mci->dev_name = dev_name(&pdev->dev);
	mci->scrub_mode = SCRUB_SW_SRC;

	/* Only a single 256MB DIMM is supported */
	dimm = *mci->dimms;
	dimm->nr_pages = MiB_TO_PAGES(SZ_256M);
	dimm->grain = 1;
	dimm->dtype = DEV_X8;

	val = readb(drvdata->io_base + CAD_DDR_DDR_CLASS);
	if ((val & 0xf) == 4)
		dimm->mtype = MEM_DDR2;
	else
		dimm->mtype = MEM_DDR3;
	dimm->edac_mode = EDAC_SECDED;

	res = edac_mc_add_mc(mci);
	if (res < 0)
		goto err;

	/* Mask out all interrupts */
	writel(0xfffff, drvdata->io_base + CAD_DDR_ECC_INT_MASK);

	irq = platform_get_irq(pdev, 0);
	res = devm_request_irq(&pdev->dev, irq, cadence_mc_err_handler,
			       0, dev_name(&pdev->dev), mci);
	if (res < 0) {
		dev_err(&pdev->dev, "Unable to request irq %d\n", irq);
		goto err2;
	}

	device_create_file(&mci->dev, &dev_attr_inject_ctrl);

	devres_close_group(&pdev->dev, NULL);

	/* Enable ECC error and port protection interrupts */
	writel(~(CAD_DDR_ECC_INT_STAT_CE | CAD_DDR_ECC_INT_STAT_UE |
			CAD_DDR_ECC_INT_STAT_PORT) & 0xfffff,
	       drvdata->io_base + CAD_DDR_ECC_INT_MASK);

	dev_info(&pdev->dev, "DDR EDAC (Error Detection And Correction): ECC %s\n",
		 (mci->edac_cap == EDAC_FLAG_NONE) ? "Disabled" : "Enabled");

	return 0;
err2:
	edac_mc_del_mc(&pdev->dev);
err:
	devres_release_group(&pdev->dev, NULL);
	edac_mc_free(mci);
	return res;
}

static int cadence_mc_remove(struct platform_device *pdev)
{
	struct mem_ctl_info *mci = platform_get_drvdata(pdev);

	device_remove_file(&mci->dev, &dev_attr_inject_ctrl);
	edac_mc_del_mc(&pdev->dev);
	edac_mc_free(mci);
	return 0;
}

static struct platform_driver cadence_mc_edac_driver = {
	.probe = cadence_mc_probe,
	.remove = cadence_mc_remove,
	.driver = {
		.name = "cadence_mc_edac",
		.of_match_table = cad_ddr_ctrl_of_match,
	},
};

module_platform_driver(cadence_mc_edac_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Renesas Electronics Europe Ltd.");
MODULE_DESCRIPTION("EDAC Driver for Cadence DDR Controller");
