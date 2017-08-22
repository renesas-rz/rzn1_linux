// SPDX-License-Identifier: GPL-2.0
/*
 * RZ/N1 CM3 firmware upload driver
 *
 * Copyright (C) 2017 Renesas Electronics Europe Limited
 *
 * Michel Pollet <michel.pollet@bp.renesas.com>, <buserror@gmail.com>
 */

#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/sysctrl-rzn1.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#ifdef	CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif

struct cm3 {
	struct device *dev;
	struct device_node *np;
	void __iomem *sram;
	size_t sram_size;
	dev_t first;
	struct cdev c_dev;
	struct class *cl;
	int reset_held : 1;
};

static int cm3_open(struct inode *i, struct file *f)
{
	struct cm3 *c = container_of(i->i_cdev, struct cm3, c_dev);

	f->private_data = c;

	return 0;
}

static int cm3_close(struct inode *i, struct file *f)
{
	struct cm3 *c = f->private_data;

	if (c->reset_held) {
		dev_dbg(c->dev, "CM3 restarted\n");
		/*
		 * We do not delay here, as it's assumed the open/write/close
		 * overhead will be long enough to introduce that 10us anyway.
		 * Works in practice too
		 */
		rzn1_sysctrl_writel(0x3, RZN1_SYSCTRL_REG_PWRCTRL_CM3);
		c->reset_held = 0;
	}

	return 0;
}

static ssize_t cm3_read(struct file *f, char __user *buf, size_t len, loff_t *off)
{
	int i;
	u8 byte;
	struct cm3 *c = f->private_data;

	if (*off >= c->sram_size)
		return 0;
	if (*off + len > c->sram_size)
		len = c->sram_size - *off;
	for (i = 0; i < len; i++) {
		byte = ioread8((u8 *)c->sram + *off + i);
		if (copy_to_user(buf + i, &byte, 1))
			return -EFAULT;
	}
	*off += len;

	return len;
}

static ssize_t cm3_write(struct file *f, const char __user *buf, size_t len, loff_t *off)
{
	int i;
	u8 byte;
	struct cm3 *c = f->private_data;

	if (*off == 0) {
		dev_dbg(c->dev, "reset CM3\n");
		rzn1_sysctrl_writel(0x5, RZN1_SYSCTRL_REG_PWRCTRL_CM3);
		c->reset_held = 1;
	}
	if (*off >= c->sram_size)
		return 0;
	if (*off + len > c->sram_size)
		len = c->sram_size - *off;
	for (i = 0; i < len; i++) {
		if (copy_from_user(&byte, buf + i, 1))
			return -EFAULT;
		iowrite8(byte, (u8 *)c->sram + *off + i);
	}
	*off += len;

	return len;
}

static const struct file_operations cm3_fops = {
	.owner = THIS_MODULE,
	.open = cm3_open,
	.release = cm3_close,
	.read = cm3_read,
	.write = cm3_write,
};

/* this is called when the firmware request is complete (even if failed) */
static void cm3_firmware_loaded(const struct firmware *fw, void *context)
{
	struct cm3 *c = context;
	int i;

	if (!fw)	/* failed to load a firmware */
		return;

	/*
	 * Not entirely sure why, but it apperars the SYSCTRL register
	 * write do not work early in boot time, perhaps the underlying API
	 * isn't ready, or something. Anyway, the firmware gets loaded
	 */
	rzn1_sysctrl_writel(0x5, RZN1_SYSCTRL_REG_PWRCTRL_CM3);
	for (i = 0; i < fw->size; i++)
		iowrite8(fw->data[i], (u8 *)c->sram + i);
	dev_info(c->dev, "Firmware loaded (%d bytes)\n", fw->size);
	udelay(10); /* required for a proper reset pulse */
	rzn1_sysctrl_writel(0x3, RZN1_SYSCTRL_REG_PWRCTRL_CM3);

	release_firmware(fw);
}

#ifdef	CONFIG_DEBUG_FS

static int cm3_power_show(void *data, uint64_t *value)
{
	*value = rzn1_sysctrl_readl(RZN1_SYSCTRL_REG_PWRCTRL_CM3);
	return 0;
}

static int cm3_power_set(void *data, uint64_t value)
{
	rzn1_sysctrl_writel(value, RZN1_SYSCTRL_REG_PWRCTRL_CM3);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(cm3_power_reg,
			cm3_power_show,
	cm3_power_set,
	"%llx\n");

static int __init cm3_debug_init(void)
{
	struct dentry *dir = debugfs_create_dir("cm3", NULL);

	debugfs_create_file("power", 0640, dir, NULL,
			    &cm3_power_reg);

	return 0;
}

late_initcall(cm3_debug_init);
#endif /* CONFIG_DEBUG_FS */

static int cm3_probe(struct platform_device *ofdev)
{
	int ret = -EINVAL;
	struct resource *res;
	struct device *dev = &ofdev->dev;
	struct device_node *np = ofdev->dev.of_node;
	struct cm3 *c;
	const char *str;

	c = devm_kzalloc(dev, sizeof(*c), GFP_KERNEL);
	if (!c)
		return -ENOMEM;
	c->dev = dev;
	c->np = np;

	res = platform_get_resource(ofdev, IORESOURCE_MEM, 0);
	c->sram = devm_ioremap_resource(c->dev, res);
	if (IS_ERR(c->sram))
		return PTR_ERR(c->sram);
	c->sram_size = resource_size(res);
	if (alloc_chrdev_region(&c->first, 0, 1, "cm3") < 0)
		goto error_chdev;
	c->cl = class_create(THIS_MODULE, "chardrv");
	if (IS_ERR(c->cl))
		goto error_chdev;
	if (!device_create(c->cl, NULL, c->first, NULL, "cm3"))
		goto error_dev;

	cdev_init(&c->c_dev, &cm3_fops);
	if (cdev_add(&c->c_dev, c->first, 1) == -1)
		goto error_add;

	platform_set_drvdata(ofdev, c);

	ret = device_property_read_string(c->dev, "firmware-name", &str);
	/* do we have an optional firmware to load? */
	if (ret == 0) {
		if (request_firmware_nowait(THIS_MODULE, 1, str, dev,
					    GFP_KERNEL, c, cm3_firmware_loaded)) {
			/* this is not really an error */
			dev_warn(c->dev, "Firmware %s load failed\n", str);
		}
	}

	dev_info(c->dev, "Started.\n");

	return 0;
error_add:
	device_destroy(c->cl, c->first);
error_dev:
	class_destroy(c->cl);
error_chdev:
	unregister_chrdev_region(c->first, 1);
	return ret;
}

static int cm3_remove(struct platform_device *pdev)
{
	struct cm3 *c = platform_get_drvdata(pdev);

	if (!c)
		return 0;
	cdev_del(&c->c_dev);
	device_destroy(c->cl, c->first);
	class_destroy(c->cl);
	unregister_chrdev_region(c->first, 1);

	return 0;
}

/*-------------------------------------------------------------------------*/
static const struct of_device_id cm3_match[] = {
	{ .compatible = "renesas,rzn1-cm3", },
	{},
};

MODULE_DEVICE_TABLE(of, cm3_match);

static struct platform_driver cm3_driver = {
	.driver = {
		.name = "rzn1_cm3",
		.owner = THIS_MODULE,
		.of_match_table = cm3_match,
	},
	.probe          = cm3_probe,
	.remove         = cm3_remove,
};

module_platform_driver(cm3_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Michel Pollet <michel.pollet@bp.renesas.com>, <buserror@gmail.com>");
MODULE_DESCRIPTION("RZ/N1 CM3 Firmware Driver");
