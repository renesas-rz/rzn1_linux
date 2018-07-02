/*
 * Renesas RZN1 Real Time Clock interface for Linux
 *
 * Copyright (C) 2014 Renesas Electronics Europe Limited
 *
 * Michel Pollet <michel.pollet@bp.renesas.com>, <buserror@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/io.h>

#define	DRIVER_NAME			"rzn1-rtc"

/*
 * Auto-generated from RTC_SUB_ipxact.xml
 */
#define RZN1_RTC_CTL0				0x00
#define RZN1_RTC_CTL0_SLSB				4
#define RZN1_RTC_CTL0_AMPM				5
#define RZN1_RTC_CTL0_CEST				6
#define RZN1_RTC_CTL0_CE				7
#define RZN1_RTC_CTL1				0x04
#define RZN1_RTC_CTL1_CT0				0
#define RZN1_RTC_CTL1_CT1				1
#define RZN1_RTC_CTL1_CT2				2
#define RZN1_RTC_CTL1_1SE				3
#define RZN1_RTC_CTL1_ALME				4
#define RZN1_RTC_CTL1_1HZE				5
#define RZN1_RTC_CTL2				0x08
#define RZN1_RTC_CTL2_WAIT				0
#define RZN1_RTC_CTL2_WST				1
#define RZN1_RTC_CTL2_RSUB				2
#define RZN1_RTC_CTL2_RSST				3
#define RZN1_RTC_CTL2_WSST				4
#define RZN1_RTC_CTL2_WUST				5
#define RZN1_RTC_SUBC				0x0c
#define RZN1_RTC_SRBU				0x10
#define RZN1_RTC_SEC				0x14
#define RZN1_RTC_MIN				0x18
#define RZN1_RTC_HOUR				0x1c
#define RZN1_RTC_WEEK				0x20
#define RZN1_RTC_DAY				0x24
#define RZN1_RTC_MONTH				0x28
#define RZN1_RTC_YEAR				0x2c
#define RZN1_RTC_TIME				0x30
#define RZN1_RTC_TIME_SEC				0
#define RZN1_RTC_TIME_SEC_MASK				0xff
#define RZN1_RTC_TIME_MIN				8
#define RZN1_RTC_TIME_MIN_MASK				0xff00
#define RZN1_RTC_TIME_HOUR				16
#define RZN1_RTC_TIME_HOUR_MASK				0xff0000
#define RZN1_RTC_CAL				0x34
#define RZN1_RTC_CAL_WEEK				0
#define RZN1_RTC_CAL_WEEK_MASK				0xff
#define RZN1_RTC_CAL_DAY				8
#define RZN1_RTC_CAL_DAY_MASK				0xff00
#define RZN1_RTC_CAL_MONTH				16
#define RZN1_RTC_CAL_MONTH_MASK				0xff0000
#define RZN1_RTC_CAL_YEAR				24
#define RZN1_RTC_CAL_YEAR_MASK				0xff000000
#define RZN1_RTC_SUBU				0x38
#define RZN1_RTC_SUBU_F0				0
#define RZN1_RTC_SUBU_F1				1
#define RZN1_RTC_SUBU_F2				2
#define RZN1_RTC_SUBU_F3				3
#define RZN1_RTC_SUBU_F4				4
#define RZN1_RTC_SUBU_F5				5
#define RZN1_RTC_SUBU_F6				6
#define RZN1_RTC_SUBU_DEV				7
#define RZN1_RTC_SCMP				0x3c
#define RZN1_RTC_ALM				0x40
#define RZN1_RTC_ALH				0x44
#define RZN1_RTC_ALW				0x48
#define RZN1_RTC_ALW_ALW0				0
#define RZN1_RTC_ALW_ALW1				1
#define RZN1_RTC_ALW_ALW2				2
#define RZN1_RTC_ALW_ALW3				3
#define RZN1_RTC_ALW_ALW4				4
#define RZN1_RTC_ALW_ALW5				5
#define RZN1_RTC_ALW_ALW6				6
#define RZN1_RTC_SECC				0x4c
#define RZN1_RTC_MINC				0x50
#define RZN1_RTC_HOURC				0x54
#define RZN1_RTC_WEEKC				0x58
#define RZN1_RTC_DAYC				0x5c
#define RZN1_RTC_MONTHC				0x60
#define RZN1_RTC_YEARC				0x64
#define RZN1_RTC_TIMEC				0x68
#define RZN1_RTC_TIMEC_SECC				0
#define RZN1_RTC_TIMEC_SECC_MASK			0xff
#define RZN1_RTC_TIMEC_MINC				8
#define RZN1_RTC_TIMEC_MINC_MASK			0xff00
#define RZN1_RTC_TIMEC_HOURC				16
#define RZN1_RTC_TIMEC_HOURC_MASK			0xff0000
#define RZN1_RTC_CALC				0x6c
#define RZN1_RTC_CALC_WEEKC				0
#define RZN1_RTC_CALC_WEEKC_MASK			0xff
#define RZN1_RTC_CALC_DAYC				8
#define RZN1_RTC_CALC_DAYC_MASK				0xff00
#define RZN1_RTC_CALC_MONC				16
#define RZN1_RTC_CALC_MONC_MASK				0xff0000
#define RZN1_RTC_CALC_YEARC				24
#define RZN1_RTC_CALC_YEARC_MASK			0xff000000
#define RZN1_RTC_TCR				0x70
#define RZN1_RTC_TCR_OS0				0
#define RZN1_RTC_TCR_OS1				1
#define RZN1_RTC_TCR_OS2				2
#define RZN1_RTC_TCR_OS3				3
#define RZN1_RTC_TCR_OSE				15
#define RZN1_RTC_EMU				0x74
#define RZN1_RTC_EMU_SVSDIS				7

static void __iomem	*rtc_base;

#define rtc_read(addr)		readl(rtc_base + (addr))
#define rtc_write(val, addr)	writel(val, rtc_base + (addr))

static irqreturn_t rtc_irq(int irq, void *_rtc)
{
	struct rtc_device *rtc = _rtc;

	rtc_update_irq(rtc, 1, RTC_IRQF | RTC_AF);

	return IRQ_HANDLED;
}

/* this hardware doesn't support "don't care" alarm fields */
static int tm2bcd(struct rtc_time *tm)
{
	if (rtc_valid_tm(tm) != 0)
		return -EINVAL;

	tm->tm_sec = bin2bcd(tm->tm_sec);
	tm->tm_min = bin2bcd(tm->tm_min);
	tm->tm_hour = bin2bcd(tm->tm_hour);
	tm->tm_mday = bin2bcd(tm->tm_mday);
	tm->tm_mon = bin2bcd(tm->tm_mon + 1);
	/* epoch == 1900 */
	tm->tm_year = bin2bcd(tm->tm_year - 100);

	return 0;
}

static void bcd2tm(struct rtc_time *tm)
{
	tm->tm_sec = bcd2bin(tm->tm_sec);
	tm->tm_min = bcd2bin(tm->tm_min);
	tm->tm_hour = bcd2bin(tm->tm_hour);
	tm->tm_mday = bcd2bin(tm->tm_mday);
	tm->tm_mon = bcd2bin(tm->tm_mon) - 1;
	/* epoch == 1900 */
	tm->tm_year = bcd2bin(tm->tm_year) + 100;
}

static int rzn1_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	int i, last = -1;

	/* read all registers, we have to, as the timer stops when we read the
	 * RTC_SEC and restarts after RTC_YEAR.
	 * So we loop 3 times, and as soon as the seconds are the same as they
	 * were in the previous pass, we stop as we can guarantee that the
	 * stamp we have is correct */
	for (i = 0; i < 3; i++) {
		tm->tm_sec = rtc_read(RZN1_RTC_SECC);
		tm->tm_min = rtc_read(RZN1_RTC_MINC);
		tm->tm_hour = rtc_read(RZN1_RTC_HOURC);
		tm->tm_mday = rtc_read(RZN1_RTC_DAYC);
		tm->tm_mon = rtc_read(RZN1_RTC_MONTHC);
		tm->tm_year = rtc_read(RZN1_RTC_YEARC);
#ifdef DEBUG
		printk("%s:%d:%d %d %d %d %d %d %d\n", __func__, __LINE__, i,
			tm->tm_sec,tm->tm_min,tm->tm_hour,tm->tm_mday,
			tm->tm_mon,tm->tm_year);
#endif
		if (tm->tm_sec == last)
			break;
		last = tm->tm_sec;
	};

	bcd2tm(tm);
	return 0;
}

static int rzn1_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	u32 val;
	u32 timeout = 100000;

#ifdef DEBUG
	printk("%s:%d %d %d %d %d %d %d\n", __func__, __LINE__,
		tm->tm_sec,tm->tm_min,tm->tm_hour,tm->tm_mday,
		tm->tm_mon,tm->tm_year);
#endif
	if (tm2bcd(tm) < 0)
		return -EINVAL;

	local_irq_disable();

	/* Disable the counter */
	val = rtc_read(RZN1_RTC_CTL2);
	val |= (1 << RZN1_RTC_CTL2_WAIT);
	rtc_write(val, RZN1_RTC_CTL2);

	/* Wait for the counter to stop */
	while (((rtc_read(RZN1_RTC_CTL2) & (1 << RZN1_RTC_CTL2_WST)) == 0) && timeout)
		timeout--;
#ifdef DEBUG
	printk("%s:%d %d %d %d %d %d %d (timout=%d)\n", __func__, __LINE__,
		tm->tm_sec,tm->tm_min,tm->tm_hour,tm->tm_mday,
		tm->tm_mon,tm->tm_year, timeout);
#endif

	rtc_write(tm->tm_year, RZN1_RTC_YEAR);
	rtc_write(tm->tm_mon, RZN1_RTC_MONTH);
	rtc_write(tm->tm_mday, RZN1_RTC_DAY);
	rtc_write(tm->tm_hour, RZN1_RTC_HOUR);
	rtc_write(tm->tm_min, RZN1_RTC_MIN);
	rtc_write(tm->tm_sec, RZN1_RTC_SEC);

	/* Enable the counter */
	val &= ~(1 << RZN1_RTC_CTL2_WAIT);
	rtc_write(val, RZN1_RTC_CTL2);

	local_irq_enable();

	return 0;
}

static struct rtc_class_ops rzn1_rtc_ops = {
	.read_time	= rzn1_rtc_read_time,
	.set_time	= rzn1_rtc_set_time,
};

static int rzn1_rtc_alarm;
static int rzn1_rtc_timer;


static const struct of_device_id rzn1_rtc_of_match[] = {
	{ .compatible	= "renesas,rzn1-rtc",	},
	{},
};
MODULE_DEVICE_TABLE(of, rzn1_rtc_of_match);

static int rzn1_rtc_probe(struct platform_device *pdev)
{
	struct resource		*res;
	struct rtc_device	*rtc;
	struct clk 		*clk;

	clk = devm_clk_get(&pdev->dev, "axi");
	if (IS_ERR(clk) || clk_prepare_enable(clk))
		dev_info(&pdev->dev, "no clock source\n");

	rzn1_rtc_timer = platform_get_irq(pdev, 0);
	if (rzn1_rtc_timer <= 0) {
		dev_dbg(&pdev->dev, "no update irq?\n");
		return -ENOENT;
	}

	rzn1_rtc_alarm = platform_get_irq(pdev, 1);
	if (rzn1_rtc_alarm <= 0) {
		dev_dbg(&pdev->dev, "no alarm irq?\n");
		return -ENOENT;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	rtc_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(rtc_base))
		return PTR_ERR(rtc_base);

	/* force to 24 hour mode and enable clock counter */
	rtc_write((1 << RZN1_RTC_CTL0_CE) |
			(1 << RZN1_RTC_CTL0_AMPM), RZN1_RTC_CTL0);

	/* Enable the counter */
	rtc_write(0, RZN1_RTC_CTL2);

	rtc = devm_rtc_device_register(&pdev->dev, pdev->name,
			&rzn1_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc)) {
		dev_dbg(&pdev->dev, "can't register RTC device, err %ld\n",
			PTR_ERR(rtc));
		return PTR_ERR(rtc);
	}
	platform_set_drvdata(pdev, rtc);

	/* clear pending irqs */
	rtc_write(0, RZN1_RTC_CTL1);

	/* handle periodic and alarm irqs */
	if (devm_request_irq(&pdev->dev, rzn1_rtc_timer, rtc_irq, 0,
			dev_name(&rtc->dev), rtc)) {
		dev_dbg(&pdev->dev, "RTC timer interrupt IRQ%d already claimed\n",
			rzn1_rtc_timer);
		return -EIO;
	}

	return 0;
}

static struct platform_driver rzn1_rtc_driver = {
	.probe		= rzn1_rtc_probe,
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = rzn1_rtc_of_match,
	},
};

module_platform_driver(rzn1_rtc_driver);

MODULE_AUTHOR("Michel Pollet <Michel.Pollet@bp.renesas.com");
MODULE_DESCRIPTION("RZN1 RTC driver");
MODULE_LICENSE("GPL");
