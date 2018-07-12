/*
 * linux/drivers/video/db9000fb.c
 * -- Digital Blocks DB9000 LCD Controller Frame Buffer Device
 * Copyright (C) 2016 Renesas Electronics Europe Ltd.
 * Copyright (C) 2010 Digital Blocks, Inc.
 *
 * Based on pxafb.c and amba-clcd.c which is:
 * Copyright (C) 1999 Eric A. Thomas.
 * Copyright (C) 2004 Jean-Frederic Clere.
 * Copyright (C) 2004 Ian Campbell.
 * Copyright (C) 2004 Jeff Lackey.
 * Based on sa1100fb.c Copyright (C) 1999 Eric A. Thomas
 * which in turn is
 * Based on acornfb.c Copyright (C) Russell King.
 * Copyright (C) 2001 ARM Limited, by David A Rusling
 * Updated to 2.5, Deep Blue Solutions Ltd.
 *
 * 2010-05-01: Guy Winter <gwinter@digitalblocks.com>
 * - ported pxafb and some amba-clcd code to DB9000
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive for
 * more details.
 *
 */

#include <linux/backlight.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/fb.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <video/display_timing.h>
#include <video/of_display_timing.h>
#include "db9000fb.h"

#define DRIVER_NAME "clcd-db9000"
#define DEF_BRIGHTNESS 0x80

/* Bits which should not be set in machine configuration structures */
#define CR1_INVALID_CONFIG_MASK	(~(DB9000_CR1_ENB | DB9000_CR1_LPE |\
			DB9000_CR1_BPP(7) | DB9000_CR1_RGB | DB9000_CR1_EPO |\
			DB9000_CR1_EBO | DB9000_CR1_DEP | DB9000_CR1_PCP |\
			DB9000_CR1_HSP | DB9000_CR1_VSP | DB9000_CR1_OPS(7) |\
			DB9000_CR1_FDW(3) | DB9000_CR1_LPS |\
			DB9000_CR1_FBP | DB9000_CR1_DEE))

#define DB9000_IMR_MASK_ALL (DB9000_ISR_OFUM | DB9000_ISR_OFOM |\
			DB9000_ISR_IFUM | DB9000_ISR_IFOM | DB9000_ISR_FERM |\
			DB9000_ISR_MBEM | DB9000_ISR_VCTM | DB9000_ISR_BAUM |\
			DB9000_ISR_LDDM | DB9000_ISR_ABLM | DB9000_ISR_ARIM |\
			DB9000_ISR_ARSM | DB9000_ISR_FBEM | DB9000_ISR_FNCM |\
			DB9000_ISR_FLCM)

/* These are the lcd controller states & actions for set_ctrlr_state */
enum {
	C_DISABLE = 0,
	C_ENABLE,
	C_DISABLE_CLKCHANGE,
	C_ENABLE_CLKCHANGE,
	C_REENABLE,
	C_DISABLE_PM,
	C_ENABLE_PM,
	C_STARTUP,
};

#define LCD_PCLK_EDGE_RISE	(0 << 9)
#define LCD_PCLK_EDGE_FALL	(1 << 9)


#define NUM_OF_FRAMEBUFFERS	2
#define PALETTE_SIZE		(128 * 4)


struct db9000fb_info {
	struct fb_info		*info;
	struct device		*dev;
	struct platform_device	*pdev;
	struct clk		*clk;
	struct clk		*bus_clk;
	struct clk		*pixel_clk;
	struct notifier_block	clk_rate_nb;

	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pins_default;
	struct pinctrl_state	*pins_sleep;
	bool			clk_enabled;

	atomic_t		usage;

	/* raw memory addresses */
	unsigned long		hsync_time;
	unsigned long		pseudo_palette[16];
/* size of the one frame buffer */
	size_t			frame_size;
/* virtual address of palette memory */
	u_int			palette_size;
	void __iomem            *mmio_base;

/* Local images/copies of device registers */
	u32			reg_cr1;
	u32			reg_htr;
	u32			reg_hvter;
	u32			reg_vtr1;
	u32			reg_vtr2;
	u32			reg_pctr;
	u32			reg_dbar;
	u32			reg_dear;
	u32			reg_pwmfr;
	u32			reg_pwmdcr;

	u32			palette[PALETTE_SIZE/4];

	u32			buswidth;
	u_char			state;
	u_char			old_state;
	u_char			task_state;
	u16			db9000_rev;
	struct mutex		ctrlr_lock;
	wait_queue_head_t	ctrlr_wait;
	struct work_struct	task;

	/* Completion - for PAN display alignment with VSYNC/BAU event */
	struct completion vsync_notifier;

#ifdef CONFIG_BACKLIGHT_DB9000_LCD
	struct backlight_device *backlight;
	u32			pwm_clock;
	u8			bl_power;
#endif

	bool			use_blinking;
};

#define TO_INF(ptr, member)	container_of(ptr, struct db9000fb_info, member)

static inline void db9000fb_backlight_power(struct db9000fb_info *fbi, int on);
static inline void db9000fb_lcd_power(struct db9000fb_info *fbi, int on);
static int db9000fb_activate_var(struct fb_var_screeninfo *var,
		struct db9000fb_info *fbi);
static void set_ctrlr_state(struct db9000fb_info *fbi, u_int state);

static inline u32
lcd_readl(struct db9000fb_info *fbi, unsigned int off)
{
	return readl(fbi->mmio_base + off);
}

static inline void
lcd_writel(struct db9000fb_info *fbi, unsigned int off, u32 val)
{
	writel(val, fbi->mmio_base + off);
}

static inline void
db9000fb_schedule_work(struct db9000fb_info *fbi, u_int state)
{
	unsigned long flags;

	local_irq_save(flags);
	/*
	 * We need to handle two requests being made at the same time.
	 * There are two important cases:
	 * 1. When we are changing VT (C_REENABLE) while unblanking (C_ENABLE)
	 *	We must perform the unblanking, which will do our REENABLE for
	 *	us.
	 * 2. When we are blanking, but immediately unblank before we have
	 *	blanked. We do the "REENABLE" thing here as well, just to be
	 *	sure.
	 */
	if (fbi->task_state == C_ENABLE && state == C_REENABLE)
		state = (u_int) -1;

	if (fbi->task_state == C_DISABLE && state == C_ENABLE)
		state = C_REENABLE;

	if (state != (u_int)-1) {
		fbi->task_state = state;
		schedule_work(&fbi->task);
	}

	local_irq_restore(flags);
}

static inline u_int convert_bitfield(u_int val, struct fb_bitfield *bf)
{
	unsigned int mask = (1 << bf->length) - 1;

	return (val >> (16 - bf->length) & mask) << bf->offset;
}

static int
db9000fb_setpalettereg(u_int regno, u_int red, u_int green, u_int blue, u_int
		trans, struct fb_info *info)
{
	struct db9000fb_info *fbi = info->par;
	u_int val;
	u16 *pal = (u16 *)&fbi->palette[0];
	u32 *pal32 = (u32 *)&fbi->palette[0];

	if (regno >= fbi->palette_size)
		return 1;

	if (info->var.grayscale) {
		pal[regno] = ((blue >> 8) & 0x00ff);
		return 0;
	}

	/* red and blue swapped */
	if (fbi->reg_cr1 & DB9000_CR1_RGB) {
		val = red;
		red = blue;
		blue = val;
	}

	if ((fbi->reg_cr1 & DB9000_CR1_OPS(5)) == DB9000_CR1_OPS(1)) {
		/* RGB, 5:5:5 format */
		val = ((red >> 1) & 0x7c00);
		val |= ((green >> 6) & 0x03e0);
		val |= ((blue >> 11) & 0x001f);
	} else {
		/* RGB, 5:6:5 format */
		val = ((red >> 0) & 0xf800);
		val |= ((green >> 5) & 0x07e0);
		val |= ((blue >> 11) & 0x001f);
	}
	pal[regno] = val;

	/* Can only write to two palette entries together */
	lcd_writel(fbi, DB9000_PALT + (regno & ~1)*2, pal32[regno/2]);

	return 0;
}

static int
db9000fb_setcolreg(u_int regno, u_int red, u_int green, u_int blue, u_int trans,
		struct fb_info *info)
{
	unsigned int val;
	int ret = 1;

	/*
	 * If greyscale is true, then we convert the RGB value to greyscale no
	 * matter what visual we are using.
	 */
	if (info->var.grayscale)
		red = green = blue = (19595 * red + 38470 * green +
				7471 * blue) >> 16;

	switch (info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		/*
		 * 16-bit True Colour. We encode the RGB value according to the
		 * RGB bitfield information.
		 */
		if (regno < 16) {
			u32 *pal = info->pseudo_palette;

			val = convert_bitfield(red, &info->var.red);
			val |= convert_bitfield(green, &info->var.green);
			val |= convert_bitfield(blue, &info->var.blue);

			pal[regno] = val;
			ret = 0;
		}
		break;

	case FB_VISUAL_PSEUDOCOLOR:
		ret = db9000fb_setpalettereg(regno, red, green, blue, trans,
				info);
		break;

	case FB_VISUAL_MONO01:
		if (regno == 0)
			ret = db9000fb_setpalettereg(0, 0, 0, 0, 0, info);
		else if (regno == 1)
			ret = db9000fb_setpalettereg(1, 0xff, 0xff, 0xff, 0xff,
					info);
		break;
	}

	return ret;
}

/*
 * db9000fb_bpp_to_cr1():
 *	Convert a bits per pixel value to the correct bit pattern for CR1
 */
static int db9000fb_bpp_to_cr1(struct fb_var_screeninfo *var)
{
	int ret = 0;

	switch (var->bits_per_pixel) {
	case 1:
		ret = DB9000_CR1_BPP(DB9000_CR1_BPP_1bpp);
		break;
	case 2:
		ret = DB9000_CR1_BPP(DB9000_CR1_BPP_2bpp);
		break;
	case 4:
		ret = DB9000_CR1_BPP(DB9000_CR1_BPP_4bpp);
		break;
	case 8:
		ret = DB9000_CR1_BPP(DB9000_CR1_BPP_8bpp);
		break;
	case 16:
		ret = DB9000_CR1_BPP(DB9000_CR1_BPP_16bpp);
		break;
	case 18:
		ret = DB9000_CR1_BPP(DB9000_CR1_BPP_18bpp);
		break;
	case 24:
	default:
		ret = DB9000_CR1_BPP(DB9000_CR1_BPP_24bpp);
	}

	return ret;
}

static int
db9000fb_check_var_in(struct fb_var_screeninfo *var)
{
	if (var->xres < 16 || var->xres > 4096 || (var->xres % 16)) {
		pr_err("invalid xres %d\n", var->xres);
		return -EINVAL;
	}

	switch (var->bits_per_pixel) {
	case 1:
	case 2:
	case 4:
	case 8:
	case 16:
	case 18:
	case 24:
	case 32:
		break;
	default:
		pr_err("invalid bit depth %d\n", var->bits_per_pixel);
		return -EINVAL;
	}

	if (var->hsync_len < 0 || var->hsync_len > 255) {
		pr_err("invalid hsync_len %d\n", var->hsync_len);
		return -EINVAL;
	}
	if (var->left_margin < 0 || var->left_margin > 1023) {
		pr_err("invalid left_margin %d\n", var->left_margin);
		return -EINVAL;
	}
	if (var->right_margin < 0 || var->right_margin > 1023) {
		pr_err("invalid right_margin %d\n", var->right_margin);
		return -EINVAL;
	}
	if (var->yres < 0 || var->yres > 4096) {
		pr_err("invalid yres %d\n", var->yres);
		return -EINVAL;
	}
	if (var->vsync_len < 0 || var->vsync_len > 255) {
		pr_err("invalid vsync_len %d\n", var->vsync_len);
		return -EINVAL;
	}
	if (var->upper_margin < 0 || var->upper_margin > 1023) {
		pr_err("invalid upper_margin %d\n", var->upper_margin);
		return -EINVAL;
	}
	if (var->lower_margin < 0 || var->lower_margin > 1023) {
		pr_err("invalid lower_margin %d\n", var->lower_margin);
		return -EINVAL;
	}
	if (var->pixclock <= 0) {
		pr_err("invalid pixel clock %d\n", var->pixclock);
		return -EINVAL;
	}
	return 0;
}
/*
 * db9000fb_check_var():
 *	Get the video params out of 'var'. If a value doesn't fit, round it up,
 *	if it's too big, return -EINVAL.
 *
 * Round up in the following order: bits_per_pixel, xres, yres, xres_virtual,
 * yres_virtual, xoffset, yoffset, grayscale, bitfields, horizontal timing,
 * vertical timing.
 */
static int
db9000fb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct db9000fb_info *fbi = info->par;

	if (db9000fb_check_var_in(var) < 0)
		return -EINVAL;

	switch (var->bits_per_pixel) {
	case 1:
	case 2:
	case 4:
	case 8:
		var->red.offset		= var->green.offset = 0;
		var->blue.offset	= var->transp.offset = 0;
		var->red.length		= var->bits_per_pixel;
		var->green.length	= var->bits_per_pixel;
		var->blue.length	= var->bits_per_pixel;
		var->transp.length	= 0;
		break;
	case 16:
		var->red.offset		= 11;
		var->red.length		= 5;
		var->green.offset	= 5;
		var->green.length	= 6;
		var->blue.offset	= 0;
		var->blue.length	= 5;
		var->transp.offset	= var->transp.length = 0;
		break;
	case 18: /* RGB666 */
		var->transp.offset	= var->transp.length = 0;
		var->red.offset		= 12;
		var->red.length		= 6;
		var->green.offset	= 6;
		var->green.length	= 6;
		var->blue.offset	= 0;
		var->blue.length	= 6;
		break;
	case 24: /* RGB888 */
		if (!fbi->use_blinking) {
			var->transp.offset	= var->transp.length = 0;
			var->red.offset		= 16;
			var->red.length		= 8;
			var->green.offset	= 8;
			var->green.length	= 8;
			var->blue.offset	= 0;
			var->blue.length	= 8;
		} else {
			/* Blinking attributes abuse the transparency field */
			var->transp.offset	= 22;
			var->transp.length	= 2;
			var->red.offset		= 15;
			var->red.length		= 7;
			var->green.offset	= 7;
			var->green.length	= 8;
			var->blue.offset	= 0;
			var->blue.length	= 7;
		}
		break;
	case 32: /* RGB888 */
		var->transp.offset	= 24;
		var->transp.length	= 0;
		var->red.offset		= 16;
		var->red.length		= 8;
		var->green.offset	= 8;
		var->green.length	= 8;
		var->blue.offset	= 0;
		var->blue.length	= 8;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static inline void db9000fb_set_truecolor(u_int is_true_color)
{
	/* do your machine-specific setup if needed */
}

#ifdef CONFIG_BACKLIGHT_DB9000_LCD
static int db9000_bl_update_status(struct backlight_device *bl)
{
	struct db9000fb_info *fbi = bl_get_data(bl);
	int power = fbi->bl_power;
	int brightness = bl->props.brightness;

	if (bl->props.fb_blank != fbi->bl_power)
		power =	bl->props.fb_blank;
	else if (bl->props.power != fbi->bl_power)
		power =	bl->props.power;

	if (power == FB_BLANK_UNBLANK && brightness < 0)
		brightness = lcd_readl(fbi, DB9000_PWMDCR);
	else if (power != FB_BLANK_UNBLANK)
		brightness = 0;

	/* If turning off, save current brightness */
	if (power)
		fbi->reg_pwmdcr = lcd_readl(fbi, DB9000_PWMDCR);

	lcd_writel(fbi, DB9000_PWMDCR, brightness);
	bl->props.fb_blank = bl->props.power = fbi->bl_power = power;

	return 0;
}

static int db9000_bl_get_brightness(struct backlight_device *bl)
{
	struct db9000fb_info *fbi = bl_get_data(bl);

	return lcd_readl(fbi, DB9000_PWMDCR);
}

static const struct backlight_ops db9000_lcd_bl_ops = {
	.update_status = db9000_bl_update_status,
	.get_brightness	= db9000_bl_get_brightness,
};

static void init_backlight(struct db9000fb_info *fbi)
{
	struct backlight_device *bl;
	struct backlight_properties props = {0, };

	if (fbi->backlight)
		return;

	/* Has a PWM clock been specified? */
	if (!fbi->pwm_clock)
		return;

	fbi->bl_power = FB_BLANK_UNBLANK;
	props.brightness = 0xff;
	props.max_brightness = 0xff;
	props.type = BACKLIGHT_RAW;
	bl = devm_backlight_device_register(fbi->dev, "backlight", fbi->dev,
			fbi, &db9000_lcd_bl_ops, &props);

	if (IS_ERR(bl)) {
		dev_err(fbi->dev, "error %ld on backlight register\n",
				PTR_ERR(bl));
		return;
	}

	fbi->backlight = bl;

	bl->props.power = FB_BLANK_UNBLANK;
	bl->props.fb_blank = FB_BLANK_UNBLANK;
	backlight_update_status(bl);
}
#endif

/*
 * db9000fb_set_par():
 *	Set the user defined part of the display for the specified console
 */
static int db9000fb_set_par(struct fb_info *info)
{
	struct db9000fb_info *fbi = info->par;

	info->var.xres_virtual = info->var.xres;
	info->var.yres_virtual = info->var.yres *
		NUM_OF_FRAMEBUFFERS;

	if (info->var.bits_per_pixel == 1)
		info->fix.visual = FB_VISUAL_MONO01;
	else if (info->var.bits_per_pixel <= 8)
		info->fix.visual = FB_VISUAL_PSEUDOCOLOR;
	else
		info->fix.visual = FB_VISUAL_TRUECOLOR;

	info->fix.line_length = (info->var.xres_virtual *
					info->var.bits_per_pixel) / 8;
	if (info->var.bits_per_pixel >= 16)
		fbi->palette_size = 0;
	else
		fbi->palette_size = 1 << info->var.bits_per_pixel;

	/* Set (any) board control register to handle new color depth */
	db9000fb_set_truecolor(info->fix.visual == FB_VISUAL_TRUECOLOR);

	db9000fb_activate_var(&info->var, fbi);

	return 0;
}

/*
 * db9000fb_blank():
 *	Blank the display by setting all palette values to zero. Note, the 16
 *	bpp mode does not really use the palette, so this will not blank the
 *	display in all modes.
 */
static int db9000fb_blank(int blank, struct fb_info *info)
{
	struct db9000fb_info *fbi = info->par;
	int i;

	switch (blank) {
	case FB_BLANK_POWERDOWN:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_NORMAL:
		if (info->fix.visual == FB_VISUAL_PSEUDOCOLOR) {
			for (i = 0; i < fbi->palette_size; i++)
				db9000fb_setpalettereg(i, 0, 0, 0, 0, info);
		}

		db9000fb_schedule_work(fbi, C_DISABLE);
	/* TODO: if (db9000fb_blank_helper) db9000fb_blank_helper(blank); */
		break;

	case FB_BLANK_UNBLANK:
	/* TODO: if (db9000fb_blank_helper) db9000fb_blank_helper(blank); */
		if (info->fix.visual == FB_VISUAL_PSEUDOCOLOR)
			fb_set_cmap(&info->cmap, info);
		db9000fb_schedule_work(fbi, C_ENABLE);
	}
	return 0;
}

static int db9000fb_open(struct fb_info *info, int user)
{
	struct db9000fb_info *fbi = info->par;

	/* Enable Controller only if its uses is zero*/
	if (atomic_inc_return(&fbi->usage) == 1)
		set_ctrlr_state(fbi, C_ENABLE);

	return 0;
}

static int db9000fb_release(struct fb_info *info, int user)
{
	struct db9000fb_info *fbi = info->par;

	if (atomic_dec_and_test(&fbi->usage))
		set_ctrlr_state(fbi, C_DISABLE);

	return 0;
}

/* Pan the display if device supports it. */
static int db9000fb_pan_display(struct fb_var_screeninfo *var,
	struct fb_info *info)
{
	struct db9000fb_info *fbi = info->par;
	u32 frame_addr;
	u_int y_bottom = var->yoffset;

	if (!(var->vmode & FB_VMODE_YWRAP))
		y_bottom += var->yres;

	BUG_ON(y_bottom > var->yres_virtual);

	frame_addr = info->fix.smem_start +
		(var->yoffset * info->fix.line_length);

	/* There are some probing calls with no buffer switch */
	if (fbi->reg_dbar != frame_addr) {
		u32 imr;

		lcd_writel(fbi, DB9000_DBAR, frame_addr);
		lcd_writel(fbi, DB9000_DEAR, frame_addr + fbi->frame_size);
		lcd_writel(fbi, DB9000_MRR,
			DB9000_MRR_DEAR_MRR(frame_addr + fbi->frame_size) |
			DB9000_MRR_MRR(DB9000_MRR_OUTST_4));

		/* Enable Base Address Update interrupt */
		imr = lcd_readl(fbi, DB9000_IMR);
		lcd_writel(fbi, DB9000_IMR, imr | DB9000_ISR_BAU);
		/*
		 * Force waiting till the current buffer is completely drawn by
		 * video controller
		 */
		wait_for_completion(&fbi->vsync_notifier);
	}

	return 0;
}

static struct fb_ops db9000fb_ops = {
	.owner		= THIS_MODULE,
	.fb_open	= db9000fb_open,
	.fb_release	= db9000fb_release,
	.fb_check_var	= db9000fb_check_var,
	.fb_set_par	= db9000fb_set_par,
	.fb_setcolreg	= db9000fb_setcolreg,
	.fb_pan_display = db9000fb_pan_display,
	.fb_fillrect	= sys_fillrect,
	.fb_copyarea	= sys_copyarea,
	.fb_imageblit	= sys_imageblit,
	.fb_blank	= db9000fb_blank,
};

/*
 * Some touchscreens need hsync information from the video driver to function
 * correctly. We export it here. Note that 'hsync_time' and the value returned
 * from db9000fb_get_hsync_time() is the *reciprocal* of the hsync period in
 * seconds.
 */
static inline void set_hsync_time(struct db9000fb_info *fbi, unsigned int pcd)
{
	unsigned long htime;

	if ((pcd == 0) || (fbi->info->var.hsync_len == 0)) {
		fbi->hsync_time = 0;
		return;
	}

	htime = clk_get_rate(fbi->clk) / (pcd * fbi->info->var.hsync_len);

	fbi->hsync_time = htime;
}

unsigned long db9000fb_get_hsync_time(struct device *dev)
{
	struct db9000fb_info *fbi = dev_get_drvdata(dev);

	/* If display is blanked/suspended, hsync isn't active */
	if (!fbi || (fbi->state != C_ENABLE))
		return 0;

	return fbi->hsync_time;
}
EXPORT_SYMBOL(db9000fb_get_hsync_time);

static u32 get_pixclk(struct db9000fb_info *fbi,
			unsigned int pixclock)
{
	unsigned long clk_rate;
	unsigned long actual_rate;
	int pcd;
	u32 ret;

	clk_rate = PICOS2KHZ(pixclock) * 1000;
	pr_debug("Clock value is %ld", clk_rate);

	/* If input pixel clock can be set to within 10% of the requested pixel
	 * clock, use it.
	 */
	actual_rate = clk_round_rate(fbi->pixel_clk, clk_rate);
	if ((clk_rate - actual_rate) < (clk_rate / 10)) {
		clk_set_rate(fbi->pixel_clk, actual_rate);
		clk_set_parent(fbi->clk, fbi->pixel_clk);
		ret = DB9000_PCTR_PCI;
	} else {
		/* Otherwise, use the bus clock */
		pcd = (clk_get_rate(fbi->bus_clk) / clk_rate) - 2;
		if (pcd < 0)
			dev_err(&fbi->pdev->dev, "Invalid pixel clock\n");

		set_hsync_time(fbi, pcd);
		clk_set_parent(fbi->clk, fbi->bus_clk);
		ret = DB9000_PCTR_PCD(pcd);
	}

#ifdef CONFIG_BACKLIGHT_DB9000_LCD
	if (fbi->pwm_clock) {
		/* div = pclk / (256 x pwm_clock) */
		u32 pwmfcd = clk_get_rate(fbi->clk) / 256;
		pwmfcd /= fbi->pwm_clock;
		fbi->reg_pwmfr = DB9000_PWMFR_PWM_FCD(pwmfcd - 1);
	}
#endif

	return ret;
}

static void setup_parallel_timing(struct db9000fb_info *fbi,
				struct fb_var_screeninfo *var)
{
	fbi->reg_pctr &= ~0x7FF;

	fbi->reg_pctr |= get_pixclk(fbi, var->pixclock);

	fbi->reg_htr =
	/* horizontal sync width */
		DB9000_HTR_HSW(var->hsync_len) |
	/* horizontal back porch */
		DB9000_HTR_HBP(var->left_margin) |
	/* Pixels per line */
		DB9000_HTR_PPL((var->xres)/16) |
	/* Horizontal Front Porch */
		DB9000_HTR_HFP(var->right_margin);

	/* Vertical and Horizontal Timing Extension write */
	if (fbi->db9000_rev >= DB9000_REVISION_1_14) {
		fbi->reg_hvter =
			DB9000_HVTER_HFPE(var->right_margin) |
			DB9000_HVTER_HBPE(var->left_margin) |
			DB9000_HVTER_VFPE(var->lower_margin) |
			DB9000_HVTER_VBPE(var->upper_margin);
	}

	fbi->reg_vtr1 =
		DB9000_VTR1_VBP(var->upper_margin) |
		DB9000_VTR1_VFP(var->lower_margin) |
		DB9000_VTR1_VSW((var->vsync_len));

	fbi->reg_vtr2 = DB9000_VTR2_LPP(var->yres);

	fbi->reg_cr1 &= ~(DB9000_CR1_HSP | DB9000_CR1_VSP);
	fbi->reg_cr1 |=
		(var->sync & FB_SYNC_HOR_HIGH_ACT) ? 0 : DB9000_CR1_HSP;
	fbi->reg_cr1 |=
		(var->sync & FB_SYNC_VERT_HIGH_ACT) ? 0 : DB9000_CR1_VSP;
	fbi->reg_dear = lcd_readl(fbi, DB9000_DBAR) + fbi->frame_size + 8;
}

/*
 * db9000fb_activate_var():
 *	Configures LCD Controller based on entries in var parameter. Settings
 *	are only written to the controller if changes were made.
 */
static int db9000fb_activate_var(struct fb_var_screeninfo *var,
				struct db9000fb_info *fbi)
{
	if (db9000fb_check_var_in(var) < 0)
		return -EINVAL;

	if (var->xres_virtual < var->xres)
		return -EINVAL;
	if (var->yres_virtual < var->yres * NUM_OF_FRAMEBUFFERS)
		return -EINVAL;

	/* Update shadow copy atomically */
	setup_parallel_timing(fbi, var);

	fbi->reg_cr1 &= ~DB9000_CR1_BPP(7);
	fbi->reg_cr1 |= db9000fb_bpp_to_cr1(var);

	if (var->bits_per_pixel == 24)
		fbi->reg_cr1 |= DB9000_CR1_FBP;
	else
		fbi->reg_cr1 &= ~DB9000_CR1_FBP;

	/*
	 * Normally, the controller simply outputs pixels straight from the
	 * buffer to the panel. Buffers less than 16bpp are effecively treated
	 * as 16bpp due to the palette. If we are connected to a panel with a
	 * 24-bit interface, we need to shift the data to the msbits.
	 */
	if (var->bits_per_pixel <= 16 && fbi->buswidth == 24)
		fbi->reg_cr1 |= DB9000_CR1_OPS(2);
	else
		fbi->reg_cr1 &= ~DB9000_CR1_OPS(2);

	/*
	 * Only update the registers if the controller is enabled and something
	 * has changed.
	 * DBAR is not checked here, it is adviced to be updated on BAU event
	 */
	if ((lcd_readl(fbi, DB9000_CR1) != fbi->reg_cr1) ||
		(lcd_readl(fbi, DB9000_HTR) != fbi->reg_htr) ||
		(lcd_readl(fbi, DB9000_VTR1) != fbi->reg_vtr1) ||
		(lcd_readl(fbi, DB9000_VTR2) != fbi->reg_vtr2) ||
		(lcd_readl(fbi, DB9000_HVTER) != fbi->reg_hvter) ||
		(lcd_readl(fbi, DB9000_PCTR) != fbi->reg_pctr) ||
		(lcd_readl(fbi, DB9000_DBAR) != fbi->reg_dbar) ||
		(lcd_readl(fbi, DB9000_DEAR) != fbi->reg_dear))
		db9000fb_schedule_work(fbi, C_REENABLE);

	return 0;
}

/*
 * NOTE! The following functions are purely helpers for set_ctrlr_state. Do not
 * call them directly; set_ctrlr_state does the correct serialisation to ensure
 * that things happen in the right way 100% of time time.
 * -- rmk
 */
static inline void db9000fb_backlight_power(struct db9000fb_info *fbi, int on)
{
#ifdef CONFIG_BACKLIGHT_DB9000_LCD
	/* Has a PWM clock been specified? */
	if (!fbi->pwm_clock)
		return;

	fbi->reg_pwmfr &= ~DB9000_PWMFR_PWM_FCE;

	if (on) {
		fbi->reg_pwmfr |= DB9000_PWMFR_PWM_FCE;
		lcd_writel(fbi, DB9000_PWMDCR, fbi->reg_pwmdcr);
	} else {
		lcd_writel(fbi, DB9000_PWMDCR, 0x0);
	}

	lcd_writel(fbi, DB9000_PWMFR, fbi->reg_pwmfr);
#endif
}

static inline void db9000fb_lcd_power(struct db9000fb_info *fbi, int on)
{
	fbi->reg_cr1 &= ~DB9000_CR1_LPE;
	if (on)
		fbi->reg_cr1 |= DB9000_CR1_LPE;
	lcd_writel(fbi, DB9000_CR1, fbi->reg_cr1);
}

static void db9000fb_setup_gpio(struct db9000fb_info *fbi, bool on)
{
	int ret;

	if (on) {
		if (!IS_ERR(fbi->pins_default)) {
			ret = pinctrl_select_state(fbi->pinctrl,
					fbi->pins_default);
			if (ret)
				dev_err(&fbi->pdev->dev,
					"could not set default pins\n");
		}
	} else {
		if (!IS_ERR(fbi->pins_sleep)) {
			ret = pinctrl_select_state(fbi->pinctrl,
					fbi->pins_sleep);
			if (ret)
				dev_err(&fbi->pdev->dev,
					"could not set pins to sleep state\n");
		}
	}
}

static void db9000fb_enable_controller(struct db9000fb_info *fbi)
{
	int i;
	u32 val;
	u32 isr;

	pr_debug("db9000fb: Enabling LCD controller\n");
	pr_debug("reg_cr1: 0x%08x\n", (unsigned int) fbi->reg_cr1);
	pr_debug("reg_htr : 0x%08x\n", (unsigned int) fbi->reg_htr);
	pr_debug("reg_vtr1: 0x%08x\n", (unsigned int) fbi->reg_vtr1);
	pr_debug("reg_vtr2: 0x%08x\n", (unsigned int) fbi->reg_vtr2);
	pr_debug("reg_pctr: 0x%08x\n", (unsigned int) fbi->reg_pctr);

	/* enable LCD controller clock */
	if (!fbi->clk_enabled) {
		fbi->clk_enabled = true;
		clk_prepare_enable(fbi->clk);
	}

	/* Write into the palette memory */
	if (fbi->palette_size > 0) {
		for (i = 0; i < (fbi->palette_size/2) ; ++i) {
			val = fbi->palette[i];
			lcd_writel(fbi, (DB9000_PALT + i*4), val);
		}
	}

	lcd_writel(fbi, DB9000_HTR, fbi->reg_htr);
	lcd_writel(fbi, DB9000_VTR1, fbi->reg_vtr1);
	lcd_writel(fbi, DB9000_VTR2, fbi->reg_vtr2);
	lcd_writel(fbi, DB9000_HVTER, fbi->reg_hvter);
	lcd_writel(fbi, DB9000_PCTR, fbi->reg_pctr | DB9000_PCTR_PCR);

	fbi->reg_dbar = fbi->info->fix.smem_start;
	fbi->reg_dear = fbi->reg_dbar + fbi->frame_size + 8;

	lcd_writel(fbi, DB9000_DBAR, fbi->reg_dbar);
	lcd_writel(fbi, DB9000_DEAR, fbi->reg_dear);

	/* configure MRR to 4 outstanding requests */
	lcd_writel(fbi, DB9000_MRR,
		DB9000_MRR_DEAR_MRR(fbi->reg_dear) |
		DB9000_MRR_MRR(DB9000_MRR_OUTST_4));

	/* enable BAU event for IRQ */
	isr = lcd_readl(fbi, DB9000_ISR);
	lcd_writel(fbi, DB9000_ISR, isr | DB9000_ISR_BAU);
	lcd_writel(fbi, DB9000_IMR, DB9000_ISR_BAU);

	lcd_writel(fbi, DB9000_CR1,
		fbi->reg_cr1 | DB9000_CR1_ENB | DB9000_CR1_DEE);
}

static void db9000fb_disable_controller(struct db9000fb_info *fbi)
{
	u32 cr1 = lcd_readl(fbi, DB9000_CR1) & ~DB9000_CR1_ENB;

	lcd_writel(fbi, DB9000_CR1, cr1);
	msleep(100);

	if (fbi->clk_enabled) {
		fbi->clk_enabled = false;
		clk_disable_unprepare(fbi->clk);
	}
}

/* db9000fb_handle_irq: Handle 'LCD DONE' interrupts. */
static irqreturn_t db9000fb_handle_irq(int irq, void *dev_id)
{
	struct db9000fb_info *fbi = dev_id;
	u32 isr = lcd_readl(fbi, DB9000_ISR);
	u32 dbar;

	if (isr & DB9000_ISR_BAU) {
		u32 imr;

		/* DMA Base Address Register Update */
		dbar = lcd_readl(fbi, DB9000_DBAR);
		if (dbar != fbi->reg_dbar) {
			fbi->reg_dbar = dbar;
			fbi->reg_dear = dbar + fbi->frame_size + 8;
			complete(&fbi->vsync_notifier);
		}

		/* Disable Base Address Update interrupt */
		imr = lcd_readl(fbi, DB9000_IMR);
		lcd_writel(fbi, DB9000_IMR, imr & ~DB9000_ISR_BAU);
	}

	lcd_writel(fbi, DB9000_ISR, isr);
	return IRQ_HANDLED;
}

/*
 * This function must be called from task context only, since it will sleep when
 * disabling the LCD controller, or if we get two contending processes trying to
 * alter state.
 */
static void set_ctrlr_state(struct db9000fb_info *fbi, u_int state)
{
	u_int old_state;

	mutex_lock(&fbi->ctrlr_lock);
	old_state = fbi->state;

	/* Hack around fbcon initialisation. */
	if (old_state == C_STARTUP && state == C_REENABLE)
		state = C_STARTUP;

	switch (state) {
	case C_DISABLE_CLKCHANGE:
		/*
		 * Disable controller for clock change. If the controller is
		 * already disabled, then do nothing.
		 */
		if (old_state != C_DISABLE && old_state != C_DISABLE_PM) {
			db9000fb_backlight_power(fbi, 0);
			db9000fb_lcd_power(fbi, 0);
			db9000fb_setup_gpio(fbi, false);
			fbi->state = state;
			db9000fb_disable_controller(fbi);
		}
		break;

	case C_DISABLE_PM:
	case C_DISABLE:
		/* Disable controller */
		if (old_state != C_DISABLE) {
			fbi->state = state;
			db9000fb_backlight_power(fbi, 0);
			db9000fb_lcd_power(fbi, 0);
			db9000fb_setup_gpio(fbi, false);
			if (old_state != C_DISABLE_CLKCHANGE)
				db9000fb_disable_controller(fbi);
		}
		break;

	case C_ENABLE_CLKCHANGE:
		/*
		 * Enable the controller after clock change. Only do this if we
		 * were disabled for the clock change.
		 */
		if (old_state == C_DISABLE_CLKCHANGE) {
			fbi->state = C_ENABLE;
			db9000fb_setup_gpio(fbi, true);
			db9000fb_lcd_power(fbi, 1);
			db9000fb_enable_controller(fbi);
			db9000fb_backlight_power(fbi, 1);
			/* TODO __db9000fb_lcd_power(fbi, 1); */
		}
		break;

	case C_REENABLE:
		/*
		 * Re-enable the controller only if it was already enabled.
		 * This is so we reprogram the control registers.
		 */
		if (old_state == C_ENABLE) {
			db9000fb_backlight_power(fbi, 0);
			db9000fb_lcd_power(fbi, 0);
			db9000fb_setup_gpio(fbi, false);
			db9000fb_disable_controller(fbi);
			msleep(100);
			db9000fb_setup_gpio(fbi, true);
			db9000fb_lcd_power(fbi, 1);
			db9000fb_enable_controller(fbi);
			db9000fb_backlight_power(fbi, 1);
		}
		break;

	case C_ENABLE_PM:
		/*
		 * Re-enable the controller after PM. This is not perfect -
		 * think about the case where we were doing a clock change, and
		 * we suspended half-way through.
		 */
		if (old_state != C_DISABLE_PM)
			break;
		/* fall through */

	case C_ENABLE:
		/*
		 * Power up the LCD screen, enable controller, and turn on the
		 * backlight.
		 */
		if (old_state != C_ENABLE) {

			fbi->state = C_ENABLE;
			db9000fb_setup_gpio(fbi, true);
			db9000fb_lcd_power(fbi, 1);
			db9000fb_enable_controller(fbi);
			db9000fb_backlight_power(fbi, 1);
		}
		break;

	case C_STARTUP:
		fbi->state = C_STARTUP;
		db9000fb_setup_gpio(fbi, true);
		db9000fb_lcd_power(fbi, 1);
		db9000fb_enable_controller(fbi);
		db9000fb_backlight_power(fbi, 1);
		msleep(100);
		db9000fb_backlight_power(fbi, 0);
		db9000fb_lcd_power(fbi, 0);
		db9000fb_setup_gpio(fbi, false);
		db9000fb_disable_controller(fbi);
		break;
	}
	mutex_unlock(&fbi->ctrlr_lock);
}

/*
 * Our LCD controller task (which is called when we blank or unblank) via
 * keventd.
 */
static void db9000fb_task(struct work_struct *work)
{
	struct db9000fb_info *fbi =
		container_of(work, struct db9000fb_info, task);
	u_int state = xchg(&fbi->task_state, -1);

	set_ctrlr_state(fbi, state);
}

/*
 * Clock speed change handler. We need to adjust the LCD timing parameters
 * when the clock is adjusted for whatever reason.
 */
static int db9000fb_clk_notifier_cb(struct notifier_block *nb, unsigned long
				    event, void *data)
{
	struct db9000fb_info *fbi = TO_INF(nb, clk_rate_nb);
	struct fb_var_screeninfo *var = &fbi->info->var;

	switch (event) {
	case PRE_RATE_CHANGE:
		set_ctrlr_state(fbi, C_DISABLE_CLKCHANGE);
		return NOTIFY_OK;
	case POST_RATE_CHANGE:
	case ABORT_RATE_CHANGE:
		setup_parallel_timing(fbi, var);
		set_ctrlr_state(fbi, C_ENABLE_CLKCHANGE);
		return NOTIFY_OK;
	default:
		return NOTIFY_DONE;
	}
}

#ifdef CONFIG_PM
/*
 * Power management hooks. Note that we won't be called from IRQ context, unlike
 * the blank functions above, so we may sleep.
 */
static int db9000fb_suspend(struct device *dev)
{
	struct db9000fb_info *fbi = dev_get_drvdata(dev);

	set_ctrlr_state(fbi, C_DISABLE_PM);
	return 0;
}

static int db9000fb_resume(struct device *dev)
{
	struct db9000fb_info *fbi = dev_get_drvdata(dev);

	set_ctrlr_state(fbi, C_ENABLE_PM);
	return 0;
}

static int db9000fb_thaw(struct device *dev)
{
	struct db9000fb_info *fbi = dev_get_drvdata(dev);

	if (!fbi->clk_enabled) {
		fbi->clk_enabled = true;
		clk_prepare_enable(fbi->clk);
	}

	fbi->state = fbi->old_state;

	return 0;
}

static int db9000fb_freeze(struct device *dev)
{
	struct db9000fb_info *fbi = dev_get_drvdata(dev);

	if (fbi->clk_enabled) {
		fbi->clk_enabled = false;
		clk_disable_unprepare(fbi->clk);
	}

	fbi->old_state = fbi->state;
	fbi->state = C_DISABLE_PM;

	return 0;
}

static const struct dev_pm_ops db9000fb_pm_ops = {
	.suspend	= db9000fb_suspend,
	.resume		= db9000fb_resume,
	.freeze		= db9000fb_freeze,
	.thaw		= db9000fb_thaw,
	.poweroff	= db9000fb_suspend,
	.restore	= db9000fb_resume,
};
#endif

static void *db9000fb_init_fbinfo(struct device *dev,
	struct db9000fb_info *fbi)
{
	struct fb_info *info = fbi->info;

	fbi->clk = clk_get(dev, NULL);
	if (IS_ERR(fbi->clk))
		return NULL;

	strcpy(info->fix.id, DRIVER_NAME);

	fbi->dev = dev;
	info->fix.type	= FB_TYPE_PACKED_PIXELS;
	info->fix.type_aux	= 0;
	info->fix.xpanstep	= 0;
	info->fix.ypanstep	= 1;
	info->fix.ywrapstep	= 1;
	info->fix.accel	= FB_ACCEL_NONE;

	info->var.nonstd	= 0;
	info->var.activate	= FB_ACTIVATE_NOW;
	info->var.height	= info->var.yres;
	info->var.width	= info->var.xres;
	info->var.accel_flags	= 0;
	info->var.vmode	= FB_VMODE_NONINTERLACED;
	info->pseudo_palette	= fbi->pseudo_palette;
	info->fbops		= &db9000fb_ops;
	info->flags		= FBINFO_DEFAULT;
	info->node		= -1;

	fbi->state		= C_STARTUP;
	fbi->task_state		= (u_char)-1;

	atomic_set(&fbi->usage, 0);
	init_waitqueue_head(&fbi->ctrlr_wait);
	INIT_WORK(&fbi->task, db9000fb_task);
	mutex_init(&fbi->ctrlr_lock);
	init_completion(&fbi->vsync_notifier);

	return fbi;
}

#if defined(CONFIG_BACKLIGHT_DB9000_LCD)
static void get_backlight_pwm_clock(struct device_node *np,
	struct db9000fb_info *fbi)
{
	int ret;
	u32 pwm_clock;

	ret = of_property_read_u32(np, "backlight-pwm-clock", &pwm_clock);
	if (!ret)
		fbi->pwm_clock = pwm_clock;
}
#endif

#if defined(CONFIG_FB_DB9000_BLINK)
static int setup_blink_mode(struct device_node *np, struct db9000fb_info *fbi)
{
	int ret;
	u32 fast_blink_ms;
	u32 pwmfcd;
	u32 blink_duty_cycle;
	u32 pwmdc;

	if (fbi->info->var.bits_per_pixel != 24)
		return 0;

	ret = of_property_read_u32(np, "blink-period-ms", &fast_blink_ms);
	if (ret)
		return 0;

	if (fast_blink_ms == 0)
		return 0;

	/* div = (pclk / 256) * fast blink period */
	pwmfcd = clk_get_rate(fbi->pixel_clk) / 256;
	pwmfcd *= fast_blink_ms;
	pwmfcd /= 1000;	/* adjustment as blink period is in ms */

	ret = of_property_read_u32(np, "blink-duty-cycle",
					&blink_duty_cycle);
	if (ret)
		blink_duty_cycle = 50; /* default % */

	if (blink_duty_cycle > 100)
		return -EINVAL;

	pwmdc = (blink_duty_cycle * 256) / 100;

	lcd_writel(fbi, DB9000_PWMFR_BLINK,
		DB9000_PWMFR_PWM_FCD(pwmfcd-1) | DB9000_PWMFR_PWM_FCE);
	lcd_writel(fbi, DB9000_PWMDCR_BLINK, pwmdc-1);
	lcd_writel(fbi, DB9000_GPIOR, 1);

	fbi->use_blinking = true;

	return 0;
}
#endif

static int db9000fb_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fb_info *info;
	struct db9000fb_info *fbi;
	struct display_timings *disp_timings;
	struct display_timing *disp_timing;
	struct fb_videomode mode;
	u32 bpp;
	struct device_node *np = pdev->dev.of_node;
	dma_addr_t fb_mem_phys;
	unsigned long fb_mem_len;
	void *fb_mem_virt;
	struct resource *res;
	int irq;
	int ret;

	info = framebuffer_alloc(sizeof(struct db9000fb_info), dev);
	if (!info)
		return -ENOMEM;

	fbi = info->par;
	fbi->info = info;

	disp_timings = of_get_display_timings(np);
	if (!disp_timings)
		return -EINVAL;

	ret = of_get_fb_videomode(np, &mode, OF_USE_NATIVE_MODE);
	if (ret) {
		dev_err(dev, "Could not get videomode\n");
		return ret;
	}

	ret = of_property_read_u32(np, "bits-per-pixel", &bpp);
	if (ret) {
		dev_err(dev, "Required bits-per-pixel property is missing\n");
		return ret;
	}

	/* setup the timing properties */
	disp_timing = disp_timings->timings[disp_timings->native_mode];
	if (disp_timing->flags & DISPLAY_FLAGS_HSYNC_LOW)
		fbi->reg_cr1 |= DB9000_CR1_HSP;
	if (disp_timing->flags & DISPLAY_FLAGS_VSYNC_LOW)
		fbi->reg_cr1 |= DB9000_CR1_VSP;
	if (disp_timing->flags & DISPLAY_FLAGS_DE_HIGH)
		fbi->reg_cr1 |= DB9000_CR1_DEP;

	/* Width of RGB data going to the panel */
	ret = of_property_read_u32(np, "bus-width", &fbi->buswidth);
	if (ret)
		fbi->buswidth = 24;

	/* Request 16 beat burst Master Bus transactions for max performance */
	fbi->reg_cr1 |= DB9000_CR1_FDW(2);

#ifdef CONFIG_BACKLIGHT_DB9000_LCD
	fbi->reg_pwmdcr = DEF_BRIGHTNESS;
#endif

	/* Pixel clock */
	fbi->pixel_clk = devm_clk_get(dev, "pclk");
	if (IS_ERR(fbi->pixel_clk))
		return PTR_ERR(fbi->pixel_clk);
	ret = clk_prepare_enable(fbi->pixel_clk);
	if (ret) {
		dev_err(dev, "Failed to enable pixel clock\n");
		return ret;
	}

	/* AHB bus clock */
	fbi->bus_clk = devm_clk_get(dev, "ahb");
	if (IS_ERR(fbi->bus_clk))
		return PTR_ERR(fbi->bus_clk);
	ret = clk_prepare_enable(fbi->bus_clk);
	if (ret) {
		dev_err(dev, "Failed to enable AHB clock\n");
		return ret;
	}

	/* allocate the framebuffer. Worst case bpp in case changed later on */
	fbi->frame_size = (mode.xres * mode.yres * 32) / 8;
	fb_mem_len = fbi->frame_size * NUM_OF_FRAMEBUFFERS;

	fb_mem_virt = dmam_alloc_coherent(dev, fb_mem_len, &fb_mem_phys,
				GFP_KERNEL);
	if (!fb_mem_virt) {
		dev_err(dev, "Failed to allocate framebuffer\n");
		return -ENOMEM;
	}

	db9000fb_init_fbinfo(dev, fbi);

	/* registers */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;

	fbi->mmio_base = devm_ioremap_resource(dev, res);
	if (!fbi->mmio_base) {
		dev_err(dev, "failed to map I/O memory\n");
		return -ENOMEM;
	}

	/* irq */
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "no IRQ defined\n");
		return -ENOENT;
	}

	ret = devm_request_irq(dev, irq, db9000fb_handle_irq, 0, pdev->name,
			       fbi);
	if (ret) {
		dev_err(dev, "request_irq failed: %d\n", ret);
		return ret;
	}

	/* pins */
	fbi->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(fbi->pinctrl))
		ret = PTR_ERR(fbi->pinctrl);

	fbi->pins_default = pinctrl_lookup_state(fbi->pinctrl,
			PINCTRL_STATE_DEFAULT);
	if (IS_ERR(fbi->pins_default))
		dev_err(dev, "could not get default pinstate\n");

	fbi->pins_sleep = pinctrl_lookup_state(fbi->pinctrl,
			PINCTRL_STATE_SLEEP);
	if (IS_ERR(fbi->pins_sleep))
		dev_dbg(dev, "could not get sleep pinstate\n");

	fb_videomode_to_var(&info->var, &mode);

	dev_info(dev, "got a %dx%dx%d LCD\n", info->var.xres,
			info->var.yres, bpp);

	/* Initialize fb_info */
	fbi->pdev = pdev;
	info->screen_base	= fb_mem_virt;
	info->fix.smem_start	= fb_mem_phys;
	info->fix.smem_len	= fb_mem_len;
	info->var.height	= info->var.yres;
	info->var.width	= info->var.xres;
	info->var.bits_per_pixel = bpp;

	if (fb_alloc_cmap(&info->cmap, 256, 0))
		return -ENOMEM;

#if defined(CONFIG_BACKLIGHT_DB9000_LCD)
	get_backlight_pwm_clock(np, fbi);
#endif

#if defined(CONFIG_FB_DB9000_BLINK)
	ret = setup_blink_mode(np, fbi);
	if (ret) {
		dev_err(dev, "Failed to set blink parameters\n");
		return ret;
	}
#endif

	ret = db9000fb_check_var(&info->var, info);
	if (ret) {
		dev_err(dev, "failed to get suitable mode\n");
		goto err_free_bl;
	}

	platform_set_drvdata(pdev, fbi);

	ret = register_framebuffer(info);
	if (ret < 0) {
		dev_err(dev, "Failed to register framebuffer device:%d\n", ret);
		goto err_clear_plat_data;
	}

	/* Register the clock change callback */
	fbi->clk_rate_nb.notifier_call = db9000fb_clk_notifier_cb;
	ret = clk_notifier_register(fbi->clk, &fbi->clk_rate_nb);
	if (ret) {
		dev_err(dev, "Unable to register clock notifier\n");
		goto err_clear_plat_data;
	}

	/* Read the core version register and print it out */
	fbi->db9000_rev = lcd_readl(fbi, DB9000_CIR);
	dev_info(dev, "Core ID reg: 0x%08X\n", fbi->db9000_rev);

	ret = db9000fb_set_par(info);
	if (ret) {
		dev_err(dev, "Failed to set parameters\n");
		goto err_clear_plat_data;
	}

#ifdef CONFIG_BACKLIGHT_DB9000_LCD
	init_backlight(fbi);
#endif
	return 0;

err_clear_plat_data:
	platform_set_drvdata(pdev, NULL);
	fb_dealloc_cmap(&info->cmap);
err_free_bl:
	clk_put(fbi->clk);

	return ret;
}

static int db9000fb_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fb_info *info = dev_get_drvdata(dev);
	struct db9000fb_info *fbi;

	if (!info)
		return 0;

	fbi = info->par;
	unregister_framebuffer(info);
	db9000fb_disable_controller(fbi);
	fb_dealloc_cmap(&info->cmap);
	clk_put(fbi->clk);

	return 0;
}

static const struct of_device_id db9000fb_id_match[] = {
	{ .compatible = "digitalblocks,db9000-clcd", },
	{}
};

static struct platform_driver db9000fb_driver = {
	.probe		= db9000fb_probe,
	.remove		= db9000fb_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= DRIVER_NAME,
		.of_match_table = db9000fb_id_match,
#ifdef CONFIG_PM
		.pm = &db9000fb_pm_ops,
#endif
	},
};

static int __init db9000fb_init(void)
{
	return platform_driver_register(&db9000fb_driver);
}
module_init(db9000fb_init);

static void __exit db9000fb_exit(void)
{
	platform_driver_unregister(&db9000fb_driver);
}
module_exit(db9000fb_exit);

MODULE_DESCRIPTION("loadable framebuffer driver for Digital Blocks DB9000");
MODULE_LICENSE("GPL");
