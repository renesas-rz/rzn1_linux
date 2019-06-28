/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2022 Renesas Electronics Europe Ltd.
 *
 * Author: Gareth Williams <gareth.williams.jx@renesas.com>
 *
 * Based on ltdc.c
 * Copyright (C) STMicroelectronics SA 2017
 *
 * Authors: Philippe Cornu <philippe.cornu@st.com>
 *          Yannick Fertre <yannick.fertre@st.com>
 *          Fabien Dessenne <fabien.dessenne@st.com>
 *          Mickael Reulier <mickael.reulier@st.com>
 */
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_device.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>

#include <linux/backlight.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/pwm.h>

#include <video/display_timing.h>

#define DB9000_FB_MAX_WIDTH	4095
#define DB9000_FB_MAX_HEIGHT	4095
#define RZN1_REGS		((void *) 1)

#define DB9000_MAX_LAYER		1

/* LCD Controller Control Register 1 */
#define DB9000_CR1			0x000
/* Horizontal Timing Register */
#define DB9000_HTR			0x008
/* Vertical Timing Register 1 */
#define DB9000_VTR1			0x00C
/* Vertical Timing Register 2 */
#define DB9000_VTR2			0x010
/* Pixel Clock Timing Register */
#define DB9000_PCTR			0x014
/* Interrupt Status Register */
#define DB9000_ISR			0x018
/* Interrupt Mask Register */
#define DB9000_IMR			0x01C
/* Interrupt Vector Register */
#define DB9000_IVR			0x020
/* Interrupt Scan Compare Register */
#define DB9000_ISCR			0x024
/* DMA Base Address Register */
#define DB9000_DBAR			0x028
/* DMA Current Address Register */
#define DB9000_DCAR			0x02C
/* DMA End Address Register */
#define DB9000_DEAR			0x030
/* DMA Horizontal and Vertical Timing Extension Register */
#define DB9000_HVTER			0x044
/* Horizontal Pixels-Per-Line Override Control */
#define DB9000_HPPLOR			0x048
/* Horizontal Pixels-Per-Line Override Enable */
#define DB9000_HPOE			BIT(31)
/* GPIO Register */
#define DB9000_GPIOR			0x1F8
/* Core Identification Register */
#define DB9000_CIR			0x1FC
/* Palette Data Words */
#define DB9000_PALT			0x200

/* Control Register 1, Offset 0x000 */
/* LCD Controller Enable */
#define DB9000_CR1_LCE			BIT(0)
/* LCD Power Enable */
#define DB9000_CR1_LPE			BIT(1)
/* LCD Bits per Pixel */
#define DB9000_CR1_BPP(x)		(((x) & 0x7) << 2)
/* RGB or BGR Format */
#define DB9000_CR1_RGB			BIT(5)
/* Data Enable Polarity */
#define DB9000_CR1_DEP			BIT(8)
/* Pixel Clock Polarity */
#define DB9000_CR1_PCP			BIT(9)
/* Horizontal Sync Polarity */
#define DB9000_CR1_HSP			BIT(10)
/* Vertical Sync Polarity */
#define DB9000_CR1_VSP			BIT(11)
/* Output Pixel Select */
#define DB9000_CR1_OPS(x)		(((x) & 0x7) << 12)
/* FIFO DMA Request Words */
#define DB9000_CR1_FDW(x)		(((x) & 0x3) << 16)
/* LCD 1 or Port Select */
#define DB9000_CR1_LPS			BIT(18)
/* Frame Buffer 24bpp Packed Word */
#define DB9000_CR1_FBP			BIT(19)

enum DB9000_CR1_BPP {
	/* 1 bit per pixel */
	DB9000_CR1_BPP_1bpp,
	/* 2 bits per pixel */
	DB9000_CR1_BPP_2bpp,
	/* 4 bits per pixel */
	DB9000_CR1_BPP_4bpp,
	/* 8 bits per pixel */
	DB9000_CR1_BPP_8bpp,
	/* 16 bits per pixel */
	DB9000_CR1_BPP_16bpp,
	/* 18 bits per pixel */
	DB9000_CR1_BPP_18bpp,
	/* 24 bits per pixel */
	DB9000_CR1_BPP_24bpp
} DB9000_CR1_BPP_VAL;

/* Horizontal Timing Register, Offset 0x008 */
/* Horizontal Front Porch */
#define DB9000_HTR_HFP(x)		(((x) & 0xff) << 0)
/* Pixels per Line */
#define DB9000_HTR_PPL(x)		(((x) & 0xff) << 8)
/* Horizontal Back Porch */
#define DB9000_HTR_HBP(x)		(((x) & 0xff) << 16)
/* Horizontal Sync Width */
#define DB9000_HTR_HSW(x)		(((x) & 0xff) << 24)

/* Vertical Timing Register 1, Offset 0x00C */
/* Vertical Sync Width */
#define DB9000_VTR1_VSW(x)		(((x) & 0xff) << 0)
/* Vertical Front Porch */
#define DB9000_VTR1_VFP(x)		(((x) & 0xff) << 8)
/* Vertical Back Porch */
#define DB9000_VTR1_VBP(x)		(((x) & 0xff) << 16)

/* Vertical Timing Register 2, Offset 0x010 */
/* Lines Per Panel */
#define DB9000_VTR2_LPP(x)		(((x) & 0xfff) << 0)

/* Vertical and Horizontal Timing Extension Register, Offset 0x044 */
/* Horizontal Front Porch Extension */
#define DB9000_HVTER_HFPE(x)		((((x) >> 8) & 0x3) << 0)
/* Horizontal Back Porch Extension */
#define DB9000_HVTER_HBPE(x)		((((x) >> 8) & 0x3) << 4)
/* Vertical Front Porch Extension */
#define DB9000_HVTER_VFPE(x)		((((x) >> 8) & 0x3) << 8)
/* Vertical Back Porch Extension */
#define DB9000_HVTER_VBPE(x)		((((x) >> 8) & 0x3) << 12)

/* clock reset select */
#define DB9000_PCTR_PCR			BIT(10)

/* Interrupt Status Register, Offset 0x018 */
#define DB9000_ISR_OFU			BIT(0) /* Output FIFO Underrun */
#define DB9000_ISR_OFO			BIT(1) /* Output FIFO Overrun */
#define DB9000_ISR_IFU			BIT(2) /* Input FIFO Underrun */
#define DB9000_ISR_IFO			BIT(3) /* Input FIFO Overrun */
#define DB9000_ISR_FER			BIT(4) /* OR of OFU, OFO, IFU, IFO */
#define DB9000_ISR_MBE			BIT(5) /* Master Bus Error */
#define DB9000_ISR_VCT			BIT(6) /* Vertical Compare Triggered */
#define DB9000_ISR_BAU			BIT(7) /* DMA Base Address Register Update to CAR */
#define DB9000_ISR_LDD			BIT(8) /* LCD Controller Disable Done */

/* Interrupt Mask Register, Offset 0x01C */
#define DB9000_IMR_OFUM			BIT(0) /* Output FIFO Underrun - Mask */
#define DB9000_IMR_OFOM			BIT(1) /* Output FIFO Overrun - Mask */
#define DB9000_IMR_IFUM			BIT(2) /* Input FIFO Underrun - Mask */
#define DB9000_IMR_IFOM			BIT(3) /* Input FIFO Overrun - Mask */
#define DB9000_IMR_FERM			BIT(4) /* OR of OFU, OFO, IFU, IFO - Mask */
#define DB9000_IMR_MBEM			BIT(5) /* Master Bus Error - Mask */
#define DB9000_IMR_VCTM			BIT(6) /* Vertical Compare Triggered - Mask */
/* DMA Base Address Register Update to CAR - Mask */
#define DB9000_IMR_BAUM			BIT(7)
#define DB9000_IMR_LDDM			BIT(8) /* LCD Controller Disable Done - Mask */

/* PWM Frequency Register */
#define DB9000_PWMFR_0			0x034
#define DB9000_PWMFR_RZN1		0x04C
/* PWM Duty Cycle Register */
#define DB9000_PWMDCR_0			0x038
#define DB9000_PWMDCR_RZN1		0x050
/* PWM Frequency Registers, Offset 0x034 and 0x04c */
#define DB9000_PWMFR_PWMFCD(x)		(((x) & 0x3fffff) << 0)
#define DB9000_PWMFR_PWMFCE		BIT(22)

static const u32 db9000_fmts[] = {
	DRM_FORMAT_RGB888,
	DRM_FORMAT_RGB565,
	DRM_FORMAT_XRGB1555,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_BGR888,
	DRM_FORMAT_BGR565,
	DRM_FORMAT_XBGR1555,
	DRM_FORMAT_XBGR8888
};

struct fps_info {
	unsigned int counter;
	ktime_t last_timestamp;
};

struct db9000 {
	struct drm_device drm;
	struct drm_connector *conn;
	struct pwm_chip pwm;
	void __iomem *regs;
	spinlock_t lock;
	struct clk *lcd_eclk;
	struct fps_info plane_fpsi[DB9000_MAX_LAYER];
	struct drm_atomic_state *suspend_state;
	struct drm_simple_display_pipe pipe;
	u8 bpp;
	int bus_width;
	size_t frame_size;
	u32 addr_pwmfr;
	u32 addr_pwmdcr;
	u32 pwm_clock;
};

static u32 reg_read(void __iomem *base, u32 reg)
{
	return readl(base + reg);
}

static void reg_write(struct db9000 *db9000, u32 reg, u32 val)
{
	writel(val, db9000->regs + reg);
}

static struct db9000 *plane_to_db9000(struct drm_plane *plane)
{
	return container_of(plane->dev, struct db9000, drm);
}

static struct db9000 *pwm_chip_to_db9000(struct pwm_chip *chip)
{
	return container_of(chip, struct db9000, pwm);
}

void db9000_bpp_setup(struct db9000 *db9000, int bpp, int bus_width,
		      bool pixel_select)
{
	u32 format;
	u32 reg_cr1 = reg_read(db9000->regs, DB9000_CR1);

	/* reset the BPP bits */
	reg_cr1 &= ~DB9000_CR1_BPP(7);
	reg_cr1 &= ~DB9000_CR1_OPS(5);
	reg_cr1 &= ~DB9000_CR1_OPS(1);
	db9000->bpp = bpp;

	switch (bpp) {
	case 16:
		if (pixel_select) {
			reg_cr1 |= DB9000_CR1_OPS(5);
			reg_cr1 |= DB9000_CR1_OPS(1);
		}

		format = DB9000_CR1_BPP(DB9000_CR1_BPP_16bpp);
		break;
	case 24:
	case 32:
	default:
		format = DB9000_CR1_BPP(DB9000_CR1_BPP_24bpp);
	}

	if (bpp <= 16 && bus_width == 24)
		reg_cr1 |= DB9000_CR1_OPS(2);
	else
		reg_cr1 &= ~DB9000_CR1_OPS(2);

	if (bpp == 24)
		reg_cr1 |= DB9000_CR1_FBP;
	else
		reg_cr1 &= ~DB9000_CR1_FBP;

	reg_cr1 |= format;
	reg_write(db9000, DB9000_CR1, reg_cr1);
}

void db9000_controller_on(struct db9000 *db9000)
{
	u32 isr;
	u32 reg_cr1 = reg_read(db9000->regs, DB9000_CR1);
	unsigned long flags;

	/* Enable controller */
	reg_cr1 |= DB9000_CR1_LCE;
	reg_cr1 |= DB9000_CR1_LPE;

	/* DMA Burst Size */
	reg_cr1 |= DB9000_CR1_FDW(2);

	/* Release pixel clock domain reset */
	reg_write(db9000, DB9000_PCTR, DB9000_PCTR_PCR);

	/* Enable BAU event for IRQ */
	spin_lock_irqsave(&db9000->lock, flags);
	isr = reg_read(db9000->regs, DB9000_ISR);
	reg_write(db9000, DB9000_ISR, isr | DB9000_ISR_BAU);
	reg_write(db9000, DB9000_IMR, DB9000_IMR_BAUM);
	spin_unlock_irqrestore(&db9000->lock, flags);

	reg_write(db9000, DB9000_CR1, reg_cr1);
}

void db9000_controller_off(struct db9000 *db9000)
{
	u32 reg_cr1 = reg_read(db9000->regs, DB9000_CR1);

	/* Disable controller */
	reg_cr1 &= ~DB9000_CR1_LCE;
	reg_cr1 &= ~DB9000_CR1_LPE;

	reg_write(db9000, DB9000_CR1, reg_cr1);
}

/* CRTC Functions */
static void db9000_pipe_enable(struct drm_simple_display_pipe *pipe,
			       struct drm_crtc_state *crtc_state,
			       struct drm_plane_state *plane_state)
{
	struct db9000 *db9000 = container_of(pipe->crtc.dev, struct db9000, drm);
	struct drm_display_mode *mode = &crtc_state->adjusted_mode;
	u32 vtr1, vtr2, hvter, htr, hpplor, dear_offset, imr;
	u32 reg_cr1 = reg_read(db9000->regs, DB9000_CR1);
	int vfront_porch = mode->vsync_start - mode->vsync_start;
	int vback_porch = mode->vtotal - mode->vsync_end;
	int hfront_porch = mode->hsync_start - mode->hsync_start;
	int hback_porch = mode->htotal - mode->hsync_end;

	if (mode->flags & DISPLAY_FLAGS_HSYNC_HIGH)
		reg_cr1 |= DB9000_CR1_HSP;
	else
		reg_cr1 &= ~DB9000_CR1_HSP;

	if (mode->flags & DISPLAY_FLAGS_VSYNC_HIGH)
		reg_cr1 |= DB9000_CR1_VSP;
	else
		reg_cr1 &= ~DB9000_CR1_VSP;

	reg_cr1 |= DB9000_CR1_DEP;

	/* Horizontal Timing Register */
	htr =	DB9000_HTR_HSW(mode->htotal) |
		DB9000_HTR_HBP(hback_porch) |
		/* Pixels per line */
		DB9000_HTR_HFP(hfront_porch);

	/* Horizontal Pixels-Per-Line Override */
	hpplor = mode->hdisplay | DB9000_HPOE;

	/* Vertical timing registers setup */
	vtr1 =	DB9000_VTR1_VBP(vback_porch) |
		DB9000_VTR1_VFP(vfront_porch) |
		DB9000_VTR1_VSW(mode->vtotal);

	vtr2 = DB9000_VTR2_LPP(mode->hdisplay);

	/* Vertical and Horizontal Timing Extension write */
	hvter =	DB9000_HVTER_HFPE(hfront_porch) |
		DB9000_HVTER_HBPE(hback_porch) |
		DB9000_HVTER_VFPE(vback_porch) |
		DB9000_HVTER_VBPE(vfront_porch);

	db9000->frame_size = mode->hdisplay * mode->vdisplay;

	/* DEAR register must be configured to the block end + 8 */
	dear_offset =
		(db9000->frame_size * db9000->bpp) / 8 + 8;

	reg_write(db9000, DB9000_CR1, reg_cr1);
	reg_write(db9000, DB9000_HTR, htr);
	reg_write(db9000, DB9000_VTR1, vtr1);
	reg_write(db9000, DB9000_VTR2, vtr2);
	reg_write(db9000, DB9000_HPPLOR, hpplor);
	reg_write(db9000, DB9000_HVTER, hvter);

	drm_dbg(&db9000->drm, "Video mode: %dx%d", mode->hdisplay,
		mode->hdisplay);

	drm_dbg(&db9000->drm, "hfp %d hbp %d hsl %d vfp %d vbp %d vsl %d\n",
		hfront_porch, hback_porch, mode->htotal,
		vfront_porch, vback_porch, mode->vtotal);

	/* Enable IRQ */
	imr = reg_read(db9000, DB9000_IMR);
	reg_write(db9000, DB9000_IMR, imr | DB9000_IMR_BAUM);
}

static void db9000_pipe_disable(struct drm_simple_display_pipe *pipe)
{
	struct db9000 *db9000 = container_of(pipe->crtc.dev, struct db9000, drm);
	u32 imr;

	/* disable IRQ */
	imr = reg_read(db9000->regs, DB9000_IMR);
	reg_write(db9000, DB9000_IMR, imr & ~DB9000_IMR_BAUM);
}

static void db9000_plane_atomic_update(struct drm_plane *plane,
				       struct drm_plane_state *oldstate)
{
	struct db9000 *db9000 = plane_to_db9000(plane);
	struct drm_plane_state *state = plane->state;
	struct drm_framebuffer *fb = state->fb;
	u32 isr, paddr, dear_offset;
	unsigned long flags;
	u32 format;

	if (!state->crtc || !fb) {
		drm_dbg(&db9000->drm, "fb or crtc NULL\n");
		return;
	}

	format = fb->format->format;

	/* The single plane is turning visible, so turn on the display */
	if (!oldstate->visible && state->visible)
		db9000_controller_on(db9000);

	/* The plane is no longer visible */
	if (oldstate->visible && !state->visible)
		db9000_controller_off(db9000);

	/* Check for format changes */
	if (format == DRM_FORMAT_RGB565 || format == DRM_FORMAT_BGR565)
		db9000_bpp_setup(db9000, 16, db9000->bus_width, false);
	else if (format == DRM_FORMAT_XRGB1555 || format == DRM_FORMAT_XBGR1555)
		db9000_bpp_setup(db9000, 16, db9000->bus_width, true);
	else if (format == DRM_FORMAT_RGB888 || format == DRM_FORMAT_BGR888)
		db9000_bpp_setup(db9000, 24, db9000->bus_width, false);
	else if (format == DRM_FORMAT_XRGB8888 || format == DRM_FORMAT_XBGR8888)
		db9000_bpp_setup(db9000, 32, db9000->bus_width, false);

	dear_offset = (db9000->frame_size * db9000->bpp) / 8 + 8;


	/* If the frame buffer has changed get the new FB address */
	if (state->fb) {
		paddr = (u32)drm_fb_cma_get_gem_addr(fb, state, 0);

		drm_dbg(&db9000->drm, "fb: phys 0x%08x\n", paddr);
		reg_write(db9000, DB9000_DBAR, paddr);
		reg_write(db9000, DB9000_DEAR,
			  paddr + dear_offset);
	}

	/* Enable BAU event */
	spin_lock_irqsave(&db9000->lock, flags);
	isr = reg_read(db9000->regs, DB9000_ISR);
	reg_write(db9000, DB9000_ISR, isr | DB9000_ISR_BAU);
	reg_write(db9000, DB9000_IMR, DB9000_IMR_BAUM);
	spin_unlock_irqrestore(&db9000->lock, flags);

	db9000->plane_fpsi->counter++;

	if (isr & DB9000_ISR_MBE) {
		if (isr & DB9000_ISR_OFU)
			DRM_ERROR("Output FIFO Underrun\n");

		if (isr & DB9000_ISR_OFO)
			DRM_ERROR("Output FIFO Overrun\n");

		if (isr & DB9000_ISR_IFU)
			DRM_ERROR("Input FIFO Underrun\n");

		if (isr & DB9000_ISR_IFO)
			DRM_ERROR("Input FIFO Overrun\n");
	}
}

static void db9000_plane_atomic_print_state(struct drm_printer *p,
					    const struct drm_plane_state *state)
{
	struct drm_plane *plane = state->plane;
	struct db9000 *db9000 = plane_to_db9000(plane);
	struct fps_info *fpsi = db9000->plane_fpsi;
	int ms_since_last;
	ktime_t now;

	now = ktime_get();
	ms_since_last = ktime_to_ms(ktime_sub(now, fpsi->last_timestamp));

	drm_printf(p, "\tuser_updates=%dfps\n",
		   DIV_ROUND_CLOSEST(fpsi->counter * 1000, ms_since_last));

	fpsi->last_timestamp = now;
	fpsi->counter = 0;
}

static const struct drm_plane_funcs db9000_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = drm_plane_cleanup,
	.reset = drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
	.atomic_print_state = db9000_plane_atomic_print_state,
};

void __maybe_unused db9000_suspend(struct drm_device *ddev)
{
	struct db9000 *db9000 = container_of(ddev,
					     struct db9000, drm);

	clk_disable_unprepare(db9000->lcd_eclk);
}

int __maybe_unused db9000_resume(struct drm_device *ddev)
{
	struct db9000 *db9000 = container_of(ddev, struct db9000, drm);
	int ret;

	ret = clk_prepare_enable(db9000->lcd_eclk);
	if (ret) {
		DRM_ERROR("failed to enable pixel clock (%d)\n", ret);
		return ret;
	}

	return 0;
}

static int db9000_pwm_request(struct pwm_chip *chip, struct pwm_device *pwm)
{
	return pm_runtime_get_sync(chip->dev);
}

static void db9000_pwm_free(struct pwm_chip *chip, struct pwm_device *pwm)
{
	pm_runtime_put(chip->dev);
}

static int db9000_pwm_enable(struct db9000 *db9000)
{
	u32 reg_pwmfr = reg_read(db9000->regs, db9000->addr_pwmfr);

	reg_pwmfr |= DB9000_PWMFR_PWMFCE;
	reg_write(db9000, db9000->addr_pwmfr, reg_pwmfr);

	return 0;
}

static void db9000_pwm_disable(struct db9000 *db9000)
{
	u32 reg_pwmfr = reg_read(db9000->regs, db9000->addr_pwmfr);

	reg_pwmfr &= ~DB9000_PWMFR_PWMFCE;
	reg_write(db9000, db9000->addr_pwmfr, reg_pwmfr);
}

static int db9000_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			    const struct pwm_state *state)
{
	struct pwm_state cur_state;
	struct db9000 *db9000 = pwm_chip_to_db9000(chip);
	int ret;

	pwm_get_state(pwm, &cur_state);

	if (state->enabled)
		ret = db9000_pwm_enable(db9000);
	else
		db9000_pwm_disable(db9000);

	if (state->period != cur_state.period) {
		u32 pwmfcd;

		pwmfcd = clk_get_rate(db9000->lcd_eclk) / 256;
		pwmfcd /= db9000->pwm_clock;
		pwmfcd = DB9000_PWMFR_PWMFCD(pwmfcd - 1);
		reg_write(db9000, db9000->addr_pwmfr, pwmfcd);
	}

	if (state->duty_cycle && (state->period != cur_state.period ||
	    state->duty_cycle != cur_state.duty_cycle)) {
		u32 dcr = div_u64((state->duty_cycle * ULL(256)),
				   state->period) - 1;

		reg_write(db9000, db9000->addr_pwmdcr, dcr);
	}

	return ret;
}

static const struct pwm_ops db9000_pwm_ops = {
	.request = db9000_pwm_request,
	.free = db9000_pwm_free,
	.apply = db9000_pwm_apply,
	.owner = THIS_MODULE,
};

static int db9000_pwm_setup(struct device *dev,
			    struct db9000 *db9000)
{
	struct pwm_chip *db9000_pwm;
	int ret;

	db9000_pwm = devm_kzalloc(dev, sizeof(*db9000_pwm), GFP_KERNEL);
	if (db9000_pwm == NULL)
		return -ENOMEM;

	db9000_pwm = &db9000->pwm;

	db9000_pwm->dev = dev;
	db9000_pwm->ops = &db9000_pwm_ops;
	db9000_pwm->base = -1;
	db9000_pwm->npwm = 1;

	ret = pwmchip_add(db9000_pwm);
	if (ret < 0) {
		dev_err(dev, "failed to register PWM chip: %d\n", ret);
		return ret;
	}

	pm_runtime_enable(dev);

	return 0;
}

static void db9000_pipe_update(struct drm_simple_display_pipe *pipe,
			       struct drm_plane_state *old_state)
{
	db9000_plane_atomic_update(&pipe->plane, old_state);
}

static const struct drm_simple_display_pipe_funcs db9000_pipe_funcs = {
	.update		= db9000_pipe_update,
	.enable		= db9000_pipe_enable,
	.disable	= db9000_pipe_disable,
};

static const struct drm_mode_config_funcs db9000_mode_config_funcs = {
	.fb_create = drm_gem_fb_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

int db9000_load(struct drm_device *ddev, int rzn1_pwm)
{
	struct platform_device *pdev = to_platform_device(ddev->dev);
	struct db9000 *db9000 = container_of(ddev, struct db9000, drm);
	struct device *dev = ddev->dev;
	struct drm_bridge *bridge;
	struct drm_panel *panel;
	struct device_node *np;
	int ret;

	np = dev->of_node;

	spin_lock_init(&db9000->lock);

	if (rzn1_pwm) {
		db9000->addr_pwmfr = DB9000_PWMFR_RZN1;
		db9000->addr_pwmdcr = DB9000_PWMDCR_RZN1;
	} else {
		db9000->addr_pwmfr = DB9000_PWMFR_0;
		db9000->addr_pwmdcr = DB9000_PWMDCR_0;
	}

	/* Memory setup */
	db9000->regs = devm_platform_get_and_ioremap_resource(pdev, 0, NULL);
	if (IS_ERR(db9000->regs))
		return PTR_ERR(db9000->regs);

	/* Clock setup */
	db9000->lcd_eclk = devm_clk_get(dev, "lcd_eclk");
	if (IS_ERR(db9000->lcd_eclk)) {
		DRM_ERROR("Unable to get pixel clock\n");
		return -ENODEV;
	}

	if (clk_prepare_enable(db9000->lcd_eclk)) {
		DRM_ERROR("Unable to prepare pixel clock\n");
		return -ENODEV;
	}

	ret = drmm_mode_config_init(ddev);
	if (ret)
		goto clk_err;

	/*
	 * set max width and height as default value.
	 * this value would be used to check framebuffer size limitation
	 * at drm_mode_addfb().
	 */
	ddev->mode_config.min_width = 0;
	ddev->mode_config.min_height = 0;
	ddev->mode_config.max_width = DB9000_FB_MAX_WIDTH;
	ddev->mode_config.max_height = DB9000_FB_MAX_HEIGHT;
	ddev->mode_config.funcs = &db9000_mode_config_funcs;

	db9000_controller_on(db9000);
	db9000_bpp_setup(db9000, db9000->bpp, db9000->bus_width, false);

	ret = db9000_pwm_setup(dev, db9000);
	if (ret)
		goto err;

	/* Panel Setup */
	ret = drm_of_find_panel_or_bridge(np, 0, 0, &panel, NULL);
	if (ret < 0) {
		DRM_ERROR("Could not get Panel");
		goto err;
	}

	/* Add endpoints panels or bridges if any */
	if (panel) {
		bridge = drm_panel_bridge_add(panel);
		if (IS_ERR(bridge)) {
			DRM_ERROR("panel-bridge endpoint\n");
			ret = PTR_ERR(bridge);
			goto err;
		}
	}

	db9000->conn = drm_panel_bridge_connector(bridge);

	ret = drm_simple_display_pipe_init(ddev,
			&db9000->pipe,
			&db9000_pipe_funcs,
			db9000_fmts, ARRAY_SIZE(db9000_fmts),
			NULL,
			db9000->conn);
	if (ret)
		goto err;

	ret = drm_simple_display_pipe_attach_bridge(&db9000->pipe, bridge);
	if (ret) {
		DRM_WARN("Failed to attach bridge to pipe\n");
		goto err;
	}

	return 0;

err:
	if (panel)
		drm_panel_bridge_remove(bridge);

	drm_mode_config_cleanup(ddev);

clk_err:
	clk_disable_unprepare(db9000->lcd_eclk);

	return ret;
}

void db9000_unload(struct drm_device *ddev)
{
	struct db9000 *db9000 = container_of(ddev, struct db9000, drm);

	drm_of_panel_bridge_remove(ddev->dev->of_node, 0, 0);
	clk_disable_unprepare(db9000->lcd_eclk);
}

DEFINE_DRM_GEM_CMA_FOPS(db9000_driver_fops);
static struct drm_driver db9000_drm_driver_data = {
	.driver_features = DRIVER_MODESET | DRIVER_GEM | DRIVER_ATOMIC,
	.name = "drm-db9000",
	.desc = "Digital Blocks DB9000 DRM Driver",
	.date = "20220615",
	.major = 1,
	.minor = 0,
	.patchlevel = 0,
	.fops = &db9000_driver_fops,
	.dumb_create = drm_gem_cma_dumb_create_internal,
	.prime_handle_to_fd = drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle = drm_gem_prime_fd_to_handle,
	DRM_GEM_CMA_DRIVER_OPS,
};

static int drv_load(struct drm_device *ddev, u32 bpp,
		    u32 bus_width, int rzn1_pwm)
{
	struct platform_device *pdev = to_platform_device(ddev->dev);
	struct db9000 *db9000 = container_of(ddev, struct db9000, drm);
	int ret;

	db9000->bus_width = bus_width;

	ret = db9000_load(ddev, rzn1_pwm);
	if (ret)
		return ret;

	drm_mode_config_reset(ddev);
	drm_kms_helper_poll_init(ddev);

	platform_set_drvdata(pdev, ddev);

	return 0;
}

static void drv_unload(struct drm_device *ddev)
{
	drm_kms_helper_poll_fini(ddev);
	db9000_unload(ddev);
	drm_mode_config_cleanup(ddev);
}

static __maybe_unused int db9000_drv_suspend(struct device *dev)
{
	struct drm_device *ddev = dev_get_drvdata(dev);
	struct db9000 *db9000 = container_of(ddev, struct db9000, drm);
	struct drm_atomic_state *state;

	db9000_controller_off(db9000);

	drm_kms_helper_poll_disable(ddev);
	state = drm_atomic_helper_suspend(ddev);
	if (IS_ERR(state)) {
		drm_kms_helper_poll_enable(ddev);
		return PTR_ERR(state);
	}
	db9000->suspend_state = state;
	db9000_suspend(ddev);

	return 0;
}

static __maybe_unused int db9000_drv_resume(struct device *dev)
{
	struct drm_device *ddev = dev_get_drvdata(dev);
	struct db9000 *db9000 = container_of(ddev, struct db9000, drm);

	db9000_resume(ddev);
	drm_atomic_helper_resume(ddev, db9000->suspend_state);
	drm_kms_helper_poll_enable(ddev);
	db9000_controller_on(db9000);

	return 0;
}

static const struct dev_pm_ops db9000_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(db9000_drv_suspend, db9000_drv_resume)
};

static int db9000_drm_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct drm_device *ddev;
	struct db9000 *db9000;
	struct device_node *np = dev->of_node;
	u32 bpp;
	u32 bus_width;
	int rzn1_pwm = 0;
	int ret;

	dma_set_coherent_mask(dev, DMA_BIT_MASK(32));

	db9000 = devm_drm_dev_alloc(dev, &db9000_drm_driver_data,
				    struct db9000, drm);
	ddev = &db9000->drm;

	/* Parse the DTB */
	ret = of_property_read_u32(np, "bits-per-pixel", &bpp);
	if (ret)
		bpp = 16;

	if (bpp != 16 && bpp != 24 && bpp != 32) {
		drm_warn(&db9000->drm, "bits-per-pixel value invalid, default is 16 bpp");
		bpp = 16;
	}

	ret = of_property_read_u32(np, "backlight-pwm-clock", &db9000->pwm_clock);
	if (ret)
		db9000->pwm_clock = 300;

	ret = of_property_read_u32(np, "bus-width", &bus_width);
	if (ret)
		bus_width = 24;

	db9000->bus_width = bus_width;
	db9000->bpp = bpp;

	rzn1_pwm = (int) of_device_get_match_data(dev);

	ret = drv_load(ddev, bpp, bus_width, rzn1_pwm);
	if (ret)
		return ret;

	ret = drm_dev_register(ddev, 0);
	if (ret)
		return ret;

	drm_fbdev_generic_setup(&db9000->drm, bpp);

	dev_info(dev, "DB9000 LCD Controller Ready");

	return 0;
}

static int db9000_drm_platform_remove(struct platform_device *pdev)
{
	struct drm_device *ddev = platform_get_drvdata(pdev);

	drv_unload(ddev);
	drm_dev_put(ddev);

	return 0;
}

static const struct of_device_id db9000_dt_ids[] = {
	{ .compatible = "digital-blocks,drm-db9000"},
	{ .compatible = "digital-blocks,drm-rzn1", .data = RZN1_REGS },
	{ /* end node */ },
};
MODULE_DEVICE_TABLE(of, db9000_dt_ids);

static struct platform_driver db9000_drm_platform_driver = {
	.probe = db9000_drm_platform_probe,
	.remove = db9000_drm_platform_remove,
	.driver = {
		.name = "drm-db9000",
		.of_match_table = db9000_dt_ids,
		.pm = &db9000_pm_ops,
	},
};

module_platform_driver(db9000_drm_platform_driver);

MODULE_AUTHOR("Gareth Williams <gareth.williams.jx@renesas.com>");
MODULE_DESCRIPTION("Digital Blocks DB9000 LCD Controller Driver");
MODULE_LICENSE("GPL v2");
