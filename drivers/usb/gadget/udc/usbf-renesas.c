/*
 * Renesas RZ/N1 USB Device usb gadget driver
 *
 * Copyright 2015 Renesas Electronics Europe Ltd.
 * Author: Michel Pollet <michel.pollet@bp.renesas.com>,<buserror@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/sysctrl-rzn1.h>

#include "usbf-renesas.h"

#ifdef	CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif

#define DRIVER_DESC     "Renesas USB Function driver"
#define DRIVER_AUTHOR   "Michel Pollet"
#define DRIVER_VERSION  "0.2"

#define DMA_ADDR_INVALID        (~(dma_addr_t)0)

static const char driver_name[] = "usbf_renesas";
static const char driver_desc[] = DRIVER_DESC;

#ifdef	CONFIG_DEBUG_FS
struct f_drv *usbf;
#endif

static struct usb_endpoint_descriptor ep0_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = USB_DIR_IN,
	.bmAttributes = USB_ENDPOINT_XFER_CONTROL,
	.wMaxPacketSize = CFG_EP0_MAX_PACKET_SIZE,
};

#ifdef	CONFIG_DEBUG_FS
static int usbf_debug_init(struct f_drv *chip);
#endif

/*
 * activate/deactivate link with host.
 */
static void pullup(struct f_drv *chip, int is_on)
{
	struct f_regs *regs = chip->regs;

	if (is_on) {
		if (!chip->pullup) {
			chip->pullup = 1;
			writel((readl(&regs->control) & ~D_USB_CONNECTB) |
				D_USB_PUE2, &regs->control);
			usb_gadget_set_state(&chip->gadget, USB_STATE_POWERED);
		}
	} else {
		chip->pullup = 0;
		writel((readl(&regs->control) & ~D_USB_PUE2) |
			D_USB_CONNECTB, &regs->control);
		usb_gadget_set_state(&chip->gadget, USB_STATE_NOTATTACHED);
	}
}

/*
 * USB Gadget Layer
 */
static int usbf_ep_enable(
	struct usb_ep *_ep,
	const struct usb_endpoint_descriptor *desc)
{
	struct f_endpoint *ep = container_of(_ep, struct f_endpoint, ep);
	struct f_drv *chip = ep->chip;
	struct f_regs *regs = chip->regs;
	unsigned long flags;

	TRACE("%s[%d] desctype %d max pktsize %d\n", __func__, ep->id,
	      desc->bDescriptorType,
	      usb_endpoint_maxp(desc));
	if (!_ep || !desc || desc->bDescriptorType != USB_DT_ENDPOINT) {
		TRACE("%s: bad ep or descriptor\n", __func__);
		return -EINVAL;
	}

	/* it might appear as we nuke the const here, but in this case,
	 * we just need the ep0 to be able to change the endpoint direction,
	 * and we know it does that on a non-const copy of its descriptor,
	 * while the other endpoints don't touch it anyway */
	ep->desc = (struct usb_endpoint_descriptor *)desc;
	ep->ep.maxpacket = usb_endpoint_maxp(desc);

	if (ep->drv->enable)
		ep->drv->enable(ep);
	if (ep->drv->set_maxpacket)
		ep->drv->set_maxpacket(ep);
	ep->disabled = 0;

	/* enable interrupts for this endpoint */
	spin_lock_irqsave(&chip->lock, flags);
	writel(readl(&regs->int_enable) | (D_USB_EP0_EN << ep->id),
	       &regs->int_enable);
	spin_unlock_irqrestore(&chip->lock, flags);

	return 0;
}

static int usbf_ep_disable(struct usb_ep *_ep)
{
	struct f_endpoint *ep = container_of(_ep, struct f_endpoint, ep);
	struct f_drv *chip = ep->chip;
	struct f_regs *regs = chip->regs;
	unsigned long flags;

	TRACE("%s(%d)\n", __func__, ep->id);

	/* disable interrupts for this endpoint */
	spin_lock_irqsave(&chip->lock, flags);
	writel(readl(&regs->int_enable) & ~(D_USB_EP0_EN << ep->id),
	       &regs->int_enable);
	spin_unlock_irqrestore(&chip->lock, flags);
	if (ep->drv->disable)
		ep->drv->disable(ep);
	ep->desc = NULL;
	ep->disabled = 1;
	return 0;
}

static struct usb_request *usbf_ep_alloc_request(
	struct usb_ep *_ep, gfp_t gfp_flags)
{
	struct f_req *req;

	if (!_ep)
		return NULL;

	req = kzalloc(sizeof(*req), gfp_flags);
	if (!req)
		return NULL;

	req->req.dma = DMA_ADDR_INVALID;
	INIT_LIST_HEAD(&req->queue);

	return &req->req;
}

static void usbf_ep_free_request(
	struct usb_ep *_ep, struct usb_request *_req)
{
	struct f_req *req;
	struct f_endpoint *ep;

	if (!_ep || !_req)
		return;

	req = container_of(_req, struct f_req, req);
	ep = container_of(_ep, struct f_endpoint, ep);

	spin_lock_irq(&ep->lock);
	list_del_init(&req->queue);
	spin_unlock_irq(&ep->lock);
	kfree(req);
}

static int usbf_ep_queue(
	struct usb_ep *_ep, struct usb_request *_req, gfp_t gfp_flags)
{
	struct f_endpoint *ep = container_of(_ep, struct f_endpoint, ep);
	struct f_drv *chip = ep->chip;
	struct f_req *req = container_of(_req, struct f_req, req);
	int was_empty = list_empty(&ep->queue);

	if (!_req || !_req->buf) {
		TRACE("%s: invalid request to ep%d\n", __func__, ep->id);
		return -EINVAL;
	}

	if (!chip || chip->state == USB_STATE_SUSPENDED) {
		TRACE("%s: request while chip suspended\n", __func__);
		return -EINVAL;
	}
	req->trace = 0;
	req->req.actual = 0;
	req->req.status = -EINPROGRESS;
	req->seq = ep->seq++; /* debug */
	req->to_host = usb_endpoint_dir_in(ep->desc);
	/* Basic criteria for using DMA on this request */
	req->use_dma = ep->has_dma && ep->id != 0 &&
			req->req.length > chip->dma_threshold;

	/* DISABLE DMA */
	req->use_dma = 0;

	/* This technically is a kludge, we test the pointer /before/ we
	 * bother to DMA map it and 'discover' it's also been mapped to an
	 * unaligned address. In testing, the lower part of the pointers
	 * stay the same anyway, so testing before we map is valid. */
	if (req->use_dma && ((u32)req->req.buf & 3)) {
		req->use_dma = 0;
		TRACEEP(ep, "%s[%d][%3d] WARNING unaligned buffer at %p -- DMA Disabled\n",
			__func__, ep->id, req->seq, req->req.buf);
	}

	/* The endpoint driver has 2 types of transfer callbacks, one for
	 * receiving and one for sending; we know at 'queue' time the direction
	 * the endpoint is, so we can install the correct one with the request
	 * and let it do the job until completion */

	/* if DMA callback is not present, fallback to PIO */
	if (req->to_host) {
		if (req->use_dma && !ep->drv->send[USBF_PROCESS_DMA])
			req->use_dma = 0;
		req->process = ep->drv->send[req->use_dma];
	} else {
		if (req->use_dma && !ep->drv->recv[USBF_PROCESS_DMA])
			req->use_dma = 0;
		req->process = ep->drv->recv[req->use_dma];
	}
	/* Prepare the buffer for DMA, if necessary */
	if (req->use_dma) {
		if (req->req.dma == DMA_ADDR_INVALID) {
			if (usb_gadget_map_request(&chip->gadget, &req->req,
						req->to_host)) {
				pr_err("dma_mapping_error\n");
				req->req.dma = DMA_ADDR_INVALID;
				req->use_dma = 0;
			} else
				req->mapped = 1;
		}
	}

	TRACEEP(ep, "%s[%d][%3d], dma:%d mapped:%d len %3d %s %s\n", __func__,
		ep->id, req->seq, req->use_dma, req->mapped,
		req->req.length,
		req->to_host ? "input" : "output",
		req->req.zero ? "ZERO" : "(no zero)");
	if (req->use_dma) /* DMA is still possible */
		dma_sync_single_for_device(
			chip->dev,
			req->req.dma, req->req.length,
			req->to_host ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
	/* This is /required/ -- you cannot queue these requests, because
	 * the upper layer doesn't assume they will be queued and will call
	 * queue() again with the same request, polluting the list; so we
	 * need to complete() it now, even tho it might actually be out of
	 * sequence (if a packet is already in the queue */
	if (req->req.length == 0) {
		req->req.status = 0;
		usb_gadget_giveback_request(&ep->ep, &req->req);
		return 0;
	}

	spin_lock_irq(&ep->lock);
	list_add_tail(&req->queue, &ep->queue);
	spin_unlock_irq(&ep->lock);

	/* kick a soft interrupt, in case we are not called from the tasklet */
	if (req->to_host || was_empty)
		tasklet_schedule(&chip->tasklet);

	return 0;
}

static int usbf_ep_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct f_endpoint *ep = container_of(_ep, struct f_endpoint, ep);
	struct f_req *req = container_of(_req, struct f_req, req);
	struct f_drv *chip = ep->chip;

	/* dequeue the request */
	spin_lock_irq(&ep->lock);
	list_del_init(&req->queue);
	spin_unlock_irq(&ep->lock);

	if (req->use_dma) {
		req->use_dma = 0;
		dma_sync_single_for_cpu(
			chip->dev,
			req->req.dma, req->req.length,
			req->to_host ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
	}
	TRACEEP(ep, "%s[%d][%3d] size %d\n", __func__,
			ep->id, req->seq, req->req.actual);

	if (req->trace ||
		(ep->trace_min && req->req.actual >= ep->trace_min &&
		(ep->trace_max == 0 || req->req.actual <= ep->trace_max))) {
		req->trace = 1;
		TRACERQ(req, "%s[%d][%3d] DUMP %d buf:%p dma %08x:\n",
			__func__, ep->id, req->seq, req->req.actual,
			req->req.buf, req->req.dma);
	//	print_hex_dump_bytes("pkt", DUMP_PREFIX_OFFSET,
	//		req->req.buf, req->req.actual);
	}
	if (req->mapped) {
		usb_gadget_unmap_request(&chip->gadget, &req->req,
				req->to_host);
		/* ^^ doesn't do this:  */
		req->req.dma = DMA_ADDR_INVALID;
		req->mapped = 0;
	}

	/* don't modify queue heads during completion callback */
	if (chip->gadget.speed == USB_SPEED_UNKNOWN)
		req->req.status = -ESHUTDOWN;

	if (req->req.complete)
		usb_gadget_giveback_request(&ep->ep, &req->req);

	return 0;
}

/*
 * This function is called repeatedly on each endpoint. Its job is to
 * 'process' the current queued request (top of the queue) and 'complete'
 * it when it's finished.
 */
int usbf_ep_process_queue(struct f_endpoint *ep)
{
	struct f_req *req;

	spin_lock_irq(&ep->lock);
	req = list_first_entry_or_null(&ep->queue, struct f_req, queue);
	spin_unlock_irq(&ep->lock);

	if (req && req->process(ep, req) == 0) {
		/* 'complete' this request */
		usbf_ep_dequeue(&ep->ep, &req->req);

		/* If there are further requests, reschedule the tasklet */
		spin_lock_irq(&ep->lock);
		req = list_first_entry_or_null(&ep->queue, struct f_req, queue);
		spin_unlock_irq(&ep->lock);
		if (req)
			return 1;
	}

	return 0;
}

static int usbf_ep_halt(struct usb_ep *_ep, int halt)
{
	struct f_endpoint *ep = container_of(_ep, struct f_endpoint, ep);
	int ret = 0;

	TRACEEP(ep, "%s[%d] halt=%d\n", __func__, ep->id, halt);
	if (ep->drv->halt)
		ret = ep->drv->halt(ep, halt);
	return ret;
}

static void usbf_ep_reset(
	struct f_endpoint *ep)
{
	TRACEEP(ep, "%s[%d] reset\n", __func__, ep->id);
	if (ep->drv->reset)
		ep->drv->reset(ep);
	ep->status = 0;
	/* flush anything that was pending */
	while (!list_empty(&ep->queue)) {
		struct f_req *req = list_first_entry(&ep->queue,
					struct f_req, queue);
		TRACEEP(ep, "%s[%d][%3d] dequeueing\n", __func__,
			ep->id, req->seq);
		req->req.status = -ECONNRESET;
		usbf_ep_dequeue(&ep->ep, &req->req);
	}
}

static struct usb_ep_ops usbf_ep_ops = {
	.enable         = usbf_ep_enable,
	.disable        = usbf_ep_disable,
	.queue          = usbf_ep_queue,
	.dequeue        = usbf_ep_dequeue,
	.set_halt       = usbf_ep_halt,
	.alloc_request  = usbf_ep_alloc_request,
	.free_request   = usbf_ep_free_request,
};

/* If there's stuff in the fifo, collect it */
static uint32_t usbf_tasklet_flush_fifo(
	struct f_drv *chip)
{
	struct f_status s;
	int i;
	uint32_t int_status = 0;
	unsigned long flags;

	spin_lock_irqsave(&chip->lock, flags);

	/* If multiple sets of interrupt status bits have been pushed onto the
	 * FIFO, or them together to get a complete set.
	 */
	if (kfifo_get(&chip->fifo, &s)) {
		/* Mask out the global EP interrupt bits */
		int_status = s.int_status & 0xf;
		for (i = 0; i < CFG_NUM_ENDPOINTS; i++) {
			chip->ep[i].status = s.ep[i];
			/* Fake the global EP interrupt bits based on what we read */
			if (s.ep[i])
				int_status |= (D_USB_EP0_INT << i);
		}
	}

	spin_unlock_irqrestore(&chip->lock, flags);

	return int_status;
}

static void usbf_tasklet_process_irq(struct f_drv *chip, uint32_t int_status)
{
	int i;

	if (int_status & D_USB_USB_RST_INT) {
		if (chip->gadget.speed != USB_SPEED_UNKNOWN) {
			TRACE("%s disconnecting\n", __func__);
			chip->gadget.speed = USB_SPEED_UNKNOWN;
			/* We assume all endpoints will be closed,
			 * so reset the DMA memory pointer for later use */
			chip->dma_ram_used = EP0_RAM_USED;
			if (chip->driver)
				chip->driver->disconnect(&chip->gadget);
			else
				dev_warn(chip->dev, "Spurious RST\n");
		}
	}
	if (int_status & D_USB_SPEED_MODE_INT) {
		if (readl(&chip->regs->status) & D_USB_SPEED_MODE)
			chip->gadget.speed = USB_SPEED_HIGH;
		else
			chip->gadget.speed = USB_SPEED_FULL;
		TRACE("**** %s speed change: %s\n", __func__,
			chip->gadget.speed == USB_SPEED_HIGH ? "High" : "Full");
	}
#if 0
	if (int_status & D_USB_SPND_INT)
		TRACE("%s suspend clear\n", __func__);
	if (int_status & D_USB_RSUM_INT)
		TRACE("%s resume\n", __func__);
#endif
	for (i = 0; i < CFG_NUM_ENDPOINTS; i++) {
		struct f_endpoint *ep = &chip->ep[i];

		if (ep->disabled)
			continue;

		if (int_status & D_USB_USB_RST_INT)
			usbf_ep_reset(ep);
		/* speed change notification for endpoints */
		if ((int_status & D_USB_SPEED_MODE_INT) &&
				ep->drv->set_maxpacket)
			ep->drv->set_maxpacket(ep);

		/* Interrupt notification */
		if ((int_status & (D_USB_EP0_INT << i)) && ep->drv->interrupt)
			ep->drv->interrupt(ep);
	}
}

static void usbf_tasklet(unsigned long data)
{
	struct f_drv *chip = (struct f_drv *)data;
	uint32_t int_status;
	int i, busy = 1;

	while ((int_status = usbf_tasklet_flush_fifo(chip)) != 0)
		usbf_tasklet_process_irq(chip, int_status);

	while (busy) {
		busy = 0;

		for (i = 0; i < CFG_NUM_ENDPOINTS; i++) {
			struct f_endpoint *ep = &chip->ep[i];

			if (!ep->disabled)
				busy |= usbf_ep_process_queue(ep);
		}
	}
}

static void usbf_attach(struct f_drv *chip)
{
	struct f_regs *regs = chip->regs;
	uint32_t ctrl = readl(&regs->control);

	/* Enable USB signal to Function PHY */
	ctrl &= ~D_USB_CONNECTB;
	/* D+ signal Pull-up */
	ctrl |=  D_USB_PUE2;
	/* Enable endpoint 0 */
	ctrl |=  D_USB_DEFAULT;

	writel(ctrl, &regs->control);
}

static void usbf_detach(struct f_drv *chip)
{
	struct f_regs *regs = chip->regs;
	uint32_t ctrl = readl(&regs->control);

	/* Disable USB signal to Function PHY */
	ctrl |=  D_USB_CONNECTB;
	/* Do not Pull-up D+ signal */
	ctrl &= ~D_USB_PUE2;
	/* Disable endpoint 0 */
	ctrl &= ~D_USB_DEFAULT;
	/* Disable the other endpoints */
	ctrl &= ~D_USB_CONF;

	writel(ctrl, &regs->control);

	writel(0, &regs->ep0.status);
}

static irqreturn_t usbf_irq(int irq, void *_chip)
{
	struct f_drv *chip = (struct f_drv *)_chip;
	struct f_regs *regs = chip->regs;
	uint32_t sysbint = readl(&regs->sysbint);

	/* clear interrupts */
	writel(sysbint, &regs->sysbint);
	if ((sysbint & D_SYS_VBUS_INT) == D_SYS_VBUS_INT) {
		/* Interrupt factor clear */
		if (readl(&regs->epctr) & D_SYS_VBUS_LEVEL) {
			TRACE("%s plugged in\n", __func__);
			usbf_attach(chip);
			usb_gadget_set_state(&chip->gadget, USB_STATE_POWERED);
		} else {
			TRACE("%s plugged out\n", __func__);
			usbf_detach(chip);
		}
	}

	return IRQ_HANDLED;
}

/*
 * Offload the IRQ flags into the FIFO ASAP, then return.
 *
 * The status flags are passed down to tasklet via a FIFO
 */
static irqreturn_t usbf_epc_irq(int irq, void *_chip)
{
	struct f_drv *chip = (struct f_drv *)_chip;
	struct f_regs *regs = chip->regs;
	struct f_status s;
	int i;
	u32 int_en;
	unsigned long flags;

	spin_lock_irqsave(&chip->lock, flags);

	/* Disable interrupts */
	int_en = readl(&regs->int_enable);
	writel(0, &regs->int_enable);

	/*
	 * WARNING: Don't use the EP interrupt bits from this status reg as they
	 * won't be coherent with the EP status registers.
	 */
	s.int_status = readl(&regs->int_status) & int_en;
	writel(~s.int_status, &regs->int_status);

	s.ep[0] = readl(&regs->ep0.status) & readl(&regs->ep0.int_enable);
	writel(~s.ep[0], &regs->ep0.status);

	for (i = 1; i < CFG_NUM_ENDPOINTS; i++) {
		s.ep[i] = readl(&regs->ep[i - 1].status) & readl(&regs->ep[i - 1].int_enable);
		writel(~s.ep[i], &regs->ep[i - 1].status);
	}

	if (!kfifo_put(&chip->fifo, s))
		dev_err(chip->dev, "intstatus fifo full!\n");

	/* Enable interrupts */
	writel(int_en, &regs->int_enable);

	spin_unlock_irqrestore(&chip->lock, flags);

	tasklet_schedule(&chip->tasklet);

	return IRQ_HANDLED;
}

/*-------------------------------------------------------------------------
	Gadget driver probe and unregister.
 --------------------------------------------------------------------------*/
static int usbf_gadget_start(struct usb_gadget *gadget,
		struct usb_gadget_driver *driver)
{
	struct f_drv *chip = container_of(gadget, struct f_drv, gadget);
	struct device_node *np = chip->dev->of_node;

	driver->driver.bus = NULL;
	/* hook up the driver */
	chip->driver = driver;
	chip->gadget.speed = driver->max_speed;

	dev_info(chip->dev, "%s bind to driver %s\n", chip->gadget.name,
			driver->driver.name);
	/* Finish initialization by getting possible overrides from DT */
	if (np) {
		int rm_len, i, pi;
		const char * const names[] = {
			[0] = kasprintf(GFP_KERNEL, "renesas,sram-conf,%s",
				driver->driver.name),
			[1] = "renesas,sram-conf",
		};

		/* reset all packet counts to default (2) */
		for (pi = 0; pi < ARRAY_SIZE(names); pi++)
			chip->ep[pi].max_nr_pkts = 0;
		for (pi = 0; pi < ARRAY_SIZE(chip->dma_pkt_count); pi++)
			chip->dma_pkt_count[pi] = 0;

		for (pi = 0; pi < ARRAY_SIZE(names); pi++) {
			const __be32 *conf = of_get_property(np,
							names[pi], &rm_len);
			TRACE("%s %s: %s : %p\n", __func__, driver->function,
				names[pi], conf);
			if (!conf)
				continue;
			rm_len /= sizeof(conf[0]);
			if (rm_len > 3) { /* per-endpoint config */
				for (i = 0; i < rm_len &&
						i < ARRAY_SIZE(chip->ep); i++)
					if (!chip->ep[i].max_nr_pkts)
						chip->ep[i].max_nr_pkts =
							be32_to_cpu(conf[i]);
			} else { /* per endpoint type config */
				for (i = 0; i < rm_len; i++)
					if (!chip->dma_pkt_count[i])
						chip->dma_pkt_count[i] =
							be32_to_cpu(conf[i]);
			}
		}
		kfree(names[0]);
	}

	return 0;
}

static int usbf_gadget_stop(struct usb_gadget *gadget)
{
	struct f_drv *chip = container_of(gadget, struct f_drv, gadget);

	chip->gadget.speed = USB_SPEED_UNKNOWN;

	dev_info(chip->dev, "unregistered gadget driver '%s'\n",
			chip->driver->driver.name);
	chip->driver = NULL;
	return 0;
}

static int usbf_gadget_pullup(struct usb_gadget *_gadget, int is_on)
{
	struct f_drv *chip = container_of(_gadget, struct f_drv, gadget);

	TRACE("%s: pullup=%d\n", __func__, is_on);

	pullup(chip, is_on);

	return 0;
}

static struct usb_gadget_ops usbf_gadget_ops = {
	.pullup = usbf_gadget_pullup,
	.udc_start = usbf_gadget_start,
	.udc_stop = usbf_gadget_stop,
};

/*-----------------------------------------------------------------------
 *	UDC device Driver operation functions				*
 *----------------------------------------------------------------------*/
static int usbf_probe(struct platform_device *ofdev)
{
	int ret = -ENOMEM, i;
	struct f_drv *chip;
	struct device_node *np = ofdev->dev.of_node;

	if (rzn1_sysctrl_readl(RZN1_SYSCTRL_REG_CFG_USB) &
			(1 << RZN1_SYSCTRL_REG_CFG_USB_H2MODE)) {
		dev_warn(&ofdev->dev, "disabled in H2 (host) mode\n");
		return -1;
	}

	chip = devm_kzalloc(&ofdev->dev, sizeof(*chip), GFP_KERNEL);
	if (chip == NULL)
		return -ENOMEM;
	chip->dev = &ofdev->dev;

	chip->clk_fw = devm_clk_get(chip->dev, "axi");
	if (IS_ERR(chip->clk_fw)) {
		ret = PTR_ERR(chip->clk_fw);
		goto clk_failed;
	}
	ret = clk_prepare_enable(chip->clk_fw);
	if (ret) {
		dev_err(chip->dev, "can not enable the clock\n");
		goto clk_failed;
	}

	chip->regs = of_iomap(np, 0);
	if (!chip->regs) {
		dev_err(chip->dev, "unable to get register bank\n");
		goto invalid_mapping;
	}
	if (!(readl(&chip->regs->usbssconf) & (1 << 16))) {
		dev_err(chip->dev, "Invalid USBF configuration, bailing\n");
		goto invalid_config;
	}
	chip->dma_threshold = 64;
	chip->dma_ram_size = 5024;

	chip->dma_ram_used = EP0_RAM_USED; /* after endpoint zero memory */

	TRACE("%s USB Conf: %08x\n", __func__, readl(&chip->regs->usbssconf));

	/* Resetting the PLL is handled via the clock driver as it has common
	 * registers with USB Host */
	writel(readl(&chip->regs->epctr) & ~D_SYS_EPC_RST, &chip->regs->epctr);

	spin_lock_init(&chip->lock);

	/* modify in register gadget process */
	chip->gadget.speed = USB_SPEED_FULL;
	chip->gadget.max_speed = USB_SPEED_HIGH;
	chip->gadget.ops = &usbf_gadget_ops;

	/* name: Identifies the controller hardware type. */
	chip->gadget.name = driver_name;
	chip->gadget.dev.parent = &ofdev->dev;
	/* gadget.ep0 is a pointer */
	chip->gadget.ep0 = &chip->ep[0].ep;

	INIT_LIST_HEAD(&chip->gadget.ep_list);
	/* we have a canned request structure to allow sending packets
	 * as reply to get_status requests */
	INIT_LIST_HEAD(&chip->setup_reply.queue);

	for (i = 0; i < CFG_NUM_ENDPOINTS; ++i) {
		struct f_endpoint init = {
			.ep = {
				.ops = &usbf_ep_ops,
				.maxpacket = CFG_EPX_MAX_PACKET_SIZE,
			},
			.id = i,
			.disabled = 1,
			.chip = chip,
		};
		struct f_endpoint *ep = chip->ep + i;

		if (!(readl(&chip->regs->usbssconf) & (1 << (16 + i)))) {
			TRACE("%s endpoint %d is not available\n", __func__, i);
			continue;
		}
		*ep = init;
		sprintf(ep->name, "ep%d", i);
		ep->ep.name = ep->name;
		if (i == 0)
			ep->ep.caps.type_control = true;
		else if (i < 6)
			ep->ep.caps.type_bulk = true;
		else if (i < 10)
			ep->ep.caps.type_int = true;
		else
			ep->ep.caps.type_iso = true;

		ep->ep.caps.dir_in = true;
		ep->ep.caps.dir_out = true;

		ep->has_dma = !!(readl(&chip->regs->usbssconf) & (1 << i));
		usb_ep_set_maxpacket_limit(&ep->ep, (unsigned short) ~0);

		INIT_LIST_HEAD(&ep->queue);
		spin_lock_init(&ep->lock);

		if (ep->id == 0) {
			usbf_ep0_init(ep);
		} else {
			usbf_epn_init(ep);
			list_add_tail(&ep->ep.ep_list,
				      &chip->gadget.ep_list);
		}
	}

	/* Finish initialization by getting possible overrides from DT */
	if (np) {
		of_property_read_u32(np,
				"renesas,dma-threshold", &chip->dma_threshold);
		of_property_read_u32(np,
				"renesas,sram-size", &chip->dma_ram_size);
	}

	/* Not specifying the sram size of the IP is not a problem per se,
	 * however if you run out your configuration of endpoint over allocates
	 * there will be no way to know, and you will have silent failure */
	if (!chip->dma_ram_size)
		dev_warn(chip->dev,
			"unknown SRAM size, any overflow will be silent\n");

	/* Tasklet and associated FIFO */
	tasklet_init(&chip->tasklet, usbf_tasklet,
			(unsigned long)chip);
	INIT_KFIFO(chip->fifo);

	/* request irqs */
	chip->usb_epc_irq = irq_of_parse_and_map(np, 0);
	if (!chip->usb_epc_irq) {
		ret = -EINVAL;
		goto err_noirq;
	}
	ret = devm_request_irq(chip->dev, chip->usb_epc_irq, usbf_epc_irq, 0,
				driver_name, chip);
	if (ret) {
		dev_err(chip->dev, "cannot request irq %d err %d\n",
				chip->usb_epc_irq, ret);
		goto err_noirq;
	}

	chip->usb_irq = irq_of_parse_and_map(np, 1);
	if (!chip->usb_irq) {
		ret = -EINVAL;
		goto err_noirq;
	}
	ret = devm_request_irq(chip->dev, chip->usb_irq, usbf_irq, 0,
				driver_name, chip);
	if (ret) {
		dev_err(chip->dev, "cannot request irq %d err %d\n",
				chip->usb_irq, ret);
		goto err_noirq;
	}

	writel(readl(&chip->regs->sysmctr) | D_SYS_WBURST_TYPE, &chip->regs->sysmctr);

	writel(D_USB_INT_SEL | D_USB_SOF_RCV | D_USB_SOF_CLK_MODE, &chip->regs->control);

	/* Enable reset and mode change interrupts */
	writel(D_USB_USB_RST_EN | D_USB_SPEED_MODE_EN, &chip->regs->int_enable);
	/* Endpoint zero is always enabled anyway */
	usbf_ep_enable(&chip->ep[0].ep, &ep0_desc);

	ret = usb_add_gadget_udc(&ofdev->dev, &chip->gadget);
	if (ret)
		goto err5;

	platform_set_drvdata(ofdev, chip);

	writel(D_SYS_VBUS_INTEN, &chip->regs->sysbinten);
#ifdef DEBUG
	for (i = 0; i < CFG_NUM_ENDPOINTS; ++i) {
		struct f_endpoint *ep = chip->ep + i;
		struct f_regs_ep *ep_reg = &chip->regs->ep[ep->id - 1];

		TRACE("%s %s %08x\n", ep->name,
			(readl(&chip->regs->usbssconf) & (1 << i)) ?
					"DMA" : "(no DMA)",
			readl(&ep_reg->pckt_adrs));
	}
#endif
	pr_info("%s completed (gadget %s)\n", __func__, chip->gadget.name);

#ifdef	CONFIG_DEBUG_FS
	usbf = chip;
	usbf_debug_init(chip);
#endif

	return 0;
err5:
err_noirq:
	clk_disable_unprepare(chip->clk_fw);
invalid_config:
clk_failed:
invalid_mapping:
	return ret;
}

static int usbf_remove(struct platform_device *ofdev)
{
	struct f_drv *chip = platform_get_drvdata(ofdev);

	TRACE("%s\n", __func__);

	usb_del_gadget_udc(&chip->gadget);

	tasklet_disable(&chip->tasklet);
	tasklet_kill(&chip->tasklet);

	/* Disable VBUS interrupt */
	writel(0, &chip->regs->sysbinten);

	iounmap(chip->regs);
	clk_disable_unprepare(chip->clk_fw);

#ifdef	CONFIG_DEBUG_FS
	debugfs_remove(chip->debugfs_regs);
	debugfs_remove(chip->debugfs_eps);
	debugfs_remove(chip->debugfs_root);
#endif

	return 0;
}

#ifdef	CONFIG_DEBUG_FS

#include <linux/debugfs.h>
#include <linux/seq_file.h>

static const char *ep_status[2][32] = { {
	"SETUP_INT",
	"STG_START_INT",
	"STG_END_INT",
	"STALL_INT",
	"IN_INT",
	"OUT_INT",
	"OUT_OR_INT",
	"OUT_NULL_INT",
	"IN_EMPTY",
	"IN_FULL",
	"IN_DATA",
	"IN_NAK_INT",
	"OUT_EMPTY",
	"OUT_FULL",
	"OUT_NULL",
	"OUT_NAK_INT",
	"PERR_NAK_INT",
	"PERR_NAK",
	"PID",
},
{
	"IN_EMPTY",
	"IN_FULL",
	"IN_DATA",
	"IN_INT",
	"IN_STALL_INT",
	"IN_NAK_ERR_INT",
	"(6)",
	"IN_END_INT",
	"(8)", "(9)",
	"IPID",
	"(11)", "(12)", "(13)", "(14)", "(15)",
	"OUT_EMPTY",
	"OUT_FULL",
	"OUT_NULL_INT",
	"OUT_INT",
	"OUT_STALL_INT",
	"OUT_NAK_ERR_INT",
	"OUT_OR_INT",
	"OUT_END_INT",
	"(24)", "(25)", "(26)", "(27)",
	"OPID",
} };

static ssize_t _usbf_dbg_show(struct seq_file *s, void *unused)
{
	struct f_drv *chip = usbf;
	int i;

	seq_puts(s, "USBF Driver Configuration\n");

	for (i = 0; i < CFG_NUM_ENDPOINTS; ++i) {
		struct f_endpoint *ep = chip->ep + i;
		struct f_regs_ep *regs =
			i ? ep->reg :
			(struct f_regs_ep *)&chip->regs->ep0;
		uint32_t reg[2] = {
			readl(&regs->status),
			ep->status };
		int ri;

		if (ep->disabled)
			continue;
		seq_printf(s, "%s: dir:%s status %08x/%08x seq:%02x %s%s: ",
			ep->name,
			usb_endpoint_dir_in(ep->desc) ? "input" : "output",
			reg[0], reg[1], ep->seq,
			ep->trace ? "TRACE " : "",
			list_empty(&ep->queue) ? "(empty)" : "(queued)"
			);
		if (!list_empty(&ep->queue)) {
			struct f_req *req = list_first_entry(&ep->queue,
							struct f_req, queue);

			seq_printf(s, "[%02x%s]:%d/%d bytes",
				req->seq,
				req->req.status == -EBUSY ? " DMA" :
					req->req.status == 0 ? " DONE?!" : "",
				req->req.actual, req->req.length);
		}
		seq_puts(s, "\n   >");

		for (ri = 0; ri < 2; ri++) {
			uint32_t r = reg[ri];
			int ci = 0;

			seq_puts(s, "\n   >");
			while (r) {
				int b = ffs(r) - 1;

				r &= ~(1 << b);
				if (ep_status[!!i][b])
					seq_printf(s, "%s ", ep_status[!!i][b]);
				else
					seq_printf(s, "[%d] ", b);
				ci = (ci + 1) % 6;
				if (!ci && r)
					seq_printf(s, "\n        ");
			}
		}
		seq_puts(s, "\n");
	}

	return 0;
}

static int _usbf_dbg_open(struct inode *inode, struct file *file)
{
	return single_open(file, _usbf_dbg_show, &inode->i_private);
}

static const struct file_operations _fops = {
	.open		= _usbf_dbg_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int usbf_debugfs_ops_ep_trace_show(void *data, uint64_t *value)
{
	struct f_drv *chip = usbf;
	int i;

	*value = 0;

	for (i = 0; i < CFG_NUM_ENDPOINTS; ++i) {
		struct f_endpoint *ep = chip->ep + i;

		*value |= ep->trace << i;
	}

	return 0;
}

static int usbf_debugfs_ops_ep_trace_set(void *data, uint64_t value)
{
	struct f_drv *chip = usbf;
	int i;

	for (i = 0; i < CFG_NUM_ENDPOINTS; ++i) {
		struct f_endpoint *ep = chip->ep + i;

		ep->trace = !!(value & (1 << i));
	}

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(usbf_debugfs_ops_ep_trace,
	usbf_debugfs_ops_ep_trace_show,
	usbf_debugfs_ops_ep_trace_set,
	"%llu\n");

static int usbf_debug_init(struct f_drv *chip)
{
	chip->debugfs_root = debugfs_create_dir("usbf", NULL);

	chip->debugfs_regs = debugfs_create_file("regs", S_IRUGO,
				chip->debugfs_root, NULL, &_fops);

	chip->debugfs_eps = debugfs_create_file("ep_trace", S_IWUGO | S_IRUGO,
				chip->debugfs_root, NULL, &usbf_debugfs_ops_ep_trace);

	return 0;
}

#endif /* CONFIG_DEBUG_FS */

/*-------------------------------------------------------------------------*/
static const struct of_device_id usbf_match[] = {
	{ .compatible = "renesas,rzn1-usbf", },
	{ .compatible = "renesas,usbf", },
	{},
};

MODULE_DEVICE_TABLE(of, usbf_match);

static struct platform_driver udc_driver = {
	.driver = {
		.name = driver_name,
		.owner = THIS_MODULE,
		.of_match_table = usbf_match,
	},
	.probe          = usbf_probe,
	.remove         = usbf_remove,
};

module_platform_driver(udc_driver);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:usbf-renesas");
