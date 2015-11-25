// SPDX-License-Identifier: GPL-2.0
/*
 * Renesas RZ/N1 USB Device usb gadget driver
 *
 * Copyright 2015 Renesas Electronics Europe Ltd.
 * Author: Michel Pollet <michel.pollet@bp.renesas.com>,<buserror@gmail.com>
 */
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include "usbf-renesas.h"

/*
 * Endpoint 0 callbacks
 */

static int usbf_ep0_flush_buffer(struct f_regs_ep0 *ep_reg, uint32_t bits)
{
	int res = 100000;

	if ((readl(&ep_reg->status) & bits) == bits)
		return res;

	writel(readl(&ep_reg->control) | D_EP0_BCLR, &ep_reg->control);

	while (res-- && ((readl(&ep_reg->status) & bits) != bits))
		;

	if (!res)
		pr_warn("%s timeout on buffer clear!\n", __func__);

	return res;
}

static void usbf_ep0_clear_inak(struct f_regs_ep0 *ep_reg)
{
	writel((readl(&ep_reg->control) | D_EP0_INAK_EN) & ~D_EP0_INAK,
	       &ep_reg->control);
}

static void usbf_ep0_clear_onak(struct f_regs_ep0 *ep_reg)
{
	writel((readl(&ep_reg->control)) & ~D_EP0_ONAK, &ep_reg->control);
}

static void usbf_ep0_stall(struct f_regs_ep0 *ep_reg)
{
	writel(readl(&ep_reg->control) | D_EP0_STL, &ep_reg->control);
}

static int usbf_ep0_enable(struct f_endpoint *ep)
{
	struct f_drv *chip = ep->chip;
	struct f_regs_ep0 *ep_reg = &chip->regs->ep0;

	writel(D_EP0_INAK_EN | D_EP0_BCLR, &ep_reg->control);
	writel(D_EP0_SETUP_EN | D_EP0_STG_START_EN |
	       D_EP0_OUT_EN,
	       &ep_reg->int_enable);
	return 0;
}

static void usbf_ep0_reset(struct f_endpoint *ep)
{
	struct f_drv *chip = ep->chip;
	struct f_regs_ep0 *ep_reg = &chip->regs->ep0;

	writel(readl(&ep_reg->control) | D_EP0_BCLR, &ep_reg->control);
}

static int usbf_ep0_send1(struct f_endpoint *ep, u32 *src, int reqlen)
{
	struct f_drv *chip = ep->chip;
	struct f_regs_ep0 *ep_reg = &chip->regs->ep0;
	u32 control;
	int len, err;
	u32 val;
	int pkt_words = reqlen / sizeof(*src);

	/* Wait until there is space to write the pkt */
	err = readl_poll_timeout(&ep_reg->status, val, (val & D_EP0_IN_EMPTY),
				 0, 10000);
	if (err)
		return -ETIMEDOUT;

	/* Note, we don't care about endianness here, as the IP
	 * and the core will have the same layout anyway, so we
	 * can happily ignore it
	 */
	if (pkt_words) {
		iowrite32_rep(&ep_reg->write, src, pkt_words);
		src += pkt_words;
	}

	control = readl(&ep_reg->control);

	/* if we have stray bytes, write them off too, and mark the
	 * control registers so it knows only 1,2,3 bytes are valid in
	 * the last write we made
	 */
	len = reqlen & (sizeof(*src) - 1);
	if (len) {
		writel(*src, &ep_reg->write);
		control |= (len << 5);
	}

	writel(control | D_EP0_DEND, &ep_reg->control);

	return 0;
}

static int usbf_ep0_send(struct f_endpoint *ep, struct f_req *req)
{
	struct f_drv *chip = ep->chip;
	struct f_regs_ep0 *ep_reg = &chip->regs->ep0;

	/* Special handling for internally generated NULL packets */
	if (!req) {
		writel(readl(&ep_reg->control) | D_EP0_DEND, &ep_reg->control);
		return 0;
	}

	if (req->req.length) {
		void *src = req->req.buf;
		int bytes = req->req.length;
		int maxpkt_bytes = ep->ep.maxpacket;
		int ret;

		while (bytes > 0) {
			int pkt_bytes = min(bytes, maxpkt_bytes);

			ret = usbf_ep0_send1(ep, src, pkt_bytes);
			if (ret < 0) {
				req->req.status = ret;
				return ret;
			}

			bytes -= pkt_bytes;
			src += pkt_bytes;
		}
		req->req.actual = req->req.length;
		req->req.status = 0;
	}

	/* UDC asking for a ZLP to follow? */
	if (req->req.length == 0 || req->req.zero)
		req->req.status = usbf_ep0_send1(ep, NULL, 0);

	TRACERQ(req, "%s[%d][%3d] sent %d\n", __func__,
		ep->id, req->seq, req->req.length);

	return req->req.status;
}

/*
 * This can be called repeatedly until the request is done
 */
static int usbf_ep0_recv(struct f_endpoint *ep, struct f_req *req)
{
	return req->req.status;
}

static void usbf_ep0_out_isr(struct f_endpoint *ep, struct f_req *req)
{
	struct f_regs_ep0 *ep_reg = &ep->chip->regs->ep0;
	u32 reqlen = readl(&ep_reg->length);
	int len = reqlen;
	u32 *buf  = req->req.buf + req->req.actual;

	TRACEEP(ep, "%s[%3d] size %d (%d/%d)\n", __func__, req->seq, len,
		req->req.actual, req->req.length);

	if (len) {
		ioread32_rep(&ep_reg->read, buf, len);
		buf += len;
	}
	req->req.actual += reqlen;

	if (reqlen != usb_endpoint_maxp(ep->desc))
		req->req.status = 0;
}

/*
 * result of setup packet
 */
#define CX_IDLE		0
#define CX_FINISH	1
#define CX_STALL	2

static void usbf_ep0_setup(struct f_endpoint *ep, struct usb_ctrlrequest *ctrl)
{
	int ret = CX_IDLE;
	struct f_drv *chip = ep->chip;
	struct f_regs_ep0 *ep_reg = &chip->regs->ep0;
	u16 value = ctrl->wValue & 0xff;

	if (ctrl->bRequestType & USB_DIR_IN)
		ep->desc->bEndpointAddress = USB_DIR_IN;
	else
		ep->desc->bEndpointAddress = USB_DIR_OUT;

	/*
	 * This is mandatory, as for the moment at least, we never get an
	 * interrupt/status flag indicating the speed has changed. And without
	 * a speed change flag, the gadget upper layer is incapable of finding
	 * a valid configuration
	 */
	if (readl(&chip->regs->status) & D_USB_SPEED_MODE)
		chip->gadget.speed = USB_SPEED_HIGH;
	else
		chip->gadget.speed = USB_SPEED_FULL;

	if ((ctrl->bRequestType & USB_TYPE_MASK) == USB_TYPE_STANDARD) {
		switch (ctrl->bRequest) {
		case USB_REQ_SET_CONFIGURATION:
			TRACEEP(ep, "usbf: set_cfg(%d)\n", value);
			if (!value) {
				/* Disable all endpoints other than EP0 */
				writel(readl(&chip->regs->control) & ~D_USB_CONF,
				       &chip->regs->control);

				usb_gadget_set_state(&chip->gadget, USB_STATE_ADDRESS);
			} else {
				/* Enable all endpoints */
				writel(readl(&chip->regs->control) | D_USB_CONF,
				       &chip->regs->control);

				usb_gadget_set_state(&chip->gadget, USB_STATE_CONFIGURED);
			}
			ret = CX_IDLE;
			break;

		case USB_REQ_SET_ADDRESS:
			TRACEEP(ep, "usbf: set_addr(0x%04X)\n", ctrl->wValue);
			writel(value << 16, &chip->regs->address);
			usb_gadget_set_state(&chip->gadget, USB_STATE_ADDRESS);
			ret = CX_FINISH;
			break;

		case USB_REQ_CLEAR_FEATURE:
			TRACEEP(ep, "usbf: clr_feature(%d, %d)\n",
				ctrl->bRequestType & 0x03, ctrl->wValue);
			switch (ctrl->wValue) {
			case 0:    /* [Endpoint] halt */
				/* TODO ? */
			/*	ep_reset(chip, ctrl->wIndex); */
				TRACEEP(ep, "endpoint reset ?!?\n");
				ret = CX_FINISH;
				break;
			case 1:    /* [Device] remote wake-up */
			case 2:    /* [Device] test mode */
			default:
				ret = CX_STALL;
				break;
			}
			break;

		case USB_REQ_SET_FEATURE:
			TRACEEP(ep, "usbf: set_feature(%d, %d)\n",
				ctrl->wValue, ctrl->wIndex & 0xf);
			switch (ctrl->wValue) {
			case 0:    /* Endpoint Halt */
				ret = CX_FINISH;
				/* TODO */
			/*	id = ctrl->wIndex & 0xf; */
				break;
			case 1:    /* Remote Wakeup */
			case 2:    /* Test Mode */
			default:
				ret = CX_STALL;
				break;
			}
			break;
		case USB_REQ_GET_STATUS:
			TRACEEP(ep, "usbf: get_status(%d, %d, type %d)\n",
				ctrl->wValue, ctrl->wIndex,
			      ctrl->bRequestType & USB_RECIP_MASK);
			chip->setup[0] = 0;
			switch (ctrl->bRequestType & USB_RECIP_MASK) {
			case USB_RECIP_DEVICE:
				chip->setup[0] = 1 << USB_DEVICE_SELF_POWERED;
				break;
			}
			/* mark it as static, don't 'free' it */
			chip->setup_reply.req.complete = NULL;
			chip->setup_reply.req.buf = &chip->setup;
			chip->setup_reply.req.length = 2;
			usb_ep_queue(&ep->ep, &chip->setup_reply.req, 0);
			ret = CX_FINISH;
			break;
		case USB_REQ_SET_DESCRIPTOR:
			TRACEEP(ep, "usbf: set_descriptor\n");
			break;
		}
	} /* if ((ctrl->bRequestType & USB_TYPE_MASK) == USB_TYPE_STANDARD) */

	if (!chip->driver) {
		dev_warn(chip->dev, "Spurious SETUP");
		ret = CX_STALL;
	} else if (ret == CX_IDLE && chip->driver->setup) {
		if (chip->driver->setup(&chip->gadget, ctrl) < 0)
			ret = CX_STALL;
		else
			ret = CX_FINISH;
	}

	switch (ret) {
	case CX_FINISH:
		break;
	case CX_STALL:
		usbf_ep0_stall(ep_reg);
		TRACEEP(ep, "usbf: cx_stall!\n");
		break;
	case CX_IDLE:
		TRACEEP(ep, "usbf: cx_idle?\n");
	default:
		break;
	}
}

static int usbf_req_is_control_no_data(struct usb_ctrlrequest *ctrl)
{
	return (ctrl->wLength == 0);
}

static int usbf_req_is_control_read(struct usb_ctrlrequest *ctrl)
{
	if (ctrl->wLength && (ctrl->bRequestType & USB_DIR_IN))
		return 1;
	return 0;
}

static int usbf_req_is_control_write(struct usb_ctrlrequest *ctrl)
{
	if (ctrl->wLength && !(ctrl->bRequestType & USB_DIR_IN))
		return 1;
	return 0;
}

static void usbf_ep0_interrupt(struct f_endpoint *ep)
{
	struct f_regs_ep0 *ep_reg = &ep->chip->regs->ep0;
	struct f_drv *chip = ep->chip;
	struct usb_ctrlrequest *ctrl = (struct usb_ctrlrequest *)chip->setup;

/*	TRACE("%s status %08x control %08x\n", __func__, ep->status, readl(&ep_reg->control)); */

	if (ep->status & D_EP0_OUT_INT) {
		struct f_req *req;

		spin_lock_irq(&ep->lock);
		req = list_first_entry_or_null(&ep->queue, struct f_req, queue);
		spin_unlock_irq(&ep->lock);

		if (req)
			usbf_ep0_out_isr(ep, req);
	}

	if (ep->status & D_EP0_SETUP_INT) {
		chip->setup[0] = readl(&chip->regs->setup_data0);
		chip->setup[1] = readl(&chip->regs->setup_data1);

		TRACEEP(ep, "%s SETUP %08x %08x dir %s len:%d\n", __func__,
			chip->setup[0], chip->setup[1],
		      (ctrl->bRequestType & USB_DIR_IN) ? "input" : "output",
		      readl(&ep->chip->regs->ep0.length));

		if (usbf_req_is_control_write(ctrl))
			usbf_ep0_clear_onak(ep_reg);

		if (usbf_req_is_control_read(ctrl)) {
			usbf_ep0_flush_buffer(ep_reg, D_EP0_IN_EMPTY);
			usbf_ep0_clear_inak(ep_reg);
		}

		usbf_ep0_setup(ep, ctrl);
	}
	if (ep->status & D_EP0_STG_START_INT) {
		TRACEEP(ep, "%s START %08x %08x (empty: %s)\n", __func__,
			chip->setup[0], chip->setup[1],
		      (ep->status & D_EP0_IN_EMPTY) ?
				"IN empty" : "IN NOT empty");

		if (usbf_req_is_control_read(ctrl))
			usbf_ep0_clear_onak(ep_reg);

		if (usbf_req_is_control_write(ctrl)) {
			usbf_ep0_flush_buffer(ep_reg, D_EP0_OUT_EMPTY);
			usbf_ep0_clear_inak(ep_reg);
		}
		if (usbf_req_is_control_no_data(ctrl)) {
			usbf_ep0_flush_buffer(ep_reg, D_EP0_IN_EMPTY);
			usbf_ep0_clear_inak(ep_reg);
		}

		/* TODO, we should send a NULL packet for Control-No-Data, but read a NULL packet for Control-Read */
		usbf_ep0_send(ep, NULL);
	}

	ep->status = 0;
}

static const struct f_endpoint_drv usbf_ep0_callbacks = {
	.enable = usbf_ep0_enable,
	/* No DMA callbacks for endpoint 0 */
	.recv[USBF_PROCESS_PIO] = usbf_ep0_recv,
	.send[USBF_PROCESS_PIO] = usbf_ep0_send,
	.interrupt = usbf_ep0_interrupt,
	.reset = usbf_ep0_reset,
};

int usbf_ep0_init(struct f_endpoint *ep)
{
	struct f_regs_ep0 *ep_reg = &ep->chip->regs->ep0;

	ep->drv = &usbf_ep0_callbacks;
	ep->ep.maxpacket = CFG_EP0_MAX_PACKET_SIZE;
	strcat(ep->name, "-control");

	usbf_ep0_flush_buffer(ep_reg, D_EP0_OUT_EMPTY | D_EP0_IN_EMPTY);

	return 0;
}
