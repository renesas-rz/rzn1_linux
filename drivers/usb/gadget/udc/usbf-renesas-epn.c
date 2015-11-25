// SPDX-License-Identifier: GPL-2.0
/*
 * Renesas RZ/N1 USB Peripheral driver (USB gadget)
 *
 * Copyright 2015 Renesas Electronics Europe Ltd.
 * Author: Michel Pollet <michel.pollet@bp.renesas.com>,<buserror@gmail.com>
 */
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include "usbf-renesas.h"

/*
 * Currently, if the 'packet size' of the descriptor is used to configure
 * the dma/sram for the endpoints, it works well for when the device starts at
 * full speed. However, if the gadget starts at high speed, the memory allocated
 * won't be enough if the link is then renegotiated  to full speed.
 * We need a bit of heuristics at enable time to know if we ought to allocate
 * more, or if we can use the descriptor values.
 */
/* Example of memory configuration with 5024 * 32 bits words
 * 5024 * 32 bits = 20096 bytes of device memory
	ep1-bulk	2 * 512		1024
	ep2-bulk	2 * 512		1024
	ep3-bulk	2 * 512		1024
	ep4-bulk	2 * 512		1024
	ep5-bulk	2 * 512		1024	   5120
	ep6-int		2 * 64		128	- No DMA
	ep7-int		2 * 64		128
	ep8-int		2 * 64		128
	ep9-int		2 * 64		128	+   512
	ep10-iso	2 * 1024	2048
	ep11-iso	2 * 1024	2048
	ep12-iso	2 * 1024	2048
	ep13-iso	2 * 1024	2048
	ep14-iso	2 * 1024	2048
	ep15-iso	2 * 1024	2048	+ 12288
						= 17920
 */
static int usbf_epn_enable(struct f_endpoint *ep)
{
	struct f_drv *chip = ep->chip;
	struct f_regs_ep *ep_reg = ep->reg;
	int in = usb_endpoint_dir_in(ep->desc);

	/* Get the number of packets from EP config, CHIP config, or default */
	if (!ep->max_nr_pkts)
		ep->max_nr_pkts = chip->dma_pkt_count[ep->type];
	if (!ep->max_nr_pkts)
		ep->max_nr_pkts = 2;

	TRACEEP(ep, "%s[%d] %s maxpackets %d/%d\n", __func__, ep->id,
		in ? "IN" : "OUT", ep->ep.maxpacket, ep->ep.maxpacket_limit);

	/* Set the internal IP base address, and increment the pointer with
	 * the number of packets we plan to keep for that endpoint
	 * Note: We do this even if this endpoint is 'no dma' as it will still
	 * need some memory to function; this is the case for the interrupt
	 * endpoints
	 */
	writel((ep->chip->dma_ram_used << 16) | usb_endpoint_maxp(ep->desc),
	       &ep_reg->pckt_adrs);
	ep->chip->dma_ram_used +=
		(ep->max_nr_pkts * usb_endpoint_maxp(ep->desc)) / 4;
	/* Warn if we go overboard */
	if (chip->dma_ram_size &&
	    ep->chip->dma_ram_used > chip->dma_ram_size) {
		dev_warn(chip->dev, "EP[%d] SRAM overflow (%d/%d)!\n",
			 ep->id, chip->dma_ram_used, chip->dma_ram_size);
	}
	TRACE("%s[%d] packets %d*%d pckt_adrs %08x\n",
	      __func__, ep->id,
		ep->max_nr_pkts, usb_endpoint_maxp(ep->desc),
		readl(&ep_reg->pckt_adrs));

	writel(in ?	0 :
			D_EPN_OUT_EN | D_EPN_OUT_END_EN,
		&ep_reg->int_enable);
	/* Clear, set endpoint direction, and enable */
	writel(D_EPN_EN | D_EPN_BCLR | (in ? 0 : D_EPN_DIR0), &ep_reg->control);
	return 0;
}

static void usbf_epn_disable(struct f_endpoint *ep)
{
	struct f_regs_ep *ep_reg = ep->reg;

	TRACEEP(ep, "%s[%d]\n", __func__, ep->id);

	/* Disable endpoint */
	writel(readl(&ep_reg->control) & ~D_EPN_EN, &ep_reg->control);
}

static void usbf_epn_reset(struct f_endpoint *ep)
{
	struct f_regs_ep *ep_reg = ep->reg;

	writel(readl(&ep_reg->control) | D_EPN_BCLR, &ep_reg->control);
}

static int usbf_epn_send1(struct f_endpoint *ep, u32 *src, int reqlen)
{
	struct f_regs_ep *ep_reg = ep->reg;
	u32 control;
	int len, err;
	u32 val;
	int pkt_words = reqlen / sizeof(*src);

	/* Wait until there is space to write the pkt */
	err = readl_poll_timeout(&ep_reg->status, val, (val & D_EPN_IN_EMPTY),
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

	writel(control | D_EPN_DEND, &ep_reg->control);

	return 0;
}

static int usbf_epn_send(struct f_endpoint *ep, struct f_req *req)
{
	struct f_regs_ep *ep_reg = ep->reg;

	/* Special handling for internally generated NULL packets */
	if (!req) {
		writel(readl(&ep_reg->control) | D_EPN_DEND, &ep_reg->control);
		return 0;
	}

	if (req->req.length) {
		void *src = req->req.buf;
		int bytes = req->req.length;
		int maxpkt_bytes = ep->ep.maxpacket;
		int ret;

		while (bytes > 0) {
			int pkt_bytes = min(bytes, maxpkt_bytes);

			ret = usbf_epn_send1(ep, src, pkt_bytes);
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
		req->req.status = usbf_epn_send1(ep, NULL, 0);

	TRACERQ(req, "%s[%d][%3d] sent %d\n", __func__,
		ep->id, req->seq, req->req.length);

	return req->req.status;
}

static int usbf_epn_pio_recv(struct f_endpoint *ep, struct f_req *req, uint32_t reqlen)
{
	struct f_regs_ep *ep_reg = ep->reg;

	u32 *buf = req->req.buf + req->req.actual;
	int len = (reqlen + sizeof(*buf) - 1) /  sizeof(*buf);

	TRACERQ(req, "%s[%d][%3d] size %d (%d/%d)\n", __func__,
		ep->id, req->seq, len,
	      req->req.actual, req->req.length);

	if (len) {
		ioread32_rep(&ep_reg->read, buf, len);
		buf += len;
	}

	req->req.actual += reqlen;

	/* If we get a whole packet, we don't know if its the end of the
	 * request yet. If it was everything, then we will get a null packet
	 * as well.
	 */
	if (reqlen != usb_endpoint_maxp(ep->desc))
		req->req.status = 0;

	return req->req.status;
}

/* This is called from the tasklet to determine if we have received data.
 * All of the processing is interrupt driven, so this function simply returns
 * the status of the request.
 */
static int usbf_epn_recv(struct f_endpoint *ep, struct f_req *req)
{
	return req->req.status;
}

/*
 * This setups a DMA transfer for both directions, to and from the device.
 */
static int usbf_epn_dma(struct f_endpoint *ep, struct f_req *req, uint32_t size)
{
	struct f_regs_ep *ep_reg = ep->reg;
	struct f_regs_epdma *dma = &ep->chip->regs->epdma[ep->id];
	u32 max_size = usb_endpoint_maxp(ep->desc);
	u32 pkt_count;
	u32 last_size;
	u32 control = 0;

	/* Should not be necessary here, but lets be paranoid */
	writel(D_EPN_STOP_MODE | D_EPN_STOP_SET | D_EPN_DMAMODE0, &ep_reg->dma_ctrl);

	/* Ensure DMA is not trying to handle any odd bytes */
	size &= ~3;

	/* trim the size to max packets worth of stuff */
	if (size > CFG_EPX_MAX_PACKET_CNT * max_size)
		size = CFG_EPX_MAX_PACKET_CNT * max_size;

	pkt_count = (size + max_size - 1) / max_size;
	last_size = size % max_size;

	/* keep it around for the usbf_epn_dma_complete() function sake */
	req->dma_pkt_count = pkt_count - 1;

	TRACERQ(req, "%s[%d][%3d] %d/%d size %d - pkt %d size %d last_size %d = %d\n",
		__func__, ep->id, req->seq,
		req->req.actual, req->req.length, size,
		(pkt_count - 1), max_size, last_size,
		((pkt_count - 1) * max_size) + last_size);

	writel(pkt_count << 16, &ep_reg->len_dcnt);
	/* set address, offset by what was already transferred */
	writel(req->req.dma + req->req.actual, &dma->epntadr);
	/* Packet size + last packet size */
	writel((last_size << 16) | max_size, &dma->epndcr2);
	/* Number of packet again, enable + direction flag */
	/* Always needs to be > 0 here, if not in burst mode */
	writel((((pkt_count ? pkt_count : 1) & 0xff) << 16) |
		D_SYS_EPN_REQEN |
		(req->to_host ? 0 : D_SYS_EPN_DIR0),
		&dma->epndcr1);

	/* Mark it so we don't try to restart a transfer */
	req->req.status = -EBUSY;
	control = D_EPN_DMA_EN | D_EPN_STOP_MODE | D_EPN_STOP_SET | D_EPN_DMAMODE0;
	if (pkt_count > 1 && last_size == 0)
		control |= D_EPN_BURST_SET;
	/* If we are sending, and are either sending a full packet or sending
	 * the exact remaining of the request using the DMA (no unaligned
	 * length) then we set the DEND flag so the DMA end will also send the
	 * packet.
	 */
	if (req->to_host) {
		if (last_size == max_size || req->req.actual + size == req->req.length)
			control |= D_EPN_DEND_SET;
	}
	/* Start it... */
	writel(control, &ep_reg->dma_ctrl);

	return 0;
}

/* Handle DMA rx complete (OUT_END) interrupt */
static void usbf_epn_out_end_isr(struct f_endpoint *ep, struct f_req *req)
{
	struct f_regs_ep *ep_reg = ep->reg;
	struct f_regs_epdma *dma = &ep->chip->regs->epdma[ep->id];

	u32 reqlen = readl(&ep_reg->len_dcnt) & D_EPN_LDATA;
	/* number of packets has been decremented from the dma_pkt_count */
	u32 pkts_left = (readl(&dma->epndcr1) >> 16) & 0xff;
	u32 pkts_got = 0;

	/* Clear REQEN, i.e. disable DMA */
	writel((req->to_host ? 0 : D_SYS_EPN_DIR0), &dma->epndcr1);

	/* The number of packets received is the difference between what we
	 * asked for and the uncompleted packets.
	 */
	if (req->dma_pkt_count)
		pkts_got = req->dma_pkt_count - pkts_left + 1;

	/* Update amount transferred.
	 * The burst and non-burst method have the same behaviour
	 * regarding the calculation of what has been transferred
	 */
	req->req.actual += pkts_got * usb_endpoint_maxp(ep->desc);
	req->req.actual += req->dma_non_burst_bytes;

	TRACERQ(req, "%s[%d][%3d] reqlen %d pkts_sent %d actual %d/%d\n",
		__func__, ep->id, req->seq,
		reqlen, pkts_got,
		req->req.actual, req->req.length);

	/* DMA it! */
	if (reqlen >= 4) {
		usbf_epn_dma(ep, req, reqlen);
		req->dma_non_burst_bytes = reqlen & ~3;
		return;
	}

	usbf_epn_pio_recv(ep, req, reqlen);
}

/* Handle rx data received (OUT) interrupt.
 * The hardware has captured a USB packet in it's internal RAM and we need to
 * use PIO or start DMA to move it out. The OUT_END interrupt tells us when the
 * DMA of one USB packet has finished, and so is used to start subsequent DMA
 * transfers for this request.
 */
static void usbf_epn_out_isr(struct f_endpoint *ep, struct f_req *req)
{
	struct f_drv *chip = ep->chip;
	struct f_regs_ep *ep_reg = ep->reg;
	struct f_regs_epdma *dma = &ep->chip->regs->epdma[ep->id];

	u32 reqlen = readl(&ep_reg->len_dcnt) & D_EPN_LDATA;

	/* if a DMA is in progress, ignore this interrupt */
	if (readl(&dma->epndcr1) & D_SYS_EPN_REQEN)
		return;

	/* If it's not a short packet, we want to do a burst DMA so
	 * increase the request length to the max we want.
	 * Otherwise, a short packet tells us the total we will get
	 */
	if (reqlen == usb_endpoint_maxp(ep->desc)) {
		reqlen = req->req.length - req->req.actual;
		req->dma_non_burst_bytes = 0;
	} else {
		req->dma_non_burst_bytes = reqlen & ~3;
	}

	TRACERQ(req, "%s[%d][%3d] reqlen %d actual %d/%d\n",
		__func__, ep->id, req->seq,
		reqlen, req->req.actual, req->req.length);

	/* DMA it! */
	if (req->use_dma && reqlen >= chip->dma_threshold) {
		usbf_epn_dma(ep, req, reqlen);
		return;
	}

	/* If using PIO, we only receive one USB packet at a time */
	if (reqlen > usb_endpoint_maxp(ep->desc))
		reqlen = usb_endpoint_maxp(ep->desc);

	usbf_epn_pio_recv(ep, req, reqlen);
}

static void usbf_epn_interrupt(struct f_endpoint *ep)
{
	struct f_req *req;

	spin_lock_irq(&ep->lock);
	req = list_first_entry_or_null(&ep->queue, struct f_req, queue);
	spin_unlock_irq(&ep->lock);

	/* Ensure we have a request, and that we have work to do on it */
	if (!req || req->req.status == 0)
		return;

	/* Note: D_EPN_OUT_INT must be before D_EPN_OUT_END_INT.
	 * Since we get OUT interrupts during DMA, we can pick up both OUT and
	 * OUT_END interrupt at the same time. If we handle the OUT_END first
	 * (and disable DMA), the OUT will look like a spurious interrupt.
	 * By calling usbf_ep_process_queue() here, we make this EP return data
	 * to other parts of the kernel quicker. This seems to be important...
	 */
	if (ep->status & D_EPN_OUT_END_INT) {
		ep->status &= ~D_EPN_OUT_END_INT;
		usbf_epn_out_end_isr(ep, req);
	} else if (ep->status & D_EPN_OUT_INT) {
		ep->status &= ~D_EPN_OUT_INT;
		usbf_epn_out_isr(ep, req);
	} else if (ep->status & D_EPN_IN_INT) {
		/*
		 * EPN_IN_INT means we have finished tx so we just want to poke the
		 * queue to ensure any pending requests are processed
		 */
		ep->status &= ~D_EPN_IN_INT;
	}

	usbf_ep_process_queue(ep);
}

static const struct f_endpoint_drv usbf_epn_callbacks = {
	.enable = usbf_epn_enable,
	.disable = usbf_epn_disable,
	/* .set_maxpacket = usbf_epn_set_maxpacket, */
	.recv[USBF_PROCESS_PIO] = usbf_epn_recv,
	.send[USBF_PROCESS_PIO] = usbf_epn_send,
	.recv[USBF_PROCESS_DMA] = usbf_epn_recv,
	.interrupt = usbf_epn_interrupt,
	.reset = usbf_epn_reset,
};

int usbf_epn_init(struct f_endpoint *ep)
{
	struct f_regs_ep *ep_reg = &ep->chip->regs->ep[ep->id - 1];
	static const char * const et[] = {
		[USBF_EP_BULK] = "-bulk",
		[USBF_EP_INT] = "-int",
		[USBF_EP_ISO] = "-iso"
	};

	ep->reg = ep_reg;
	ep->type = (readl(&ep_reg->control) >> 24) & 3;
	ep->drv = &usbf_epn_callbacks;
	strcat(ep->name, et[ep->type]);

	return 0;
}
