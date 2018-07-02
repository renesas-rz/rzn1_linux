// SPDX-License-Identifier: GPL-2.0
/*
 * RZ/N1 clock-event
 *
 * Copyright 2018 Renesas Electronics Europe Ltd.
 * Author: Phil Edworthy <phil.edworthy@renesas.com>
 *
 * Based on timer-keystone.c
 * Copyright 2013 Texas Instruments, Inc.
 * Author: Ivan Khoronzhuk <ivan.khoronzhuk@ti.com>
 *
 * The RZ/N1 Timer block is pretty simple. It has 8 count-up timers, each with
 * their own set of registers. After these registers, there are some registers
 * common to all of the timers, but we don't need them as they provide a way to
 * get the status of all the timers and clear all of the interrupts.
 *
 * Since we want to use these as hrtimers, we register a per-cpu clockevent
 * timer. We use timer 6 & 7 for this as these are 32-bit timers whereas the
 * others are only 16-bit. Obviously, this driver can only be used on devices
 * with a maximum of 2 CPUs.
 * We use one of the 16-bit timers as the sched clock and clocksource, though
 * there is little point as this timer is only used on ARM devices that already
 * provide these. At 25MHz, it's not even wide enough to handle a jiffy.
 */

#define pr_fmt(fmt)	"rzn1-timer: " fmt

#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/cpuhotplug.h>
#include <linux/interrupt.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/sched_clock.h>
#include <linux/slab.h>

#define MASK(nr)		(((nr) == 32) ? 0xffffffffUL : BIT(nr) - 1)

#define TIMER_NAME		"rzn1-timer"

/* Per-timer register offsets */
#define LOAD_COUNT		0x00
#define CURRENT_COUNT		0x04
#define CONTROL			0x08
#define  CTRL_INT_EN		BIT(3)
#define  CTRL_EN		BIT(2)
#define  CTRL_RELOAD		BIT(1)
#define  CTRL_DIV25		BIT(0)
#define CLEAR_INT		0x0c	/* Clear Interrupt */
#define STATUS_INT0		0x10	/* Interrupt Status before mask */
#define STATUS_INT1		0x14	/* Interrupt Status after mask */

#define TIMER_REG_SIZE		0x20
#define NR_TIMERS		8
#define NR_16_BIT_TIMERS	6
#define CLK_PRESCALER		25

#define ONESHOT			1
#define PERIODIC		0

/**
 * struct to hold timer data for one source
 * @base: timer memory base address
 * @index: sub-timer source index
 * @width: width in bits
 * @rate: clock rate
 * @hz_period: cycles per HZ period
 * @ce: event device based on timer
 */
struct rzn1_timer {
	void __iomem *base;
	unsigned int index;
	unsigned int width;
	unsigned long rate;
	unsigned long hz_period;
	struct clock_event_device ce;
};

static u32 rzn1_timer_readl(struct rzn1_timer *timer, unsigned long rg)
{
	return readl_relaxed(timer->base + rg);
}

static void rzn1_timer_writel(
	struct rzn1_timer *timer, u32 val, unsigned long rg)
{
	writel_relaxed(val, timer->base + rg);
}

/**
 * rzn1_timer_barrier: write memory barrier
 * use explicit barrier to avoid using readl/writel non relaxed function
 * variants, because in our case non relaxed variants hide the true places
 * where barrier is needed.
 */
static void rzn1_timer_barrier(struct rzn1_timer *timer)
{
	__iowmb();
}

/**
 * rzn1_timer_config: configures timer to work in oneshot/periodic modes.
 * @period: cycles number to configure for
 * @mode: ONESHOT or PERIODIC
 */
static int rzn1_timer_config(struct rzn1_timer *timer, u32 period, int mode)
{
	u32 ctrl;
	u32 off;

	ctrl = rzn1_timer_readl(timer, CONTROL);
	off = ctrl & ~CTRL_EN;

	/* set enable mode */
	if (mode == ONESHOT) {
		ctrl &= ~CTRL_RELOAD;
		ctrl |= CTRL_EN;
	} else {
		/* periodic */
		ctrl |= CTRL_RELOAD;
		ctrl |= CTRL_EN;
	}

	/* disable timer */
	rzn1_timer_writel(timer, off, CONTROL);
	/* here we have to be sure the timer has been disabled */
	rzn1_timer_barrier(timer);

	/* reset counter to zero, set new period */
	rzn1_timer_writel(timer, 0, CURRENT_COUNT);
	rzn1_timer_writel(timer, period, LOAD_COUNT);

	/*
	 * enable timer
	 * here we have to be sure that registers have been written.
	 */
	rzn1_timer_barrier(timer);
	rzn1_timer_writel(timer, ctrl, CONTROL);

	return 0;
}

static void rzn1_timer_disable(struct rzn1_timer *timer)
{
	u32 ctrl;

	ctrl = rzn1_timer_readl(timer, CONTROL);
	ctrl &= ~CTRL_EN;
	rzn1_timer_writel(timer, ctrl, CONTROL);
}

static irqreturn_t rzn1_timer_interrupt(int irq, void *dev_id)
{
	struct rzn1_timer *timer = dev_id;

	/* ack the interrupt */
	rzn1_timer_readl(timer, CLEAR_INT);
	rzn1_timer_barrier(timer);

	timer->ce.event_handler(&timer->ce);
	return IRQ_HANDLED;
}

static int rzn1_clkevt_set_next_event(unsigned long cycles,
				  struct clock_event_device *evt)
{
	struct rzn1_timer *timer = container_of(evt, struct rzn1_timer, ce);

	return rzn1_timer_config(timer, cycles, ONESHOT);
}

static int rzn1_clkevt_shutdown(struct clock_event_device *evt)
{
	struct rzn1_timer *timer = container_of(evt, struct rzn1_timer, ce);

	rzn1_timer_disable(timer);
	return 0;
}

static int rzn1_clkevt_set_periodic(struct clock_event_device *evt)
{
	struct rzn1_timer *timer = container_of(evt, struct rzn1_timer, ce);

	rzn1_timer_config(timer, timer->hz_period, PERIODIC);
	return 0;
}

/* sched timer */
static struct rzn1_timer *rzn1_sched_timer;
static u64 notrace rzn1_sched_read(void)
{
	return rzn1_timer_readl(rzn1_sched_timer, CURRENT_COUNT);
}

static cycle_t rzn1_clksrc_read(struct clocksource *cs)
{
	return rzn1_timer_readl(rzn1_sched_timer, CURRENT_COUNT);
}

static struct clocksource rzn1_clocksource = {
	.name = "rzn1_timer_clocksource",
	.rating = 200,
	.flags = CLOCK_SOURCE_IS_CONTINUOUS | CLOCK_SOURCE_SUSPEND_NONSTOP,
	.read = rzn1_clksrc_read,
};

/* per-cpu clockevents */
static struct rzn1_timer *g_timers;
static int rzn1_local_timer_starting_cpu(unsigned int cpu)
{
	struct rzn1_timer *timer = &g_timers[cpu + NR_16_BIT_TIMERS];

	timer->ce.cpumask = cpumask_of(cpu);
	irq_force_affinity(timer->ce.irq, cpumask_of(cpu));
	clockevents_config_and_register(&timer->ce, timer->rate, 1, MASK(timer->width));

	return 0;
}

static int rzn1_local_timer_dying_cpu(unsigned int cpu)
{
	return 0;
}

static int __init rzn1_timer_init(struct device_node *np)
{
	struct rzn1_timer *timers;
	unsigned long base_rate;
	struct clk *clk;
	u32 timer_block_nr = 0;
	int error;
	void *base;
	int timer_count = 0, i;

	/* optional property, allows for naming timers as timer0, timer1 */
	of_property_read_u32(np, "renesas,timer-number", &timer_block_nr);

	timers = kcalloc(NR_TIMERS, sizeof(struct rzn1_timer), GFP_KERNEL);
	if (!timers)
		return -ENOMEM;
	/* May have multiple timer blocks, avoid clash */
	if (!g_timers)
		g_timers = timers;

	base = of_iomap(np, 0);
	if (!base) {
		pr_err("Can't remap registers\n");
		return -ENXIO;
	}

	clk = of_clk_get(np, 0);
	if (IS_ERR(clk)) {
		pr_err("Can't get clock\n");
		iounmap(base);
		return PTR_ERR(clk);
	}

	error = clk_prepare_enable(clk);
	if (error) {
		pr_err("Can't enable clock\n");
		goto err;
	}

	base_rate = clk_get_rate(clk);

	pr_info("base clock is %lu Hz\n", base_rate);

	for (i = 0; i < NR_TIMERS; i++, timer_count++) {
		struct rzn1_timer *timer = &timers[i];
		u32 prescale = 0;
		u32 rating = 150;

		/* optional, per-timer overrride property for the prescaler */
		of_property_read_u32_index(np, "renesas,clk-prescaler",
					i, &prescale);

		/* optional, per-timer overrride property for the rating */
		of_property_read_u32_index(np, "renesas,timer-rating",
					i, &rating);

		timer->index = i;
		timer->base = base + (i * TIMER_REG_SIZE);
		if (i < NR_16_BIT_TIMERS)
			timer->width = 16;
		else
			timer->width = 32;
		if (prescale)
			timer->rate = base_rate / CLK_PRESCALER;
		else
			timer->rate = base_rate;
		timer->hz_period = DIV_ROUND_UP(timer->rate, HZ);

		pr_debug("%u clock %lu\n", timer->index, timer->rate);

		/* disable and init */
		rzn1_timer_writel(timer, 0, CONTROL);
		rzn1_timer_barrier(timer);
		rzn1_timer_writel(timer, 0, CURRENT_COUNT);

		/* scheduler clock and clocksource */
		if (i == 0) {
			/* May have multiple timer blocks, avoid clash */
			if (rzn1_sched_timer)
				continue;

			/* Set the prescaler, start the timer, no interrupts */
			if (prescale)
				rzn1_timer_writel(timer, CTRL_DIV25, CONTROL);
			rzn1_timer_config(timer, MASK(timer->width), PERIODIC);

			/* scheduler clock */
			rzn1_sched_timer = timer;
			sched_clock_register(rzn1_sched_read, timer->width, timer->rate);

			/* clocksource */
			rzn1_clocksource.mask = CLOCKSOURCE_MASK(timer->width);
			clocksource_register_hz(&rzn1_clocksource, timer->rate);
			continue;
		}

		/* clockevents */
		timer->ce.features = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT;
		timer->ce.set_next_event = rzn1_clkevt_set_next_event;
		timer->ce.set_state_shutdown = rzn1_clkevt_shutdown;
		timer->ce.set_state_periodic = rzn1_clkevt_set_periodic;
		timer->ce.cpumask = cpu_possible_mask;
		timer->ce.rating = rating;
		timer->ce.owner = THIS_MODULE;
		timer->ce.name = kasprintf(GFP_KERNEL, TIMER_NAME "%d:%d",
						timer_block_nr, i);
		timer->ce.irq  = irq_of_parse_and_map(np, i);
		if (timer->ce.irq <= 0) {
			pr_err("failed to map interrupts\n");
			error = -EINVAL;
			goto err_irq;
		}

		error = request_irq(timer->ce.irq, rzn1_timer_interrupt,
					IRQF_TIMER, timer->ce.name, timer);
		if (error) {
			pr_err("failed to setup irq\n");
			goto err_irq;
		}

		/* enable timer interrupts and set the prescaler*/
		if (prescale)
			rzn1_timer_writel(timer, CTRL_INT_EN | CTRL_DIV25, CONTROL);
		else
			rzn1_timer_writel(timer, CTRL_INT_EN, CONTROL);

		/* Reg 16-bit timers, the others will be done per-cpu hotplug*/
		if (i < NR_16_BIT_TIMERS)
			clockevents_config_and_register(&timer->ce, timer->rate,
					1, MASK(timer->width));
	}

	/* May have multiple timer blocks, avoid clash */
	if (!g_timers) {
		/* Install and invoke hotplug callbacks */
		return cpuhp_setup_state(CPUHP_AP_RZN1_GIC_TIMER_STARTING,
					 "AP_RZN1_TIMER_STARTING",
					 rzn1_local_timer_starting_cpu,
					 rzn1_local_timer_dying_cpu);
	}
	return 0;

err_irq:
	for (i = 1; i < timer_count; i++)
		irq_dispose_mapping(timers[i].ce.irq);
err:
	clk_put(clk);
	iounmap(base);
	kfree(timers);
	return error;
}

CLOCKSOURCE_OF_DECLARE(rzn1_timer, "renesas,rzn1-timer", rzn1_timer_init);
