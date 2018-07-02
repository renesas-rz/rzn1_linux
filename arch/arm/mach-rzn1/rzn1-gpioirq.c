/*
 * RZ/N1 GPIO IRQ Muxer interface
 *
 * Copyright (C) 2017 Renesas Electronics Europe Limited
 *
 * On RZ/N1 devices, there are 3 Synopsys DesignWare GPIO blocks each configured
 * to have 32 interrupt outputs, so we have a total of 96 GPIO interrupts.
 * All of these are passed to the GPIO IRQ Muxer, which maps 8 of the GPIO
 * interrupts to GIC interrupts.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/pinctrl-rzn1.h>
#include <linux/irqchip/chained_irq.h>

#define DRIVER_DESC     "Renesas RZ/N1 GPIO IRQ Mux Driver"
#define DRIVER_AUTHOR   "Phil Edworthy <phil.edworthy@renesas.com>"
#define DRIVER_VERSION  "0.1"

#define GIRQ_NR_GIC_IRQS	8
#define GIRQ_IRQ_COUNT		96

struct girq_priv {
	struct device *dev;
	struct irq_chip irq_chip;
	struct irq_domain *irq_domain;
	int gic_irq[GIRQ_NR_GIC_IRQS];
	int gpio_irq[GIRQ_NR_GIC_IRQS];
};

static void girq_enable(struct irq_data *d)
{
	/*
	 * Either irq_chip->irq_enable must call something or you have to
	 * implement irq_chip->irq_setup.
	 */
}

static void girq_handler(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct girq_priv *p = irq_desc_get_handler_data(desc);
	int irq = irq_desc_get_irq(desc);
	int i;

	chained_irq_enter(chip, desc);

	for (i = 0; i < GIRQ_NR_GIC_IRQS; i++) {
		if (irq == p->gic_irq[i]) {
			generic_handle_irq(irq_find_mapping(p->irq_domain,
					   p->gpio_irq[i]));
			break;
		}
	}
	if (i == GIRQ_NR_GIC_IRQS)
		dev_err(p->dev, "got unused interrupt %d\n", irq);

	chained_irq_exit(chip, desc);
}

static int girq_domain_map(struct irq_domain *h, unsigned int irq,
				irq_hw_number_t hwirq)
{
	struct girq_priv *p = h->host_data;

	irq_set_chip_data(irq, h->host_data);
	irq_set_chip_and_handler(irq, &p->irq_chip, handle_simple_irq);

	return 0;
}

static const struct irq_domain_ops girq_domain_ops = {
	.map	= girq_domain_map,
};

static int girq_probe(struct platform_device *pdev)
{
	struct girq_priv *p;
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	struct irq_chip *irq_chip;
	const char *name = dev_name(&pdev->dev);
	int i;

	p = devm_kzalloc(dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;
	p->dev = dev;

	/* Get the static GIC interrupts */
	for (i = 0; i < GIRQ_NR_GIC_IRQS; i++) {
		int irq = irq_of_parse_and_map(dev->of_node, i);
		if (!irq) {
			dev_err(dev, "cannot get interrupt\n");
			return -ENOENT;
		}

		irq_set_chained_handler_and_data(irq, girq_handler, p);
		p->gic_irq[i] = irq;
	}

	/* Create IRQ domain for the interrupts coming from the GPIO blocks */
	irq_chip = &p->irq_chip;
	irq_chip->name = name;
	irq_chip->irq_enable = girq_enable;

	p->irq_domain = irq_domain_add_linear(np, GIRQ_IRQ_COUNT,
					      &girq_domain_ops, p);
	if (!p->irq_domain) {
		dev_err(&pdev->dev, "cannot initialize irq domain\n");
		return -ENXIO;
	}

	/* Map the GPIO interrupts to the GIC interrupts */
	for (i = 0; i < GIRQ_NR_GIC_IRQS; i++) {
		char prop[16];

		sprintf(prop, "gpioirq-%c", '0' + i);
		if (!of_property_read_s32(np, prop, &p->gpio_irq[i])) {
			rzn1_pinctrl_gpioint_select(i, p->gpio_irq[i]);
			dev_dbg(p->dev, "GIC IRQ %d mapped to GPIO IRQ %d\n",
				p->gic_irq[i], p->gpio_irq[i]);
		}
	}

	platform_set_drvdata(pdev, p);

	dev_info(&pdev->dev, "probed\n");

	return 0;
}

static int girq_remove(struct platform_device *pdev)
{
	struct girq_priv *p = platform_get_drvdata(pdev);

	irq_domain_remove(p->irq_domain);
	return 0;
}

static const struct of_device_id girq_match[] = {
	{ .compatible = "renesas,rzn1-gpioirq", },
	{},
};

MODULE_DEVICE_TABLE(of, girq_match);

static struct platform_driver girq_driver = {
	.driver = {
		.name = "gpio_irq_mux",
		.owner = THIS_MODULE,
		.of_match_table = girq_match,
	},
	.probe          = girq_probe,
	.remove         = girq_remove,
};

module_platform_driver(girq_driver);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_LICENSE("GPL");
