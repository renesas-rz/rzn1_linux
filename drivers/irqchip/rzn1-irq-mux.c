// SPDX-License-Identifier: GPL-2.0
/*
 * RZ/N1 GPIO Interrupt Multiplexer
 *
 * Copyright (C) 2018 Renesas Electronics Europe Limited
 *
 * On RZ/N1 devices, there are 3 Synopsys DesignWare GPIO blocks each configured
 * to have 32 interrupt outputs, so we have a total of 96 GPIO interrupts.
 * All of these are passed to the GPIO IRQ Muxer, which selects 8 of the GPIO
 * interrupts to pass onto the GIC.
 */

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>

#define MAX_NR_INPUT_IRQS	96
#define MAX_NR_OUTPUT_IRQS	8

/*
 * "interrupt-map" consists of 1 interrupt cell, 0 address cells, phandle to
 * interrupt parent, and parent interrupt specifier (3 cells for GIC), giving
 * a total of 5 cells.
 */
#define IMAP_LENGTH		5

struct irqmux_priv;
struct irqmux_one {
	unsigned int irq;
	unsigned int src_hwirq;
	struct irqmux_priv *priv;
};

struct irqmux_priv {
	struct device *dev;
	struct irq_domain *irq_domain;
	unsigned int nr_irqs;
	struct irqmux_one mux[MAX_NR_OUTPUT_IRQS];
};

static irqreturn_t irqmux_handler(int irq, void *data)
{
	struct irqmux_one *mux = data;
	struct irqmux_priv *priv = mux->priv;
	unsigned int virq;

	virq = irq_find_mapping(priv->irq_domain, mux->src_hwirq);

	generic_handle_irq(virq);

	return IRQ_HANDLED;
}

static int irqmux_domain_map(struct irq_domain *h, unsigned int irq,
			     irq_hw_number_t hwirq)
{
	irq_set_chip_data(irq, h->host_data);
	irq_set_chip_and_handler(irq, &dummy_irq_chip, handle_simple_irq);

	return 0;
}

static const struct irq_domain_ops irqmux_domain_ops = {
	.map = irqmux_domain_map,
};

static int irqmux_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct resource *res;
	u32 __iomem *regs;
	struct irqmux_priv *priv;
	unsigned int i;
	int nr_irqs;
	int ret;
	const __be32 *imap;
	int imaplen;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	platform_set_drvdata(pdev, priv);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(regs))
		return PTR_ERR(regs);

	nr_irqs = of_irq_count(np);
	if (nr_irqs < 0)
		return nr_irqs;

	if (nr_irqs > MAX_NR_OUTPUT_IRQS) {
		dev_err(dev, "too many output interrupts\n");
		return -ENOENT;
	}

	priv->nr_irqs = nr_irqs;

	/* Look for the interrupt-map */
	imap = of_get_property(np, "interrupt-map", &imaplen);
	if (!imap)
		return -ENOENT;
	imaplen /= IMAP_LENGTH * sizeof(__be32);

	/* Sometimes not all muxs are used */
	if (imaplen < priv->nr_irqs)
		priv->nr_irqs = imaplen;

	/* Create IRQ domain for the interrupts coming from the GPIO blocks */
	priv->irq_domain = irq_domain_add_linear(np, MAX_NR_INPUT_IRQS,
						 &irqmux_domain_ops, priv);
	if (!priv->irq_domain)
		return -ENOMEM;

	for (i = 0; i < MAX_NR_INPUT_IRQS; i++)
		irq_create_mapping(priv->irq_domain, i);

	for (i = 0; i < priv->nr_irqs; i++) {
		struct irqmux_one *mux = &priv->mux[i];

		ret = irq_of_parse_and_map(np, i);
		if (ret < 0) {
			ret = -ENOENT;
			goto err;
		}

		mux->irq = ret;
		mux->priv = priv;

		/*
		 * We need the first cell of the interrupt-map to configure
		 * the hardware.
		 */
		mux->src_hwirq = be32_to_cpu(*imap);
		imap += IMAP_LENGTH;

		dev_info(dev, "%u: %u mapped irq %u\n", i,  mux->src_hwirq,
			 mux->irq);

		ret = devm_request_irq(dev, mux->irq, irqmux_handler,
				       IRQF_SHARED | IRQF_NO_THREAD,
				       "irqmux", mux);
		if (ret < 0) {
			dev_err(dev, "failed to request IRQ: %d\n", ret);
			goto err;
		}

		/* Set up the hardware to pass the interrupt through */
		writel(mux->src_hwirq, &regs[i]);
	}

	dev_info(dev, "probed, %d gpio interrupts\n", priv->nr_irqs);

	return 0;

err:
	while (i--)
		irq_dispose_mapping(priv->mux[i].irq);
	irq_domain_remove(priv->irq_domain);

	return ret;
}

static int irqmux_remove(struct platform_device *pdev)
{
	struct irqmux_priv *priv = platform_get_drvdata(pdev);
	unsigned int i;

	for (i = 0; i < priv->nr_irqs; i++)
		irq_dispose_mapping(priv->mux[i].irq);
	irq_domain_remove(priv->irq_domain);

	return 0;
}

static const struct of_device_id irqmux_match[] = {
	{ .compatible = "renesas,rzn1-gpioirqmux", },
	{ /* sentinel */ },
};

MODULE_DEVICE_TABLE(of, irqmux_match);

static struct platform_driver irqmux_driver = {
	.driver = {
		.name = "gpio_irq_mux",
		.owner = THIS_MODULE,
		.of_match_table = irqmux_match,
	},
	.probe = irqmux_probe,
	.remove = irqmux_remove,
};

module_platform_driver(irqmux_driver);

MODULE_DESCRIPTION("Renesas RZ/N1 GPIO IRQ Multiplexer Driver");
MODULE_AUTHOR("Phil Edworthy <phil.edworthy@renesas.com>");
MODULE_LICENSE("GPL v2");
