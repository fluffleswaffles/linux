// SPDX-License-Identifier: GPL-2.0
/*
 * Nokia RapuYama GPIO (N8/E6/E7/C7 application ASIC)
 *
 * No public documentation exists, reversed from egpio.dll of the E6 RM-609
 * firmware and from how other drivers drive their pins through it (the BT
 * uart reset/wake lines, OneNAND irq pin, usb and power resources).
 *
 * Every bank has 32 pins and works with "write a bit" registers, so no
 * read-modify-write is needed. Banks 0-4 are bidirectional, banks 5 and 6
 * only have the output part. Offsets from the block base (phys 0x20031900):
 *
 *   out bank k (0-5):  0x90 + 0x10 * k   read: output latch
 *                      0x94 + 0x10 * k   write: bits -> high
 *                      0x98 + 0x10 * k   write: bits -> low
 *   out bank 6:        0x1b0 / 0x1b4 / 0x1b8
 *   in bank k (0-4):   0xf0 + 0x10 * k   read: pin level
 *                      0xf4 + 0x10 * k   write: bits -> input
 *                      0xf8 + 0x10 * k   write: bits -> output
 *
 * The direction can't be read back, so it is shadowed.
 *
 * Pin interrupts: every irq capable pin has its own line on the main INTC
 * (table at egpio 0x800c1bfc, ry_irq_ranges below), the per bank enable /
 * ack / trigger regs live in a second block (phys 0x20033400, 0x40 per bank):
 *
 *   0x04 bank enable (1 while any pin is enabled)   0x08 raw state
 *   0x10 write bits -> enable                       0x14 write bits -> disable
 *   0x18 enabled mask                               0x20 write bits -> ack
 *   0x28 / 0x2c write bits -> edge / level          0x34 / 0x38 polarity
 *
 * Which of 0x34/0x38 is high/rising is a guess (egpio takes 1 -> 0x38,
 * 2 -> 0x34 and the callers don't tell), check it on hw.
 */

#include <linux/bitops.h>
#include <linux/gpio/driver.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/mod_devicetable.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#define RY_GPIO_BANKS		7
#define RY_GPIO_IN_BANKS	5
#define RY_GPIO_PER_BANK	32

#define RY_OUT(k)	((k) == 6 ? 0x1b0 : 0x90 + 0x10 * (k))
#define RY_OUT_SET	0x4
#define RY_OUT_CLR	0x8
#define RY_IN(k)	(0xf0 + 0x10 * (k))
#define RY_IN_DIR_IN	0x4
#define RY_IN_DIR_OUT	0x8

#define RY_IRQ_BANK(k)	(0x40 * (k))
#define RY_IRQ_BANK_EN	0x04
#define RY_IRQ_EN	0x10
#define RY_IRQ_DIS	0x14
#define RY_IRQ_ENABLED	0x18
#define RY_IRQ_ACK	0x20
#define RY_IRQ_EDGE	0x28
#define RY_IRQ_LEVEL	0x2c
#define RY_IRQ_POL_LO	0x34	/* guess */
#define RY_IRQ_POL_HI	0x38	/* guess */

/* gpio offset -> main intc line, from the egpio pin table */
static const struct {
	u16 first, last, irq;
} ry_irq_ranges[] = {
	{   0,  87, 316 },
	{  88, 109, 290 },
	{ 110, 120, 279 },
	{ 121, 122, 404 },
	{ 128, 128, 273 },
	{ 135, 135, 278 },
	{ 152, 152, 275 },
};

struct rapuyama_gpio {
	struct gpio_chip gc;
	void __iomem *base;
	void __iomem *irq_base;
	spinlock_t lock;
	u32 dir_out[RY_GPIO_BANKS];	/* shadow, 1 = output */
};

static int ry_bank(unsigned int off)
{
	return off / RY_GPIO_PER_BANK;
}

static u32 ry_bit(unsigned int off)
{
	return BIT(off % RY_GPIO_PER_BANK);
}

static int rapuyama_gpio_get_direction(struct gpio_chip *gc, unsigned int off)
{
	struct rapuyama_gpio *ry = gpiochip_get_data(gc);
	int k = ry_bank(off);

	if (k >= RY_GPIO_IN_BANKS)
		return GPIO_LINE_DIRECTION_OUT;
	return (ry->dir_out[k] & ry_bit(off)) ? GPIO_LINE_DIRECTION_OUT :
						GPIO_LINE_DIRECTION_IN;
}

static int rapuyama_gpio_get(struct gpio_chip *gc, unsigned int off)
{
	struct rapuyama_gpio *ry = gpiochip_get_data(gc);
	int k = ry_bank(off);

	if (k >= RY_GPIO_IN_BANKS || (ry->dir_out[k] & ry_bit(off)))
		return !!(readl(ry->base + RY_OUT(k)) & ry_bit(off));
	return !!(readl(ry->base + RY_IN(k)) & ry_bit(off));
}

static void rapuyama_gpio_set(struct gpio_chip *gc, unsigned int off, int val)
{
	struct rapuyama_gpio *ry = gpiochip_get_data(gc);
	int k = ry_bank(off);

	writel(ry_bit(off), ry->base + RY_OUT(k) +
	       (val ? RY_OUT_SET : RY_OUT_CLR));
}

static int rapuyama_gpio_direction_input(struct gpio_chip *gc,
					 unsigned int off)
{
	struct rapuyama_gpio *ry = gpiochip_get_data(gc);
	int k = ry_bank(off);
	unsigned long flags;

	if (k >= RY_GPIO_IN_BANKS)
		return -EINVAL;

	spin_lock_irqsave(&ry->lock, flags);
	writel(ry_bit(off), ry->base + RY_IN(k) + RY_IN_DIR_IN);
	ry->dir_out[k] &= ~ry_bit(off);
	spin_unlock_irqrestore(&ry->lock, flags);
	return 0;
}

static int rapuyama_gpio_direction_output(struct gpio_chip *gc,
					  unsigned int off, int val)
{
	struct rapuyama_gpio *ry = gpiochip_get_data(gc);
	int k = ry_bank(off);
	unsigned long flags;

	/* level first, like the firmware drivers do */
	rapuyama_gpio_set(gc, off, val);
	if (k >= RY_GPIO_IN_BANKS)
		return 0;

	spin_lock_irqsave(&ry->lock, flags);
	writel(ry_bit(off), ry->base + RY_IN(k) + RY_IN_DIR_OUT);
	ry->dir_out[k] |= ry_bit(off);
	spin_unlock_irqrestore(&ry->lock, flags);
	return 0;
}

static void __iomem *ry_irq_reg(struct rapuyama_gpio *ry, irq_hw_number_t hw,
				unsigned int reg)
{
	return ry->irq_base + RY_IRQ_BANK(ry_bank(hw)) + reg;
}

static void rapuyama_gpio_irq_mask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct rapuyama_gpio *ry = gpiochip_get_data(gc);
	irq_hw_number_t hw = irqd_to_hwirq(d);
	unsigned long flags;

	spin_lock_irqsave(&ry->lock, flags);
	writel(ry_bit(hw), ry_irq_reg(ry, hw, RY_IRQ_DIS));
	/* firmware drops the bank enable once nothing is left on */
	if (!readl(ry_irq_reg(ry, hw, RY_IRQ_ENABLED)))
		writel(0, ry_irq_reg(ry, hw, RY_IRQ_BANK_EN));
	spin_unlock_irqrestore(&ry->lock, flags);

	irq_chip_mask_parent(d);
	gpiochip_disable_irq(gc, hw);
}

static void rapuyama_gpio_irq_unmask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct rapuyama_gpio *ry = gpiochip_get_data(gc);
	irq_hw_number_t hw = irqd_to_hwirq(d);
	unsigned long flags;

	gpiochip_enable_irq(gc, hw);
	spin_lock_irqsave(&ry->lock, flags);
	writel(1, ry_irq_reg(ry, hw, RY_IRQ_BANK_EN));
	writel(ry_bit(hw), ry_irq_reg(ry, hw, RY_IRQ_EN));
	spin_unlock_irqrestore(&ry->lock, flags);
	irq_chip_unmask_parent(d);
}

static void rapuyama_gpio_irq_ack(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct rapuyama_gpio *ry = gpiochip_get_data(gc);
	irq_hw_number_t hw = irqd_to_hwirq(d);

	writel(ry_bit(hw), ry_irq_reg(ry, hw, RY_IRQ_ACK));
}

static int rapuyama_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct rapuyama_gpio *ry = gpiochip_get_data(gc);
	irq_hw_number_t hw = irqd_to_hwirq(d);
	unsigned int trig, pol;
	unsigned long flags;

	switch (type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_EDGE_RISING:
		trig = RY_IRQ_EDGE;
		pol = RY_IRQ_POL_HI;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		trig = RY_IRQ_EDGE;
		pol = RY_IRQ_POL_LO;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		trig = RY_IRQ_LEVEL;
		pol = RY_IRQ_POL_HI;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		trig = RY_IRQ_LEVEL;
		pol = RY_IRQ_POL_LO;
		break;
	default:
		return -EINVAL;
	}

	spin_lock_irqsave(&ry->lock, flags);
	writel(ry_bit(hw), ry_irq_reg(ry, hw, trig));
	writel(ry_bit(hw), ry_irq_reg(ry, hw, pol));
	spin_unlock_irqrestore(&ry->lock, flags);

	if (trig == RY_IRQ_EDGE)
		irq_set_handler_locked(d, handle_edge_irq);
	else
		irq_set_handler_locked(d, handle_level_irq);
	return 0;
}

static const struct irq_chip rapuyama_gpio_irq_chip = {
	.name			= "rapuyama-gpio",
	.irq_mask		= rapuyama_gpio_irq_mask,
	.irq_unmask		= rapuyama_gpio_irq_unmask,
	.irq_ack		= rapuyama_gpio_irq_ack,
	.irq_set_type		= rapuyama_gpio_irq_set_type,
	.flags			= IRQCHIP_IMMUTABLE,
	GPIOCHIP_IRQ_RESOURCE_HELPERS,
};

static int rapuyama_gpio_child_to_parent(struct gpio_chip *gc,
					 unsigned int child,
					 unsigned int child_type,
					 unsigned int *parent,
					 unsigned int *parent_type)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ry_irq_ranges); i++) {
		if (child >= ry_irq_ranges[i].first &&
		    child <= ry_irq_ranges[i].last) {
			*parent = ry_irq_ranges[i].irq +
				  child - ry_irq_ranges[i].first;
			*parent_type = IRQ_TYPE_NONE; /* intc is level, no types */
			return 0;
		}
	}
	return -EINVAL;
}

/* the intc takes a single cell */
static int rapuyama_gpio_parent_fwspec(struct gpio_chip *gc,
				       union gpio_irq_fwspec *gfwspec,
				       unsigned int parent_hwirq,
				       unsigned int parent_type)
{
	struct irq_fwspec *fwspec = &gfwspec->fwspec;

	fwspec->fwnode = gc->irq.parent_domain->fwnode;
	fwspec->param_count = 1;
	fwspec->param[0] = parent_hwirq;
	return 0;
}

static bool rapuyama_gpio_valid_irq(struct gpio_chip *gc, unsigned int off)
{
	unsigned int parent, type;

	return !rapuyama_gpio_child_to_parent(gc, off, 0, &parent, &type);
}

static void rapuyama_gpio_init_valid_mask(struct gpio_chip *gc,
					  unsigned long *valid_mask,
					  unsigned int ngpios)
{
	unsigned int i;

	for (i = 0; i < ngpios; i++)
		if (!rapuyama_gpio_valid_irq(gc, i))
			clear_bit(i, valid_mask);
}

static int rapuyama_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *parent_np;
	struct irq_domain *parent;
	struct rapuyama_gpio *ry;
	struct gpio_irq_chip *girq;

	ry = devm_kzalloc(dev, sizeof(*ry), GFP_KERNEL);
	if (!ry)
		return -ENOMEM;

	ry->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ry->base))
		return PTR_ERR(ry->base);

	spin_lock_init(&ry->lock);

	ry->gc.label = dev_name(dev);
	ry->gc.parent = dev;
	ry->gc.owner = THIS_MODULE;
	ry->gc.base = -1;
	ry->gc.ngpio = RY_GPIO_BANKS * RY_GPIO_PER_BANK;
	ry->gc.get_direction = rapuyama_gpio_get_direction;
	ry->gc.direction_input = rapuyama_gpio_direction_input;
	ry->gc.direction_output = rapuyama_gpio_direction_output;
	ry->gc.get = rapuyama_gpio_get;
	ry->gc.set = rapuyama_gpio_set;

	/* irqs are optional, the pins work without the second reg block */
	ry->irq_base = devm_platform_ioremap_resource(pdev, 1);
	parent_np = of_irq_find_parent(dev->of_node);
	if (!IS_ERR(ry->irq_base) && parent_np) {
		parent = irq_find_host(parent_np);
		of_node_put(parent_np);
		if (!parent)
			return -EPROBE_DEFER;

		girq = &ry->gc.irq;
		gpio_irq_chip_set_chip(girq, &rapuyama_gpio_irq_chip);
		girq->fwnode = dev_fwnode(dev);
		girq->parent_domain = parent;
		girq->child_to_parent_hwirq = rapuyama_gpio_child_to_parent;
		girq->populate_parent_alloc_arg = rapuyama_gpio_parent_fwspec;
		girq->init_valid_mask = rapuyama_gpio_init_valid_mask;
		girq->handler = handle_bad_irq;
		girq->default_type = IRQ_TYPE_NONE;
	} else {
		of_node_put(parent_np);
	}

	return devm_gpiochip_add_data(dev, &ry->gc, ry);
}

static const struct of_device_id rapuyama_gpio_of_match[] = {
	{ .compatible = "nokia,rapuyama-gpio" },
	{ }
};
MODULE_DEVICE_TABLE(of, rapuyama_gpio_of_match);

static struct platform_driver rapuyama_gpio_driver = {
	.probe = rapuyama_gpio_probe,
	.driver = {
		.name = "rapuyama-gpio",
		.of_match_table = rapuyama_gpio_of_match,
	},
};
builtin_platform_driver(rapuyama_gpio_driver);

MODULE_DESCRIPTION("Nokia RapuYama GPIO driver");
MODULE_LICENSE("GPL");