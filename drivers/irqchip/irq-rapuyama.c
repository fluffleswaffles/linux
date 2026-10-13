// SPDX-License-Identifier: GPL-2.0
/*
 * Nokia RapuYama interrupt controller (N8/E6/E7/C7 application ASIC)
 *
 * No public documentation exists. The register layout was reversed from the
 * Symbian^3 ASSP extension (karap.dll) of the E6 RM-609 firmware: its IRQ/FIQ
 * dispatchers and its Interrupt::Bind/Enable/Disable/SetPriority code.
 *
 * The controller hands out the number of the highest pending line: reading
 * INTC_IRQ_NUM takes it (-1 = nothing pending), a read of INTC_IRQ_END once
 * the queue is empty finishes the IRQ. Lines are masked and unmasked by
 * writing their number, not a bit, so there are no read-modify-write races.
 */

#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/of_address.h>

#include <asm/exception.h>

#define INTC_ENABLE		0x04	/* 1 = controller on */
#define INTC_IRQ_NUM		0x08	/* read: take pending irq, ~0 = none */
#define INTC_IRQ_UNMASK		0x0c	/* write line number */
#define INTC_IRQ_MASK		0x10	/* write line number, ~0 = all */
#define INTC_CTRL1C		0x1c	/* firmware writes 0 at init */
#define INTC_CFG_SEL		0x20	/* write line number to configure */
#define INTC_CFG		0x24	/* priority of the selected line, 0..7 */
#define INTC_FIQ_NUM		0x48
#define INTC_FIQ_MASK		0x50
#define INTC_IRQ_END		0x80	/* read when the irq queue is empty */
#define INTC_CTRL84		0x84	/* firmware writes 5 at init */
#define INTC_IRQ_NEST		0x90
#define INTC_FIQ_NEST		0x9c

#define INTC_NR_IRQS		421	/* firmware checks n <= 0x1a4 */

/*
 * the gpio / peripheral secondary interrupt banks, in the clock-power block
 * (phys 0x20033400 + 0x40 * k). firmware FUN_800b8b18 masks all of them at
 * init (write ~0 to bank+0x14, drain bank+0x28 until -1), otherwise a source
 * symbian left enabled keeps asserting the main line -> irq storm the moment
 * interrupts are turned on (died exactly at local_irq_enable without this)
 */
#define INTC_SEC_STRIDE		0x40
#define INTC_SEC_DIS		0x14	/* write ~0 to disable every line in the bank */
#define INTC_SEC_NUM		0x28	/* read until -1 to drain */
#define INTC_SEC_BANKS		5
#define INTC_DEF_PRIO		2	/* what Interrupt::Bind uses */
#define INTC_NONE		(~0u)

#ifdef CONFIG_ARCH_RAPUYAMA
void rapuyama_checkpoint(int n);
#else
static inline void rapuyama_checkpoint(int n) { }
#endif

static void __iomem *intc_base;
static void __iomem *intc_sec;	/* secondary (gpio/periph) banks, optional */
static struct irq_domain *intc_domain;

static void rapuyama_intc_mask(struct irq_data *d)
{
	writel_relaxed(d->hwirq, intc_base + INTC_IRQ_MASK);
}

static void rapuyama_intc_unmask(struct irq_data *d)
{
	writel_relaxed(d->hwirq, intc_base + INTC_IRQ_UNMASK);
}

static struct irq_chip rapuyama_intc_chip = {
	.name		= "rapuyama-intc",
	.irq_mask	= rapuyama_intc_mask,
	.irq_unmask	= rapuyama_intc_unmask,
};

static int rapuyama_intc_alloc(struct irq_domain *d, unsigned int virq,
			       unsigned int nr_irqs, void *arg)
{
	struct irq_fwspec *fwspec = arg;
	irq_hw_number_t hw;
	unsigned int type, i;
	int ret;

	ret = irq_domain_translate_onecell(d, fwspec, &hw, &type);
	if (ret)
		return ret;

	for (i = 0; i < nr_irqs; i++) {
		writel_relaxed(hw + i, intc_base + INTC_CFG_SEL);
		writel_relaxed(INTC_DEF_PRIO, intc_base + INTC_CFG);
		irq_domain_set_info(d, virq + i, hw + i, &rapuyama_intc_chip,
				    NULL, handle_level_irq, NULL, NULL);
		irq_set_probe(virq + i);
	}
	return 0;
}

/* hierarchical so the gpio block can stack its pin irqs on top */
static const struct irq_domain_ops rapuyama_intc_ops = {
	.translate	= irq_domain_translate_onecell,
	.alloc		= rapuyama_intc_alloc,
	.free		= irq_domain_free_irqs_common,
};

static void __exception_irq_entry rapuyama_intc_handle_irq(struct pt_regs *regs)
{
	int guard = 2 * INTC_NR_IRQS;
	u32 hw;

	/* cp20: first entry into the intc handler = an interrupt was delivered */
	rapuyama_checkpoint(20);

	/*
	 * read the pending line, dispatch, read the eoi (+0x80). bounded so a
	 * line that keeps re-asserting (bad source ack, or a line with no linux
	 * handler) can't lock the cpu forever: after too many iterations we
	 * mask that line at the main bank and bail, which keeps the kernel
	 * alive and makes a stuck source obvious instead of a silent hang.
	 */
	while ((hw = readl_relaxed(intc_base + INTC_IRQ_NUM)) != INTC_NONE) {
		pr_info_once("rapuyama-intc: first irq hwirq=%u\n", hw);
		if (hw >= INTC_NR_IRQS ||
		    generic_handle_domain_irq(intc_domain, hw) || --guard <= 0) {
			writel_relaxed(hw, intc_base + INTC_IRQ_MASK);
			if (guard <= 0)
				break;
		}
		readl_relaxed(intc_base + INTC_IRQ_END);
	}
	readl_relaxed(intc_base + INTC_IRQ_END);
}

/* take whatever is still latched, bounded in case the block is dead */
static void rapuyama_intc_drain(unsigned int num_reg)
{
	int n = INTC_NR_IRQS;

	while (n-- && readl_relaxed(intc_base + num_reg) != INTC_NONE)
		;
}

static int __init rapuyama_intc_of_init(struct device_node *node,
					struct device_node *parent)
{
	pr_info("rapuyama-intc: probe, parent=%p\n", parent);
	intc_base = of_iomap(node, 0);
	if (!intc_base)
		return -ENOMEM;

	/* same order as the firmware init */
	writel_relaxed(1, intc_base + INTC_ENABLE);
	writel_relaxed(0, intc_base + INTC_CTRL1C);
	writel_relaxed(INTC_NONE, intc_base + INTC_IRQ_MASK);
	writel_relaxed(5, intc_base + INTC_CTRL84);
	rapuyama_intc_drain(INTC_IRQ_NUM);
	writel_relaxed(INTC_NONE, intc_base + INTC_FIQ_MASK);
	rapuyama_intc_drain(INTC_FIQ_NUM);

	/*
	 * mask + drain the secondary gpio/peripheral banks, else a source
	 * symbian left enabled storms the main line at local_irq_enable
	 */
	intc_sec = of_iomap(node, 1);
	if (intc_sec) {
		int b, n;

		for (b = 0; b < INTC_SEC_BANKS; b++) {
			void __iomem *bank = intc_sec + b * INTC_SEC_STRIDE;

			writel_relaxed(INTC_NONE, bank + INTC_SEC_DIS);
			n = INTC_NR_IRQS;
			while (n-- && readl_relaxed(bank + INTC_SEC_NUM) != INTC_NONE)
				;
		}
	}

	writel_relaxed(0, intc_base + INTC_IRQ_NEST);
	writel_relaxed(0, intc_base + INTC_FIQ_NEST);

	intc_domain = irq_domain_create_linear(of_fwnode_handle(node),
					       INTC_NR_IRQS,
					       &rapuyama_intc_ops, NULL);
	if (!intc_domain) {
		iounmap(intc_base);
		return -ENOMEM;
	}

	set_handle_irq(rapuyama_intc_handle_irq);
	pr_info("rapuyama-intc: domain=%p ready, handler installed\n", intc_domain);
	return 0;
}
IRQCHIP_DECLARE(rapuyama_intc, "nokia,rapuyama-intc", rapuyama_intc_of_init);