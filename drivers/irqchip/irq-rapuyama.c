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
#include <linux/bitmap.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/jiffies.h>

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
 * init (write ~0 to bank+0x14, drain bank+0x3c until -1), otherwise a source
 * symbian left enabled keeps asserting the main line -> irq storm the moment
 * interrupts are turned on (died exactly at local_irq_enable without this)
 */
#define INTC_SEC_STRIDE		0x40
#define INTC_SEC_DIS		0x14	/* write ~0 to disable every line in the bank */
#define INTC_SEC_NUM		0x3c	/* read until -1 to drain */
#define INTC_SEC_BANKS		5
#define INTC_DEF_PRIO		2	/* what Interrupt::Bind uses */
#define INTC_NONE		(~0u)

/*
 * The modem (CMT) link IRQ. sisa_wrapper FUN_80104200 binds line 0x1e (30) to
 * the SISA ISR FUN_80104194; its status bit is 0x1000 at phys 0x20040280. On
 * this on-chip modem the APE's enable state for this line is (very likely) the
 * "APE awake/alive" signal the modem's ~200 s supervision watches: our init
 * masks every primary line, so line 30 goes disabled, the modem concludes the
 * APE is gone and resets the whole SoC after ~200 s (the random reset bug, and
 * why even a bare-metal spin with IRQs masked dies at ~200 s). So keep line 30
 * enabled, and when it fires ack it (we have no ISI stack to drain its ring)
 * without ever permanently masking it. Toggle with irq_rapuyama.modem_link=0.
 *
 * The modem interrupt has its own status/enable/ack registers at phys
 * 0x20040280/284/288 (karap FUN_800b828c/FUN_800b8234/FUN_800b8260): bit 0x1000
 * is line 0x1e. The real ISR acks the source by writing 0x1000 to the ACK reg
 * (0x288) - NOT just the INTC EOI. First hw test (2026-10-16) did only the INTC
 * EOI and the modem reset FASTER (~17 s): the source stayed asserted/unacked, so
 * the modem escalated. Now we write the proper modem ACK on every line-30 IRQ.
 */
#define INTC_MODEM_LINE		30
#define MODEM_REGS_PHYS		0x20040000u	/* one page covers 0x180..0x788 */
#define MODEM_STATUS		0x280	/* bit 0x1000 = line 0x1e pending */
#define MODEM_ENABLE		0x284	/* write 0x1000 to enable */
#define MODEM_ACK		0x288	/* write 0x1000 to ack/clear */
#define MODEM_BIT		0x1000
/*
 * Real modem message FIFOs, from the MMU section map at ROM 0x80002094 (the c6
 * window is NOT linear for these high pages): firmware virt 0xc6e00000 -> phys
 * 0x20500000 (RX, modem->APE), virt 0xc6f00000 -> phys 0x40100000 (TX, APE->
 * modem). Each is a byte-FIFO port at base + chan*0x1000 (ASIC send vt+0x48:
 * strb 0xc0, bytes, strb 0x00). The modem fills RX and raises line 0x1e; if the
 * APE never drains RX the modem concludes the link is dead and resets the SoC
 * (the random 50-210 s reset). So drain RX on every line-30 IRQ.
 */
/*
 * The modem message FIFOs (firmware virt 0xc6e00000->phys 0x20500000 RX,
 * 0xc6f00000->0x40100000 TX, from the MMU section map) are NOT reachable from
 * our Linux: a read of 0x20500000 EXTERNAL-ABORTs (that data domain needs the
 * modem clock/power our boot does not set up). The 0x20040280 control regs ARE
 * reachable. So we only ack the line-30 IRQ here; servicing the FIFO link needs
 * the modem-interface clock brought up first (future work).
 */
#define MODEM_RSTREASON		0x20040400u	/* bits[23:20] = last reset reason (rebootdriver) */
#define MODEM_TXMB		0x180	/* APE->modem mailbox word (modwatch) */
#define MODEM_RXMB		0x580	/* modem->APE mailbox word */
#define MODEM_STAT		0x300	/* modem status (0x300c = both lines) */
#define MODEM_DB0		0x780	/* doorbell 0 (toggles in live Symbian) */
#define MODEM_DB1		0x784	/* doorbell 1 */
static void __iomem *modem_regs;
/* 17.10: all modem pokes default off, the death was linux overwriting the modem
 * fw in dram (e6.dts memory node), these were guesses on the wrong regs */
static int modem_link = 0;
module_param(modem_link, int, 0444);
/*
 * Capture the modem's line-30 traffic. With the modem interface clock now
 * brought up by rapuyama_modem.c, the mailbox writes should stick and the RX
 * FIFO (which external-aborted while the clock was off) may be readable. This
 * logs the first N firings so we can see the exact request the modem makes -
 * the keepalive alone let the phone reach only ~213 s (the modem's max window),
 * i.e. the modem wants a specific response, not a generic ping.
 */
static int modem_capture = 0;
module_param(modem_capture, int, 0444);
static int cap_count;
/*
 * Mailbox handshake. modwatch showed live Symbian exchanging messages every
 * ~24 s through mailbox 0x20040180(TX)/0x580(RX) with doorbells 0x780/0x784,
 * configured by karap FUN_800b8dc8 as [0x184]=1 [0x188]=6. Bare Linux never
 * configures it, so the modem parks in sync (0x55555555) and resets at its max
 * window (~213 s, exactly what we saw). Enable the mailbox channel and clear the
 * doorbells each modem IRQ (the APE's half of the handshake). modem_hs=0 off.
 */
static int modem_hs = 0;
module_param(modem_hs, int, 0444);
#define MODEM_MB_TXCFG4		0x184	/* FUN_800b8dc8: =1 */
#define MODEM_MB_TXCFG8		0x188	/* FUN_800b8dc8: =6 */
#define MODEM_DB0_IDLE		0x00000e80
#define MODEM_DB1_IDLE		0x00000000

#ifdef CONFIG_ARCH_RAPUYAMA
__printf(1, 2) void rapuyama_note(const char *fmt, ...);	/* mach-rapuyama */
#else
static inline __printf(1, 2) void rapuyama_note(const char *fmt, ...) { }
#endif

/*
 * an out of range number from INTC_IRQ_NUM is treated as spurious (EOI and
 * leave) unless it keeps coming: masking every line on the first one also
 * stops the timer, the soc watchdog then resets the phone silently
 */
#define INTC_BAD_LIMIT		1000

static void __iomem *intc_base;
static unsigned int intc_bad_in_row;

/*
 * optional modem-firmware host (drivers/misc/rapuyama_isasw.c): when it is
 * hosting the ISASW guest it registers a hook here that we call for every
 * serviced line, so the guest's scheduler runs on modem activity too, not just
 * on its own timer. unset (and the whole thing off) by default.
 */
static void (*isasw_irq_hook)(int line);

void rapuyama_isasw_set_irq_hook(void (*fn)(int line));
void rapuyama_isasw_set_irq_hook(void (*fn)(int line))
{
	isasw_irq_hook = fn;
}

/*
 * guest (ISASW) device lines. the modem fw host registers the intc lines its own
 * isrs own; we then deliver them to the host (via isasw_irq_hook) and, crucially,
 * do NOT route them through the linux domain (they are unmapped there, which
 * would mask them permanently). instead we mask + defer-unmask like the modem
 * line, so the guest keeps being fed on every modem request and can service its
 * ISI link (otherwise the link is dead and the modem resets the ape ~200 s).
 */
static DECLARE_BITMAP(intc_guest_lines, INTC_NR_IRQS);
static DECLARE_BITMAP(intc_guest_pending, INTC_NR_IRQS);
static bool intc_have_guest_lines;

void rapuyama_isasw_set_guest_lines(const u16 *lines, unsigned int n);
void rapuyama_isasw_set_guest_lines(const u16 *lines, unsigned int n)
{
	unsigned int i;

	bitmap_zero(intc_guest_lines, INTC_NR_IRQS);
	for (i = 0; i < n; i++)
		if (lines[i] < INTC_NR_IRQS) {
			set_bit(lines[i], intc_guest_lines);
			writel(lines[i], intc_base + INTC_IRQ_UNMASK);
		}
	intc_have_guest_lines = n > 0;
}

/*
 * service the guest lines that fired, from the host's OWN (process/timer)
 * context - NOT from the hard-irq handler, where running the guest trampoline
 * (an mmu switch into symbian context) is unsafe. the isasw timer calls this;
 * for each pending guest line we run its isr via fn() then re-unmask it.
 */
void rapuyama_isasw_service_guest(void (*fn)(int line));
void rapuyama_isasw_service_guest(void (*fn)(int line))
{
	unsigned int line;

	if (!intc_have_guest_lines)
		return;
	for_each_set_bit(line, intc_guest_pending, INTC_NR_IRQS) {
		clear_bit(line, intc_guest_pending);
		if (fn)
			fn(line);
		writel(line, intc_base + INTC_IRQ_UNMASK);
	}
}
static void __iomem *intc_sec;	/* secondary (gpio/periph) banks, optional */
static u32 sec_svc[INTC_SEC_BANKS];	/* lines to keep serviced (modem IPC etc.) */
static struct delayed_work sec_work;
static bool sec_on;
static struct irq_domain *intc_domain;
static DECLARE_BITMAP(intc_deferred_unmask, INTC_NR_IRQS);
static bool intc_dispatching;

static void rapuyama_intc_mask(struct irq_data *d)
{
	clear_bit(d->hwirq, intc_deferred_unmask);
	writel_relaxed(d->hwirq, intc_base + INTC_IRQ_MASK);
}

static void rapuyama_intc_unmask(struct irq_data *d)
{
	if (intc_dispatching) {
		set_bit(d->hwirq, intc_deferred_unmask);
		return;
	}

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
	unsigned int line;
	bool disable_all = false;
	u32 hw;

	intc_dispatching = true;
	/*
	 * Keep lines masked until the pending queue has been drained and EOI
	 * written. The firmware's timer ISR follows this order; unmasking from
	 * handle_level_irq() before EOI can immediately queue the same level IRQ.
	 */
	while ((hw = readl_relaxed(intc_base + INTC_IRQ_NUM)) != INTC_NONE) {
		pr_info_once("rapuyama-intc: first irq hwirq=%u\n", hw);

		if (unlikely(hw >= INTC_NR_IRQS)) {
			if (intc_bad_in_row < 16)
				rapuyama_note("e6: intc invalid pending hwirq %#x (%u in a row)\n",
					      hw, intc_bad_in_row + 1);
			if (++intc_bad_in_row < INTC_BAD_LIMIT)
				break;
			rapuyama_note("e6: intc keeps reporting hwirq %#x, masking all IRQs\n",
				      hw);
			pr_err_once("rapuyama-intc: invalid pending hwirq=%u, masking all IRQs\n",
				    hw);
			writel_relaxed(INTC_NONE, intc_base + INTC_IRQ_MASK);
			bitmap_zero(intc_deferred_unmask, INTC_NR_IRQS);
			disable_all = true;
			break;
		}
		intc_bad_in_row = 0;

		/* let the modem-fw host run the guest for its own lines (no-op
		 * unless isasw_host=1; it filters to the guest line set itself) */
		if (isasw_irq_hook)
			isasw_irq_hook(hw);

		/*
		 * Count every valid dequeued IRQ, including unmapped lines.
		 * Otherwise a controller repeatedly reporting an unhandled line
		 * can keep this early boot path spinning forever.
		 */
		if (unlikely(--guard <= 0)) {
			clear_bit(hw, intc_deferred_unmask);
			writel_relaxed(hw, intc_base + INTC_IRQ_MASK);
			rapuyama_note("e6: intc queue did not drain, masked hwirq %u\n",
				      hw);
			pr_err_once("rapuyama-intc: pending IRQ queue did not drain\n");
			break;
		}

		/*
		 * guest-owned modem line: the host's isr already ran via the hook
		 * above and clears/acks its own source. keep it OUT of the linux
		 * domain (unmapped => would be permanently masked); mask + defer so
		 * it re-unmasks after EOI and keeps feeding the guest next time.
		 */
		if (intc_have_guest_lines && test_bit(hw, intc_guest_lines)) {
			set_bit(hw, intc_guest_pending);
			writel_relaxed(hw, intc_base + INTC_IRQ_MASK);
			continue;	/* serviced later from isasw timer ctx */
		}

		if (modem_link && hw == INTC_MODEM_LINE) {
			/*
			 * modem (SISA) link line. Ack the MODEM source the way the
			 * firmware ISR does (write 0x1000 to the ack reg 0x20040288)
			 * so the modem sees its interrupt acknowledged - the INTC
			 * EOI alone is not enough (that died in ~17 s). We have no
			 * ISI stack to drain the APE-side callback ring, but there
			 * is nothing in it here; the modem just needs the ack.
			 * Mask now to leave the inner loop, deferred bit re-unmasks
			 * after EOI => never permanently masked (that = APE dead).
			 */
			/*
			 * NOTE: draining the RX FIFO at phys 0x20500000 EXTERNAL-ABORTS
			 * from Linux (that data domain needs modem clock/power our boot
			 * does not bring up; the 0x20040280 control regs are reachable,
			 * the FIFO is not). So do NOT touch the FIFO - just ack both
			 * lines (0x1000, 0x2000 -> 0x20040288, = firmware FUN_800b8260).
			 */
			if (modem_capture && cap_count < 16 && modem_regs) {
				u32 is = readl_relaxed(modem_regs + MODEM_STATUS);
				u32 st = readl_relaxed(modem_regs + MODEM_STAT);
				u32 tx = readl_relaxed(modem_regs + MODEM_TXMB);
				u32 rx = readl_relaxed(modem_regs + MODEM_RXMB);
				u32 d0 = readl_relaxed(modem_regs + MODEM_DB0);
				u32 d1 = readl_relaxed(modem_regs + MODEM_DB1);

				rapuyama_note("e6: modem irq #%d is=%08x st=%08x tx=%08x rx=%08x db=%08x/%08x\n",
					      cap_count, is, st, tx, rx, d0, d1);
				cap_count++;
			}
			/* APE half of the mailbox handshake: clear the doorbells the
			 * modem set, so it sees the APE consuming its signal. */
			if (modem_hs && modem_regs) {
				writel_relaxed(MODEM_DB0_IDLE, modem_regs + MODEM_DB0);
				writel_relaxed(MODEM_DB1_IDLE, modem_regs + MODEM_DB1);
			}
			if (modem_regs) {
				writel_relaxed(0x1000, modem_regs + MODEM_ACK);
				writel_relaxed(0x2000, modem_regs + MODEM_ACK);
			}
			set_bit(hw, intc_deferred_unmask);
			writel_relaxed(hw, intc_base + INTC_IRQ_MASK);
			continue;
		}

		if (generic_handle_domain_irq(intc_domain, hw)) {
			clear_bit(hw, intc_deferred_unmask);
			writel_relaxed(hw, intc_base + INTC_IRQ_MASK);
			rapuyama_note("e6: intc unmapped hwirq %u, masked\n", hw);
		}
	}
	readl(intc_base + INTC_IRQ_END);

	intc_dispatching = false;
	if (disable_all)
		return;

	for_each_set_bit(line, intc_deferred_unmask, INTC_NR_IRQS) {
		clear_bit(line, intc_deferred_unmask);
		writel(line, intc_base + INTC_IRQ_UNMASK);
	}
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
			u32 en, raw;

			/*
			 * log what Symbian left ENABLED here before we mask it:
			 * +0x18 enabled mask, +0x08 raw state, +0x04 bank enable.
			 * one of these enabled lines is the modem/CMT inter-processor
			 * interrupt we must keep servicing (masking it = the modem
			 * resets the SoC after its supervision timeout, the ~210 s bug).
			 */
			en = readl_relaxed(bank + 0x18);
			raw = readl_relaxed(bank + 0x08);
			rapuyama_note("e6: intc sec bank %d: en=%08x raw=%08x bank_en=%08x\n",
				      b, en, raw, readl_relaxed(bank + 0x04));
			/* remember Symbian's enabled set, minus lines asserting right
			 * now (those are the continuous storm sources we must not
			 * re-enable). the rest - incl the periodic modem IPC - get
			 * serviced later once boot is past the storm window. */
			sec_svc[b] = en & ~raw;

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

	/*
	 * Keep the modem (SISA) link line enabled from the start so the modem's
	 * supervision sees the APE listening on its IRQ. We masked every primary
	 * line above; re-enable just this one. Its firings are acked in the
	 * dispatch loop without ever permanently masking it.
	 */
	if (modem_link) {
		modem_regs = ioremap(MODEM_REGS_PHYS, 0x1000);
		if (modem_regs) {
			/* make sure the modem IRQ source is enabled, ack any stale
			 * pending, then enable the INTC line (FUN_800b8234 + ack). */
			writel_relaxed(MODEM_BIT, modem_regs + MODEM_ENABLE);
			writel_relaxed(MODEM_BIT, modem_regs + MODEM_ACK);
			/* mailbox channel config (karap FUN_800b8dc8) is done by
			 * rapuyama_modem.c, which maps the clock region too. */
		}
		{
			void __iomem *rr = ioremap(MODEM_RSTREASON, 4);

			if (rr) {
				u32 v = readl_relaxed(rr);

				rapuyama_note("e6: reset-reason 0x20040400=%08x bits[23:20]=%x\n",
					      v, (v >> 20) & 0xf);
				iounmap(rr);
			}
		}
		writel_relaxed(INTC_MODEM_LINE, intc_base + INTC_IRQ_UNMASK);
		rapuyama_note("e6: modem link line %u kept enabled, regs=%p (modem_link=1)\n",
			      INTC_MODEM_LINE, modem_regs);
	}

	pr_info("rapuyama-intc: domain=%p ready, handler installed\n", intc_domain);
	return 0;
}
/*
 * secondary-bank servicing: the modem/CMT sends the APE an inter-processor
 * interrupt on one of these banks and expects it serviced; Symbian services it,
 * and if it stops the modem resets the whole SoC (the ~210 s bug). We disabled
 * all bank lines at init to kill a boot storm; here, once boot is well past the
 * storm window, we re-enable the lines Symbian had on (minus the ones that were
 * asserting at init = storm sources) and ACK them every 10 ms, which is the
 * "APE is alive and serviced your IPC" signal. ACK is a background poll so it
 * works even if the aggregate main line gets masked.
 */
#define SEC_SVC_DELAY_MS	3000
#define SEC_POLL_MS		10

static void rapuyama_sec_work_fn(struct work_struct *w)
{
	int b;

	if (!intc_sec)
		return;

	if (!sec_on) {
		for (b = 0; b < INTC_SEC_BANKS; b++)
			if (sec_svc[b])
				writel_relaxed(sec_svc[b],
					       intc_sec + b * INTC_SEC_STRIDE + 0x10);
		sec_on = true;
		rapuyama_note("e6: intc sec service armed %08x %08x %08x %08x %08x\n",
			      sec_svc[0], sec_svc[1], sec_svc[2], sec_svc[3], sec_svc[4]);
	}

	for (b = 0; b < INTC_SEC_BANKS; b++) {
		void __iomem *bank = intc_sec + b * INTC_SEC_STRIDE;
		u32 pend = readl_relaxed(bank + 0x08) & sec_svc[b];

		if (pend)
			writel_relaxed(pend, bank + 0x20);	/* ack */
	}

	schedule_delayed_work(&sec_work, msecs_to_jiffies(SEC_POLL_MS));
}

static int __init rapuyama_sec_service_start(void)
{
	if (!intc_sec)
		return 0;
	INIT_DELAYED_WORK(&sec_work, rapuyama_sec_work_fn);
	schedule_delayed_work(&sec_work, msecs_to_jiffies(SEC_SVC_DELAY_MS));
	return 0;
}
late_initcall(rapuyama_sec_service_start);

/*
 * Modem EM (Energy-Management) keepalive — the actual fix target.
 * The modem resets the APE when the APE stops sending it EM power-state
 * messages (resource 0xA9). Symbian's empmIsaIf.dll streams these on every
 * idle/wake; bare Linux never does -> modem resets after 50-210 s. Here we
 * send the modem the EM "awake" message periodically through its TX byte FIFO
 * (phys 0x40100000 + chan*0x1000, framed 0xc0 ... 0x00 - the ASIC send,
 * FUN_800beac8). Message body = empmIsaIf FUN_801012ec state 1 (awake):
 *   [3]=0x47 [9]=4 [0xa..0xf]=2,1,5,0xc,4,4 [0x10]=0 [0x14..0x17]=(0x111) ;
 *   ISI length at [4,5]=0x0012. Header bytes the IAD/phonet layer adds are not
 *   yet known, so this is a best-effort first attempt; its MAIN purpose is to
 *   answer the gating unknown: does writing the TX FIFO abort (modem iface
 *   clock) the way reading the RX FIFO (0x20500000) did? Toggle em_keepalive=0.
 */
/*
 * NOTE: the real EM keepalive now lives in drivers/misc/rapuyama_modem.c, which
 * also writes the send doorbell (0x401fd800) and brings up the modem-interface
 * clock - the two things this early sketch was missing, so it never actually
 * reached the modem. Default this one OFF to avoid sending a second, malformed
 * frame on the same FIFO channel. Re-enable with irq_rapuyama.em_keepalive=1
 * only to isolate the irqchip path. */
#define EM_FIFO_TX	0x40100000u
#define EM_PERIOD_MS	5000
static int em_keepalive;
module_param(em_keepalive, int, 0444);
static void __iomem *em_tx;
static struct timer_list em_timer;
static u32 em_counter = 0x111;		/* empmIsaIf uses a LIVE, advancing counter */
static u8 em_awake_msg[0x18] = {
	[3] = 0x47, [9] = 4,
	[0xa] = 2, [0xb] = 1, [0xc] = 5, [0xd] = 0xc, [0xe] = 4, [0xf] = 4,
	[4] = 0x00, [5] = 0x12,			/* ISI length = 0x18 - 6 */
};

static void em_keepalive_fn(struct timer_list *t)
{
	int i;

	if (em_tx) {
		/*
		 * Advance the counter every send - the modem likely watches it
		 * climb as proof the APE is live (empmIsaIf sources it from a
		 * live value). Stored big-endian at bytes 0x14..0x17.
		 */
		em_counter++;
		em_awake_msg[0x14] = (em_counter >> 24) & 0xff;
		em_awake_msg[0x15] = (em_counter >> 16) & 0xff;
		em_awake_msg[0x16] = (em_counter >> 8) & 0xff;
		em_awake_msg[0x17] = em_counter & 0xff;
		/* frame to channel 0: 0xc0, message bytes, 0x00 (ASIC send) */
		writeb_relaxed(0xc0, em_tx);
		for (i = 0; i < (int)sizeof(em_awake_msg); i++)
			writeb_relaxed(em_awake_msg[i], em_tx);
		writeb_relaxed(0x00, em_tx);
		if (em_counter < 0x111 + 6)
			rapuyama_note("e6: EM keepalive sent #%u\n", em_counter - 0x111);
	}
	mod_timer(&em_timer, jiffies + msecs_to_jiffies(EM_PERIOD_MS));
}

static int __init rapuyama_em_keepalive_start(void)
{
	if (!em_keepalive)
		return 0;
	em_tx = ioremap(EM_FIFO_TX, 0x1000);
	rapuyama_note("e6: EM keepalive arming, tx=%p (resource 0xA9, %d ms)\n",
		      em_tx, EM_PERIOD_MS);
	timer_setup(&em_timer, em_keepalive_fn, 0);
	/* fire the first one soon so we learn the TX-access answer early */
	mod_timer(&em_timer, jiffies + msecs_to_jiffies(3000));
	return 0;
}
late_initcall(rapuyama_em_keepalive_start);

IRQCHIP_DECLARE(rapuyama_intc, "nokia,rapuyama-intc", rapuyama_intc_of_init);