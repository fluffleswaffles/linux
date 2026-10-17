// SPDX-License-Identifier: GPL-2.0
/*
 * rapuyama reset theories
 */

#include <linux/io.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>

#ifdef CONFIG_ARCH_RAPUYAMA
void rapuyama_note(const char *fmt, ...);
#else
static inline void rapuyama_note(const char *fmt, ...) { }
#endif

/*
 * bit0 1Hz fc/mst/soc sampler (safe)
 * bit1 also sample the 0x30000000 block (external-abort risk)
 * bit2 "ping catcher": poll the INTC secondary-bank raw regs at 100 ms and log
 *      any bit that changes. If the external supervisor asks the APE a question
 *      via an interrupt that Linux leaves masked, its raw bit rises a few ms-s
 *      before the reset - that is the ping. (safe: 0x20033400 is always clocked)
 */
static unsigned int probe = 0x5;
module_param(probe, uint, 0644);
MODULE_PARM_DESC(probe, "bit0 1Hz sampler, bit1 c30 block (abort risk), bit2 INTC-raw ping catcher");

/*
 * if set, log the survival predicted by the phase hypothesis so the run
 * self-checks: death is the first supervisor check on a free-running ~P grid
 * after the handoff, so the two measured deaths are k*P apart in fc ticks
 * (baseline 0x050eac9f, neutral 0x072c71b2 -> dP=1083.5 s, P~216.7 s at k=5).
 * A hit within +-3 s of pexp is strong confirmation; a miss falsifies that P.
 */
static unsigned int pexp;
module_param(pexp, uint, 0644);
MODULE_PARM_DESC(pexp, "predicted survival seconds (phase hypothesis self-check), 0 = off");

static unsigned int period = 1;		/* sampler period, seconds */
module_param(period, uint, 0644);

/* fast counter 0x20033350 (timer-rapuyama.c): write latch, read value. 32768 Hz */
#define FC_PHYS		0x20033350u
#define FC_LATCH_VAL	0xc6033354u	/* same literal ekern uses */
#define MST_COUNT_PHYS	0x200337a4u	/* ms down-counter, readable (timerprobe) */
#define SOC_PHYS	0x200336c4u	/* SoC watchdog counter (reads 0x55555555) */
#define C30_PHYS	0x30000000u	/* system/clock block the recorder never sampled */

/* INTC secondary banks (irq-rapuyama.c): phys 0x20033400 + 0x40*k, +0x08 = raw
 * pending (non-destructive, unlike primary IRQ_NUM which pops the line) */
#define SEC_PHYS	0x20033400u
#define SEC_STRIDE	0x40u
#define SEC_RAW		0x08u
#define SEC_BANKS	5

static void __iomem *fc, *mst, *soc, *c30, *sec;
static struct delayed_work thr_work, ping_work;
static unsigned long started;
static u32 fc_prev;
static u32 sec_prev[SEC_BANKS];

static u32 thr_fc(void)
{
	if (!fc)
		return 0;
	writel(FC_LATCH_VAL, fc + 0x04);
	return readl(fc + 0x00);
}

/* 100 ms: log only banks whose raw changed, anchored to the absolute fc clock */
static void ping_do(struct work_struct *w)
{
	int k;

	for (k = 0; k < SEC_BANKS; k++) {
		u32 raw = readl(sec + k * SEC_STRIDE + SEC_RAW);

		if (raw != sec_prev[k]) {
			rapuyama_note("thr ping fc=%08x bank%d raw %08x->%08x\n",
				      thr_fc(), k, sec_prev[k], raw);
			sec_prev[k] = raw;
		}
	}
	schedule_delayed_work(&ping_work, HZ / 10);
}

static void thr_do(struct work_struct *w)
{
	unsigned int secs = (jiffies - started) / HZ;
	u32 fcv = thr_fc(), mstv = 0, socv = 0;

	if (mst)
		mstv = readl(mst);
	if (soc)
		socv = readl(soc);

	rapuyama_note("thr %us fc=%08x d=%d mst=%06x soc=%08x\n",
		      secs, fcv, (int)(fcv - fc_prev), mstv & 0xffffff, socv);
	fc_prev = fcv;

	if ((probe & 0x2) && c30) {
		int i;

		/* commit the intent to ramcon BEFORE the read: if 0x30000000 is
		 * gated under Linux this readl external-aborts and this is the
		 * last line we get, which itself answers "is c30 reachable". */
		rapuyama_note("thr %us c30 read (abort risk)\n", secs);
		for (i = 0; i < 0x40; i += 0x10)
			rapuyama_note("thr   c30+%02x=%08x %08x %08x %08x\n", i,
				      readl(c30 + i), readl(c30 + i + 4),
				      readl(c30 + i + 8), readl(c30 + i + 0xc));
	}

	schedule_delayed_work(&thr_work, period * HZ);
}

static int __init rapuyama_theories_init(void)
{
	u32 sctlr = 0, vbar = 0, fc0;
	int k;

	if (!(probe & 0x7))
		return 0;

	asm volatile("mrc p15, 0, %0, c1, c0, 0" : "=r" (sctlr));
	asm volatile("mrc p15, 0, %0, c12, c0, 0" : "=r" (vbar));

	fc = ioremap(FC_PHYS, 8);
	mst = ioremap(MST_COUNT_PHYS, 4);
	soc = ioremap(SOC_PHYS, 4);
	if (probe & 0x2)
		c30 = ioremap(C30_PHYS, 0x40);
	if (probe & 0x4)
		sec = ioremap(SEC_PHYS, SEC_BANKS * SEC_STRIDE);

	fc0 = thr_fc();

	rapuyama_note("thr: armed probe=%#x period=%us fc0=%08x (fc @death - fc0 = absolute time since boot)\n",
		      probe, period, fc0);
	if (pexp)
		rapuyama_note("thr: PREDICT death ~%us (fc ~%08x); hit +-3s confirms the phase, miss falsifies it\n",
			      pexp, fc0 + pexp * 32768);
	/*
	 * honest status, NOT a claim that each is tested here:
	 *  #3 only actually tested with probe=0x3 AND real counter offsets (soc is
	 *     a 0x55555555 constant, mst is the 1ms reload aliased, fc never zeroes
	 *     - none can show an unknown counter hit 0; first wdfind 0x30000000 in
	 *     live Symbian to find a real down-counter, then sample it here).
	 *  #4 abort/undef would Oops into ramcon (MMU on, Linux vectors), so a clean
	 *     ramcon at death excludes them; only FIQ stays silent (needs a handler).
	 *  #5 every tick ts proves the CPU ran, but ramcon is DRAM so it cannot split
	 *     "CPU halted" from "DRAM halted" - needs an I-cache-resident MMIO blink.
	 */
	rapuyama_note("thr: #3 NOT tested unless probe=0x3+known offsets; #4 SCTLR.V=%d VBAR=%08x; #5 ts=CPU-alive only (DRAM-coupled)\n",
		      (sctlr >> 13) & 1, vbar);
	rapuyama_note("thr: not reachable from Linux: #1 MCE-RESP(try mailbox 0x20040580, not the aborting RX FIFO 0x20500000) #2 secure-auto #6 thermal(freezer) #7 reset-reason(boot_reason ISI) #8 awake-gpio(live Symbian)\n");

	fc_prev = fc0;
	started = jiffies;

	if (probe & 0x1) {
		INIT_DELAYED_WORK(&thr_work, thr_do);
		schedule_delayed_work(&thr_work, period * HZ);
	}
	if ((probe & 0x4) && sec) {
		for (k = 0; k < SEC_BANKS; k++)
			sec_prev[k] = readl(sec + k * SEC_STRIDE + SEC_RAW);
		rapuyama_note("thr: ping catcher armed, banks raw %08x %08x %08x %08x %08x\n",
			      sec_prev[0], sec_prev[1], sec_prev[2], sec_prev[3], sec_prev[4]);
		INIT_DELAYED_WORK(&ping_work, ping_do);
		schedule_delayed_work(&ping_work, HZ / 10);
	}
	return 0;
}
late_initcall(rapuyama_theories_init);