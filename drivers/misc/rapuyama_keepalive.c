// SPDX-License-Identifier: GPL-2.0
/*
 * Nokia RapuYama "throw everything at the random-reboot" keepalive.
 *
 * After the Symbian handoff the phone gets reset by an external source every
 * ~30-210 s that survives both watchdogs we already feed (SoC 0x200336c4 and
 * PMIC raw reg 0x17). This driver, running with the MMU on (the context the
 * secure monitor expects, unlike the bare-metal kickloop where smc #0 hung),
 * periodically does EVERY candidate keepalive/disable we have reversed, each
 * gated by a bit of the `mask` cmdline param, so we can bisect by changing
 * rapuyama_keepalive.mask=<bits> in bootargs with no rebuild:
 *
 *   bit0 0x01  SoC watchdog   0x200336c4 = 0x3fffffff
 *   bit1 0x02  PMIC raw 0x17  = 0x30        (the wdt, cmd 0x57)
 *   bit2 0x04  PMIC raw 0x10  = 0x7000      (power.dll logical reg 0xe, cmd 0x50)
 *   bit3 0x08  secure smc #0  (mmu on, zeroed buffer via c12) - the monitor call
 *   bit4 0x10  sisa self-irq  0x20040284 = 0x3000 (raise modem lines 30/31)
 *
 * if the phone survives past ~210 s with some mask -> that bit's action is (or
 * includes) the fix; bisect down. if it still dies with mask=0x1f -> the killer
 * is none of these (untouchable from the APE, e.g. modem hw supervision).
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#ifdef CONFIG_ARCH_RAPUYAMA
void rapuyama_note(const char *fmt, ...);
#else
static inline void rapuyama_note(const char *fmt, ...) { }
#endif

/*
 * default = the feedable set (SoC + both PMIC regs + sisa), NOT the smc (0x08):
 * the smc may not return (it hung from bare metal), which with irqs off would
 * wedge the cpu and SoC-reset in ~13 s, hiding the other results. test smc in a
 * separate run with mask=0x08.
 */
static unsigned int mask = 0x17;
module_param(mask, uint, 0644);
MODULE_PARM_DESC(mask, "bitmask of keepalive actions (see source)");

/* physical bases */
#define CBUS_PHYS	0x20030700u
#define SOC_PHYS	0x200336c4u
#define SISA_PHYS	0x20040280u

#define CBUS_EN		0x04
#define CBUS_BUSY	0x08
#define CBUS_WR_CMD	0x1c
#define CBUS_DATA	0x20
#define CBUS_BITS	0x2c
#define CBUS_WR_MODE	0x3c

#define FEED_SECS	90

static void __iomem *cbus;	/* 0x20030700 */
static void __iomem *soc;	/* 0x200336c4 */
static void __iomem *sisa;	/* 0x20040280 (status), +4 set, +8 clear */
static void *smcbuf;		/* zeroed page for the secure call */
static u32 smcbuf_phys;
static struct delayed_work ka_work;
static unsigned long started;

/* cbus write, raw PMIC register addr; cmd = dev<<6 | reg (see rapuyama_wdt) */
static void ka_cbus(u32 dev, u32 reg, u32 val)
{
	int n = 0x800;

	writel(1, cbus + CBUS_EN);
	writel(1, cbus + CBUS_WR_MODE);
	writel(16 - 1, cbus + CBUS_BITS);
	writel(val, cbus + CBUS_DATA);
	writel(dev << 6 | reg, cbus + CBUS_WR_CMD);
	while (readl(cbus + CBUS_BUSY) && --n)
		cpu_relax();
	writel(0, cbus + CBUS_EN);
}

/* smc #0 the way securitykext does, mmu on. r0=0, buffer range passed via c12.
 * .inst keeps it building without an arch-extension switch. irqs masked like the
 * firmware prologue - if the monitor never returns the cpu is stuck here and the
 * SoC watchdog resets us in ~13 s (that itself tells us smc hangs even mmu-on) */
static void ka_smc(void)
{
	unsigned long flags;

	local_irq_save(flags);
	asm volatile(
		"mov	r1, %0\n\t"
		"add	r2, r1, #0x60\n\t"
		"mcrr	p15, 0, r2, r1, c12\n\t"
		"mov	r0, #0\n\t"
		"mcr	p15, 0, r0, c7, c10, 4\n\t"	/* DSB */
		"mcr	p15, 0, r0, c7, c5, 4\n\t"	/* ISB */
		".inst	0xe1600070\n\t"			/* smc #0 */
		:
		: "r" (smcbuf_phys)
		: "r0", "r1", "r2", "memory", "cc");
	local_irq_restore(flags);
}

static void ka_do(struct work_struct *w)
{
	unsigned int secs = (jiffies - started) / HZ;

	if (mask & 0x01)
		writel(0x3fffffff, soc);
	if (mask & 0x02)
		ka_cbus(1, 0x17, 0x30);
	if (mask & 0x04)
		ka_cbus(1, 0x10, 0x7000);
	if (mask & 0x10)
		writel(0x3000, sisa + 0x04);	/* set lines 30/31 */
	if (mask & 0x08)
		ka_smc();			/* last: may not return */

	rapuyama_note("keepalive: alive %u s, mask %#x done\n", secs, mask);
	schedule_delayed_work(&ka_work, FEED_SECS * HZ);
}

static int __init rapuyama_keepalive_init(void)
{
	cbus = ioremap(CBUS_PHYS, 0x100);
	soc = ioremap(SOC_PHYS & ~0xfffu, 0x1000);
	sisa = ioremap(SISA_PHYS, 0x10);
	if (!cbus || !soc || !sisa) {
		rapuyama_note("keepalive: ioremap failed\n");
		return -ENOMEM;
	}
	soc += SOC_PHYS & 0xfffu;

	smcbuf = (void *)get_zeroed_page(GFP_KERNEL);
	if (smcbuf)
		smcbuf_phys = __pa(smcbuf);

	started = jiffies;
	rapuyama_note("keepalive: armed, mask %#x, period %d s\n", mask, FEED_SECS);
	INIT_DELAYED_WORK(&ka_work, ka_do);
	schedule_delayed_work(&ka_work, HZ); /* first pass ~1 s in */
	return 0;
}
late_initcall(rapuyama_keepalive_init);