// SPDX-License-Identifier: GPL-2.0
/*
 * self-learning disable one subsystem per boot and remember sweep to hunt 
 * the external ~210 s reset by elimination. State lives in a
 * reserved DRAM word block at 0x1f0c0000 that survives the warm reset, so it
 * progresses across reboots on its own: each boot it disables the next clock
 * gate (modem / co-processor ones first), a heartbeat records how long that
 * config survived, and the NEXT boot reads that - if a gate made the phone live
 * past SWEEP_WIN seconds it LOCKS that gate off (the found fix stays applied),
 * otherwise it advances to the next gate. A gate that hangs Linux just records
 * ~0 s and gets skipped. Enable with rapuyama_sweep.go=1 on the cmdline.
 *
 * Gate table + disable semantics from drivers/clk/clk-rapuyama.c: cgc base phys
 * 0x20032f00, R+4 = set bits, R+8 = clear bits, polarity 1 => set means ON (so
 * to turn OFF: pol?clear:set).
 */

#include <linux/io.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>

#ifdef CONFIG_ARCH_RAPUYAMA
void rapuyama_note(const char *fmt, ...);
#else
static inline void rapuyama_note(const char *fmt, ...) { }
#endif

static int go;
module_param(go, int, 0644);

#define CGC_PHYS	0x20032f00u
#define CGC_SET		0x4
#define CGC_CLR		0x8
#define ST_PHYS		0x1f0c0000u
#define ST_MAGIC	0x53575032u	/* 'SWP2' */
#define SWEEP_WIN	300		/* survive this many s with a gate off = found */

struct gate { u16 reg; u16 mask; u8 pol; };

/* suspicious first: hsusb/modem (0x15,0x16) + the odd-polarity domains, then the rest */
static const struct gate order[] = {
	{ 0x250, 0x2000, 0 }, { 0x250, 0x1000, 0 },		/* 0x15 0x16 */
	{ 0x150, 0x0008, 0 }, { 0x150, 0x0004, 0 },		/* 0x1e 0x1f */
	{ 0x240, 0x0010, 0 }, { 0x150, 0x0200, 0 },		/* 0x29 0x2a */
	{ 0x190, 0x0020, 1 }, { 0x190, 0x0102, 1 }, { 0x190, 0x0004, 1 }, /* 0x17-0x19 */
	{ 0x190, 0x0200, 1 }, { 0x190, 0x0008, 1 },		/* 0x2c 0x2d */
	{ 0x280, 0x0080, 1 }, { 0x280, 0x0100, 1 },		/* 0x25 0x26 */
	{ 0x1b0, 0x0010, 1 }, { 0x1b0, 0x0020, 1 }, { 0x1b0, 0x0040, 1 }, { 0x1b0, 0x0008, 1 },
	{ 0x180, 0x0020, 1 }, { 0x180, 0x0040, 1 }, { 0x180, 0x0800, 1 }, { 0x180, 0x0010, 1 },
	{ 0x1d0, 0x0010, 1 }, { 0x1d0, 0x0040, 1 }, { 0x1d0, 0x0080, 1 }, { 0x1d0, 0x0001, 1 },
	{ 0x1d0, 0x0004, 1 }, { 0x1d0, 0x0008, 1 }, { 0x1d0, 0x0100, 1 }, { 0x1d0, 0x0400, 1 },
	{ 0x1d0, 0x0800, 1 }, { 0x1d0, 0x0002, 1 }, { 0x1d0, 0x2000, 1 }, { 0x1d0, 0x1000, 1 },
	{ 0x1d0, 0x0020, 1 }, { 0x1d0, 0x8000, 1 }, { 0x0e0, 0x0080, 1 },
};
#define NORDER	(sizeof(order) / sizeof(order[0]))

struct sweep_state {
	u32 magic;
	u32 idx;	/* order index being tested */
	u32 done;	/* 1 = found+locked, 2 = exhausted */
	u32 surv[64];	/* seconds survived per order index */
	u32 best;
};

static void __iomem *cgc;
static struct sweep_state *st;
static struct delayed_work hb;
static unsigned long t0;

static void gate_off(const struct gate *g)
{
	writel(g->mask, cgc + g->reg + (g->pol ? CGC_CLR : CGC_SET));
}

static void hb_fn(struct work_struct *w)
{
	if (st->idx < NORDER)
		st->surv[st->idx] = (jiffies - t0) / HZ;
	schedule_delayed_work(&hb, 5 * HZ);
}

static int __init rapuyama_sweep_init(void)
{
	if (!go)
		return 0;

	cgc = ioremap(CGC_PHYS, 0x400);
	st = ioremap(ST_PHYS, sizeof(*st));
	if (!cgc || !st)
		return -ENOMEM;

	if (st->magic != ST_MAGIC) {
		memset(st, 0, sizeof(*st));
		st->magic = ST_MAGIC;
		st->idx = 0;
	} else if (!st->done) {
		/* the previous boot was testing order[idx]; surv[idx] = how long it
		 * lived. decide before touching anything this boot. */
		if (st->idx < NORDER && st->surv[st->idx] >= SWEEP_WIN) {
			st->done = 1;
			st->best = st->idx;	/* this gate off = the fix, lock it */
		} else {
			st->idx++;
			if (st->idx >= NORDER)
				st->done = 2;
		}
	}

	if (st->done == 1) {
		gate_off(&order[st->best]);
		rapuyama_note("sweep: FOUND - gate order %u off survived %u s, locked\n",
			      st->best, st->surv[st->best]);
		return 0;
	}
	if (st->done == 2) {
		rapuyama_note("sweep: EXHAUSTED all %u gates, none helped\n", (u32)NORDER);
		return 0;
	}

	st->surv[st->idx] = 0;	/* committed before we risk a hang */
	rapuyama_note("sweep: testing order %u (reg %#x mask %#x pol %u)\n",
		      st->idx, order[st->idx].reg, order[st->idx].mask, order[st->idx].pol);
	gate_off(&order[st->idx]);

	t0 = jiffies;
	INIT_DELAYED_WORK(&hb, hb_fn);
	schedule_delayed_work(&hb, 2 * HZ);
	return 0;
}
late_initcall(rapuyama_sweep_init);