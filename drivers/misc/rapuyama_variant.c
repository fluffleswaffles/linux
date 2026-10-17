// SPDX-License-Identifier: GPL-2.0
/*
 * Nokia RapuYama variant hardware init, replicated from the firmware so Linux
 * does the same setup steps Symbian's ecust.dll (FUN_800bede4) + power.dll do at
 * cold boot and that our port had been SKIPPING entirely - in particular the
 * modem/ISA interface block at 0x20040000-0x20044000. The random ~210 s reset is
 * the modem (CMT) resetting the whole SoC; configuring its interface the way the
 * firmware does is the missing step.
 *
 * Exact sequence from ecust.dll.c FUN_800bede4 cold-boot path (the else branch
 * of the superpage+0x724 == 0xe550cafe deep-sleep-resume check) plus power.dll's
 * *0x20033358 = 0 and the ecust clock-divider call FUN_800bec88(3,0xf,2).
 * All offsets are from the 0x20030000 peripheral base.
 */

#include <linux/io.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/moduleparam.h>

/*
 * off by default: writing these cold-boot values over a live system can gate a
 * domain Symbian left running (it broke USB in testing). set groups via cmdline:
 *   rapuyama_variant.en=1  sysctrl+clock (risky, may break peripherals)
 *   rapuyama_variant.en=2  modem/ISA interface only (0x20040000+)
 *   rapuyama_variant.en=3  both
 */
static int en;
module_param(en, int, 0644);

#ifdef CONFIG_ARCH_RAPUYAMA
void rapuyama_note(const char *fmt, ...);
#else
static inline void rapuyama_note(const char *fmt, ...) { }
#endif

#define BASE	0x20030000u
#define SIZE	0x14000u	/* up to 0x20043fff */

static void __iomem *v;
static inline void wr(u32 phys, u32 val) { writel(val, v + (phys - BASE)); }

static int __init rapuyama_variant_init(void)
{
	int i;

	v = ioremap(BASE, SIZE);
	if (!v) {
		rapuyama_note("variant: ioremap failed\n");
		return -ENOMEM;
	}

	if (!en) {
		iounmap(v);
		v = NULL;
		return 0;	/* default: do nothing (keeps the working baseline) */
	}

	if (en & 1) {
	/* ecust clock divider: FUN_800bec88(3,0xf,2) -> 0x20032644 = (2) | (0xf<<5) */
	wr(0x20032644, 2 | (0xf << 5));

	/* ecust sysctrl config (FUN_800bede4) */
	wr(0x200330d8, 0xbfff);
	wr(0x20033010, 0x3494);
	wr(0x20033034, 0x6d00);
	wr(0x20033088, 0x8000);
	wr(0x200330b8, 1);
	wr(0x20033198, 0x200);
	wr(0x20033084, 8);
	wr(0x200330b4, 8);

	/* power.dll DPowerController init */
	wr(0x20033358, 0);
	}

	if (!(en & 2)) {
		rapuyama_note("variant: en=%d (sysctrl only)\n", en);
		return 0;
	}

	/* ecust modem/ISA interface init: 6 channels * 0x400, clear +0x488 */
	for (i = 0; i < 6; i++)
		wr(0x20040780 + i * 0x400 + 0x488, 0);
	/* clear 4 words at +0x1cc0 */
	for (i = 0; i < 4; i++)
		wr(0x20040780 + i * 4 + 0x1cc0, 0);

	/* ecust modem channel-control enable sequences (0x20043104 region) */
	wr(0x20043104, 2);
	wr(0x20043158, 1);
	wr(0x20043100, 0);
	wr(0x20043204, 2);
	wr(0x20043258, 1);
	wr(0x20043204, 0);
	wr(0x20042904, 2);
	wr(0x20042958, 1);
	wr(0x20042904, 0);

	rapuyama_note("variant: ecust+power+modem-iface init replayed\n");
	return 0;
}
arch_initcall(rapuyama_variant_init);