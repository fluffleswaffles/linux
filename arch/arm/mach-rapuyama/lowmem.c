// SPDX-License-Identifier: GPL-2.0
/*
 * Nokia E6: ram below the kernel.
 *
 * The booter boots the Image in place in the biggest block Symbian gives it,
 * 0x19a00000..0x1a000000 high, and arm drops all ram below PHYS_OFFSET. The
 * rapuyama TEXT_OFFSET (0x08a08000, arch/arm/Makefile) puts PHYS_OFFSET under
 * the ram start, so the ram from 0x11600000 up to the kernel is lowmem again.
 *
 * Symbian's pages in there stay reserved: everything its kernel L1 maps (the
 * isah copy the PA_SWD bridge runs on + the live TTBR1 table) and the L2
 * tables themselves. Free Symbian ram is unmapped, that is what we get back.
 *
 * e6.lowmem=0: drop the ram below the kernel like before. e6.lowwatch=1 then
 * checksums every page of it that Symbian doesn't map, every 30 s for 10 min,
 * and logs the ones that change: something (modem dma?) still writing there.
 *
 * e6.lowmem=2: keep only what can still be in use instead of every mapped
 * page: the L2 tables, everything symbian's kernel core maps (va 0xc0000000
 * ..0xc9ffffff and 0xfc000000 up: superpage, page directories and tables,
 * kernel data, heap, stacks: the PA_SWD bridge runs symbian kernel code that
 * reads them; releasing them killed the first secure call), all pages behind
 * a table that also maps io or symbian-top (device windows), the va
 * 0x90000000 table (the DSP window area on the E6) and the bridge's pages
 * (secure_wdt.c). The rest are chunks (multimedia sections, d3/d4..), E5
 * watch run: nothing wrote them for 10 min (modem idle).
 *
 * e6.lowwatch=2 (with e6.lowmem=1, debug, changes nothing): print where every
 * L2 table of symbian's kernel L1 copy points (below/low/high/top/io) and
 * checksum the kept symbian pages instead, to find which of them are dead.
 * Cached reads through the linear map (ram can't be remapped uncached); one
 * pass reads ~36 MB, far more than the 256 KB L2, so nothing stays cached.
 */
#include <linux/bitmap.h>
#include <linux/export.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/memblock.h>
#include <linux/sizes.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>
#include <asm/early_ioremap.h>
#include <asm/sections.h>

#include "lowmem.h"

#define RY_RAM_LO	0x11600000	/* below: modem-run code, see e6.dts */
#define RY_RAM_HI	0x20000000
#define RY_NPG		((RY_RAM_HI - RY_RAM_LO) >> PAGE_SHIFT)
#define RY_ISAH_PHYS	0x1f0f1000
#define RY_ISAH_MAGIC	0x48415349
#define RY_ISAH_L1	0x1000		/* byte offset of the kernel L1 copy */
#define RY_SYM_L1	0x10000000	/* symbian ttbr1 without an isah block */
#define RY_CHUNK_MAP	SZ_32M		/* what the booter mapped of its block */
#define RY_WATCH_PERIOD	(30 * HZ)
#define RY_WATCH_PASSES	20
#define RY_WATCH_MAXLOG	16
#define RY_VC_PUSH	0x12c000	/* 640 x 480 x 4 */
#define RY_VC_TX	0x2000		/* one vchi tx slot (0x1020) */
#define RY_DTB_OFF	0x1f00000	/* where the booter puts the dtb */

/*
 * gfx dma buffers for videocore.c (push buffer + vchi tx slot), right after
 * the kernel in our own block: nothing of symbian's there, and low. the gfx
 * dma timed out reading a tx slot at 0x1bef9180 and up, worked up to 0x1b7d1180
 */
phys_addr_t rapuyama_vc_push;
EXPORT_SYMBOL(rapuyama_vc_push);
phys_addr_t rapuyama_vc_tx;
EXPORT_SYMBOL(rapuyama_vc_tx);

static DECLARE_BITMAP(ry_sym, RY_NPG);
static DECLARE_BITMAP(ry_keep, RY_NPG);	/* e6.lowmem=2: still needed */
static unsigned int ry_lowmem;		/* 0 off, 1 keep mapped, 2 keep needed */
static unsigned int ry_lowwatch;	/* 1: unmapped pages, 2: kept pages */
static bool ry_walk_report;
static phys_addr_t ry_kbase;

static int __init ry_lowmem_param(char *s)
{
	return kstrtouint(s, 0, &ry_lowmem);
}
early_param("e6.lowmem", ry_lowmem_param);

static int __init ry_lowwatch_param(char *s)
{
	return kstrtouint(s, 0, &ry_lowwatch);
}
early_param("e6.lowwatch", ry_lowwatch_param);

static bool __init ry_is_ram(phys_addr_t pa)
{
	return pa >= 0x10000000 && pa < RY_RAM_HI;
}

static void __init ry_set(unsigned long *map, phys_addr_t pa, phys_addr_t len)
{
	for (; len; pa += PAGE_SIZE, len -= PAGE_SIZE)
		if (pa >= RY_RAM_LO && pa < RY_RAM_HI)
			__set_bit((pa - RY_RAM_LO) >> PAGE_SHIFT, map);
}

static void __init ry_mark(phys_addr_t pa, phys_addr_t len)
{
	ry_set(ry_sym, pa, len);
}

/* secure_wdt.c: a page the PA_SWD bridge touches, below the kernel */
void __init rapuyama_lowmem_keep(phys_addr_t pa)
{
	ry_set(ry_keep, pa & PAGE_MASK, PAGE_SIZE);
	ry_set(ry_sym, pa & PAGE_MASK, PAGE_SIZE);
}

static const char *const ry_cls_name[] = { "below", "low", "high", "top", "io" };

/* where a page of symbian's lands for the e6.lowwatch=2 report */
static int __init ry_cls(phys_addr_t pa)
{
	if (pa < 0x10000000 || pa >= RY_RAM_HI)
		return 4;
	if (pa < RY_RAM_LO)
		return 0;
	if (pa < ry_kbase)
		return 1;
	if (pa < 0x1fb60000)	/* symbian-top in e5.dts */
		return 2;
	return 3;
}

/* e6.lowmem=2: symbian kernel core, read by the code the PA_SWD bridge runs */
static bool __init ry_core_va(u32 va)
{
	return (va >= 0xc0000000 && va < 0xca000000) || va >= 0xfc000000;
}

static void __init ry_walk_l2(phys_addr_t tp, u32 va)
{
	unsigned int n[5] = { 0 };
	u32 lo = ~0U, hi = 0, pa;
	u32 *l2, e;
	int j;

	ry_mark(tp & PAGE_MASK, PAGE_SIZE);
	ry_set(ry_keep, tp & PAGE_MASK, PAGE_SIZE);
	l2 = early_memremap(tp, 0x400);
	if (!l2)
		return;
	for (j = 0; j < 256; j++) {
		e = l2[j];
		if (e & 2) {
			pa = e & PAGE_MASK;
			ry_mark(pa, PAGE_SIZE);
		} else if (e & 1) {
			pa = e & 0xffff0000;
			ry_mark(pa, SZ_64K);
		} else {
			continue;
		}
		n[ry_cls(pa)]++;
		lo = min(lo, pa);
		hi = max(hi, pa);
	}
	/* device windows and the DSP area: keep every page of the table */
	if (n[3] || n[4] || va == 0x90000000 || ry_core_va(va))
		for (j = 0; j < 256; j++) {
			e = l2[j];
			if (e & 2)
				ry_set(ry_keep, e & PAGE_MASK, PAGE_SIZE);
			else if (e & 1)
				ry_set(ry_keep, e & 0xffff0000, SZ_64K);
		}
	early_memunmap(l2, 0x400);
	if (ry_walk_report)
		pr_info("rapuyama: lowmap va %08x l2 %08x (%s): below %u low %u high %u top %u io %u, pa %08x..%08x\n",
			va, (u32)tp, ry_cls_name[ry_cls(tp)], n[0], n[1], n[2],
			n[3], n[4], n[0] + n[1] + n[2] + n[3] + n[4] ? lo : 0, hi);
}

/* short-descriptor L1: sections, supersections and L2 tables in ram */
static void __init ry_walk_l1(phys_addr_t l1pa)
{
	u32 *l1, d;
	int i;

	l1 = early_memremap(l1pa, SZ_16K);
	if (!l1)
		return;
	for (i = 0; i < 4096; i++) {
		d = l1[i];
		if ((d & 3) == 2 && (d & BIT(18)))
			ry_mark(d & 0xff000000, SZ_16M);
		else if ((d & 3) == 2)
			ry_mark(d & 0xfff00000, SZ_1M);
		if ((d & 3) == 2 && ry_core_va((u32)i << 20))
			ry_set(ry_keep, d & ((d & BIT(18)) ? 0xff000000 : 0xfff00000),
			       (d & BIT(18)) ? SZ_16M : SZ_1M);
		else if ((d & 3) == 1 && ry_is_ram(d & ~0x3ffU))
			ry_walk_l2(d & ~0x3ffU, (u32)i << 20);
		else
			continue;
		if (ry_walk_report && (d & 3) == 2 && ry_cls(d & 0xfff00000) < 4)
			pr_info("rapuyama: lowmap va %08x section pa %08x (%s)\n",
				(u32)i << 20, d & 0xfff00000,
				ry_cls_name[ry_cls(d & 0xfff00000)]);
	}
	early_memunmap(l1, SZ_16K);
}

/* machine .reserve: memblock can't grow yet, keep the whole range for now */
void __init rapuyama_lowmem_reserve(void)
{
	u32 *h;
	u32 ttbr1 = RY_SYM_L1;
	phys_addr_t push;

	ry_kbase = round_down(__pa_symbol(_text), SZ_2M);
	rapuyama_swd_reserve();
	push = round_up(__pa_symbol(_end), SZ_64K);
	if (push + RY_VC_PUSH + RY_VC_TX <= ry_kbase + RY_DTB_OFF &&
	    !memblock_reserve(push, RY_VC_PUSH + RY_VC_TX)) {
		rapuyama_vc_push = push;
		rapuyama_vc_tx = push + RY_VC_PUSH;
	}
	if (ry_kbase <= RY_RAM_LO)
		return;
	h = early_memremap(RY_ISAH_PHYS, PAGE_SIZE);
	if (h && h[0] == RY_ISAH_MAGIC) {
		ttbr1 = h[3] & ~0x3fffU;
		early_memunmap(h, PAGE_SIZE);
		/* report the isah copy only, the live table is near-identical */
		ry_walk_report = ry_lowwatch == 2;
		ry_walk_l1(RY_ISAH_PHYS + RY_ISAH_L1);
		ry_walk_report = false;
	} else if (h) {
		early_memunmap(h, PAGE_SIZE);
	}
	if (ry_is_ram(ttbr1))
		ry_walk_l1(ttbr1);
	/* the booter's own mapping of our block, that is the kernel */
	bitmap_clear(ry_sym, (ry_kbase - RY_RAM_LO) >> PAGE_SHIFT,
		     min_t(phys_addr_t, RY_CHUNK_MAP, RY_RAM_HI - ry_kbase) >> PAGE_SHIFT);
	bitmap_clear(ry_keep, (ry_kbase - RY_RAM_LO) >> PAGE_SHIFT,
		     min_t(phys_addr_t, RY_CHUNK_MAP, RY_RAM_HI - ry_kbase) >> PAGE_SHIFT);
	if (!ry_lowmem) {
		memblock_remove(RY_RAM_LO, ry_kbase - RY_RAM_LO);
		return;
	}
	memblock_reserve(RY_RAM_LO, ry_kbase - RY_RAM_LO);
}

/* machine .init_early: memblock can grow now, hand out all but symbian's pages */
void __init rapuyama_lowmem_init_early(void)
{
	unsigned long *map = ry_lowmem == 2 ? ry_keep : ry_sym;
	unsigned int rs, re, kept = 0, low = 0;

	if (!ry_lowmem || ry_kbase <= RY_RAM_LO)
		return;
	memblock_phys_free(RY_RAM_LO, ry_kbase - RY_RAM_LO);
	for_each_set_bitrange(rs, re, map, RY_NPG) {
		memblock_reserve(RY_RAM_LO + ((phys_addr_t)rs << PAGE_SHIFT),
				 (phys_addr_t)(re - rs) << PAGE_SHIFT);
		kept += re - rs;
		if (RY_RAM_LO + ((phys_addr_t)rs << PAGE_SHIFT) < ry_kbase)
			low += re - rs;
	}
	pr_info("rapuyama: lowmem: %llu MB below the kernel back, %u KB of it kept for symbian (%u KB above)\n",
		(u64)(ry_kbase - RY_RAM_LO) >> 20, low * 4, (kept - low) * 4);
	if (ry_lowmem == 2)
		pr_info("rapuyama: lowmem=2: %u KB of symbian's mapped pages released\n",
			(unsigned int)(bitmap_weight(ry_sym, RY_NPG) - kept) * 4);
}

static u32 *ry_watch_virt;
static u32 *ry_watch_sum;
static unsigned int ry_watch_npg, ry_watch_pass;
static struct delayed_work ry_watch_work;

static u32 ry_page_sum(const u32 *p)
{
	u32 sum = 0;
	int i;

	for (i = 0; i < PAGE_SIZE / 4; i++)
		sum = rol32(sum, 5) ^ READ_ONCE(p[i]);
	return sum;
}

static void ry_watch_scan(struct work_struct *work)
{
	unsigned int pg, start = 0, run = 0, changed = 0, logged = 0;
	phys_addr_t a, b;
	u32 sum;

	for (pg = 0; pg <= ry_watch_npg; pg++) {
		bool hit = false;

		/*
		 * mode 2 skips no-map holes (ramcon with the isah block): not in
		 * the linear map, and pfn_valid() is true for them on arm
		 */
		if (pg < ry_watch_npg && test_bit(pg, ry_sym) == (ry_lowwatch == 2) &&
		    (ry_lowwatch != 2 ||
		     memblock_is_map_memory(RY_RAM_LO + ((phys_addr_t)pg << PAGE_SHIFT)))) {
			sum = ry_page_sum(ry_watch_virt + (pg << PAGE_SHIFT) / 4);
			hit = ry_watch_pass && sum != ry_watch_sum[pg];
			ry_watch_sum[pg] = sum;
			cond_resched();
		}
		if (hit) {
			if (!run++)
				start = pg;
			changed++;
			continue;
		}
		if (run && logged++ < RY_WATCH_MAXLOG) {
			a = RY_RAM_LO + ((phys_addr_t)start << PAGE_SHIFT);
			b = a + ((phys_addr_t)run << PAGE_SHIFT);
			pr_info("rapuyama: lowwatch pass %u: %pa..%pa changed (%u pages)\n",
				ry_watch_pass, &a, &b, run);
		}
		run = 0;
	}
	if (ry_watch_pass)
		pr_info("rapuyama: lowwatch pass %u of %u: %u pages changed\n",
			ry_watch_pass, RY_WATCH_PASSES, changed);
	if (ry_watch_pass++ < RY_WATCH_PASSES)
		schedule_delayed_work(&ry_watch_work, RY_WATCH_PERIOD);
}

static int __init ry_watch_init(void)
{
	unsigned int sym;

	/* only with lowmem=1: with 2 most of these pages are linux's */
	if (ry_lowwatch == 2 && ry_lowmem == 1 && ry_kbase > RY_RAM_LO) {
		/* the kept pages are reserved linux ram: the linear map sees them */
		ry_watch_npg = RY_NPG;
		ry_watch_virt = phys_to_virt(RY_RAM_LO);
		ry_watch_sum = vzalloc(ry_watch_npg * sizeof(u32));
		if (!ry_watch_sum)
			return 0;
		pr_info("rapuyama: lowwatch: watching the %u KB kept for symbian\n",
			(unsigned int)bitmap_weight(ry_sym, RY_NPG) * 4);
		INIT_DELAYED_WORK(&ry_watch_work, ry_watch_scan);
		schedule_delayed_work(&ry_watch_work, 0);
		return 0;
	}
	if (ry_lowmem || ry_lowwatch != 1 || ry_kbase <= RY_RAM_LO)
		return 0;
	ry_watch_npg = (ry_kbase - RY_RAM_LO) >> PAGE_SHIFT;
	sym = bitmap_weight(ry_sym, ry_watch_npg);
	/* not linux ram here: an uncached map sees what others write */
	ry_watch_virt = memremap(RY_RAM_LO, ry_kbase - RY_RAM_LO, MEMREMAP_WC);
	ry_watch_sum = vzalloc(ry_watch_npg * sizeof(u32));
	if (!ry_watch_virt || !ry_watch_sum) {
		pr_warn("rapuyama: lowwatch: no map\n");
		return 0;
	}
	pr_info("rapuyama: lowwatch: %u MB below the kernel, %u KB mapped by symbian, watching the other %u KB\n",
		ry_watch_npg >> 8, sym * 4, (ry_watch_npg - sym) * 4);
	INIT_DELAYED_WORK(&ry_watch_work, ry_watch_scan);
	schedule_delayed_work(&ry_watch_work, 0);
	return 0;
}
late_initcall(ry_watch_init);