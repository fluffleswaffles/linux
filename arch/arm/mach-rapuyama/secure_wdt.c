// SPDX-License-Identifier: GPL-2.0
/* PA_SWD command 0x1f, normally sent by ISASW every timeout, 30s (E6, E5) */
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/memblock.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/slab.h>
#include <linux/sizes.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>
#include <asm/cacheflush.h>
#include <asm/early_ioremap.h>
#include <asm/sections.h>
#include <asm/tlbflush.h>

#include "lowmem.h"

#define ISAH_PHYS	0x1f0f1000
#define ISAH_MAGIC	0x48415349
#define ISAH_L1		0x400		/* word offset of the kernel L1 copy */
#define GUEST_VIRT	0xc6800000
#define CODE_PHYS	0x10480000
#define CODE_SIZE	0x70000
#define IDENT_ATTR	0x1c0e

/* one firmware image: everything below was checked against its dump */
struct swd_fw {
	const char *compat;
	u32 isah_ver;
	u32 guest_phys, guest_size, guest_end;
	u32 sig_off[3], sig[3];		/* alloc, free, pet prologues */
	u32 token_lit;			/* code offset of the token pointer */
	u32 sp_off;			/* guest offset of the parked task sp */
	u32 alloc_fn, free_fn, pet_fn;
};

static const struct swd_fw swd_fws[] = {
	{	/* E6: the values of the original E6-only bridge */
		.compat = "nokia,e6", .isah_ver = 1,
		.guest_phys = 0x1f8b0000, .guest_size = 0x30000,
		.guest_end = 0xc6c4c000,
		.sig_off = { 0x61abe, 0x619b8, 0x62b38 },
		.sig = { 0x4605b570, 0x4606b5f8, 0x4605b5fe },
		.token_lit = 0x58b90, .sp_off = 0x1b7bc,
		.alloc_fn = 0xc6261abf, .free_fn = 0xc62619b9,
		.pet_fn = 0xc6262b39,
	},
	{	/* E5 fw 101.003, dump1 + run 4 (e5bot3 v2 isah) */
		.compat = "nokia,e5", .isah_ver = 2,
		.guest_phys = 0x1fd50000, .guest_size = 0x36000,
		.guest_end = 0xc6a80000,
		.sig_off = { 0x69442, 0x6933c, 0x6a2ee },
		.sig = { 0x4605b570, 0x4606b5f8, 0x4605b5fe },
		.token_lit = 0x61a64, .sp_off = 0x35f74,
		.alloc_fn = 0xc6369443, .free_fn = 0xc636933d,
		.pet_fn = 0xc636a2ef,
	},
};

/* E5 v2 isah: the 8 ISASW MBs, {va, L1 desc} at words 0x20.. */
static const u32 e5_isah_mb[8] = {
	0xc6300000, 0xc6400000, 0xc6500000, 0xc6600000,
	0xc6800000, 0xc6900000, 0xc6a00000, 0xc6b00000,
};

/* offsets are shared with secure_wdt_call.S */
struct swd_call {
	u32 ttbr0, ttbcr, dacr, context, prrr, nmrr, sp, cpsr;
	u32 pgd, guest_context, guest_prrr, guest_nmrr, guest_sp;
	u32 alloc_fn, free_fn, pet_fn;
	u32 token, transport, result, response, request_ptr, response_ptr, stage;
	u32 sctlr, guest_sctlr;
};

extern void rapuyama_swd_call(struct swd_call *call);
extern void rapuyama_note(const char *fmt, ...);

static bool swd_enabled;
static u32 *swd_pgd;
static struct swd_call *swd;
static struct delayed_work swd_work;
static u32 swd_calls;
static int swd_keep_state;	/* E5: 1 all symbian pages safe, -1 not */

static int __init swd_param(char *s)
{
	return kstrtobool(s, &swd_enabled);
}
early_param("e6.swd", swd_param);

/*
 * E5: the pet path leaves ISASW. Its VA->PA helper calls symbian's host table
 * at 0xc80088f4 -> 0x80107988 -> 0x800a03f0, a software walk that reads the
 * page directory (0xc1000000), the page table bitmap (0xc0030000) and
 * SPageTableInfo (0xfc000000 + (L2 pa >> 12) * 32); the PA lookup reads the
 * protected app table at 0xcb294000, the monitor the PA_SWD image. Below the
 * kernel nothing is linux ram, above it nothing of symbian's is kept, so
 * reserve those pages here, before anything is allocated.
 */
static const u32 e5_keep_va[] __initconst = {
	0xc80088f4, 0x80107988, 0x800a03f0, 0x80065b28, 0x80066b28,
	0x8008c23c, 0xc0030000, 0xc1003000,
};
static const u32 e5_walk_mb[] __initconst = { 0xc63, 0xc68, 0xc6b, 0xcb2 };
#define E5_PAT_PTR	0xc6831104	/* -> 0xcb294000 in run 4 */
#define E5_PAT_MAX	64
#define E5_PA_MAX	SZ_64K

static u32 __init swd_l1(u32 mb)
{
	u32 *l1, d;

	l1 = early_memremap(ISAH_PHYS + ISAH_L1 * 4 + mb * 4, 4);
	if (!l1)
		return 0;
	d = *l1;
	early_memunmap(l1, 4);
	return d;
}

/* translate through symbian's kernel L1 copy, ~0 if unmapped */
static u32 __init swd_va2pa(u32 va)
{
	u32 d = swd_l1(va >> 20), *l2, e;

	if ((d & 3) == 2 && !(d & BIT(18)))
		return (d & 0xfff00000) | (va & 0xfffff);
	if ((d & 3) != 1)
		return ~0U;
	l2 = early_memremap((d & ~0x3ffU) + ((va >> 12) & 0xff) * 4, 4);
	if (!l2)
		return ~0U;
	e = *l2;
	early_memunmap(l2, 4);
	if (e & 2)
		return (e & 0xfffff000) | (va & 0xfff);
	if (e & 1)
		return (e & 0xffff0000) | (va & 0xffff);
	return ~0U;
}

static bool __init swd_keep(u32 va, phys_addr_t kbase)
{
	u32 pa = swd_va2pa(va);
	phys_addr_t pg = pa & PAGE_MASK;

	if (pa == ~0U) {
		pr_err("rapuyama-swd: keep %08x unmapped\n", va);
		return false;
	}
	if (pg < kbase || !memblock_is_memory(pg)) {
		/* below the kernel is linux ram with e6.lowmem: lowmem keeps it */
		if (memblock_is_memory(pg))
			rapuyama_lowmem_keep(pg);
		pr_info("rapuyama-swd: keep %08x pa %08x below the kernel\n", va, pa);
		return true;
	}
	if (pg < __pa_symbol(_end)) {
		pr_err("rapuyama-swd: keep %08x pa %08x under the kernel image\n",
		       va, pa);
		return false;
	}
	memblock_reserve(pg, PAGE_SIZE);
	pr_info("rapuyama-swd: keep %08x pa %08x reserved\n", va, pa);
	return true;
}

static u32 __init swd_read(u32 va)
{
	u32 pa = swd_va2pa(va), *p, v;

	if (pa == ~0U)
		return 0;
	p = early_memremap(pa, 4);
	if (!p)
		return 0;
	v = *p;
	early_memunmap(p, 4);
	return v;
}

void __init rapuyama_swd_reserve(void)
{
	phys_addr_t kbase = round_down(__pa_symbol(_text), SZ_2M);
	u32 *h, rec, pat, off = 0, len = 0, va, d;
	bool ok = true;
	int i;

	if (!swd_enabled ||
	    !of_flat_dt_is_compatible(of_get_flat_dt_root(), "nokia,e5"))
		return;
	h = early_memremap(ISAH_PHYS, PAGE_SIZE);
	if (!h)
		return;
	i = h[0] == ISAH_MAGIC && h[1] == 2;
	early_memunmap(h, PAGE_SIZE);
	if (!i) {
		pr_err("rapuyama-swd: no v2 isah block, nothing kept\n");
		swd_keep_state = -1;
		return;
	}
	for (i = 0; i < ARRAY_SIZE(e5_keep_va); i++)
		ok &= swd_keep(e5_keep_va[i], kbase);
	for (i = 0; i < ARRAY_SIZE(e5_walk_mb); i++) {
		d = swd_l1(e5_walk_mb[i]);
		if ((d & 3) != 1) {
			ok = false;
			continue;
		}
		ok &= swd_keep(0xfc000000 + ((d & ~0x3ffU) >> 12) * 32, kbase);
	}
	/* {offset, length?, ..., name at +0x14}, 0xff name ends the table */
	pat = swd_read(E5_PAT_PTR);
	pr_info("rapuyama-swd: protected app table %08x\n", pat);
	for (rec = 0; pat && rec < E5_PAT_MAX; rec++) {
		va = pat + rec * 0x20;
		if (!(va & 0xfff) || !rec)
			ok &= swd_keep(va, kbase);
		d = swd_read(va + 0x14);
		if ((d & 0xff) == 0xff)
			break;
		if (d == 0x535f4150 &&		/* "PA_S" */
		    (swd_read(va + 0x18) & 0xffffff) == 0x4457) { /* "WD\0" */
			off = swd_read(va);
			len = swd_read(va + 4);
			break;
		}
	}
	if (!off) {
		pr_err("rapuyama-swd: no PA_SWD record\n");
		ok = false;
	} else {
		pr_info("rapuyama-swd: PA_SWD record %u off %x len %x\n",
			rec, off, len);
		if (!len || len > E5_PA_MAX)
			len = E5_PA_MAX;
		va = pat + off;
		for (d = va & PAGE_MASK; d < va + len; d += PAGE_SIZE)
			ok &= swd_keep(d, kbase);
	}
	swd_keep_state = ok ? 1 : -1;
}

static void swd_ping(struct work_struct *work)
{
	u32 *pgd;
	u32 ttbr0, ttbcr, code_pa, data_pa, ci, di, old_code, old_data;
	unsigned long flags;
	u32 timeout;

	code_pa = (u32)virt_to_phys(rapuyama_swd_call);
	data_pa = (u32)virt_to_phys(swd);
	ci = code_pa >> 20;
	di = data_pa >> 20;
	preempt_disable();
	local_irq_save(flags);
	asm volatile("mrc p15, 0, %0, c2, c0, 0" : "=r"(ttbr0));
	asm volatile("mrc p15, 0, %0, c2, c0, 2" : "=r"(ttbcr));
	if (ttbcr & 7) {
		local_irq_restore(flags);
		preempt_enable();
		pr_err("rapuyama-swd: unsupported Linux TTBCR %08x\n", ttbcr);
		return;
	}
	pgd = phys_to_virt(ttbr0 & ~0x3fffU);
	old_code = pgd[ci];
	old_data = pgd[di];
	pgd[ci] = (code_pa & 0xfff00000) | IDENT_ATTR;
	pgd[di] = (data_pa & 0xfff00000) | IDENT_ATTR;
	flush_cache_all();
	local_flush_tlb_all();
	swd->stage = 0;
	swd->transport = ~0U;
	swd->result = ~0U;
	rapuyama_note("swd: call %u token %u, entering PA_SWD 0x1f\n",
		      swd_calls + 1, swd->token);
	((void (*)(struct swd_call *))(uintptr_t)code_pa)
		((struct swd_call *)(uintptr_t)data_pa);
	pgd[ci] = old_code;
	pgd[di] = old_data;
	flush_cache_all();
	local_flush_tlb_all();
	local_irq_restore(flags);
	preempt_enable();

	swd_calls++;
	timeout = swd->response & 0xffff;
	/* only the first pet goes to the console, the rest are routine */
	if (swd_calls == 1)
		pr_info("rapuyama-swd: call %u stage %u transport %d result %u timeout %u token %u\n",
			swd_calls, swd->stage, (s32)swd->transport,
			swd->result, timeout, swd->response >> 16);
	else
		pr_debug("rapuyama-swd: call %u stage %u transport %d result %u timeout %u token %u\n",
			 swd_calls, swd->stage, (s32)swd->transport,
			 swd->result, timeout, swd->response >> 16);
	if (swd->stage != 60 || swd->transport != 1 || swd->result != 1 ||
	    timeout <= 30 || timeout > 3600) {
		pr_err("rapuyama-swd: secure pet failed (call %u stage %u transport %d result %u timeout %u)\n",
		       swd_calls, swd->stage, (s32)swd->transport,
		       swd->result, timeout);
		return;
	}
	swd->token = swd->response >> 16;
	schedule_delayed_work(&swd_work, (timeout - 30) * HZ);
}

/* the isah block must belong to this firmware's MMU layout */
static bool __init swd_isah_ok(const struct swd_fw *fw, const u32 *h)
{
	int i;

	if (h[0] != ISAH_MAGIC || h[1] != fw->isah_ver)
		return false;
	if (fw->isah_ver == 1)
		return h[0x10] == 0xc62bb5d7 && h[0x11] == 0xc62bb5e1;
	if (swd_keep_state != 1)
		return false;
	for (i = 0; i < ARRAY_SIZE(e5_isah_mb); i++)
		if (h[0x20 + 2 * i] != e5_isah_mb[i] ||
		    h[0x21 + 2 * i] != h[ISAH_L1 + (e5_isah_mb[i] >> 20)] ||
		    (h[0x21 + 2 * i] & 3) != 1)
			return false;
	return true;
}

static int __init rapuyama_swd_init(void)
{
	const struct swd_fw *fw = NULL;
	const u32 *h;
	u8 *guest;
	u32 *code;
	u32 token_ptr, sp, code_pa, data_pa;
	int i, ret = -ENODEV;

	BUILD_BUG_ON(offsetof(struct swd_call, stage) != 88);
	BUILD_BUG_ON(offsetof(struct swd_call, guest_sctlr) != 96);
	if (!swd_enabled)
		return 0;
	for (i = 0; i < ARRAY_SIZE(swd_fws); i++)
		if (of_machine_is_compatible(swd_fws[i].compat))
			fw = &swd_fws[i];
	if (!fw)
		return 0;
	h = memremap(ISAH_PHYS, 0x5000, MEMREMAP_WB);
	guest = memremap(fw->guest_phys, fw->guest_size, MEMREMAP_WB);
	code = memremap(CODE_PHYS, CODE_SIZE, MEMREMAP_WB);
	if (!h || !guest || !code)
		goto out;
	/* firmware-specific bridge: reject another image before calling any code */
	if (!swd_isah_ok(fw, h))
		goto out;
	for (i = 0; i < 3; i++)
		if (get_unaligned_le32((u8 *)code + fw->sig_off[i]) != fw->sig[i])
			goto out;
	token_ptr = READ_ONCE(code[fw->token_lit / 4]);
	if (token_ptr < GUEST_VIRT || token_ptr + 4 > GUEST_VIRT + fw->guest_size)
		goto out;
	/* borrow the stopped server task's stack; the ROM VA-to-PA helper uses
	 * Symbian's original page tables, so an identity Linux stack won't work */
	sp = get_unaligned_le32(guest + fw->sp_off) & ~7U;
	if (sp < GUEST_VIRT + 0x1000 || sp > fw->guest_end)
		goto out;
	swd_pgd = (u32 *)__get_free_pages(GFP_KERNEL | __GFP_ZERO, 2);
	swd = (struct swd_call *)get_zeroed_page(GFP_KERNEL);
	if (!swd_pgd || !swd) {
		ret = -ENOMEM;
		goto out;
	}
	memcpy(swd_pgd, h + ISAH_L1, 0x4000);
	code_pa = (u32)virt_to_phys(rapuyama_swd_call);
	data_pa = (u32)virt_to_phys(swd);
	swd_pgd[code_pa >> 20] = (code_pa & 0xfff00000) | IDENT_ATTR;
	swd_pgd[data_pa >> 20] = (data_pa & 0xfff00000) | IDENT_ATTR;
	swd->pgd = (u32)virt_to_phys(swd_pgd) | (h[3] & 0x7f);
	swd->guest_context = h[6];
	swd->guest_prrr = h[8];
	swd->guest_nmrr = h[9];
	swd->guest_sctlr = h[7];
	swd->guest_sp = sp;
	swd->alloc_fn = fw->alloc_fn;
	swd->free_fn = fw->free_fn;
	swd->pet_fn = fw->pet_fn;
	swd->token = get_unaligned_le16(guest + token_ptr - GUEST_VIRT + 2);
	INIT_DELAYED_WORK(&swd_work, swd_ping);
	pr_info("rapuyama-swd: PA_SWD 0x1f ready (%s), token %u guest stack %08x\n",
		fw->compat, swd->token, sp);
	schedule_delayed_work(&swd_work, 5 * HZ);
	ret = 0;
out:
	if (h)
		memunmap((void *)h);
	if (guest)
		memunmap(guest);
	if (code)
		memunmap(code);
	if (ret) {
		if (swd_pgd)
			free_pages((unsigned long)swd_pgd, 2);
		if (swd)
			free_page((unsigned long)swd);
		pr_err("rapuyama-swd: firmware setup rejected (%d)\n", ret);
	}
	return ret;
}
late_initcall(rapuyama_swd_init);
