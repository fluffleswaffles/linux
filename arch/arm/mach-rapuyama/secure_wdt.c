// SPDX-License-Identifier: GPL-2.0
/* E6 firmware PA_SWD command 0x1f, normally sent by ISASW every timeout, 30s */
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>
#include <asm/cacheflush.h>
#include <asm/tlbflush.h>

#define ISAH_PHYS	0x1f0f1000
#define ISAH_MAGIC	0x48415349
#define GUEST_PHYS	0x1f8b0000
#define GUEST_VIRT	0xc6800000
#define GUEST_SIZE	0x30000
#define CODE_PHYS	0x10480000
#define IDENT_ATTR	0x1c0e

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

static int __init swd_param(char *s)
{
	return kstrtobool(s, &swd_enabled);
}
early_param("e6.swd", swd_param);

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
	pr_info("rapuyama-swd: call %u stage %u transport %d result %u timeout %u token %u\n",
		swd_calls, swd->stage, (s32)swd->transport, swd->result,
		timeout, swd->response >> 16);
	if (swd->stage != 60 || swd->transport != 1 || swd->result != 1 ||
	    timeout <= 30 || timeout > 3600) {
		pr_err("rapuyama-swd: secure pet failed, stopping\n");
		return;
	}
	swd->token = swd->response >> 16;
	schedule_delayed_work(&swd_work, (timeout - 30) * HZ);
}

static int __init rapuyama_swd_init(void)
{
	const u32 *h;
	u8 *guest;
	u32 *code;
	u32 token_ptr, sp, code_pa, data_pa;
	int ret = -ENODEV;

	BUILD_BUG_ON(offsetof(struct swd_call, stage) != 88);
	BUILD_BUG_ON(offsetof(struct swd_call, guest_sctlr) != 96);
	if (!swd_enabled || !of_machine_is_compatible("nokia,e6"))
		return 0;
	h = memremap(ISAH_PHYS, 0x5000, MEMREMAP_WB);
	guest = memremap(GUEST_PHYS, GUEST_SIZE, MEMREMAP_WB);
	code = memremap(CODE_PHYS, 0x70000, MEMREMAP_WB);
	if (!h || !guest || !code)
		goto out;
	/* firmware-specific bridge: reject another image before calling any code */
	if (h[0] != ISAH_MAGIC || h[1] != 1 || h[0x10] != 0xc62bb5d7 ||
	    h[0x11] != 0xc62bb5e1)
		goto out;
	if (get_unaligned_le32((u8 *)code + 0x61abe) != 0x4605b570 ||
	    get_unaligned_le32((u8 *)code + 0x619b8) != 0x4606b5f8 ||
	    get_unaligned_le32((u8 *)code + 0x62b38) != 0x4605b5fe)
		goto out;
	token_ptr = READ_ONCE(code[0x58b90 / 4]);
	if (token_ptr < GUEST_VIRT || token_ptr + 4 > GUEST_VIRT + GUEST_SIZE)
		goto out;
	/* borrow the stopped server task's stack; the ROM VA-to-PA helper uses
	 * Symbian's original page tables, so an identity Linux stack won't work */
	sp = get_unaligned_le32(guest + 0x1b7bc) & ~7U;
	if (sp < GUEST_VIRT + 0x1000 || sp > 0xc6c4c000)
		goto out;
	swd_pgd = (u32 *)__get_free_pages(GFP_KERNEL | __GFP_ZERO, 2);
	swd = (struct swd_call *)get_zeroed_page(GFP_KERNEL);
	if (!swd_pgd || !swd) {
		ret = -ENOMEM;
		goto out;
	}
	memcpy(swd_pgd, h + 0x400, 0x4000);
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
	swd->alloc_fn = 0xc6261abf;
	swd->free_fn = 0xc62619b9;
	swd->pet_fn = 0xc6262b39;
	swd->token = get_unaligned_le16(guest + token_ptr - GUEST_VIRT + 2);
	INIT_DELAYED_WORK(&swd_work, swd_ping);
	pr_info("rapuyama-swd: PA_SWD 0x1f ready, token %u guest stack %08x\n",
		swd->token, sp);
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
