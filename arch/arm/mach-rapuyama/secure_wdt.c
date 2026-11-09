// SPDX-License-Identifier: GPL-2.0
/* E6 firmware PA_SWD command 0x1f, normally sent by ISASW every timeout, 30s */
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/unaligned.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <uapi/linux/rapuyama_simlock.h>
#include <asm/cacheflush.h>
#include <asm/tlbflush.h>
#include "secure_heap.h"

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
	u32 request_size;
};

struct simlock_call {
	struct swd_call call;
	u8 request[0x1c8];
	u8 data[];
};

extern void rapuyama_swd_call(struct swd_call *call);
extern void rapuyama_note(const char *fmt, ...);

static bool swd_enabled;
static u32 *swd_pgd;
static struct swd_call *swd;
static u8 *swd_guest, *swd_heap;
static struct delayed_work swd_work;
static u32 swd_calls;
static DEFINE_MUTEX(secure_lock);

static int __init swd_param(char *s)
{
	return kstrtobool(s, &swd_enabled);
}
early_param("e6.swd", swd_param);

static int secure_call(struct swd_call *call)
{
	u32 *pgd;
	u32 ttbr0, ttbcr, code_pa, data_pa, ci, di, old_code, old_data, old_guest_data;
	unsigned long flags;
	unsigned int request_size = call->request_size;
	unsigned int response_size = 8;

	mutex_lock(&secure_lock);
	if (request_size == 0x1c8) {
		request_size += get_unaligned_le16((u8 *)call + 104 + 0x1c2);
		response_size = 0x1c;
	}
	if (!secure_heap_room(swd_heap, get_unaligned_le32(swd_guest + 0x19080),
			      request_size, response_size)) {
		mutex_unlock(&secure_lock);
		return -ENOMEM;
	}
	code_pa = (u32)virt_to_phys(rapuyama_swd_call);
	data_pa = (u32)virt_to_phys(call);
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
		mutex_unlock(&secure_lock);
		return -EINVAL;
	}
	pgd = phys_to_virt(ttbr0 & ~0x3fffU);
	old_code = pgd[ci];
	old_data = pgd[di];
	old_guest_data = swd_pgd[di];
	swd_pgd[di] = (data_pa & 0xfff00000) | IDENT_ATTR;
	pgd[ci] = (code_pa & 0xfff00000) | IDENT_ATTR;
	pgd[di] = (data_pa & 0xfff00000) | IDENT_ATTR;
	flush_cache_all();
	local_flush_tlb_all();
	call->stage = 0;
	call->transport = ~0U;
	call->result = ~0U;
	rapuyama_note("secure: entering original wrapper %08x\n", call->pet_fn);
	((void (*)(struct swd_call *))(uintptr_t)code_pa)
		((struct swd_call *)(uintptr_t)data_pa);
	pgd[ci] = old_code;
	pgd[di] = old_data;
	swd_pgd[di] = old_guest_data;
	flush_cache_all();
	local_flush_tlb_all();
	local_irq_restore(flags);
	preempt_enable();
	mutex_unlock(&secure_lock);
	return call->stage == 60 ? 0 : -EIO;
}

static ssize_t devlock_read(struct file *file, char __user *buffer,
			   size_t size, loff_t *offset)
{
	struct swd_call *call;
	u32 reply[3];
	int ret;

	if (size != sizeof(reply))
		return -EINVAL;
	call = (struct swd_call *)get_zeroed_page(GFP_KERNEL);
	if (!call)
		return -ENOMEM;
	*call = *swd;
	call->pet_fn = 0xc6263983;
	call->request_size = 12;
	call->token = 0;
	ret = secure_call(call);
	reply[0] = call->transport;
	reply[1] = call->result;
	reply[2] = call->response;
	free_page((unsigned long)call);
	if (ret)
		return ret;
	return copy_to_user(buffer, reply, sizeof(reply)) ? -EFAULT : sizeof(reply);
}

static const struct file_operations devlock_fops = {
	.open = nonseekable_open,
	.read = devlock_read,
};
static struct miscdevice devlock_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "rapuyama-devlock",
	.fops = &devlock_fops,
	.mode = 0400,
};

static long simlock_ioctl(struct file *file, unsigned int command, unsigned long arg)
{
	struct rapuyama_simlock query;
	struct simlock_call *check;
	unsigned int order, size;
	int ret;

	if (command != RAPUYAMA_SIMLOCK_CHECK)
		return -ENOTTY;
	if (copy_from_user(&query, (void __user *)arg, sizeof(query)))
		return -EFAULT;
	if (query.data_length > 0xffff || query.request[2] != 1 ||
	    get_unaligned_le16(query.request + 0x1c2) != query.data_length ||
	    get_unaligned_le32(query.request + 0x1c4))
		return -EINVAL;
	size = sizeof(*check) + query.data_length + sizeof(query.response);
	order = get_order(size);
	/* One aligned allocation keeps the entire bounce buffer in one section. */
	check = (void *)__get_free_pages(GFP_KERNEL | __GFP_ZERO, order);
	if (!check)
		return -ENOMEM;
	check->call = *swd;
	check->call.pet_fn = 0xc6263fc9;
	check->call.request_size = sizeof(check->request);
	memcpy(check->request, query.request, sizeof(check->request));
	if (copy_from_user(check->data, u64_to_user_ptr(query.data), query.data_length)) {
		ret = -EFAULT;
		goto out;
	}
	ret = secure_call(&check->call);
	if (ret)
		goto out;
	query.transport = check->call.transport;
	memcpy(query.response, check->data + query.data_length, sizeof(query.response));
	ret = copy_to_user((void __user *)arg, &query, sizeof(query)) ? -EFAULT : 0;
out:
	memzero_explicit(check, size);
	free_pages((unsigned long)check, order);
	return ret;
}

static const struct file_operations simlock_fops = {
	.open = nonseekable_open,
	.unlocked_ioctl = simlock_ioctl,
};
static struct miscdevice simlock_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "rapuyama-simlock",
	.fops = &simlock_fops,
	.mode = 0400,
};

static void swd_ping(struct work_struct *work)
{
	u32 timeout;

	if (secure_call(swd)) {
		pr_err("rapuyama-swd: secure call failed, stopping\n");
		return;
	}

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
	BUILD_BUG_ON(offsetof(struct swd_call, request_size) != 100);
	BUILD_BUG_ON(offsetof(struct simlock_call, request) != 104);
	BUILD_BUG_ON(offsetof(struct simlock_call, data) != 104 + 0x1c8);
	if (!swd_enabled || !of_machine_is_compatible("nokia,e6"))
		return 0;
	h = memremap(ISAH_PHYS, 0x5000, MEMREMAP_WB);
	guest = memremap(GUEST_PHYS, GUEST_SIZE, MEMREMAP_WB);
	swd_heap = memremap(0x1ffe0000, 0x8000, MEMREMAP_WB);
	code = memremap(CODE_PHYS, 0x70000, MEMREMAP_WB);
	if (!h || !guest || !code || !swd_heap)
		goto out;
	/* firmware-specific bridge: reject another image before calling any code */
	if (h[0] != ISAH_MAGIC || h[1] != 1 || h[0x10] != 0xc62bb5d7 ||
	    h[0x11] != 0xc62bb5e1)
		goto out;
	if (get_unaligned_le32((u8 *)code + 0x61abe) != 0x4605b570 ||
	    get_unaligned_le32((u8 *)code + 0x619b8) != 0x4606b5f8 ||
	    get_unaligned_le32((u8 *)code + 0x62b38) != 0x4605b5fe ||
	    get_unaligned_le32((u8 *)code + 0x63982) != 0x4605b5fe ||
	    get_unaligned_le32((u8 *)code + 0x63986) != 0x4c6d460e ||
	    get_unaligned_le32((u8 *)code + 0x63fc8) != 0xb085b5f3 ||
	    get_unaligned_le32((u8 *)code + 0x63fcc) != 0x48bf4605)
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
	swd->request_size = 8;
	swd_guest = guest;
	swd->token = get_unaligned_le16(guest + token_ptr - GUEST_VIRT + 2);
	INIT_DELAYED_WORK(&swd_work, swd_ping);
	pr_info("rapuyama-swd: PA_SWD 0x1f ready, token %u guest stack %08x\n",
		swd->token, sp);
	schedule_delayed_work(&swd_work, 5 * HZ);
	if (misc_register(&devlock_device))
		pr_err("rapuyama-swd: device-lock query unavailable\n");
	if (misc_register(&simlock_device))
		pr_err("rapuyama-swd: SIM-lock check unavailable\n");
	ret = 0;
out:
	if (h)
		memunmap((void *)h);
	if (guest && ret)
		memunmap(guest);
	if (code)
		memunmap(code);
	if (ret) {
		if (swd_heap)
			memunmap(swd_heap);
		if (swd_pgd)
			free_pages((unsigned long)swd_pgd, 2);
		if (swd)
			free_page((unsigned long)swd);
		pr_err("rapuyama-swd: firmware setup rejected (%d)\n", ret);
	}
	return ret;
}
late_initcall(rapuyama_swd_init);
