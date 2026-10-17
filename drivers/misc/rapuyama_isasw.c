// SPDX-License-Identifier: GPL-2.0
/*
 * host the modem mcu firmware (SOS*ISASW) inside linux.
*/

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <asm/cacheflush.h>
#include <asm/tlbflush.h>
#include <asm/pgtable.h>

#ifdef CONFIG_ARCH_RAPUYAMA
__printf(1, 2) void rapuyama_note(const char *fmt, ...);
void rapuyama_isasw_set_irq_hook(void (*fn)(int line));
void rapuyama_isasw_set_guest_lines(const u16 *lines, unsigned int n);
void rapuyama_isasw_service_guest(void (*fn)(int line));
#else
static inline __printf(1, 2) void rapuyama_note(const char *fmt, ...) { }
static inline void rapuyama_isasw_set_irq_hook(void (*fn)(int line)) { }
static inline void rapuyama_isasw_set_guest_lines(const u16 *l, unsigned int n) { }
static inline void rapuyama_isasw_service_guest(void (*fn)(int line)) { }
#endif

/* deliver the guest's own modem lines to it so it services its ISI link (the
 * dead link is what resets the ape ~200s). gated: isasw_feedirq=1 */
static int isasw_feedirq;
module_param(isasw_feedirq, int, 0444);

static int isasw_host;
module_param(isasw_host, int, 0444);
MODULE_PARM_DESC(isasw_host, "1 = host the modem mcu firmware (unverified, see source)");

/* how often to drive the guest scheduler when no modem line is firing. karap
 * ran cb[2] on every irq incl the 1 ms tick; the MTC WD task wants 800 ms, so
 * 50 ms leaves plenty of margin */
static int isasw_period_ms = 50;
module_param(isasw_period_ms, int, 0644);

/* 0 (default) = drive the guest timers/queues but DON'T run its scheduler cb[2].
 * build #90 proved cb[2] dispatches real cellular task handlers that the linux
 * environment cannot satisfy and hard-resets the soc on the 2nd tick (the crash
 * is cb[2]'s SECOND run; the first returned clean, rgen=0). 1 = run cb[2] every
 * call (crashes ~4s). 2 = run cb[2] on the FIRST call only, then never again:
 * injects the one safe scheduler pass to see if it feeds the ~220s external
 * supervision that still resets sched=0 at the top of the stock window */
static int isasw_sched;
module_param(isasw_sched, int, 0644);

/* the isah handoff block the booter leaves for us (same as mach-rapuyama) */
#define ISAH_PHYS	0x1f0f1000u
#define ISAH_SIZE	0x5000
#define ISAH_MAGIC	0x48415349
#define ISAH_TTBR0	2	/* word index */
#define ISAH_TTBR1	3
#define ISAH_TTBCR	4
#define ISAH_DACR	5
#define ISAH_CTX	6
#define ISAH_SCTLR	7
#define ISAH_PRRR	8
#define ISAH_NMRR	9
#define ISAH_CB		0x10	/* cb[0..5] */
#define ISAH_L1		0x400	/* 4096-entry kernel L1 copy */

/* guest karap tables, as VAs valid once we are in symbian context */
#define GUEST_ISR_TBL	0xc8003340u	/* {arg, handler} per intc line */

/* guest scheduler state in its ram (c6800000 = phys 0x1f8b0000), readable from
 * linux directly without a context switch. cb[1] uses exactly these two bytes */
#define GUEST_RAM_PHYS	0x1f8b0000u
#define GUEST_RAM_SIZE	0x30000
#define G_PENDING	0x1b2e1		/* byte: non-zero = a task needs scheduling */
#define G_CURTASK	0x1b2e2		/* byte: current task id (0x30 = idle) */
/* chain-trace offsets (c6800000-relative): fiqueue ring c681b038 head+8/tail+10,
 * delayed ring c681b2c4 head+0x14/tail+0x16, MTC WD timer 0x108 block state byte
 * (DAT_c62c1a3c c681fd30 + 0x108*0x1c + 0x18 + 0x12). lets the klog show exactly
 * where the expire->post->drain->ready chain breaks if cb1 stays 0 */
#define G_FQ_HEAD	0x1b040
#define G_FQ_TAIL	0x1b042
#define G_DQ_HEAD	0x1b2d8
#define G_DQ_TAIL	0x1b2da
#define G_TMR108_ST	0x21a3a

/* os_timer expiry engine FUN_c62c1412 (thumb): reads the hw counter, expires
 * due timers by real elapsed time and posts their events. called every tick in
 * irq mode to give the guest its timebase (its own tick is a masked FIQ) */
#define TIMER_EXPIRE_FN	0xc62c1413u

/* the timer/isr post to these two rings; karap drains them from sw-irq lines
 * 32/33 (masked under linux), so we drain them ourselves each tick, in irq mode,
 * only when non-empty (the drainers fault on an empty ring). rings + drainers
 * (thumb) dumped from fiqueue.c */
#define FIQ_RING	0xc681b038u	/* head +8, tail +10 */
#define FIQ_DRAIN_FN	0xc62bb969u	/* FUN_c62bb968 */
#define DQ_RING		0xc681b2c4u	/* head +0x14, tail +0x16 */
#define DQ_DRAIN_FN	0xc62bb9d9u	/* FUN_c62bb9d8 */

/*
 * intc lines the guest binds its own isrs to, with the isr handler VA (thumb,
 * guest image, stable across boots since it loads at a fixed VA) dumped from
 * symbian's table c8003340 (arg was 0 for all of them). calling the real isr on
 * a line firing lets the guest process the incoming modem message and mark its
 * task ready, instead of only re-running the scheduler.
 */
static const struct {
	u16 line;
	u32 handler;
} guest_isr[] = {
	/* 32/33 (fiqueue/delayed drainers) are driven centrally every tick with an
	 * empty-guard, not as line isrs (their drainers fault on an empty ring) */
	{  19, 0xc62cb6db }, {  21, 0xc62cb7cf }, {  23, 0xc62cb561 },
	{  26, 0xc6208939 }, {  36, 0xc62d9b3b }, { 102, 0xc6244bc9 },
	{ 105, 0xc62449dd },
	{ 125, 0xc6207e51 }, { 126, 0xc6252131 }, { 127, 0xc62520ab },
	{ 153, 0xc62edd15 }, { 211, 0xc62edd41 }, { 246, 0xc62ca6d5 },
};

static u32 *replica;		/* our 16 kb L1, = copy of symbian's + identity */
static u32 replica_phys;
static struct timer_list isasw_timer;

/*
 * scratch shared with the asm trampoline. it lives in an identity-mapped page so
 * the trampoline can reach it both before and after the mmu switch. offsets are
 * duplicated as asm constants below - keep in sync.
 */
struct isasw_scratch {
	u32 ttbr0_lin, ttbcr_lin, dacr_lin, ctx_lin, prrr_lin, nmrr_lin, sp_lin;
	u32 ttbr0_new, ttbcr_new, dacr_new, ctx_new, prrr_new, nmrr_new, isp;
	u32 cb0, cb1, cb2, isr, isr_arg, usp;
	u32 cb1_ret, cb2_done;	/* trampoline writes these back for diagnostics */
	u32 tick, iisp;		/* os_timer expire fn + irq-mode stack */
	u32 fqptr, fqfn;	/* fiqueue ring + drainer (drain if non-empty) */
	u32 dqptr, dqfn;	/* delayed-func queue ring + drainer */
	u32 run_sched;		/* 0 = skip cb[2] (diagnostic: isolate the WD reset) */
	u32 dbg_obj;		/* in: sos_dispatcher obj (c8-vtable only maps here) */
	u32 dbg_idle, dbg_reset;	/* out: vtable[0x10]/[0x08] impl addrs */
};
#define S_TTBR0_LIN	0
#define S_TTBCR_LIN	4
#define S_DACR_LIN	8
#define S_CTX_LIN	12
#define S_PRRR_LIN	16
#define S_NMRR_LIN	20
#define S_SP_LIN	24
#define S_TTBR0_NEW	28
#define S_TTBCR_NEW	32
#define S_DACR_NEW	36
#define S_CTX_NEW	40
#define S_PRRR_NEW	44
#define S_NMRR_NEW	48
#define S_ISP		52
#define S_CB0		56
#define S_CB1		60
#define S_CB2		64
#define S_ISR		68
#define S_ISR_ARG	72
#define S_USP		76
#define S_CB1_RET	80
#define S_CB2_DONE	84
#define S_TICK		88
#define S_IISP		92
#define S_FQPTR		96
#define S_FQFN		100
#define S_DQPTR		104
#define S_DQFN		108
#define S_RUNSCHED	112
#define S_DBG_OBJ	116
#define S_DBG_IDLE	120
#define S_DBG_RESET	124

static struct isasw_scratch *scratch;	/* kernel VA */
static void *stack_pg;			/* identity stack page */
static unsigned int isasw_calls;	/* diagnostics */
static unsigned int isasw_cb2_runs;	/* how many times the guest scheduler ran */
static u8 *guest_ram;			/* 0x1f8b0000, guest scheduler state */
static u32 cb[6];
static u32 ctx_new, prrr_new, nmrr_new;

extern void rapuyama_isasw_tramp(struct isasw_scratch *s_identity);

/*
 * the trampoline. position independent: the only memory it touches is through r4
 * (the scratch, passed in r0). it runs at its own IDENTITY VA (= its phys), which
 * we map into both the current linux pgd and the replica before calling. after it
 * loads the replica TTBR0 every linux VA is gone, so it also switches sp to an
 * identity stack and takes the guest cb/isr pointers from the scratch.
 */
asm("	.text					\n"
"	.align	2				\n"
"	.global	rapuyama_isasw_tramp		\n"
"rapuyama_isasw_tramp:				\n"
"	push	{r4-r7, lr}			\n"
"	mov	r4, r0		@ r4 = scratch (identity), guest preserves r4	\n"
	/* save linux cp15 + sp */
"	mrc	p15, 0, r0, c2, c0, 0		\n"
"	str	r0, [r4, #0]			\n"	/* S_TTBR0_LIN */
"	mrc	p15, 0, r0, c2, c0, 2		\n"
"	str	r0, [r4, #4]			\n"
"	mrc	p15, 0, r0, c3, c0, 0		\n"
"	str	r0, [r4, #8]			\n"
"	mrc	p15, 0, r0, c13, c0, 1		\n"
"	str	r0, [r4, #12]			\n"
"	mrc	p15, 0, r0, c10, c2, 0		\n"
"	str	r0, [r4, #16]			\n"
"	mrc	p15, 0, r0, c10, c2, 1		\n"
"	str	r0, [r4, #20]			\n"
"	str	sp, [r4, #24]			\n"
"	mov	r0, #0				\n"
"	mcr	p15, 0, r0, c7, c10, 4	@ dsb	\n"
	/* load symbian context: attrs/ctx first, then ttbcr+ttbr0 */
"	mov	r0, #0xffffffff			\n"
"	mcr	p15, 0, r0, c3, c0, 0	@ dacr = all manager	\n"
"	ldr	r0, [r4, #44]			\n"	/* prrr_new */
"	mcr	p15, 0, r0, c10, c2, 0		\n"
"	ldr	r0, [r4, #48]			\n"	/* nmrr_new */
"	mcr	p15, 0, r0, c10, c2, 1		\n"
"	ldr	r0, [r4, #40]			\n"	/* ctx_new */
"	mcr	p15, 0, r0, c13, c0, 1		\n"
"	mov	r0, #0				\n"
"	mcr	p15, 0, r0, c7, c5, 4	@ isb	\n"
"	ldr	r0, [r4, #32]			\n"	/* ttbcr_new */
"	mcr	p15, 0, r0, c2, c0, 2		\n"
"	ldr	r0, [r4, #28]			\n"	/* ttbr0_new */
"	mcr	p15, 0, r0, c2, c0, 0		\n"
"	mov	r0, #0				\n"
"	mcr	p15, 0, r0, c7, c5, 4	@ isb	\n"
"	mcr	p15, 0, r0, c8, c7, 0	@ tlbiall	\n"
"	mcr	p15, 0, r0, c7, c5, 6	@ bpiall	\n"
"	mcr	p15, 0, r0, c7, c10, 4	@ dsb	\n"
"	mcr	p15, 0, r0, c7, c5, 4	@ isb	\n"
	/* now in symbian context: use the identity stack */
"	ldr	sp, [r4, #52]			\n"	/* isp */
	/* diag: in symbian ctx the sos_dispatcher obj (guest ram) and its vtable
	 * (c8 kernel space) are both mapped, so read the real idle (+0x10) and
	 * reset (+8) impl addrs. gated by dbg_obj (C sets it only on call #0) */
"	ldr	r0, [r4, #116]	@ dbg_obj		\n"
"	cmp	r0, #0				\n"
"	beq	4f				\n"
"	ldr	r0, [r0]	@ vtable ptr		\n"
"	cmp	r0, #0				\n"
"	beq	4f				\n"
"	ldr	r1, [r0, #0x10]	@ idle impl		\n"
"	str	r1, [r4, #120]			\n"
"	ldr	r1, [r0, #0x08]	@ reset impl		\n"
"	str	r1, [r4, #124]			\n"
"4:						\n"
	/* cb[0]: enter-irq hook */
"	ldr	r0, [r4, #56]			\n"
"	blx	r0				\n"
	/* BISECT (#86): run the os_timer expiry + isr in SVC, NOT irq mode. #80
	 * (cb0/cb1/cb2 only, svc) was stable; adding the timer call in irq mode
	 * crashed with a data abort that bad_mode'd. running it in svc means any
	 * fault is taken in svc (which linux's vectors accept) so we get a real
	 * faulting address instead of the opaque bad_mode. drains dropped for now
	 * (they need irq mode and are empty on the idle-path first calls) */
"	ldr	r0, [r4, #88]	@ tick (os_timer expire)	\n"
"	cmp	r0, #0				\n"
"	beq	1f				\n"
"	blx	r0				\n"
"1:						\n"
"	ldr	r1, [r4, #68]			\n"	/* isr */
"	cmp	r1, #0				\n"
"	beq	3f				\n"
"	ldr	r0, [r4, #72]			\n"	/* isr_arg */
"	blx	r1				\n"
"3:						\n"
	/* cb[1]: leave-irq hook, returns 1 if the scheduler must run */
"	ldr	r0, [r4, #60]			\n"
"	blx	r0				\n"
"	mov	r5, r0				\n"
"	str	r5, [r4, #80]	@ cb1_ret (diag)	\n"
"	cmp	r5, #1				\n"
"	bne	2f				\n"
"	ldr	r0, [r4, #112]	@ run_sched (0 = skip cb[2])	\n"
"	cmp	r0, #0				\n"
"	beq	2f				\n"
	/* karap runs cb[2] in undefined mode (0x1b) with its own banked stack.
	 * we match the mode + a dedicated identity stack, but keep irqs+fiqs
	 * masked (0xdb) - enabling them in foreign context is unsafe. r4/r6 are
	 * not banked so they survive the mode change; the guest preserves r4-r11 */
"	mrs	r6, cpsr			\n"
"	cpsid	if, #0x1b	@ -> UND, irq+fiq masked	\n"
"	mov	r7, sp		@ save linux sp_und	\n"
"	ldr	sp, [r4, #76]	@ UND identity stack	\n"
"	ldr	r0, [r4, #64]	@ cb[2] scheduler	\n"
"	blx	r0				\n"
"	mov	sp, r7		@ restore linux sp_und	\n"
"	msr	cpsr_c, r6	@ back to svc, original i/f	\n"
"	mov	r0, #1				\n"
"	str	r0, [r4, #84]	@ cb2_done (diag)	\n"
"2:						\n"
	/* restore linux context */
"	mov	r0, #0				\n"
"	mcr	p15, 0, r0, c7, c10, 4	@ dsb	\n"
"	ldr	r0, [r4, #8]			\n"
"	mcr	p15, 0, r0, c3, c0, 0	@ dacr	\n"
"	ldr	r0, [r4, #16]			\n"
"	mcr	p15, 0, r0, c10, c2, 0	@ prrr	\n"
"	ldr	r0, [r4, #20]			\n"
"	mcr	p15, 0, r0, c10, c2, 1	@ nmrr	\n"
"	ldr	r0, [r4, #12]			\n"
"	mcr	p15, 0, r0, c13, c0, 1	@ ctx	\n"
"	mov	r0, #0				\n"
"	mcr	p15, 0, r0, c7, c5, 4	@ isb	\n"
"	ldr	r0, [r4, #4]			\n"
"	mcr	p15, 0, r0, c2, c0, 2	@ ttbcr	\n"
"	ldr	r0, [r4, #0]			\n"
"	mcr	p15, 0, r0, c2, c0, 0	@ ttbr0	\n"
"	mov	r0, #0				\n"
"	mcr	p15, 0, r0, c7, c5, 4	@ isb	\n"
"	mcr	p15, 0, r0, c8, c7, 0	@ tlbiall	\n"
"	mcr	p15, 0, r0, c7, c5, 6	@ bpiall	\n"
"	mcr	p15, 0, r0, c7, c10, 4	@ dsb	\n"
"	mcr	p15, 0, r0, c7, c5, 4	@ isb	\n"
"	ldr	sp, [r4, #24]			\n"
"	pop	{r4-r7, pc}			\n"
"	.previous				\n");

/*
 * a hardware section descriptor for pa with attrs that are Normal cacheable +
 * executable + RW under BOTH remaps. with TEX remap on, {TEX[0],C,B} index the
 * PRRR/NMRR; index 7 ({1,1,1}) is Normal in linux's remap and in symbian's
 * (prrr 0x000aa8a4 -> idx7 = normal, nmrr 0xa840a840 -> cacheable), so the
 * trampoline is a valid memory type before AND after the switch regardless of
 * whose remap is active. bits: TEX[0]=1 (bit12), C=1 (bit3), B=1 (bit2),
 * AP=0b11 (bits[11:10], priv+user rw), XN=0, domain 0, section type (0b10)
 */
#define IDENT_SECT_ATTRS	0x1c0e

static u32 ident_section_desc(u32 pa)
{
	return (pa & 0xfff00000) | IDENT_SECT_ATTRS;
}

/* add/remove an identity section (va == pa) in a hw L1 */
static void ident_add(u32 *l1, u32 pa)
{
	l1[pa >> 20] = ident_section_desc(pa);
}

static void ident_del(u32 *l1, u32 pa)
{
	l1[pa >> 20] = 0;
}

static void isasw_call(u32 isr, u32 isr_arg)
{
	u32 *lin_pgd;
	u32 ttbr0, tramp_pa, scratch_pa, stack_pa, f;
	unsigned long flags;

	scratch->ttbr0_new = replica_phys | 0x18;	/* attrs from symbian ttbr1 */
	scratch->ttbcr_new = 0;				/* N=0: whole 4 GB via TTBR0 */
	scratch->ctx_new = ctx_new;
	scratch->prrr_new = prrr_new;
	scratch->nmrr_new = nmrr_new;
	/* three identity stacks in the 32 kb stack region, well separated: svc
	 * (cb0/cb1) at the top, irq (timer+isr) in the middle, und (cb2) lower */
	scratch->isp = (u32)virt_to_phys(stack_pg) + 0x8000 - 16;
	scratch->iisp = (u32)virt_to_phys(stack_pg) + 0x5000 - 16;
	scratch->usp = (u32)virt_to_phys(stack_pg) + 0x2800 - 16;
	scratch->tick = TIMER_EXPIRE_FN;
	scratch->fqptr = FIQ_RING;
	scratch->fqfn = FIQ_DRAIN_FN;
	scratch->dqptr = DQ_RING;
	scratch->dqfn = DQ_DRAIN_FN;
	/* sched=2: one safe cb[2] pass on the first call only, then never (the 2nd
	 * run is the #90 crash); otherwise run_sched = isasw_sched as-is */
	scratch->run_sched = (isasw_sched == 2) ? (isasw_calls == 0) : isasw_sched;
	/* read the sos_dispatcher idle/reset impls once, in symbian ctx (c8 vtable
	 * is unmapped under linux). obj ptr = *DAT_c62efd5c, statically 0xc68029c0 */
	scratch->dbg_obj = (isasw_calls == 0) ? 0xc68029c0 : 0;
	scratch->cb0 = cb[0];
	scratch->cb1 = cb[1];
	scratch->cb2 = cb[2];
	scratch->isr = isr;
	scratch->isr_arg = isr_arg;
	scratch->cb1_ret = 0xffffffff;
	scratch->cb2_done = 0;

	tramp_pa = (u32)virt_to_phys(rapuyama_isasw_tramp);
	scratch_pa = (u32)virt_to_phys(scratch);
	stack_pa = (u32)virt_to_phys(stack_pg);

	local_irq_save(flags);

	/* identity-map the trampoline, its scratch and its stack in the live pgd
	 * so the branch into identity space resolves; the replica already has them */
	asm volatile("mrc p15, 0, %0, c2, c0, 0" : "=r"(ttbr0));
	lin_pgd = phys_to_virt(ttbr0 & ~0x3fffu);
	ident_add(lin_pgd, tramp_pa);
	ident_add(lin_pgd, scratch_pa);
	ident_add(lin_pgd, stack_pa);
	flush_cache_all();
	local_flush_tlb_all();

	/* a "pre" marker before the switch: if the klog ends here, the first mmu
	 * switch into symbian context faulted (see the post-call note below) */
	if (isasw_calls < 3)
		rapuyama_note("isasw: pre call #%u, switching mmu to symbian ctx\n",
			      isasw_calls + 1);

	/* call the trampoline AT its identity VA, with the identity scratch ptr */
	f = tramp_pa;
	((void (*)(struct isasw_scratch *))(uintptr_t)f)
		((struct isasw_scratch *)(uintptr_t)scratch_pa);

	ident_del(lin_pgd, tramp_pa);
	ident_del(lin_pgd, scratch_pa);
	ident_del(lin_pgd, stack_pa);
	flush_cache_all();
	local_flush_tlb_all();

	local_irq_restore(flags);

	/*
	 * diagnostics so the FIRST hardware flash is conclusive (no flash-and-retry
	 * loop): the "pre" line is written before the mmu switch, so if the klog
	 * (ramcon survives the reset) ends at "pre" the first switch itself faulted
	 * (attrs/trampoline); if it reaches "post" the switch + guest calls
	 * returned, and cb1_ret / cb2_done say how far the guest got.
	 */
	if (scratch->dbg_obj)	/* was call #0: report the resolved impls */
		rapuyama_note("isasw: sisa impl idle=%08x reset=%08x\n",
			      scratch->dbg_idle, scratch->dbg_reset);
	isasw_calls++;
	if (scratch->cb2_done)
		isasw_cb2_runs++;
	if (isasw_calls <= 3 || (isasw_calls % 100) == 0) {
		u16 fqh = 0, fqt = 0, dqh = 0, dqt = 0;
		u8 t108 = 0xff;

		if (guest_ram) {
			fqh = *(u16 *)(guest_ram + G_FQ_HEAD);
			fqt = *(u16 *)(guest_ram + G_FQ_TAIL);
			dqh = *(u16 *)(guest_ram + G_DQ_HEAD);
			dqt = *(u16 *)(guest_ram + G_DQ_TAIL);
			t108 = guest_ram[G_TMR108_ST];
		}
		rapuyama_note("isasw: %us post #%u cb1=%u cb2=%u sched=%u pend=%u task=%02x fq=%u/%u dq=%u/%u t108=%02x rst=%02x rgen=%u\n",
			      jiffies_to_msecs(jiffies) / 1000, isasw_calls,
			      scratch->cb1_ret, scratch->cb2_done, isasw_cb2_runs,
			      guest_ram ? guest_ram[G_PENDING] : 0xff,
			      guest_ram ? guest_ram[G_CURTASK] : 0xff,
			      fqh, fqt, dqh, dqt, t108,
			      /* guest hw_reset_generate: reason byte c680187c, flag c68265e0 */
			      guest_ram ? guest_ram[0x187c] : 0xff,
			      guest_ram ? guest_ram[0x265e0] : 0xff);
	}
}

/* a guest line fired (called from the irqchip, irqs already off). run its isr +
 * the scheduler in guest context. the handler/arg come from symbian's table, so
 * read them inside the switch is cleanest - but we only have the pointer in guest
 * VA space, so pass the line and let the trampoline-side... no: resolve here from
 * the table VA is impossible in linux ctx. instead we rely on the scheduler path
 * (cb[2]) which re-drives all ready tasks; the per-line isr is best-effort via
 * the known handler table. keep it simple: just drive the scheduler. */
static void isasw_irq_hook(int line)
{
	int i;

	if (!replica)
		return;
	for (i = 0; i < ARRAY_SIZE(guest_isr); i++)
		if (guest_isr[i].line == line) {
			/* run the guest's own isr for this line (then cb[1]/cb[2]
			 * process whatever it woke); arg is 0 for every guest line */
			isasw_call(guest_isr[i].handler, 0);
			return;
		}
}

static void isasw_tick(struct timer_list *t)
{
	isasw_call(0, 0);
	/* service any modem lines that fired, here in timer ctx (safe), running
	 * each guest isr via isasw_irq_hook; this feeds ISASW its own link */
	if (isasw_feedirq)
		rapuyama_isasw_service_guest(isasw_irq_hook);
	mod_timer(&isasw_timer, jiffies + msecs_to_jiffies(isasw_period_ms));
}

static int __init rapuyama_isasw_init(void)
{
	const u32 *h;
	int i;

	if (!isasw_host)
		return 0;

	h = memremap(ISAH_PHYS, ISAH_SIZE, MEMREMAP_WB);
	if (!h || h[0] != ISAH_MAGIC) {
		rapuyama_note("isasw: no handoff block, not hosting\n");
		if (h)
			memunmap((void *)h);
		return 0;
	}

	/* replica L1 = symbian's saved 16 kb kernel L1 (4096 entries). built once;
	 * since it is a verbatim copy, every guest VA (c6/c8/f0) and the rom resolve
	 * exactly as they did under symbian */
	replica = (u32 *)__get_free_pages(GFP_KERNEL, 2);	/* 16 kb, 16 kb aligned */
	scratch = kzalloc(sizeof(*scratch), GFP_KERNEL);
	stack_pg = (void *)__get_free_pages(GFP_KERNEL, 3);	/* 32 kb: svc+irq+und stacks */
	if (!replica || !scratch || !stack_pg) {
		rapuyama_note("isasw: alloc failed\n");
		goto out;
	}
	for (i = 0; i < 4096; i++)
		replica[i] = h[ISAH_L1 + i];
	replica_phys = (u32)virt_to_phys(replica);

	for (i = 0; i < 6; i++)
		cb[i] = h[ISAH_CB + i];
	ctx_new = h[ISAH_CTX];
	prrr_new = h[ISAH_PRRR];
	nmrr_new = h[ISAH_NMRR];

	/* for reading the guest scheduler state (pending/curtask) in the log */
	guest_ram = memremap(GUEST_RAM_PHYS, GUEST_RAM_SIZE, MEMREMAP_WB);

	/* pre-map the trampoline/scratch/stack identity in the replica too */
	ident_add(replica, (u32)virt_to_phys(rapuyama_isasw_tramp));
	ident_add(replica, (u32)virt_to_phys(scratch));
	ident_add(replica, (u32)virt_to_phys(stack_pg));
	flush_cache_all();

	rapuyama_note("isasw: hosting ON. cb0=%08x cb1=%08x cb2=%08x ctx=%08x replica=%08x period=%d ms\n",
		      cb[0], cb[1], cb[2], ctx_new, replica_phys, isasw_period_ms);
	rapuyama_note("isasw: UNVERIFIED - watch for a fault right after the first call\n");

	/*
	 * one-time: resolve the bse_sisa "sos_dispatcher" vtable so its real idle
	 * (+0x10, FUN_c62efa04 = mtc_sisa_idle_loop) and reset (+8) impl addresses
	 * can be decompiled offline from the rom image. DAT_c62efd5c (rom literal)
	 * = 0xc68029c0 = the dispatcher object in guest ram; its first word is the
	 * vtable VA. the idle handler is the candidate APE->companion-core heartbeat
	 * that stops when cb[2] is off and may be the ~220s external reset we see.
	 */
	if (guest_ram) {
		u32 vt = *(u32 *)(guest_ram + 0x29c0);	/* *(c68029c0) */
		u32 idle = 0, rst = 0;

		if (vt >= 0xc6800000 && vt < 0xc6800000 + GUEST_RAM_SIZE) {
			idle = *(u32 *)(guest_ram + (vt - 0xc6800000) + 0x10);
			rst = *(u32 *)(guest_ram + (vt - 0xc6800000) + 0x08);
		}
		rapuyama_note("isasw: sisa disp vt=%08x idle=%08x reset=%08x\n",
			      vt, idle, rst);
	}

	memunmap((void *)h);

	if (isasw_feedirq) {
		u16 lines[ARRAY_SIZE(guest_isr)];
		int i;

		/* register + unmask the guest lines; they are serviced from the timer
		 * (isasw_tick) via rapuyama_isasw_service_guest, NOT from hard irq.
		 * do NOT install the hard-irq hook here - running the trampoline from
		 * the intc handler is unsafe (that was the #103 instant crash) */
		for (i = 0; i < ARRAY_SIZE(guest_isr); i++)
			lines[i] = guest_isr[i].line;
		rapuyama_isasw_set_guest_lines(lines, ARRAY_SIZE(guest_isr));
		rapuyama_note("isasw: feeding %u guest modem lines (deferred to timer)\n",
			      (unsigned)ARRAY_SIZE(guest_isr));
	} else {
		rapuyama_isasw_set_irq_hook(isasw_irq_hook);
	}
	timer_setup(&isasw_timer, isasw_tick, 0);
	/* give the rest of boot a moment, then start driving the guest */
	mod_timer(&isasw_timer, jiffies + msecs_to_jiffies(2000));
	return 0;
out:
	kfree(scratch);
	if (replica)
		free_pages((unsigned long)replica, 2);
	if (stack_pg)
		free_pages((unsigned long)stack_pg, 3);
	memunmap((void *)h);
	return 0;
}
late_initcall(rapuyama_isasw_init);

MODULE_DESCRIPTION("Nokia RapuYama modem-firmware (ISASW) host");
MODULE_LICENSE("GPL");