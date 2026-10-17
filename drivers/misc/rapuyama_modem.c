// SPDX-License-Identifier: GPL-2.0
/*
 * Nokia RapuYama modem (CMT) link driver - full port of the Symbian modem
 * transport, reverse-engineered from karap.dll + ecust.dll + empmIsaIf.dll +
 * isaaccessextension.dll + isimessagekernel.dll.
 *
 * GOAL: keep the modem's APE-link serviced so the modem (a separate CPU that is
 * the SoC reset master) does not reset the whole chip ~50-210 s after Linux
 * takes over from Symbian.
 *
 * ---- root cause (converged) ----
 * The modem resets the APE when the APE stops sending it Energy-Management (EM)
 * power-state messages. Symbian's empmIsaIf.dll streams these on every idle/wake
 * transition (resource 0xA9); bare Linux never does, so after the modem's
 * tolerance window (~50-210 s) it resets the APE. The fix is to periodically
 * send the modem the EM "awake" message, exactly as empmIsaIf FUN_801012ec
 * state 1 builds it, through the modem TX byte FIFO - the same path the ASIC
 * send leaf FUN_800beac8 uses.
 *
 * ---- transport (ecust FUN_800beac8, the whole hardware effect) ----
 *   port = 0x40100000 + chan*0x1000;   // TX FIFO byte port (virt 0xc6f00000)
 *   *port = 0xc0;                       // frame start
 *   for each message byte: *port = byte;
 *   *port = 0x00;                       // frame end
 *   *(u8 *)0x401fd800 = (u8)chan;       // doorbell (virt 0xc6ffd800)
 * It is a byte FIFO: always write the one port address, never port+i. The
 * trailing doorbell write is what tells the modem a frame is queued - the prior
 * irqchip attempt omitted it, which is why it never worked.
 *
 * ---- the EM "awake" message (empmIsaIf FUN_801012ec state 1), 0x18 bytes ----
 *   [3]=0x47 (resource/group)  [4]=0x12 [5]=0x00 (ISI length = 0x18-6, little
 *   endian as FUN_8022c548 writes it)  [9]=0x04 (msg id)
 *   [0xa..0xf]=02 01 05 0c 04 04  [0x10]=EM-state byte
 *   [0x14..0x17]=(counter + 0x111) big-endian.
 * Header bytes [0,1,2,6,7,8] (media/device/object/transaction) are added by the
 * IAD routing in the firmware and are not pinned statically; 0 is the
 * best-effort start, refine from an e6hook.sis capture if the modem rejects it.
 *
 * ---- modem interface clock (ecust FUN_800bede4 cold path) ----
 * The FIFO data domain external-aborts until the modem interface is brought up
 * (reading the RX FIFO 0x20500000 proved this). FUN_800bede4 does it; we port
 * the targeted subset (clock mux FUN_800bec88(3,0xf,2) + the 0x20033xxx control
 * writes + the per-channel resets, channel 0 skipped). Gated by iface_init=1;
 * set iface_init=0 to test whether the domain is already open (clk_ignore_unused
 * leaves Symbian's clocks on). A prior full ecust cold-init broke USB, so this
 * subset is deliberately narrow - watch for a USB regression on the first run.
 *
 * ---- line 0x1e (30) ----
 * The modem also fires INTC line 30 (~30 s) as a status IRQ. The irqchip owns
 * the INTC-level ack (irq_rapuyama.modem_link=1); this driver does not need to
 * receive line 30 for the keepalive, so the DT interrupt is optional. The SISA
 * ring (Layer 0/1) is kept for a future two-way ISI stack but is dormant.
 */

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/timer.h>

#ifdef CONFIG_ARCH_RAPUYAMA
void rapuyama_note(const char *fmt, ...);
#else
static inline void rapuyama_note(const char *fmt, ...) { }
#endif

/* reg[0] modem control region @ 0x20040000 (swirq + per-channel resets) */
#define SWIRQ_STATUS		0x280		/* bit 0x1000 = line 0x1e pending */
#define SWIRQ_SET		0x284		/* write 0x1000 to raise line 0x1e */
#define SWIRQ_CLEAR		0x288		/* write 0x1000 to ack line 0x1e */
#define SWIRQ_BIT		0x1000
#define CHAN_RESET_BASE		0x780		/* per-ch: +ch*0x400 + 0x488 (FUN_800bede4) */

#define MODEM_LINE		0x1e		/* INTC line 30 */

/* reg[3] clock controller @ 0x20032000 (modem-interface bring-up) */
#define CLK_MUX_SEL3		0x644		/* FUN_800bec88 sel=3 (DAT_800befe0) */
#define CLK_MUX_TRIG		0x64c		/* sel|sel>>0x1c => +8 (0x2003264c) */
#define CLK_MUX_VAL		0x9480		/* DAT_800befe4 */
#define CLK_MUX_PULSE		0xd400
#define CLK_MUX_READY		0x10		/* poll bit on the trigger reg */
#define CLK_C010		0x1010		/* 0x20033010 = 0x3494  */
#define CLK_C034		0x1034		/* 0x20033034 = 0x6d00  */
#define CLK_C084		0x1084		/* 0x20033084 = 8       */
#define CLK_C088		0x1088		/* 0x20033088 = 0x8000  */
#define CLK_C0B4		0x10b4		/* 0x200330b4 = 8       */
#define CLK_C0B8		0x10b8		/* 0x200330b8 = 1       */
#define CLK_C0D8		0x10d8		/* 0x200330d8 = 0xbfff  */
#define CLK_C198		0x1198		/* 0x20033198 = 0x200   */

/* EM keepalive message layout (empmIsaIf FUN_801012ec state 1) */
#define EM_MSG_LEN		0x18
#define EM_CHAN			0		/* ch0 = EM/control (skip byte = 0) */

/*
 * Supervision mailbox (phys 0x20040180 TX / 0x580 RX / 0x780/0x784 doorbells).
 * PROVEN the live modem channel: e6beac.sis caught 0 sends on the ASIC FIFO
 * (FUN_800beac8/0x40100000) in 4 min, while modwatch showed this mailbox cycling
 * every ~24 s. The APE's fixed TX word burst, captured from live Symbian
 * (logs/modwatch.log), bracketed by the 0x55555555/0xaaaaaaaa sync markers.
 * Driven continuously (no long idle) so the modem always sees the APE active.
 */
#define MB_TX			0x180	/* APE->modem word */
#define MB_RX			0x580	/* modem->APE word */
#define MB_DB0			0x780	/* APE doorbell */
#define MB_DB1			0x784	/* modem doorbell */
static const u32 mb_cycle[] = {
	0x55555555, 0xaaaaaaaa, 0x00000001, 0x0000026c, 0x0001000b,
	0x00000004, 0x0000000c, 0x00000020, 0x00000008, 0x00000004,
};
#define MB_POLL_MS		10

#define RING_ENTRIES		0x50		/* firmware FUN_80104200: 0x50 slots */

static int iface_init = 1;
module_param(iface_init, int, 0444);
MODULE_PARM_DESC(iface_init, "bring up the modem interface clock (ecust FUN_800bede4 subset)");

/* rapuyama_modem.enable=0 makes probe bail without touching the modem, so a run
 * is a neutral measurement of the reset (nothing pokes the CMT). default off
 * since 17.10: the mailbox/fifo here are not the modem link (see the handoff) */
static int enable = 0;
module_param(enable, int, 0444);
MODULE_PARM_DESC(enable, "0 = do not touch the modem at all (neutral measurement run)");

/* one deferred-call entry, 0x1c bytes, matching the firmware slot layout */
struct sisa_entry {
	u32 fn;		/* +0x00 callback (0 = empty slot)      */
	u32 a1;		/* +0x04 */
	u32 a2;		/* +0x08 */
	u32 a3;		/* +0x0c */
	u32 a4;		/* +0x10 */
	u32 result;	/* +0x14 (0xffffffed = pending)         */
	u32 sync;	/* +0x18 (0 = async, 1 = sync w/ result) */
} __packed;

struct sisa_ring {
	struct sisa_entry *base;
	u16 tail;	/* read index (ISR drains here)  */
	u16 head;	/* write index (post here)       */
	u16 size;	/* number of slots               */
	u16 count;	/* pending entries               */
};

struct rapuyama_modem {
	struct device *dev;
	void __iomem *ctrl;	/* reg[0] 0x20040000 block     */
	void __iomem *tx;	/* reg[1] 0x40100000 TX FIFO   */
	void __iomem *door;	/* reg[2] 0x401fd800 doorbell  */
	void __iomem *clk;	/* reg[3] 0x20032000 clock ctl */
	int irq;		/* virq of INTC line 30 (optional) */
	struct sisa_ring ring;
	struct timer_list em_timer;
	int mb_idx;		/* position in mb_cycle[] */
	int mb_max;		/* furthest step reached (progress) */
	u32 mb_last_rx;		/* last modem word seen */
	u32 mb_tick;		/* poll counter for throttled logging */
	spinlock_t lock;	/* guards the ring + swirq + mailbox */
};

static struct rapuyama_modem *g_modem;

/* --- Layer 2: modem interface clock mux (port of ecust FUN_800bec88 sel=3) --- */
static void clk_mux_enable(struct rapuyama_modem *m)
{
	int i;

	/* *sel = (b & 0x1f) | ((a & 0xffff07ff) << 5), a=0xf b=2 => 0x1e2 */
	writel_relaxed((2 & 0x1f) | ((0xf & 0xffff07ffu) << 5), m->clk + CLK_MUX_SEL3);
	udelay(1);
	writel_relaxed(CLK_MUX_VAL, m->clk + CLK_MUX_TRIG);
	udelay(1);
	writel_relaxed(CLK_MUX_PULSE, m->clk + CLK_MUX_TRIG);
	udelay(1);
	writel_relaxed(CLK_MUX_VAL, m->clk + CLK_MUX_TRIG);
	/* spin for the ready bit, bounded (firmware spins ~DAT_800befe8 times) */
	for (i = 0; i < 1000; i++) {
		if (readl_relaxed(m->clk + CLK_MUX_TRIG) & CLK_MUX_READY)
			break;
		udelay(1);
	}
}

/* --- Layer 2: bring up the modem interface (port of FUN_800bede4 cold path) --- */
static void modem_iface_up(struct rapuyama_modem *m)
{
	int ch;

	clk_mux_enable(m);

	writel_relaxed(0xbfff, m->clk + CLK_C0D8);
	writel_relaxed(0x3494, m->clk + CLK_C010);
	writel_relaxed(0x6d00, m->clk + CLK_C034);
	writel_relaxed(0x8000, m->clk + CLK_C088);

	clk_mux_enable(m);

	writel_relaxed(1, m->clk + CLK_C0B8);
	writel_relaxed(0x200, m->clk + CLK_C198);
	writel_relaxed(8, m->clk + CLK_C084);
	writel_relaxed(8, m->clk + CLK_C0B4);

	/* per-channel reset: channels 1..5 (the skip table is {0} => ch0 kept) */
	for (ch = 1; ch < 6; ch++)
		writel_relaxed(0, m->ctrl + CHAN_RESET_BASE + ch * 0x400 + 0x488);

	/*
	 * mailbox channel init, full port of karap FUN_800b8dc8: configures the
	 * APE<->modem mailbox so the modem leaves its 0x55555555 sync state.
	 * ctrl=0x20040000, clk(sysctrl)=0x20032000. The garbled odd-address write
	 * in the decomp (orr r0,r0,r0,lsr#18) is a disasm artifact, skipped.
	 */
	writel_relaxed(1, m->ctrl + 0x184);	/* 0x20040184 mailbox TX ch */
	writel_relaxed(6, m->ctrl + 0x188);	/* 0x20040188 */
	writel_relaxed(1, m->clk + 0x1604);	/* 0x20033604 sysctrl */
	writel_relaxed(3, m->clk + 0x1608);	/* 0x20033608 */
	writel_relaxed(0, m->clk + 0x1620);	/* 0x20033620..0x30 cleared */
	writel_relaxed(0, m->clk + 0x1624);
	writel_relaxed(0, m->clk + 0x1628);
	writel_relaxed(0, m->clk + 0x162c);
	writel_relaxed(0, m->clk + 0x1630);

	rapuyama_note("e6: modem iface clock + mailbox brought up\n");
}

/*
 * Supervision mailbox handshake (VideoCore-style command/response). The modem
 * drives its word on 0x580; we must present the matching APE word on 0x180 and
 * toggle the doorbell so it advances - spamming tx blindly (build #64) left the
 * modem crawling one word per ~30 s and it reset. Here we FOLLOW the modem:
 * whenever its 0x580 word changes, step our fixed tx sequence, present the next
 * word, assert our doorbell (0x780) and ack the modem's (0x784). Progress is
 * logged every ~10 s so a flash shows how far the handshake gets.
 */
static void em_keepalive(struct timer_list *t)
{
	struct rapuyama_modem *m = from_timer(m, t, em_timer);
	unsigned long flags;
	u32 rx;

	spin_lock_irqsave(&m->lock, flags);
	rx = readl_relaxed(m->ctrl + MB_RX);
	if (rx != m->mb_last_rx) {		/* modem advanced -> respond */
		m->mb_last_rx = rx;
		m->mb_idx = (m->mb_idx + 1) % ARRAY_SIZE(mb_cycle);
		writel_relaxed(mb_cycle[m->mb_idx], m->ctrl + MB_TX);
		writel_relaxed(0xffffffff, m->ctrl + MB_DB0);	/* APE word ready */
		writel_relaxed(0, m->ctrl + MB_DB1);		/* ack modem doorbell */
		if (m->mb_idx > m->mb_max)
			m->mb_max = m->mb_idx;
	}
	if (++m->mb_tick >= 1000) {		/* ~10 s at 10 ms */
		m->mb_tick = 0;
		rapuyama_note("e6: mb rx=%08x idx=%d max=%d db=%08x/%08x\n",
			      rx, m->mb_idx, m->mb_max,
			      readl_relaxed(m->ctrl + MB_DB0),
			      readl_relaxed(m->ctrl + MB_DB1));
	}
	spin_unlock_irqrestore(&m->lock, flags);

	mod_timer(&m->em_timer, jiffies + msecs_to_jiffies(MB_POLL_MS));
}

/* --- Layer 1: ring drain (port of FUN_801040cc + the FUN_80104194 loop) --- */
static void sisa_drain(struct rapuyama_modem *m)
{
	struct sisa_ring *r = &m->ring;

	while (r->count) {
		struct sisa_entry *e = &r->base[r->tail];

		/*
		 * Our ring only ever holds Linux callbacks; until Layer 3
		 * registers any, fn is 0 and we just consume the slot. Symbian
		 * ROM code cannot be called safely from Linux.
		 */
		if (e->fn) {
			void (*cb)(u32, u32, u32, u32) = (void *)(uintptr_t)e->fn;

			cb(e->a1, e->a2, e->a3, e->a4);
		}
		e->fn = 0;
		r->tail = (r->tail + 1) % r->size;
		r->count--;
	}
}

/* --- Layer 1: post a deferred call + raise line 0x1e (port of FUN_8010445c) --- */
static int __maybe_unused sisa_post(struct rapuyama_modem *m, u32 fn,
				    u32 a1, u32 a2, u32 a3, u32 a4)
{
	struct sisa_ring *r = &m->ring;
	unsigned long flags;

	spin_lock_irqsave(&m->lock, flags);
	if (r->count >= r->size) {
		spin_unlock_irqrestore(&m->lock, flags);
		return -ENOSPC;
	}
	r->base[r->head] = (struct sisa_entry){ .fn = fn, .a1 = a1, .a2 = a2,
						.a3 = a3, .a4 = a4,
						.result = 0xffffffed, .sync = 0 };
	r->head = (r->head + 1) % r->size;
	r->count++;
	/* raise line 0x1e: clear then set pending (karap FUN_800b8260/FUN_800b8234) */
	writel_relaxed(SWIRQ_BIT, m->ctrl + SWIRQ_CLEAR);
	writel_relaxed(SWIRQ_BIT, m->ctrl + SWIRQ_SET);
	spin_unlock_irqrestore(&m->lock, flags);
	return 0;
}

/* --- Layer 0: the line-30 ISR (port of FUN_80104194) --- */
static irqreturn_t rapuyama_modem_isr(int irq, void *data)
{
	struct rapuyama_modem *m = data;
	unsigned long flags;

	spin_lock_irqsave(&m->lock, flags);
	sisa_drain(m);
	/* ack the software-interrupt source (karap FUN_800b8260: 0x1000 -> 0x288) */
	writel_relaxed(SWIRQ_BIT, m->ctrl + SWIRQ_CLEAR);
	spin_unlock_irqrestore(&m->lock, flags);
	return IRQ_HANDLED;
}

/* map a reg range without reserving it (shared SoC registers) */
static void __iomem *rapuyama_map(struct platform_device *pdev, int idx)
{
	struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, idx);

	if (!res) {
		dev_err(&pdev->dev, "no reg %d\n", idx);
		return NULL;
	}
	return devm_ioremap(&pdev->dev, res->start, resource_size(res));
}

static int rapuyama_modem_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rapuyama_modem *m;
	int ret;

	if (!enable) {
		rapuyama_note("e6: modem driver disabled (enable=0), neutral run\n");
		return 0;
	}

	m = devm_kzalloc(dev, sizeof(*m), GFP_KERNEL);
	if (!m)
		return -ENOMEM;
	m->dev = dev;
	spin_lock_init(&m->lock);

	/*
	 * Map with devm_ioremap, NOT devm_platform_ioremap_resource: the clock
	 * controller window (reg 3) and the modem control region (reg 0) are
	 * shared with clk-rapuyama / the irqchip, so an exclusive
	 * request_mem_region would -EBUSY and the probe would fail (it did).
	 * Only reg 0 (ctrl, mailbox) and reg 3 (clk) are used now; the FIFO regs
	 * stay in the DT but the FIFO path is dead (e6beac proved 0 sends).
	 */
	m->ctrl = rapuyama_map(pdev, 0);
	m->clk = rapuyama_map(pdev, 3);
	if (!m->ctrl || !m->clk)
		return -ENXIO;

	m->ring.base = devm_kcalloc(dev, RING_ENTRIES, sizeof(struct sisa_entry),
				    GFP_KERNEL);
	if (!m->ring.base)
		return -ENOMEM;
	m->ring.size = RING_ENTRIES;

	/* line 30 is optional: the irqchip owns the INTC ack, the keepalive does
	 * not need it, and with modem_link=1 the line never reaches us anyway. */
	m->irq = platform_get_irq_optional(pdev, 0);
	if (m->irq > 0) {
		ret = devm_request_irq(dev, m->irq, rapuyama_modem_isr, 0,
				       dev_name(dev), m);
		if (ret)
			return dev_err_probe(dev, ret, "request modem IRQ\n");
		writel_relaxed(SWIRQ_BIT, m->ctrl + SWIRQ_CLEAR);
	}

	if (iface_init)
		modem_iface_up(m);

	g_modem = m;
	platform_set_drvdata(pdev, m);

	/* kick the mailbox handshake: present the first sync word + our doorbell,
	 * seed last_rx so the poll only steps when the modem actually advances. */
	m->mb_last_rx = readl_relaxed(m->ctrl + MB_RX);
	writel_relaxed(mb_cycle[0], m->ctrl + MB_TX);
	writel_relaxed(0xffffffff, m->ctrl + MB_DB0);
	timer_setup(&m->em_timer, em_keepalive, 0);
	mod_timer(&m->em_timer, jiffies + msecs_to_jiffies(2000));

	rapuyama_note("e6: MODEM-HANDSHAKE-FOLLOW build, poll %dms, iface_init=%d\n",
		      MB_POLL_MS, iface_init);
	dev_info(dev, "RapuYama modem handshake-follow driver initialised\n");
	return 0;
}

static void rapuyama_modem_remove(struct platform_device *pdev)
{
	struct rapuyama_modem *m = platform_get_drvdata(pdev);

	timer_delete_sync(&m->em_timer);
	g_modem = NULL;
}

static const struct of_device_id rapuyama_modem_of_match[] = {
	{ .compatible = "nokia,rapuyama-modem" },
	{ }
};
MODULE_DEVICE_TABLE(of, rapuyama_modem_of_match);

static struct platform_driver rapuyama_modem_driver = {
	.probe = rapuyama_modem_probe,
	.remove = rapuyama_modem_remove,
	.driver = {
		.name = "rapuyama-modem",
		.of_match_table = rapuyama_modem_of_match,
	},
};
module_platform_driver(rapuyama_modem_driver);

MODULE_DESCRIPTION("Nokia RapuYama modem (CMT) EM keepalive + SISA link driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rapuyama-modem");