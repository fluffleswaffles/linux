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
 *   bit0 0x01  asic wd kick the way the modem sw does it (see below)
 *   bit1 0x02  gazoo wd kick the way the modem sw does it (see below)
 *   bit2 0x04  PMIC raw 0x10  = 0x7000      (power.dll logical reg 0xe, cmd 0x50)
 *   bit3 0x08  secure smc #0  (mmu on, zeroed buffer via c12) - the monitor call
 *   bit4 0x10  sisa self-irq  0x20040284 = 0x3000 (raise modem lines 30/31)
 *   bit7 0x80  EM keepalive: send the decoded EM "awake" ISI frame to the modem
 *              FIFO (0x40100000) every period - THE fix (docs/modem-link-spec.md,
 *              docs/modem-em-keepalive-plan.md). tunables: em_chan/em_state/em_skipclk
 *
 * if the phone survives past ~210 s with some mask -> that bit's action is (or
 * includes) the fix; bisect down. if it still dies with mask=0x1f -> the killer
 * is none of these (untouchable from the APE, e.g. modem hw supervision).
 *
 * bit0/bit1 come from the decrypted modem sw (SOS*ISASW, dram dump 17.10,
 * work/ghidra/isasw.bin.c). its MTC WD task kicks on timer 0x108 every 800 ms
 * (1400 ms in some states), FUN_c62b9e8a -> hw_wdog.c FUN_c62da8c0(bits):
 *   bit0: if the asic wd is on (byte at c68266d4 == 1, set by FUN_c62da8e2
 *         which also writes 0x50000 to 0x200336c8) -> 0x200336c4 = 0
 *   bit1: hw_gazoo.c FUN_c62f03e0(0x17003f, 0x20): cbus dev 1 reg 0x17,
 *         read-modify-write the low 6 bits to 0x20, inside the hw semaphore
 *         0x20040204 (spin until it reads >= 1, release with 0xffffffff)
 * the modem sw's cbus master (hw_csm.c) never touches +0x3c, unlike karap /
 * rapuyama_wdt. the guest data (c68.. = phys 0x1f8b0000) is the shm@1f8b0000
 * reservation, linux leaves it alone so the flag is still what the modem sw
 * left there at the handoff.
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
/* 0x10 (sisa poke) dropped from the default 17.10, it writes modem-side regs */
static unsigned int mask = 0x7;
module_param(mask, uint, 0644);
MODULE_PARM_DESC(mask, "bitmask of keepalive actions (see source)");

/*
 * once the cbus-write trace (e6cbus) reveals a hidden periodic PMIC pet, set it
 * here with NO rebuild: rapuyama_keepalive.mask=0x40 petreg=0xNN petval=0xVVVV
 * replicates FUN_800b8494(1, petreg, _, petval) every period. petreg is the raw
 * PMIC register (the cbus command byte = dev<<6|petreg).
 */
static unsigned int petreg;
static unsigned int petval;
module_param(petreg, uint, 0644);
module_param(petval, uint, 0644);

/*
 * bit7 0x80 = the real fix: the modem resets the APE when it stops receiving
 * periodic Energy-Management "awake" ISI messages. send the decoded 24-byte EM
 * AWAKE frame over the modem FIFO every FEED_SECS. the tunables that still want
 * a hardware refinement are cmdline params (no rebuild): em_chan (FIFO channel),
 * em_state (message byte 0x10), and em_skipclk (1 = do NOT run the iface clock
 * bring-up, in case it is already up / to isolate a USB regression).
 * see docs/modem-em-keepalive-plan.md + docs/modem-link-spec.md.
 */
static unsigned int em_chan;
static unsigned int em_state;
static unsigned int em_skipclk;
module_param(em_chan, uint, 0644);
module_param(em_state, uint, 0644);
module_param(em_skipclk, uint, 0644);

/*
 * the ISI routing header bytes [0,1,2,6,7,8] = media, receiver-device,
 * sender-device, receiver-object, sender-object, transaction. symbian's IAD
 * fills them at send time from the channel's connection; they are NOT static, so
 * they are cmdline-tunable here (default 0) to bisect on hardware without a
 * rebuild. #96 showed the transport works but survival dropped (~63 s) with these
 * all 0 - the modem reads the frame but likely rejects the bad routing.
 */
static unsigned int em_h0, em_h1, em_h2, em_h6, em_h7, em_h8;
module_param(em_h0, uint, 0644);
module_param(em_h1, uint, 0644);
module_param(em_h2, uint, 0644);
module_param(em_h6, uint, 0644);
module_param(em_h7, uint, 0644);
module_param(em_h8, uint, 0644);

/* em_scan=1: before sending, scan the preserved modem SHM (0x1f8b0000) for real
 * EM awake/sleep ISI frames left by symbian and dump their 24 bytes - this gives
 * the real routing header [0,1,2,6,7,8] + channel that the hand-crafted frame is
 * missing, without any symbian probe (just a read of mapped ram) */
static unsigned int em_scan = 1;
module_param(em_scan, uint, 0644);

/*
 * em_sweep=1: auto-bisect the FIFO channel across reboots. the modem demuxes by
 * FIFO channel; the EM channel (0..8) is runtime connection state we cannot pin
 * statically. store the channel to try in a reset-surviving scratch word (phys
 * 0x1f0f0000, free in the ramcon reserved region, below the isah block at
 * 0x1f0f1000); each boot sends EM on that channel and advances it, so the user
 * just reflashes once and reboots ~9 times - the boot that survives >>220 s
 * names the EM channel in the klog ("em sweep testing chan N").
 */
static unsigned int em_sweep;
module_param(em_sweep, uint, 0644);
#define SWEEP_PHYS	0x1f0f0000u
#define SWEEP_MAGIC	0x454d5357u	/* 'EMSW' */

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
#define CBUS_RD_CMD	0x18

#define FEED_SECS	30	/* comfortably beats any ~210 s supervision timeout */
#define WD_MS		800	/* the modem sw's MTC WD timer 0x108 */

/* EM-keepalive transport (docs/modem-link-spec.md, leaf FUN_800beac8) */
#define FIFO_PHYS	0x40100000u	/* TX byte FIFO base, + chan*0x1000 */
#define DOOR_PHYS	0x401fd800u	/* send doorbell: write chan */
#define EMCLK_PHYS	0x20032000u	/* clock mux window (sel3 at +0x644) */
#define EMCTL_PHYS	0x20033000u	/* iface ctrl window (+0xb8, +0x84, +0xd8) */
#define EM_MUX_SEL3	0x644		/* FUN_800bec88 sel=3 reg, within EMCLK */
#define EM_CTL_B8	0xb8		/* = 1 */
#define EM_CTL_84	0x84		/* = 8 */

#define HWSEM_PHYS	0x20040204u	/* hw semaphore 0, c6040200 + 4 */
#define SYSRST_PHYS	0x20033018u	/* hw_reset_reason_get reads this */
#define GUEST_PHYS	0x1f8b0000u	/* modem sw data, c6800000 */
#define GUEST_SIZE	0x44d000
#define G_WDON		0x266d4		/* c68266d4: asic wd enabled */
#define G_RST		0x0187c		/* c680187c: [1] reset reason, [2] loop count */
#define G_RSTVALID	0x265e0		/* c68265e0: 1 = [1] is valid */

static void __iomem *cbus;	/* 0x20030700 */
static void __iomem *soc;	/* 0x200336c4 */
static void __iomem *sisa;	/* 0x20040280 (status), +4 set, +8 clear */
static void __iomem *hwsem;	/* 0x20040204 */
static u8 *guest;		/* 0x1f8b0000, reserved ram */
static void __iomem *em_tx;	/* FIFO_PHYS, one 4 KiB page (channel em_chan) */
static void __iomem *em_door;	/* DOOR_PHYS page */
static void __iomem *em_clk;	/* EMCLK_PHYS page (mux) */
static void __iomem *em_ctl;	/* EMCTL_PHYS page (iface ctrl) */
/* EM sequence counter. the modem likely rejects frames whose counter is below
 * the last value symbian sent before handoff (stale -> treated as errors, which
 * is why bursts of our frames reset it faster). start high so every frame is
 * strictly greater than whatever symbian left; em_ctr0 is cmdline-tunable */
static unsigned int em_ctr0 = 0x10000000;
module_param(em_ctr0, uint, 0644);
static u32 em_ctr;
static unsigned int em_sends;
static void *smcbuf;		/* zeroed page for the secure call */
static u32 smcbuf_phys;
static struct delayed_work ka_work;
static struct delayed_work wd_work;
static unsigned long started;
static unsigned int wd_kicks, wd_fails;

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

/* hw_csm.c busy wait (FUN_c62efffc): 0x100 polls of +8 */
static int g_cbus_wait(void)
{
	int n = 0x100;

	while (readl(cbus + CBUS_BUSY)) {
		if (!--n)
			return -ETIMEDOUT;
		udelay(1);
	}
	return 0;
}

/* FUN_c62f00a4 / FUN_c62f0038, 15 = nbits the modem sw passes */
static int g_cbus_read(u32 dev, u32 reg, u32 *val)
{
	int ret;

	writel(1, cbus + CBUS_EN);
	writel(0xf, cbus + CBUS_BITS);
	writel(dev << 6 | reg | 0x20, cbus + CBUS_RD_CMD);
	ret = g_cbus_wait();
	*val = readl(cbus + CBUS_DATA);
	writel(0, cbus + CBUS_EN);
	return ret;
}

static int g_cbus_write(u32 dev, u32 reg, u32 val)
{
	int ret;

	writel(1, cbus + CBUS_EN);
	writel(0xf, cbus + CBUS_BITS);
	writel(val, cbus + CBUS_DATA);
	writel(dev << 6 | reg, cbus + CBUS_WR_CMD);
	ret = g_cbus_wait();
	writel(0, cbus + CBUS_EN);
	return ret;
}

/* FUN_c62dafcc/FUN_c62dafe4, the modem sw spins forever, we give up after 1 ms */
static bool hwsem_get(void)
{
	int n = 1000;

	while ((int)readl(hwsem) < 1) {
		if (!--n)
			return false;
		udelay(1);
	}
	return true;
}

static void hwsem_put(void)
{
	writel(0xffffffff, hwsem);
}

/*
 * modem-interface clock bring-up, the minimal subset of ecust cold-init
 * FUN_800bede4: the mux enable FUN_800bec88(3,0xf,2) + the two iface ctrl writes.
 * only needed if symbian did NOT leave the iface clocked (em_skipclk=0). the full
 * cold-init does much more and a prior replica broke usb, so we keep to the subset.
 * FUN_800bec88(sel=3): sel reg 0x20032644, paired trigger reg sel+8 (0x2003264c);
 * write the mux value, pulse the trigger 0x9480 -> 0xd400 -> 0x9480, spin on bit4.
 */
static void modem_iface_clock_up(void)
{
	int n = 0x4000;

	/* sel3 = (b & 0x1f) | ((a & 0xffff07ff) << 5) = 2 | (0xf << 5) = 0x1e2 */
	writel(0x1e2, em_clk + EM_MUX_SEL3);
	udelay(2);
	writel(0x9480, em_clk + EM_MUX_SEL3 + 8);	/* trigger */
	udelay(2);
	writel(0xd400, em_clk + EM_MUX_SEL3 + 8);
	udelay(2);
	writel(0x9480, em_clk + EM_MUX_SEL3 + 8);
	while (!(readl(em_clk + EM_MUX_SEL3 + 8) & 0x10) && --n)
		udelay(1);
	writel(1, em_ctl + EM_CTL_B8);			/* 0x200330b8 = 1 */
	writel(8, em_ctl + EM_CTL_84);			/* 0x20033084 = 8 */
}

/*
 * send one EM AWAKE power-state ISI frame to the modem FIFO (channel em_chan):
 * 0xc0, the 24 message bytes, 0x00, all to the one FIFO byte port, then ring the
 * doorbell with the channel. the modem resets the APE if it stops seeing these.
 * bytes decoded from empmIsaIf FUN_801012ec (docs/modem-link-spec.md). the ISI
 * header bytes [0,1,2,6,7,8] are filled by symbian's alloc+IAD routing - left 0
 * here, refine from a wire capture if the modem ignores the frame.
 */
static void em_send(void)
{
	u8 m[24] = {0};
	u32 c = em_ctr++;
	int i;

	m[0x03] = 0x47;		/* message-group */
	m[0x04] = 0x12;		/* length low = 0x18 - 6 */
	m[0x05] = 0x00;		/* length high */
	m[0x09] = 0x04;		/* msg id */
	m[0x0a] = 0x02; m[0x0b] = 0x01; m[0x0c] = 0x05; m[0x0d] = 0x0c;
	m[0x0e] = 0x04; m[0x0f] = 0x04;
	m[0x10] = (u8)em_state;		/* EM state flag */
	m[0x14] = c >> 24; m[0x15] = c >> 16; m[0x16] = c >> 8; m[0x17] = c;
	/* ISI routing header (cmdline-tunable, default 0): media/rdev/sdev/robj/sobj/txn */
	m[0x00] = (u8)em_h0; m[0x01] = (u8)em_h1; m[0x02] = (u8)em_h2;
	m[0x06] = (u8)em_h6; m[0x07] = (u8)em_h7; m[0x08] = (u8)em_h8;

	writeb(0xc0, em_tx);
	for (i = 0; i < (int)sizeof(m); i++)
		writeb(m[i], em_tx);
	writeb(0x00, em_tx);
	writeb((u8)em_chan, em_door);
	em_sends++;
}

/* FUN_c62da8c0(bits) + FUN_c62f03e0(0x17003f, 0x20) */
static void g_wd_kick(void)
{
	unsigned long flags;
	u32 v;
	int ret = 0;

	if ((mask & 0x01) && guest && READ_ONCE(guest[G_WDON]) == 1)
		writel(0, soc);
	if (!(mask & 0x02))
		return;
	/* rapuyama_wdt uses the same cbus master, irqs off keeps us from
	 * interleaving with it (single core) */
	local_irq_save(flags);
	if (!hwsem_get()) {
		ret = -EBUSY;
	} else {
		ret = g_cbus_read(1, 0x17, &v);
		if (!ret)
			ret = g_cbus_write(1, 0x17, (v & ~0x3fu) | 0x20);
		hwsem_put();
	}
	local_irq_restore(flags);
	if (ret)
		wd_fails++;
	else
		wd_kicks++;
}

static void wd_do(struct work_struct *w)
{
	g_wd_kick();
	schedule_delayed_work(&wd_work, msecs_to_jiffies(WD_MS));
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

	if (mask & 0x04)
		ka_cbus(1, 0x10, 0x7000);
	if (mask & 0x10) {
		/* pulse: clear then set, so every period is a fresh rising edge on
		 * the modem doorbell. a bare set latches once (the modem handler
		 * that would clear it is not running) and never re-triggers, so it
		 * could not feed a supervisor that wants a periodic heartbeat */
		writel(0x3000, sisa + 0x08);	/* clear lines 30/31 */
		writel(0x3000, sisa + 0x04);	/* set -> rising edge */
	}
	if ((mask & 0x40) && petreg)
		ka_cbus(1, petreg, petval);	/* the discovered pet, from cmdline */
	if ((mask & 0x80) && em_tx)
		em_send();			/* THE fix: periodic EM awake to modem */
	if (mask & 0x08)
		ka_smc();			/* last: may not return */

	rapuyama_note("keepalive: alive %u s, mask %#x done, wd kicks %u fails %u em %u\n",
		      secs, mask, wd_kicks, wd_fails, em_sends);
	schedule_delayed_work(&ka_work, FEED_SECS * HZ);
}

/*
 * hw_conf record 0xd (FUN_c62ef392(0xd,0)+8) = the startup reason the boot
 * loader hands over. records are {tag, len, data..}, tag 2 ends the list. the
 * list sat at 0x100acf18 in the 17.10 dump, record 0xd at +0x208. mtc_sys.c
 * decodes: bit6 state transition, bit7 sw reset, bit8 hw watchdog, bit11
 * secure violation, bit4 = check the pre-reset magic first
 */
#define HWC_PHYS	0x100a0000u
#define HWC_SIZE	0x10000

static void ka_log_hwconf(void)
{
	u32 __iomem *p = ioremap(HWC_PHYS, HWC_SIZE);
	unsigned int i;

	if (!p)
		return;
	/* the tag 0xc record (len 16) sits right before the 0xd one (len 12) */
	for (i = 4; i + 3 < HWC_SIZE / 4; i++) {
		if (readl(p + i) == 0xd && readl(p + i + 1) == 0xc &&
		    readl(p + i - 4) == 0xc && readl(p + i - 3) == 0x10) {
			rapuyama_note("keepalive: hw_conf startup reason %08x @%08x\n",
				      readl(p + i + 2), HWC_PHYS + i * 4);
			break;
		}
	}
	if (i + 3 >= HWC_SIZE / 4)
		rapuyama_note("keepalive: hw_conf record 0xd not found\n");
	iounmap(p);
}

/* what the modem sw decoded at this boot = why the LAST boot died
 * (FUN_c62da680 / FUN_c62da714), plus the raw regs it decodes from */
static void ka_log_reason(void)
{
	void __iomem *rst = ioremap(SYSRST_PHYS & ~0xfffu, 0x1000);
	unsigned long flags;
	u32 v = 0;
	int ret = -EBUSY;

	ka_log_hwconf();
	if (guest)
		rapuyama_note("keepalive: modem sw rst valid=%u reason=%#x loop=%u wd_on=%u\n",
			      guest[G_RSTVALID], guest[G_RST + 1], guest[G_RST + 2],
			      guest[G_WDON]);
	if (rst) {
		rapuyama_note("keepalive: 0x20033018=%08x\n",
			      readl(rst + (SYSRST_PHYS & 0xfffu)));
		iounmap(rst);
	}
	rapuyama_note("keepalive: asic wd 0x200336c0..cc %08x %08x %08x %08x\n",
		      readl(soc - 4), readl(soc), readl(soc + 4), readl(soc + 8));
	local_irq_save(flags);
	if (hwsem_get()) {
		ret = g_cbus_read(1, 0x17, &v);
		hwsem_put();
	}
	local_irq_restore(flags);
	rapuyama_note("keepalive: gazoo 0x17=%04x (%d)\n", v, ret);
}

static int __init rapuyama_keepalive_init(void)
{
	cbus = ioremap(CBUS_PHYS, 0x100);
	soc = ioremap(SOC_PHYS & ~0xfffu, 0x1000);
	sisa = ioremap(SISA_PHYS, 0x10);
	hwsem = ioremap(HWSEM_PHYS, 4);
	if (!cbus || !soc || !sisa || !hwsem) {
		rapuyama_note("keepalive: ioremap failed\n");
		return -ENOMEM;
	}
	soc += SOC_PHYS & 0xfffu;
	/* reserved but mapped ram, memremap gives the linear map */
	guest = memremap(GUEST_PHYS, GUEST_SIZE, MEMREMAP_WB);
	ka_log_reason();

	smcbuf = (void *)get_zeroed_page(GFP_KERNEL);
	if (smcbuf)
		smcbuf_phys = __pa(smcbuf);

	/* scan SHM for a real EM frame (resource 0x47, msg id 4, 0xc/0xd = 05/0c)
	 * and dump 24 bytes so we can read the real routing header off the wire */
	if (em_scan) {
		/* scan BOTH reserved modem shm regions: the big data area and the
		 * small 0x1ffe0000 mailbox (likely where ISI headers live). match any
		 * frame carrying the EM resource 0x47 (loose) and dump 24 bytes so we
		 * read the real routing header [0,1,2,6,7,8] + find the channel */
		static const struct { u32 phys, size; } rgn[] = {
			{ GUEST_PHYS, GUEST_SIZE },
			{ 0x1ffe0000u, 0x20000u },
		};
		unsigned int r, off, hits = 0;

		for (r = 0; r < ARRAY_SIZE(rgn); r++) {
			u8 *base = (r == 0 && guest) ? guest :
				   memremap(rgn[r].phys, rgn[r].size, MEMREMAP_WB);

			if (!base)
				continue;
			for (off = 0; off + 24 <= rgn[r].size && hits < 16; off++) {
				u8 *m = base + off;

				if (m[3] != 0x47 || m[9] == 0)
					continue;
				rapuyama_note("keepalive: em? %08x+%05x: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
					      rgn[r].phys, off, m[0], m[1], m[2], m[3],
					      m[4], m[5], m[6], m[7], m[8], m[9], m[10],
					      m[11], m[12], m[13], m[14], m[15], m[16],
					      m[17], m[18], m[19], m[20], m[21], m[22], m[23]);
				hits++;
			}
			if (r != 0 || !guest)
				memunmap(base);
		}
		if (!hits)
			rapuyama_note("keepalive: em scan found no 0x47 frame\n");
	}

	em_ctr = em_ctr0;	/* start the sequence high (above symbian's last) */

	/* EM-keepalive (bit7): map the modem FIFO, doorbell and (unless em_skipclk)
	 * bring the iface clock up, then ka_do sends the awake frame each period */
	if (mask & 0x80) {
		em_tx = ioremap(FIFO_PHYS + em_chan * 0x1000, 0x1000);
		em_door = ioremap(DOOR_PHYS & ~0xfffu, 0x1000);
		em_clk = ioremap(EMCLK_PHYS, 0x1000);
		em_ctl = ioremap(EMCTL_PHYS, 0x1000);
		if (!em_tx || !em_door || !em_clk || !em_ctl) {
			rapuyama_note("keepalive: em ioremap failed\n");
		} else {
			em_door += DOOR_PHYS & 0xfffu;
			if (!em_skipclk)
				modem_iface_clock_up();
			rapuyama_note("keepalive: em armed chan %u skipclk %u state %#x\n",
				      em_chan, em_skipclk, em_state);
		}
	}

	started = jiffies;
	rapuyama_note("keepalive: armed, mask %#x, period %d s\n", mask, FEED_SECS);
	INIT_DELAYED_WORK(&ka_work, ka_do);
	schedule_delayed_work(&ka_work, HZ); /* first pass ~1 s in */
	INIT_DELAYED_WORK(&wd_work, wd_do);
	schedule_delayed_work(&wd_work, 0);
	return 0;
}
late_initcall(rapuyama_keepalive_init);