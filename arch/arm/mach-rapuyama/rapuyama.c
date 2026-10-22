// SPDX-License-Identifier: GPL-2.0
/*
 * Nokia RapuYama (N8/E6/E7/C7) machine
 *
 * Mostly a plain DT machine. It also maps the first 1 MB of the SoC bus
 * (phys 0x20000000: cbus master, clock/power block, soc reset) at a fixed
 * address so early code can reach it, which the bring-up checkpoints use:
 * with "e6.stop=N" on the command line the kernel powers the phone off (pmic
 * watchdog expire over cbus, like symbian's force power off) when it reaches
 * checkpoint N. Phone switching off right away = got there, the usual soc
 * watchdog reset ~12 s in = died before it. No console needed.
 */

#include <linux/init.h>
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdesc.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kernel_stat.h>
#include <linux/math64.h>
#include <linux/irqflags.h>
#include <linux/console.h>
#include <linux/kmsg_dump.h>
#include <linux/sched.h>
#include <linux/sched/debug.h>
#include <linux/export.h>
#include <linux/signal.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <asm/bug.h>
#include <asm/cacheflush.h>
#include <asm/irq_regs.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#define RY_BUS_PHYS	0x20000000
#define RY_BUS_VIRT	0xf8000000
#define RY_BUS(off)	((void __iomem *)(RY_BUS_VIRT + (off)))

/* cbus master, offsets from karap 0x800b8494 (base ptr 0x20030704 there) */
#define RY_CBUS		0x30700
#define CBUS_EN		0x04
#define CBUS_BUSY	0x08
#define CBUS_WR_CMD	0x1c
#define CBUS_DATA	0x20
#define CBUS_BITS	0x2c
#define CBUS_WR_MODE	0x3c

/*
 * the ramcon buffer (earlycon=ramcon,0x1f000000, reserved no-map in the dt),
 * mapped a second time here so the stuck-kernel dump below can write into it
 * directly, without printk and console_sem
 */
#define RY_RAMCON_PHYS	0x1f000000
#define RY_RAMCON_VIRT	0xf8100000
#define RY_RAMCON_MAGIC	0x474c3645u	/* 'E6LG', same layout as rapuyama_earlycon */
#define RY_RAMCON_SIZE	0xf0000u
#define RY_RAMCON_HDR	8u

static struct map_desc rapuyama_io_desc[] __initdata = {
	{
		.virtual	= RY_BUS_VIRT,
		.pfn		= __phys_to_pfn(RY_BUS_PHYS),
		.length		= SZ_1M,
		.type		= MT_DEVICE,
	},
	{
		.virtual	= RY_RAMCON_VIRT,
		.pfn		= __phys_to_pfn(RY_RAMCON_PHYS),
		.length		= SZ_1M,
		.type		= MT_DEVICE,
	},
};

static int rapuyama_stop;
static bool rapuyama_mapped;

/*
 * reading a SoC block that is unpowered or unclocked ends in a precise
 * external abort (fsr 0x1008). that happens with the usb block when the phone
 * was booted without a cable: symbian had the modem power the usb side down,
 * and only the modem can bring it back. rapuyama_safe_readl() guards one load
 * so a driver can find out the block is dead instead of oopsing on it.
 */
static unsigned long rapuyama_safe_pc;	/* the guarded ldr, 0 = none armed */
static bool rapuyama_safe_faulted;

static int rapuyama_ext_abort(unsigned long addr, unsigned int fsr,
			      struct pt_regs *regs)
{
	if (!rapuyama_safe_pc || instruction_pointer(regs) != rapuyama_safe_pc)
		return 1;	/* not ours: the usual "unhandled fault" oops */
	rapuyama_safe_faulted = true;
	regs->ARM_pc += 4;	/* skip the ldr, the result stays ~0 */
	return 0;
}

int rapuyama_safe_readl(const void __iomem *addr, u32 *val);

/* 0 and the value, or -EIO if the read aborted (block powered down) */
int rapuyama_safe_readl(const void __iomem *addr, u32 *val)
{
	unsigned long flags, pc;
	u32 v = ~0u;
	bool faulted;

	local_irq_save(flags);
	rapuyama_safe_faulted = false;
	asm volatile(
		"	adr	%1, 1f\n"
		"	str	%1, [%3]\n"
		"1:	ldr	%0, [%2]\n"
		: "+r" (v), "=&r" (pc)
		: "r" (addr), "r" (&rapuyama_safe_pc)
		: "memory");
	WRITE_ONCE(rapuyama_safe_pc, 0);
	faulted = READ_ONCE(rapuyama_safe_faulted);
	local_irq_restore(flags);

	*val = v;
	return faulted ? -EIO : 0;
}
EXPORT_SYMBOL_GPL(rapuyama_safe_readl);

static void __init rapuyama_init_early(void)
{
	hook_fault_code(8, rapuyama_ext_abort, SIGBUS, 0,
			"external abort on non-linefetch");
}

static int __init rapuyama_stop_param(char *s)
{
	return kstrtoint(s, 0, &rapuyama_stop);
}
early_param("e6.stop", rapuyama_stop_param);

#ifdef CONFIG_SERIAL_RAPUYAMA_EARLYCON
void rapuyama_ramcon_set_full(phys_addr_t phys, void __iomem *virt);
#else
static inline void rapuyama_ramcon_set_full(phys_addr_t phys, void __iomem *virt) { }
#endif

static void __init rapuyama_map_io(void)
{
	iotable_init(rapuyama_io_desc, ARRAY_SIZE(rapuyama_io_desc));
	rapuyama_mapped = true;
	/* the earlycon only got one page of the ramcon buffer, give it all */
	rapuyama_ramcon_set_full(RY_RAMCON_PHYS, (void __iomem *)RY_RAMCON_VIRT);
}

/* pmic dev 1 reg 0x17 (watchdog) = 0 -> the pmic switches the phone off */
static void rapuyama_pmic_off(void)
{
	void __iomem *cb = RY_BUS(RY_CBUS);
	int n = 0x10000;

	writel(1, cb + CBUS_EN);
	writel(1, cb + CBUS_WR_MODE);
	writel(15, cb + CBUS_BITS);
	writel(0, cb + CBUS_DATA);
	writel((1 << 6) | 0x17, cb + CBUS_WR_CMD);
	while (readl(cb + CBUS_BUSY) && --n)
		cpu_relax();
	writel(0, cb + CBUS_EN);
}

void rapuyama_checkpoint(int n);

void rapuyama_checkpoint(int n)
{
	if (!rapuyama_mapped || n != rapuyama_stop)
		return;
	local_irq_disable();
	rapuyama_pmic_off();
	for (;;)
		cpu_relax();
}

/*
 * bring-up: the soc watchdog counter (0x200336c4, symbian kicks it with
 * 0x3fffffff every 3 s) resets the phone ~13 s after the last kick, and the
 * real watchdog driver only probes at subsys_initcall. feed it from the ms
 * timer irq until then. one plain write, no cbus, so it can't race the
 * driver's pmic transfers. the pmic one (48 s) is left to the driver.
 * the driver calls rapuyama_early_wdt_stop() once it feeds the watchdog
 * itself, so a later hang still ends in a (warm) soc reset instead of
 * being kept alive by the timer irq until the pmic powers the phone off.
 *
 * if the driver isn't up after e6.wdtfeed seconds (default 15) the kernel is
 * stuck somewhere with irqs still alive: dump what the cpu was doing and all
 * tasks into the log, then stop feeding. the soc watchdog then does a warm
 * reset ~13 s later, well before the pmic one (48 s from the booter's kick)
 * powers off and wipes the ram, so ramread.sis gets the dump. 0 = never stop
 */
#define RY_SOC_WDT	0x336c4
#define RY_SOC_WDT_VAL	0x3fffffff

static bool rapuyama_early_wdt_done;
static unsigned int rapuyama_wdt_feed_s = 15;
static unsigned int rapuyama_wdt_kicks;

static int __init rapuyama_wdtfeed_param(char *s)
{
	return kstrtouint(s, 0, &rapuyama_wdt_feed_s);
}
early_param("e6.wdtfeed", rapuyama_wdtfeed_param);

/*
 * stall detector, once the watchdog driver runs: the watchdog core pings
 * every 1.5 s from a SCHED_FIFO kthread. if the ms timer irq still runs but
 * no ping came for e6.stall seconds (default 5, 0 = off), the kernel is
 * stuck with irqs alive (irq storm, a task spinning, a deadlock): dump it
 * the same way as the early case. the soc watchdog resets ~13 s after the
 * last ping, so the dump is in ramcon before the warm reset.
 */
static unsigned int rapuyama_stall_s = 5;
static unsigned int rapuyama_alive_ticks;

/*
 * phase calibration from the booter (boot_app RB_CALIB, isah block words
 * 0x100..): the 32 khz fast counter at the handoff and at the last secure
 * request symbian sent. the counter runs on across the handoff, so the
 * heartbeat can say how long ago both were
 */
#define RY_FC		0x33350
#define RY_FC_LATCH	0x33354
static u32 rapuyama_calib_fc_handoff, rapuyama_calib_fc_req;
static u32 rapuyama_calib_reqs;
static bool rapuyama_calib_ok;
/* calibration v2 */
static int rapuyama_calib_due_ms;
static bool rapuyama_calib_due_ok;
/* calibration v3 */
static u32 rapuyama_calib_kick_fc;
static bool rapuyama_calib_kick_ok;
/* calibration v2 */
static u32 rapuyama_calib_sym_ms;

static u32 rapuyama_fc(void)
{
	writel_relaxed(0xc6033354u, RY_BUS(RY_FC_LATCH));
	return readl_relaxed(RY_BUS(RY_FC));
}
static unsigned long rapuyama_last_ping;
static bool rapuyama_pinged;
static bool rapuyama_stall_dumped;

static int __init rapuyama_stall_param(char *s)
{
	return kstrtouint(s, 0, &rapuyama_stall_s);
}
early_param("e6.stall", rapuyama_stall_param);

void rapuyama_early_wdt_kick(void);
void rapuyama_early_wdt_stop(void);
void rapuyama_wdt_pinged(void);
__printf(1, 2) void rapuyama_note(const char *fmt, ...);

/* append straight to the ramcon ring, same format as the earlycon */
static void rapuyama_raw_write(const char *s, size_t n)
{
	void __iomem *base = (void __iomem *)RY_RAMCON_VIRT;
	u32 len;

	if (readl(base) != RY_RAMCON_MAGIC)
		return;
	len = readl(base + 4);
	while (n--) {
		writeb(*s++, base + RY_RAMCON_HDR + (len % RY_RAMCON_SIZE));
		len++;
	}
	writel(len, base + 4);
}

static __printf(1, 2) void rapuyama_raw_printf(const char *fmt, ...)
{
	static char buf[256];
	va_list args;
	int n;

	va_start(args, fmt);
	n = vscnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);
	rapuyama_raw_write(buf, n);
}

/*
 * for drivers (irqchip, watchdog): a line straight into ramcon, no printk,
 * usable from any context, including with every irq masked
 */
void rapuyama_note(const char *fmt, ...)
{
	static char buf[256];
	unsigned long flags;
	va_list args;
	int n;

	if (!rapuyama_mapped)
		return;
	local_irq_save(flags);
	va_start(args, fmt);
	n = vscnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);
	rapuyama_raw_write(buf, n);
	local_irq_restore(flags);
}

/* every irq that fired so far: a storm shows up as one huge count */
static void rapuyama_raw_irq_counts(void)
{
	struct irq_desc *desc;
	unsigned int i, n;

	rapuyama_raw_printf("e6: irq counts (irq hwirq count name):\n");
	for_each_irq_desc(i, desc) {
		n = kstat_irqs_cpu(i, 0);
		if (!n)
			continue;
		rapuyama_raw_printf("e6:   %u %lu %u %s\n", i,
				    desc->irq_data.hwirq, n,
				    desc->action && desc->action->name ?
				    desc->action->name : "-");
	}
}

/* irqs that fired since the last call, so a lone modem event before the reset shows up */
static unsigned int rapuyama_irq_last[512];

static void rapuyama_raw_irq_deltas(void)
{
	struct irq_desc *desc;
	unsigned int i, n;
	char buf[256];
	int len = 0;

	for_each_irq_desc(i, desc) {
		if (i >= ARRAY_SIZE(rapuyama_irq_last))
			break;
		n = kstat_irqs_cpu(i, 0);
		if (n == rapuyama_irq_last[i])
			continue;
		len += scnprintf(buf + len, sizeof(buf) - len, " %u/%lu:%u", i,
				 desc->irq_data.hwirq, n - rapuyama_irq_last[i]);
		rapuyama_irq_last[i] = n;
	}
	buf[len] = 0;
	rapuyama_raw_printf("e6: irqs (irq/hwirq:n)%s\n", buf);
}

/*
 * the console may be what is stuck (console_sem held forever, nothing gets
 * printed while the kernel runs on), so after the dump copy the whole printk
 * ring buffer into ramcon by hand. it's lockless, readable from any context
 */
static void rapuyama_raw_replay_log(void)
{
	static char line[1024];
	struct kmsg_dump_iter iter;
	size_t len;

	rapuyama_raw_printf("\n===== e6: raw replay of the printk buffer =====\n");
	kmsg_dump_rewind(&iter);
	while (kmsg_dump_get_line(&iter, false, line, sizeof(line), &len))
		rapuyama_raw_write(line, len);
	rapuyama_raw_printf("===== e6: end of replay =====\n");
}

/* dump what the cpu and every task are doing, then replay printk to ramcon */
static void rapuyama_dump_stuck(const char *why)
{
	struct pt_regs *regs = get_irq_regs();

	/* the short version first, written raw so it lands no matter what */
	rapuyama_raw_printf("\n===== e6: %s, uptime %u ms =====\n", why,
			    jiffies_to_msecs(jiffies - INITIAL_JIFFIES));
	rapuyama_raw_printf("e6: current %s pid %d, in_task %d\n",
			    current->comm, task_pid_nr(current), (int)!!in_task());
	if (regs)
		rapuyama_raw_printf("e6: interrupted pc %pS lr %pS sp %08lx cpsr %08lx\n",
				    (void *)regs->ARM_pc, (void *)regs->ARM_lr,
				    regs->ARM_sp, regs->ARM_cpsr);
	if (console_trylock()) {
		rapuyama_raw_printf("e6: console_sem was free\n");
		console_unlock();
	} else {
		rapuyama_raw_printf("e6: console_sem is HELD by someone\n");
	}

	rapuyama_raw_irq_counts();

	/* the long version into the printk buffer, then copied out by hand */
	pr_emerg("rapuyama: %s, kernel looks stuck. interrupted at:\n", why);
	if (regs)
		show_regs(regs);
	pr_emerg("rapuyama: all tasks:\n");
	show_state();
	pr_emerg("rapuyama: the soc watchdog is not fed any more, warm reset soon\n");

	rapuyama_raw_replay_log();
}

/* the watchdog driver pinged (watchdog core thread or userspace) */
void rapuyama_wdt_pinged(void)
{
	WRITE_ONCE(rapuyama_last_ping, jiffies);
	WRITE_ONCE(rapuyama_pinged, true);
}

static void rapuyama_stall_check(void)
{
	char why[64];

	if (!rapuyama_stall_s || rapuyama_stall_dumped ||
	    !READ_ONCE(rapuyama_pinged))
		return;
	if (!time_after(jiffies, READ_ONCE(rapuyama_last_ping) +
				 rapuyama_stall_s * HZ))
		return;
	rapuyama_stall_dumped = true;
	snprintf(why, sizeof(why), "watchdog not pinged for %u ms",
		 jiffies_to_msecs(jiffies - READ_ONCE(rapuyama_last_ping)));
	rapuyama_dump_stuck(why);
}

/*
 * heartbeat straight into ramcon (not the console): after a reset the dump
 * shows how long linux lived. replaces the old init.sh one, which spammed
 * the console every 10 s
 */
static void rapuyama_raw_reg_changes(void);

static void rapuyama_alive(void)
{
	unsigned int up;
	u32 fc;

	if (++rapuyama_alive_ticks < 100)
		return;
	rapuyama_alive_ticks = 0;
	up = jiffies_to_msecs(jiffies - INITIAL_JIFFIES) / 1000;
	if (!READ_ONCE(rapuyama_calib_ok)) {
		rapuyama_raw_printf("e6: alive, uptime %u s\n", up);
		rapuyama_raw_irq_deltas();
		rapuyama_raw_reg_changes();
		return;
	}
	fc = rapuyama_fc();
	if (rapuyama_calib_due_ok)
		rapuyama_raw_printf("e6: alive, uptime %u s, %u s after the handoff, symbian uptime %u s\n",
				    up, (fc - rapuyama_calib_fc_handoff) >> 15,
				    (rapuyama_calib_sym_ms +
				     (u32)(((u64)(fc - rapuyama_calib_fc_handoff) * 1000) >> 15)) / 1000);
	else if (rapuyama_calib_due_ok)
		rapuyama_raw_printf("e6: alive, uptime %u s, %u s after the handoff, %d s after the secure request was due\n",
				    up, (fc - rapuyama_calib_fc_handoff) >> 15,
				    (int)(((u64)(fc - rapuyama_calib_fc_handoff) * 1000) >> 15) / 1000 -
				    rapuyama_calib_due_ms / 1000);
	else if (rapuyama_calib_reqs)
		rapuyama_raw_printf("e6: alive, uptime %u s, %u s after the handoff, %u s after the last secure request\n",
				    up, (fc - rapuyama_calib_fc_handoff) >> 15,
				    (fc - rapuyama_calib_fc_req) >> 15);
	else
		rapuyama_raw_printf("e6: alive, uptime %u s, %u s after the handoff\n",
				    up, (fc - rapuyama_calib_fc_handoff) >> 15);
	rapuyama_raw_irq_deltas();
	rapuyama_raw_reg_changes();
}

/*
 * flight recorder: the phone dies by an outside reset (2-3 s after usb and
 * the screen stop, back into symbian) with nothing in the log. every 100 ms
 * snapshot the always-on regs (clock gates, power domains, secondary irq
 * banks, gpio levels, soc watchdog...) into a ring at the end of the ramcon
 * buffer, so after the reset tools/parse_flight.py shows what moved in the
 * last ~25 s, and how long the cpu kept going after the usb died.
 * layout at ramcon +0xf8000: u32 'E6FR', u32 samples written, u32 nregs,
 * u32 nslots, u32 reg offsets[nregs] (from phys 0x20000000), then from +0x100
 * the slots: u32 seq, u32 uptime ms, u32 regs[nregs]
 */
#define RY_FR_OFF	0xf8000u
#define RY_FR_MAGIC	0x52463645u	/* 'E6FR' */
#define RY_FR_DATA	0x100u

static const u32 rapuyama_fr_regs[] = {
	0x33350, /* 32k fast counter */
	0x336c4, /* soc watchdog counter */
	0x40280, /* sisa self irq lines 30/31 */
	0x40400, /* boot mode nibble */
	0x33194, /* trace/crashlog debug reg */
	/* clock gates / power domains (clk-rapuyama, power_resources) */
	0x33008, 0x33010, 0x33038, 0x33058, 0x33080, 0x330b0, 0x330d0,
	0x33130, 0x33150, 0x33178, 0x33180,
	/* secondary irq banks: raw state, enabled mask */
	0x33408, 0x33418, 0x33448, 0x33458, 0x33488, 0x33498,
	0x334c8, 0x334d8, 0x33508, 0x33518,
	/* gpio levels, banks 0-4 */
	0x319f0, 0x31a00, 0x31a10, 0x31a20, 0x31a30,
};

#define RY_FR_NREGS	ARRAY_SIZE(rapuyama_fr_regs)
#define RY_FR_SLOT	((2 + RY_FR_NREGS) * 4)
#define RY_FR_NSLOTS	((0x100000u - RY_FR_OFF - RY_FR_DATA) / RY_FR_SLOT)

static u32 rapuyama_fr_seq;
/* bits that moved since the heartbeat last printed them, per fr reg */
static u32 rapuyama_fr_prev[RY_FR_NREGS], rapuyama_fr_chg[RY_FR_NREGS];

static void rapuyama_flight_record(void)
{
	void __iomem *fr = (void __iomem *)RY_RAMCON_VIRT + RY_FR_OFF;
	void __iomem *slot;
	unsigned int i;

	if (!rapuyama_fr_seq) {
		writel(0, fr + 4);
		writel(RY_FR_NREGS, fr + 8);
		writel(RY_FR_NSLOTS, fr + 12);
		for (i = 0; i < RY_FR_NREGS; i++)
			writel(rapuyama_fr_regs[i], fr + 16 + i * 4);
		writel(RY_FR_MAGIC, fr);
	}
	slot = fr + RY_FR_DATA + (rapuyama_fr_seq % RY_FR_NSLOTS) * RY_FR_SLOT;
	writel(rapuyama_fr_seq, slot);
	writel(jiffies_to_msecs(jiffies - INITIAL_JIFFIES), slot + 4);
	for (i = 0; i < RY_FR_NREGS; i++) {
		u32 v = readl_relaxed(RY_BUS(rapuyama_fr_regs[i]));

		writel(v, slot + 8 + i * 4);
		if (rapuyama_fr_seq)
			rapuyama_fr_chg[i] |= v ^ rapuyama_fr_prev[i];
		rapuyama_fr_prev[i] = v;
	}
	writel(++rapuyama_fr_seq, fr + 4);
}

/* the counters (first two regs) always move, skip them */
static void rapuyama_raw_reg_changes(void)
{
	char buf[384];
	unsigned int i;
	int len = 0;

	for (i = 2; i < RY_FR_NREGS; i++) {
		if (!rapuyama_fr_chg[i])
			continue;
		len += scnprintf(buf + len, sizeof(buf) - len, " %x:%08x",
				 rapuyama_fr_regs[i], rapuyama_fr_chg[i]);
		rapuyama_fr_chg[i] = 0;
	}
	buf[len] = 0;
	rapuyama_raw_printf("e6: bits (reg:changed)%s\n", buf);
}

/* called from the ms timer irq every ~100 ms */
void rapuyama_early_wdt_kick(void)
{
	char why[48];

	if (!rapuyama_mapped)
		return;
	rapuyama_alive();
	rapuyama_flight_record();

	/* ALWAYS feed the SoC watchdog from hardirq context - this is a
	 * hardware backup that can't be blocked by thread scheduling.
	 * The watchdog driver also feeds it, but redundant writes are safe.
	 */
	writel_relaxed(RY_SOC_WDT_VAL, RY_BUS(RY_SOC_WDT));

	if (READ_ONCE(rapuyama_early_wdt_done)) {
		rapuyama_stall_check();
		return;
	}

	if (!rapuyama_wdt_feed_s ||
	    ++rapuyama_wdt_kicks < rapuyama_wdt_feed_s * 10)
		return;

	WRITE_ONCE(rapuyama_early_wdt_done, true);
	snprintf(why, sizeof(why), "no watchdog driver after %u s",
		 rapuyama_wdt_feed_s);
	rapuyama_dump_stuck(why);
}

void rapuyama_early_wdt_stop(void)
{
	WRITE_ONCE(rapuyama_early_wdt_done, true);
	pr_info("rapuyama: early soc watchdog feeding stopped, driver took over (stall detector %u s, ramcon heartbeat 10 s)\n",
		rapuyama_stall_s);
}

/*
 * shared ram watch. symbian maps these ram ranges in its 0xc6 hw window, so
 * something other than the arm (modem / secure side) uses them. the dt
 * reserves them (still mapped, see e6.dts). every 2 s each 4 KB page gets a checksum, a page that
 * changed since the last pass goes to the log with its first words. nothing
 * here = nobody writes there and the random deaths are something else.
 * e6.shmwatch=0 turns it off
 */
static const struct {
	phys_addr_t base;
	size_t size;
} rapuyama_shm[] = {
	{ 0x11525000, 0x2000 },
	{ 0x1f8b0000, 0x44d000 },
	{ 0x1ffe0000, 0x20000 },
};

#define RY_SHM_PERIOD (2 * HZ)
#define RY_SHM_MAXLOG 8	/* changed pages logged per pass */

static bool rapuyama_shm_on = true;
static u32 *rapuyama_shm_virt[ARRAY_SIZE(rapuyama_shm)];
static u32 *rapuyama_shm_sum[ARRAY_SIZE(rapuyama_shm)];
static struct debugfs_blob_wrapper rapuyama_shm_blob[ARRAY_SIZE(rapuyama_shm)];
static unsigned int rapuyama_shm_pass;
static struct delayed_work rapuyama_shm_work;

static int __init rapuyama_shmwatch_param(char *s)
{
	return kstrtobool(s, &rapuyama_shm_on);
}
early_param("e6.shmwatch", rapuyama_shmwatch_param);

static u32 rapuyama_shm_page_sum(const u32 *p)
{
	u32 sum = 0;
	int i;

	for (i = 0; i < PAGE_SIZE / 4; i++)
		sum = rol32(sum, 5) ^ READ_ONCE(p[i]);
	return sum;
}

static void rapuyama_shm_scan(struct work_struct *work)
{
	unsigned int r, pg, npg, changed = 0, nonzero;
	const u32 *p;
	phys_addr_t pa;
	u32 sum;

	for (r = 0; r < ARRAY_SIZE(rapuyama_shm); r++) {
		if (!rapuyama_shm_virt[r])
			continue;
		npg = rapuyama_shm[r].size >> PAGE_SHIFT;
		nonzero = 0;
		/* cached linear map: drop our lines so we see what the other
		 * side wrote. we never write here, so nothing dirty is lost */
		__cpuc_flush_dcache_area(rapuyama_shm_virt[r], rapuyama_shm[r].size);
		for (pg = 0; pg < npg; pg++) {
			p = rapuyama_shm_virt[r] + (pg << PAGE_SHIFT) / 4;
			sum = rapuyama_shm_page_sum(p);
			if (sum)
				nonzero++;
			pa = rapuyama_shm[r].base + (pg << PAGE_SHIFT);
			if (rapuyama_shm_pass && sum != rapuyama_shm_sum[r][pg] &&
			    changed++ < RY_SHM_MAXLOG)
				pr_info("rapuyama: shm %pa changed: %08x %08x %08x %08x %08x %08x %08x %08x\n",
					&pa, p[0], p[1], p[2], p[3],
					p[4], p[5], p[6], p[7]);
			rapuyama_shm_sum[r][pg] = sum;
			cond_resched();
		}
		if (!rapuyama_shm_pass)
			pr_info("rapuyama: shm %pa +%zx: %u of %u pages not zero\n",
				&rapuyama_shm[r].base, rapuyama_shm[r].size,
				nonzero, npg);
	}
	if (changed > RY_SHM_MAXLOG)
		pr_info("rapuyama: shm %u more pages changed\n",
			changed - RY_SHM_MAXLOG);
	rapuyama_shm_pass++;
	schedule_delayed_work(&rapuyama_shm_work, RY_SHM_PERIOD);
}

static ssize_t rapuyama_shm_read(struct file *file, char __user *buf,
			       size_t count, loff_t *pos)
{
	struct debugfs_blob_wrapper *blob = file->private_data;

	return simple_read_from_buffer(buf, count, pos, blob->data, blob->size);
}

static const struct file_operations rapuyama_shm_fops = {
	.open = simple_open,
	.read = rapuyama_shm_read,
	.llseek = default_llseek,
};

static int __init rapuyama_shm_init(void)
{
	struct dentry *dir;
	unsigned int r;

	if (!rapuyama_shm_on || !of_machine_is_compatible("nokia,rapuyama"))
		return 0;
	dir = debugfs_create_dir("rapuyama-shm", NULL);
	for (r = 0; r < ARRAY_SIZE(rapuyama_shm); r++) {
		/* reserved but mapped ram: this is just its linear map */
		rapuyama_shm_virt[r] = memremap(rapuyama_shm[r].base,
						rapuyama_shm[r].size,
						MEMREMAP_WB);
		rapuyama_shm_sum[r] = kcalloc(rapuyama_shm[r].size >> PAGE_SHIFT,
					      sizeof(u32), GFP_KERNEL);
		if (!rapuyama_shm_virt[r] || !rapuyama_shm_sum[r]) {
			pr_warn("rapuyama: shm %pa not watched\n",
				&rapuyama_shm[r].base);
			if (rapuyama_shm_virt[r])
				memunmap(rapuyama_shm_virt[r]);
			kfree(rapuyama_shm_sum[r]);
			rapuyama_shm_virt[r] = NULL;
		}
		if (rapuyama_shm_virt[r] && !IS_ERR_OR_NULL(dir)) {
			char name[16];

			snprintf(name, sizeof(name), "%08x", (u32)rapuyama_shm[r].base);
			rapuyama_shm_blob[r].data = rapuyama_shm_virt[r];
			rapuyama_shm_blob[r].size = rapuyama_shm[r].size;
			debugfs_create_file(name, 0400, dir, &rapuyama_shm_blob[r],
					    &rapuyama_shm_fops);
		}
	}
	INIT_DELAYED_WORK(&rapuyama_shm_work, rapuyama_shm_scan);
	schedule_delayed_work(&rapuyama_shm_work, 0);
	return 0;
}
late_initcall(rapuyama_shm_init);

/*
 * isasw handoff block from the booter (runblob_ldd.cpp SaveIsaHandoff): the
 * cellular mcu sw runs on this cpu through karap's irq hooks, these are the
 * hook addresses + the symbian mmu state they ran under. only logged for now,
 * hosting the hooks needs to know what they are first
 */
#define RY_ISAH_PHYS	0x1f0f1000	/* between ramcon and the flight recorder */
#define RY_ISAH_SIZE	0x5000
#define RY_ISAH_L1	0x400		/* word index of the 16 kb L1 copy */
#define RY_ISAH_MAGIC	0x48415349
#define RY_CALIB	0x100		/* word index of the phase calibration */
#define RY_CALIB_MAGIC	0x424c4143	/* 'CALB' */

static int __init rapuyama_isah_init(void)
{
	const u32 *h;
	int i;

	if (!of_machine_is_compatible("nokia,rapuyama"))
		return 0;
	h = memremap(RY_ISAH_PHYS, RY_ISAH_SIZE, MEMREMAP_WB);
	if (!h)
		return 0;
	if (h[RY_CALIB] == RY_CALIB_MAGIC) {
		const u32 *c = h + RY_CALIB;

		rapuyama_calib_fc_handoff = c[2];
		rapuyama_calib_fc_req = c[3];
		rapuyama_calib_reqs = c[4];
		if (c[1] >= 2 && c[5] != 0xffffffff && c[5] != 0) {
			rapuyama_calib_due_ms = (int)(c[6] - c[7]) * (int)c[8] / 1000;
			rapuyama_calib_sym_ms = div_u64((u64)c[7] * c[8], 1000);
			rapuyama_calib_due_ok = true;
		}
		if (c[1] >= 2)
			pr_info("rapuyama: calib: secure timer state %x trigger %u tick %u period %u us -> next request %d ms after the handoff\n",
				c[5], c[6], c[7], c[8], rapuyama_calib_due_ms);
		if (c[1] >= 3 && c[9] != 1) {
			rapuyama_calib_kick_fc = c[10];
			rapuyama_calib_kick_ok = true;
			pr_info("rapuyama: calib: request sent at the handoff -> %d, %u ms before the handoff\n",
				(int)c[9], (u32)(((u64)(c[2] - c[10]) * 1000) >> 15));
		}
		WRITE_ONCE(rapuyama_calib_ok, true);
		if (c[4])
			pr_info("rapuyama: calib v%u: handoff fc=%08x, last secure request fc=%08x (%u ms before the handoff, %u sightings), cmd=%08x PAid=%08x\n",
				c[1], c[2], c[3],
				(u32)(((u64)(c[2] - c[3]) * 1000) >> 15), c[4],
				c[0x10], c[0x11]);
		else
			pr_info("rapuyama: calib v%u: handoff fc=%08x, no secure request seen\n",
				c[1], c[2]);
	}
	if (h[0] != RY_ISAH_MAGIC) {
		pr_info("rapuyama: isah: no handoff block (%08x)\n", h[0]);
		goto out;
	}
	pr_info("rapuyama: isah v%u ttbr0=%08x ttbr1=%08x ttbcr=%08x dacr=%08x ctx=%08x sctlr=%08x prrr=%08x nmrr=%08x\n",
		h[1], h[2], h[3], h[4], h[5], h[6], h[7], h[8], h[9]);
	for (i = 0; i < 6; i++)
		pr_info("rapuyama: isah cb[%d]=%08x pa=%08x\n", i, h[0x10 + i], h[0xf0 + i]);
	pr_info("rapuyama: isah obj=%08x pa=%08x\n", h[0x18], h[0xf6]);
	for (i = 0; i < 0x40; i += 8)
		pr_info("rapuyama: isah obj+%02x: %08x %08x %08x %08x %08x %08x %08x %08x\n",
			i * 4, h[0x20 + i], h[0x21 + i], h[0x22 + i], h[0x23 + i],
			h[0x24 + i], h[0x25 + i], h[0x26 + i], h[0x27 + i]);
	for (i = 0; i < 0x40; i += 8)
		pr_info("rapuyama: isah sisa+%02x: %08x %08x %08x %08x %08x %08x %08x %08x\n",
			i * 4, h[0x60 + i], h[0x61 + i], h[0x62 + i], h[0x63 + i],
			h[0x64 + i], h[0x65 + i], h[0x66 + i], h[0x67 + i]);
	pr_info("rapuyama: isah superpage=%08x machinedata=%08x\n", h[0xa0], h[0xa1]);
	for (i = 0; i < 0x40; i += 8)
		pr_info("rapuyama: isah md+%02x: %08x %08x %08x %08x %08x %08x %08x %08x\n",
			i * 4, h[0xb0 + i], h[0xb1 + i], h[0xb2 + i], h[0xb3 + i],
			h[0xb4 + i], h[0xb5 + i], h[0xb6 + i], h[0xb7 + i]);
	/* the kernel L1 stays in the block (byte 0x1000), print only the sections
	 * the hooks live in so the log stays short */
	for (i = 0; i < 7; i++) {
		u32 va = i < 6 ? h[0x10 + i] : h[0x18];

		if (va)
			pr_info("rapuyama: isah l1[%03x]=%08x (va %08x)\n",
				va >> 20, h[RY_ISAH_L1 + (va >> 20)], va);
	}
out:
	memunmap((void *)h);
	return 0;
}
late_initcall(rapuyama_isah_init);

static const char *const rapuyama_dt_compat[] __initconst = {
	"nokia,rapuyama",
	NULL
};

DT_MACHINE_START(RAPUYAMA, "Nokia RapuYama")
	.dt_compat	= rapuyama_dt_compat,
	.map_io		= rapuyama_map_io,
	.init_early	= rapuyama_init_early,
MACHINE_END