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
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdesc.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kernel_stat.h>
#include <linux/irqflags.h>
#include <linux/console.h>
#include <linux/kmsg_dump.h>
#include <linux/sched.h>
#include <linux/sched/debug.h>
#include <linux/export.h>
#include <linux/signal.h>
#include <asm/bug.h>
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
static void rapuyama_alive(void)
{
	if (++rapuyama_alive_ticks < 100)
		return;
	rapuyama_alive_ticks = 0;
	rapuyama_raw_printf("e6: alive, uptime %u s\n",
			    jiffies_to_msecs(jiffies - INITIAL_JIFFIES) / 1000);
}

/* called from the ms timer irq every ~100 ms */
void rapuyama_early_wdt_kick(void)
{
	char why[48];

	if (!rapuyama_mapped)
		return;
	rapuyama_alive();
	if (READ_ONCE(rapuyama_early_wdt_done)) {
		rapuyama_stall_check();
		return;
	}

	writel_relaxed(RY_SOC_WDT_VAL, RY_BUS(RY_SOC_WDT));

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

static const char *const rapuyama_dt_compat[] __initconst = {
	"nokia,rapuyama",
	NULL
};

DT_MACHINE_START(RAPUYAMA, "Nokia RapuYama")
	.dt_compat	= rapuyama_dt_compat,
	.map_io		= rapuyama_map_io,
	.init_early	= rapuyama_init_early,
MACHINE_END