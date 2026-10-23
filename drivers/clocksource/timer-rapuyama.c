// SPDX-License-Identifier: GPL-2.0
/*
 * Nokia RapuYama timers (N8/E6/E7/C7 application ASIC)
 *
 * No public documentation exists, reversed from the E6 RM-609 firmware:
 *
 * - ms timer (karap.dll "BindMsTick", power.dll idle code): a down counter
 *   clocked from the 38.4 MHz reference. MST_COUNT sets/reads the current
 *   count, MST_RELOAD is loaded when it expires and the interrupt fires.
 *   Symbian runs it at 0x9600 (1 ms) and rewrites MST_RELOAD in the isr,
 *   the idle code writes n * 0x9600 to MST_RELOAD to sleep n ticks.
 *   e6.oneshot=1: oneshot too (high res timers, no 1000 irqs/s): the next
 *   event goes into MST_COUNT, MST_RELOAD parks at 100 ms so an irq still
 *   comes at least that often (soc watchdog, flight recorder, stall check)
 *
 * - free running counter (ekern NKern::FastCounter, frequency 0x8000):
 *   32 bit, 32768 Hz, any write to the latch register snapshots it.
 */

#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/sched_clock.h>

#define MST_COUNT	0x24
#define MST_RELOAD	0x28

#define FC_VALUE	0x00
#define FC_LATCH	0x04

#define MST_DEF_RATE	38400000
#define FC_DEF_RATE	32768

/*
 * ekern writes its own virtual address of the latch reg as the value, probably
 * don't care, but write the exact same thing until that is checked
 */
#define FC_LATCH_VAL	0xc6033354

#define MST_BASE_HZ	1000
#define MST_ONESHOT_MAX_MS	100
#define FC_KICK_TICKS	(FC_DEF_RATE / 10)	/* 100 ms of the fast counter */

static void __iomem *mst_base;
static void __iomem *fc_base;
static u32 mst_period;
static u32 mst_tick_accumulator;
static u32 mst_park;		/* MST_ONESHOT_MAX_MS in timer cycles */
static u32 fc_last_kick;
static bool mst_oneshot;

static int __init rapuyama_oneshot_param(char *s)
{
	return kstrtobool(s, &mst_oneshot);
}
early_param("e6.oneshot", rapuyama_oneshot_param);

static u64 notrace rapuyama_fc_read(void)
{
	writel_relaxed(FC_LATCH_VAL, fc_base + FC_LATCH);
	return readl_relaxed(fc_base + FC_VALUE);
}

static u64 rapuyama_fc_cs_read(struct clocksource *cs)
{
	return rapuyama_fc_read();
}

static struct clocksource rapuyama_fc_cs = {
	.name	= "rapuyama-fc",
	.rating	= 200,
	.read	= rapuyama_fc_cs_read,
	.mask	= CLOCKSOURCE_MASK(32),
	.flags	= CLOCK_SOURCE_IS_CONTINUOUS,
};

static int rapuyama_mst_set_periodic(struct clock_event_device *evt)
{
	mst_tick_accumulator = 0;
	writel_relaxed(mst_period, mst_base + MST_RELOAD);
	writel_relaxed(mst_period, mst_base + MST_COUNT);
	return 0;
}

/*
 * the counter can't be stopped (symbian never does), so park it as far
 * away as it goes. the width is unknown, 0xffffffff is >= 111 s at 38.4 MHz
 * if it is 32 bit and still just a spurious tick if it is narrower
 */
static int rapuyama_mst_shutdown(struct clock_event_device *evt)
{
	writel_relaxed(~0u, mst_base + MST_RELOAD);
	writel_relaxed(~0u, mst_base + MST_COUNT);
	return 0;
}

static int rapuyama_mst_set_oneshot(struct clock_event_device *evt)
{
	writel_relaxed(mst_park, mst_base + MST_RELOAD);
	writel_relaxed(mst_park, mst_base + MST_COUNT);
	return 0;
}

static int rapuyama_mst_next_event(unsigned long delta,
				   struct clock_event_device *evt)
{
	writel_relaxed(mst_park, mst_base + MST_RELOAD);
	writel_relaxed(delta, mst_base + MST_COUNT);
	return 0;
}

static struct clock_event_device rapuyama_mst_ced = {
	.name			= "rapuyama-mst",
	.features		= CLOCK_EVT_FEAT_PERIODIC,
	.rating			= 300,
	.set_state_periodic	= rapuyama_mst_set_periodic,
	.set_state_shutdown	= rapuyama_mst_shutdown,
};

#ifdef CONFIG_ARCH_RAPUYAMA
void rapuyama_checkpoint(int n);
void rapuyama_early_wdt_kick(void);
#else
static inline void rapuyama_checkpoint(int n) { }
static inline void rapuyama_early_wdt_kick(void) { }
#endif

static u32 mst_wdt_ticks;

static irqreturn_t rapuyama_mst_irq(int irq, void *dev_id)
{
	struct clock_event_device *evt = dev_id;

	/* checkpoint 11: a real timer interrupt arrived (e6.stop=11) */
	rapuyama_checkpoint(11);

	/* feed the soc watchdog every ~100 ms until the wdt driver is up */
	if (clockevent_state_oneshot(evt)) {
		u32 fc = rapuyama_fc_read();

		writel_relaxed(mst_park, mst_base + MST_RELOAD);	/* ack */
		if (fc - fc_last_kick >= FC_KICK_TICKS) {
			fc_last_kick = fc;
			rapuyama_early_wdt_kick();
		}
		if (evt->event_handler)
			evt->event_handler(evt);
		return IRQ_HANDLED;
	}
	if (++mst_wdt_ticks >= 100) {
		mst_wdt_ticks = 0;
		rapuyama_early_wdt_kick();
	}

	/*
	 * A pending hardware interrupt can outlive a clockevent state change.
	 * Acknowledge and park it, but only deliver ticks while the core has
	 * this device in periodic mode.
	 */
	if (!clockevent_state_periodic(evt)) {
		writel_relaxed(~0u, mst_base + MST_RELOAD);
		return IRQ_HANDLED;
	}

	/* The firmware ISR reloads before NTimerQ::Tick; this also acks the IRQ. */
	writel_relaxed(mst_period, mst_base + MST_RELOAD);
	mst_tick_accumulator += HZ;
	if (mst_tick_accumulator < MST_BASE_HZ)
		return IRQ_HANDLED;
	mst_tick_accumulator -= MST_BASE_HZ;

	if (WARN_ON_ONCE(!evt->event_handler))
		return IRQ_HANDLED;
	evt->event_handler(evt);
	return IRQ_HANDLED;
}

static unsigned long __init rapuyama_rate(struct device_node *np,
					  const char *name, unsigned long def)
{
	struct clk *clk = of_clk_get_by_name(np, name);
	unsigned long rate;

	if (IS_ERR(clk))
		return def;
	rate = clk_get_rate(clk);
	clk_put(clk);
	return rate ? rate : def;
}

static int __init rapuyama_timer_init(struct device_node *np)
{
	unsigned long mst_rate, fc_rate;
	int irq, ret;

	if (HZ > MST_BASE_HZ) {
		pr_err("rapuyama-timer: HZ=%d exceeds the 1 ms timer resolution\n",
		       HZ);
		return -EINVAL;
	}

	mst_base = of_iomap(np, 0);
	fc_base = of_iomap(np, 1);
	if (!mst_base || !fc_base) {
		pr_err("rapuyama-timer: can't map registers\n");
		return -ENXIO;
	}

	irq = irq_of_parse_and_map(np, 0);
	if (!irq) {
		pr_err("rapuyama-timer: can't map irq\n");
		return -EINVAL;
	}

	mst_rate = rapuyama_rate(np, "ref", MST_DEF_RATE);
	fc_rate = rapuyama_rate(np, "fc", FC_DEF_RATE);

	rapuyama_early_wdt_kick();

	sched_clock_register(rapuyama_fc_read, 32, fc_rate);
	ret = clocksource_register_hz(&rapuyama_fc_cs, fc_rate);
	if (ret)
		return ret;

	/*
	 * The only period confirmed on hardware is 1 ms (0x9600 at 38.4 MHz).
	 * Keep the down-counter in that range and divide interrupts to Linux HZ.
	 */
	mst_period = DIV_ROUND_CLOSEST(mst_rate, MST_BASE_HZ);
	ret = request_irq(irq, rapuyama_mst_irq, IRQF_TIMER | IRQF_IRQPOLL,
			  "rapuyama-mst", &rapuyama_mst_ced);
	if (ret)
		return ret;

	rapuyama_mst_ced.irq = irq;
	rapuyama_mst_ced.cpumask = cpu_possible_mask;
	mst_park = mst_rate / 1000 * MST_ONESHOT_MAX_MS;
	if (mst_oneshot) {
		rapuyama_mst_ced.features |= CLOCK_EVT_FEAT_ONESHOT;
		rapuyama_mst_ced.set_state_oneshot = rapuyama_mst_set_oneshot;
		rapuyama_mst_ced.set_next_event = rapuyama_mst_next_event;
		/* min ~25 us: the core doesn't ask for less than it can arm */
		clockevents_config_and_register(&rapuyama_mst_ced, mst_rate,
						0x400, mst_park);
	} else {
		clockevents_register_device(&rapuyama_mst_ced);
	}

	pr_info("rapuyama-timer: ms timer %lu Hz, %u Hz base / %u Hz Linux tick (irq %d), counter %lu Hz%s\n",
		mst_rate, MST_BASE_HZ, HZ, irq, fc_rate,
		mst_oneshot ? ", oneshot" : "");
	return 0;
}
TIMER_OF_DECLARE(rapuyama, "nokia,rapuyama-timer", rapuyama_timer_init);