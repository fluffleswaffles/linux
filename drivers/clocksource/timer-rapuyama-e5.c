// SPDX-License-Identifier: GPL-2.0
/*
 * Nokia E5-00 (RapuYama) system tick and fast counter
 *
 * Reversed from the E5 RM-632 101.003 ROM and checked with read-only probes on
 * a live phone (docs/devices/e5-00/hw-map.md, plan section 7.6). Not the E6
 * ms timer: that one is irq 242 at 0x200337a0 (timer-rapuyama.c).
 *
 * - tick, irq 35: a hardware periodic down counter at 0x20040180 + 0x24 that
 *   reloads from 0x1e00 = 7680. At ~7.68 MHz (38.4 MHz / 5, measured
 *   7.66 MHz +-5 %) that is exactly 1 ms. Symbian arms it by writing 0x1e00 to
 *   the counter (ROM 0x8009e2a4) and its isr (ROM 0x8009d778) re-arms by
 *   polling 0x20040180 + 0x44 until it reads 0, then writing 0x1e00 to it
 * - fast counter (NKern::FastCounter, 32768 Hz): same as timer-rapuyama.c
 *
 * Unverified until a kernel runs on the phone: that the write to +0x44
 * acknowledges the irq line, and the trigger type of irq 35. Symbian has
 * already run the block init (0x20033600 and 0x20040180 +4/+8) at handoff.
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

#define TICK_COUNT	0x24	/* down counter, write the reload to (re)start */
#define TICK_ACK	0x44	/* reads non-zero until the last write synced */
#define TICK_RELOAD	0x1e00	/* 7680 counts of the ~7.68 MHz clock = 1 ms */
#define TICK_SYNC_LOOPS	100000

#define FC_VALUE	0x00
#define FC_LATCH	0x04
#define FC_DEF_RATE	32768
/* ekern writes its own virtual address of the latch reg, keep the same value */
#define FC_LATCH_VAL	0xc6033354

#define TICK_BASE_HZ	1000
#define WDT_KICK_TICKS	100

#ifdef CONFIG_ARCH_RAPUYAMA
void rapuyama_early_wdt_kick(void);
#else
static inline void rapuyama_early_wdt_kick(void) { }
#endif

static void __iomem *tick_base;
static void __iomem *fc_base;
static u32 tick_accumulator;
static u32 tick_wdt_ticks;

static u64 notrace rapuyama_e5_fc_read(void)
{
	writel_relaxed(FC_LATCH_VAL, fc_base + FC_LATCH);
	return readl_relaxed(fc_base + FC_VALUE);
}

static u64 rapuyama_e5_fc_cs_read(struct clocksource *cs)
{
	return rapuyama_e5_fc_read();
}

static struct clocksource rapuyama_e5_fc_cs = {
	.name	= "rapuyama-e5-fc",
	.rating	= 200,
	.read	= rapuyama_e5_fc_cs_read,
	.mask	= CLOCKSOURCE_MASK(32),
	.flags	= CLOCK_SOURCE_IS_CONTINUOUS,
};

/* the isr's sequence: wait for the previous write to sync, then reload */
static void rapuyama_e5_tick_rearm(void)
{
	int n = TICK_SYNC_LOOPS;

	while (readl_relaxed(tick_base + TICK_ACK) && --n)
		cpu_relax();
	writel_relaxed(TICK_RELOAD, tick_base + TICK_ACK);
}

static int rapuyama_e5_set_periodic(struct clock_event_device *evt)
{
	tick_accumulator = 0;
	writel_relaxed(TICK_RELOAD, tick_base + TICK_COUNT);
	return 0;
}

/*
 * The counter cannot be stopped (symbian never does): leave it running, the
 * isr keeps acknowledging it and only delivers ticks in periodic mode
 */
static int rapuyama_e5_shutdown(struct clock_event_device *evt)
{
	return 0;
}

static struct clock_event_device rapuyama_e5_ced = {
	.name			= "rapuyama-e5-tick",
	.features		= CLOCK_EVT_FEAT_PERIODIC,
	.rating			= 300,
	.set_state_periodic	= rapuyama_e5_set_periodic,
	.set_state_shutdown	= rapuyama_e5_shutdown,
};

static irqreturn_t rapuyama_e5_irq(int irq, void *dev_id)
{
	struct clock_event_device *evt = dev_id;

	rapuyama_e5_tick_rearm();

	/* feed the soc watchdog until the wdt driver is up */
	if (++tick_wdt_ticks >= WDT_KICK_TICKS) {
		tick_wdt_ticks = 0;
		rapuyama_early_wdt_kick();
	}

	if (!clockevent_state_periodic(evt))
		return IRQ_HANDLED;

	/* hardware ticks at 1 kHz, deliver HZ of them */
	tick_accumulator += HZ;
	if (tick_accumulator < TICK_BASE_HZ)
		return IRQ_HANDLED;
	tick_accumulator -= TICK_BASE_HZ;

	if (WARN_ON_ONCE(!evt->event_handler))
		return IRQ_HANDLED;
	evt->event_handler(evt);
	return IRQ_HANDLED;
}

static unsigned long __init rapuyama_e5_fc_rate(struct device_node *np)
{
	struct clk *clk = of_clk_get_by_name(np, "fc");
	unsigned long rate;

	if (IS_ERR(clk))
		return FC_DEF_RATE;
	rate = clk_get_rate(clk);
	clk_put(clk);
	return rate ? rate : FC_DEF_RATE;
}

static int __init rapuyama_e5_timer_init(struct device_node *np)
{
	unsigned long fc_rate;
	int irq, ret;

	if (HZ > TICK_BASE_HZ) {
		pr_err("rapuyama-e5-timer: HZ=%d exceeds the 1 ms tick\n", HZ);
		return -EINVAL;
	}

	tick_base = of_iomap(np, 0);
	fc_base = of_iomap(np, 1);
	if (!tick_base || !fc_base) {
		pr_err("rapuyama-e5-timer: can't map registers\n");
		return -ENXIO;
	}

	irq = irq_of_parse_and_map(np, 0);
	if (!irq) {
		pr_err("rapuyama-e5-timer: can't map irq\n");
		return -EINVAL;
	}

	fc_rate = rapuyama_e5_fc_rate(np);

	rapuyama_early_wdt_kick();

	sched_clock_register(rapuyama_e5_fc_read, 32, fc_rate);
	ret = clocksource_register_hz(&rapuyama_e5_fc_cs, fc_rate);
	if (ret)
		return ret;

	ret = request_irq(irq, rapuyama_e5_irq, IRQF_TIMER | IRQF_IRQPOLL,
			  "rapuyama-e5-tick", &rapuyama_e5_ced);
	if (ret)
		return ret;

	rapuyama_e5_ced.irq = irq;
	rapuyama_e5_ced.cpumask = cpu_possible_mask;
	clockevents_register_device(&rapuyama_e5_ced);

	pr_info("rapuyama-e5-timer: 1 kHz base / %u Hz Linux tick (irq %d), counter %lu Hz\n",
		HZ, irq, fc_rate);
	return 0;
}

TIMER_OF_DECLARE(rapuyama_e5, "nokia,rapuyama-e5-timer", rapuyama_e5_timer_init);
