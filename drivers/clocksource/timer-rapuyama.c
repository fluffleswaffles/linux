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

static void __iomem *mst_base;
static void __iomem *fc_base;
static u32 mst_period;

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

static struct clock_event_device rapuyama_mst_ced = {
	.name			= "rapuyama-mst",
	.features		= CLOCK_EVT_FEAT_PERIODIC,
	.rating			= 300,
	.set_state_periodic	= rapuyama_mst_set_periodic,
	.set_state_shutdown	= rapuyama_mst_shutdown,
};

#ifdef CONFIG_ARCH_RAPUYAMA
void rapuyama_checkpoint(int n);
#else
static inline void rapuyama_checkpoint(int n) { }
#endif

static irqreturn_t rapuyama_mst_irq(int irq, void *dev_id)
{
	struct clock_event_device *evt = dev_id;

	/* checkpoint 11: a real timer interrupt arrived (e6.stop=11) */
	rapuyama_checkpoint(11);

	/* the firmware isr does this before NTimerQ::Tick, acts as the ack */
	if (clockevent_state_periodic(evt))
		writel_relaxed(mst_period, mst_base + MST_RELOAD);
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

	sched_clock_register(rapuyama_fc_read, 32, fc_rate);
	ret = clocksource_register_hz(&rapuyama_fc_cs, fc_rate);
	if (ret)
		return ret;

	mst_period = DIV_ROUND_CLOSEST(mst_rate, HZ);
	ret = request_irq(irq, rapuyama_mst_irq, IRQF_TIMER | IRQF_IRQPOLL,
			  "rapuyama-mst", &rapuyama_mst_ced);
	if (ret)
		return ret;

	rapuyama_mst_ced.irq = irq;
	rapuyama_mst_ced.cpumask = cpu_possible_mask;
	clockevents_register_device(&rapuyama_mst_ced);

	pr_info("rapuyama-timer: ms timer %lu Hz (irq %d), counter %lu Hz\n",
		mst_rate, irq, fc_rate);
	return 0;
}
TIMER_OF_DECLARE(rapuyama, "nokia,rapuyama-timer", rapuyama_timer_init);