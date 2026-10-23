// SPDX-License-Identifier: GPL-2.0
/*
 * Nokia RapuYama watchdogs (N8/E6/E7/C7)
 *
 * No public documentation exists, reversed from the E6 RM-609 firmware
 * (power.dll, sisa_wrapper.dll, karap.dll). Symbian kicks two things every
 * 3 s and can't stop either of them:
 *
 * - the PMIC watchdog: register 0x17 of CBUS device 1, a Retu style PMIC
 *   (same register map as on the N800). The value is the timeout in seconds,
 *   symbian writes 48. Writing 0 powers the phone off right away (that is how
 *   sisa_wrapper does "force power off"), so 0 is never written here.
 * - a SoC counter at 0x200336c4 that gets 0x3fffffff on every kick.
 *
 * The CBUS master (karap 0x800b8414/0x800b8494, base pointer 0x20030704 in
 * the firmware, offsets here are from 0x20030700):
 *   0x04 enable (1 around a transfer)   0x08 busy
 *   0x18 read cmd  (dev << 6 | reg | 0x20)
 *   0x1c write cmd (dev << 6 | reg)     0x20 data
 *   0x2c bit count - 1                  0x3c 1 before a write
 * (first version had all of these 4 bytes low, which wrote the data into the
 * write cmd reg and powered the phone off)
 *
 * Restart (optional third reg, rebootdriver.ldd): write 2 to 0x20033114.
 *
 * Both run from the moment symbian booted us, so the device is marked as
 * running and the watchdog core keeps it fed until userspace takes over.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/soc/rapuyama-pmic.h>
#include <linux/spinlock.h>
#include <linux/watchdog.h>
#include <linux/workqueue.h>

#define CBUS_EN		0x04
#define CBUS_BUSY	0x08
#define CBUS_RD_CMD	0x18
#define CBUS_WR_CMD	0x1c
#define CBUS_DATA	0x20
#define CBUS_BITS	0x2c
#define CBUS_WR_MODE	0x3c
#define CBUS_READ	0x20
#define CBUS_TRIES	0x800

#define PMIC_DEV	1
#define PMIC_REG_WDT	0x17
#define PMIC_WDT_MAX	63	/* 6 bit seconds, like the retu */
#define PMIC_WDT_DEF	48	/* what symbian uses */

#define SOC_WDT_RELOAD	0x3fffffff
/*
 * symbian kicks every 3 s. the soc counter runs out much sooner than the
 * pmic one: the first linux boot that kicked only every 24 s (timeout / 2)
 * got a soc reset ~13 s in, so the core has to ping at symbian's pace
 */
#define RY_HW_HEARTBEAT_MS	3000

struct rapuyama_wdt {
	struct watchdog_device wdd;
	void __iomem *cbus;
	void __iomem *soc;
	void __iomem *reset;	/* optional */
	spinlock_t lock;
	struct delayed_work dump;
	u16 pmic_last[2][32];
};

#ifdef CONFIG_ARCH_RAPUYAMA
void rapuyama_early_wdt_stop(void);	/* arch/arm/mach-rapuyama */
void rapuyama_wdt_pinged(void);		/* stall detector there */
#else
static inline void rapuyama_early_wdt_stop(void) { }
static inline void rapuyama_wdt_pinged(void) { }
#endif

/* rebootdriver.ldd 0x800cd0bc: irqs off, write 2, spin */
#define SOC_RESET_VAL	2

static int rapuyama_cbus_wait(struct rapuyama_wdt *rw)
{
	int n = CBUS_TRIES;

	udelay(1);
	while (readl(rw->cbus + CBUS_BUSY)) {
		if (!--n)
			return -ETIMEDOUT;
		udelay(1);
	}
	return 0;
}

/* rw->lock held */
static int __rapuyama_cbus_write(struct rapuyama_wdt *rw, u32 dev, u32 reg,
				 u16 val)
{
	int ret;

	writel(1, rw->cbus + CBUS_EN);
	writel(1, rw->cbus + CBUS_WR_MODE);
	writel(16 - 1, rw->cbus + CBUS_BITS);
	writel(val, rw->cbus + CBUS_DATA);
	writel(dev << 6 | reg, rw->cbus + CBUS_WR_CMD);
	ret = rapuyama_cbus_wait(rw);
	writel(0, rw->cbus + CBUS_EN);
	return ret;
}

static int rapuyama_cbus_write(struct rapuyama_wdt *rw, u32 dev, u32 reg,
			       u16 val)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&rw->lock, flags);
	ret = __rapuyama_cbus_write(rw, dev, reg, val);
	spin_unlock_irqrestore(&rw->lock, flags);
	return ret;
}

/* karap FUN_800b8414: no write mode, read flag 0x20 in the cmd */
/* rw->lock held */
static int __rapuyama_cbus_read(struct rapuyama_wdt *rw, u32 dev, u32 reg,
				u16 *val)
{
	int ret;

	writel(1, rw->cbus + CBUS_EN);
	writel(16 - 1, rw->cbus + CBUS_BITS);
	writel(dev << 6 | reg | CBUS_READ, rw->cbus + CBUS_RD_CMD);
	ret = rapuyama_cbus_wait(rw);
	if (!ret)
		*val = readl(rw->cbus + CBUS_DATA);
	writel(0, rw->cbus + CBUS_EN);
	return ret;
}

static int rapuyama_cbus_read(struct rapuyama_wdt *rw, u32 dev, u32 reg,
			      u16 *val)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&rw->lock, flags);
	ret = __rapuyama_cbus_read(rw, dev, reg, val);
	spin_unlock_irqrestore(&rw->lock, flags);
	return ret;
}

/*
 * gazoo adc (isasw hw_ad.c FUN_c62eff18): dev 1 reg 8, bit 10 = on, bits
 * 15:11 = hw channel, bits 9:0 = the result right after. the same steps as
 * the modem's energy management, under one lock with irqs off
 */
#define PMIC_REG_ADC	0x08
#define PMIC_ADC_ON	BIT(10)
#define PMIC_ADC_CH	GENMASK(15, 11)
#define PMIC_ADC_RES	GENMASK(9, 0)
#define PMIC_ADC_NCH	19		/* hw channels isasw maps to */

static int rapuyama_adc_read(struct rapuyama_wdt *rw, unsigned int ch, u16 *raw)
{
	unsigned long flags;
	u16 v;
	int ret;

	spin_lock_irqsave(&rw->lock, flags);
	ret = __rapuyama_cbus_read(rw, PMIC_DEV, PMIC_REG_ADC, &v);
	if (!ret)
		ret = __rapuyama_cbus_write(rw, PMIC_DEV, PMIC_REG_ADC,
					    v | PMIC_ADC_ON);
	if (!ret)
		ret = __rapuyama_cbus_write(rw, PMIC_DEV, PMIC_REG_ADC,
					    (v & ~PMIC_ADC_CH) | PMIC_ADC_ON |
					    FIELD_PREP(PMIC_ADC_CH, ch));
	if (!ret)
		ret = __rapuyama_cbus_read(rw, PMIC_DEV, PMIC_REG_ADC, &v);
	if (!ret) {
		*raw = FIELD_GET(PMIC_ADC_RES, v);
		ret = __rapuyama_cbus_write(rw, PMIC_DEV, PMIC_REG_ADC,
					    v & ~PMIC_ADC_ON);
	}
	spin_unlock_irqrestore(&rw->lock, flags);
	return ret;
}

static struct rapuyama_wdt *rapuyama_pmic;

/* for drivers/power/supply/rapuyama-battery.c */
int rapuyama_pmic_adc(unsigned int ch, u16 *raw)
{
	struct rapuyama_wdt *rw = READ_ONCE(rapuyama_pmic);

	if (!rw)
		return -EPROBE_DEFER;
	if (ch >= PMIC_ADC_NCH)
		return -EINVAL;
	return rapuyama_adc_read(rw, ch, raw);
}
EXPORT_SYMBOL_GPL(rapuyama_pmic_adc);

/* all hw channels raw, to tell which is which (vbat, btemp, bsi, vchar) */
static ssize_t adc_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct rapuyama_wdt *rw = dev_get_drvdata(dev);
	unsigned int ch;
	int len = 0;
	u16 raw;

	for (ch = 0; ch < PMIC_ADC_NCH; ch++) {
		if (rapuyama_adc_read(rw, ch, &raw))
			len += sysfs_emit_at(buf, len, "%2u -\n", ch);
		else
			len += sysfs_emit_at(buf, len, "%2u %4u\n", ch, raw);
	}
	return len;
}
static DEVICE_ATTR_RO(adc);

static struct attribute *rapuyama_wdt_attrs[] = {
	&dev_attr_adc.attr,
	NULL
};
ATTRIBUTE_GROUPS(rapuyama_wdt);

/*
 * the reset comes 240 s after symbian's last ~210 s service, maybe a pmic
 * timer: log every pmic reg that changed, period in s, 0 = off. only the regs
 * karap's cbus table (0x800bc268) knows
 */
static unsigned int pmic_dump;
module_param(pmic_dump, uint, 0444);

static const u32 rapuyama_pmic_regs[2] = {
	0x70dbe77f,	/* dev 1: 0-6 8-a d-11 13 14 16 17 1c-1e */
	0xff7ff7ff,	/* dev 2: 0-a c-16 18-1f */
};

static void rapuyama_pmic_dump(struct work_struct *work)
{
	struct rapuyama_wdt *rw = container_of(to_delayed_work(work),
					       struct rapuyama_wdt, dump);
	static bool first = true;
	char buf[512];
	int len = 0;
	u32 d, r;
	u16 v;

	for (d = 0; d < 2; d++) {
		for (r = 0; r < 32; r++) {
			if (!(rapuyama_pmic_regs[d] & BIT(r)))
				continue;
			if (rapuyama_cbus_read(rw, d + 1, r, &v))
				continue;
			if (!first && v == rw->pmic_last[d][r])
				continue;
			len += scnprintf(buf + len, sizeof(buf) - len,
					 " %u.%02x:%04x", d + 1, r, v);
			rw->pmic_last[d][r] = v;
		}
	}
	first = false;
	pr_info("rapuyama-wdt: pmic%s\n", len ? buf : " -");
	schedule_delayed_work(&rw->dump, msecs_to_jiffies(pmic_dump * 1000));
}

static int rapuyama_wdt_ping(struct watchdog_device *wdd)
{
	struct rapuyama_wdt *rw = watchdog_get_drvdata(wdd);
	int ret;

	if (!wdd->timeout)
		return -EINVAL; /* 0 would power the phone off */
	writel(SOC_WDT_RELOAD, rw->soc);
	rapuyama_wdt_pinged();
	ret = rapuyama_cbus_write(rw, PMIC_DEV, PMIC_REG_WDT, wdd->timeout);
	if (ret)
		dev_warn_ratelimited(wdd->parent, "pmic kick failed: %d\n", ret);
	return ret;
}

static int rapuyama_wdt_set_timeout(struct watchdog_device *wdd,
				    unsigned int t)
{
	wdd->timeout = t;
	return rapuyama_wdt_ping(wdd);
}

/*
 * power off = expire the pmic watchdog right now, same as symbian's sisa
 * "force power off" and mainline retu-mfd's retu_power_off
 */
static int rapuyama_wdt_power_off(struct sys_off_data *data)
{
	struct rapuyama_wdt *rw = data->cb_data;

	rapuyama_cbus_write(rw, PMIC_DEV, PMIC_REG_WDT, 0);
	for (;;)
		cpu_relax();
	return NOTIFY_DONE;
}

static int rapuyama_wdt_restart(struct sys_off_data *data)
{
	struct rapuyama_wdt *rw = data->cb_data;

	local_irq_disable();
	writel(SOC_RESET_VAL, rw->reset);
	for (;;)
		cpu_relax();
	return NOTIFY_DONE;
}

static const struct watchdog_info rapuyama_wdt_info = {
	.options	= WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING,
	.identity	= "RapuYama PMIC watchdog",
};

static const struct watchdog_ops rapuyama_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= rapuyama_wdt_ping,
	.ping		= rapuyama_wdt_ping,
	.set_timeout	= rapuyama_wdt_set_timeout,
};

static int rapuyama_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rapuyama_wdt *rw;
	int ret;

	rw = devm_kzalloc(dev, sizeof(*rw), GFP_KERNEL);
	if (!rw)
		return -ENOMEM;
	platform_set_drvdata(pdev, rw);

	rw->cbus = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(rw->cbus))
		return PTR_ERR(rw->cbus);
	rw->soc = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(rw->soc))
		return PTR_ERR(rw->soc);
	rw->reset = devm_platform_ioremap_resource(pdev, 2);
	if (IS_ERR(rw->reset))
		rw->reset = NULL;

	spin_lock_init(&rw->lock);

	rw->wdd.info = &rapuyama_wdt_info;
	rw->wdd.ops = &rapuyama_wdt_ops;
	rw->wdd.min_timeout = 1;
	rw->wdd.max_timeout = PMIC_WDT_MAX;
	rw->wdd.timeout = PMIC_WDT_DEF;
	rw->wdd.max_hw_heartbeat_ms = RY_HW_HEARTBEAT_MS;
	rw->wdd.parent = dev;
	watchdog_init_timeout(&rw->wdd, 0, dev);
	watchdog_set_nowayout(&rw->wdd, true);
	watchdog_set_drvdata(&rw->wdd, rw);

	/* symbian left it running, feed it now and let the core keep going */
	ret = rapuyama_wdt_ping(&rw->wdd);
	if (ret)
		dev_warn(dev, "first kick failed: %d\n", ret);
	set_bit(WDOG_HW_RUNNING, &rw->wdd.status);

	ret = devm_watchdog_register_device(dev, &rw->wdd);
	if (ret)
		return ret;

	/* the watchdog core feeds it from here on, stop the timer irq kick */
	rapuyama_early_wdt_stop();

	ret = devm_register_sys_off_handler(dev, SYS_OFF_MODE_POWER_OFF,
					    SYS_OFF_PRIO_DEFAULT,
					    rapuyama_wdt_power_off, rw);
	if (ret)
		dev_warn(dev, "no power off handler: %d\n", ret);

	if (rw->reset) {
		ret = devm_register_sys_off_handler(dev, SYS_OFF_MODE_RESTART,
						    SYS_OFF_PRIO_DEFAULT,
						    rapuyama_wdt_restart, rw);
		if (ret)
			dev_warn(dev, "no restart handler: %d\n", ret);
	}

	if (pmic_dump) {
		INIT_DELAYED_WORK(&rw->dump, rapuyama_pmic_dump);
		schedule_delayed_work(&rw->dump, 0);
	}

	WRITE_ONCE(rapuyama_pmic, rw);
	dev_info(dev, "pmic + soc watchdog, %u s\n", rw->wdd.timeout);
	return 0;
}

static const struct of_device_id rapuyama_wdt_of_match[] = {
	{ .compatible = "nokia,rapuyama-wdt" },
	{ }
};
MODULE_DEVICE_TABLE(of, rapuyama_wdt_of_match);

static struct platform_driver rapuyama_wdt_driver = {
	.probe = rapuyama_wdt_probe,
	.driver = {
		.name = "rapuyama-wdt",
		.of_match_table = rapuyama_wdt_of_match,
		.dev_groups = rapuyama_wdt_groups,
	},
};
/*
 * early, so the soc watchdog (~13 s) gets fed before slow or risky drivers
 * (usb, i2c) probe and a crash in one of them isn't hidden behind a reset
 */
static int __init rapuyama_wdt_init(void)
{
	return platform_driver_register(&rapuyama_wdt_driver);
}
subsys_initcall(rapuyama_wdt_init);

MODULE_DESCRIPTION("Nokia RapuYama watchdog driver");
MODULE_LICENSE("GPL");