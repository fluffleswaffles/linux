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
 * The CBUS master (karap 0x800b8414/0x800b8494) is a register block:
 *   0x04 enable (1 around a transfer)   0x08 busy
 *   0x14 read cmd  (dev << 6 | reg | 0x20)
 *   0x18 write cmd (dev << 6 | reg)     0x1c data
 *   0x28 bit count - 1                  0x38 1 before a write
 *
 * Both run from the moment symbian booted us, so the device is marked as
 * running and the watchdog core keeps it fed until userspace takes over.
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/watchdog.h>

#define CBUS_EN		0x04
#define CBUS_BUSY	0x08
#define CBUS_RD_CMD	0x14
#define CBUS_WR_CMD	0x18
#define CBUS_DATA	0x1c
#define CBUS_BITS	0x28
#define CBUS_WR_MODE	0x38
#define CBUS_READ	0x20
#define CBUS_TRIES	0x800

#define PMIC_DEV	1
#define PMIC_REG_WDT	0x17
#define PMIC_WDT_MAX	63	/* 6 bit seconds, like the retu */
#define PMIC_WDT_DEF	48	/* what symbian uses */

#define SOC_WDT_RELOAD	0x3fffffff

struct rapuyama_wdt {
	struct watchdog_device wdd;
	void __iomem *cbus;
	void __iomem *soc;
	spinlock_t lock;
};

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

static int rapuyama_cbus_write(struct rapuyama_wdt *rw, u32 dev, u32 reg,
			       u16 val)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&rw->lock, flags);
	writel(1, rw->cbus + CBUS_EN);
	writel(1, rw->cbus + CBUS_WR_MODE);
	writel(16 - 1, rw->cbus + CBUS_BITS);
	writel(val, rw->cbus + CBUS_DATA);
	writel(dev << 6 | reg, rw->cbus + CBUS_WR_CMD);
	ret = rapuyama_cbus_wait(rw);
	writel(0, rw->cbus + CBUS_EN);
	spin_unlock_irqrestore(&rw->lock, flags);
	return ret;
}

static int rapuyama_wdt_ping(struct watchdog_device *wdd)
{
	struct rapuyama_wdt *rw = watchdog_get_drvdata(wdd);

	if (!wdd->timeout)
		return -EINVAL; /* 0 would power the phone off */
	writel(SOC_WDT_RELOAD, rw->soc);
	return rapuyama_cbus_write(rw, PMIC_DEV, PMIC_REG_WDT, wdd->timeout);
}

static int rapuyama_wdt_set_timeout(struct watchdog_device *wdd,
				    unsigned int t)
{
	wdd->timeout = t;
	return rapuyama_wdt_ping(wdd);
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

	rw->cbus = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(rw->cbus))
		return PTR_ERR(rw->cbus);
	rw->soc = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(rw->soc))
		return PTR_ERR(rw->soc);

	spin_lock_init(&rw->lock);

	rw->wdd.info = &rapuyama_wdt_info;
	rw->wdd.ops = &rapuyama_wdt_ops;
	rw->wdd.min_timeout = 1;
	rw->wdd.max_timeout = PMIC_WDT_MAX;
	rw->wdd.timeout = PMIC_WDT_DEF;
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
	},
};
module_platform_driver(rapuyama_wdt_driver);

MODULE_DESCRIPTION("Nokia RapuYama watchdog driver");
MODULE_LICENSE("GPL");