// SPDX-License-Identifier: GPL-2.0
/*
 * Nokia RapuYama modem IRQ handler.
 *
 * The ISA/SISA transport is not implemented here. In particular, 0x2040000c
 * is the INTC IRQ-unmask register, not a modem DFC queue register.
 */

#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/timer.h>

#ifdef CONFIG_ARCH_RAPUYAMA
void rapuyama_note(const char *fmt, ...);
#else
static inline void rapuyama_note(const char *fmt, ...) { }
#endif

#define MODEM_DFC_VAL		0x1000
#define MODEM_KICK_INTERVAL_MS	60000

struct rapuyama_modem {
	void __iomem *status_reg;
	void __iomem *enable_reg;
	void __iomem *ack_reg;
	struct timer_list timer;
};

static irqreturn_t rapuyama_modem_irq(int irq, void *data)
{
	struct rapuyama_modem *modem = data;
	u32 status = readl_relaxed(modem->status_reg);

	if (!(status & MODEM_DFC_VAL))
		return IRQ_NONE;

	writel_relaxed(MODEM_DFC_VAL, modem->ack_reg);
	return IRQ_HANDLED;
}

static void rapuyama_modem_kick(struct rapuyama_modem *modem)
{
	/* Clear stale status before re-enabling the modem interrupt. */
	writel_relaxed(MODEM_DFC_VAL, modem->ack_reg);
	writel_relaxed(MODEM_DFC_VAL, modem->enable_reg);
	rapuyama_note("modem: irq re-armed, status=%08x\n",
		      readl_relaxed(modem->status_reg));
}

static void rapuyama_modem_timer(struct timer_list *timer)
{
	struct rapuyama_modem *modem = from_timer(modem, timer, timer);

	rapuyama_modem_kick(modem);
	mod_timer(&modem->timer,
		  jiffies + msecs_to_jiffies(MODEM_KICK_INTERVAL_MS));
}

static int rapuyama_modem_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rapuyama_modem *modem;
	struct resource *res;
	int irq;
	int ret;

	modem = devm_kzalloc(dev, sizeof(*modem), GFP_KERNEL);
	if (!modem)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	modem->status_reg = devm_ioremap_resource(dev, res);
	if (IS_ERR(modem->status_reg))
		return PTR_ERR(modem->status_reg);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	modem->enable_reg = devm_ioremap_resource(dev, res);
	if (IS_ERR(modem->enable_reg))
		return PTR_ERR(modem->enable_reg);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	modem->ack_reg = devm_ioremap_resource(dev, res);
	if (IS_ERR(modem->ack_reg))
		return PTR_ERR(modem->ack_reg);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return dev_err_probe(dev, irq, "failed to get modem IRQ\n");

	ret = devm_request_irq(dev, irq, rapuyama_modem_irq, 0,
			       dev_name(dev), modem);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request modem IRQ\n");

	timer_setup(&modem->timer, rapuyama_modem_timer, 0);
	platform_set_drvdata(pdev, modem);

	rapuyama_modem_kick(modem);
	mod_timer(&modem->timer,
		  jiffies + msecs_to_jiffies(MODEM_KICK_INTERVAL_MS));

	dev_info(dev, "modem IRQ handler installed\n");
	return 0;
}

static void rapuyama_modem_remove(struct platform_device *pdev)
{
	struct rapuyama_modem *modem = platform_get_drvdata(pdev);

	del_timer_sync(&modem->timer);
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

MODULE_DESCRIPTION("Nokia RapuYama modem IRQ handler");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rapuyama-modem");
