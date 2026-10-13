// SPDX-License-Identifier: GPL-2.0
/*
 * Nokia RapuYama MUSB glue (N8/E6/E7/C7)
 *
 * No public documentation exists, reversed from usb.dll of the E6 RM-609
 * firmware: the usb block (phys 0x20038800) is a Mentor MUSB core with the TI
 * OMAP2430 style OTG wrapper at +0x400 (revision, sysconfig, sysstatus,
 * interfsel, forcestdby) and a ULPI phy inside the Gazoo power asic. usb.dll
 * binds irq 183 (core) and 215 (dma) and sets the wrapper up exactly like
 * omap2430.c: soft reset, wait for reset done, leave forced standby, smart
 * idle + smart standby, 12 pin ULPI.
 *
 * The phy and the vbus/charger side are left as symbian set them up (usb mass
 * storage works before the jump), so a nop transceiver is enough for gadget
 * mode. Peripheral only for now.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/usb/usb_phy_generic.h>

#include "musb_core.h"

#define RY_OTG_REVISION		0x400
#define RY_OTG_SYSCONFIG	0x404
#define RY_SYSCONFIG_SOFTRST	BIT(1)
#define RY_SYSCONFIG_AUTOIDLE	BIT(0)
#define RY_SYSCONFIG_IDLE	0x2010	/* smart standby + smart idle */
#define RY_OTG_SYSSTATUS	0x408
#define RY_SYSSTATUS_RESETDONE	BIT(0)
#define RY_OTG_INTERFSEL	0x40c
#define RY_INTERFSEL_ULPI_12PIN	BIT(0)
#define RY_OTG_FORCESTDBY	0x414
#define RY_FORCESTDBY_ENABLE	BIT(0)

struct rapuyama_glue {
	struct platform_device	*musb;
};

static int rapuyama_musb_wrapper_init(struct musb *musb)
{
	void __iomem *b = musb->mregs;
	int n = 10;

	/* usb.dll 0x80189550: reset, then poll reset done 10 x 5 ms */
	writel(RY_SYSCONFIG_SOFTRST, b + RY_OTG_SYSCONFIG);
	while (!(readl(b + RY_OTG_SYSSTATUS) & RY_SYSSTATUS_RESETDONE)) {
		if (!--n)
			return -ETIMEDOUT;
		usleep_range(5000, 6000);
	}

	writel(readl(b + RY_OTG_FORCESTDBY) & ~RY_FORCESTDBY_ENABLE,
	       b + RY_OTG_FORCESTDBY);
	writel(readl(b + RY_OTG_SYSCONFIG) & ~RY_SYSCONFIG_AUTOIDLE,
	       b + RY_OTG_SYSCONFIG);
	writel(readl(b + RY_OTG_SYSCONFIG) | RY_SYSCONFIG_IDLE,
	       b + RY_OTG_SYSCONFIG);
	writel(readl(b + RY_OTG_SYSCONFIG) & ~RY_SYSCONFIG_AUTOIDLE,
	       b + RY_OTG_SYSCONFIG);
	writel(RY_INTERFSEL_ULPI_12PIN, b + RY_OTG_INTERFSEL);
	return 0;
}

static int rapuyama_musb_init(struct musb *musb)
{
	struct device *dev = musb->controller;
	int ret;

	musb->xceiv = devm_usb_get_phy_by_phandle(dev->parent, "phys", 0);
	if (IS_ERR(musb->xceiv))
		return dev_err_probe(dev, PTR_ERR(musb->xceiv),
				     "no transceiver\n");

	ret = rapuyama_musb_wrapper_init(musb);
	if (ret) {
		dev_err(dev, "wrapper reset timed out\n");
		return ret;
	}

	dev_info(dev, "otg wrapper rev %#x\n",
		 readl(musb->mregs + RY_OTG_REVISION));
	return 0;
}

static int rapuyama_musb_exit(struct musb *musb)
{
	return 0;
}

/* pio only for bring-up, the dma side (irq 215) comes later */
static const struct musb_platform_ops rapuyama_musb_ops = {
	.quirks		= MUSB_INDEXED_EP,
	.fifo_mode	= 4,
	.init		= rapuyama_musb_init,
	.exit		= rapuyama_musb_exit,
};

/* same as the omap2430/omap3 musb instances this ip comes from */
static const struct musb_hdrc_config rapuyama_musb_config = {
	.multipoint	= true,
	.num_eps	= 16,
	.ram_bits	= 12,
};

static const struct musb_hdrc_platform_data rapuyama_musb_pdata = {
	.mode		= MUSB_PERIPHERAL,
	.config		= &rapuyama_musb_config,
	.platform_ops	= &rapuyama_musb_ops,
};

static int rapuyama_musb_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rapuyama_glue *glue;
	struct platform_device *musb;
	struct clk_bulk_data *clks;
	int ret;

	glue = devm_kzalloc(dev, sizeof(*glue), GFP_KERNEL);
	if (!glue)
		return -ENOMEM;

	/* EHwClkDomHSUSB + EHwClkHSUSB, what usb.dll asks for */
	ret = devm_clk_bulk_get_all_enable(dev, &clks);
	if (ret < 0)
		return dev_err_probe(dev, ret, "clocks\n");

	musb = platform_device_alloc("musb-hdrc", PLATFORM_DEVID_AUTO);
	if (!musb)
		return -ENOMEM;

	musb->dev.parent = dev;
	musb->dev.dma_mask = &musb->dev.coherent_dma_mask;
	musb->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	device_set_of_node_from_dev(&musb->dev, dev);
	glue->musb = musb;
	platform_set_drvdata(pdev, glue);

	ret = platform_device_add_resources(musb, pdev->resource,
					    pdev->num_resources);
	if (ret)
		goto err_put;

	ret = platform_device_add_data(musb, &rapuyama_musb_pdata,
				       sizeof(rapuyama_musb_pdata));
	if (ret)
		goto err_put;

	ret = platform_device_add(musb);
	if (ret)
		goto err_put;
	return 0;

err_put:
	platform_device_put(musb);
	return ret;
}

static void rapuyama_musb_remove(struct platform_device *pdev)
{
	struct rapuyama_glue *glue = platform_get_drvdata(pdev);

	platform_device_unregister(glue->musb);
}

static const struct of_device_id rapuyama_musb_of_match[] = {
	{ .compatible = "nokia,rapuyama-musb" },
	{ }
};
MODULE_DEVICE_TABLE(of, rapuyama_musb_of_match);

static struct platform_driver rapuyama_musb_driver = {
	.probe		= rapuyama_musb_probe,
	.remove_new	= rapuyama_musb_remove,
	.driver		= {
		.name	= "musb-rapuyama",
		.of_match_table = rapuyama_musb_of_match,
	},
};
module_platform_driver(rapuyama_musb_driver);

MODULE_DESCRIPTION("Nokia RapuYama MUSB glue");
MODULE_LICENSE("GPL");