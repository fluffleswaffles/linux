// SPDX-License-Identifier: GPL-2.0
/* rapuyama hs i2c, from isasw hal_i2c_hsi2c.c and hal_i2c_rapu.c */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#define HS_STAT 0x08
#define HS_SYSS 0x10
#define HS_BUF 0x14
#define HS_CON 0x24
#define HS_SYSTEST 0x3c
#define HS_EN (BIT(15) | BIT(7)) /* isasw uses 0x8080, not omap's 0x8000 */
#define HS_EVENTS 0x603f
#define HS_IO_MODE (BIT(15) | GENMASK(13, 12))
#define HS_SCL_IN BIT(3)
#define HS_SCL_OUT BIT(2)
#define HS_SDA_IN BIT(1)
#define HS_SDA_OUT BIT(0)

struct ry_hs {
	void __iomem *base;
	struct i2c_adapter adap;
	struct i2c_algo_bit_data bit;
	u32 lines;
};

/* adapter locking serializes the output shadow and all line accesses */
static void ry_hs_setsda(void *data, int value)
{
	struct ry_hs *hs = data;

	hs->lines = (hs->lines & ~HS_SDA_OUT) | (value ? HS_SDA_OUT : 0);
	writel(hs->lines, hs->base + HS_SYSTEST);
}

static void ry_hs_setscl(void *data, int value)
{
	struct ry_hs *hs = data;

	hs->lines = (hs->lines & ~HS_SCL_OUT) | (value ? HS_SCL_OUT : 0);
	writel(hs->lines, hs->base + HS_SYSTEST);
}

static int ry_hs_getsda(void *data)
{
	struct ry_hs *hs = data;

	return !!(readl(hs->base + HS_SYSTEST) & HS_SDA_IN);
}

static int ry_hs_getscl(void *data)
{
	struct ry_hs *hs = data;

	return !!(readl(hs->base + HS_SYSTEST) & HS_SCL_IN);
}

static int ry_hs_pre_xfer(struct i2c_adapter *adap)
{
	struct ry_hs *hs = i2c_get_adapdata(adap);

	/* mode 3 uses open-drain outputs; ones release the lines */
	hs->lines = HS_IO_MODE | HS_SCL_OUT | HS_SDA_OUT;
	writel(hs->lines, hs->base + HS_SYSTEST);
	udelay(5);
	if (!ry_hs_getscl(hs) || !ry_hs_getsda(hs)) {
		writel(0, hs->base + HS_SYSTEST);
		return -EBUSY;
	}
	return 0;
}

static void ry_hs_post_xfer(struct i2c_adapter *adap)
{
	struct ry_hs *hs = i2c_get_adapdata(adap);

	/* the bit algorithm has sent stop and released both outputs */
	writel(0, hs->base + HS_SYSTEST);
}

/* initialize the controller without starting its native master */
static int ry_hs_reset(struct ry_hs *hs)
{
	u32 st;
	int ret;

	writel(0, hs->base + HS_CON);
	writel(0, hs->base + 0x30);
	writel(0xe9, hs->base + 0x34);
	writel(0xeb, hs->base + 0x38);
	writel(2, hs->base + 0x20);
	udelay(1000);
	writel(HS_EN, hs->base + HS_CON);
	ret = readl_poll_timeout(hs->base + HS_SYSS, st, st & 1, 10, 10000);
	if (ret)
		return ret;
	writel(0, hs->base + HS_CON);
	writel(0x1f8, hs->base + 0x28);
	/* line IO needs no controller interrupts */
	writel(0, hs->base + 0x04);
	/* reset already emptied both FIFOs; request one byte per event */
	writel(0, hs->base + HS_BUF);
	writel(HS_EVENTS, hs->base + HS_STAT);
	/* retain both clocks while the controller is active */
	writel(0x308, hs->base + 0x20);
	writel(HS_EN, hs->base + HS_CON);
	udelay(1);
	return 0;
}

#ifdef CONFIG_ARCH_RAPUYAMA
int rapuyama_safe_readl(const void __iomem *addr, u32 *val);
#else
static int rapuyama_safe_readl(const void __iomem *addr, u32 *val)
{
	*val = readl(addr);
	return 0;
}
#endif

static int ry_hs_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct clk *clk;
	struct ry_hs *hs;
	void __iomem *cgc, *pads, *pll;
	unsigned long flags;
	u32 st;
	int ret;

	hs = devm_kzalloc(dev, sizeof(*hs), GFP_KERNEL);
	if (!hs)
		return -ENOMEM;
	hs->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(hs->base))
		return PTR_ERR(hs->base);
	/* the mux and gate registers are shared with other cgc clients */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res)
		return -EINVAL;
	cgc = devm_ioremap(dev, res->start, resource_size(res));
	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (!res)
		return -EINVAL;
	pads = devm_ioremap(dev, res->start, resource_size(res));
	res = platform_get_resource(pdev, IORESOURCE_MEM, 3);
	if (!res)
		return -EINVAL;
	pll = devm_ioremap(dev, res->start, resource_size(res));
	if (!cgc || !pads || !pll)
		return -ENOMEM;
	/* FUN_c62ee62c(3) followed by the full FUN_c62ee6c2(3) power path */
	local_irq_save(flags);
	st = readl(cgc + 0x34) & ~0x1c00;
	writel(st, cgc + 0x34);
	writel(st | 0xc00, cgc + 0x34);
	writel(1, cgc + 0xd4);
	writel(0x40, cgc + 0xb4);
	writel(0x800, pads + 0x38);
	local_irq_restore(flags);
	/* retain the shared ACCPLL configuration; only clear power-down */
	writel(0x8000, cgc + 0x88);
	ret = readl_poll_timeout(pll, st, st & BIT(4), 10, 10000);
	if (ret)
		return dev_err_probe(dev, ret, "ACCPLL did not lock\n");
	clk = devm_clk_get_enabled(dev, "ick");
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "no interface clock\n");
	udelay(10);
	dev_info(dev, "hs i2c port 3: clocks and pads ready\n");
	ret = rapuyama_safe_readl(hs->base + HS_STAT, &st);
	if (ret)
		return dev_err_probe(dev, ret, "controller is not accessible\n");
	ret = ry_hs_reset(hs);
	if (ret)
		return dev_err_probe(dev, ret, "controller reset timed out\n");
	/* the native FIFO stalls on rev 3a; line IO was verified on the phone */
	hs->bit.data = hs;
	hs->bit.setsda = ry_hs_setsda;
	hs->bit.setscl = ry_hs_setscl;
	hs->bit.getsda = ry_hs_getsda;
	hs->bit.getscl = ry_hs_getscl;
	hs->bit.pre_xfer = ry_hs_pre_xfer;
	hs->bit.post_xfer = ry_hs_post_xfer;
	hs->bit.udelay = 5;
	hs->bit.timeout = msecs_to_jiffies(50);
	hs->bit.can_do_atomic = true;
	hs->adap.owner = THIS_MODULE;
	hs->adap.algo = &i2c_bit_algo;
	hs->adap.algo_data = &hs->bit;
	hs->adap.dev.parent = dev;
	hs->adap.dev.of_node = dev->of_node;
	hs->adap.retries = 2;
	strscpy(hs->adap.name, "RapuYama HS I2C port 3", sizeof(hs->adap.name));
	i2c_set_adapdata(&hs->adap, hs);
	ret = devm_i2c_add_adapter(dev, &hs->adap);
	if (!ret)
		dev_info(dev, "bus %d, SYSTEST line IO, 5 us half-cycle\n", hs->adap.nr);
	return ret;
}

static const struct of_device_id ry_hs_match[] = {
	{ .compatible = "nokia,rapuyama-hs-i2c" },
	{ }
};
MODULE_DEVICE_TABLE(of, ry_hs_match);

static struct platform_driver ry_hs_driver = {
	.probe = ry_hs_probe,
	.driver = {
		.name = "rapuyama-hs-i2c",
		.of_match_table = ry_hs_match,
	},
};
module_platform_driver(ry_hs_driver);
MODULE_DESCRIPTION("Nokia RapuYama HS I2C");
MODULE_LICENSE("GPL");