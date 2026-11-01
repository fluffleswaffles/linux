// SPDX-License-Identifier: GPL-2.0
/*
 * Nokia E5-00 RapuYama SPI2 controller (the WL1251 wlan bus), polled PIO
 *
 * No public documentation exists, reversed from spi2.dll of the E5 RM-632
 * firmware (101.003), the polled paths only (tx 0x8015f040, rx 0x8015f0dc,
 * full duplex 0x8015f2b8, config 0x8015fa80, init 0x8015ec80). Every
 * transfer here is a full duplex one (tx zeros when there's no tx buffer),
 * that is the simplest of the three firmware paths. The irq (193) and the
 * dma side (0x20030000) are not used. Offsets from the block base (phys
 * 0x20020600):
 *
 *   0x04 enable (1 on, 0 off)
 *   0x08 frame pad bits (16 - bits % 16, 0 when a multiple of 16)
 *   0x0c 0x34 0x40  halfwords per frame - 1 (tx frame / tx fifo / rx fifo
 *        threshold), all three written with the same value
 *   0x10 cpol   0x14 1 for mode 0 and 3 (sampling edge)
 *   0x18 0 = full duplex, 1 = tx only / rx only
 *   0x1c 0x20 divider, sclk = 96 MHz / (div + 1), both written
 *   0x28 tx busy   0x2c tx fifo (16 bit)   0x30 tx fifo space
 *   0x38 rx fifo (16 bit)   0x3c rx fifo level
 *   0x44 7 while receiving (rx clocking), 0 after
 *   0x48 0x58 always 1 for the wlan (meaning unknown)
 *   0x4c bit cs clear = chip select asserted, set = released (manual; the
 *        firmware's Modify32 0x8009c23c is (reg, set, clear), idle = 3)
 *   0x5c apply: write 0, poll until it reads != 0
 *   0x60 irq enable (left 0)
 *
 * The wire is 16-bit frames, MSB first (the wlan pdd swaps the halfwords
 * of every 32-bit word before handing it over), so a 32-bit word goes out
 * as its high halfword then its low one.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>

#define RY_SPI_EN	0x04
#define RY_SPI_PAD	0x08
#define RY_SPI_TXFRM	0x0c
#define RY_SPI_CPOL	0x10
#define RY_SPI_CPHA	0x14
#define RY_SPI_HALF	0x18
#define RY_SPI_DIV0	0x1c
#define RY_SPI_DIV1	0x20
#define RY_SPI_TXBUSY	0x28
#define RY_SPI_TXFIFO	0x2c
#define RY_SPI_TXSPACE	0x30
#define RY_SPI_TXTHR	0x34
#define RY_SPI_RXFIFO	0x38
#define RY_SPI_RXLVL	0x3c
#define RY_SPI_RXTHR	0x40
#define RY_SPI_RXCLK	0x44
#define RY_SPI_F48	0x48
#define RY_SPI_CS	0x4c
#define RY_SPI_F58	0x58
#define RY_SPI_APPLY	0x5c
/* 0x660: symbian calls it around the irq bind (ec80), but the shifter is frozen
 * while it is 0 (phone, 2026-11-02): 0 holds the tx fifo, 1 sends it at once */
#define RY_SPI_RUN	0x60
#define RY_SPI_SIZE	0x70

#define RY_SPI_FCLK	96000000
#define RY_SPI_DIV_MIN	1	/* the firmware's fastest: 48 MHz */
#define RY_SPI_DIV_MAX	0x7cff	/* its slowest table entry */
#define RY_SPI_TIMEOUT_US 10000

struct rapuyama_spi {
	void __iomem *base;
	struct device *dev;
	u32 div;		/* programmed divider, 0 = not yet */
	u32 mode;
	bool dumped;
};

static void ry_spi_dump(struct rapuyama_spi *ry, const char *why)
{
	/* the regs the firmware touches, minus the two fifos */
	static const u8 regs[] = {
		0x04, 0x08, 0x0c, 0x10, 0x14, 0x18, 0x1c, 0x20, 0x28, 0x30,
		0x34, 0x3c, 0x40, 0x44, 0x48, 0x4c, 0x58, 0x5c, 0x60, 0x64,
		0x6c,
	};
	int i;

	for (i = 0; i < ARRAY_SIZE(regs); i++)
		dev_info(ry->dev, "%s: +%02x = %08x\n", why, regs[i],
			 readl(ry->base + regs[i]));
}

static int ry_spi_apply(struct rapuyama_spi *ry)
{
	u32 v;
	int ret;

	writel(0, ry->base + RY_SPI_APPLY);
	ret = readl_poll_timeout_atomic(ry->base + RY_SPI_APPLY, v, v, 1,
					RY_SPI_TIMEOUT_US);
	if (ret)
		dev_err(ry->dev, "apply timeout\n");
	return ret;
}

/* 0x8015fa80 for full duplex with 16-bit frames, skipped when unchanged */
static int ry_spi_config(struct rapuyama_spi *ry, u32 hz, u32 mode)
{
	u32 div = clamp(DIV_ROUND_UP(RY_SPI_FCLK, hz ?: 1) - 1,
			RY_SPI_DIV_MIN, RY_SPI_DIV_MAX);

	mode &= SPI_MODE_X_MASK;
	if (div == ry->div && mode == ry->mode)
		return 0;

	writel(0, ry->base + RY_SPI_HALF);
	writel(0, ry->base + RY_SPI_TXFRM);
	writel(0, ry->base + RY_SPI_PAD);
	writel(0, ry->base + RY_SPI_RXTHR);
	writel(0, ry->base + RY_SPI_TXTHR);
	writel(div, ry->base + RY_SPI_DIV0);
	writel(div, ry->base + RY_SPI_DIV1);
	writel(!!(mode & SPI_CPOL), ry->base + RY_SPI_CPOL);
	writel(mode == SPI_MODE_0 || mode == SPI_MODE_3,
	       ry->base + RY_SPI_CPHA);
	writel(1, ry->base + RY_SPI_F58);
	writel(1, ry->base + RY_SPI_F48);
	ry->div = 0;
	if (ry_spi_apply(ry))
		return -ETIMEDOUT;
	ry->div = div;
	ry->mode = mode;
	dev_dbg(ry->dev, "%u Hz (div %u), mode %u\n",
		RY_SPI_FCLK / (div + 1), div, mode);
	return 0;
}

static void rapuyama_spi_set_cs(struct spi_device *spi, bool level)
{
	struct rapuyama_spi *ry = spi_controller_get_devdata(spi->controller);
	u32 bit = BIT(spi_get_chipselect(spi, 0));
	u32 v = readl(ry->base + RY_SPI_CS);

	/* the bit follows the pin: clear = low = asserted */
	writel(level ? v | bit : v & ~bit, ry->base + RY_SPI_CS);
}

/* halfword i of the transfer, in wire order */
static u16 ry_tx_half(const void *buf, unsigned int bpw, unsigned int i)
{
	const u8 *b = buf;
	u32 w;

	if (!buf)
		return 0;
	switch (bpw) {
	case 8:
		return b[2 * i] << 8 | b[2 * i + 1];
	case 16:
		return ((const u16 *)buf)[i];
	default:
		w = ((const u32 *)buf)[i / 2];
		return (i & 1) ? w & 0xffff : w >> 16;
	}
}

static void ry_rx_half(void *buf, unsigned int bpw, unsigned int i, u16 h)
{
	u8 *b = buf;
	u32 *w;

	if (!buf)
		return;
	switch (bpw) {
	case 8:
		b[2 * i] = h >> 8;
		b[2 * i + 1] = h;
		break;
	case 16:
		((u16 *)buf)[i] = h;
		break;
	default:
		w = &((u32 *)buf)[i / 2];
		if (i & 1)
			*w = (*w & 0xffff0000) | h;
		else
			*w = (*w & 0xffff) | (u32)h << 16;
	}
}

/* 0x8015f2b8: one frame ahead, then fill / wait rx / drain */
static int rapuyama_spi_transfer_one(struct spi_controller *ctlr,
				     struct spi_device *spi,
				     struct spi_transfer *t)
{
	struct rapuyama_spi *ry = spi_controller_get_devdata(ctlr);
	unsigned int bpw = t->bits_per_word;
	unsigned int n = t->len / 2, tx = 0, rx = 0;
	u32 v;
	int ret;

	if (t->len & 1)
		return -EINVAL;

	writel(7, ry->base + RY_SPI_RXCLK);
	ret = ry_spi_config(ry, t->speed_hz, spi->mode);
	if (ret)
		goto out;

	/* a stale rx halfword would shift every later read by one frame */
	while (readl(ry->base + RY_SPI_RXLVL))
		readw(ry->base + RY_SPI_RXFIFO);

	if (n)
		writew(ry_tx_half(t->tx_buf, bpw, tx++), ry->base + RY_SPI_TXFIFO);
	while (rx < n) {
		if (tx < n)
			writew(ry_tx_half(t->tx_buf, bpw, tx++),
			       ry->base + RY_SPI_TXFIFO);
		ret = readl_poll_timeout_atomic(ry->base + RY_SPI_RXLVL, v, v,
						0, RY_SPI_TIMEOUT_US);
		if (ret) {
			dev_err(ry->dev, "rx timeout, %u/%u halfwords\n", rx, n);
			if (!ry->dumped) {
				ry->dumped = true;
				ry_spi_dump(ry, "timeout");
			}
			ry->div = 0;	/* reprogram next time */
			goto out;
		}
		ry_rx_half(t->rx_buf, bpw, rx++,
			   readw(ry->base + RY_SPI_RXFIFO));
	}
out:
	writel(0, ry->base + RY_SPI_RXCLK);
	return ret;
}

static int rapuyama_spi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct spi_controller *ctlr;
	struct rapuyama_spi *ry;
	struct clk *clk;
	int ret;

	ctlr = devm_spi_alloc_host(dev, sizeof(*ry));
	if (!ctlr)
		return -ENOMEM;
	ry = spi_controller_get_devdata(ctlr);
	ry->dev = dev;

	ry->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ry->base))
		return PTR_ERR(ry->base);

	/* firmware order: res 0x1c (block) then res 0x0b (ext sys clk) */
	clk = devm_clk_get_enabled(dev, "fck");
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "fck\n");
	clk = devm_clk_get_optional_enabled(dev, "sys");
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "sys\n");

	ry_spi_dump(ry, "probe");

	/* 0x8015ea88 + 0x8015ec80 without binding the irqs */
	writel(1, ry->base + RY_SPI_EN);
	writel(1, ry->base + RY_SPI_RUN);
	writel(0, ry->base + RY_SPI_RXCLK);
	writel(3, ry->base + RY_SPI_CS);	/* both released, the reset value */
	ret = ry_spi_config(ry, 1000000, SPI_MODE_0);
	if (ret)
		return ret;
	ry_spi_dump(ry, "init");

	ctlr->dev.of_node = dev->of_node;
	ctlr->bus_num = -1;
	ctlr->num_chipselect = 1;
	ctlr->mode_bits = SPI_CPOL | SPI_CPHA;
	ctlr->bits_per_word_mask = SPI_BPW_MASK(8) | SPI_BPW_MASK(16) |
				   SPI_BPW_MASK(32);
	ctlr->min_speed_hz = RY_SPI_FCLK / (RY_SPI_DIV_MAX + 1);
	ctlr->max_speed_hz = RY_SPI_FCLK / (RY_SPI_DIV_MIN + 1);
	ctlr->set_cs = rapuyama_spi_set_cs;
	ctlr->transfer_one = rapuyama_spi_transfer_one;

	return devm_spi_register_controller(dev, ctlr);
}

static const struct of_device_id rapuyama_spi_of_match[] = {
	{ .compatible = "nokia,rapuyama-e5-spi" },
	{ }
};
MODULE_DEVICE_TABLE(of, rapuyama_spi_of_match);

static struct platform_driver rapuyama_spi_driver = {
	.probe = rapuyama_spi_probe,
	.driver = {
		.name = "rapuyama-e5-spi",
		.of_match_table = rapuyama_spi_of_match,
	},
};
module_platform_driver(rapuyama_spi_driver);

MODULE_DESCRIPTION("Nokia E5-00 RapuYama SPI2 controller (polled)");
MODULE_LICENSE("GPL");
