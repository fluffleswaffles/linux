// SPDX-License-Identifier: GPL-2.0
/*
 * Nokia RapuYama general purpose DMA (GDMA), slave transfers (E5-00)
 *
 * No public documentation exists. Reversed from the E5 RM-632 firmware:
 * dma.dll (TRapDmac, Symbian DMA framework v1 PSL) and EPBUSMV.DLL (the
 * memory card driver, its only DMA client checked so far). Confirmed on the
 * phone by the channel Symbian left programmed after its last card write.
 *
 * Six channels, 1 KB apart. Channel 0 is never touched by the firmware's init
 * (reserved), so the dt channel mask normally leaves it out. A transfer is
 * one block per channel: no hardware descriptors, the irq of the channel
 * (intc line = channel number) fires when it is done. The firmware masks that
 * line in its isr and only unmasks it when it starts the next block on the
 * channel, so this does the same and needs no ack register.
 *
 * Global registers (base):
 *   0x00 state, 0x04 set, 0x08 clear: bit ch (left clear for slave
 *        transfers), bit 8 + ch = the memory side does word accesses
 *   0x88 + 4 * ch: dma request line (peripheral) of the channel
 * Channel registers (base + 0x480 + 0x400 * ch):
 *   0x04 claim (1 while used), 0x08 run (2 = peripheral flow, 1 = memory to
 *   memory, 0 = stop, reads 0 when stopped/done), 0x0c/0x10 unaligned tail
 *   word/bits (unused: blocks here are aligned), 0x14 src, 0x18 src step
 *   (bytes, 0 = fixed), 0x1c src width (bits), 0x20 dst, 0x24 dst step,
 *   0x28 dst width (bits), 0x2c elements - 1 (24 bits, 0xffffff when done),
 *   0x34 1 (from the firmware's peripheral table)
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_dma.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "virt-dma.h"

#define GDMA_NCH		6

#define GDMA_SET		0x04
#define GDMA_CLR		0x08
#define GDMA_DRQ(ch)		(0x88 + 4 * (ch))

#define GDMA_CH(ch)		(0x480 + 0x400 * (ch))
#define CH_CLAIM		0x04
#define CH_RUN			0x08
#define CH_TAIL_BITS		0x10
#define CH_SRC			0x14
#define CH_SRC_STEP		0x18
#define CH_SRC_WIDTH		0x1c
#define CH_DST			0x20
#define CH_DST_STEP		0x24
#define CH_DST_WIDTH		0x28
#define CH_COUNT		0x2c
#define CH_X34			0x34

#define RUN_PERIPH		2
#define COUNT_MAX		BIT(24)

struct gdma_sg {
	dma_addr_t addr;
	u32 len;
};

struct gdma_desc {
	struct virt_dma_desc vd;
	enum dma_transfer_direction dir;
	unsigned int nsg;
	unsigned int cur;
	struct gdma_sg sg[] __counted_by(nsg);
};

struct gdma_chan {
	struct virt_dma_chan vc;
	struct gdma *gd;
	void __iomem *regs;
	unsigned int id;
	int irq;
	bool irq_on;
	bool clk_on;
	u32 drq;
	struct dma_slave_config cfg;
	struct gdma_desc *desc;
};

struct gdma {
	struct dma_device ddev;
	void __iomem *base;
	struct clk *clk;
	spinlock_t glock;	/* global set/clear and drq registers */
	struct gdma_chan chan[GDMA_NCH];
};

static struct gdma_chan *to_gdma_chan(struct dma_chan *c)
{
	return container_of(c, struct gdma_chan, vc.chan);
}

static struct gdma_desc *to_gdma_desc(struct virt_dma_desc *vd)
{
	return container_of(vd, struct gdma_desc, vd);
}

static void gdma_desc_free(struct virt_dma_desc *vd)
{
	kfree(to_gdma_desc(vd));
}

/* the firmware's stop: write 0, wait until the channel reads stopped */
static int gdma_stop(struct gdma_chan *c)
{
	u32 v;

	writel(0, c->regs + CH_RUN);
	return readl_poll_timeout_atomic(c->regs + CH_RUN, v, !v, 1, 1000);
}

/* widest access the memory side allows: 4, 2 or 1 bytes */
static unsigned int gdma_mem_width(dma_addr_t addr, u32 len)
{
	if (!((addr | len) & 3))
		return 4;
	if (!((addr | len) & 1))
		return 2;
	return 1;
}

static void gdma_start_sg(struct gdma_chan *c)
{
	struct gdma_desc *d = c->desc;
	struct gdma_sg *sg = &d->sg[d->cur];
	struct gdma *gd = c->gd;
	unsigned int mw = gdma_mem_width(sg->addr, sg->len), dw;
	unsigned long flags;
	u32 dev_addr;

	if (d->dir == DMA_DEV_TO_MEM) {
		dev_addr = c->cfg.src_addr;
		dw = c->cfg.src_addr_width;
	} else {
		dev_addr = c->cfg.dst_addr;
		dw = c->cfg.dst_addr_width;
	}

	writel(1, c->regs + CH_CLAIM);
	if (gdma_stop(c))
		dev_warn(gd->ddev.dev, "ch%u does not stop\n", c->id);
	writel(0, c->regs + CH_TAIL_BITS);
	writel(1, c->regs + CH_X34);

	spin_lock_irqsave(&gd->glock, flags);
	writel(BIT(c->id), gd->base + GDMA_CLR);
	writel(BIT(8 + c->id), gd->base + (mw == 4 ? GDMA_SET : GDMA_CLR));
	writel(c->drq, gd->base + GDMA_DRQ(c->id));
	spin_unlock_irqrestore(&gd->glock, flags);

	if (d->dir == DMA_DEV_TO_MEM) {
		writel(dev_addr, c->regs + CH_SRC);
		writel(0, c->regs + CH_SRC_STEP);
		writel(dw * 8, c->regs + CH_SRC_WIDTH);
		writel(sg->addr, c->regs + CH_DST);
		writel(mw, c->regs + CH_DST_STEP);
		writel(mw * 8, c->regs + CH_DST_WIDTH);
	} else {
		writel(sg->addr, c->regs + CH_SRC);
		writel(mw, c->regs + CH_SRC_STEP);
		writel(mw * 8, c->regs + CH_SRC_WIDTH);
		writel(dev_addr, c->regs + CH_DST);
		writel(0, c->regs + CH_DST_STEP);
		writel(dw * 8, c->regs + CH_DST_WIDTH);
	}
	writel(sg->len / min(mw, dw) - 1, c->regs + CH_COUNT);
	writel(RUN_PERIPH, c->regs + CH_RUN);

	if (!c->irq_on) {
		c->irq_on = true;
		enable_irq(c->irq);
	}
}

/*
 * The clock is on only while a channel has work, as Symbian leaves it off
 * when idle. No register state has to survive: every block reprograms the
 * channel and the global set/clear/drq registers. clk core counts the users.
 * vc.lock held.
 */
static void gdma_clk_get(struct gdma_chan *c)
{
	if (!c->clk_on && !clk_enable(c->gd->clk))
		c->clk_on = true;
}

static void gdma_clk_put(struct gdma_chan *c)
{
	if (c->clk_on) {
		c->clk_on = false;
		clk_disable(c->gd->clk);
	}
}

/* vc.lock held */
static void gdma_start_next(struct gdma_chan *c)
{
	struct virt_dma_desc *vd = vchan_next_desc(&c->vc);

	if (!vd) {
		c->desc = NULL;
		if (c->clk_on)
			writel(0, c->regs + CH_CLAIM);
		gdma_clk_put(c);
		return;
	}
	gdma_clk_get(c);
	list_del(&vd->node);
	c->desc = to_gdma_desc(vd);
	c->desc->cur = 0;
	gdma_start_sg(c);
}

static irqreturn_t gdma_irq(int irq, void *data)
{
	struct gdma_chan *c = data;
	struct gdma_desc *d;

	spin_lock(&c->vc.lock);
	/* masked until the next block is started, as the firmware does */
	disable_irq_nosync(irq);
	c->irq_on = false;

	d = c->desc;
	if (!d)
		goto out;
	if (readl(c->regs + CH_RUN)) {
		/* not done yet: keep waiting */
		dev_dbg(c->gd->ddev.dev, "ch%u irq while running\n", c->id);
		c->irq_on = true;
		enable_irq(irq);
		goto out;
	}
	if (++d->cur < d->nsg) {
		gdma_start_sg(c);
		goto out;
	}
	vchan_cookie_complete(&d->vd);
	gdma_start_next(c);
out:
	spin_unlock(&c->vc.lock);
	return IRQ_HANDLED;
}

static struct dma_async_tx_descriptor *
gdma_prep_slave_sg(struct dma_chan *chan, struct scatterlist *sgl,
		   unsigned int sg_len, enum dma_transfer_direction dir,
		   unsigned long flags, void *context)
{
	struct gdma_chan *c = to_gdma_chan(chan);
	enum dma_slave_buswidth w;
	struct scatterlist *sg;
	struct gdma_desc *d;
	unsigned int i;

	if (dir == DMA_DEV_TO_MEM)
		w = c->cfg.src_addr_width;
	else if (dir == DMA_MEM_TO_DEV)
		w = c->cfg.dst_addr_width;
	else
		return NULL;
	if (w != DMA_SLAVE_BUSWIDTH_1_BYTE && w != DMA_SLAVE_BUSWIDTH_2_BYTES &&
	    w != DMA_SLAVE_BUSWIDTH_4_BYTES)
		return NULL;

	d = kzalloc(struct_size(d, sg, sg_len), GFP_NOWAIT);
	if (!d)
		return NULL;
	d->nsg = sg_len;
	d->dir = dir;

	for_each_sg(sgl, sg, sg_len, i) {
		u32 len = sg_dma_len(sg);
		dma_addr_t a = sg_dma_address(sg);

		if (!len || len % w ||
		    len / min_t(unsigned int, gdma_mem_width(a, len), w) > COUNT_MAX) {
			kfree(d);
			return NULL;
		}
		d->sg[i].addr = a;
		d->sg[i].len = len;
	}
	return vchan_tx_prep(&c->vc, &d->vd, flags);
}

static int gdma_slave_config(struct dma_chan *chan,
			     struct dma_slave_config *cfg)
{
	struct gdma_chan *c = to_gdma_chan(chan);

	c->cfg = *cfg;
	return 0;
}

static void gdma_issue_pending(struct dma_chan *chan)
{
	struct gdma_chan *c = to_gdma_chan(chan);
	unsigned long flags;

	spin_lock_irqsave(&c->vc.lock, flags);
	if (vchan_issue_pending(&c->vc) && !c->desc)
		gdma_start_next(c);
	spin_unlock_irqrestore(&c->vc.lock, flags);
}

static int gdma_terminate_all(struct dma_chan *chan)
{
	struct gdma_chan *c = to_gdma_chan(chan);
	unsigned long flags;
	LIST_HEAD(head);

	spin_lock_irqsave(&c->vc.lock, flags);
	if (c->desc) {
		gdma_stop(c);
		writel(0, c->regs + CH_CLAIM);
		if (c->irq_on) {
			c->irq_on = false;
			disable_irq_nosync(c->irq);
		}
		vchan_terminate_vdesc(&c->desc->vd);
		c->desc = NULL;
	}
	gdma_clk_put(c);
	vchan_get_all_descriptors(&c->vc, &head);
	spin_unlock_irqrestore(&c->vc.lock, flags);
	vchan_dma_desc_free_list(&c->vc, &head);
	return 0;
}

static void gdma_synchronize(struct dma_chan *chan)
{
	struct gdma_chan *c = to_gdma_chan(chan);

	synchronize_irq(c->irq);
	vchan_synchronize(&c->vc);
}

static enum dma_status gdma_tx_status(struct dma_chan *chan,
				      dma_cookie_t cookie,
				      struct dma_tx_state *state)
{
	return dma_cookie_status(chan, cookie, state);
}

static void gdma_free_chan_resources(struct dma_chan *chan)
{
	vchan_free_chan_resources(to_virt_chan(chan));
}

/* one cell: the dma request line of the peripheral; any free channel */
static struct dma_chan *gdma_xlate(struct of_phandle_args *args,
				   struct of_dma *ofdma)
{
	struct gdma *gd = ofdma->of_dma_data;
	struct dma_chan *chan;

	if (args->args_count != 1)
		return NULL;
	chan = dma_get_any_slave_channel(&gd->ddev);
	if (chan)
		to_gdma_chan(chan)->drq = args->args[0];
	return chan;
}

static int gdma_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dma_device *dd;
	struct clk *clk;
	struct gdma *gd;
	u32 mask = 0x3e;
	int i, n = 0, ret;

	gd = devm_kzalloc(dev, sizeof(*gd), GFP_KERNEL);
	if (!gd)
		return -ENOMEM;
	gd->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(gd->base))
		return PTR_ERR(gd->base);
	/* off, the block reads all ones: enabled per transfer, gdma_clk_get() */
	clk = devm_clk_get_prepared(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "no clock\n");
	gd->clk = clk;
	spin_lock_init(&gd->glock);
	of_property_read_u32(dev->of_node, "dma-channel-mask", &mask);

	dd = &gd->ddev;
	dd->dev = dev;
	INIT_LIST_HEAD(&dd->channels);
	dma_cap_set(DMA_SLAVE, dd->cap_mask);
	dma_cap_set(DMA_PRIVATE, dd->cap_mask);
	dd->src_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_1_BYTE) |
			      BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) |
			      BIT(DMA_SLAVE_BUSWIDTH_4_BYTES);
	dd->dst_addr_widths = dd->src_addr_widths;
	dd->directions = BIT(DMA_DEV_TO_MEM) | BIT(DMA_MEM_TO_DEV);
	dd->residue_granularity = DMA_RESIDUE_GRANULARITY_DESCRIPTOR;
	dd->device_prep_slave_sg = gdma_prep_slave_sg;
	dd->device_config = gdma_slave_config;
	dd->device_issue_pending = gdma_issue_pending;
	dd->device_terminate_all = gdma_terminate_all;
	dd->device_synchronize = gdma_synchronize;
	dd->device_tx_status = gdma_tx_status;
	dd->device_free_chan_resources = gdma_free_chan_resources;
	dma_set_max_seg_size(dev, SZ_16M);

	for (i = 0; i < GDMA_NCH; i++) {
		struct gdma_chan *c = &gd->chan[i];

		if (!(mask & BIT(i)))
			continue;
		c->gd = gd;
		c->id = i;
		c->regs = gd->base + GDMA_CH(i);
		c->vc.desc_free = gdma_desc_free;
		c->irq = platform_get_irq(pdev, n++);
		if (c->irq < 0)
			return c->irq;
		ret = devm_request_irq(dev, c->irq, gdma_irq, IRQF_NO_AUTOEN,
				       dev_name(dev), c);
		if (ret)
			return ret;
		vchan_init(&c->vc, dd);
	}

	ret = dmaenginem_async_device_register(dd);
	if (ret)
		return ret;
	ret = of_dma_controller_register(dev->of_node, gdma_xlate, gd);
	if (ret)
		return ret;
	platform_set_drvdata(pdev, gd);
	dev_info(dev, "%d channels (mask 0x%02x)\n", n, mask);
	return 0;
}

static void gdma_remove(struct platform_device *pdev)
{
	of_dma_controller_free(pdev->dev.of_node);
}

static const struct of_device_id gdma_of_match[] = {
	{ .compatible = "nokia,rapuyama-gdma" },
	{ }
};
MODULE_DEVICE_TABLE(of, gdma_of_match);

static struct platform_driver gdma_driver = {
	.probe = gdma_probe,
	.remove = gdma_remove,
	.driver = {
		.name = "rapuyama-gdma",
		.of_match_table = gdma_of_match,
	},
};
module_platform_driver(gdma_driver);

MODULE_DESCRIPTION("Nokia RapuYama GDMA");
MODULE_LICENSE("GPL");
