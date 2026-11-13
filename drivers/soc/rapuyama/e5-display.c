// SPDX-License-Identifier: GPL-2.0-only
/*
 * Nokia E5-00 (RapuYama) display: KMS for the 320x240 MIPI DCS command-mode
 * panel behind the OMAP2-style RFBI that Symbian leaves set up (8-bit bus,
 * RGB888 as three bytes per pixel). The hardware side is the one of the
 * fbdev driver (drivers/video/fbdev/rapuyama-e5-fb.c, kept as a fallback);
 * the DRM side follows the E6 driver (videocore.c): one fixed mode, a simple
 * display pipe with GEM shmem shadow planes and damage clips, no vblank irq
 * (the atomic helpers fake the events), fbdev + fbcon through the drm fbdev
 * emulation.
 *
 * A commit copies the changed rows of the plane into the driver's own
 * contiguous buffer (vmem, XRGB8888) and wakes the push thread, which pushes
 * the union of the rows changed since its last push. Commits do not wait for
 * a push (~18 ms for a full frame), faster frames get merged. Rows are
 * compared with what is in vmem, so the full commits weston sends before
 * each real one cost no push.
 *
 * CPU push: vmem goes to the panel GRAM through RFBI CMD/PARAM writes
 * (window PASET/CASET, RAMWR, then R,G,B per pixel), the way d_display.ldd's
 * fill-rectangle routine does it (ROM 0x801820a8). RFBI is not reset or
 * reprogrammed.
 *
 * DMA push (default; dma=0 turns it off; needs the DISPC reg + clock): DISPC
 * fetches the rectangle from vmem and feeds RFBI in stall mode, as
 * d_display.ldd's push (ROM 0x80180498, DISPC part 0x801791d0) does: GFX_BA0
 * = rect start, GFX_POSITION 0, GFX_SIZE = SIZE_LCD = (h-1)<<16 | (w-1),
 * GFX_ROW_INC = stride - w*4 + 1, GFX_PIXEL_INC 1, CONTROL |= GOLCD then |=
 * LCDENABLE, 13 us, window + PIXEL_CNT = w*h, RAMWR, RFBI CONTROL |= ENABLE |
 * ITE; done at DISPC FRAMEDONE (IRQSTATUS bit 0; irq 146 with FRAMEDONE |
 * GFXENDWIN in IRQENABLE during the push, dma_irq=0 or a missing irq polls it;
 * an irq wait timeout logs once and polls from then on), then RFBI
 * CONTROL &= ~0x11 (the firmware's stop, 0x8018060c). Symbian's DISPC setup
 * is kept: stall mode, GFX format 8 (xRGB8888, = vmem), 24 data lines. A
 * timeout logs the registers and falls back to the CPU path for good. The
 * DMA needs the ViSSI clock (power resource 0x18 = cgc gate 0x17), which is
 * on only during a push, as in the firmware.
 *
 * Screen brightness is the panel's own DCS backlight control: WRCTRLD 0x53
 * = 0x2c (BCTRL | DD | BL on), then WRDISBV 0x51 = level 0..255, exposed as
 * /sys/class/backlight/panel. DCS commands go out under push_lock so they
 * never land inside a RAMWR stream.
 *
 * Pipe disable (fbcon's console blank and FBIOBLANK through the fbdev
 * emulation, or a KMS client switching the crtc off) puts the panel to
 * sleep: WRDISBV 0, display off 0x28, sleep in 0x10. Enable: sleep out
 * 0x11, 120 ms, display on 0x29, the brightness again and a full push. No
 * pushes while asleep.
 *
 * Overlay (e5-keyd's key picture): the platform device's overlay (XRGB8888,
 * rows 120..239) and overlay_on attributes, i.e.
 * /sys/class/graphics/fb0/device/overlay as with the fbdev driver. While it
 * is on, those rows are pushed from the overlay instead of vmem.
 */
#include <drm/drm_atomic_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fbdev_shmem.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_gem_shmem_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_rect.h>
#include <drm/drm_simple_kms_helper.h>
#include <linux/backlight.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kthread.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/vmalloc.h>
#include <linux/wait.h>

#include "e5-display-core.h"

#define E5D_WIDTH		320
#define E5D_CPP			4
#define E5D_STRIDE		(E5D_WIDTH * E5D_CPP)
#define E5D_SIZE		(E5D_STRIDE * E5D_HEIGHT)
/* overlay (e5-keyd's key picture): opaque, the bottom half of the panel */
#define E5D_OVL_Y		120
#define E5D_OVL_SIZE		(E5D_STRIDE * (E5D_HEIGHT - E5D_OVL_Y))

/* RFBI registers (OMAP2 layout, offsets used by d_display.ldd) */
#define RFBI_SYSSTATUS		0x14
#define  RFBI_SYSSTATUS_BUSY	BIT(8)
#define RFBI_CONTROL		0x40
#define  RFBI_CONTROL_ENABLE	BIT(0)
#define  RFBI_CONTROL_ITE	BIT(4)	/* internal trigger, cleared by hw */
#define RFBI_PIXEL_CNT		0x44
#define RFBI_CMD		0x4c
#define RFBI_PARAM		0x50

/* DISPC registers (OMAP2 layout, offsets used by d_display.ldd) */
#define DISPC_IRQSTATUS		0x18
#define DISPC_IRQENABLE		0x1c
#define  DISPC_IRQ_FRAMEDONE	BIT(0)
#define  DISPC_IRQ_GFXENDWIN	BIT(7)
#define DISPC_CONTROL		0x40
#define  DISPC_CONTROL_LCDENABLE	BIT(0)
#define  DISPC_CONTROL_GOLCD	BIT(5)
#define DISPC_SIZE_LCD		0x7c
#define DISPC_GFX_BA0		0x80
#define DISPC_GFX_POSITION	0x88
#define DISPC_GFX_SIZE		0x8c
#define DISPC_GFX_ROW_INC	0xac
#define DISPC_GFX_PIXEL_INC	0xb0

#define E5D_DMA_TIMEOUT_US	200000
#define E5D_DMA_IRQS		(DISPC_IRQ_FRAMEDONE | DISPC_IRQ_GFXENDWIN)

/* MIPI DCS */
#define DCS_CASET		0x2a
#define DCS_PASET		0x2b
#define DCS_RAMWR		0x2c
#define DCS_SLPIN		0x10
#define DCS_SLPOUT		0x11
#define DCS_DISPOFF		0x28
#define DCS_DISPON		0x29
#define DCS_WRDISBV		0x51	/* display brightness */
#define DCS_WRCTRLD		0x53	/* ctrl display */
#define  DCS_CTRLD_ON		0x2c	/* BCTRL | DD | BL */

#define E5D_BL_MAX		255

#define E5D_BUSY_LOOPS		100000

static bool busy_poll = true;
module_param(busy_poll, bool, 0644);
MODULE_PARM_DESC(busy_poll, "poll RFBI SYSSTATUS busy before every write (firmware behaviour, default on)");

static bool dma = true;
module_param(dma, bool, 0644);
MODULE_PARM_DESC(dma, "push through DISPC + RFBI DMA instead of CPU writes (default on; needs the DISPC reg and clock in the dt)");

static bool dma_irq = true;
module_param(dma_irq, bool, 0644);
MODULE_PARM_DESC(dma_irq, "wait for the dma push on DISPC IRQ 146 instead of polling IRQSTATUS (default on; a timeout switches it off)");

static unsigned int dma_irq_timeout_ms = 200;
module_param(dma_irq_timeout_ms, uint, 0644);
MODULE_PARM_DESC(dma_irq_timeout_ms, "how long a push waits for the irq before falling back to polling (default 200; 1 forces the fallback)");

struct e5d {
	struct drm_device drm;
	struct drm_simple_display_pipe pipe;
	struct drm_connector conn;

	struct device *dev;
	void __iomem *rfbi;
	void __iomem *dispc;	/* NULL: CPU path only */
	struct clk *dma_clk;	/* gfx dma clock, on during a dma push */
	int irq;		/* DISPC irq, 0: polling only */
	struct completion dma_done;
	bool dma_waiting;	/* a push waits on dma_done (IRQENABLE set) */
	u32 dma_status;		/* FRAMEDONE/GFXENDWIN bits the isr acked, ORed */
	u8 *vmem;		/* what the panel shows (or will after a push) */
	dma_addr_t vmem_dma;	/* valid when dispc is set */
	u8 *row;		/* one converted row, for the comparison */
	struct mutex fb_lock;	/* vmem writes, row */
	u8 *ovl;		/* rows E5D_OVL_Y.. while ovl_on, instead of vmem */
	dma_addr_t ovl_dma;	/* valid when dispc is set */
	bool ovl_on;		/* push_lock */

	/* rows to push; the thread takes them */
	spinlock_t band_lock;
	struct e5d_band band;
	wait_queue_head_t wq;
	struct task_struct *thread;

	struct mutex push_lock;	/* one panel transfer at a time */
	struct backlight_device *bl;
	bool dead;		/* rfbi stuck busy: stop touching it */
	bool asleep;		/* pipe disabled: panel in sleep, no pushes */
	bool dma_broken;	/* dma push timed out: CPU path from now on */
	bool dma_seen;		/* first dma completion logged */
	bool timed[2];		/* first full frame time logged, per path */

	struct dentry *debugfs;
	u64 pushes;		/* push thread */
	u64 push_us_last;
};

static inline struct e5d *to_e5d(struct drm_device *drm)
{
	return container_of(drm, struct e5d, drm);
}

/* ---- panel (RFBI, DISPC), from rapuyama-e5-fb.c ---- */

static bool e5d_wait(struct e5d *e5d)
{
	unsigned int n;

	if (!busy_poll)
		return true;
	for (n = 0; n < E5D_BUSY_LOOPS; n++)
		if (!(readl_relaxed(e5d->rfbi + RFBI_SYSSTATUS) & RFBI_SYSSTATUS_BUSY))
			return true;
	if (!e5d->dead)
		dev_err(e5d->dev, "rfbi stuck busy (sysstatus %#x), panel updates stopped\n",
			readl_relaxed(e5d->rfbi + RFBI_SYSSTATUS));
	e5d->dead = true;
	return false;
}

static inline bool e5d_cmd(struct e5d *e5d, u8 cmd)
{
	if (!e5d_wait(e5d))
		return false;
	writel_relaxed(cmd, e5d->rfbi + RFBI_CMD);
	return true;
}

static inline bool e5d_param(struct e5d *e5d, u8 val)
{
	if (!e5d_wait(e5d))
		return false;
	writel_relaxed(val, e5d->rfbi + RFBI_PARAM);
	return true;
}

/* 16-bit DCS parameters go out as two bytes, high first (vtable +0x78) */
static bool e5d_range(struct e5d *e5d, u8 cmd, u16 start, u16 end)
{
	return e5d_cmd(e5d, cmd) &&
	       e5d_param(e5d, start >> 8) && e5d_param(e5d, start & 0xff) &&
	       e5d_param(e5d, end >> 8) && e5d_param(e5d, end & 0xff);
}

/*
 * CPU path, push_lock held. src = panel row 0 of the source (vmem, or the
 * overlay minus E5D_OVL_Y rows)
 */
static void e5d_push_cpu(struct e5d *e5d, const u8 *src, u32 x0, u32 y0,
			 u32 x1, u32 y1)
{
	u32 x, y;

	/* firmware order: PASET (rows) then CASET (columns), then RAMWR */
	if (!e5d_range(e5d, DCS_PASET, y0, y1 - 1) ||
	    !e5d_range(e5d, DCS_CASET, x0, x1 - 1) ||
	    !e5d_cmd(e5d, DCS_RAMWR))
		return;

	for (y = y0; y < y1; y++) {
		const u32 *p = (const u32 *)(src + y * E5D_STRIDE) + x0;

		for (x = x0; x < x1; x++, p++) {
			u32 v = READ_ONCE(*p);

			if (!e5d_param(e5d, v >> 16) || !e5d_param(e5d, v >> 8) ||
			    !e5d_param(e5d, v))
				return;
		}
		cond_resched();
	}
}

static void e5d_dma_regs(struct e5d *e5d, const char *why)
{
	dev_err(e5d->dev, "dma %s: dispc irqstatus %#x control %#x, rfbi control %#x sysstatus %#x pixel_cnt %#x\n",
		why, readl(e5d->dispc + DISPC_IRQSTATUS),
		readl(e5d->dispc + DISPC_CONTROL),
		readl(e5d->rfbi + RFBI_CONTROL), readl(e5d->rfbi + RFBI_SYSSTATUS),
		readl(e5d->rfbi + RFBI_PIXEL_CNT));
}

/*
 * DISPC irq: only ours while a push has IRQENABLE set. Spurious (nothing of
 * ours enabled and pending) -> IRQ_NONE; pending without a waiting push is
 * acked and dropped. The status is ORed (FRAMEDONE and GFXENDWIN may come in
 * two runs) and stored before the completion; only FRAMEDONE completes.
 */
static irqreturn_t e5d_dma_interrupt(int irq, void *data)
{
	struct e5d *e5d = data;
	u32 st = readl(e5d->dispc + DISPC_IRQSTATUS) &
		 readl(e5d->dispc + DISPC_IRQENABLE) & E5D_DMA_IRQS;

	if (!st)
		return IRQ_NONE;
	writel(st, e5d->dispc + DISPC_IRQSTATUS);
	if (READ_ONCE(e5d->dma_waiting)) {
		WRITE_ONCE(e5d->dma_status, READ_ONCE(e5d->dma_status) | st);
		if (st & DISPC_IRQ_FRAMEDONE)
			complete(&e5d->dma_done);
	}
	return IRQ_HANDLED;
}

/* IRQENABLE off, then no isr is running or will touch dma_status */
static void e5d_dma_irq_off(struct e5d *e5d)
{
	writel(0, e5d->dispc + DISPC_IRQENABLE);
	synchronize_irq(e5d->irq);
	WRITE_ONCE(e5d->dma_waiting, false);
}

/* DMA path (d_display.ldd 0x80180498), push_lock held. false: not pushed */
static bool e5d_push_dma(struct e5d *e5d, dma_addr_t src, u32 x0, u32 y0,
			 u32 x1, u32 y1)
{
	void __iomem *d = e5d->dispc;
	u32 w = x1 - x0, h = y1 - y0, size = (h - 1) << 16 | (w - 1);
	bool use_irq = dma_irq && e5d->irq;
	bool irq_on = false;	/* IRQENABLE set, dma_waiting */
	u32 v;

	if (readl(d + DISPC_CONTROL) & DISPC_CONTROL_LCDENABLE) {
		e5d_dma_regs(e5d, "busy at start");
		return false;
	}
	if (clk_enable(e5d->dma_clk)) {
		e5d->dma_broken = true;
		dev_err(e5d->dev, "dma clock enable failed, cpu path\n");
		return false;
	}
	/* the cpu cache (commit copies, overlay writes) to ram */
	dma_sync_single_for_device(e5d->dev, src + y0 * E5D_STRIDE,
				   h * E5D_STRIDE, DMA_TO_DEVICE);
	writel(E5D_DMA_IRQS, d + DISPC_IRQSTATUS);
	if (use_irq) {
		reinit_completion(&e5d->dma_done);
		WRITE_ONCE(e5d->dma_status, 0);
		WRITE_ONCE(e5d->dma_waiting, true);
		writel(E5D_DMA_IRQS, d + DISPC_IRQENABLE);
		irq_on = true;
	}

	writel(src + y0 * E5D_STRIDE + x0 * E5D_CPP, d + DISPC_GFX_BA0);
	writel(0, d + DISPC_GFX_POSITION);
	writel(size, d + DISPC_GFX_SIZE);
	writel(size, d + DISPC_SIZE_LCD);
	writel(E5D_STRIDE - w * E5D_CPP + 1, d + DISPC_GFX_ROW_INC);
	writel(1, d + DISPC_GFX_PIXEL_INC);
	writel(readl(d + DISPC_CONTROL) | DISPC_CONTROL_GOLCD, d + DISPC_CONTROL);
	writel(readl(d + DISPC_CONTROL) | DISPC_CONTROL_LCDENABLE, d + DISPC_CONTROL);
	udelay(13);

	if (!e5d_range(e5d, DCS_PASET, y0, y1 - 1) ||
	    !e5d_range(e5d, DCS_CASET, x0, x1 - 1) || !e5d_wait(e5d))
		goto stop;
	writel(w * h, e5d->rfbi + RFBI_PIXEL_CNT);
	if (!e5d_cmd(e5d, DCS_RAMWR) || !e5d_wait(e5d))
		goto stop;
	writel(readl(e5d->rfbi + RFBI_CONTROL) | RFBI_CONTROL_ENABLE | RFBI_CONTROL_ITE,
	       e5d->rfbi + RFBI_CONTROL);

	if (use_irq) {
		bool got = wait_for_completion_timeout(&e5d->dma_done,
					msecs_to_jiffies(dma_irq_timeout_ms));

		e5d_dma_irq_off(e5d);
		irq_on = false;
		if (!got)
			got = try_wait_for_completion(&e5d->dma_done);
		v = READ_ONCE(e5d->dma_status);
		if (!got)
			v |= readl(d + DISPC_IRQSTATUS);
		if (!got) {
			/* never again; the push itself is still checked by polling */
			dev_warn(e5d->dev, "no dma irq within %u ms (raw irqstatus %#x), polling from now on\n",
				 dma_irq_timeout_ms, v);
			dma_irq = false;
		}
		if (!(v & DISPC_IRQ_FRAMEDONE) &&
		    readl_poll_timeout(d + DISPC_IRQSTATUS, v,
				       v & DISPC_IRQ_FRAMEDONE, 1000, E5D_DMA_TIMEOUT_US)) {
			e5d_dma_regs(e5d, "timeout, back to the cpu path");
			e5d->dma_broken = true;
			goto stop;
		}
	} else if (readl_poll_timeout(d + DISPC_IRQSTATUS, v, v & DISPC_IRQ_FRAMEDONE,
				      1000, E5D_DMA_TIMEOUT_US)) {
		e5d_dma_regs(e5d, "timeout, back to the cpu path");
		e5d->dma_broken = true;
		goto stop;
	}
	if (!e5d->dma_seen) {	/* evidence for the first transfer */
		e5d->dma_seen = true;
		dev_info(e5d->dev, "dma done: dispc irqstatus %#x control %#x\n",
			 v, readl(d + DISPC_CONTROL));
	}
	e5d_wait(e5d);
	writel(E5D_DMA_IRQS, d + DISPC_IRQSTATUS);
stop:
	if (irq_on)		/* only the early exits get here still armed */
		e5d_dma_irq_off(e5d);
	writel(readl(e5d->rfbi + RFBI_CONTROL) &
	       ~(RFBI_CONTROL_ENABLE | RFBI_CONTROL_ITE), e5d->rfbi + RFBI_CONTROL);
	if (e5d->dma_broken)
		writel(readl(d + DISPC_CONTROL) & ~DISPC_CONTROL_LCDENABLE,
		       d + DISPC_CONTROL);
	clk_disable(e5d->dma_clk);
	return !e5d->dma_broken && !e5d->dead;
}

/* one rectangle from one source, push_lock held. true: went by dma */
static bool e5d_push_src(struct e5d *e5d, const u8 *src, dma_addr_t src_dma,
			 u32 x0, u32 y0, u32 x1, u32 y1)
{
	bool use_dma = dma && e5d->dispc && !e5d->dma_broken;

	if (use_dma && !e5d_push_dma(e5d, src_dma, x0, y0, x1, y1)) {
		use_dma = false;
		if (e5d->dead)
			return false;
	}
	if (!use_dma)
		e5d_push_cpu(e5d, src, x0, y0, x1, y1);
	return use_dma;
}

/*
 * push the rectangle [x0,x1) x [y0,y1) to the panel: from vmem, except rows
 * E5D_OVL_Y.. while the overlay is on (commits keep updating vmem, the
 * panel shows the overlay there)
 */
static void e5d_push(struct e5d *e5d, u32 x0, u32 y0, u32 x1, u32 y1)
{
	ktime_t t0 = ktime_get();
	bool use_dma;
	s64 us;

	mutex_lock(&e5d->push_lock);
	if (e5d->dead || e5d->asleep)
		goto out;
	if (e5d->ovl_on && y1 > E5D_OVL_Y) {
		const u32 skip = E5D_OVL_Y * E5D_STRIDE;

		if (y0 < E5D_OVL_Y)
			e5d_push_src(e5d, e5d->vmem, e5d->vmem_dma, x0, y0, x1, E5D_OVL_Y);
		if (e5d->dead)
			goto out;
		use_dma = e5d_push_src(e5d, e5d->ovl - skip, e5d->ovl_dma - skip, x0,
				       max_t(u32, y0, E5D_OVL_Y), x1, y1);
	} else {
		use_dma = e5d_push_src(e5d, e5d->vmem, e5d->vmem_dma, x0, y0, x1, y1);
	}
	if (e5d->dead)
		goto out;

	us = ktime_us_delta(ktime_get(), t0);
	e5d->pushes++;
	e5d->push_us_last = us;
	if (!e5d->timed[use_dma] && x0 == 0 && y0 == 0 &&
	    x1 == E5D_WIDTH && y1 == E5D_HEIGHT) {
		e5d->timed[use_dma] = true;
		dev_info(e5d->dev, "full frame push %lld us (%s, busy_poll %d)\n",
			 us, use_dma ? "dma" : "cpu", busy_poll);
	}
out:
	mutex_unlock(&e5d->push_lock);
}

/* DCS sleep in, push_lock held (fbdev driver's blank) */
static int e5d_panel_sleep(struct e5d *e5d)
{
	int ret = 0;

	if (e5d->dead)
		return -EIO;
	if (e5d->asleep)
		return 0;
	if (!e5d_cmd(e5d, DCS_WRDISBV) || !e5d_param(e5d, 0) ||
	    !e5d_cmd(e5d, DCS_DISPOFF) || !e5d_cmd(e5d, DCS_SLPIN))
		ret = -EIO;
	e5d->asleep = true;
	/* DCS: 120 ms before the next sleep out */
	msleep(120);
	return ret;
}

/* DCS sleep out + display on + brightness, push_lock held (unblank) */
static int e5d_panel_wake(struct e5d *e5d)
{
	/*
	 * the set level: backlight_get_brightness() is 0 while the core's
	 * fb notifier still has it blanked (it runs after the fb_blank op)
	 */
	int level = e5d->bl ? e5d->bl->props.brightness : E5D_BL_MAX;

	if (e5d->dead)
		return -EIO;
	if (!e5d->asleep)
		return 0;
	e5d->asleep = false;
	if (!e5d_cmd(e5d, DCS_SLPOUT))
		return -EIO;
	msleep(120);
	if (!e5d_cmd(e5d, DCS_DISPON) ||
	    !e5d_cmd(e5d, DCS_WRCTRLD) || !e5d_param(e5d, DCS_CTRLD_ON) ||
	    !e5d_cmd(e5d, DCS_WRDISBV) || !e5d_param(e5d, level))
		return -EIO;
	return 0;
}

/* ---- push thread ---- */

/* rows [y0, y1) changed, the thread pushes their union */
static void e5d_mark_dirty(struct e5d *e5d, u32 y0, u32 y1)
{
	unsigned long flags;

	spin_lock_irqsave(&e5d->band_lock, flags);
	e5d_band_add(&e5d->band, y0, y1);
	spin_unlock_irqrestore(&e5d->band_lock, flags);
	wake_up(&e5d->wq);
}

static bool e5d_take_dirty(struct e5d *e5d, u32 *y0, u32 *y1)
{
	unsigned long flags;
	bool ret;

	spin_lock_irqsave(&e5d->band_lock, flags);
	ret = e5d_band_take(&e5d->band, y0, y1);
	spin_unlock_irqrestore(&e5d->band_lock, flags);
	return ret;
}

static bool e5d_dirty(struct e5d *e5d)
{
	unsigned long flags;
	bool ret;

	spin_lock_irqsave(&e5d->band_lock, flags);
	ret = e5d->band.y0 < e5d->band.y1;
	spin_unlock_irqrestore(&e5d->band_lock, flags);
	return ret;
}

/* sleeps until rows are dirty: an idle display costs no CPU */
static int e5d_thread(void *arg)
{
	struct e5d *e5d = arg;
	u32 y0, y1;

	while (!kthread_should_stop()) {
		wait_event_interruptible(e5d->wq, e5d_dirty(e5d) ||
					 kthread_should_stop());
		if (e5d_take_dirty(e5d, &y0, &y1))
			e5d_push(e5d, 0, y0, E5D_WIDTH, y1);
	}
	return 0;
}

/* ---- drm ---- */

/*
 * the plane rectangle r (framebuffer pixels) into vmem, row by row, only
 * the rows that differ; those get pushed
 */
static void e5d_copy_plane(struct e5d *e5d, struct drm_plane_state *state,
			   const struct drm_rect *r)
{
	struct drm_shadow_plane_state *shadow = to_drm_shadow_plane_state(state);
	struct drm_framebuffer *fb = state->fb;
	const struct drm_rect screen = DRM_RECT_INIT(0, 0, E5D_WIDTH, E5D_HEIGHT);
	struct e5d_band changed = { 0, 0 };
	struct drm_rect src, rect = *r;
	u32 cpp = fb->format->cpp[0];
	const u8 *s;
	int idx;

	/* fb coordinates -> panel coordinates (the plane covers the crtc) */
	drm_rect_fp_to_int(&src, &state->src);
	drm_rect_translate(&rect, -src.x1, -src.y1);
	if (!drm_rect_intersect(&rect, &screen))
		return;
	s = (const u8 *)shadow->data[0].vaddr + src.y1 * fb->pitches[0] +
	    src.x1 * cpp;

	if (!drm_dev_enter(&e5d->drm, &idx))
		return;
	if (!drm_gem_fb_begin_cpu_access(fb, DMA_FROM_DEVICE)) {
		mutex_lock(&e5d->fb_lock);
		e5d_copy_rows(e5d->vmem, E5D_STRIDE, E5D_CPP, s, fb->pitches[0],
			      cpp, rect.x1, rect.x2, rect.y1, rect.y2, e5d->row,
			      &changed);
		mutex_unlock(&e5d->fb_lock);
		drm_gem_fb_end_cpu_access(fb, DMA_FROM_DEVICE);
	}
	drm_dev_exit(idx);
	if (changed.y0 < changed.y1)
		e5d_mark_dirty(e5d, changed.y0, changed.y1);
}

static void e5d_pipe_enable(struct drm_simple_display_pipe *pipe,
			    struct drm_crtc_state *crtc_state,
			    struct drm_plane_state *plane_state)
{
	struct e5d *e5d = to_e5d(pipe->crtc.dev);
	struct drm_rect rect;
	int idx, ret;

	if (!drm_dev_enter(&e5d->drm, &idx))
		return;
	mutex_lock(&e5d->push_lock);
	ret = e5d_panel_wake(e5d);
	mutex_unlock(&e5d->push_lock);
	drm_dev_exit(idx);
	if (ret)
		dev_err(e5d->dev, "panel wake failed\n");

	if (plane_state->fb) {
		drm_rect_fp_to_int(&rect, &plane_state->src);
		e5d_copy_plane(e5d, plane_state, &rect);
	}
	/* the panel keeps its gram in sleep, but nothing was pushed since */
	e5d_mark_dirty(e5d, 0, E5D_HEIGHT);
}

static void e5d_pipe_disable(struct drm_simple_display_pipe *pipe)
{
	struct e5d *e5d = to_e5d(pipe->crtc.dev);
	int idx, ret;

	if (!drm_dev_enter(&e5d->drm, &idx))
		return;
	mutex_lock(&e5d->push_lock);
	ret = e5d_panel_sleep(e5d);
	mutex_unlock(&e5d->push_lock);
	drm_dev_exit(idx);
	if (ret)
		dev_err(e5d->dev, "panel sleep failed\n");
}

/* without clips (weston) the damage is the whole plane: rows are diffed */
static void e5d_pipe_update(struct drm_simple_display_pipe *pipe,
			    struct drm_plane_state *old_state)
{
	struct drm_plane_state *state = pipe->plane.state;
	struct drm_rect rect;

	if (state->fb && drm_atomic_helper_damage_merged(old_state, state, &rect))
		e5d_copy_plane(to_e5d(pipe->crtc.dev), state, &rect);
}

static const struct drm_simple_display_pipe_funcs e5d_pipe_funcs = {
	.enable = e5d_pipe_enable,
	.disable = e5d_pipe_disable,
	.update = e5d_pipe_update,
	DRM_GEM_SIMPLE_DISPLAY_PIPE_SHADOW_PLANE_FUNCS,
};

static const u32 e5d_formats[] = { DRM_FORMAT_XRGB8888 };
static const u64 e5d_modifiers[] = { DRM_FORMAT_MOD_LINEAR,
				     DRM_FORMAT_MOD_INVALID };

static const struct drm_display_mode e5d_mode = {
	DRM_MODE_INIT(60, E5D_WIDTH, E5D_HEIGHT, 53, 40),
};

static int e5d_conn_get_modes(struct drm_connector *conn)
{
	return drm_connector_helper_get_modes_fixed(conn, &e5d_mode);
}

static const struct drm_connector_helper_funcs e5d_conn_helper_funcs = {
	.get_modes = e5d_conn_get_modes,
};

static const struct drm_connector_funcs e5d_conn_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static const struct drm_mode_config_funcs e5d_mode_config_funcs = {
	.fb_create = drm_gem_fb_create_with_dirty,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

DEFINE_DRM_GEM_FOPS(e5d_drm_fops);

static const struct drm_driver e5d_drm_driver = {
	.driver_features = DRIVER_MODESET | DRIVER_GEM | DRIVER_ATOMIC,
	.name = "rapuyama-e5",
	.desc = "Nokia E5-00 RapuYama RFBI panel",
	.date = "20261113",
	.major = 1,
	.fops = &e5d_drm_fops,
	DRM_GEM_SHMEM_DRIVER_OPS,
};

static int e5d_drm_init(struct e5d *e5d)
{
	struct drm_device *drm = &e5d->drm;
	int ret;

	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;
	drm->mode_config.min_width = drm->mode_config.max_width = E5D_WIDTH;
	drm->mode_config.min_height = drm->mode_config.max_height = E5D_HEIGHT;
	drm->mode_config.preferred_depth = 24;
	drm->mode_config.funcs = &e5d_mode_config_funcs;
	/*
	 * mmap writers of /dev/fb0 (X fbdev, SDL 1.2): the fbdev driver's
	 * 10 ms, not the default 50 ms; read once by the fbdev setup
	 */
	drm->mode_config.fbdev_defio_delay_ms = 10;

	drm_connector_helper_add(&e5d->conn, &e5d_conn_helper_funcs);
	ret = drm_connector_init(drm, &e5d->conn, &e5d_conn_funcs,
				 DRM_MODE_CONNECTOR_DPI);
	if (ret)
		return ret;
	ret = drm_simple_display_pipe_init(drm, &e5d->pipe, &e5d_pipe_funcs,
					   e5d_formats, ARRAY_SIZE(e5d_formats),
					   e5d_modifiers, &e5d->conn);
	if (ret)
		return ret;
	drm_plane_enable_fb_damage_clips(&e5d->pipe.plane);
	drm_mode_config_reset(drm);
	return 0;
}

/* ---- backlight ---- */

static int e5d_bl_update(struct backlight_device *bl)
{
	struct e5d *e5d = bl_get_data(bl);
	int level = backlight_get_brightness(bl);
	int ret = 0;

	mutex_lock(&e5d->push_lock);
	if (e5d->asleep)		/* sent again on wake */
		goto out;
	if (e5d->dead ||
	    !e5d_cmd(e5d, DCS_WRCTRLD) || !e5d_param(e5d, DCS_CTRLD_ON) ||
	    !e5d_cmd(e5d, DCS_WRDISBV) || !e5d_param(e5d, level))
		ret = -EIO;
out:
	mutex_unlock(&e5d->push_lock);
	return ret;
}

static const struct backlight_ops e5d_bl_ops = {
	.update_status	= e5d_bl_update,
};

/* ---- overlay (sysfs on the platform device = fb0/device) ---- */

/* repaint the overlay rows from whatever is shown there now */
static void e5d_ovl_damage(struct e5d *e5d)
{
	e5d_mark_dirty(e5d, E5D_OVL_Y, E5D_HEIGHT);
}

/*
 * overlay: the picture for rows E5D_OVL_Y..E5D_HEIGHT-1, xrgb8888, stride
 * E5D_STRIDE, written in one go or in chunks (sysfs splits big writes);
 * shown after the chunk that ends at E5D_OVL_SIZE
 */
static ssize_t overlay_write(struct file *file, struct kobject *kobj,
			     struct bin_attribute *attr, char *buf,
			     loff_t off, size_t count)
{
	struct e5d *e5d = dev_get_drvdata(kobj_to_dev(kobj));

	if (!e5d->ovl)
		return -ENOMEM;
	if (off < 0 || off + count > E5D_OVL_SIZE)
		return -EINVAL;
	mutex_lock(&e5d->push_lock);
	memcpy(e5d->ovl + off, buf, count);
	mutex_unlock(&e5d->push_lock);
	if (off + count == E5D_OVL_SIZE && READ_ONCE(e5d->ovl_on))
		e5d_ovl_damage(e5d);
	return count;
}
static BIN_ATTR_WO(overlay, E5D_OVL_SIZE);

static ssize_t overlay_on_show(struct device *dev, struct device_attribute *attr,
			       char *buf)
{
	struct e5d *e5d = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", READ_ONCE(e5d->ovl_on));
}

static ssize_t overlay_on_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct e5d *e5d = dev_get_drvdata(dev);
	bool on;
	int ret;

	ret = kstrtobool(buf, &on);
	if (ret)
		return ret;
	if (on && !e5d->ovl)
		return -ENOMEM;
	mutex_lock(&e5d->push_lock);
	e5d->ovl_on = on;
	mutex_unlock(&e5d->push_lock);
	e5d_ovl_damage(e5d);
	return count;
}
static DEVICE_ATTR_RW(overlay_on);

static struct attribute *e5d_attrs[] = {
	&dev_attr_overlay_on.attr,
	NULL
};

static struct bin_attribute *e5d_bin_attrs[] = {
	&bin_attr_overlay,
	NULL
};

static const struct attribute_group e5d_group = {
	.attrs = e5d_attrs,
	.bin_attrs = e5d_bin_attrs,
};
__ATTRIBUTE_GROUPS(e5d);

/* ---- probe ---- */

static void e5d_free_bufs(struct e5d *e5d)
{
	if (e5d->ovl && e5d->dispc)
		dma_unmap_single(e5d->dev, e5d->ovl_dma, E5D_OVL_SIZE, DMA_TO_DEVICE);
	kfree(e5d->ovl);
	kfree(e5d->row);
	if (!e5d->vmem)
		return;
	if (e5d->dispc) {
		dma_unmap_single(e5d->dev, e5d->vmem_dma, E5D_SIZE, DMA_TO_DEVICE);
		free_pages_exact(e5d->vmem, PAGE_ALIGN(E5D_SIZE));
	} else {
		vfree(e5d->vmem);
	}
}

/* vmem (contiguous for the DISPC when it is there), overlay, row */
static int e5d_alloc_bufs(struct e5d *e5d)
{
	struct device *dev = e5d->dev;

	if (e5d->dispc) {
		/* DISPC reads it: physically contiguous, cached + streaming sync */
		e5d->vmem = alloc_pages_exact(PAGE_ALIGN(E5D_SIZE),
					      GFP_KERNEL | __GFP_ZERO);
		if (e5d->vmem) {
			e5d->vmem_dma = dma_map_single(dev, e5d->vmem, E5D_SIZE,
						       DMA_TO_DEVICE);
			if (dma_mapping_error(dev, e5d->vmem_dma)) {
				free_pages_exact(e5d->vmem, PAGE_ALIGN(E5D_SIZE));
				e5d->vmem = NULL;
			}
		}
		if (!e5d->vmem) {
			dev_warn(dev, "no contiguous buffer, cpu path only\n");
			e5d->dispc = NULL;
		}
	}
	if (!e5d->vmem)
		e5d->vmem = vzalloc(PAGE_ALIGN(E5D_SIZE));
	e5d->row = kmalloc(E5D_STRIDE, GFP_KERNEL);
	if (!e5d->vmem || !e5d->row)
		return -ENOMEM;
	/* overlay: kmalloc is contiguous at this size; optional (sysfs -ENOMEM) */
	e5d->ovl = kzalloc(E5D_OVL_SIZE, GFP_KERNEL);
	if (e5d->ovl && e5d->dispc) {
		e5d->ovl_dma = dma_map_single(dev, e5d->ovl, E5D_OVL_SIZE,
					      DMA_TO_DEVICE);
		if (dma_mapping_error(dev, e5d->ovl_dma)) {
			kfree(e5d->ovl);
			e5d->ovl = NULL;
		}
	}
	return 0;
}

static void e5d_debugfs_init(struct e5d *e5d)
{
	e5d->debugfs = debugfs_create_dir("rapuyama-e5", NULL);
	debugfs_create_u64("pushes", 0444, e5d->debugfs, &e5d->pushes);
	debugfs_create_u64("push_us_last", 0444, e5d->debugfs,
			   &e5d->push_us_last);
}

static int e5d_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct backlight_properties props = {
		.type		= BACKLIGHT_RAW,
		.max_brightness	= E5D_BL_MAX,
		.brightness	= E5D_BL_MAX,
	};
	struct e5d *e5d;
	int ret;

	e5d = devm_drm_dev_alloc(dev, &e5d_drm_driver, struct e5d, drm);
	if (IS_ERR(e5d))
		return PTR_ERR(e5d);
	e5d->dev = dev;
	mutex_init(&e5d->fb_lock);
	mutex_init(&e5d->push_lock);
	spin_lock_init(&e5d->band_lock);
	init_waitqueue_head(&e5d->wq);
	init_completion(&e5d->dma_done);
	platform_set_drvdata(pdev, e5d);

	e5d->rfbi = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(e5d->rfbi))
		return PTR_ERR(e5d->rfbi);

	/* DISPC (2nd reg) is optional: without it the CPU path only */
	if (platform_get_resource(pdev, IORESOURCE_MEM, 1)) {
		e5d->dispc = devm_platform_ioremap_resource(pdev, 1);
		if (IS_ERR(e5d->dispc))
			return PTR_ERR(e5d->dispc);
		/* gfx dma clock; without it (old dtb) the dma can't run */
		e5d->dma_clk = devm_clk_get_optional_prepared(dev, NULL);
		if (IS_ERR(e5d->dma_clk))
			return PTR_ERR(e5d->dma_clk);
		if (!e5d->dma_clk) {
			dev_warn(dev, "no dma clock in the dt, cpu path only\n");
			e5d->dispc = NULL;
		}
	}

	ret = e5d_alloc_bufs(e5d);
	if (ret)
		goto err_bufs;

	/* DISPC irq, optional: without it (or if the request fails) polling */
	if (e5d->dispc && dma_irq) {
		int irq = platform_get_irq_optional(pdev, 0);

		if (irq == -EPROBE_DEFER) {
			ret = irq;
			goto err_bufs;
		}
		if (irq > 0) {
			/* nothing enabled until a push; symbian may have left bits set */
			writel(0, e5d->dispc + DISPC_IRQENABLE);
			writel(E5D_DMA_IRQS, e5d->dispc + DISPC_IRQSTATUS);
			ret = devm_request_irq(dev, irq, e5d_dma_interrupt, 0,
					       "e5-display", e5d);
			if (ret) {
				dev_warn(dev, "dispc irq %d unavailable (%d), completion by polling\n",
					 irq, ret);
			} else {
				e5d->irq = irq;
				dev_info(dev, "dma completion on irq %d\n", irq);
			}
		} else {
			dev_info(dev, "no dispc irq in the dt, completion by polling\n");
		}
	}

	/* start from a known (black) panel; this push is also the timing sample */
	e5d_push(e5d, 0, 0, E5D_WIDTH, E5D_HEIGHT);
	if (e5d->dead) {
		ret = -EIO;
		goto err_bufs;
	}

	/* the panel level symbian left can't be read back: start at full */
	e5d->bl = devm_backlight_device_register(dev, "panel", dev, e5d,
						 &e5d_bl_ops, &props);
	if (IS_ERR(e5d->bl)) {
		dev_warn(dev, "no backlight device (%pe)\n", e5d->bl);
		e5d->bl = NULL;
	} else {
		backlight_update_status(e5d->bl);
	}

	ret = e5d_drm_init(e5d);
	if (ret)
		goto err_bufs;

	e5d->thread = kthread_run(e5d_thread, e5d, "e5-display");
	if (IS_ERR(e5d->thread)) {
		ret = PTR_ERR(e5d->thread);
		goto err_bufs;
	}
	ret = drm_dev_register(&e5d->drm, 0);
	if (ret)
		goto err_thread;
	/* fbcon and /dev/fb0 through the drm fbdev emulation */
	drm_fbdev_shmem_setup(&e5d->drm, 32);
	e5d_debugfs_init(e5d);

	dev_info(dev, "%ux%u xrgb8888, rfbi cpu path%s\n", E5D_WIDTH,
		 E5D_HEIGHT, e5d->dispc ? (e5d->irq && dma_irq ? " + dispc dma (irq)" :
					   " + dispc dma (polled)") : "");
	return 0;

err_thread:
	kthread_stop(e5d->thread);
err_bufs:
	e5d_free_bufs(e5d);
	return ret;
}

static void e5d_remove(struct platform_device *pdev)
{
	struct e5d *e5d = platform_get_drvdata(pdev);

	debugfs_remove_recursive(e5d->debugfs);
	drm_dev_unplug(&e5d->drm);
	drm_atomic_helper_shutdown(&e5d->drm);
	kthread_stop(e5d->thread);
	e5d_free_bufs(e5d);
}

static const struct of_device_id e5d_of_match[] = {
	{ .compatible = "nokia,rapuyama-e5-rfbi" },
	{ }
};
MODULE_DEVICE_TABLE(of, e5d_of_match);

static struct platform_driver e5d_driver = {
	.probe	= e5d_probe,
	.remove	= e5d_remove,
	.driver	= {
		.name		= "rapuyama-e5-display",
		.of_match_table	= e5d_of_match,
		.dev_groups	= e5d_groups,
	},
};

/* late: drivers/soc links before drivers/gpu, drm core is not up at module_init */
static int __init e5d_init(void)
{
	return platform_driver_register(&e5d_driver);
}
late_initcall(e5d_init);

static void __exit e5d_exit(void)
{
	platform_driver_unregister(&e5d_driver);
}
module_exit(e5d_exit);

MODULE_DESCRIPTION("Nokia E5-00 RapuYama RFBI panel KMS driver");
MODULE_LICENSE("GPL");
