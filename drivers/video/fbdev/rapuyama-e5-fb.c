// SPDX-License-Identifier: GPL-2.0-only
/*
 * Nokia E5-00 (RapuYama) display: fbdev over the OMAP2-style RFBI that
 * Symbian leaves set up for a MIPI DCS command-mode panel (320x240, 8-bit
 * bus, RGB888 as three bytes per pixel).
 *
 * CPU path only: a shadow buffer in normal memory (XRGB8888) is pushed to the
 * panel GRAM through RFBI CMD/PARAM writes (window PASET/CASET, RAMWR, then
 * R,G,B per pixel), the way d_display.ldd's fill-rectangle routine does it
 * (ROM 0x801820a8). DISPC is not touched, RFBI is not reset or reprogrammed.
 * Measured on the device (D2, devmem): PASET = rows, CASET = columns, origin
 * top left, window offsets 0.
 *
 * Screen brightness is the panel's own DCS backlight control (P1, tried by
 * hand on the device): WRCTRLD 0x53 = 0x2c (BCTRL | DD | BL on), then
 * WRDISBV 0x51 = level 0..255. Exposed as /sys/class/backlight/panel; the
 * commands go out under push_lock so they never land inside a RAMWR stream.
 * The gazoo pwm (rapuyama-e5-bl.c) is not the screen.
 *
 * Blanking (fbcon's console blank, consoleblank= / setterm, or the fb0
 * blank attribute) puts the panel to sleep: WRDISBV 0, display off 0x28,
 * sleep in 0x10. Unblank: sleep out 0x11, 120 ms, display on 0x29, the
 * brightness again and a full frame push. No pushes while asleep. 0x28 and
 * 0x10 are standard DCS; d_display.ldd only uses 0x11/0x29 (wake side).
 */
#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/io.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>

#define E5FB_WIDTH		320
#define E5FB_HEIGHT		240
#define E5FB_BPP		32
#define E5FB_STRIDE		(E5FB_WIDTH * E5FB_BPP / 8)
#define E5FB_SIZE		(E5FB_STRIDE * E5FB_HEIGHT)

/* RFBI registers (OMAP2 layout, offsets used by d_display.ldd) */
#define RFBI_SYSSTATUS		0x14
#define  RFBI_SYSSTATUS_BUSY	BIT(8)
#define RFBI_CMD		0x4c
#define RFBI_PARAM		0x50

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

#define E5FB_BL_MAX		255

#define E5FB_BUSY_LOOPS		100000

static bool busy_poll = true;
module_param(busy_poll, bool, 0644);
MODULE_PARM_DESC(busy_poll, "poll RFBI SYSSTATUS busy before every write (firmware behaviour, default on)");

static unsigned int refresh_ms = 40;
module_param(refresh_ms, uint, 0644);
MODULE_PARM_DESC(refresh_ms, "delay between a drawing op and the panel push in ms (default 40)");

struct e5fb {
	struct device *dev;
	void __iomem *rfbi;
	struct fb_info *info;
	u8 *vmem;

	/* pending damage, in pixels, x1/y1 exclusive; empty when x0 >= x1 */
	spinlock_t dmg_lock;
	u32 x0, y0, x1, y1;
	struct delayed_work work;

	struct mutex push_lock;	/* one panel transfer at a time */
	struct backlight_device *bl;
	bool dead;		/* rfbi stuck busy: stop touching it */
	bool asleep;		/* blanked: panel in sleep, no pushes */
	bool timed;		/* first full frame time logged */
	u32 pseudo_palette[16];
};

static bool e5fb_wait(struct e5fb *fb)
{
	unsigned int n;

	if (!busy_poll)
		return true;
	for (n = 0; n < E5FB_BUSY_LOOPS; n++)
		if (!(readl_relaxed(fb->rfbi + RFBI_SYSSTATUS) & RFBI_SYSSTATUS_BUSY))
			return true;
	if (!fb->dead)
		dev_err(fb->dev, "rfbi stuck busy (sysstatus %#x), panel updates stopped\n",
			readl_relaxed(fb->rfbi + RFBI_SYSSTATUS));
	fb->dead = true;
	return false;
}

static inline bool e5fb_cmd(struct e5fb *fb, u8 cmd)
{
	if (!e5fb_wait(fb))
		return false;
	writel_relaxed(cmd, fb->rfbi + RFBI_CMD);
	return true;
}

static inline bool e5fb_param(struct e5fb *fb, u8 val)
{
	if (!e5fb_wait(fb))
		return false;
	writel_relaxed(val, fb->rfbi + RFBI_PARAM);
	return true;
}

/* 16-bit DCS parameters go out as two bytes, high first (vtable +0x78) */
static bool e5fb_range(struct e5fb *fb, u8 cmd, u16 start, u16 end)
{
	return e5fb_cmd(fb, cmd) &&
	       e5fb_param(fb, start >> 8) && e5fb_param(fb, start & 0xff) &&
	       e5fb_param(fb, end >> 8) && e5fb_param(fb, end & 0xff);
}

/* push the rectangle [x0,x1) x [y0,y1) of the shadow buffer to the panel */
static void e5fb_push(struct e5fb *fb, u32 x0, u32 y0, u32 x1, u32 y1)
{
	ktime_t t0 = ktime_get();
	u32 x, y;

	mutex_lock(&fb->push_lock);
	if (fb->dead || fb->asleep)
		goto out;
	/* firmware order: PASET (rows) then CASET (columns), then RAMWR */
	if (!e5fb_range(fb, DCS_PASET, y0, y1 - 1) ||
	    !e5fb_range(fb, DCS_CASET, x0, x1 - 1) ||
	    !e5fb_cmd(fb, DCS_RAMWR))
		goto out;

	for (y = y0; y < y1; y++) {
		const u32 *p = (const u32 *)(fb->vmem + y * E5FB_STRIDE) + x0;

		for (x = x0; x < x1; x++, p++) {
			u32 v = READ_ONCE(*p);

			if (!e5fb_param(fb, v >> 16) || !e5fb_param(fb, v >> 8) ||
			    !e5fb_param(fb, v))
				goto out;
		}
		cond_resched();
	}

	if (!fb->timed && x0 == 0 && y0 == 0 &&
	    x1 == E5FB_WIDTH && y1 == E5FB_HEIGHT) {
		fb->timed = true;
		dev_info(fb->dev, "full frame push %lld us (busy_poll %d)\n",
			 ktime_us_delta(ktime_get(), t0), busy_poll);
	}
out:
	mutex_unlock(&fb->push_lock);
}

static int e5fb_bl_update(struct backlight_device *bl)
{
	struct e5fb *fb = bl_get_data(bl);
	int level = backlight_get_brightness(bl);
	int ret = 0;

	mutex_lock(&fb->push_lock);
	if (fb->asleep)		/* sent again on unblank */
		goto out;
	if (fb->dead ||
	    !e5fb_cmd(fb, DCS_WRCTRLD) || !e5fb_param(fb, DCS_CTRLD_ON) ||
	    !e5fb_cmd(fb, DCS_WRDISBV) || !e5fb_param(fb, level))
		ret = -EIO;
out:
	mutex_unlock(&fb->push_lock);
	return ret;
}

static const struct backlight_ops e5fb_bl_ops = {
	.update_status	= e5fb_bl_update,
};

static void e5fb_damage(struct e5fb *fb, u32 x, u32 y, u32 w, u32 h)
{
	unsigned long flags;
	u32 x1, y1;

	if (!w || !h || x >= E5FB_WIDTH || y >= E5FB_HEIGHT)
		return;
	x1 = min_t(u32, x + w, E5FB_WIDTH);
	y1 = min_t(u32, y + h, E5FB_HEIGHT);

	spin_lock_irqsave(&fb->dmg_lock, flags);
	if (fb->x0 >= fb->x1) {
		fb->x0 = x;
		fb->y0 = y;
		fb->x1 = x1;
		fb->y1 = y1;
	} else {
		fb->x0 = min(fb->x0, x);
		fb->y0 = min(fb->y0, y);
		fb->x1 = max(fb->x1, x1);
		fb->y1 = max(fb->y1, y1);
	}
	spin_unlock_irqrestore(&fb->dmg_lock, flags);
}

static void e5fb_work(struct work_struct *work)
{
	struct e5fb *fb = container_of(to_delayed_work(work), struct e5fb, work);
	unsigned long flags;
	u32 x0, y0, x1, y1;

	spin_lock_irqsave(&fb->dmg_lock, flags);
	x0 = fb->x0;
	y0 = fb->y0;
	x1 = fb->x1;
	y1 = fb->y1;
	fb->x0 = fb->x1 = 0;
	spin_unlock_irqrestore(&fb->dmg_lock, flags);

	if (x0 < x1)
		e5fb_push(fb, x0, y0, x1, y1);
}

/* drawing ops may run in atomic context (printk -> fbcon): only record */
static void e5fb_damage_area(struct fb_info *info, u32 x, u32 y, u32 w, u32 h)
{
	struct e5fb *fb = info->par;

	e5fb_damage(fb, x, y, w, h);
	schedule_delayed_work(&fb->work, msecs_to_jiffies(refresh_ms));
}

static void e5fb_damage_range(struct fb_info *info, off_t off, size_t len)
{
	u32 first = off / E5FB_STRIDE;
	u32 last = (off + len - 1) / E5FB_STRIDE;

	if (!len)
		return;
	e5fb_damage_area(info, 0, first, E5FB_WIDTH, last - first + 1);
}

FB_GEN_DEFAULT_DEFERRED_SYSMEM_OPS(e5fb, e5fb_damage_range, e5fb_damage_area)

/* mmap writes: called from the defio work with the touched pages */
static void e5fb_deferred_io(struct fb_info *info, struct list_head *pagereflist)
{
	struct e5fb *fb = info->par;
	struct fb_deferred_io_pageref *ref;
	unsigned long lo = ULONG_MAX, hi = 0;

	list_for_each_entry(ref, pagereflist, list) {
		lo = min(lo, ref->offset);
		hi = max(hi, ref->offset + PAGE_SIZE);
	}
	if (lo < hi && lo < E5FB_SIZE) {
		hi = min_t(unsigned long, hi, E5FB_SIZE);
		e5fb_damage(fb, 0, lo / E5FB_STRIDE, E5FB_WIDTH,
			    DIV_ROUND_UP(hi, E5FB_STRIDE) - lo / E5FB_STRIDE);
	}
	/* push now, together with anything the drawing ops queued */
	cancel_delayed_work(&fb->work);
	e5fb_work(&fb->work.work);
}

/* process context (console work, sysfs): sleeping is fine */
static int e5fb_blank(int blank, struct fb_info *info)
{
	struct e5fb *fb = info->par;
	bool sleep = blank != FB_BLANK_UNBLANK;
	/* the set level: backlight_get_brightness() is 0 while the core's
	 * fb notifier still has it blanked (it runs after this op) */
	int level = fb->bl ? fb->bl->props.brightness : E5FB_BL_MAX;
	int ret = 0;

	mutex_lock(&fb->push_lock);
	if (fb->dead) {
		ret = -EIO;
	} else if (sleep && !fb->asleep) {
		if (!e5fb_cmd(fb, DCS_WRDISBV) || !e5fb_param(fb, 0) ||
		    !e5fb_cmd(fb, DCS_DISPOFF) || !e5fb_cmd(fb, DCS_SLPIN))
			ret = -EIO;
		fb->asleep = true;
		/* DCS: 120 ms before the next sleep out */
		msleep(120);
	} else if (!sleep && fb->asleep) {
		fb->asleep = false;
		if (!e5fb_cmd(fb, DCS_SLPOUT))
			ret = -EIO;
		msleep(120);
		if (!ret && (!e5fb_cmd(fb, DCS_DISPON) ||
			     !e5fb_cmd(fb, DCS_WRCTRLD) || !e5fb_param(fb, DCS_CTRLD_ON) ||
			     !e5fb_cmd(fb, DCS_WRDISBV) || !e5fb_param(fb, level)))
			ret = -EIO;
	}
	mutex_unlock(&fb->push_lock);
	if (!sleep && !ret) {
		/* the panel keeps its gram in sleep, but nothing was pushed since */
		e5fb_damage(fb, 0, 0, E5FB_WIDTH, E5FB_HEIGHT);
		schedule_delayed_work(&fb->work, 0);
	}
	if (ret)
		dev_err(fb->dev, "%s failed\n", sleep ? "blank" : "unblank");
	return ret;
}

static int e5fb_setcolreg(unsigned int regno, unsigned int red, unsigned int green,
			  unsigned int blue, unsigned int transp, struct fb_info *info)
{
	struct e5fb *fb = info->par;

	if (regno >= ARRAY_SIZE(fb->pseudo_palette))
		return -EINVAL;
	fb->pseudo_palette[regno] = ((red >> 8) << 16) | ((green >> 8) << 8) |
				    (blue >> 8);
	return 0;
}

static const struct fb_ops e5fb_ops = {
	.owner		= THIS_MODULE,
	FB_DEFAULT_DEFERRED_OPS(e5fb),
	.fb_setcolreg	= e5fb_setcolreg,
	.fb_blank	= e5fb_blank,
};

static int e5fb_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct backlight_properties props = {
		.type		= BACKLIGHT_RAW,
		.max_brightness	= E5FB_BL_MAX,
		.brightness	= E5FB_BL_MAX,
	};
	struct fb_deferred_io *defio;
	struct fb_info *info;
	struct e5fb *fb;
	int ret;

	info = framebuffer_alloc(sizeof(*fb), dev);
	if (!info)
		return -ENOMEM;
	fb = info->par;
	fb->dev = dev;
	fb->info = info;
	spin_lock_init(&fb->dmg_lock);
	mutex_init(&fb->push_lock);
	INIT_DELAYED_WORK(&fb->work, e5fb_work);

	fb->rfbi = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(fb->rfbi)) {
		ret = PTR_ERR(fb->rfbi);
		goto err_release;
	}

	fb->vmem = vzalloc(PAGE_ALIGN(E5FB_SIZE));
	if (!fb->vmem) {
		ret = -ENOMEM;
		goto err_release;
	}

	defio = devm_kzalloc(dev, sizeof(*defio), GFP_KERNEL);
	if (!defio) {
		ret = -ENOMEM;
		goto err_vfree;
	}
	defio->delay = msecs_to_jiffies(refresh_ms);
	defio->deferred_io = e5fb_deferred_io;

	strscpy(info->fix.id, "rapuyama-e5", sizeof(info->fix.id));
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.visual = FB_VISUAL_TRUECOLOR;
	info->fix.line_length = E5FB_STRIDE;
	info->fix.smem_len = E5FB_SIZE;
	info->fix.accel = FB_ACCEL_NONE;

	info->var.xres = info->var.xres_virtual = E5FB_WIDTH;
	info->var.yres = info->var.yres_virtual = E5FB_HEIGHT;
	info->var.bits_per_pixel = E5FB_BPP;
	info->var.red = (struct fb_bitfield){ .offset = 16, .length = 8 };
	info->var.green = (struct fb_bitfield){ .offset = 8, .length = 8 };
	info->var.blue = (struct fb_bitfield){ .offset = 0, .length = 8 };
	info->var.activate = FB_ACTIVATE_NOW;
	/* 2.36" panel, about 48 x 36 mm */
	info->var.width = 48;
	info->var.height = 36;
	info->var.vmode = FB_VMODE_NONINTERLACED;

	info->fbops = &e5fb_ops;
	info->flags = FBINFO_VIRTFB;
	info->screen_buffer = fb->vmem;
	info->pseudo_palette = fb->pseudo_palette;
	info->fbdefio = defio;

	ret = fb_deferred_io_init(info);
	if (ret)
		goto err_vfree;

	platform_set_drvdata(pdev, fb);

	/* start from a known (black) panel; this push is also the timing sample */
	e5fb_push(fb, 0, 0, E5FB_WIDTH, E5FB_HEIGHT);
	if (fb->dead) {
		ret = -EIO;
		goto err_defio;
	}

	/* the panel level symbian left can't be read back: start at full */
	fb->bl = backlight_device_register("panel", dev, fb, &e5fb_bl_ops, &props);
	if (IS_ERR(fb->bl)) {
		dev_warn(dev, "no backlight device (%pe)\n", fb->bl);
		fb->bl = NULL;
	} else {
		backlight_update_status(fb->bl);
	}

	ret = register_framebuffer(info);
	if (ret)
		goto err_bl;

	dev_info(dev, "fb%d: %ux%u xrgb8888, rfbi cpu path\n", info->node,
		 E5FB_WIDTH, E5FB_HEIGHT);
	return 0;

err_bl:
	backlight_device_unregister(fb->bl);
err_defio:
	fb_deferred_io_cleanup(info);
err_vfree:
	vfree(fb->vmem);
err_release:
	framebuffer_release(info);
	return ret;
}

static void e5fb_remove(struct platform_device *pdev)
{
	struct e5fb *fb = platform_get_drvdata(pdev);
	struct fb_info *info = fb->info;

	unregister_framebuffer(info);
	backlight_device_unregister(fb->bl);
	cancel_delayed_work_sync(&fb->work);
	fb_deferred_io_cleanup(info);
	vfree(fb->vmem);
	framebuffer_release(info);
}

static const struct of_device_id e5fb_of_match[] = {
	{ .compatible = "nokia,rapuyama-e5-rfbi" },
	{ }
};
MODULE_DEVICE_TABLE(of, e5fb_of_match);

static struct platform_driver e5fb_driver = {
	.probe	= e5fb_probe,
	.remove	= e5fb_remove,
	.driver	= {
		.name		= "rapuyama-e5-fb",
		.of_match_table	= e5fb_of_match,
	},
};
module_platform_driver(e5fb_driver);

MODULE_DESCRIPTION("Nokia E5-00 RapuYama RFBI panel framebuffer");
MODULE_LICENSE("GPL");
