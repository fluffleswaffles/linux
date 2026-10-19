// SPDX-License-Identifier: GPL-2.0-only
/*
 * Boots the VideoCore over the ViSSI host bus, runs VCHI on top of it,
 * brings the LCD panel up through the display command service (DSPC) and
 * exposes a 640x480 XRGB8888 KMS device (/dev/dri/card0, plus /dev/fb0
 * through the drm fbdev emulation). Frames are pushed into a dispman
 * resource with vc_bufmanx (service BFMX) and VCHI bulk transfers.
 * Everything here follows the Symbian drivers in the E6 ROM (vcdriver.dll,
 * vchi.dll, dispcommanddrv.dll, display_chipset, vcdispman.dll,
 * vc_bufmanx); the message sequences were captured by running that code in
 * an emulator, see vc-link-notes.md in rapuyama-linux.
 * The bus is driven by PIO from one kernel thread that owns it completely:
 * it boots the VC, sets the display up and then pushes dirty frames.
 */
#include <drm/drm_atomic_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fbdev_shmem.h>
#include <drm/drm_format_helper.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_gem_shmem_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>
#include <linux/backlight.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kthread.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/unaligned.h>
#include <linux/vmalloc.h>
#include <linux/wait.h>

/* host bus block (phys 0x20025800) */
#define VC_CTRL	0x10
#define VC_STAT	0x14 /* bit 8: bus cycle busy, bit 0: reset busy */
#define VC_DMACTL 0x40
#define VC_DMACNT 0x44 /* halfwords the gfx dma feeds in */
#define VC_CHAN	0x4c /* channel word for the next transfer */
#define VC_WR 0x50 /* one 16-bit write */
#define VC_RD 0x58 /* write 0 starts a 16-bit read, read gets it */
#define VC_MODE	0x60
#define VC_TIM0	0x64
#define VC_TIM1	0x68
#define VC_6C 0x6c
#define VC_90 0x90

#define VC_MODE_DATA 0x100023
#define VC_MODE_STATUS 0x110023

/* status word, see vc_status() */
#define VC_ST_DATA BIT(29)
#define VC_ST_LEN GENMASK(23, 0)

/* cycle timings for 597.33 MHz, FUN_8032fa3c: boot ROM pair, normal pair */
#define VC_TIM_BOOT0 0x08008080
#define VC_TIM_BOOT1 0x023c03c3
#define VC_TIM_NORM0 0x05008050
#define VC_TIM_NORM1 0x017c01c3

#define VC_BOOT_MAGIC0 0xafe45251
#define VC_BOOT_MAGIC1 0xf001bc4e

/* VCHI */
#define FOURCC(a, b, c, d) \
	((u32)(a) | (u32)(b) << 8 | (u32)(c) << 16 | (u32)(d) << 24)
#define FCC_CTRL FOURCC('C', 'T', 'R', 'L')
#define FCC_BULX FOURCC('B', 'U', 'L', 'X')
#define FCC_PAD FOURCC('_', '_', '_', '_')
#define FCC_DISP FOURCC('D', 'I', 'S', 'P')
#define FCC_UPDH FOURCC('U', 'P', 'D', 'H')
#define FCC_DSPC FOURCC('D', 'S', 'P', 'C')
#define FCC_BFMX FOURCC('B', 'F', 'M', 'X')
/* khronos (EGL/GLES/OpenVG) services, registered by vc-main next to DSPC
 * and DISP (stored as NRHK/NHHK/NAHK), the same names as the open
 * raspberry pi khrn client (raspberrypi/userland interface/khronos) */
#define FCC_KHRN FOURCC('K', 'H', 'R', 'N')
#define FCC_KHHN FOURCC('K', 'H', 'H', 'N')
#define FCC_KHAN FOURCC('K', 'H', 'A', 'N')
#define FCC_EGLD FOURCC('E', 'G', 'L', 'D')	/* VC side of khegl_ldd, semaphore signals */

#define VCHI_MAGIC 0xaf05
#define VCHI_PAD 0x50fa
#define VCHI_SLOT 0x1020		/* host tx slot, as the VC asks for */
#define VCHI_SLOT_RX 0x1010		/* host rx slot we announce */
#define VCHI_HOST_SLOTS 15
#define VCHI_MAX_MSG 0x800
#define VCHI_BULK_PIECE 0x18000		/* vchi +0x357c */

#define CTRL_CONNECT 0
#define CTRL_QUERY 1
#define CTRL_BULK_REQ 2
#define CTRL_REPLY 5

/* panel / display */
/*
 * gfx dma engine (MeSSI side, phys 0x20024800, irq 0x92), vcdriver.dll
 * FUN_8032a154 (init), FUN_80329c8c (self test), FUN_80329ec8 (one piece),
 * FUN_8032a020 (irq: status +0x18, write the bits back to ack)
 */
#define G_CTRL	0x10	/* 2 = reset, 0x2015 = run */
#define G_STAT	0x14	/* bit 0: reset busy */
#define G_IRQ	0x18	/* bit 0: piece done, write back to ack */
#define G_1C	0x1c
#define G_GO	0x40	/* 0x8908 | 0x20, then bit 0 = go */
#define G_44	0x44
#define G_4C	0x4c
#define G_CNT	0x5c	/* moves while the engine runs */
#define G_60	0x60
#define G_64	0x64
#define G_MODE	0x68	/* 0x700000 = self test */
#define G_6C	0x6c
#define G_SIZE	0x7c	/* (w - 1) << 16 | (h - 1), w in halfwords */
#define G_ADDR	0x80	/* source phys */
#define G_88	0x88
#define G_SIZE2	0x8c
#define G_W	0x80	/* halfwords per line, Symbian always uses 0x80 */
#define G_LINE	(G_W * 2)

#define VC_W 640
#define VC_H 480
/* 4 bytes a pixel (XRGB8888) or 2 with videocore.rgb565=1 */
#define VC_PITCH (vc->pitch)
#define VC_FB_SIZE (VC_PITCH * VC_H)

static inline u16 vc_565(u32 p)
{
	return (p >> 8 & 0xf800) | (p >> 5 & 0x07e0) | (p >> 3 & 0x001f);
}

#define DSPC_ID	4		/* display chipset interface, E6 = 4 */
#define DSPC_ALL 0xffff

#define DISP_REPLY 0x80000000
#define DISP_RES_CREATE	0x03
#define DISP_OPEN 0x08
#define DISP_SET_BG	0x0d
#define DISP_UPDATE_START 0x10
#define DISP_SUBMIT_SYNC 0x12
#define DISP_ELEMENT_ADD 0x13
#define DISP_ELEMENT_MOD 0x14
#define DISP_ELEMENT_ATTR 0x17	/* ElementChangeAttributes, no reply */
#define ELEM_CHANGE_DEST BIT(2)
#define ELEM_CHANGE_SRC BIT(3)

#define VC_IMG_RES_TYPE	0x2b		/* FUN_80345920(4), XRGB8888 */
#define VC_IMG_BUF_TYPE	0x20008		/* FUN_80345974(4) */
#define VC_IMG_RES_565	1		/* FUN_80345920(0x20), RGB565 */
#define VC_IMG_BUF_565	0x20004		/* FUN_80345974(0x20), 16 bpp */

#define BUFM_WRITE_IMAGE 6

static unsigned int brightness = 0x3f;
module_param(brightness, uint, 0644);
MODULE_PARM_DESC(brightness, "panel DCS 0x51 value (Symbian boots with 0x3f)");

static bool loader_b;
module_param(loader_b, bool, 0444);
MODULE_PARM_DESC(loader_b, "boot with loader B (Symbian's power-up path) instead of A");

static bool matrix;
module_param(matrix, bool, 0444);
MODULE_PARM_DESC(matrix, "send an identity colour matrix instead of Symbian's all zero one");

static bool test_pattern = true;
module_param(test_pattern, bool, 0444);
MODULE_PARM_DESC(test_pattern, "show colour bars for 3 s once the display is up");

/* we boot the VC cold, so the panel needs Symbian's cold power-up path */
static int cold = 1;
module_param(cold, int, 0444);
MODULE_PARM_DESC(cold, "panel power-up path: 1 = cold (default), 0 = warm, -1 = by DCS 0x0a like Symbian");

static bool dma;
module_param(dma, bool, 0444);
MODULE_PARM_DESC(dma, "push frames with the gfx dma engine instead of PIO");

static bool khrn_probe;
module_param(khrn_probe, bool, 0444);
MODULE_PARM_DESC(khrn_probe, "after bring-up, try to open the VC khronos services (KHRN/KHHN/KHAN) and log the answer");

static bool disp_probe;
module_param(disp_probe, bool, 0444);
MODULE_PARM_DESC(disp_probe, "after bring-up, move the screen element down 2 s, scroll it up 2 s (dispman ElementChangeAttributes test)");

static bool khrn_draw;
module_param(khrn_draw, bool, 0444);
MODULE_PARM_DESC(khrn_draw, "the VC GL server clears a 320x240 window (magenta), shown in the middle for 10 s");

static bool rgb565;
module_param(rgb565, bool, 0444);
MODULE_PARM_DESC(rgb565, "push 16 bit RGB565 frames (half the bytes on the slow bus)");

static bool partial;
module_param(partial, bool, 0644);
MODULE_PARM_DESC(partial, "push only the changed rows, BROKEN on the phone (the VC ignores the write rects)");

static unsigned int debug = 1;
module_param(debug, uint, 0644);
MODULE_PARM_DESC(debug, "log VCHI messages: 1 = bring-up and first frame, 2 = all");

/* CTRL type 2 from the VC: {2, fourcc, size, u8 channel, ?, rx size, offset} */
struct vc_bulk_req {
	u32 fourcc;
	u32 size;
	u32 rx_size;
	u32 offset;
	bool valid;
};

struct vc_khrn_req;

struct vc {
	struct device *dev;
	void __iomem *bus;
	struct gpio_desc *run;
	struct clk *clk;

	/* gfx dma (videocore.dma=1): push is then physically contiguous */
	void __iomem *gfx, *pll, *gdiv;
	bool gfx_ok;	/* init passed */
	int dma_log;	/* log the steps of the first dma piece */
	bool dma_ok;	/* frames go by dma (after the PIO bring-up) */
	dma_addr_t push_phys;

	/* frame stats, logged every 10 s */
	unsigned int st_frames, st_rows;
	u64 st_ns, st_max;
	unsigned long st_next;
	struct task_struct *thread;

	u32 last_status;
	bool rx_armed;

	/* VCHI */
	u8 rx[0x1100];
	u8 tx[VCHI_SLOT] __aligned(64);	/* gfx dma reads it */
	u8 bulk_msg[VCHI_MAX_MSG];
	u32 tx_cur;
	u32 credits_owed; /* rx slots we freed, returned in the next tx */
	int tx_credits;	/* VC rx slots we may still fill */
	bool vc_connected;
	bool host_connected;
	u32 pending_query[8];
	unsigned int npending;
	u32 svc_reply_fourcc, svc_reply_flags;
	bool svc_reply_seen;
	u32 disp_reply;
	bool disp_reply_seen;
	u8 bufm_reply[0x40];
	/* khronos replies (khrn probe) */
	u8 khrn_rx[0x200];
	u32 khrn_rx_len, khrn_rx_fcc;
	bool khrn_rx_seen;
	bool bufm_reply_seen;
	struct vc_bulk_req bulk_req[8];
	u32 dspc_token;
	u8 dspc_rx[0x40];
	u32 dspc_rx_len;
	bool dspc_rx_seen;

	/* display */
	u32 display, resource, element;
	bool ready;
	bool first_refresh;
	bool te_on;
	unsigned int frames;

	u32 pitch;	/* VC_W * 2 or 4 */
	bool push_ready;	/* push already holds the frame (test pattern) */
	unsigned int partial_log;
	u8 row[VC_W * 4];	/* one converted row for the diff */

	/* drm: the plane is copied into fb, the thread pushes fb */
	struct drm_device drm;
	struct drm_simple_display_pipe pipe;
	struct drm_connector conn;
	void *fb;
	/* the push reads a snapshot, a commit mid transfer would mix two frames.
	 * fb_lock covers fb and the snapshot copy */
	void *push;
	struct mutex fb_lock;
	bool dirty;
	/* rows changed since the last push, [band_y0, band_y1), empty = full */
	spinlock_t band_lock;
	u32 band_y0, band_y1;

	/* backlight class: the thread sends DCS 0x51 when bl_pending is set */
	struct backlight_device *bl;
	bool bl_pending;
	wait_queue_head_t wq;

	/* /dev/vc-khrn: one request at a time, served by the thread */
	struct miscdevice misc;
	struct mutex khrn_lock;
	struct vc_khrn_req *kreq;
	bool khrn_open;
};

static inline u32 vc_rd(struct vc *vc, u32 off)
{
	return readl(vc->bus + off);
}

static inline void vc_wr(struct vc *vc, u32 off, u32 v)
{
	writel(v, vc->bus + off);
}

static int vc_idle(struct vc *vc)
{
	unsigned int i;

	for (i = 0; i < 2000000; i++)
		if (!(vc_rd(vc, VC_STAT) & 0x100))
			return 0;
	dev_err_ratelimited(vc->dev, "host bus stuck busy\n");
	return -EIO;
}

static int vc_chan(struct vc *vc, u32 w)
{
	int ret = vc_idle(vc);

	if (!ret)
		vc_wr(vc, VC_CHAN, w);
	return ret;
}

static int vc_wr16(struct vc *vc, u32 v)
{
	int ret = vc_idle(vc);

	if (!ret)
		vc_wr(vc, VC_WR, v & 0xffff);
	return ret;
}

static int vc_rd16(struct vc *vc, u32 *v)
{
	int ret = vc_idle(vc);

	if (ret)
		return ret;
	vc_wr(vc, VC_RD, 0);
	ret = vc_idle(vc);
	if (!ret)
		*v = vc_rd(vc, VC_RD) & 0xffff;
	return ret;
}

/*
 * status word (FUN_8032bb18): two halfword reads in status mode, a | b << 15,
 * with the firmware's resync rules. bit 29 = data waiting, low 24 bits =
 * length - 1.
 */
static int vc_status(struct vc *vc, u32 *st)
{
	u32 a, b;
	int ret;

	vc_wr(vc, VC_MODE, VC_MODE_STATUS);
	ret = vc_rd16(vc, &a);
	if (!ret)
		ret = vc_rd16(vc, &b);
	if (!ret && (a & 0x8000)) {
		a = b;
		ret = vc_rd16(vc, &b);
	}
	if (!ret && (b & 0x4000) && !(vc->last_status & VC_ST_DATA)) {
		ret = vc_rd16(vc, &a);
		if (!ret)
			ret = vc_rd16(vc, &b);
	}
	if (!ret)
		*st = vc->last_status = a | b << 15;
	vc_wr(vc, VC_MODE, VC_MODE_DATA);
	return ret;
}

/*
 * one transfer: channel word, halfwords, then channel word 0. The firmware's
 * completion DFC (FUN_80329a48 -> FUN_8032a98c) writes the 0 after every
 * transfer, that is what ends it for the VC.
 */
static int vc_tx(struct vc *vc, const void *buf, u32 len, u32 chan)
{
	const u8 *p = buf;
	u32 i;
	int ret;

	ret = vc_chan(vc, chan);
	for (i = 0; !ret && i < len; i += 2)
		ret = vc_wr16(vc, p[i] | (i + 1 < len ? p[i + 1] << 8 : 0));
	if (!ret)
		ret = vc_chan(vc, 0);
	return ret;
}

/*
 * one bulk piece through the gfx dma: channel word, the 256-byte lines by
 * dma, a short tail by PIO, channel word 0 (FUN_80329ec8, FUN_8032a020,
 * FUN_8032a98c). a timeout turns dma off for good (that frame is lost)
 */
static int vc_tx_dma(struct vc *vc, const u8 *p, dma_addr_t phys, u32 len,
		     u32 chan)
{
	u32 lines = len / G_LINE, i, st;
	int ret;

	ret = vc_chan(vc, chan);
	if (ret)
		return ret;
	if (lines) {
		if (vc->dma_log)
			dev_info(vc->dev, "dma 1st piece: addr %pad, %u lines\n",
				 &phys, lines);
		writel(phys, vc->gfx + G_ADDR);
		writel((G_W - 1) << 16 | (lines - 1), vc->gfx + G_SIZE);
		writel((G_W - 1) << 16 | (lines - 1), vc->gfx + G_SIZE2);
		writel(0, vc->gfx + G_6C);
		writel(0x8908, vc->gfx + G_GO);
		writel(0x8908 | 0x20, vc->gfx + G_GO);
		if (vc->dma_log)
			dev_info(vc->dev, "dma 1st piece: go\n");
		writel(0x8908 | 0x21, vc->gfx + G_GO);
		if (vc->dma_log)
			dev_info(vc->dev, "dma 1st piece: vc count + enable\n");
		vc_wr(vc, VC_DMACNT, lines * G_W);
		vc_wr(vc, VC_DMACTL, vc_rd(vc, VC_DMACTL) | 0x11);
		if (vc->dma_log)
			dev_info(vc->dev, "dma 1st piece: wait\n");
		/* no sleep: it would be a whole jiffy (periodic timer) */
		ret = readl_poll_timeout_atomic(vc->gfx + G_IRQ, st, st & 1, 1,
						100000);
		if (vc->dma_log) {
			dev_info(vc->dev, "dma 1st piece: %s, irq %08x\n",
				 ret ? "timeout" : "done", st);
			vc->dma_log = 0;
		}
		writel(readl(vc->gfx + G_IRQ) & 0x1ffff, vc->gfx + G_IRQ);
		if (!ret)
			ret = vc_idle(vc);
		vc_wr(vc, VC_DMACTL, vc_rd(vc, VC_DMACTL) & ~0x11);
		if (ret) {
			dev_err(vc->dev, "gfx dma piece timed out, PIO from now on\n");
			vc->dma_ok = false;
			return ret;
		}
	}
	for (i = lines * G_LINE; !ret && i < len; i += 2)
		ret = vc_wr16(vc, p[i] | (i + 1 < len ? p[i + 1] << 8 : 0));
	if (!ret)
		ret = vc_chan(vc, 0);
	return ret;
}

/*
 * FUN_8032a118 (divider: gfx clock <= 173 MHz from the pll), FUN_8032f904
 * (reset), FUN_8032a154 (regs), FUN_80329c8c + FUN_80329c10 (self test:
 * 8 halfwords from memory in test mode, must end with +0x18 bit 0)
 */
static int vc_gfx_init(struct vc *vc)
{
	u32 r, f, d, t, st;
	int ret;

	/* a step log each, the ramcon keeps the last one if a reg kills us */
	dev_info(vc->dev, "gfx dma init: pll\n");
	r = readl(vc->pll);
	f = ((r >> 5) & 0x7ff) * 38400 / ((r & 0x1f) + 1);	/* kHz */
	d = max(1u, (f + 173000 - 1) / 173000);
	dev_info(vc->dev, "gfx dma init: pll %08x = %u kHz, divider %u\n", r, f, d);
	writel((readl(vc->gdiv) & ~0x30) | ((d - 1) & 3) << 4, vc->gdiv);

	dev_info(vc->dev, "gfx dma init: reset\n");
	writel(2, vc->gfx + G_CTRL);
	ret = readl_poll_timeout(vc->gfx + G_STAT, st, !(st & 1), 10, 10000);
	writel(0, vc->gfx + G_CTRL);
	if (ret)
		return ret;

	dev_info(vc->dev, "gfx dma init: regs\n");
	writel(0x2015, vc->gfx + G_CTRL);
	writel(0x4005, vc->gfx + G_44);
	writel(0x18108, vc->gfx + G_GO);
	writel(0x200000, vc->gfx + G_64);
	writel(0x900000, vc->gfx + G_MODE);
	writel(0x4000, vc->gfx + G_6C);
	writel(0, vc->gfx + G_88);
	writel(0x7ff, vc->gfx + G_60);
	writel(0xffffffff, vc->gfx + G_IRQ);
	writel(0x5641, vc->gfx + G_1C);
	writel(0, vc->gfx + G_4C);
	writel(1, vc->gfx + 0xac);
	writel(1, vc->gfx + 0xb0);
	writel(1, vc->gfx + 0xb4);
	t = readl(vc->gfx + 0xa8) + readl(vc->gfx + 0xd4) + readl(vc->gfx + 0x164);
	writel((t - 0x100) | (t - 1) << 16, vc->gfx + 0xa4);
	writel(t - 0x100, vc->gfx + 0x22c);
	writel(0x8d, vc->gfx + 0xa0);
	writel(vc->push_phys, vc->gfx + G_ADDR);

	/* no self test: the test mode run killed the phone */
	dev_info(vc->dev, "gfx dma: pll %u kHz, div %u, init done\n", f, d);
	vc->dma_log = 1;
	return ret;
}

static void vc_timings(struct vc *vc, bool boot)
{
	if (vc_idle(vc))
		return;
	vc_wr(vc, VC_TIM0, boot ? VC_TIM_BOOT0 : VC_TIM_NORM0);
	vc_wr(vc, VC_TIM1, boot ? VC_TIM_BOOT1 : VC_TIM_NORM1);
}

/* FUN_8032fb08 + FUN_8032a368, only if nobody (Symbian) left it set up */
static int vc_bus_init(struct vc *vc)
{
	unsigned int i;

	if (vc_rd(vc, VC_MODE) != VC_MODE_DATA) {
		vc_wr(vc, VC_CTRL, 0x0a);
		for (i = 0; i < 1000000 && (vc_rd(vc, VC_STAT) & 1); i++)
			cpu_relax();
		vc_wr(vc, VC_CTRL, 0x11);
		vc_wr(vc, VC_MODE, VC_MODE_DATA);
		vc_wr(vc, VC_6C, 0x10);
		vc_wr(vc, VC_90, 0x6d60);
		vc_wr(vc, VC_DMACTL, 0x64);
		if (vc_idle(vc))
			return -EIO;
		vc_wr(vc, VC_DMACTL, vc_rd(vc, VC_DMACTL) & ~0x11);
	}
	return vc_chan(vc, 0);
}

/*
 * one message from the VC: status until bit 29, then (len + 1) bytes as
 * halfwords, max 0x800 per round, and two status reads after the last one.
 */
static int vc_rx(struct vc *vc, u8 *buf, u32 max, u32 *len,
		 unsigned int timeout_ms)
{
	unsigned long end = jiffies + msecs_to_jiffies(timeout_ms);
	u32 st, got = 0, v, n, c, i;
	bool first = true;
	unsigned int spins = 0;
	int ret;

	if (!vc->rx_armed) {
		ret = vc_chan(vc, 0);
		if (ret)
			return ret;
		vc->rx_armed = true;
	}
	/* the firmware drops one status read at the start */
	ret = vc_status(vc, &st);
	if (ret)
		return ret;
	for (;;) {
		for (;;) {
			ret = vc_status(vc, &st);
			if (ret)
				return ret;
			if (st & VC_ST_DATA)
				break;
			/* timeout 0 = one look (the thread loop polls) */
			if (!timeout_ms && first)
				return -EAGAIN;
			if (time_after(jiffies, end))
				return first ? -EAGAIN : -ETIMEDOUT;
			/* periodic timer only: any sleep is a jiffy, so spin ~2 ms first */
			if (spins++ < 200)
				udelay(10);
			else
				usleep_range(200, 500);
		}
		n = (st & VC_ST_LEN) + 1;
		c = min(n, 0x800u);
		if (got + c > max)
			return -EOVERFLOW;
		for (i = 0; i < c; i += 2) {
			ret = vc_rd16(vc, &v);
			if (ret)
				return ret;
			buf[got + i] = v;
			if (i + 1 < c)
				buf[got + i + 1] = v >> 8;
		}
		got += c;
		first = false;
		if (c == n)
			break;
		/* a message is coming: the rest gets the normal wait */
		end = jiffies + msecs_to_jiffies(timeout_ms ? timeout_ms : 20);
	}
	vc_status(vc, &st);
	vc_status(vc, &st);
	*len = got;
	return 0;
}

/* ---- boot (DVCDriver::Boot) ---- */

static int vc_rx_boot(struct vc *vc, u32 *type, u32 *value)
{
	__le32 *w = (__le32 *)vc->rx;
	u32 n;
	int ret;

	ret = vc_rx(vc, vc->rx, sizeof(vc->rx), &n, 2000);
	if (ret)
		return ret;
	if (n != 32 || le32_to_cpu(w[0]) != VC_BOOT_MAGIC0 ||
	    le32_to_cpu(w[7]) != VC_BOOT_MAGIC1) {
		dev_err(vc->dev, "unexpected boot message (%u bytes)\n", n);
		return -EPROTO;
	}
	*type = le32_to_cpu(w[1]) >> 24;
	*value = le32_to_cpu(w[1]) & 0xffffff;
	return 0;
}

static int vc_send_image(struct vc *vc, const char *name)
{
	const struct firmware *img, *trl;
	char path[48];
	__le32 hdr[8];
	u8 *buf;
	u32 len, i;
	int ret;

	snprintf(path, sizeof(path), "vc/%s.bin", name);
	ret = request_firmware(&img, path, vc->dev);
	if (ret)
		return ret;
	snprintf(path, sizeof(path), "vc/%s.trl", name);
	ret = request_firmware(&trl, path, vc->dev);
	if (ret)
		goto out_img;
	if (trl->size != 20) {
		ret = -EINVAL;
		goto out;
	}
	len = ALIGN(img->size, 16);
	buf = vzalloc(len);
	if (!buf) {
		ret = -ENOMEM;
		goto out;
	}
	memcpy(buf, img->data, img->size);

	hdr[0] = cpu_to_le32(VC_BOOT_MAGIC0);
	hdr[1] = cpu_to_le32(0x02000000 | (img->size & 0xffffff));
	memcpy(&hdr[2], trl->data, 20);
	hdr[7] = cpu_to_le32(VC_BOOT_MAGIC1);

	/* image on tx channel 1 (word 3), then the header on tx 0 (word 2) */
	ret = vc_tx(vc, buf, len, 3);
	if (!ret)
		ret = vc_tx(vc, hdr, sizeof(hdr), 2);
	vfree(buf);
	for (i = 0; i < 5; i++)
		msleep(20);
out:
	release_firmware(trl);
out_img:
	release_firmware(img);
	return ret;
}

static int vc_boot(struct vc *vc)
{
	u32 type, value;
	int ret;

	ret = vc_bus_init(vc);
	if (ret)
		return ret;
	/* gfx dma init where Symbian does it: after the bus init, VC in reset */
	if (dma) {
		gpiod_set_value_cansleep(vc->run, 0);
		vc->gfx_ok = !vc_gfx_init(vc);
	}
	vc_timings(vc, true);
	vc->rx_armed = false;
	vc->last_status = 0;

	gpiod_set_value_cansleep(vc->run, 0);
	msleep(50);
	gpiod_set_value_cansleep(vc->run, 1);

	ret = vc_rx_boot(vc, &type, &value);
	if (ret)
		return ret;
	if (type != 1)
		return -EPROTO;

	/* loader A acks (3/1) and then says hello (1), loader B says hello at once */
	ret = vc_send_image(vc, loader_b ? "vc-loader-b" : "vc-loader-a");
	if (ret)
		return ret;
	ret = vc_rx_boot(vc, &type, &value);
	if (ret)
		return ret;
	dev_info(vc->dev, "loader answered %u/%u\n", type, value);
	if (type == 3 && value == 1)
		ret = vc_rx_boot(vc, &type, &value);
	if (ret || type != 1)
		return ret ?: -EPROTO;

	ret = vc_send_image(vc, "vc-main");
	if (ret)
		return ret;
	ret = vc_rx_boot(vc, &type, &value);
	if (ret || type != 3 || value != 1)
		return ret ?: -EPROTO;

	vc_timings(vc, false);
	dev_info(vc->dev, "VideoCore booted\n");
	return 0;
}

static void vchi_slot_begin(struct vc *vc)
{
	vc->tx_cur = 0;
}

/*
 * one entry: u16 0xaf05 | u16 entry size | u32 fourcc | u32 timestamp |
 * u16 credit | u16 payload size | payload, 16-byte aligned
 */
static int vchi_slot_msg(struct vc *vc, u32 fourcc, const void *data, u32 n)
{
	u32 ent = ALIGN(n + 16, 16);
	u8 *h = vc->tx + vc->tx_cur;

	if (vc->tx_cur + ent > VCHI_SLOT - 16 || n > VCHI_MAX_MSG)
		return -E2BIG;
	memset(h, 0, ent);
	put_unaligned_le16(VCHI_MAGIC, h);
	put_unaligned_le16(ent, h + 2);
	put_unaligned_le32(fourcc, h + 4);
	put_unaligned_le16(vc->credits_owed, h + 12);
	vc->credits_owed = 0;
	put_unaligned_le16(n, h + 14);
	if (n)
		memcpy(h + 16, data, n);
	vc->tx_cur += ent;
	return 0;
}

static int vchi_pump(struct vc *vc, unsigned int timeout_ms, bool *cond);

/* close the slot like FUN_80331994: pad to 0x1010, then a 16-byte end entry */
static int vchi_slot_send(struct vc *vc)
{
	u8 *h;
	int ret;

	while (vc->tx_credits <= 0) {
		ret = vchi_pump(vc, 1000, NULL);
		if (ret)
			return ret;
		if (vc->tx_credits <= 0) {
			dev_warn_ratelimited(vc->dev, "no VCHI tx credits\n");
			return -EBUSY;
		}
	}
	if (vc->tx_cur < VCHI_SLOT - 16) {
		h = vc->tx + vc->tx_cur;
		memset(h, 0, VCHI_SLOT - 16 - vc->tx_cur);
		put_unaligned_le16(VCHI_PAD, h);
		put_unaligned_le16(VCHI_SLOT - 16 - vc->tx_cur, h + 2);
		put_unaligned_le32(FCC_PAD, h + 4);
	}
	h = vc->tx + VCHI_SLOT - 16;
	memset(h, 0, 16);
	put_unaligned_le16(VCHI_MAGIC, h);
	put_unaligned_le16(16, h + 2);
	put_unaligned_le32(FCC_PAD, h + 4);
	vc->tx_credits--;
	/* Symbian: PIO only up to 48 bytes, everything longer by dma */
	if (vc->dma_ok) {
		dma_addr_t a = dma_map_single(vc->dev, vc->tx, VCHI_SLOT,
					      DMA_TO_DEVICE);

		if (!dma_mapping_error(vc->dev, a)) {
			ret = vc_tx_dma(vc, vc->tx, a, VCHI_SLOT, 0);
			dma_unmap_single(vc->dev, a, VCHI_SLOT, DMA_TO_DEVICE);
			return ret;
		}
	}
	return vc_tx(vc, vc->tx, VCHI_SLOT, 0);
}

static void vchi_log(struct vc *vc, const char *dir, u32 fourcc,
		     const void *data, u32 n)
{
	if (debug > 1 || (debug && vc->frames < 2))
		dev_info(vc->dev, "%s %4.4s %3u: %*ph\n", dir, (char *)&fourcc,
			 n, min_t(u32, n, 32), data);
}

static int vchi_send(struct vc *vc, u32 fourcc, const void *data, u32 n)
{
	int ret;

	vchi_log(vc, "tx", fourcc, data, n);
	vchi_slot_begin(vc);
	ret = vchi_slot_msg(vc, fourcc, data, n);
	if (!ret)
		ret = vchi_slot_send(vc);
	return ret;
}

static void vchi_ctrl_msg(struct vc *vc, const u8 *pl, u32 size)
{
	u32 type = get_unaligned_le32(pl);
	unsigned int i;

	switch (type) {
	case CTRL_CONNECT:
		if (size >= 16) {
			vc->vc_connected = true;
			vc->tx_credits = get_unaligned_le32(pl + 12);
		}
		break;
	case CTRL_QUERY:
		/* the VC asks for a host service: we have none */
		if (size >= 8 && vc->npending < ARRAY_SIZE(vc->pending_query))
			vc->pending_query[vc->npending++] =
				get_unaligned_le32(pl + 4);
		break;
	case CTRL_BULK_REQ:
		/* {2, fourcc, size, channel byte, ...}: the VC wants data */
		if (size >= 12) {
			for (i = 0; i < ARRAY_SIZE(vc->bulk_req); i++)
				if (!vc->bulk_req[i].valid)
					break;
			if (i == ARRAY_SIZE(vc->bulk_req)) {
				dev_warn(vc->dev, "bulk request queue full\n");
				break;
			}
			vc->bulk_req[i].fourcc = get_unaligned_le32(pl + 4);
			vc->bulk_req[i].size = get_unaligned_le32(pl + 8);
			vc->bulk_req[i].rx_size = vc->bulk_req[i].size;
			vc->bulk_req[i].offset = 0;
			if (size >= 0x1c) {
				vc->bulk_req[i].rx_size = get_unaligned_le32(pl + 0x14);
				vc->bulk_req[i].offset = get_unaligned_le32(pl + 0x18);
			}
			vc->bulk_req[i].valid = true;
		}
		break;
	case CTRL_REPLY:
		if (size >= 12) {
			vc->svc_reply_fourcc = get_unaligned_le32(pl + 4);
			vc->svc_reply_flags = get_unaligned_le32(pl + 8);
			vc->svc_reply_seen = true;
		}
		break;
	}
}

static void vchi_handle_slot(struct vc *vc, u32 n)
{
	u32 off = 0;

	vc->credits_owed++;
	while (off + 16 <= n) {
		const u8 *h = vc->rx + off;
		u32 magic = get_unaligned_le16(h);
		u32 ent = get_unaligned_le16(h + 2);
		u32 fcc = get_unaligned_le32(h + 4);
		u32 size = get_unaligned_le16(h + 14);
		const u8 *pl = h + 16;

		if (ent < 16 || off + ent > n || size > ent - 16)
			return;
		if (magic == VCHI_MAGIC) {
			vc->tx_credits += get_unaligned_le16(h + 12);
			if (fcc != FCC_PAD && (size || fcc != FCC_CTRL))
				vchi_log(vc, "rx", fcc, pl, size);
			if (fcc == FCC_CTRL && size >= 4) {
				vchi_ctrl_msg(vc, pl, size);
			} else if (fcc == FCC_DISP && size == 4) {
				vc->disp_reply = get_unaligned_le32(pl);
				vc->disp_reply_seen = true;
			} else if (fcc == FCC_BFMX) {
				memset(vc->bufm_reply, 0, sizeof(vc->bufm_reply));
				memcpy(vc->bufm_reply, pl,
				       min_t(u32, size, sizeof(vc->bufm_reply)));
				vc->bufm_reply_seen = true;
			} else if (fcc == FCC_KHRN || fcc == FCC_KHHN ||
				   fcc == FCC_KHAN) {
				vc->khrn_rx_len = min_t(u32, size,
							sizeof(vc->khrn_rx));
				memcpy(vc->khrn_rx, pl, vc->khrn_rx_len);
				vc->khrn_rx_fcc = fcc;
				vc->khrn_rx_seen = true;
			} else if (fcc == FCC_DSPC) {
				/* {token, status[, len, data]} */
				vc->dspc_rx_len = min_t(u32, size,
							sizeof(vc->dspc_rx));
				memcpy(vc->dspc_rx, pl, vc->dspc_rx_len);
				vc->dspc_rx_seen = true;
			}
			/* UPDH: ignored */
		}
		off += ent;
	}
}

/* answer "not here" to the VC's service queries, once we are connected */
static void vchi_answer_queries(struct vc *vc)
{
	while (vc->host_connected && vc->npending) {
		u32 r[3] = { CTRL_REPLY, vc->pending_query[--vc->npending], 0 };

		vchi_send(vc, FCC_CTRL, r, sizeof(r));
	}
}

/*
 * read and handle slots until *cond or the timeout. Nothing is sent while a
 * slot is being parsed: sending may have to read (credits) into vc->rx.
 */
static int vchi_pump(struct vc *vc, unsigned int timeout_ms, bool *cond)
{
	unsigned long end = jiffies + msecs_to_jiffies(timeout_ms);
	u32 n;
	int ret;

	while (!(cond && *cond)) {
		/* timeout 0 (the thread loop): drain what is there and go */
		ret = vc_rx(vc, vc->rx, sizeof(vc->rx), &n, timeout_ms ? 20 : 0);
		if (!ret) {
			vchi_handle_slot(vc, n);
			vchi_answer_queries(vc);
			if (!timeout_ms)
				continue;
		} else if (ret != -EAGAIN) {
			return ret;
		}
		if (!timeout_ms || time_after(jiffies, end))
			return cond ? -ETIMEDOUT : 0;
	}
	return 0;
}

/* hand rx slots back when the VC keeps talking and we don't */
static void vchi_return_credits(struct vc *vc)
{
	if (vc->host_connected && vc->credits_owed >= VCHI_HOST_SLOTS / 2)
		vchi_send(vc, FCC_CTRL, NULL, 0);
}

static int vchi_connect(struct vc *vc)
{
	u32 hello[5] = { CTRL_CONNECT, 1, VCHI_SLOT_RX, VCHI_HOST_SLOTS,
			 VCHI_MAX_MSG };
	int ret;

	ret = vchi_pump(vc, 5000, &vc->vc_connected);
	if (ret)
		return ret;
	if (vc->tx_credits <= 0)
		vc->tx_credits = 1;
	ret = vchi_send(vc, FCC_CTRL, hello, sizeof(hello));
	if (ret)
		return ret;
	vc->host_connected = true;
	vchi_answer_queries(vc);
	return 0;
}

/*
 * client open (FUN_80336d14): {1, fourcc, flags}, flags = 0x10 | 8 (service
 * has a callback) | 4 (callback param) | 2 (crc). The VC answers {5, fourcc,
 * flags}: bit 0 available, bit 1 crc, bit 2/3 unaligned bulk rx/tx.
 */
static int vchi_open_flags(struct vc *vc, u32 fourcc, u32 flags)
{
	u32 req[3] = { CTRL_QUERY, fourcc, flags };
	int ret;

	vc->svc_reply_seen = false;
	ret = vchi_send(vc, FCC_CTRL, req, sizeof(req));
	if (!ret)
		ret = vchi_pump(vc, 3000, &vc->svc_reply_seen);
	if (ret)
		return ret;
	if (!(vc->svc_reply_flags & 1)) {
		dev_err(vc->dev, "service %4.4s not available\n", (char *)&fourcc);
		return -ENODEV;
	}
	if (vc->svc_reply_flags & 2)
		dev_warn(vc->dev, "%4.4s: the VC wants crcs, not implemented\n",
			 (char *)&fourcc);
	return 0;
}

static int vchi_open(struct vc *vc, u32 fourcc)
{
	return vchi_open_flags(vc, fourcc, 0x10);
}

static struct vc_bulk_req *vchi_find_bulk_req(struct vc *vc, u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(vc->bulk_req); i++)
		if (vc->bulk_req[i].valid && vc->bulk_req[i].fourcc == fourcc)
			return &vc->bulk_req[i];
	return NULL;
}

/*
 * bulk transmit (FUN_803375f0 -> FUN_803338ec / FUN_803337f8 / FUN_80333678 /
 * FUN_803334bc / FUN_803331cc), for a service that allows unaligned bulk
 * (BFMX: callback param set -> service +0x6c). One BULX message carries a
 * 0x1c-byte header {fourcc, size, piece size, crc, rx size, u16 head
 * offset, u8 has rx, u8 0, u16 head bytes, u16 tail bytes} plus the
 * unaligned head/tail inline:
 * below the VC's max message (0x800) it all goes inline, no request
 * otherwise the VC first asks with CTRL type 2; the 32-byte aligned part
 * goes on bulk channel 1 in pieces of 0x18000: channel word
 * (flags | 1) & 7 = 1, the last piece 3 (flag 2), each ended with word 0
 */
static int vchi_bulk_tx(struct vc *vc, u32 fourcc, const u8 *data, u32 size)
{
	struct vc_bulk_req *req;
	unsigned long end = jiffies + msecs_to_jiffies(2000);
	u8 *msg = vc->bulk_msg;
	u32 rx_size, tail, n, done, piece;
	int ret;

	if (0x1c + ALIGN(size, 4) <= VCHI_MAX_MSG) {
		memset(msg, 0, 0x1c);
		put_unaligned_le32(fourcc, msg);
		put_unaligned_le32(size, msg + 4);
		put_unaligned_le32(VCHI_BULK_PIECE, msg + 8);
		put_unaligned_le16(size, msg + 24);
		memcpy(msg + 0x1c, data, size);
		n = 0x1c + ALIGN(size, 4);
		memset(msg + 0x1c + size, 0, n - 0x1c - size);
		return vchi_send(vc, FCC_BULX, msg, n);
	}

	while (!(req = vchi_find_bulk_req(vc, fourcc))) {
		ret = vchi_pump(vc, 20, NULL);
		if (ret)
			return ret;
		if (time_after(jiffies, end)) {
			dev_err(vc->dev, "no bulk request from the VC\n");
			return -ETIMEDOUT;
		}
	}
	req->valid = false;
	if (debug > 1 || (debug && vc->frames < 2))
		dev_info(vc->dev, "bulk %u bytes, VC asked %u (rx %u, off %u)\n",
			 size, req->size, req->rx_size, req->offset);

	/* data is page aligned: no head, the aligned part, the rest as tail */
	rx_size = min(size & ~31u, req->rx_size);
	tail = size - rx_size;
	if (0x1c + ALIGN(tail, 4) > VCHI_MAX_MSG)
		return -EINVAL;
	memset(msg, 0, 0x1c);
	put_unaligned_le32(fourcc, msg);
	put_unaligned_le32(size, msg + 4);
	put_unaligned_le32(VCHI_BULK_PIECE, msg + 8);
	put_unaligned_le32(rx_size, msg + 16);
	put_unaligned_le16((u16)-req->offset, msg + 20);
	msg[22] = 1;
	put_unaligned_le16(tail, msg + 26);
	memcpy(msg + 0x1c, data + rx_size, tail);
	n = 0x1c + ALIGN(tail, 4);
	memset(msg + 0x1c + tail, 0, n - 0x1c - tail);
	ret = vchi_send(vc, FCC_BULX, msg, n);
	if (ret)
		return ret;

	for (done = 0; done < rx_size; done += piece) {
		piece = min(rx_size - done, (u32)VCHI_BULK_PIECE);
		if (vc->dma_ok && data >= (u8 *)vc->push &&
		    data < (u8 *)vc->push + VC_FB_SIZE) {
			dma_addr_t a = vc->push_phys + (data + done - (u8 *)vc->push);

			dma_sync_single_for_device(vc->dev, a, piece, DMA_TO_DEVICE);
			ret = vc_tx_dma(vc, data + done, a, piece,
					done + piece == rx_size ? 3 : 1);
		} else
			ret = vc_tx(vc, data + done, piece,
				    done + piece == rx_size ? 3 : 1);
		if (ret)
			return ret;
	}
	return 0;
}

/* ---- dispcommanddrv ---- */

/*
 * request: u16 display | u16 interface id | u32 cmd | 0x100000 | u32 token,
 * then u32 arg (simple commands) or the payload (config, matrix, batch)
 */
static int dspc_send(struct vc *vc, u16 display, u16 id, u32 cmd, u32 token,
		     const void *pl, u32 n)
{
	u8 buf[0x200];

	if (n + 12 > sizeof(buf))
		return -E2BIG;
	put_unaligned_le16(display, buf);
	put_unaligned_le16(id, buf + 2);
	put_unaligned_le32(cmd | 0x100000, buf + 4);
	put_unaligned_le32(token, buf + 8);
	if (n)
		memcpy(buf + 12, pl, n);
	return vchi_send(vc, FCC_DSPC, buf, ALIGN(n + 12, 4));
}

/*
 * wait for the VC's answer on @tok ({token, status[, len, data]}), like the
 * Symbian chipset does before its next step (DFC 0x80359700).
 * Returns the status or a negative error.
 */
static int dspc_wait(struct vc *vc, u32 tok)
{
	unsigned long end = jiffies + msecs_to_jiffies(1000);
	int ret;

	for (;;) {
		if (vc->dspc_rx_seen && vc->dspc_rx_len >= 8 &&
		    get_unaligned_le32(vc->dspc_rx) == tok)
			return get_unaligned_le32(vc->dspc_rx + 4);
		if (time_after(jiffies, end))
			return -ETIMEDOUT;
		vc->dspc_rx_seen = false;
		ret = vchi_pump(vc, 100, &vc->dspc_rx_seen);
		if (ret && ret != -ETIMEDOUT)
			return ret;
	}
}

/* send a DSPC request with a fresh token and wait for its answer */
static int dspc_call(struct vc *vc, u16 display, u16 id, u32 cmd, const void *pl, u32 n)
{
	u32 tok = vc->dspc_token++;
	int ret;

	vc->dspc_rx_seen = false;
	ret = dspc_send(vc, display, id, cmd, tok, pl, n);
	if (ret)
		return ret;
	ret = dspc_wait(vc, tok);
	if (ret < 0 && ret != -ETIMEDOUT)
		return ret;
	if (ret)
		dev_warn(vc->dev, "DSPC cmd %#x: %s %d\n", cmd,
			 ret < 0 ? "no answer" : "status", ret);
	return 0;
}

static int dspc_simple(struct vc *vc, u32 cmd, u32 arg)
{
	__le32 a = cpu_to_le32(arg);

	return dspc_call(vc, DSPC_ALL, DSPC_ID, cmd, &a, 4);
}

/*
 * panel config for the E6 panel (id 01 8b 79, FUN_80349e58 case 0x79 on
 * 0x72) as FUN_80359c44 + FUN_80340d50 build it for interface 4: 640x480,
 * format 2, porches 10/20/2 and 3/3/3, 20 MHz pixel clock, 2 MHz cmd clock
 */
static int dspc_config(struct vc *vc)
{
	static const u16 h16[12] = { VC_W, VC_H, 2, 0, 1, 0xffff,
				     10, 20, 2, 3, 3, 3 };
	u8 pl[32];
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(h16); i++)
		put_unaligned_le16(h16[i], pl + 2 * i);
	put_unaligned_le32(20000000, pl + 24);
	put_unaligned_le32(2000000, pl + 28);
	return dspc_send(vc, 0, DSPC_ID, 3, 0, pl, sizeof(pl));
}

struct dcs {
	u8 cmd;
	u8 len;
	u8 data[3];
};

/* batch (cmd 0x40): u32 count, per command {1, 0, dcs, padded len, len, data} */
static int dspc_batch(struct vc *vc, const struct dcs *c, unsigned int n)
{
	u8 pl[0x180];
	u32 off = 4, pad;
	unsigned int i;

	put_unaligned_le32(n, pl);
	for (i = 0; i < n; i++) {
		pad = ALIGN(c[i].len, 4);
		if (off + 20 + pad > sizeof(pl))
			return -E2BIG;
		put_unaligned_le32(1, pl + off);
		put_unaligned_le32(0, pl + off + 4);
		put_unaligned_le32(c[i].cmd, pl + off + 8);
		put_unaligned_le32(pad, pl + off + 12);
		put_unaligned_le32(c[i].len, pl + off + 16);
		off += 20;
		memset(pl + off, 0, pad);
		memcpy(pl + off, c[i].data, c[i].len);
		off += pad;
	}
	return dspc_call(vc, DSPC_ALL, DSPC_ID, 0x40, pl, off);
}

/*
 * DCS read: batch with one type 0 record {0, token, dcs, 0, len}, sent with
 * header token 0 (FUN_80341168); the VC answers on the record's token with
 * {token, status, len, data} (FUN_803403c0)
 */
static int dspc_read(struct vc *vc, u8 cmd, u8 *data, u32 n)
{
	u32 tok = vc->dspc_token++;
	__le32 pl[6] = { cpu_to_le32(1), 0, cpu_to_le32(tok),
			 cpu_to_le32(cmd), 0, cpu_to_le32(n) };
	int ret;

	vc->dspc_rx_seen = false;
	ret = dspc_send(vc, DSPC_ALL, DSPC_ID, 0x40, 0, pl, sizeof(pl));
	if (ret)
		return ret;
	ret = dspc_wait(vc, tok);
	if (ret < 0)
		return ret;
	if (ret || vc->dspc_rx_len < 12)
		return -EIO;
	n = min(n, vc->dspc_rx_len - 12);
	memcpy(data, vc->dspc_rx + 12, n);
	return n;
}

/*
 * read the panel's status registers like Symbian (power mode, madctl,
 * colmod, image, signal), log them and return the power mode (0x0a)
 */
static int vc_panel_status(struct vc *vc)
{
	static const u8 regs[] = { 0x0a, 0x0b, 0x0c, 0x0d, 0x0e };
	char buf[64];
	unsigned int i, len = 0;
	int ret, pm = -EIO;
	u8 v;

	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		ret = dspc_read(vc, regs[i], &v, 1);
		if (ret == 1) {
			len += scnprintf(buf + len, sizeof(buf) - len,
					 " %02x=%02x", regs[i], v);
			if (!i)
				pm = v;
		} else {
			len += scnprintf(buf + len, sizeof(buf) - len,
					 " %02x=(%d)", regs[i], ret);
			if (!i)
				pm = ret;
		}
	}
	dev_info(vc->dev, "panel status:%s\n", buf);
	return pm;
}

/* panel id reads (dc db da), as Symbian does before powering the panel */
static void vc_panel_id(struct vc *vc)
{
	u8 id[3] = { };
	int i;

	for (i = 0; i < 3; i++)
		dspc_read(vc, 0xdc - i, &id[i], 1);
	dev_info(vc->dev, "panel id %02x %02x %02x\n", id[2], id[1], id[0]);
}

/*
 * cmd 7, colour matrix (FUN_8034dabc layout): display 0, param 0, 3 rows of
 * {r, g, b, offset} with 1.0 = 1 << 23, 3 more words, then the enable word.
 * All zero is "no matrix", what Symbian sends without a matrix layer.
 */
static int dspc_matrix(struct vc *vc)
{
	u8 pl[0x40] = { };
	unsigned int r;

	if (matrix) {
		for (r = 0; r < 3; r++)
			put_unaligned_le32(1 << 23, pl + 16 * r + 4 * r);
		put_unaligned_le32(0x800000, pl + 0x3c);
	}

	return dspc_call(vc, 0, 0, 7, pl, sizeof(pl));
}

/*
 * Symbian panel power-up, op 1 (init), as tools/run_panel.py runs it from
 * kern.bin. It reads 0a..0e first and branches on 0x0a bit 2 (display on):
 *
 * warm (left on by the boot loader): config, power 0x81, backlight, mode,
 *   reads; op 8; the first refresh then sends the zero matrix and composes.
 * cold: power 0x41, 0x40, 245 ms, id reads, config, power 0x81, 51 ms,
 *   backlight + sleep out (0x11), 120 ms, config, zero matrix, mode,
 *   compose, display on (0x29), reads; op 8 3 s later.
 *
 * The timer ticks of the Symbian state machine are milliseconds.
 */
static const struct dcs panel_mode[] = {
	{ 0x26, 1, { 0x01 } },
	{ 0x36, 1, { 0x00 } },
	{ 0x3a, 1, { 0x70 } },
};

/* op 8 */
static int vc_panel_op8(struct vc *vc)
{
	const struct dcs on[] = {
		{ 0x29, 0 },
		{ 0x26, 1, { 0x01 } },
		{ 0x36, 1, { 0x00 } },
		{ 0x3a, 1, { 0x70 } },
		{ 0x38, 0 },
		{ 0x13, 0 },
		{ 0x53, 1, { 0x3f } },
		{ 0x51, 1, { brightness } },
		{ 0x55, 1, { 0x00 } },
	};

	return dspc_batch(vc, on, ARRAY_SIZE(on));
}

static int vc_panel_warm(struct vc *vc)
{
	const struct dcs bl[] = {
		{ 0x53, 1, { 0x3f } },
		{ 0x51, 1, { brightness } },
		{ 0x55, 1, { 0x00 } },
	};
	int ret;

	vc_panel_id(vc);
	ret = dspc_config(vc);
	if (!ret)
		ret = dspc_simple(vc, 2, 0x81);		/* power mode 11 */
	if (!ret)
		ret = dspc_batch(vc, bl, ARRAY_SIZE(bl));
	if (!ret)
		ret = dspc_config(vc);
	if (!ret)
		ret = dspc_batch(vc, panel_mode, ARRAY_SIZE(panel_mode));
	if (ret)
		return ret;
	vc_panel_status(vc);
	return vc_panel_op8(vc);
}

static int vc_panel_cold(struct vc *vc)
{
	const struct dcs bl_wake[] = {
		{ 0x53, 1, { 0x3f } },
		{ 0x51, 1, { brightness } },
		{ 0x55, 1, { 0x00 } },
		{ 0x11, 0 },
	};
	int ret;

	ret = dspc_simple(vc, 2, 0x41);			/* power mode 9 */
	if (!ret)
		ret = dspc_simple(vc, 2, 0x40);		/* power mode 8 */
	if (ret)
		return ret;
	vchi_pump(vc, 245, NULL);
	vc_panel_id(vc);
	ret = dspc_config(vc);
	if (!ret)
		ret = dspc_simple(vc, 2, 0x81);		/* power mode 11 */
	if (ret)
		return ret;
	vchi_pump(vc, 51, NULL);
	ret = dspc_batch(vc, bl_wake, ARRAY_SIZE(bl_wake));
	if (ret)
		return ret;
	vchi_pump(vc, 120, NULL);
	ret = dspc_config(vc);
	if (!ret)
		ret = dspc_matrix(vc);
	if (!ret)
		ret = dspc_batch(vc, panel_mode, ARRAY_SIZE(panel_mode));
	return ret;
}

/* install (config + ping), then op 1; returns 1 for the cold path */
static int vc_panel_on(struct vc *vc)
{
	int ret, pm;
	bool is_cold;

	ret = vchi_open(vc, FCC_DSPC);
	if (!ret)
		ret = dspc_config(vc);
	if (!ret)
		ret = dspc_simple(vc, 5, 0);		/* ping */
	if (ret)
		return ret;
	pm = vc_panel_status(vc);
	is_cold = cold >= 0 ? cold : pm < 0 || !(pm & 0x04);
	dev_info(vc->dev, "panel power-up: %s path\n", is_cold ? "cold" : "warm");
	ret = is_cold ? vc_panel_cold(vc) : vc_panel_warm(vc);
	return ret ? ret : is_cold;
}

/* ---- dispman (vcdispman) ---- */

static int disp_cmd(struct vc *vc, u32 cmd, const u32 *args, unsigned int n,
		    bool sync, u32 *out)
{
	__le32 w[18];		/* cmd + 17 args (GCE element add) */
	unsigned int i;
	int ret;

	if (n >= ARRAY_SIZE(w))
		return -E2BIG;
	w[0] = cpu_to_le32(cmd | (sync ? DISP_REPLY : 0));
	for (i = 0; i < n; i++)
		w[1 + i] = cpu_to_le32(args[i]);
	vc->disp_reply_seen = false;
	ret = vchi_send(vc, FCC_DISP, w, 4 * (n + 1));
	if (ret || !sync)
		return ret;
	ret = vchi_pump(vc, 3000, &vc->disp_reply_seen);
	if (ret) {
		dev_err(vc->dev, "dispman %#x: no reply\n", cmd);
		return ret;
	}
	if (out)
		*out = vc->disp_reply;
	return 0;
}

/* ---- bufman (vc_bufmanx) ---- */

/*
 * write image (FUN_8033b29c): {6, resource, image type, 0, w, h, pitch,
 * src rect (16.16), dst rect}, reply {type, status, nchunks, rows per
 * chunk, chunk bytes, ?, last chunk bytes}, one bulk per chunk, final
 * {type, status}
 */
/*
 * rows y..y+h. the VC ignores the dst rect position, so the image is the
 * whole frame and the band is both src and dst rect. its reply says how many
 * bytes it wants: only the band -> data from row y, more -> from row 0
 */
static int bufm_write(struct vc *vc, u32 y, u32 h)
{
	u32 m[15] = { BUFM_WRITE_IMAGE, vc->resource,
		      rgb565 ? VC_IMG_BUF_565 : VC_IMG_BUF_TYPE, 0,
		      VC_W, VC_H, VC_PITCH, 0, y << 16, VC_W << 16, h << 16,
		      0, y, VC_W, h };
	u8 *data = vc->push + y * VC_PITCH;
	u32 status, nchunks, rows, csize, last, i, size, off, total, room;
	__le32 w[15];
	int ret;

	/* the test pattern puts its frame straight into push */
	if (!vc->push_ready) {
		mutex_lock(&vc->fb_lock);
		memcpy(data, vc->fb + y * VC_PITCH, h * VC_PITCH);
		mutex_unlock(&vc->fb_lock);
	}

	for (i = 0; i < ARRAY_SIZE(m); i++)
		w[i] = cpu_to_le32(m[i]);
	vc->bufm_reply_seen = false;
	ret = vchi_send(vc, FCC_BFMX, w, sizeof(w));
	if (!ret)
		ret = vchi_pump(vc, 2000, &vc->bufm_reply_seen);
	if (ret)
		return ret;
	/* FUN_8033b248 skips word 0 (the message type, 3) */
	status = get_unaligned_le32(vc->bufm_reply + 4);
	nchunks = get_unaligned_le32(vc->bufm_reply + 8);
	rows = get_unaligned_le32(vc->bufm_reply + 12);
	csize = get_unaligned_le32(vc->bufm_reply + 16);
	last = get_unaligned_le32(vc->bufm_reply + 24);
	if (status) {
		dev_err(vc->dev, "bufman write: status %#x\n", status);
		return -EIO;
	}
	total = nchunks ? (nchunks - 1) * csize + last : 0;
	if (total > h * VC_PITCH)
		data = vc->push;
	room = VC_FB_SIZE - (data - (u8 *)vc->push);
	if (h < VC_H && vc->partial_log < 4) {
		vc->partial_log++;
		dev_info(vc->dev, "partial rows %u+%u: VC asked %u x %u rows, %u bytes -> data from row %u\n",
			 y, h, nchunks, rows, total,
			 (u32)((data - (u8 *)vc->push) / VC_PITCH));
	}
	for (i = 0; i < nchunks; i++) {
		size = i + 1 == nchunks ? last : csize;
		off = VC_PITCH * rows * i;
		if (off + size > room) {
			dev_err(vc->dev, "bufman chunk %u out of range\n", i);
			return -EINVAL;
		}
		ret = vchi_bulk_tx(vc, FCC_BFMX, data + off, size);
		if (ret)
			return ret;
	}
	vc->bufm_reply_seen = false;
	ret = vchi_pump(vc, 2000, &vc->bufm_reply_seen);
	if (ret)
		return ret;
	status = get_unaligned_le32(vc->bufm_reply + 4);
	if (status) {
		dev_err(vc->dev, "bufman write done: status %#x\n", status);
		return -EIO;
	}
	return 0;
}

/* the display chipset's compose (FUN_8034d0f4) for one full-screen layer */
static int vc_display_setup(struct vc *vc)
{
	u32 a[16], update;
	int ret;

	ret = vchi_open(vc, FCC_DISP);
	if (!ret)
		ret = vchi_open(vc, FCC_UPDH);
	if (!ret)
		ret = vchi_open_flags(vc, FCC_BFMX, 0x1c);
	if (ret)
		return ret;

	a[0] = 0;
	ret = disp_cmd(vc, DISP_OPEN, a, 1, true, &vc->display);
	if (ret)
		return ret;
	if (!vc->display) {
		dev_err(vc->dev, "DisplayOpen(0) failed\n");
		return -ENODEV;
	}
	a[0] = rgb565 ? VC_IMG_RES_565 : VC_IMG_RES_TYPE;
	a[1] = VC_W;
	a[2] = VC_H;
	ret = disp_cmd(vc, DISP_RES_CREATE, a, 3, true, &vc->resource);
	if (ret)
		return ret;
	if (!vc->resource) {
		dev_err(vc->dev, "ResourceCreate failed\n");
		return -ENOMEM;
	}
	ret = bufm_write(vc, 0, VC_H);
	if (ret)
		return ret;

	a[0] = 1;
	ret = disp_cmd(vc, DISP_UPDATE_START, a, 1, true, &update);
	if (ret)
		return ret;
	a[0] = update;
	a[1] = vc->display;
	a[2] = (u32)-400;			/* layer */
	a[3] = 0;
	a[4] = 0;
	a[5] = VC_W;
	a[6] = VC_H;
	a[7] = vc->resource;
	a[8] = 0;
	a[9] = 0;
	a[10] = VC_W << 16;
	a[11] = VC_H << 16;
	a[12] = 1;				/* fixed alpha */
	a[13] = 0xff;
	a[14] = 0;
	a[15] = 0;				/* transform */
	ret = disp_cmd(vc, DISP_ELEMENT_ADD, a, 16, true, &vc->element);
	if (ret)
		return ret;
	a[0] = update;
	a[1] = vc->display;
	a[2] = 0;
	a[3] = 0;
	a[4] = 0;
	ret = disp_cmd(vc, DISP_SET_BG, a, 5, false, NULL);
	if (ret)
		return ret;
	a[0] = update;
	return disp_cmd(vc, DISP_SUBMIT_SYNC, a, 1, true, NULL);
}

/*
 * every later refresh (Symbian op 3, case 7): DCS 0x35 00 (tearing effect
 * line on, the first refresh leaves it off with 0x34), push the framebuffer,
 * then ElementModified with the dirty rect {update, element, x, y, w, h}
 */
static int vc_display_update(struct vc *vc, u32 y, u32 h)
{
	static const struct dcs te_on[] = { { 0x35, 1, { 0x00 } } };
	static const struct dcs first[] = {
		{ 0x3a, 1, { 0x70 } },
		{ 0x35, 1, { 0x00 } },
		{ 0x34, 0 },
	};
	u32 a[6], update;
	int ret;

	/* Symbian repeats 0x35 every refresh, the dma mode sends it once */
	ret = 0;
	if (vc->first_refresh)
		ret = dspc_batch(vc, first, ARRAY_SIZE(first));
	else if (!vc->dma_ok || !vc->te_on)
		ret = dspc_batch(vc, te_on, ARRAY_SIZE(te_on));
	if (!ret && !vc->first_refresh)
		vc->te_on = true;
	vc->first_refresh = false;
	if (ret)
		return ret;
	ret = bufm_write(vc, y, h);
	if (ret)
		return ret;
	a[0] = 1;
	ret = disp_cmd(vc, DISP_UPDATE_START, a, 1, true, &update);
	if (ret)
		return ret;
	a[0] = update;
	a[1] = vc->element;
	a[2] = 0;
	a[3] = y;
	a[4] = VC_W;
	a[5] = h;
	ret = disp_cmd(vc, DISP_ELEMENT_MOD, a, 6, false, NULL);
	if (ret)
		return ret;
	a[0] = update;
	return disp_cmd(vc, DISP_SUBMIT_SYNC, a, 1, true, NULL);
}

/* ---- thread ---- */

/* 8 colour bars over a white/black split for 3 s, drawn straight into push */
static void vc_mark_dirty(struct vc *vc, u32 y0, u32 y1);

static void vc_test_pattern(struct vc *vc)
{
	static const u32 bars[8] = { 0xffffff, 0xffff00, 0x00ffff, 0x00ff00,
				     0xff00ff, 0xff0000, 0x0000ff, 0x000000 };
	u8 *fb = vc->push;
	unsigned int x, y;
	u32 c;
	int ret;

	for (y = 0; y < VC_H; y++)
		for (x = 0; x < VC_W; x++) {
			c = y < VC_H * 3 / 4 ? bars[x * 8 / VC_W] :
			    (x < VC_W / 2 ? 0xffffff : 0);
			if (rgb565)
				((u16 *)fb)[y * VC_W + x] = vc_565(c);
			else
				((u32 *)fb)[y * VC_W + x] = c;
		}
	vc->push_ready = true;
	ret = vc_display_update(vc, 0, VC_H);
	vc->push_ready = false;
	if (!ret)
		vc->frames++;
	dev_info(vc->dev, "test pattern: %d\n", ret);
	vchi_pump(vc, 3000, NULL);
	/* whatever weston drew meanwhile is in fb: push all of it */
	vc_mark_dirty(vc, 0, VC_H);
}

/*
 * one ElementChangeAttributes (vcdispman FUN_80346198): {update, element,
 * flags, layer, opacity, mask, transform, [dest rect], [src rect (16.16)]},
 * the rects only when their flag is set, dest first
 */
static int vc_element_attr(struct vc *vc, const u32 *dst, const u32 *src)
{
	u32 a[15], update;
	unsigned int n = 7, i;
	int ret;

	a[0] = 1;
	ret = disp_cmd(vc, DISP_UPDATE_START, a, 1, true, &update);
	if (ret)
		return ret;
	a[0] = update;
	a[1] = vc->element;
	a[2] = (dst ? ELEM_CHANGE_DEST : 0) | (src ? ELEM_CHANGE_SRC : 0);
	a[3] = 0;
	a[4] = 0xff;
	a[5] = 0;
	a[6] = 0;
	for (i = 0; dst && i < 4; i++)
		a[n++] = dst[i];
	for (i = 0; src && i < 4; i++)
		a[n++] = src[i];
	ret = disp_cmd(vc, DISP_ELEMENT_ATTR, a, n, false, NULL);
	if (ret)
		return ret;
	a[0] = update;
	return disp_cmd(vc, DISP_SUBMIT_SYNC, a, 1, true, NULL);
}

/*
 * path 2 check, visible on the panel: the whole screen moves down 60 px
 * (dest rect) for 2 s, then scrolls up 60 px (src rect) for 2 s, then back.
 * if it moves, the VC can place/scroll elements with no pixels on the bus
 */
static void vc_disp_probe(struct vc *vc)
{
	static const u32 dst_down[4] = { 0, 60, VC_W, VC_H - 60 };
	static const u32 src_top[4] = { 0, 0, VC_W << 16, (VC_H - 60) << 16 };
	static const u32 dst_full[4] = { 0, 0, VC_W, VC_H };
	static const u32 src_scroll[4] = { 0, 60 << 16, VC_W << 16,
					   (VC_H - 60) << 16 };
	static const u32 dst_scroll[4] = { 0, 0, VC_W, VC_H - 60 };
	static const u32 src_full[4] = { 0, 0, VC_W << 16, VC_H << 16 };
	int ret;

	ret = vc_element_attr(vc, dst_down, src_top);
	dev_info(vc->dev, "disp probe: screen down 60 px: %d\n", ret);
	vchi_pump(vc, 2000, NULL);
	ret = vc_element_attr(vc, dst_scroll, src_scroll);
	dev_info(vc->dev, "disp probe: scrolled up 60 px: %d\n", ret);
	vchi_pump(vc, 2000, NULL);
	ret = vc_element_attr(vc, dst_full, src_full);
	dev_info(vc->dev, "disp probe: back: %d\n", ret);
}

/* khronos rpc on KHRN, the rpi merge buffer: a make current first, then the calls */
#define KHRN_MAKECURRENT 0x4008
#define KHRN_CREATE_GLES20 0x4002
#define KHRN_DESTROY_GL 0x4005
#define KHRN_PID 0x6e6f6b69	/* any client id */
#define KHRN_CREATE_SURFACE 0x4000
#define KHRN_DESTROY_SURFACE 0x4004
#define KHRN_SWAP_BUFFERS 0x400a
/* VC III GLES 2 ids (khclient_vc3), the rpi 0x7xxx ones do not exist here */
#define GL_CLEAR_ID 0x202c
#define GL_CLEAR_COLOR_ID 0x2003
#define GL_COLOR_BUFFER_BIT 0x4000
#define KHRN_GL20 2		/* EGL_SERVER_GL20 */
/* khclient_vc3 config 13: RGB565, no depth, no mask */
#define KHRN_RGB_565 0xd20d
/* register a surface handle before it is shown, with reply */
#define DISP_REGISTER 0x1c
#define KHRN_FMT_NONE 0xffffffff
#define DISP_ELEMENT_REMOVE 0x15
/* GCE element add {update, surface, flags, key, display, layer, dst x y w h,
 * src 16.16, alpha flags, opacity, buffer index}, answers the element */
#define DISP_GCE_ELEMENT_ADD 0x1e
#define VC_IMG_TF_RGB565 25	/* rpi vc_image_types.h numbering */
#define KHRN_WINDOW 0xe6000001	/* any key, see vc_khrn_draw */
#define KHRN_GCE_SURFACE 0x96000001	/* one-handle GCE surface, id 1 */
#define KHRN_FLUSH_WAIT 0x4009
#define KHRN_CREATE_WRAPPED 0x4012
#define KHRN_CREATE_GLOBAL_IMAGE 0x4019
#define KHRN_DESTROY_GLOBAL_IMAGE 0x401c
#define KHRN_PIXEL_RGB_565 3		/* EGL_PIXEL_FORMAT_RGB_565_BRCM */
#define KHRN_PIXEL_RENDER_GLES2 (1 << 5)

/*
 * mc = the make current words after the pid {gltype, gl ctx, draw, read, vg
 * ctx, vg surf} (NULL = null make current), then n words of calls. with
 * reply: waits for one KHRN message (raw in vc->khrn_rx), *res = word 0
 */
static int khrn_send(struct vc *vc, const u32 *mc, const u32 *call,
		     unsigned int n, bool reply, u32 *res)
{
	__le32 w[9 + 24];
	unsigned int i;
	int ret;

	if (n > 24)
		return -E2BIG;
	memset(w, 0, sizeof(w));
	w[0] = cpu_to_le32(KHRN_MAKECURRENT);
	w[1] = cpu_to_le32(KHRN_PID);
	for (i = 0; mc && i < 6; i++)
		w[3 + i] = cpu_to_le32(mc[i]);
	for (i = 0; i < n; i++)
		w[9 + i] = cpu_to_le32(call[i]);
	vc->khrn_rx_seen = false;
	ret = vchi_send(vc, FCC_KHRN, w, 4 * (9 + n));
	if (ret || !reply)
		return ret;
	ret = vchi_pump(vc, 2000, &vc->khrn_rx_seen);
	if (ret)
		return ret;
	dev_info(vc->dev, "khrn: reply on %4.4s, %u bytes: %*ph\n",
		 (char *)&vc->khrn_rx_fcc, vc->khrn_rx_len,
		 min_t(u32, vc->khrn_rx_len, 32), vc->khrn_rx);
	if (vc->khrn_rx_len < 4)
		return -EPROTO;
	if (res)
		*res = get_unaligned_le32(vc->khrn_rx);
	return 0;
}

static int khrn_call(struct vc *vc, const u32 *call, unsigned int n, bool reply,
		     u32 *res)
{
	return khrn_send(vc, NULL, call, n, reply, res);
}

/*
 * khrn probe: open the services, create a GLES 2 context (0x4002) and
 * destroy it (0x4005). a handle back = the GL server talks
 */
static void vc_gl_element_remove(struct vc *vc, u32 el)
{
	u32 a[2], update;

	a[0] = 1;
	if (disp_cmd(vc, DISP_UPDATE_START, a, 1, true, &update))
		return;
	a[0] = update;
	a[1] = el;
	disp_cmd(vc, DISP_ELEMENT_REMOVE, a, 2, false, NULL);
	a[0] = update;
	disp_cmd(vc, DISP_SUBMIT_SYNC, a, 1, true, NULL);
}

/* reply word i of a "res" reply, the VC puts its length in front */
static u32 khrn_word(struct vc *vc, unsigned int i)
{
	if (vc->khrn_rx_len >= 4 &&
	    get_unaligned_le32(vc->khrn_rx) == vc->khrn_rx_len - 4)
		i++;
	return vc->khrn_rx_len >= 4 * (i + 1) ?
		get_unaligned_le32(vc->khrn_rx + 4 * i) : 0;
}

/* clear surf with rgba (float bits) and wait for the VC (0x4009 {1, 0}) */
static int vc_khrn_clear(struct vc *vc, u32 ctx, u32 surf, const u32 *rgba)
{
	u32 mc[6] = { KHRN_GL20, ctx, surf, surf, 0, 0 };
	u32 draw[10], r = 0;
	int ret;

	draw[0] = GL_CLEAR_COLOR_ID;
	memcpy(draw + 1, rgba, 16);
	draw[5] = GL_CLEAR_ID;
	draw[6] = GL_COLOR_BUFFER_BIT;
	draw[7] = KHRN_FLUSH_WAIT;
	draw[8] = 1;
	draw[9] = 0;
	ret = khrn_send(vc, mc, draw, 10, true, &r);
	dev_info(vc->dev, "khrn draw: clear + flush on %#x -> %d, %#x%s\n", surf,
		 ret, r, ret ? "" : "  THE VC RENDERED");
	return ret;
}

/* wrap an image the VC already has into an EGL surface (0x4012), 0 = failed */
static u32 vc_khrn_wrap(struct vc *vc, u32 h0, u32 h1)
{
	u32 ws[6] = { KHRN_CREATE_WRAPPED, h0, h1, KHRN_FMT_NONE,
		      KHRN_FMT_NONE, 0 };
	u32 surf;
	int ret;

	ret = khrn_call(vc, ws, 6, true, NULL);
	surf = ret ? 0 : khrn_word(vc, 0);
	dev_info(vc->dev, "khrn draw: wrap %#x %#x -> %d, %#x, handle %#x\n",
		 h0, h1, ret, surf, khrn_word(vc, 1));
	return surf;
}

static void vc_khrn_destroy_surface(struct vc *vc, u32 surf)
{
	u32 d[2] = { KHRN_DESTROY_SURFACE, surf };

	/* no reply on VC III (khclient FUN_0001ed98) */
	if (surf)
		khrn_call(vc, d, 2, false, NULL);
}

/* GCE element of a surface registered with DISP 0x1c, shown 10 s */
static void vc_gce_show(struct vc *vc, u32 sid, u32 handle, u32 w, u32 h)
{
	u32 a[17], update, el = 0;
	int ret;

	a[0] = 1;
	if (disp_cmd(vc, DISP_UPDATE_START, a, 1, true, &update))
		return;
	a[0] = update;
	a[1] = sid;
	a[2] = 0;
	a[3] = handle;
	a[4] = vc->display;
	a[5] = 10;			/* layer, ours is -400 */
	a[6] = (VC_W - w) / 2;
	a[7] = (VC_H - h) / 2;
	a[8] = w;
	a[9] = h;
	a[10] = 0;
	a[11] = 0;
	a[12] = w << 16;
	a[13] = h << 16;
	a[14] = 0;			/* bit 0 adds a slot check i did not follow */
	a[15] = 0xff;
	a[16] = 0;			/* buffer index */
	dev_info(vc->dev, "khrn draw: 2e GCE element of %#x, LOOK AT THE MIDDLE\n", sid);
	ret = disp_cmd(vc, DISP_GCE_ELEMENT_ADD, a, 17, true, &el);
	dev_info(vc->dev, "khrn draw: 2e GCE element add -> %d, %#x\n", ret, el);
	a[0] = update;
	disp_cmd(vc, DISP_SUBMIT_SYNC, a, 1, true, NULL);
	vchi_pump(vc, 10000, NULL);	/* 10 s, long enough to see */
	if (!ret && el && el != (u32)-1)
		vc_gl_element_remove(vc, el);
}

/*
 * khrn_draw test: a global image cleared green, then a 320x240 window
 * cleared magenta and shown as a GCE element for 10 s
 */
static void vc_khrn_draw(struct vc *vc)
{
	static const u32 green[4] = { 0, 0x3f800000, 0, 0x3f800000 };
	static const u32 magenta[4] = { 0x3f800000, 0, 0x3f800000, 0x3f800000 };
	const u32 W = 320, H = 240;
	u32 create[3] = { KHRN_CREATE_GLES20, 0, 1 };
	u32 gi[4] = { KHRN_CREATE_GLOBAL_IMAGE, W, H,
		      KHRN_PIXEL_RGB_565 | KHRN_PIXEL_RENDER_GLES2 };
	/* VC III layout (khclient_vc3 FUN_000096d4): {win, buffers, w, h,
	 * color, depth, mask, multi, largest, mipmap, sem avail, sem window} */
	u32 cs[13] = { KHRN_CREATE_SURFACE, 0, 1, W, H, KHRN_RGB_565,
		       KHRN_FMT_NONE, KHRN_FMT_NONE, 0, 0, 0, 0, 0 };
	u32 sw[6] = { KHRN_SWAP_BUFFERS, 0, W, H, 0, 0 };
	u32 mc[6] = { KHRN_GL20, 0, 0, 0, 0, 0 };
	u32 ctx = 0, id0, id1, surf, reg = 0, d[3], a[5];
	int ret;

	/* the same flags as /dev/vc-khrn (vc_khrn_serve), it reuses the services */
	if (vchi_open_flags(vc, FCC_KHRN, 0x1c) || vchi_open(vc, FCC_KHHN) ||
	    vchi_open(vc, FCC_KHAN)) {
		dev_info(vc->dev, "khrn draw: services not available\n");
		return;
	}
	vc->khrn_open = true;
	ret = khrn_call(vc, create, 3, true, &ctx);
	dev_info(vc->dev, "khrn draw: context -> %d, %#x\n", ret, ctx);
	if (ret || !ctx)
		return;

	ret = khrn_call(vc, gi, 4, true, NULL);
	id0 = ret ? 0 : khrn_word(vc, 0);
	id1 = ret ? 0 : khrn_word(vc, 1);
	dev_info(vc->dev, "khrn draw: 1 global image -> %d, id %#x %#x\n", ret, id0, id1);
	if (id0 || id1) {
		surf = vc_khrn_wrap(vc, id0, id1);
		if (surf && vc_khrn_clear(vc, ctx, surf, green))
			goto out_ctx;		/* the VC hangs, nothing more will answer */
		vc_khrn_destroy_surface(vc, surf);
		d[0] = KHRN_DESTROY_GLOBAL_IMAGE;
		d[1] = id0;
		d[2] = id1;
		khrn_call(vc, d, 3, true, NULL);
	}

	/*
	 * the window handle is only a key: the swap posts the buffer under it in
	 * the VC handle table, DISP 0x1c and 0x1e must use the same key
	 */
	cs[1] = KHRN_WINDOW;
	ret = khrn_call(vc, cs, 13, true, NULL);
	/* reply {w, h, serverbuffer, handle} */
	surf = ret ? 0 : khrn_word(vc, 2);
	dev_info(vc->dev, "khrn draw: 2 window surface -> %d, %#x, handle %#x\n",
		 ret, surf, khrn_word(vc, 3));
	if (!surf)
		goto out_ctx;
	/* one-handle GCE surface, TF_RGB565, answer (n << 28) | buffer size */
	a[0] = KHRN_GCE_SURFACE;
	a[1] = KHRN_WINDOW;
	a[2] = VC_IMG_TF_RGB565;
	a[3] = W;
	a[4] = H;
	ret = disp_cmd(vc, DISP_REGISTER, a, 5, true, &reg);
	dev_info(vc->dev, "khrn draw: register %#x -> %d, %#x (0x28000 = 565 tiled)\n",
		 KHRN_WINDOW, ret, reg);
	if (ret || (s32)reg < 0)	/* bad args answer -2 (0x43d34) */
		reg = 0;
	if (vc_khrn_clear(vc, ctx, surf, magenta))
		goto out_ctx;
	/* VC III swap {serverbuffer, w, h, window, preserve}, no reply */
	sw[1] = surf;
	sw[4] = KHRN_WINDOW;
	mc[1] = ctx;
	mc[2] = surf;
	mc[3] = surf;
	ret = khrn_send(vc, mc, sw, 6, false, NULL);
	dev_info(vc->dev, "khrn draw: 2a swap sent -> %d\n", ret);
	vchi_pump(vc, 500, NULL);	/* the worker posts the buffer */
	if (!ret && reg)
		vc_gce_show(vc, KHRN_GCE_SURFACE, KHRN_WINDOW, W, H);
	vc_khrn_destroy_surface(vc, surf);
out_ctx:
	d[0] = KHRN_DESTROY_GL;
	d[1] = ctx;
	khrn_call(vc, d, 2, false, NULL);
	vchi_pump(vc, 200, NULL);
	dev_info(vc->dev, "khrn draw: cleaned up\n");
}

static void vc_khrn_probe(struct vc *vc)
{
	static const u32 fcc[] = { FCC_KHRN, FCC_KHHN, FCC_KHAN };
	u32 create[3] = { KHRN_CREATE_GLES20, 0, 1 };
	u32 destroy[2] = { KHRN_DESTROY_GL, 0 };
	unsigned int i;
	u32 ctx = 0;
	int ret;

	for (i = 0; i < ARRAY_SIZE(fcc); i++) {
		ret = vchi_open(vc, fcc[i]);
		dev_info(vc->dev, "khrn probe: %4.4s -> %s (%d, flags %#x)\n",
			 (char *)&fcc[i], ret ? "NOT available" : "AVAILABLE",
			 ret, vc->svc_reply_flags);
		if (ret)
			return;
	}
	ret = khrn_call(vc, create, ARRAY_SIZE(create), true, &ctx);
	dev_info(vc->dev, "khrn probe: create GLES20 context -> %d, handle %#x%s\n",
		 ret, ctx, !ret && ctx ? "  THE GL SERVER ANSWERS" : "");
	if (ret || !ctx)
		return;
	destroy[1] = ctx;
	ret = khrn_call(vc, destroy, ARRAY_SIZE(destroy), false, NULL);
	vchi_pump(vc, 200, NULL);
	dev_info(vc->dev, "khrn probe: destroy context -> %d\n", ret);
}

static void vc_khrn_serve(struct vc *vc);

static int vc_thread(void *arg)
{
	static const struct dcs disp_on[] = { { 0x29, 0 } };
	static const struct dcs first[] = {
		{ 0x3a, 1, { 0x70 } },
		{ 0x35, 1, { 0x00 } },
		{ 0x34, 0 },
	};
	struct vc *vc = arg;
	bool is_cold = false;
	int ret;

	ret = vc_boot(vc);
	if (!ret)
		ret = vchi_connect(vc);
	if (!ret)
		ret = vc_panel_on(vc);
	if (ret > 0) {
		/* cold: compose, display on, op 8 three seconds later */
		is_cold = true;
		ret = vc_display_setup(vc);
		if (!ret)
			ret = dspc_batch(vc, disp_on, ARRAY_SIZE(disp_on));
		if (!ret) {
			vc_panel_status(vc);
			vchi_pump(vc, 3000, NULL);
			ret = vc_panel_op8(vc);
		}
	} else if (!ret) {
		/* warm: the first refresh (op 3) composes */
		ret = dspc_matrix(vc);
		if (!ret)
			ret = dspc_batch(vc, first, ARRAY_SIZE(first));
		if (!ret)
			ret = vc_display_setup(vc);
	}
	if (ret) {
		dev_err(vc->dev, "VideoCore bring-up failed: %d\n", ret);
		/* stay around for kthread_stop() */
		while (!kthread_should_stop())
			schedule_timeout_interruptible(HZ);
		return ret;
	}
	/* the cold path has not refreshed yet: op 3 with the first-refresh DCS */
	vc->first_refresh = is_cold;
	if (is_cold)
		WRITE_ONCE(vc->dirty, true);
	vc->ready = true;
	dev_info(vc->dev, "display up, %ux%u\n", VC_W, VC_H);
	if (khrn_probe)
		vc_khrn_probe(vc);
	vc->dma_ok = vc->gfx_ok;
	vc->st_next = jiffies + 10 * HZ;
	if (test_pattern)
		vc_test_pattern(vc);
	if (disp_probe)
		vc_disp_probe(vc);
	if (khrn_draw)
		vc_khrn_draw(vc);

	while (!kthread_should_stop()) {
		wait_event_interruptible_timeout(vc->wq,
						 READ_ONCE(vc->dirty) ||
						 READ_ONCE(vc->bl_pending) ||
						 READ_ONCE(vc->kreq) ||
						 kthread_should_stop(),
						 msecs_to_jiffies(50));
		/* keep the VC's messages (and our rx credits) moving */
		vchi_pump(vc, 0, NULL);
		vchi_return_credits(vc);
		if (READ_ONCE(vc->kreq))
			vc_khrn_serve(vc);
		if (READ_ONCE(vc->bl_pending)) {
			struct dcs bl[] = {
				{ 0x51, 1, { backlight_get_brightness(vc->bl) } },
			};

			WRITE_ONCE(vc->bl_pending, false);
			ret = dspc_batch(vc, bl, ARRAY_SIZE(bl));
			if (ret)
				dev_err_ratelimited(vc->dev,
						    "brightness: %d\n", ret);
		}
		if (time_after(jiffies, vc->st_next)) {
			if (vc->st_frames)
				dev_info(vc->dev, "%u frames in 10 s (%s): push avg %llu us, max %llu us, avg %u rows\n",
					 vc->st_frames, vc->dma_ok ? "dma" : "pio",
					 div_u64(vc->st_ns, vc->st_frames * 1000),
					 div_u64(vc->st_max, 1000),
					 vc->st_rows / vc->st_frames);
			vc->st_frames = vc->st_rows = 0;
			vc->st_ns = vc->st_max = 0;
			vc->st_next = jiffies + 10 * HZ;
		}
		if (READ_ONCE(vc->dirty)) {
			u32 y0 = 0, y1 = VC_H;
			unsigned long flags;
			u64 t0;

			spin_lock_irqsave(&vc->band_lock, flags);
			WRITE_ONCE(vc->dirty, false);
			/* the first refresh composes, it always goes out whole */
			if (partial && !vc->first_refresh &&
			    vc->band_y0 < vc->band_y1) {
				y0 = vc->band_y0;
				y1 = vc->band_y1;
			}
			vc->band_y0 = VC_H;
			vc->band_y1 = 0;
			spin_unlock_irqrestore(&vc->band_lock, flags);
			t0 = ktime_get_ns();
			ret = vc_display_update(vc, y0, y1 - y0);
			t0 = ktime_get_ns() - t0;
			if (!ret && vc->frames < UINT_MAX)
				vc->frames++;
			if (!ret) {
				vc->st_frames++;
				vc->st_rows += y1 - y0;
				vc->st_ns += t0;
				vc->st_max = max(vc->st_max, t0);
			}
			if (!ret && debug && vc->frames == 1)
				vc_panel_status(vc);
			if (ret)
				dev_err_ratelimited(vc->dev,
						    "frame update failed: %d\n",
						    ret);
		}
	}
	return 0;
}

/* ---- drm ---- */

/*
 * KMS on top of the push thread: one fixed 640x480 plane. a commit copies
 * the damage into vc->fb and wakes the thread, faster frames get merged. no
 * vblank irq, the atomic helpers fake the events
 */

static inline struct vc *to_vc(struct drm_device *drm)
{
	return container_of(drm, struct vc, drm);
}

/* rows [y0, y1) changed, the thread pushes their union */
static void vc_mark_dirty(struct vc *vc, u32 y0, u32 y1)
{
	unsigned long flags;

	spin_lock_irqsave(&vc->band_lock, flags);
	vc->band_y0 = min(vc->band_y0, y0);
	vc->band_y1 = max(vc->band_y1, y1);
	WRITE_ONCE(vc->dirty, true);
	spin_unlock_irqrestore(&vc->band_lock, flags);
	wake_up(&vc->wq);
}

/* pixels x1..x2 of one plane row into one vc->fb row (converting to 565) */
static void vc_row(struct vc *vc, u8 *d, const u8 *s,
		   const struct drm_format_info *f, u32 x1, u32 x2)
{
	u32 x;

	if (!rgb565 || f->format == DRM_FORMAT_RGB565) {
		memcpy(d + x1 * f->cpp[0], s + x1 * f->cpp[0],
		       (x2 - x1) * f->cpp[0]);
		return;
	}
	for (x = x1; x < x2; x++)
		((u16 *)d)[x] = vc_565(((const u32 *)s)[x]);
}

/*
 * weston sends a commit without clips (= whole screen) before every real
 * one, so diff the rows against what is on screen. fb_lock held
 */
static void vc_copy_changed_rows(struct vc *vc, const struct iosys_map *src,
				 const struct drm_framebuffer *fb)
{
	const u8 *s = src->vaddr;
	u8 *d = vc->fb;
	u32 y, y0 = VC_H, y1 = 0;

	for (y = 0; y < VC_H; y++, s += fb->pitches[0], d += VC_PITCH) {
		vc_row(vc, vc->row, s, fb->format, 0, VC_W);
		if (!memcmp(d, vc->row, VC_PITCH))
			continue;
		memcpy(d, vc->row, VC_PITCH);
		y0 = min(y0, y);
		y1 = y + 1;
	}
	if (y0 < y1)
		vc_mark_dirty(vc, y0, y1);
}

static void vc_copy_rect(struct vc *vc, struct drm_plane_state *state,
			 const struct drm_rect *rect, bool full)
{
	struct drm_shadow_plane_state *shadow = to_drm_shadow_plane_state(state);
	struct drm_framebuffer *fb = state->fb;
	const u8 *s = shadow->data[0].vaddr;
	u32 y;
	int idx;

	if (!drm_dev_enter(&vc->drm, &idx))
		return;
	if (!drm_gem_fb_begin_cpu_access(fb, DMA_FROM_DEVICE)) {
		mutex_lock(&vc->fb_lock);
		if (full)
			vc_copy_changed_rows(vc, &shadow->data[0], fb);
		else {
			for (y = rect->y1; y < rect->y2; y++)
				vc_row(vc, vc->fb + y * VC_PITCH,
				       s + y * fb->pitches[0], fb->format,
				       rect->x1, rect->x2);
			vc_mark_dirty(vc, rect->y1, rect->y2);
		}
		mutex_unlock(&vc->fb_lock);
		drm_gem_fb_end_cpu_access(fb, DMA_FROM_DEVICE);
	}
	drm_dev_exit(idx);
}

static void vc_pipe_enable(struct drm_simple_display_pipe *pipe,
			   struct drm_crtc_state *crtc_state,
			   struct drm_plane_state *plane_state)
{
	struct drm_rect rect = DRM_RECT_INIT(0, 0, VC_W, VC_H);
	struct vc *vc = to_vc(pipe->crtc.dev);

	vc_copy_rect(vc, plane_state, &rect, false);
}

static void vc_pipe_disable(struct drm_simple_display_pipe *pipe)
{
	struct vc *vc = to_vc(pipe->crtc.dev);

	mutex_lock(&vc->fb_lock);
	memset(vc->fb, 0, VC_FB_SIZE);
	mutex_unlock(&vc->fb_lock);
	vc_mark_dirty(vc, 0, VC_H);
}

static void vc_pipe_update(struct drm_simple_display_pipe *pipe,
			   struct drm_plane_state *old_state)
{
	struct drm_plane_state *state = pipe->plane.state;
	struct drm_rect rect;

	if (state->fb && drm_atomic_helper_damage_merged(old_state, state, &rect)) {
		if (debug > 1)
			dev_info(to_vc(pipe->crtc.dev)->dev,
				 "commit: %u clips, damage " DRM_RECT_FMT "\n",
				 drm_plane_get_damage_clips_count(state),
				 DRM_RECT_ARG(&rect));
		vc_copy_rect(to_vc(pipe->crtc.dev), state, &rect,
			     !drm_plane_get_damage_clips_count(state));
	}
}

static const struct drm_simple_display_pipe_funcs vc_pipe_funcs = {
	.enable = vc_pipe_enable,
	.disable = vc_pipe_disable,
	.update = vc_pipe_update,
	DRM_GEM_SIMPLE_DISPLAY_PIPE_SHADOW_PLANE_FUNCS,
};

static const u32 vc_formats[] = { DRM_FORMAT_XRGB8888, DRM_FORMAT_RGB565 };
static const u64 vc_modifiers[] = { DRM_FORMAT_MOD_LINEAR,
				    DRM_FORMAT_MOD_INVALID };

/* 2.46" VGA, 50 x 37 mm, 60 Hz */
static const struct drm_display_mode vc_mode = {
	DRM_MODE_INIT(60, VC_W, VC_H, 50, 37),
};

static int vc_conn_get_modes(struct drm_connector *conn)
{
	return drm_connector_helper_get_modes_fixed(conn, &vc_mode);
}

static const struct drm_connector_helper_funcs vc_conn_helper_funcs = {
	.get_modes = vc_conn_get_modes,
};

static const struct drm_connector_funcs vc_conn_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static const struct drm_mode_config_funcs vc_mode_config_funcs = {
	.fb_create = drm_gem_fb_create_with_dirty,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

DEFINE_DRM_GEM_FOPS(vc_drm_fops);

static const struct drm_driver vc_drm_driver = {
	.driver_features = DRIVER_MODESET | DRIVER_GEM | DRIVER_ATOMIC,
	.name = "rapuyama-vc",
	.desc = "Nokia RapuYama VideoCore display",
	.date = "20261018",
	.major = 1,
	.fops = &vc_drm_fops,
	DRM_GEM_SHMEM_DRIVER_OPS,
};

static int vc_drm_init(struct vc *vc)
{
	struct drm_device *drm = &vc->drm;
	int ret;

	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;
	drm->mode_config.min_width = drm->mode_config.max_width = VC_W;
	drm->mode_config.min_height = drm->mode_config.max_height = VC_H;
	drm->mode_config.preferred_depth = 24;
	drm->mode_config.funcs = &vc_mode_config_funcs;

	drm_connector_helper_add(&vc->conn, &vc_conn_helper_funcs);
	ret = drm_connector_init(drm, &vc->conn, &vc_conn_funcs,
				 DRM_MODE_CONNECTOR_DSI);
	if (ret)
		return ret;
	/* RGB565 only in rgb565 mode (vc->fb holds 565 then) */
	ret = drm_simple_display_pipe_init(drm, &vc->pipe, &vc_pipe_funcs,
					   vc_formats, rgb565 ? 2 : 1,
					   vc_modifiers, &vc->conn);
	if (ret)
		return ret;
	drm_plane_enable_fb_damage_clips(&vc->pipe.plane);
	drm_mode_config_reset(drm);
	return 0;
}

/* ---- backlight ---- */

/* the bus belongs to the thread, so only flag it. 0 when blanked/off */
static int vc_bl_update(struct backlight_device *bl)
{
	struct vc *vc = bl_get_data(bl);

	WRITE_ONCE(vc->bl_pending, true);
	wake_up(&vc->wq);
	return 0;
}

static const struct backlight_ops vc_bl_ops = {
	.update_status = vc_bl_update,
};

static int vc_bl_register(struct vc *vc)
{
	struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.max_brightness = 0xff,
		.brightness = min(brightness, 0xffu),
	};

	vc->bl = devm_backlight_device_register(vc->dev, "rapuyama-vc", vc->dev,
						vc, &vc_bl_ops, &props);
	return PTR_ERR_OR_ZERO(vc->bl);
}

/* ---- /dev/vc-khrn: khronos rpc for userspace ---- */

/* userspace builds the whole message, the driver only moves it (layout in
 * linux/vc-khrn/vc-khrn.h) */
struct vc_khrn_msg {
	__u32 fourcc;		/* 'KHRN' or 'DISP' */
	__u32 len;		/* message bytes */
	__u64 data;
	__u64 bulk;		/* the pieces back to back */
	__u64 bulk_sizes;	/* __u32 per piece, each one VCHI bulk */
	__u32 nbulk;
	__u32 reply_max;	/* 0 = no reply, else wait for one message */
	__u64 reply;
	__u32 reply_len;	/* out */
	__u32 pad;
};
#define VC_KHRN_IOC_MSG _IOWR('V', 1, struct vc_khrn_msg)
/* the dispman display the driver opened (DISP 0x1e and friends need it) */
#define VC_KHRN_IOC_DISPLAY _IOR('V', 2, __u32)
#define VC_KHRN_MAX_BULK (1 << 20)
#define VC_KHRN_MAX_PIECES 64	/* glShaderSource sends one per string */

struct vc_khrn_req {
	u32 fourcc, len, nbulk, reply_max, reply_len;
	u8 msg[VCHI_MAX_MSG];
	u8 *bulk;
	u32 sizes[VC_KHRN_MAX_PIECES];
	u8 reply[0x200];
	int ret;
	struct completion done;
};

/* in the thread: the VCHI code is not reentrant */
static void vc_khrn_serve(struct vc *vc)
{
	struct vc_khrn_req *r = vc->kreq;
	bool *seen = r->fourcc == FCC_DISP ? &vc->disp_reply_seen :
		     &vc->khrn_rx_seen;
	u32 i, off;
	int ret = 0;

	if (!vc->khrn_open) {
		/* small bulks go inline like BFMX (open answer flags 0xd) */
		ret = vchi_open_flags(vc, FCC_KHRN, 0x1c);
		if (!ret)
			ret = vchi_open(vc, FCC_KHHN);
		if (!ret)
			ret = vchi_open(vc, FCC_KHAN);
		vc->khrn_open = !ret;
	}
	*seen = false;
	if (!ret)
		ret = vchi_send(vc, r->fourcc, r->msg, r->len);
	for (i = 0, off = 0; !ret && i < r->nbulk; off += r->sizes[i++])
		ret = vchi_bulk_tx(vc, r->fourcc, r->bulk + off, r->sizes[i]);
	if (!ret && r->reply_max)
		ret = vchi_pump(vc, 2000, seen);
	r->reply_len = 0;
	if (!ret && r->reply_max) {
		if (r->fourcc == FCC_DISP) {
			put_unaligned_le32(vc->disp_reply, r->reply);
			r->reply_len = 4;
		} else {
			r->reply_len = min3(vc->khrn_rx_len, r->reply_max,
					    (u32)sizeof(r->reply));
			memcpy(r->reply, vc->khrn_rx, r->reply_len);
		}
	}
	r->ret = ret;
	WRITE_ONCE(vc->kreq, NULL);
	complete(&r->done);
}

static long vc_khrn_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	struct vc *vc = container_of(f->private_data, struct vc, misc);
	struct vc_khrn_msg m;
	struct vc_khrn_req *r;
	u32 i, total;
	long ret;

	if (cmd == VC_KHRN_IOC_DISPLAY) {
		if (!READ_ONCE(vc->ready))
			return -EAGAIN;
		return put_user(vc->display, (__u32 __user *)arg);
	}
	if (cmd != VC_KHRN_IOC_MSG)
		return -ENOTTY;
	if (copy_from_user(&m, (void __user *)arg, sizeof(m)))
		return -EFAULT;
	if ((m.fourcc != FCC_KHRN && m.fourcc != FCC_DISP) ||
	    !m.len || m.len > VCHI_MAX_MSG || m.nbulk > VC_KHRN_MAX_PIECES ||
	    (m.nbulk && m.fourcc != FCC_KHRN))
		return -EINVAL;
	if (!READ_ONCE(vc->ready))
		return -EAGAIN;
	r = kzalloc(sizeof(*r), GFP_KERNEL);
	if (!r)
		return -ENOMEM;
	r->fourcc = m.fourcc;
	r->len = m.len;
	r->nbulk = m.nbulk;
	r->reply_max = m.reply_max;
	init_completion(&r->done);
	ret = -EFAULT;
	if (copy_from_user(r->msg, u64_to_user_ptr(m.data), m.len) ||
	    copy_from_user(r->sizes, u64_to_user_ptr(m.bulk_sizes),
			   m.nbulk * sizeof(u32)))
		goto out;
	for (i = 0, total = 0; i < m.nbulk; i++) {
		ret = -EINVAL;
		/* the bulk tail goes in one VCHI message, see vchi_bulk_tx */
		if (!r->sizes[i] || r->sizes[i] > VC_KHRN_MAX_BULK ||
		    total + r->sizes[i] > VC_KHRN_MAX_BULK)
			goto out;
		total += r->sizes[i];
	}
	if (total) {
		ret = -ENOMEM;
		r->bulk = kvmalloc(total, GFP_KERNEL);
		if (!r->bulk)
			goto out;
		ret = -EFAULT;
		if (copy_from_user(r->bulk, u64_to_user_ptr(m.bulk), total))
			goto out;
	}
	mutex_lock(&vc->khrn_lock);
	WRITE_ONCE(vc->kreq, r);
	wake_up(&vc->wq);
	/* uninterruptible: the thread owns r until it completes */
	wait_for_completion(&r->done);
	mutex_unlock(&vc->khrn_lock);
	ret = r->ret;
	if (!ret && r->reply_len) {
		m.reply_len = r->reply_len;
		if (r->reply_len > m.reply_max ||
		    copy_to_user(u64_to_user_ptr(m.reply), r->reply, r->reply_len) ||
		    copy_to_user((void __user *)arg, &m, sizeof(m)))
			ret = -EFAULT;
	}
out:
	kvfree(r->bulk);
	kfree(r);
	return ret;
}

static const struct file_operations vc_khrn_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = vc_khrn_ioctl,
};

/* ---- platform ---- */

static void vc_push_free(struct vc *vc)
{
	if (!dma) {
		vfree(vc->push);
		return;
	}
	dma_unmap_single(vc->dev, vc->push_phys, VC_FB_SIZE, DMA_TO_DEVICE);
	free_pages_exact(vc->push, VC_FB_SIZE);
}

static int rapuyama_videocore_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct vc *vc;
	int ret;

	vc = devm_drm_dev_alloc(dev, &vc_drm_driver, struct vc, drm);
	if (IS_ERR(vc))
		return PTR_ERR(vc);
	vc->dev = dev;
	vc->dspc_token = 0x100;
	init_waitqueue_head(&vc->wq);
	spin_lock_init(&vc->band_lock);
	mutex_init(&vc->fb_lock);
	mutex_init(&vc->khrn_lock);
	vc->band_y0 = VC_H;	/* empty band = push everything */
	platform_set_drvdata(pdev, vc);

	/* the gfx DMA engine shares the area, so no exclusive request */
	vc->bus = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(vc->bus))
		return PTR_ERR(vc->bus);
	vc->clk = devm_clk_get_optional_enabled(dev, NULL);
	if (IS_ERR(vc->clk))
		return dev_err_probe(dev, PTR_ERR(vc->clk), "no ViSSI clock\n");
	vc->run = devm_gpiod_get(dev, "run", GPIOD_OUT_HIGH);
	if (IS_ERR(vc->run))
		return dev_err_probe(dev, PTR_ERR(vc->run), "no run gpio\n");

	vc->pitch = VC_W * (rgb565 ? 2 : 4);
	vc->fb = vzalloc(VC_FB_SIZE);
	if (!vc->fb)
		return -ENOMEM;
	/* dma: the push buffer must be physically contiguous */
	if (dma) {
		vc->gfx = devm_platform_ioremap_resource(pdev, 1);
		vc->pll = devm_platform_ioremap_resource(pdev, 2);
		vc->gdiv = devm_platform_ioremap_resource(pdev, 3);
		if (IS_ERR(vc->gfx) || IS_ERR(vc->pll) || IS_ERR(vc->gdiv) ||
		    dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(32))) {
			dev_warn(dev, "no gfx dma regs in the dt, PIO\n");
			dma = false;
		}
	}
	if (dma) {
		vc->push = alloc_pages_exact(VC_FB_SIZE, GFP_KERNEL | __GFP_ZERO);
		if (vc->push) {
			vc->push_phys = dma_map_single(dev, vc->push, VC_FB_SIZE,
						       DMA_TO_DEVICE);
			if (dma_mapping_error(dev, vc->push_phys)) {
				free_pages_exact(vc->push, VC_FB_SIZE);
				vc->push = NULL;
			}
		}
		if (!vc->push) {
			dev_warn(dev, "no contiguous push buffer, PIO\n");
			dma = false;
		}
	}
	if (!dma)
		vc->push = vzalloc(VC_FB_SIZE);
	if (!vc->push) {
		vfree(vc->fb);
		return -ENOMEM;
	}
	ret = vc_drm_init(vc);
	if (!ret)
		ret = vc_bl_register(vc);
	if (ret)
		goto err_fb;

	vc->thread = kthread_run(vc_thread, vc, "videocore");
	if (IS_ERR(vc->thread)) {
		ret = PTR_ERR(vc->thread);
		goto err_fb;
	}
	ret = drm_dev_register(&vc->drm, 0);
	if (ret)
		goto err_thread;
	/* fbcon keeps working through the drm fbdev emulation */
	drm_fbdev_shmem_setup(&vc->drm, 32);
	vc->misc.minor = MISC_DYNAMIC_MINOR;
	vc->misc.name = "vc-khrn";
	vc->misc.fops = &vc_khrn_fops;
	vc->misc.mode = 0666;
	if (misc_register(&vc->misc))
		dev_warn(dev, "no /dev/vc-khrn\n");
	return 0;

err_thread:
	kthread_stop(vc->thread);
err_fb:
	vc_push_free(vc);
	vfree(vc->fb);
	return ret;
}

static void rapuyama_videocore_remove(struct platform_device *pdev)
{
	struct vc *vc = platform_get_drvdata(pdev);

	if (vc->misc.this_device)
		misc_deregister(&vc->misc);
	drm_dev_unplug(&vc->drm);
	drm_atomic_helper_shutdown(&vc->drm);
	kthread_stop(vc->thread);
	vc_push_free(vc);
	vfree(vc->fb);
}

static void rapuyama_videocore_shutdown(struct platform_device *pdev)
{
	struct vc *vc = platform_get_drvdata(pdev);

	drm_atomic_helper_shutdown(&vc->drm);
}

static const struct of_device_id rapuyama_videocore_of_match[] = {
	{ .compatible = "nokia,rapuyama-videocore" },
	{ }
};
MODULE_DEVICE_TABLE(of, rapuyama_videocore_of_match);

static struct platform_driver rapuyama_videocore_driver = {
	.probe = rapuyama_videocore_probe,
	.remove = rapuyama_videocore_remove,
	.shutdown = rapuyama_videocore_shutdown,
	.driver = {
		.name = "rapuyama-videocore",
		.of_match_table = rapuyama_videocore_of_match,
	},
};
/* late: drivers/soc links before drivers/gpu, drm core is not up at module_init */
static int __init rapuyama_videocore_init(void)
{
	return platform_driver_register(&rapuyama_videocore_driver);
}
late_initcall(rapuyama_videocore_init);

static void __exit rapuyama_videocore_exit(void)
{
	platform_driver_unregister(&rapuyama_videocore_driver);
}
module_exit(rapuyama_videocore_exit);

MODULE_DESCRIPTION("Nokia RapuYama BCM2727 VideoCore display driver");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE("vc/vc-loader-a.bin");
MODULE_FIRMWARE("vc/vc-loader-a.trl");
MODULE_FIRMWARE("vc/vc-main.bin");
MODULE_FIRMWARE("vc/vc-main.trl");