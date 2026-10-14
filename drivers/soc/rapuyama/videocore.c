// SPDX-License-Identifier: GPL-2.0-only
/*
 * Boots the VideoCore over the ViSSI host bus, runs VCHI on top of it,
 * brings the LCD panel up through the display command service (DSPC) and
 * exposes a 640x480 XRGB8888 fbdev. Frames are pushed into a dispman
 * resource with vc_bufmanx (service BFMX) and VCHI bulk transfers.
 * Everything here follows the Symbian drivers in the E6 ROM (vcdriver.dll,
 * vchi.dll, dispcommanddrv.dll, display_chipset, vcdispman.dll,
 * vc_bufmanx); the message sequences were captured by running that code in
 * an emulator, see vc-link-notes.md in rapuyama-linux.
 * The bus is driven by PIO from one kernel thread that owns it completely:
 * it boots the VC, sets the display up and then pushes dirty frames.
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/fb.h>
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/unaligned.h>
#include <linux/vmalloc.h>
#include <linux/wait.h>

/* host bus block (phys 0x20025800) */
#define VC_CTRL	0x10
#define VC_STAT	0x14 /* bit 8: bus cycle busy, bit 0: reset busy */
#define VC_DMACTL 0x40
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
#define VC_W 640
#define VC_H 480
#define VC_PITCH (VC_W * 4)
#define VC_FB_SIZE (VC_PITCH * VC_H)

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

#define VC_IMG_RES_TYPE	0x2b		/* FUN_80345920(4), XRGB8888 */
#define VC_IMG_BUF_TYPE	0x20008		/* FUN_80345974(4) */

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

/*
 * we always boot the VC from scratch, so the panel reads 0a = 1c (display
 * on) but is not driven: Symbian's warm path leaves it dark, the cold one
 * (power 0x41/0x40/0x81 + sleep out) brings it up
 */
static int cold = 1;
module_param(cold, int, 0444);
MODULE_PARM_DESC(cold, "panel power-up path: 1 = cold (default), 0 = warm, -1 = by DCS 0x0a like Symbian");

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

struct vc {
	struct device *dev;
	void __iomem *bus;
	struct gpio_desc *run;
	struct clk *clk;
	struct task_struct *thread;

	u32 last_status;
	bool rx_armed;

	/* VCHI */
	u8 rx[0x1100];
	u8 tx[VCHI_SLOT];
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
	unsigned int frames;

	/* fbdev */
	struct fb_info *info;
	void *fb;
	u32 pseudo_palette[16];
	bool dirty;
	wait_queue_head_t wq;
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
			if (time_after(jiffies, end))
				return first ? -EAGAIN : -ETIMEDOUT;
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
		end = jiffies + msecs_to_jiffies(timeout_ms);
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

	/*
	 * loader A acks (3/1) and then says hello (1); loader B, what Symbian's
	 * power-up path sends, answered with a hello straight away in vcprobe
	 */
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
		ret = vc_rx(vc, vc->rx, sizeof(vc->rx), &n, 20);
		if (!ret) {
			vchi_handle_slot(vc, n);
			vchi_answer_queries(vc);
		} else if (ret != -EAGAIN) {
			return ret;
		}
		if (time_after(jiffies, end))
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
	__le32 w[17];
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
static int bufm_write(struct vc *vc)
{
	u32 m[15] = { BUFM_WRITE_IMAGE, vc->resource, VC_IMG_BUF_TYPE, 0,
		      VC_W, VC_H, VC_PITCH, 0, 0, VC_W << 16, VC_H << 16,
		      0, 0, VC_W, VC_H };
	u32 status, nchunks, rows, csize, last, i, size, off;
	__le32 w[15];
	int ret;

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
	for (i = 0; i < nchunks; i++) {
		size = i + 1 == nchunks ? last : csize;
		off = VC_PITCH * rows * i;
		if (off + size > VC_FB_SIZE) {
			dev_err(vc->dev, "bufman chunk %u out of range\n", i);
			return -EINVAL;
		}
		ret = vchi_bulk_tx(vc, FCC_BFMX, vc->fb + off, size);
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
	a[0] = VC_IMG_RES_TYPE;
	a[1] = VC_W;
	a[2] = VC_H;
	ret = disp_cmd(vc, DISP_RES_CREATE, a, 3, true, &vc->resource);
	if (ret)
		return ret;
	if (!vc->resource) {
		dev_err(vc->dev, "ResourceCreate failed\n");
		return -ENOMEM;
	}
	ret = bufm_write(vc);
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
static int vc_display_update(struct vc *vc)
{
	static const struct dcs te_on[] = { { 0x35, 1, { 0x00 } } };
	static const struct dcs first[] = {
		{ 0x3a, 1, { 0x70 } },
		{ 0x35, 1, { 0x00 } },
		{ 0x34, 0 },
	};
	u32 a[6], update;
	int ret;

	if (vc->first_refresh)
		ret = dspc_batch(vc, first, ARRAY_SIZE(first));
	else
		ret = dspc_batch(vc, te_on, ARRAY_SIZE(te_on));
	vc->first_refresh = false;
	if (ret)
		return ret;
	ret = bufm_write(vc);
	if (ret)
		return ret;
	a[0] = 1;
	ret = disp_cmd(vc, DISP_UPDATE_START, a, 1, true, &update);
	if (ret)
		return ret;
	a[0] = update;
	a[1] = vc->element;
	a[2] = 0;
	a[3] = 0;
	a[4] = VC_W;
	a[5] = VC_H;
	ret = disp_cmd(vc, DISP_ELEMENT_MOD, a, 6, false, NULL);
	if (ret)
		return ret;
	a[0] = update;
	return disp_cmd(vc, DISP_SUBMIT_SYNC, a, 1, true, NULL);
}

/* ---- thread ---- */

/* 8 colour bars over a white/black split, 3 s, then the console back */
static void vc_test_pattern(struct vc *vc)
{
	static const u32 bars[8] = { 0xffffff, 0xffff00, 0x00ffff, 0x00ff00,
				     0xff00ff, 0xff0000, 0x0000ff, 0x000000 };
	u32 *fb = vc->fb, *save;
	unsigned int x, y;
	int ret;

	save = vmalloc(VC_FB_SIZE);
	if (!save)
		return;
	memcpy(save, fb, VC_FB_SIZE);
	for (y = 0; y < VC_H; y++)
		for (x = 0; x < VC_W; x++)
			fb[y * VC_W + x] = y < VC_H * 3 / 4 ?
				bars[x * 8 / VC_W] :
				(x < VC_W / 2 ? 0xffffff : 0);
	ret = vc_display_update(vc);
	if (!ret)
		vc->frames++;
	dev_info(vc->dev, "test pattern: %d\n", ret);
	vchi_pump(vc, 3000, NULL);
	memcpy(fb, save, VC_FB_SIZE);
	vfree(save);
	WRITE_ONCE(vc->dirty, true);
}

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
	if (test_pattern)
		vc_test_pattern(vc);

	while (!kthread_should_stop()) {
		wait_event_interruptible_timeout(vc->wq,
						 READ_ONCE(vc->dirty) ||
						 kthread_should_stop(),
						 msecs_to_jiffies(50));
		/* keep the VC's messages (and our rx credits) moving */
		vchi_pump(vc, 0, NULL);
		vchi_return_credits(vc);
		if (READ_ONCE(vc->dirty)) {
			WRITE_ONCE(vc->dirty, false);
			ret = vc_display_update(vc);
			if (!ret && vc->frames < UINT_MAX)
				vc->frames++;
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

/* ---- fbdev ---- */

static void vc_fb_dirty(struct fb_info *info)
{
	struct vc *vc = info->par;

	WRITE_ONCE(vc->dirty, true);
	wake_up(&vc->wq);
}

static void vc_fb_damage_range(struct fb_info *info, off_t off, size_t len)
{
	vc_fb_dirty(info);
}

static void vc_fb_damage_area(struct fb_info *info, u32 x, u32 y,
			      u32 width, u32 height)
{
	vc_fb_dirty(info);
}

FB_GEN_DEFAULT_DEFERRED_SYSMEM_OPS(vc_fb, vc_fb_damage_range,
				   vc_fb_damage_area)

static int vc_fb_setcolreg(unsigned int regno, unsigned int red,
			   unsigned int green, unsigned int blue,
			   unsigned int transp, struct fb_info *info)
{
	struct vc *vc = info->par;

	if (regno >= ARRAY_SIZE(vc->pseudo_palette))
		return -EINVAL;
	vc->pseudo_palette[regno] = (red >> 8) << 16 | (green >> 8) << 8 |
				    (blue >> 8);
	return 0;
}

static const struct fb_ops vc_fb_ops = {
	.owner		= THIS_MODULE,
	FB_DEFAULT_DEFERRED_OPS(vc_fb),
	.fb_setcolreg	= vc_fb_setcolreg,
};

static void vc_fb_deferred_io(struct fb_info *info, struct list_head *pages)
{
	vc_fb_dirty(info);
}

static struct fb_deferred_io vc_fb_defio = {
	.delay		= HZ / 20,
	.deferred_io	= vc_fb_deferred_io,
};

static int vc_fb_register(struct vc *vc)
{
	struct fb_info *info;
	int ret;

	info = framebuffer_alloc(0, vc->dev);
	if (!info)
		return -ENOMEM;
	info->par = vc;
	info->fbops = &vc_fb_ops;
	info->screen_buffer = vc->fb;
	info->pseudo_palette = vc->pseudo_palette;

	strscpy(info->fix.id, "rapuyama-vc", sizeof(info->fix.id));
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.visual = FB_VISUAL_TRUECOLOR;
	info->fix.line_length = VC_PITCH;
	info->fix.smem_len = VC_FB_SIZE;
	info->fix.accel = FB_ACCEL_NONE;

	info->var.xres = info->var.xres_virtual = VC_W;
	info->var.yres = info->var.yres_virtual = VC_H;
	info->var.bits_per_pixel = 32;
	info->var.red.offset = 16;
	info->var.red.length = 8;
	info->var.green.offset = 8;
	info->var.green.length = 8;
	info->var.blue.offset = 0;
	info->var.blue.length = 8;
	info->var.activate = FB_ACTIVATE_NOW;
	info->var.height = 50;		/* 2.46" VGA */
	info->var.width = 37;

	info->fbdefio = &vc_fb_defio;
	ret = fb_deferred_io_init(info);
	if (ret)
		goto err_release;
	ret = register_framebuffer(info);
	if (ret)
		goto err_defio;
	vc->info = info;
	return 0;

err_defio:
	fb_deferred_io_cleanup(info);
err_release:
	framebuffer_release(info);
	return ret;
}

static void vc_fb_unregister(struct vc *vc)
{
	if (!vc->info)
		return;
	unregister_framebuffer(vc->info);
	fb_deferred_io_cleanup(vc->info);
	framebuffer_release(vc->info);
	vc->info = NULL;
}

/* ---- platform ---- */

static int rapuyama_videocore_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct vc *vc;
	int ret;

	vc = devm_kzalloc(dev, sizeof(*vc), GFP_KERNEL);
	if (!vc)
		return -ENOMEM;
	vc->dev = dev;
	vc->dspc_token = 0x100;
	init_waitqueue_head(&vc->wq);
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

	vc->fb = vzalloc(VC_FB_SIZE);
	if (!vc->fb)
		return -ENOMEM;
	ret = vc_fb_register(vc);
	if (ret)
		goto err_fb;

	vc->thread = kthread_run(vc_thread, vc, "videocore");
	if (IS_ERR(vc->thread)) {
		ret = PTR_ERR(vc->thread);
		goto err_reg;
	}
	return 0;

err_reg:
	vc_fb_unregister(vc);
err_fb:
	vfree(vc->fb);
	return ret;
}

static void rapuyama_videocore_remove(struct platform_device *pdev)
{
	struct vc *vc = platform_get_drvdata(pdev);

	kthread_stop(vc->thread);
	vc_fb_unregister(vc);
	vfree(vc->fb);
}

static const struct of_device_id rapuyama_videocore_of_match[] = {
	{ .compatible = "nokia,rapuyama-videocore" },
	{ }
};
MODULE_DEVICE_TABLE(of, rapuyama_videocore_of_match);

static struct platform_driver rapuyama_videocore_driver = {
	.probe = rapuyama_videocore_probe,
	.remove = rapuyama_videocore_remove,
	.driver = {
		.name = "rapuyama-videocore",
		.of_match_table = rapuyama_videocore_of_match,
	},
};
module_platform_driver(rapuyama_videocore_driver);

MODULE_DESCRIPTION("Nokia RapuYama BCM2727 VideoCore display driver");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE("vc/vc-loader-a.bin");
MODULE_FIRMWARE("vc/vc-loader-a.trl");
MODULE_FIRMWARE("vc/vc-main.bin");
MODULE_FIRMWARE("vc/vc-main.trl");