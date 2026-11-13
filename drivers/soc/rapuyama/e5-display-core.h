/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Nokia E5 display: pure pixel and band helpers. No hardware access, so the
 * same header is built into the kernel driver and host-tested on the PC
 * (linux/e5-display/tests).
 */
#ifndef E5_DISPLAY_CORE_H
#define E5_DISPLAY_CORE_H

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/string.h>
#endif

#define E5D_HEIGHT	240

/* XRGB8888 -> RGB565, truncating (same as the E6 vc_565) */
static inline u16 e5d_xrgb_to_565(u32 xrgb)
{
	return (u16)(((xrgb >> 8) & 0xf800) | ((xrgb >> 5) & 0x07e0) |
		     ((xrgb >> 3) & 0x001f));
}

/* RGB565 -> XRGB8888, replicating the top bits into the low ones */
static inline u32 e5d_565_to_xrgb(u16 p)
{
	u32 r5 = (p >> 11) & 0x1f, g6 = (p >> 5) & 0x3f, b5 = p & 0x1f;
	u32 r8 = r5 << 3 | r5 >> 2, g8 = g6 << 2 | g6 >> 4, b8 = b5 << 3 | b5 >> 2;

	return r8 << 16 | g8 << 8 | b8;
}

/* A dirty row range [y0, y1); empty when y0 >= y1. */
struct e5d_band {
	u32 y0, y1;
};

static inline void e5d_band_add(struct e5d_band *b, u32 y0, u32 y1)
{
	if (y0 > E5D_HEIGHT)
		y0 = E5D_HEIGHT;
	if (y1 > E5D_HEIGHT)
		y1 = E5D_HEIGHT;
	if (y0 >= y1)
		return;
	if (b->y0 >= b->y1) {
		b->y0 = y0;
		b->y1 = y1;
		return;
	}
	if (y0 < b->y0)
		b->y0 = y0;
	if (y1 > b->y1)
		b->y1 = y1;
}

/* Take the band and empty it; false when it was empty. */
static inline bool e5d_band_take(struct e5d_band *b, u32 *y0, u32 *y1)
{
	if (b->y0 >= b->y1)
		return false;
	*y0 = b->y0;
	*y1 = b->y1;
	b->y0 = b->y1 = 0;
	return true;
}

/* Read one pixel (cpp 2 = RGB565, 4 = XRGB8888) as XRGB8888. */
static inline u32 e5d_read_px(const u8 *p, u32 cpp)
{
	u16 v16;
	u32 v32;

	if (cpp == 2) {
		memcpy(&v16, p, 2);
		return e5d_565_to_xrgb(v16);
	}
	memcpy(&v32, p, 4);
	return v32;
}

/* Write an XRGB8888 pixel in the destination format. */
static inline void e5d_write_px(u8 *p, u32 cpp, u32 xrgb)
{
	if (cpp == 2) {
		u16 v16 = e5d_xrgb_to_565(xrgb);

		memcpy(p, &v16, 2);
	} else {
		memcpy(p, &xrgb, 4);
	}
}

/*
 * Convert src rows y0..y1, columns x0..x1, into the dst format. Each row is
 * built in row_tmp (at least (x1 - x0) * dst_cpp bytes) and written to dst
 * only when it differs; the rows written are added to *changed.
 */
static inline void e5d_copy_rows(u8 *dst, u32 dst_pitch, u32 dst_cpp,
				 const u8 *src, u32 src_pitch, u32 src_cpp,
				 u32 x0, u32 x1, u32 y0, u32 y1,
				 u8 *row_tmp, struct e5d_band *changed)
{
	u32 y, x, len;

	if (x0 >= x1)
		return;
	len = (x1 - x0) * dst_cpp;
	for (y = y0; y < y1; y++) {
		const u8 *s = src + (size_t)y * src_pitch + (size_t)x0 * src_cpp;
		u8 *d = dst + (size_t)y * dst_pitch + (size_t)x0 * dst_cpp;

		for (x = 0; x < x1 - x0; x++)
			e5d_write_px(row_tmp + x * dst_cpp, dst_cpp,
				     e5d_read_px(s + x * src_cpp, src_cpp));
		if (memcmp(d, row_tmp, len)) {
			memcpy(d, row_tmp, len);
			e5d_band_add(changed, y, y + 1);
		}
	}
}

#endif /* E5_DISPLAY_CORE_H */
