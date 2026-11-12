/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _UAPI_LINUX_RAPUYAMA_MDI_H
#define _UAPI_LINUX_RAPUYAMA_MDI_H

#include <linux/ioctl.h>
#include <linux/types.h>

struct rapuyama_mdi_status {
	__u32 version;
	__u32 ap;
	__u32 dsp;
	__u32 rx_interrupts;
	__u32 tx_interrupts;
	__u32 event_interrupts;
};

#define RAPUYAMA_MDI_STATUS	_IOR('M', 0, struct rapuyama_mdi_status)
/* 0 off, 1 copied buffers (AP data + DSP window copies), 2 AP copies with the
 * DSP window mmap()ed directly (mmap the whole window at its firmware address) */
#define RAPUYAMA_MDI_BUFFERS	_IOW('M', 1, __u32)
/* copied buffers: write runtime memory into the real DSP window (buffers the
 * firmware hands to the DSP by address, e.g. the CELL_DCH uplink transport
 * blocks, which no descriptor names) */
struct rapuyama_mdi_put {
	__u32 offset;		/* in the DSP window */
	__u32 size;		/* 1..4096 */
	__u64 buffer;		/* user address of the bytes */
};
#define RAPUYAMA_MDI_DSP_PUT	_IOW('M', 2, struct rapuyama_mdi_put)
/* copied buffers: read the real DSP window into runtime memory (buffers the DSP
 * hands the firmware by address, e.g. HSDPA MAC-hs PDUs); size 1..8192 */
#define RAPUYAMA_MDI_DSP_GET	_IOW('M', 3, struct rapuyama_mdi_put)

#endif
