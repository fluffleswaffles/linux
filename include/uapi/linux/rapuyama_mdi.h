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

#endif
