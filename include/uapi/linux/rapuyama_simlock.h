/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _UAPI_LINUX_RAPUYAMA_SIMLOCK_H
#define _UAPI_LINUX_RAPUYAMA_SIMLOCK_H
#include <linux/ioctl.h>
#include <linux/types.h>

struct rapuyama_simlock {
	__u8 request[0x1c8];
	__u32 data_length;
	__aligned_u64 data;
	__u32 transport;
	__u8 response[0x1c];
};
#define RAPUYAMA_SIMLOCK_CHECK _IOWR('R', 0x30, struct rapuyama_simlock)
#endif