/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _UAPI_LINUX_RAPUYAMA_ENERGY_H
#define _UAPI_LINUX_RAPUYAMA_ENERGY_H
#include <linux/ioctl.h>
#include <linux/types.h>

#define RAPUYAMA_ENERGY_VERSION 3
#define RAPUYAMA_ENERGY_OWNED 1
#define RAPUYAMA_ENERGY_REVOKED 2
#define RAPUYAMA_ENERGY_THERMAL 4
#define RAPUYAMA_ENERGY_USB_BLOCKED 8
#define RAPUYAMA_ENERGY_DYNAMIC_BLOCKED 16

struct rapuyama_energy_status {
	__u32 version, flags, usb_budget_ua, reserved;
};
struct rapuyama_energy_io {
	__u32 address, reg, value, write;
};
struct rapuyama_energy_gpio {
	__u32 operation, value, reserved[2];
};
struct rapuyama_energy_pmic {
	__u32 field, value, write, reserved;
};
#define RAPUYAMA_ENERGY_GPIO_READ 0
#define RAPUYAMA_ENERGY_GPIO_CONFIG 1
#define RAPUYAMA_ENERGY_GPIO_TYPE 2
#define RAPUYAMA_ENERGY_GPIO_ACK 3
#define RAPUYAMA_ENERGY_GPIO_ENABLE 4
#define RAPUYAMA_ENERGY_GPIO_DISABLE 5
#define RAPUYAMA_ENERGY_GPIO_EVENT 6
#define RAPUYAMA_ENERGY_ACQUIRE _IOWR('E', 0x60, struct rapuyama_energy_status)
#define RAPUYAMA_ENERGY_STATUS _IOR('E', 0x61, struct rapuyama_energy_status)
#define RAPUYAMA_ENERGY_IO _IOWR('E', 0x62, struct rapuyama_energy_io)
#define RAPUYAMA_ENERGY_HEARTBEAT _IO('E', 0x63)
#define RAPUYAMA_ENERGY_RELEASE _IO('E', 0x64)
#define RAPUYAMA_ENERGY_GPIO _IOWR('E', 0x65, struct rapuyama_energy_gpio)
#define RAPUYAMA_ENERGY_PMIC _IOWR('E', 0x66, struct rapuyama_energy_pmic)
#endif