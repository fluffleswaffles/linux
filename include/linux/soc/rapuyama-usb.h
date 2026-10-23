/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __LINUX_SOC_RAPUYAMA_USB_H
#define __LINUX_SOC_RAPUYAMA_USB_H

#include <linux/kconfig.h>

#if IS_REACHABLE(CONFIG_USB_MUSB_RAPUYAMA)
unsigned int rapuyama_usb_current_ma(void);
#else
static inline unsigned int rapuyama_usb_current_ma(void) { return 0; }
#endif

#endif