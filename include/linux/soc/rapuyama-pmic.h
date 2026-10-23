/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __LINUX_SOC_RAPUYAMA_PMIC_H
#define __LINUX_SOC_RAPUYAMA_PMIC_H

#include <linux/types.h>

/* gazoo pmic adc, raw 10 bit, hw channel 0..18 (drivers/watchdog/rapuyama_wdt.c) */
int rapuyama_pmic_adc(unsigned int ch, u16 *raw);

#endif