/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __MACH_RAPUYAMA_LOWMEM_H
#define __MACH_RAPUYAMA_LOWMEM_H

void rapuyama_lowmem_reserve(void);
void rapuyama_swd_reserve(void);
void rapuyama_lowmem_init_early(void);
void rapuyama_lowmem_keep(phys_addr_t pa);

#endif