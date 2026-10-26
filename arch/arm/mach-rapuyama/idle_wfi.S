/* SPDX-License-Identifier: GPL-2.0-only */
#include <linux/linkage.h>

	.text
	.balign 32
/* scratch is uncached; l2_sync and clock are device mappings */
ENTRY(rapuyama_idle_wfi)
	push	{r4, r5}
	mov	r3, r2
	mov	r4, #0
	mcr	p15, 0, r4, c7, c10, 5
	str	r4, [r0]
	str	r4, [r1]
1:	ldr	r0, [r1]
	cmp	r0, #0
	bne	1b
	mrc	p15, 0, r5, c1, c0, 0
	bic	r0, r5, #0x800
	mcr	p15, 0, r0, c1, c0, 0
	adr	r0, .Lprefetch_start
	adr	r1, .Lprefetch_end
	bic	r0, r0, #31
	add	r1, r1, #32
	bic	r1, r1, #31
2:	mcr	p15, 0, r0, c7, c13, 1
	add	r0, r0, #32
	cmp	r0, r1
	ble	2b
.Lprefetch_start:
	ldr	r0, [r3, #0x130]
	ldr	r1, [r3, #0x120]
	.rept 4
	nop
	.endr
	mov	r0, #0
	mcr	p15, 0, r0, c7, c0, 4
	.rept 14
	nop
	.endr
.Lclock_delay:
	.rept 10
	nop
	.endr
	ldr	r0, [r3, #0x130]
	tst	r0, #0x400
	beq	.Lclock_delay
	.rept 10
	nop
	.endr
3:	ldr	r0, [r3, #0x120]
	tst	r0, #0xc000
	bne	3b
	mcr	p15, 0, r5, c1, c0, 0
.Lprefetch_end:
	pop	{r4, r5}
	bx	lr
ENDPROC(rapuyama_idle_wfi)