/* SPDX-License-Identifier: GPL-2.0 */
#include <linux/linkage.h>

	.text
	.balign 4096
ENTRY(rapuyama_swd_call)
	push	{r4-r11, lr}
	mov	r4, r0
	mrs	r0, cpsr
	str	r0, [r4, #28]
	cpsid	if
	mrc	p15, 0, r0, c1, c0, 0
	str	r0, [r4, #92]
	mrc	p15, 0, r0, c2, c0, 0
	str	r0, [r4, #0]
	mrc	p15, 0, r0, c2, c0, 2
	str	r0, [r4, #4]
	mrc	p15, 0, r0, c3, c0, 0
	str	r0, [r4, #8]
	mrc	p15, 0, r0, c13, c0, 1
	str	r0, [r4, #12]
	mrc	p15, 0, r0, c10, c2, 0
	str	r0, [r4, #16]
	mrc	p15, 0, r0, c10, c2, 1
	str	r0, [r4, #20]
	str	sp, [r4, #24]
	mov	r0, #0
	mcr	p15, 0, r0, c7, c10, 4
	mvn	r0, #0
	mcr	p15, 0, r0, c3, c0, 0
	ldr	r0, [r4, #40]
	mcr	p15, 0, r0, c10, c2, 0
	ldr	r0, [r4, #44]
	mcr	p15, 0, r0, c10, c2, 1
	ldr	r0, [r4, #36]
	mcr	p15, 0, r0, c13, c0, 1
	mov	r0, #0
	mcr	p15, 0, r0, c2, c0, 2
	ldr	r0, [r4, #32]
	mcr	p15, 0, r0, c2, c0, 0
	ldr	r0, [r4, #96]
	mcr	p15, 0, r0, c1, c0, 0
	mov	r0, #0
	mcr	p15, 0, r0, c7, c5, 4
	mcr	p15, 0, r0, c8, c7, 0
	mcr	p15, 0, r0, c7, c5, 6
	mcr	p15, 0, r0, c7, c10, 4
	mcr	p15, 0, r0, c7, c5, 4
	ldr	sp, [r4, #48]
	mov	r0, #10
	str	r0, [r4, #88]
	mov	r0, #8
	ldr	r3, [r4, #52]
	blx	r3
	str	r0, [r4, #80]
	cmp	r0, #0
	beq	.Lrestore
	mov	r0, #8
	ldr	r3, [r4, #52]
	blx	r3
	str	r0, [r4, #84]
	cmp	r0, #0
	beq	.Lfree_request
	mov	r1, r0
	ldr	r0, [r4, #80]
	ldr	r2, [r4, #64]
	strh	r2, [r0, #4]
	mov	r2, #30
	str	r2, [r4, #88]
	ldr	r3, [r4, #60]
	blx	r3
	str	r0, [r4, #68]
	ldr	r1, [r4, #84]
	ldmia	r1, {r0, r2}
	str	r0, [r4, #72]
	str	r2, [r4, #76]
	mov	r0, #40
	str	r0, [r4, #88]
	ldr	r0, [r4, #84]
	ldr	r3, [r4, #56]
	blx	r3
.Lfree_request:
	ldr	r0, [r4, #80]
	ldr	r3, [r4, #56]
	blx	r3
.Lrestore:
	mov	r0, #0
	mcr	p15, 0, r0, c7, c10, 4
	ldr	r0, [r4, #8]
	mcr	p15, 0, r0, c3, c0, 0
	ldr	r0, [r4, #16]
	mcr	p15, 0, r0, c10, c2, 0
	ldr	r0, [r4, #20]
	mcr	p15, 0, r0, c10, c2, 1
	ldr	r0, [r4, #12]
	mcr	p15, 0, r0, c13, c0, 1
	ldr	r0, [r4, #4]
	mcr	p15, 0, r0, c2, c0, 2
	ldr	r0, [r4, #0]
	mcr	p15, 0, r0, c2, c0, 0
	ldr	r0, [r4, #92]
	mcr	p15, 0, r0, c1, c0, 0
	mov	r0, #0
	mcr	p15, 0, r0, c7, c5, 4
	mcr	p15, 0, r0, c8, c7, 0
	mcr	p15, 0, r0, c7, c5, 6
	mcr	p15, 0, r0, c7, c10, 4
	mcr	p15, 0, r0, c7, c5, 4
	ldr	sp, [r4, #24]
	ldr	r0, [r4, #28]
	msr	cpsr_c, r0
	mov	r0, #60
	str	r0, [r4, #88]
	pop	{r4-r11, pc}
ENDPROC(rapuyama_swd_call)
