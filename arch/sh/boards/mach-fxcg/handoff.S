/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Casio fx-CG: hand the machine over to another kernel, as the loader's
 * trampoline does (fxcg-linux-loader): copied to the on-chip IL memory
 * and entered there with r4 = the entry point. It silences the interrupt
 * controller, the timers and the watchdog, drops the caches (dirty lines
 * included: nothing of this kernel is needed any more) and jumps.
 * Position independent: PC-relative literals only, no calls, no stack.
 */
#include <linux/linkage.h>

	.text
	.balign	4
ENTRY(fxcg_handoff_start)
	mov	r4, r8

	mov.l	.Lsr, r0		! MD=1 RB=0 BL=1 IMASK=15
	ldc	r0, sr

	! INTC: every IPR field to 0, every IMR bit set
	mov.l	.Lintc, r1
	mov	#12, r2
	mov	#0, r0
1:	mov.w	r0, @r1
	dt	r2
	bf/s	1b
	 add	#4, r1
	mov.l	.Limr, r1
	mov	#13, r2
	mov	#-1, r0
2:	mov.b	r0, @r1
	dt	r2
	bf/s	2b
	 add	#4, r1

	! stop TMU0-2 and the RCLK watchdog
	mov.l	.Ltstr, r1
	mov	#0, r0
	mov.b	r0, @r1
	mov.l	.Lrwtcsr, r1
	mov.w	.Lrwtstop, r0
	mov.w	r0, @r1

	! invalidate the operand and instruction caches, keep them on
	mov.l	.Lccr, r1
	mov.l	@r1, r0
	mov.w	.Lccrinv, r2
	or	r2, r0
	mov.l	r0, @r1
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop

	jmp	@r8
	 nop

	.balign	4
.Lsr:		.long	0x500000f0
.Lintc:		.long	0xa4080000
.Limr:		.long	0xa4080080
.Ltstr:		.long	0xa4490004
.Lrwtcsr:	.long	0xa4520004
.Lccr:		.long	0xff00001c
.Lrwtstop:	.word	0xa500
.Lccrinv:	.word	0x0808
	.balign	4
ENTRY(fxcg_handoff_end)
