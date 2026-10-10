/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Boot progress markers for bring-up on real calculators
 * (CONFIG_FXCG_DEBUG_MARKERS): coloured squares drawn straight into the
 * R61524 GRAM, at the bottom right of the screen, by code that runs
 * before the console exists. They stay visible until the console clears
 * the screen, so a hang shows how far the boot got.
 *
 * Only uses fixed P2 addresses: callable with the MMU off and from the
 * zImage decompressor.
 */
#ifndef __ASM_SH_MACH_FXCG_DEBUG_MARKER_H
#define __ASM_SH_MACH_FXCG_DEBUG_MARKER_H

#include <mach/fxcg.h>

#define FXCG_MARK_YELLOW	0xffe0
#define FXCG_MARK_GREEN		0x07e0
#define FXCG_MARK_RED		0xf800
#define FXCG_MARK_WHITE		0xffff

#define FXCG_MARK_SIZE		20
#define FXCG_MARK_STEP		24

static inline void fxcg_mark_reg(unsigned short reg, unsigned short val)
{
	volatile unsigned char *prdr = (void *)0xa405013c;
	volatile unsigned short *lcd = (void *)0xb4000000;

	*prdr &= ~0x10;		/* RS = 0: register number */
	*lcd = reg;
	*prdr |= 0x10;		/* RS = 1: data */
	*lcd = val;
}

/* Fill square number @slot (0 = rightmost) with an RGB565 colour. */
static inline void fxcg_debug_mark(int slot, unsigned short color)
{
	volatile unsigned short *lcd = (void *)0xb4000000;
	int x1 = FXCG_LCD_WIDTH - 1 - slot * FXCG_MARK_STEP;
	int x0 = x1 - FXCG_MARK_SIZE + 1;
	int y1 = FXCG_LCD_HEIGHT - 1, y0 = y1 - FXCG_MARK_SIZE + 1;
	int i;

	/* the Casio OS area is at panel H = 389 - x, V = y */
	fxcg_mark_reg(0x210, 389 - x1);
	fxcg_mark_reg(0x211, 389 - x0);
	fxcg_mark_reg(0x212, y0);
	fxcg_mark_reg(0x213, y1);
	fxcg_mark_reg(0x200, 0);
	fxcg_mark_reg(0x201, 0);
	fxcg_mark_reg(0x202, color);	/* first pixel; RS stays 1 */
	for (i = 1; i < FXCG_MARK_SIZE * FXCG_MARK_SIZE; i++)
		*lcd = color;
}

#endif /* __ASM_SH_MACH_FXCG_DEBUG_MARKER_H */
