/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __ASM_SH_MACH_FXCG_H
#define __ASM_SH_MACH_FXCG_H

/* Area of the 396x224 R61524 panel that the Casio OS uses */
#define FXCG_LCD_WIDTH		384
#define FXCG_LCD_HEIGHT		216

void fxcg_console_init(void);

/* the NOR flash */
#define FXCG_FLASH_SECTOR	0x20000		/* erase unit */

struct mutex;
/* held to read the flash while it may be programmed, and to program it */
extern struct mutex fxcg_flash_lock;

bool fxcg_nor_blank(unsigned long off, unsigned long len);
int fxcg_nor_program(unsigned long off, const void *src, unsigned long len,
		     unsigned long lo, unsigned long hi);
int fxcg_nor_erase(unsigned long off, unsigned long lo, unsigned long hi);

#endif /* __ASM_SH_MACH_FXCG_H */
