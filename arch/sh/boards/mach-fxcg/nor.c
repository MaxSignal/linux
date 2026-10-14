// SPDX-License-Identifier: GPL-2.0
/*
 * Casio fx-CG: programming and erasing the NOR flash
 *
 * The flash takes the standard AMD/JEDEC command set (16-bit bus): the
 * unlock cycles 0xaa at byte address 0xaaa and 0x55 at 0x554, then 0xa0
 * and the word for a program, 0x80, the unlock cycles again and 0x30 at
 * the sector for a sector erase (128 KiB); the embedded algorithm is done
 * when DQ6 stops toggling. While it runs, the whole flash reads as
 * status, so nothing may read it meanwhile: callers hold fxcg_flash_lock,
 * which readers of the flash take too, and each program or erase runs
 * with interrupts off in code that waits without calling anything, as
 * part of the kernel may run from the flash (this file stays in RAM).
 *
 * Every call names the range it is allowed to touch and is refused
 * outside it.
 */
#include <linux/io.h>
#include <linux/irqflags.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/unaligned.h>
#include <asm/cacheflush.h>
#include <mach/fxcg.h>

#define FLASH		0x80000000UL	/* P1: cached view of the NOR flash */
#define FLASH_UC	0xa0000000UL	/* P2: uncached, for the commands */

/*
 * Polls of DQ6 (two reads each, under a microsecond) before giving up: a
 * word takes some 10 us to program, a sector about a second to erase.
 */
#define POLLS_PROGRAM	200000
#define POLLS_ERASE	50000000

static __always_inline void nor_unlock(void)
{
	__raw_writew(0xaa, FLASH_UC + 0xaaa);
	__raw_writew(0x55, FLASH_UC + 0x554);
}

/*
 * Wait until the program or erase algorithm at off stops toggling DQ6.
 * Interrupts are off: no jiffies, no calls.
 */
static __always_inline int nor_wait(unsigned long off, unsigned long polls)
{
	u16 a, b;

	while (polls--) {
		a = __raw_readw(FLASH_UC + off);
		b = __raw_readw(FLASH_UC + off);
		if (!((a ^ b) & 0x40))
			return 0;
		if (b & 0x20) {		/* DQ5: over the time limit */
			a = __raw_readw(FLASH_UC + off);
			b = __raw_readw(FLASH_UC + off);
			if (!((a ^ b) & 0x40))
				return 0;
			break;
		}
	}
	__raw_writew(0xf0, FLASH_UC);	/* reset: back to the array */
	return -EIO;
}

static bool nor_allowed(unsigned long off, unsigned long len,
			unsigned long lo, unsigned long hi)
{
	return off >= lo && off <= hi && len <= hi - off &&
	       !(off & 1) && !(len & 1);
}

bool fxcg_nor_blank(unsigned long off, unsigned long len)
{
	return !memchr_inv((const void *)(FLASH + off), 0xff, len);
}

/* Program len bytes at off (erased there) within [lo, hi). */
int fxcg_nor_program(unsigned long off, const void *src, unsigned long len,
		     unsigned long lo, unsigned long hi)
{
	unsigned long i, flags;
	int ret = 0;

	lockdep_assert_held(&fxcg_flash_lock);
	if (WARN_ON(!nor_allowed(off, len, lo, hi)))
		return -EINVAL;
	for (i = 0; i < len && !ret; i += 2) {
		u16 w = get_unaligned_be16(src + i);

		if (w == 0xffff)
			continue;
		if ((__raw_readw(FLASH_UC + off + i) & w) != w) {
			ret = -EIO;	/* would need a 0 -> 1 */
			break;
		}
		local_irq_save(flags);
		nor_unlock();
		__raw_writew(0xa0, FLASH_UC + 0xaaa);
		__raw_writew(w, FLASH_UC + off + i);
		ret = nor_wait(off + i, POLLS_PROGRAM);
		local_irq_restore(flags);
		if (!ret && __raw_readw(FLASH_UC + off + i) != w)
			ret = -EIO;
	}
	__flush_invalidate_region((void *)(FLASH + off), len);
	if (ret)
		pr_err("fxcg: programming the flash at %#lx failed\n", off + i);
	return ret;
}

/* Erase the 128 KiB sector at off within [lo, hi). */
int fxcg_nor_erase(unsigned long off, unsigned long lo, unsigned long hi)
{
	unsigned long flags;
	int ret;

	lockdep_assert_held(&fxcg_flash_lock);
	if (WARN_ON(!nor_allowed(off, FXCG_FLASH_SECTOR, lo, hi) ||
		    off % FXCG_FLASH_SECTOR))
		return -EINVAL;
	local_irq_save(flags);
	nor_unlock();
	__raw_writew(0x80, FLASH_UC + 0xaaa);
	nor_unlock();
	__raw_writew(0x30, FLASH_UC + off);
	ret = nor_wait(off, POLLS_ERASE);
	local_irq_restore(flags);
	__flush_invalidate_region((void *)(FLASH + off), FXCG_FLASH_SECTOR);
	if (!ret && !fxcg_nor_blank(off, FXCG_FLASH_SECTOR))
		ret = -EIO;
	if (ret)
		pr_err("fxcg: erasing the flash sector at %#lx failed\n", off);
	return ret;
}
