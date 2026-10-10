// SPDX-License-Identifier: GPL-2.0
/*
 * Casio fx-CG: install the kernel that runs in place, and its romfs, in
 * the free flash below 0xb40000, remove it, or start it
 *
 *	cat /mnt/fls/xip.bin > /dev/xipflash	install (or update)
 *	echo -n fxcgRMV1 > /dev/xipflash	remove: erase all of it
 *	echo -n fxcgBOOT > /dev/xipflash	start it, in place of this kernel
 *
 * xip.bin (scripts/mkxip.py in fxcg20-Linux-bundle) is the flash
 * from a sector boundary up to 0xb40000: a copy of the 64-byte header
 * ("fxcgXIP1", see xip.c), the romfs 256 bytes further, the kernel, and
 * the header again in the last 256 bytes, where the loader and the XIP
 * kernel read it. Writing streams it in place: each sector is erased
 * when the data reaches it, then programmed. Closing the file checks the
 * header and the CRCs of what is in the flash.
 *
 * Only these sectors may be erased: those already erased, and those of
 * the image the flash holds now, as either of its two headers describes
 * it. Anything else (the Casio OS) makes the write fail before the flash
 * is touched. Sectors are written in increasing order, so the old header
 * at the top goes last and the new one at the bottom comes first: a
 * reset at any point leaves one of them describing what is there, and
 * installing again works. Sectors of a bigger image installed before are
 * erased once the new one checks out, and removing erases every sector
 * of the image: the flash is then erased again, as before the install.
 *
 * The kernel doing this runs from RAM (not CONFIG_XIP_KERNEL): while the
 * flash is programmed it cannot be read.
 */
#include <linux/crc32.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/unaligned.h>
#include <mach/fxcg.h>

#define XIP_END		0x00b40000UL	/* the language data starts here */
#define XIP_HEADER	(XIP_END - 0x100)
#define XIP_LOWEST	0x00600000UL	/* never below, whatever the flash says */
#define HEADER_SIZE	64
#define FLASH		0x80000000UL
#define IL_MEM		0xe5200000UL	/* on-chip, for the hand-over */

extern const u8 fxcg_handoff_start[], fxcg_handoff_end[];

struct xip_header {
	u32 kstart, ksize, kcrc, rstart, rsize, rcrc, entry;
};

static bool parse_header(const u8 *h, struct xip_header *x)
{
	if (memcmp(h, "fxcgXIP1", 8) || get_unaligned_be32(h + 8) != HEADER_SIZE ||
	    ~crc32_le(~0, h, 60) != get_unaligned_be32(h + 60))
		return false;
	x->kstart = get_unaligned_be32(h + 12);
	x->ksize = get_unaligned_be32(h + 16);
	x->kcrc = get_unaligned_be32(h + 20);
	x->rstart = get_unaligned_be32(h + 24);
	x->rsize = get_unaligned_be32(h + 28);
	x->rcrc = get_unaligned_be32(h + 32);
	x->entry = get_unaligned_be32(h + 36);
	/* the image starts 256 bytes before the romfs, on a sector */
	return x->rstart >= XIP_LOWEST + 0x100 &&
	       (x->rstart - 0x100) % FXCG_FLASH_SECTOR == 0 &&
	       x->rstart + x->rsize <= x->kstart &&
	       x->kstart % FXCG_FLASH_SECTOR == 0 &&
	       x->kstart + x->ksize <= XIP_HEADER &&
	       x->entry >= x->kstart && x->entry < x->kstart + x->ksize;
}

static unsigned long image_start(const struct xip_header *x)
{
	return x->rstart - 0x100;
}

static struct {
	unsigned long start;		/* of the image being written */
	unsigned long pos;		/* next flash address to program */
	unsigned long erased;		/* sectors below this one are done */
	unsigned long old_lo;		/* the image there was: [old_lo, END) */
	u8 head[HEADER_SIZE];		/* collecting the first header */
	unsigned int nhead;
	bool removed;
	u8 odd;				/* a byte waiting for its pair */
	bool has_odd, failed;
	u8 *buf;
} st;

static DEFINE_MUTEX(xip_open_lock);

/* Where the image in the flash now begins (END if there is none). */
static unsigned long current_image(void)
{
	struct xip_header x;
	unsigned long lo = XIP_END;

	if (parse_header((const u8 *)(FLASH + XIP_HEADER), &x))
		lo = image_start(&x);
	/* an install that stopped half way: its first header */
	for (unsigned long s = XIP_LOWEST; s < XIP_END; s += FXCG_FLASH_SECTOR)
		if (parse_header((const u8 *)(FLASH + s), &x) &&
		    image_start(&x) == s && s < lo)
			lo = s;
	return lo;
}

static int xip_open(struct inode *inode, struct file *file)
{
	if (!(file->f_mode & FMODE_WRITE) || (file->f_mode & FMODE_READ))
		return -EINVAL;
	if (!mutex_trylock(&xip_open_lock))
		return -EBUSY;
	memset(&st, 0, sizeof(st));
	st.buf = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!st.buf) {
		mutex_unlock(&xip_open_lock);
		return -ENOMEM;
	}
	mutex_lock(&fxcg_flash_lock);
	st.old_lo = current_image();
	mutex_unlock(&fxcg_flash_lock);
	return nonseekable_open(inode, file);
}

/* The first header is in: check where the image goes before anything. */
static int start_image(void)
{
	struct xip_header x;
	unsigned long s;

	if (!parse_header(st.head, &x)) {
		pr_err("xipflash: not an XIP image\n");
		return -EINVAL;
	}
	for (s = image_start(&x); s < XIP_END; s += FXCG_FLASH_SECTOR)
		if (s < st.old_lo && !fxcg_nor_blank(s, FXCG_FLASH_SECTOR)) {
			pr_err("xipflash: the flash at %#lx is in use, not erased\n", s);
			return -ENOSPC;
		}
	st.start = st.pos = st.erased = image_start(&x);
	pr_info("xipflash: writing %lu KiB at %#lx\n",
		(XIP_END - st.start) >> 10, st.start);
	return 0;
}

/* Program n bytes (even) at st.pos, erasing sectors as it gets to them. */
static int put(const u8 *p, unsigned long n)
{
	int ret;

	if (n > XIP_END - st.pos)
		return -EFBIG;
	while (st.erased < st.pos + n) {
		if (!fxcg_nor_blank(st.erased, FXCG_FLASH_SECTOR)) {
			ret = fxcg_nor_erase(st.erased, st.start, XIP_END);
			if (ret)
				return ret;
		}
		st.erased += FXCG_FLASH_SECTOR;
	}
	ret = fxcg_nor_program(st.pos, p, n, st.start, XIP_END);
	if (!ret)
		st.pos += n;
	return ret;
}

/* Erase the sectors in [lo, hi) that are not erased (all ours). */
static int erase_range(unsigned long lo, unsigned long hi)
{
	int ret;

	for (; lo < hi; lo += FXCG_FLASH_SECTOR)
		if (!fxcg_nor_blank(lo, FXCG_FLASH_SECTOR)) {
			ret = fxcg_nor_erase(lo, lo, XIP_END);
			if (ret)
				return ret;
		}
	return 0;
}

static int remove_image(void)
{
	int ret;

	if (st.old_lo == XIP_END) {
		pr_info("xipflash: nothing to remove\n");
		st.removed = true;
		return 0;
	}
	ret = erase_range(st.old_lo, XIP_END);
	if (ret)
		return ret;
	pr_info("xipflash: removed, %#lx-%#lx erased\n", st.old_lo, XIP_END);
	st.removed = true;
	return 0;
}

/* The kernel the flash holds, checked, or 0. */
static unsigned long installed_entry(void)
{
	struct xip_header x;

	if (!parse_header((const u8 *)(FLASH + XIP_HEADER), &x) ||
	    ~crc32_le(~0, (const u8 *)(FLASH + x.kstart), x.ksize) != x.kcrc ||
	    ~crc32_le(~0, (const u8 *)(FLASH + x.rstart), x.rsize) != x.rcrc)
		return 0;
	return x.entry;
}

/* Start the kernel in the flash: no return if it is there. */
static int boot_image(void)
{
	unsigned long entry = installed_entry();
	void (*handoff)(unsigned long);

	if (!entry) {
		pr_err("xipflash: no kernel in the flash to start\n");
		return -ENOENT;
	}
	pr_info("xipflash: starting the kernel at %#lx\n", entry);
	local_irq_disable();
	memcpy((void *)IL_MEM, fxcg_handoff_start,
	       fxcg_handoff_end - fxcg_handoff_start);
	handoff = (void (*)(unsigned long))IL_MEM;
	handoff(entry);
	unreachable();
}

static ssize_t xip_write(struct file *file, const char __user *ubuf,
			 size_t len, loff_t *ppos)
{
	size_t done = 0;
	int ret = 0;

	if (st.failed || st.removed)
		return -EIO;
	mutex_lock(&fxcg_flash_lock);
	while (done < len && !ret) {
		/* st.buf[0] is kept free for a byte left over from before */
		size_t n = min_t(size_t, len - done, PAGE_SIZE - 1);
		u8 *p = st.buf + 1;

		if (copy_from_user(p, ubuf + done, n)) {
			ret = -EFAULT;
			break;
		}
		done += n;
		if (!st.start) {	/* the first header, before anything */
			unsigned int m = min_t(size_t, HEADER_SIZE - st.nhead, n);

			memcpy(st.head + st.nhead, p, m);
			st.nhead += m;
			p += m;
			n -= m;
			if (st.nhead >= 8 && !memcmp(st.head, "fxcgRMV1", 8)) {
				ret = remove_image();
				break;
			}
			if (st.nhead >= 8 && !memcmp(st.head, "fxcgBOOT", 8)) {
				ret = boot_image();	/* returns if it cannot */
				break;
			}
			if (st.nhead < HEADER_SIZE)
				continue;
			ret = start_image() ?: put(st.head, HEADER_SIZE);
			if (ret)
				break;
		}
		if (st.has_odd) {
			*--p = st.odd;
			n++;
			st.has_odd = false;
		}
		if (n & 1) {
			st.odd = p[--n];
			st.has_odd = true;
		}
		if (n)
			ret = put(p, n);
	}
	if (ret)
		st.failed = true;
	mutex_unlock(&fxcg_flash_lock);
	return ret ? ret : done;
}

/* At close: is what the flash holds now the whole image, intact? */
static int xip_flush(struct file *file, fl_owner_t id)
{
	struct xip_header x;
	int ret = 0;

	if ((!st.nhead && !st.failed) || st.removed)
		return 0;	/* nothing written through this file (yet) */
	if (st.failed)
		return -EIO;	/* already said why */
	if (st.has_odd || !st.start || st.pos != XIP_END) {
		pr_err("xipflash: incomplete image (up to %#lx of %#lx)\n",
		       st.pos, XIP_END);
		return -EIO;
	}
	mutex_lock(&fxcg_flash_lock);
	if (!installed_entry() ||
	    !parse_header((const u8 *)(FLASH + XIP_HEADER), &x) ||
	    image_start(&x) != st.start)
		ret = -EIO;
	/* a bigger image there was: its sectors below this one */
	if (!ret && st.old_lo < st.start)
		ret = erase_range(st.old_lo, st.start);
	mutex_unlock(&fxcg_flash_lock);
	if (ret)
		pr_err("xipflash: the image in the flash does not check out\n");
	else
		pr_info("xipflash: kernel at %#x, romfs at %#x: installed\n",
			x.kstart, x.rstart);
	return ret;
}

static int xip_release(struct inode *inode, struct file *file)
{
	kfree(st.buf);
	st.buf = NULL;
	mutex_unlock(&xip_open_lock);
	return 0;
}

static const struct file_operations xip_fops = {
	.owner		= THIS_MODULE,
	.open		= xip_open,
	.write		= xip_write,
	.flush		= xip_flush,
	.release	= xip_release,
};

static struct miscdevice xip_dev = {
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= "xipflash",
	.fops		= &xip_fops,
	.mode		= 0200,
};

static int __init xip_flash_init(void)
{
	return misc_register(&xip_dev);
}
device_initcall(xip_flash_init);
