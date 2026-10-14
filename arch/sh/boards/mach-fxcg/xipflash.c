// SPDX-License-Identifier: GPL-2.0
/*
 * Casio fx-CG: install the flash part of a kernel built with
 * CONFIG_FXCG_ROM_TEXT, or remove it
 *
 *	cat /mnt/fls/flash.bin > /dev/xipflash	install (or update)
 *	echo -n fxcgRMV1 > /dev/xipflash	remove: erase it all
 *
 * The flash part goes to the two erased areas that the Casio OS neither
 * checks at boot nor uses for its storage memory, 0xc00000-0xc7ffff and
 * 0xb20000-0xb3ffff. flash.bin (fxcg20-Linux-bundle scripts/romkernel.py)
 * is a 64-byte index ("fxcgROMI", the count, the address and length of
 * each area's image, CRC-32) and the images; each image starts with a
 * 64-byte header ("fxcgROM1", address, length, the CRC-32 of the rest,
 * CRC-32 of the header).
 *
 * Nothing else is ever erased or programmed: an area may be written only
 * if it is erased or starts with such a header (an earlier install),
 * which is checked for both areas before the flash is touched. Each
 * image is written from its start, header first, so that an install
 * that is cut short leaves an area that is erased or marked as ours,
 * and installing again works. Closing the file checks the CRCs of what
 * the flash holds.
 *
 * This kernel runs from RAM (FXCG_ROM_TEXT is off): while the flash is
 * programmed it cannot be read.
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

#define FLASH		0x80000000UL	/* P1: cached view */
#define HDR		64
#define NR_AREAS	2

static const struct {
	unsigned long start, size;
} areas[NR_AREAS] = {
	{ 0x00c00000, 0x80000 },
	{ 0x00b20000, 0x20000 },
};

static bool crc_ok(const u8 *h)
{
	return ~crc32_le(~0, h, HDR - 4) == get_unaligned_be32(h + HDR - 4);
}

/* Does area i start with a header of ours (whatever is after it)? */
static bool area_ours(int i)
{
	const u8 *h = (const u8 *)(FLASH + areas[i].start);

	return !memcmp(h, "fxcgROM1", 8) && crc_ok(h) &&
	       get_unaligned_be32(h + 8) == areas[i].start;
}

static bool area_blank(int i)
{
	return fxcg_nor_blank(areas[i].start, areas[i].size);
}

/* Is what area i holds a complete image (header and CRC)? */
static bool area_intact(int i)
{
	const u8 *h = (const u8 *)(FLASH + areas[i].start);
	u32 len = get_unaligned_be32(h + 12);

	return area_ours(i) && len >= HDR && len <= areas[i].size &&
	       ~crc32_le(~0, h + HDR, len - HDR) == get_unaligned_be32(h + 16);
}

static int erase_area(int i)
{
	unsigned long s, end = areas[i].start + areas[i].size;
	int ret;

	for (s = areas[i].start; s < end; s += FXCG_FLASH_SECTOR)
		if (!fxcg_nor_blank(s, FXCG_FLASH_SECTOR)) {
			ret = fxcg_nor_erase(s, areas[i].start, end);
			if (ret)
				return ret;
		}
	return 0;
}

static struct {
	u8 index[HDR];			/* collecting the index */
	unsigned int nindex;
	bool started, done, failed;
	int area;			/* being written */
	unsigned long pos;		/* next address to program in it */
	unsigned long end;		/* end of its image */
	unsigned long len[NR_AREAS];
	u8 odd;				/* a byte waiting for its pair */
	bool has_odd;
	u8 *buf;
} st;

static DEFINE_MUTEX(xip_open_lock);

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
	return nonseekable_open(inode, file);
}

/* Both areas must be erased or ours, before anything is written. */
static int check_areas(void)
{
	int i;

	for (i = 0; i < NR_AREAS; i++)
		if (!area_blank(i) && !area_ours(i)) {
			pr_err("xipflash: the flash at %#lx is in use, not erased\n",
			       areas[i].start);
			return -ENOSPC;
		}
	return 0;
}

static int start_install(void)
{
	int i, ret;

	if (!crc_ok(st.index) ||
	    get_unaligned_be32(st.index + 8) != NR_AREAS) {
		pr_err("xipflash: not a flash image\n");
		return -EINVAL;
	}
	for (i = 0; i < NR_AREAS; i++) {
		if (get_unaligned_be32(st.index + 12 + 8 * i) != areas[i].start)
			return -EINVAL;
		st.len[i] = get_unaligned_be32(st.index + 16 + 8 * i);
		if (st.len[i] < HDR || st.len[i] > areas[i].size || st.len[i] & 1)
			return -EINVAL;
	}
	ret = check_areas();
	if (ret)
		return ret;
	pr_info("xipflash: writing %lu + %lu bytes\n", st.len[0], st.len[1]);
	st.started = true;
	st.area = -1;
	return 0;
}

/* Program n bytes (even), moving to the next area as each one fills. */
static int put(const u8 *p, unsigned long n)
{
	int ret;

	while (n) {
		unsigned long k;

		if (st.area < 0 || st.pos == st.end) {
			if (++st.area == NR_AREAS)
				return -EFBIG;
			ret = erase_area(st.area);
			if (ret)
				return ret;
			st.pos = areas[st.area].start;
			st.end = st.pos + st.len[st.area];
		}
		k = min(n, st.end - st.pos);
		ret = fxcg_nor_program(st.pos, p, k, areas[st.area].start,
				       areas[st.area].start + areas[st.area].size);
		if (ret)
			return ret;
		st.pos += k;
		p += k;
		n -= k;
	}
	return 0;
}

static int remove_all(void)
{
	int i, ret;

	for (i = 0; i < NR_AREAS; i++) {
		if (area_blank(i))
			continue;
		if (!area_ours(i)) {
			pr_warn("xipflash: the flash at %#lx is not ours, left alone\n",
				areas[i].start);
			continue;
		}
		ret = erase_area(i);
		if (ret)
			return ret;
	}
	pr_info("xipflash: removed\n");
	return 0;
}

static ssize_t xip_write(struct file *file, const char __user *ubuf,
			 size_t len, loff_t *ppos)
{
	size_t done = 0;
	int ret = 0;

	if (st.failed || st.done)
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
		if (!st.started) {
			unsigned int m = min_t(size_t, HDR - st.nindex, n);

			memcpy(st.index + st.nindex, p, m);
			st.nindex += m;
			p += m;
			n -= m;
			if (st.nindex >= 8 && !memcmp(st.index, "fxcgRMV1", 8)) {
				ret = remove_all();
				st.done = true;
				break;
			}
			if (st.nindex >= 8 && memcmp(st.index, "fxcgROMI", 8)) {
				pr_err("xipflash: not a flash image\n");
				ret = -EINVAL;
				break;
			}
			if (st.nindex < HDR)
				continue;
			ret = start_install();
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

/* At close: does the flash hold the whole image, intact? */
static int xip_flush(struct file *file, fl_owner_t id)
{
	int i, ret = 0;

	if (st.failed)
		return -EIO;	/* already said why */
	if (st.done || !st.nindex)
		return 0;
	if (!st.started || st.has_odd || st.area != NR_AREAS - 1 ||
	    st.pos != st.end) {
		pr_err("xipflash: incomplete image\n");
		return -EIO;
	}
	mutex_lock(&fxcg_flash_lock);
	for (i = 0; i < NR_AREAS; i++)
		if (!area_intact(i))
			ret = -EIO;
	mutex_unlock(&fxcg_flash_lock);
	if (ret)
		pr_err("xipflash: the flash does not check out\n");
	else
		pr_info("xipflash: installed\n");
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
