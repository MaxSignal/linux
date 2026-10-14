// SPDX-License-Identifier: GPL-2.0
/*
 * Casio fx-CG10/fx-CG20 storage memory file system ("fls")
 *
 * The Casio OS keeps its storage memory (\\fls0, the FAT16 disk that the
 * calculator also offers over USB) in the NOR flash behind its own flash
 * translation layer:
 *
 *  - the storage area runs from 0xc80000 to the end of the flash in 128
 *    KiB sectors. A sector in use starts with a 4 KiB header page whose
 *    first 24 bytes are three copies of { ~seq, seq }, seq numbering the
 *    sectors in the order they were taken into use; then 30 slots of 4 KiB
 *    and 64 records of 64 bytes from offset 0x1f000. Record n + 1
 *    describes slot n; only the newest sector is ever partly filled;
 *  - a record: type (0x22 header page, 0x11 data, 0x88 erase log), 0x0f,
 *    a big-endian counter, the logical 4 KiB page in the slot, the
 *    CRC-16/CCITT-FALSE of the slot, a 3-byte Hamming code per 512 bytes
 *    of it, 5a 5a 5a, and a Hamming code over bytes 0-33 of the record;
 *  - the counter of a page is 0xfffffffe for its first copy and one less
 *    for each new one: the lowest is the valid copy, a page without a
 *    copy reads as 0xff. Before a sector is erased, an erase log record
 *    (the sector's number, and from offset 0x200 how many times each
 *    sector was erased) goes into the sector being filled.
 *
 * At mount the driver rebuilds the page map from the records and reads
 * the FAT16 partition of that disk:
 *
 *	mount -t fls fls /mnt/fls	(-o ro: never written)
 *
 * A new copy of a 4 KiB page goes into the next free slot with a counter
 * one below the old copy's, so that a reset at any time leaves the old or
 * the new copy valid. When few sectors are erased, the sector with the
 * fewest valid copies is emptied, logged and erased. Changed pages wait
 * in a few RAM buffers until a file is closed or synced, a name changes,
 * or a buffer is needed: "sync" before a reset.
 *
 * Long file names are shown (non-ASCII characters as '?'); names created
 * here must be ASCII.
 */
#include <linux/bitops.h>
#include <linux/ctype.h>
#include <linux/gfp.h>
#include <linux/fs.h>
#include <linux/fs_context.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/minmax.h>
#include <linux/sched.h>
#include <linux/uio.h>
#include <linux/slab.h>
#include <linux/statfs.h>
#include <linux/string.h>
#include <linux/time.h>
#include <linux/unaligned.h>
#include <mach/fxcg.h>

/* ------------------------------------------------------ record format */

#define SLOT_SIZE	0x1000UL
#define RECORD_SIZE	64
#define REC_HEADER	0x22
#define REC_DATA	0x11
#define REC_ERASED	0x88
#define ERASES		0x200		/* erase counts in an erase log slot */

static u16 crc16_ccitt(u16 c, const u8 *p, size_t n)
{
	static const u16 t[16] = {
		0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
		0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
	};

	while (n--) {
		c = c << 4 ^ t[(c >> 12) ^ (*p >> 4)];
		c = c << 4 ^ t[(c >> 12) ^ (*p++ & 15)];
	}
	return c;
}

/*
 * Hamming code of n bytes seen as a block of 2^bits bytes padded with
 * 0xff (which changes nothing). Line parity bit 2k is the parity of the
 * bytes whose address has bit k clear, bit 2k + 1 of those with it set;
 * column parities likewise over the bit positions.
 */
static void hamming(const u8 *p, unsigned int n, unsigned int bits,
		    u32 *lp, u8 *cp)
{
	unsigned int i, k, addr = 0, odd = 0;
	u8 col = 0;

	for (i = 0; i < n; i++) {
		col ^= p[i];
		if (parity8(p[i])) {
			addr ^= i;
			odd ^= 1;
		}
	}
	*lp = 0;
	for (k = 0; k < bits; k++)
		*lp |= (odd ^ (addr >> k & 1)) << 2 * k | (addr >> k & 1) << (2 * k + 1);
	*cp = parity8(col & 0x55) | parity8(col & 0xaa) << 1 |
	      parity8(col & 0x33) << 2 | parity8(col & 0xcc) << 3 |
	      parity8(col & 0x0f) << 4 | parity8(col & 0xf0) << 5;
}

/*
 * The record of a slot holding len bytes of data followed by 0xff up to
 * 4 KiB.
 */
static void make_record(u8 *r, u8 type, u32 count, u16 page,
			const u8 *data, unsigned int len)
{
	unsigned int i, n;
	u32 lp;
	u8 cp;
	u16 crc;

	memset(r, 0xff, RECORD_SIZE);
	r[0] = type;
	r[1] = 0x0f;
	put_unaligned_be32(count, r + 2);
	put_unaligned_be16(page, r + 6);
	crc = crc16_ccitt(0xffff, data, len);
	for (i = len; i < SLOT_SIZE; i++)
		crc = crc16_ccitt(crc, (const u8 *)"\xff", 1);
	put_unaligned_be16(crc, r + 8);
	for (i = 0; i < 8; i++) {
		n = i * 512 < len ? min(len - i * 512, 512U) : 0;
		hamming(data + i * 512, n, 9, &lp, &cp);
		r[10 + 3 * i] = ~(cp << 2 | lp >> 16);
		r[11 + 3 * i] = ~(lp >> 8);
		r[12 + 3 * i] = ~lp;
	}
	memset(r + 34, 0x5a, 3);
	hamming(r, 34, 8, &lp, &cp);
	r[37] = ~cp | 0xc0;
	r[38] = ~lp;
	r[39] = ~(lp >> 8);
}

/* ---------------------------------------------------------- NOR flash */

#define FLASH		0x80000000UL	/* P1: cached view of the NOR flash */
#define AREA_START	0x00c80000UL	/* the storage sectors, to the end */
#define AREA_END	0x02000000UL
#define FLASH_SECTOR	FXCG_FLASH_SECTOR

static const u8 *flash(unsigned long off)
{
	return (const u8 *)(FLASH + off);
}

static bool blank(const u8 *p, size_t n)
{
	return !memchr_inv(p, 0xff, n);
}

/* the storage area only: nor.c refuses anything else */
static int nor_program(unsigned long off, const u8 *src, unsigned long len)
{
	return fxcg_nor_program(off, src, len, AREA_START, AREA_END);
}

static int nor_erase(unsigned long off)
{
	return fxcg_nor_erase(off, AREA_START, AREA_END);
}

/* ---------------------------------------------- flash translation layer */

#define NR_SECTORS	((AREA_END - AREA_START) / FLASH_SECTOR)
#define NR_SLOTS	30
#define RECORDS		0x1f000UL
#define SEC_FREE	0xff		/* erased */
#define SEC_DIRTY	0xfe		/* neither in use nor erased */
#define FREE_RESERVE	8		/* erased sectors kept for the Casio OS too */

#define MAX_PAGES	0x1100		/* the 16 MiB disk has 0x1013 pages */
#define UNMAPPED	0xffff

static u16 *page_map;			/* page -> sector << 5 | slot */

static struct {
	u8 used[NR_SECTORS];		/* slots taken, or SEC_FREE/SEC_DIRTY */
	u8 valid[NR_SECTORS];		/* slots with the valid copy of a page */
	u32 seq;			/* highest sector number in use */
	u32 erase_count;		/* counter of the last erase log record */
	u32 erases[NR_SECTORS];		/* times each sector was erased */
	int head;			/* the sector being filled, -1: none */
	unsigned int nr_free;
	bool in_gc;
	u8 *gc_buf;
} ftl;

static unsigned long sector_off(unsigned int sec)
{
	return AREA_START + sec * FLASH_SECTOR;
}

static unsigned long slot_off(u16 loc)
{
	return sector_off(loc >> 5) + SLOT_SIZE + (loc & 31) * SLOT_SIZE;
}

/* record of slot (loc & 31); slot -1 (record 0) is the header page */
static unsigned long record_off(u16 loc)
{
	return sector_off(loc >> 5) + RECORDS + ((loc & 31) + 1) * RECORD_SIZE;
}

static const u8 *slot_addr(u16 loc)
{
	return flash(slot_off(loc));
}

static const u8 *record_addr(u16 loc)
{
	return flash(record_off(loc));
}

static bool sector_header_ok(const u8 *h)
{
	int i;

	for (i = 0; i < 24; i += 8)
		if (get_unaligned_be32(h + i) != ~get_unaligned_be32(h + i + 4))
			return false;
	return true;
}

static int build_page_map(void)
{
	unsigned int sec, slot, pages = 0;
	unsigned int newest = 0;
	const u8 *erases = NULL;

	/* 8.5 KiB: kmalloc would take 16 */
	page_map = alloc_pages_exact(MAX_PAGES * sizeof(*page_map), GFP_KERNEL);
	if (!page_map)
		return -ENOMEM;
	memset(page_map, 0xff, MAX_PAGES * sizeof(*page_map));
	memset(&ftl, 0, sizeof(ftl));
	ftl.erase_count = 0xffffffff;
	ftl.head = -1;

	for (sec = 0; sec < NR_SECTORS; sec++) {
		const u8 *base = flash(sector_off(sec));

		if (!sector_header_ok(base)) {
			ftl.used[sec] = blank(base, FLASH_SECTOR) ? SEC_FREE : SEC_DIRTY;
			ftl.nr_free += ftl.used[sec] == SEC_FREE;
			continue;
		}
		ftl.seq = max(ftl.seq, get_unaligned_be32(base + 4));
		for (slot = 0; slot < NR_SLOTS; slot++) {
			u16 loc = sec << 5 | slot;
			const u8 *r = record_addr(loc);
			unsigned int page = get_unaligned_be16(r + 6);

			if (!blank(r, RECORD_SIZE))
				ftl.used[sec] = slot + 1;
			if (r[0] == REC_ERASED &&
			    get_unaligned_be32(r + 2) < ftl.erase_count) {
				ftl.erase_count = get_unaligned_be32(r + 2);
				erases = slot_addr(loc) + ERASES;
			}
			if (r[0] != REC_DATA || page >= MAX_PAGES)
				continue;
			if (page_map[page] == UNMAPPED)
				pages++;
			else if (get_unaligned_be32(r + 2) >=
				 get_unaligned_be32(record_addr(page_map[page]) + 2))
				continue;	/* an older copy */
			page_map[page] = loc;
		}
	}
	for (sec = 0; sec < MAX_PAGES; sec++)
		if (page_map[sec] != UNMAPPED)
			ftl.valid[page_map[sec] >> 5]++;
	for (sec = 0; erases && sec < NR_SECTORS; sec++)
		ftl.erases[sec] = erases[3 * sec] << 16 |
				  erases[3 * sec + 1] << 8 | erases[3 * sec + 2];
	for (sec = 0; sec < NR_SECTORS; sec++)
		if (ftl.used[sec] < SEC_DIRTY &&
		    get_unaligned_be32(flash(sector_off(sec)) + 4) == ftl.seq)
			newest = sec + 1;
	if (newest && ftl.used[newest - 1] < NR_SLOTS)
		ftl.head = newest - 1;
	pr_info("fls: %u pages of the storage memory in use, %u sectors free\n",
		pages, ftl.nr_free);
	return 0;
}

static int ftl_gc(void);

/* Take an erased sector into use as the one being filled. */
static int ftl_open(void)
{
	unsigned int i, sec, start = ftl.head < 0 ? 0 : ftl.head + 1;
	u8 hdr[24], r[RECORD_SIZE];
	int ret;

	for (i = 0; !ftl.in_gc && ftl.nr_free < FREE_RESERVE && i < 4; i++) {
		ret = ftl_gc();
		if (ret == -ENOSPC)
			break;
		if (ret)
			return ret;
	}
	if (ftl.head >= 0 && ftl.used[ftl.head] < NR_SLOTS)
		return 0;		/* the collection opened one */
	if (!ftl.nr_free)
		return -ENOSPC;
	for (i = 0; i < NR_SECTORS; i++) {
		sec = (start + i) % NR_SECTORS;
		if (ftl.used[sec] == SEC_FREE)
			break;
	}
	ftl.used[sec] = 0;		/* whatever happens, not free any more */
	ftl.nr_free--;
	ftl.head = -1;
	if (!blank(flash(sector_off(sec)), FLASH_SECTOR)) {
		ftl.used[sec] = SEC_DIRTY;
		return -EIO;
	}
	for (i = 0; i < 24; i += 8) {
		put_unaligned_be32(~(ftl.seq + 1), hdr + i);
		put_unaligned_be32(ftl.seq + 1, hdr + i + 4);
	}
	make_record(r, REC_HEADER, 0xffffffff, 0xffff, hdr, sizeof(hdr));
	ret = nor_program(sector_off(sec), hdr, sizeof(hdr));
	if (!ret)
		ret = nor_program(sector_off(sec) + RECORDS, r, 40);
	if (ret) {
		ftl.used[sec] = SEC_DIRTY;
		return ret;
	}
	ftl.seq++;
	ftl.valid[sec] = 0;
	ftl.head = sec;
	return 0;
}

/* The next free slot of the sector being filled. */
static int ftl_slot(void)
{
	int ret;
	u16 loc;

	for (;;) {
		if (ftl.head < 0 || ftl.used[ftl.head] >= NR_SLOTS) {
			ret = ftl_open();
			if (ret)
				return ret;
		}
		loc = ftl.head << 5 | ftl.used[ftl.head]++;
		/* skip a slot a reset left half written */
		if (blank(slot_addr(loc), SLOT_SIZE) &&
		    blank(record_addr(loc), RECORD_SIZE))
			return loc;
	}
}

/* Write a slot and its record; the location, or an error. */
static int ftl_put(u8 type, u32 count, u16 page, const u8 *data,
		   unsigned int len)
{
	u8 r[RECORD_SIZE];
	int loc, ret;

	loc = ftl_slot();
	if (loc < 0)
		return loc;
	make_record(r, type, count, page, data, len);
	ret = nor_program(slot_off(loc), data, len);
	if (!ret)
		ret = nor_program(record_off(loc), r, 40);
	return ret ?: loc;
}

/* A new copy of a disk page, newer than the one there is. */
static int ftl_store(unsigned int page, const u8 *data)
{
	u16 old = page_map[page];
	u32 count = 0xfffffffe;
	int loc;

	if (old != UNMAPPED)
		count = get_unaligned_be32(record_addr(old) + 2) - 1;
	loc = ftl_put(REC_DATA, count, page, data, SLOT_SIZE);
	if (loc < 0)
		return loc;
	if (old != UNMAPPED)
		ftl.valid[old >> 5]--;
	page_map[page] = loc;
	ftl.valid[loc >> 5]++;
	return 0;
}

static int ftl_write(unsigned int page, const u8 *data)
{
	u16 old = page_map[page];

	if (old == UNMAPPED ? blank(data, SLOT_SIZE) :
	    !memcmp(slot_addr(old), data, SLOT_SIZE))
		return 0;		/* nothing changes */
	return ftl_store(page, data);
}

/* Log and erase a sector whose pages all have newer copies elsewhere. */
static int ftl_erase(unsigned int sec)
{
	u8 *log = ftl.gc_buf;
	unsigned int i;
	int ret;

	memset(log, 0xff, ERASES);
	put_unaligned_be16(sec, log);
	for (i = 0; i < NR_SECTORS; i++) {
		u32 n = ftl.erases[i] + (i == sec);

		log[ERASES + 3 * i] = n >> 16;
		log[ERASES + 3 * i + 1] = n >> 8;
		log[ERASES + 3 * i + 2] = n;
	}
	ret = ftl_put(REC_ERASED, ftl.erase_count - 1, 0xffff, log,
		      ERASES + 3 * NR_SECTORS);
	if (ret < 0)
		return ret;
	ftl.erase_count--;
	ftl.erases[sec]++;
	ftl.used[sec] = SEC_DIRTY;
	ret = nor_erase(sector_off(sec));
	if (ret)
		return ret;
	ftl.used[sec] = SEC_FREE;
	ftl.valid[sec] = 0;
	ftl.nr_free++;
	return 0;
}

/* Erase one more sector, after copying its valid pages if it has any. */
static int ftl_gc(void)
{
	unsigned int sec, slot, best = NR_SECTORS;
	int ret = 0;

	for (sec = 0; sec < NR_SECTORS; sec++) {
		if (ftl.used[sec] == SEC_DIRTY) {
			best = sec;
			break;
		}
		if (ftl.used[sec] != NR_SLOTS || sec == ftl.head)
			continue;
		if (best == NR_SECTORS || ftl.valid[sec] < ftl.valid[best])
			best = sec;
	}
	/* copying its pages and the log must take less than a sector */
	if (best == NR_SECTORS || (ftl.used[best] == NR_SLOTS &&
				   ftl.valid[best] + 1 >= NR_SLOTS))
		return -ENOSPC;

	ftl.in_gc = true;
	for (slot = 0; slot < NR_SLOTS && ftl.used[best] == NR_SLOTS; slot++) {
		u16 loc = best << 5 | slot;
		const u8 *r = record_addr(loc);
		unsigned int page = get_unaligned_be16(r + 6);

		if (r[0] != REC_DATA || page >= MAX_PAGES ||
		    page_map[page] != loc)
			continue;
		memcpy(ftl.gc_buf, slot_addr(loc), SLOT_SIZE);
		ret = ftl_store(page, ftl.gc_buf);
		if (ret)
			break;
	}
	if (!ret)
		ret = ftl_erase(best);
	ftl.in_gc = false;
	return ret;
}

/* -------------------------------------------------- disk page buffers */

/*
 * Changed disk pages wait in these until fls_flush(); reads see them.
 * The flash is only read and written with fxcg_flash_lock held.
 */
#define NR_BUFS		4

static struct fls_buf {
	u8 *data;
	u16 page;
	bool dirty;
	unsigned long used;
} bufs[NR_BUFS];
static unsigned long buf_clock;

static const u8 ff_sector[512] = { [0 ... 511] = 0xff };

/* Where disk page n is: a buffer, the flash, NULL if never written. */
static const u8 *page_ptr(u32 page)
{
	int i;

	for (i = 0; i < NR_BUFS; i++)
		if (bufs[i].data && bufs[i].page == page)
			return bufs[i].data;
	if (page >= MAX_PAGES || page_map[page] == UNMAPPED)
		return NULL;
	return slot_addr(page_map[page]);
}

/* Where 512-byte sector n of the disk is, NULL if never written (0xff). */
static const u8 *disk_sector(u32 n)
{
	const u8 *p = page_ptr(n / 8);

	return p ? p + n % 8 * 512 : NULL;
}

static void read_disk(u32 n, unsigned int off, void *buf, unsigned int len)
{
	memcpy(buf, (disk_sector(n) ?: ff_sector) + off, len);
}

static int buf_flush(struct fls_buf *b)
{
	int ret;

	if (!b->dirty)
		return 0;
	ret = ftl_write(b->page, b->data);
	if (!ret)
		b->dirty = false;
	return ret;
}

static int fls_flush(void)
{
	int i, ret = 0;

	for (i = 0; i < NR_BUFS; i++)
		ret = buf_flush(&bufs[i]) ?: ret;
	return ret;
}

/* A buffer holding disk sector n, to change it. */
static u8 *buf_sector(u32 n)
{
	u32 page = n / 8;
	struct fls_buf *b = NULL;
	const u8 *p;
	int i, ret;

	/* sector 0 (the MBR) is fls_bmap()'s "past the end": never write it */
	if (!n || page >= MAX_PAGES || !bufs[0].data)
		return ERR_PTR(-EIO);
	for (i = 0; i < NR_BUFS; i++) {
		if (bufs[i].page == page) {
			b = &bufs[i];
			goto found;
		}
		if (!b || bufs[i].used < b->used)
			b = &bufs[i];
	}
	ret = buf_flush(b);
	if (ret)
		return ERR_PTR(ret);
	b->page = UNMAPPED;
	p = page_ptr(page);
	if (p)
		memcpy(b->data, p, SLOT_SIZE);
	else
		memset(b->data, 0xff, SLOT_SIZE);
	b->page = page;
found:
	b->used = ++buf_clock;
	b->dirty = true;
	return b->data + n % 8 * 512;
}

static int write_disk(u32 n, unsigned int off, const void *buf,
		      unsigned int len)
{
	u8 *p = buf_sector(n);

	if (IS_ERR(p))
		return PTR_ERR(p);
	memcpy(p + off, buf, len);
	return 0;
}

static int fill_disk(u32 n, int c)
{
	u8 *p = buf_sector(n);

	if (IS_ERR(p))
		return PTR_ERR(p);
	memset(p, c, 512);
	return 0;
}

/* Get ready to write: the buffers. */
static int fls_rw_init(void)
{
	int i;

	if (bufs[0].data)
		return 0;
	ftl.gc_buf = kmalloc(SLOT_SIZE, GFP_KERNEL);
	for (i = 0; i < NR_BUFS && ftl.gc_buf; i++) {
		bufs[i].data = kmalloc(SLOT_SIZE, GFP_KERNEL);
		bufs[i].page = UNMAPPED;
		if (!bufs[i].data)
			break;
	}
	if (i == NR_BUFS)
		return 0;
	while (i--)
		kfree(bufs[i].data);
	memset(bufs, 0, sizeof(bufs));
	kfree(ftl.gc_buf);
	ftl.gc_buf = NULL;
	return -ENOMEM;
}

/* ------------------------------------------------------------ FAT16 */

#define FLS_MAGIC	0x464c5330	/* "FLS0" */
#define ROOT_INO	1
#define FAT_EOC		0xfff8
#define ATTR_RO		0x01
#define ATTR_VOLUME	0x08
#define ATTR_DIR	0x10
#define ATTR_ARCH	0x20
#define ATTR_LFN	0x0f

static struct fls_volume {
	u32 fat, root, data;		/* disk sectors */
	unsigned int spc;		/* sectors per cluster */
	unsigned int spf, nr_fats;	/* sectors per FAT, FATs */
	unsigned int root_sectors;
	unsigned int clusters;		/* data clusters */
	unsigned int next_free;		/* where to look for a free cluster */
} vol;				/* under fxcg_flash_lock */

struct fls_inode {
	u16 start;			/* first cluster, 0: none */
	u16 nr_chain, cap;
	u16 *chain;			/* its clusters, read on first use */
	struct inode vfs;
};

static inline struct fls_inode *FLS_I(struct inode *inode)
{
	return container_of(inode, struct fls_inode, vfs);
}

static bool is_root(struct fls_inode *fi)
{
	return fi->vfs.i_ino == ROOT_INO;
}

static u16 fat_entry(u16 cluster)
{
	__le16 v;

	read_disk(vol.fat + cluster / 256, cluster % 256 * 2, &v, 2);
	return le16_to_cpu(v);
}

static int fat_set(u16 cluster, u16 v)
{
	__le16 x = cpu_to_le16(v);
	unsigned int i;
	int ret = 0;

	for (i = 0; i < vol.nr_fats && !ret; i++)
		ret = write_disk(vol.fat + i * vol.spf + cluster / 256,
				 cluster % 256 * 2, &x, 2);
	return ret;
}

/* A free cluster, made the end of a chain and linked after prev if any. */
static int fat_alloc(u16 prev)
{
	unsigned int i, c;
	int ret;

	for (i = 0; i < vol.clusters; i++) {
		c = 2 + (vol.next_free - 2 + i) % vol.clusters;
		if (fat_entry(c))
			continue;
		ret = fat_set(c, 0xffff);
		if (!ret && prev)
			ret = fat_set(prev, c);
		if (ret)
			return ret;
		vol.next_free = c + 1;
		return c;
	}
	return -ENOSPC;
}

static u32 cluster_sector(u16 c)
{
	return vol.data + (c - 2) * vol.spc;
}

static int zero_cluster(u16 c)
{
	unsigned int i;
	int ret = 0;

	for (i = 0; i < vol.spc && !ret; i++)
		ret = fill_disk(cluster_sector(c) + i, 0);
	return ret;
}

static unsigned int walk_chain(u16 start, u16 *chain)
{
	unsigned int n = 0;
	u16 c;

	for (c = start; c >= 2 && c < FAT_EOC && n < vol.clusters;
	     c = fat_entry(c)) {
		if (chain)
			chain[n] = c;
		n++;
	}
	return n;
}

/* The cluster chain of a file or directory, read once. */
static int fls_chain(struct fls_inode *fi)
{
	unsigned int n;

	if (fi->chain || !fi->start)
		return 0;
	n = walk_chain(fi->start, NULL);
	fi->chain = kmalloc_array(n ?: 1, sizeof(u16), GFP_KERNEL);
	if (!fi->chain)
		return -ENOMEM;
	fi->nr_chain = walk_chain(fi->start, fi->chain);
	fi->cap = n ?: 1;
	return 0;
}

/* Make the chain nr clusters long (new ones zeroed if zero). */
static int fls_grow(struct fls_inode *fi, unsigned int nr, bool zero)
{
	int c, ret = fls_chain(fi);

	if (ret || nr <= fi->nr_chain)
		return ret;
	if (nr > fi->cap) {
		unsigned int cap = round_up(nr, 16);
		u16 *chain = krealloc_array(fi->chain, cap, sizeof(u16),
					    GFP_KERNEL);

		if (!chain)
			return -ENOMEM;
		fi->chain = chain;
		fi->cap = cap;
	}
	while (fi->nr_chain < nr) {
		c = fat_alloc(fi->nr_chain ? fi->chain[fi->nr_chain - 1] : 0);
		if (c < 0)
			return c;
		if (!fi->nr_chain)
			fi->start = c;
		fi->chain[fi->nr_chain++] = c;
		if (zero) {
			ret = zero_cluster(c);
			if (ret)
				return ret;
		}
	}
	return 0;
}

/* Free the clusters of the chain from cluster nr on. */
static int fls_shrink(struct fls_inode *fi, unsigned int nr)
{
	int ret = fls_chain(fi);

	if (ret || nr >= fi->nr_chain)
		return ret;
	ret = nr ? fat_set(fi->chain[nr - 1], 0xffff) : 0;
	while (!ret && fi->nr_chain > nr)
		ret = fat_set(fi->chain[--fi->nr_chain], 0);
	if (!fi->nr_chain)
		fi->start = 0;
	return ret;
}

/* Disk sector of sector idx of a file or directory, 0 past its end. */
static u32 fls_bmap(struct fls_inode *fi, u32 idx)
{
	u32 c;

	if (is_root(fi))
		return idx < vol.root_sectors ? vol.root + idx : 0;
	c = idx / vol.spc;
	if (c >= fi->nr_chain)
		return 0;
	return cluster_sector(fi->chain[c]) + idx % vol.spc;
}

static void fat_time(struct timespec64 ts, u8 *t)
{
	struct tm tm;

	/* 1980-01-01 to 2107-12-31 */
	time64_to_tm(clamp_t(time64_t, ts.tv_sec, 315532800, 4354819199LL),
		     0, &tm);
	put_unaligned_le16(tm.tm_hour << 11 | tm.tm_min << 5 | tm.tm_sec / 2, t);
	put_unaligned_le16((tm.tm_year - 80) << 9 | (tm.tm_mon + 1) << 5 |
			   tm.tm_mday, t + 2);
}

/* ------------------------------------------------------- directories */

/* One directory entry, with its long name if it has one. */
struct fls_dirent {
	char name[256];			/* the long name, or the short one */
	int len;
	char sname[13];
	int slen;
	u8 e[32];			/* the short entry */
	loff_t first, pos;		/* first entry (long name) and its own */
	unsigned long ino;		/* where the short entry is on the disk */
};

static u8 lfn_checksum(const u8 *e)
{
	u8 sum = 0;
	int i;

	for (i = 0; i < 11; i++)
		sum = (sum >> 1 | sum << 7) + e[i];
	return sum;
}

static int short_name(const u8 *e, char *name)
{
	int i, n = 0;

	for (i = 0; i < 8 && e[i] != ' '; i++)
		name[n++] = (e[12] & 0x08) ? tolower(e[i]) : e[i];
	if (e[8] != ' ')
		name[n++] = '.';
	for (i = 8; i < 11 && e[i] != ' '; i++)
		name[n++] = (e[12] & 0x10) ? tolower(e[i]) : e[i];
	if (n && name[0] == 0x05)
		name[0] = 0xe5;
	return n;
}

static const u8 lfn_at[13] = { 1, 3, 5, 7, 9, 14, 16, 18, 20, 22, 24, 28, 30 };

/* Collect part seq of a long name from entry e. */
static void lfn_part(const u8 *e, int seq, struct fls_dirent *d)
{
	int i;

	for (i = 0; i < 13; i++) {
		u16 ch = get_unaligned_le16(e + lfn_at[i]);

		if (ch == 0 || ch == 0xffff)
			break;
		d->name[(seq - 1) * 13 + i] = ch < 0x80 ? ch : '?';
	}
}

static int dir_entry(struct fls_inode *dir, loff_t pos, u8 *e, u32 *sec)
{
	*sec = fls_bmap(dir, pos / 16);
	if (!*sec)
		return -ENOENT;
	read_disk(*sec, pos % 16 * 32, e, 32);
	return 0;
}

/*
 * Walk a directory from entry *pos on and call fn for each file and
 * subdirectory, until it returns false; *pos is then that entry.
 */
static int fls_walk(struct fls_inode *dir, loff_t *pos,
		    bool (*fn)(void *arg, struct fls_dirent *d), void *arg)
{
	struct fls_dirent *d __free(kfree) = kmalloc(sizeof(*d), GFP_KERNEL);
	int lfn_seq = 0, ret;
	u8 lfn_sum = 0, *e;
	loff_t lfn_first = 0;
	u32 sec;

	if (!d)
		return -ENOMEM;
	ret = fls_chain(dir);
	if (ret)
		return ret;
	e = d->e;

	for (;; (*pos)++) {
		int seq;

		if (dir_entry(dir, *pos, e, &sec) || e[0] == 0)
			return 0;
		if (e[0] == 0xe5) {
			lfn_seq = 0;
			continue;
		}
		if (e[11] == ATTR_LFN) {
			seq = e[0] & 0x1f;
			if (e[0] & 0x40) {	/* the last part comes first */
				memset(d->name, 0, sizeof(d->name));
				lfn_sum = e[13];
				lfn_first = *pos;
			} else if (seq != lfn_seq - 1 || e[13] != lfn_sum) {
				seq = 0;
			}
			lfn_seq = seq >= 1 && seq <= 19 ? seq : 0;
			if (lfn_seq)
				lfn_part(e, seq, d);
			continue;
		}
		if (e[11] & ATTR_VOLUME || !memcmp(e, ".          ", 11) ||
		    !memcmp(e, "..         ", 11)) {
			lfn_seq = 0;
			continue;
		}
		d->slen = short_name(e, d->sname);
		if (lfn_seq == 1 && lfn_checksum(e) == lfn_sum) {
			d->len = strnlen(d->name, 255);
			d->first = lfn_first;
		} else {
			memcpy(d->name, d->sname, d->slen);
			d->len = d->slen;
			d->first = *pos;
		}
		lfn_seq = 0;
		d->pos = *pos;
		d->ino = (unsigned long)sec << 4 | (*pos % 16);
		if (d->len && !fn(arg, d))
			return 0;
	}
}

static bool fls_nonempty(void *arg, struct fls_dirent *d)
{
	*(bool *)arg = true;
	return false;
}

static int fls_dir_empty(struct fls_inode *dir)
{
	bool found = false;
	loff_t pos = 0;
	int ret = fls_walk(dir, &pos, fls_nonempty, &found);

	return ret ?: found ? -ENOTEMPTY : 0;
}

struct fls_find {
	const struct qstr *name;	/* by name */
	unsigned long ino;		/* or by place */
	const u8 *sname;		/* or by short name */
	struct fls_dirent found;
	bool hit;
};

static bool fls_match(void *arg, struct fls_dirent *d)
{
	struct fls_find *f = arg;
	const struct qstr *q = f->name;

	if (q ? (d->len != q->len || strncasecmp(d->name, q->name, d->len)) &&
		(d->slen != q->len || strncasecmp(d->sname, q->name, d->slen)) :
	    f->sname ? memcmp(d->e, f->sname, 11) : d->ino != f->ino)
		return true;
	f->found = *d;
	f->hit = true;
	return false;
}

static struct fls_find *fls_find(struct fls_inode *dir,
				 const struct qstr *name, unsigned long ino,
				 const u8 *sname)
{
	struct fls_find *f = kmalloc(sizeof(*f), GFP_KERNEL);
	loff_t pos = 0;
	int ret;

	if (!f)
		return ERR_PTR(-ENOMEM);
	f->name = name;
	f->ino = ino;
	f->sname = sname;
	f->hit = false;
	ret = fls_walk(dir, &pos, fls_match, f);
	if (ret) {
		kfree(f);
		return ERR_PTR(ret);
	}
	return f;
}

/* Mark the entries of the file whose short entry is at ino deleted. */
static int fls_del_entry(struct fls_inode *dir, unsigned long ino)
{
	struct fls_find *f __free(kfree) = fls_find(dir, NULL, ino, NULL);
	u8 e[32];
	loff_t pos;
	u32 sec;
	int ret = 0;

	if (IS_ERR(f))
		return PTR_ERR(no_free_ptr(f));
	if (!f->hit)
		return -ENOENT;
	for (pos = f->found.first; pos <= f->found.pos && !ret; pos++) {
		ret = dir_entry(dir, pos, e, &sec);
		if (!ret)
			ret = write_disk(sec, pos % 16 * 32, "\xe5", 1);
	}
	return ret;
}

static const char fat_bad[] = "\"*/:<>?\\|";
static const char short_ok[] = "!#$%&'()-@^_`{}~";

static bool short_char(char c)
{
	return isalnum(c) || strchr(short_ok, c);
}

/*
 * The 8.3 name of name in e[0..10] and its case flags in e[12]; true if
 * that says it all, false if a long name is needed.
 */
static bool make_short(const struct qstr *q, u8 *e)
{
	const char *s = q->name, *dot = strrchr(s, '.');
	int i, n = 0, len = q->len;
	int blen = dot ? dot - s : len, elen = dot ? len - blen - 1 : 0;
	bool fits = blen >= 1 && blen <= 8 && elen <= 3 &&
		    (!dot || strchr(s, '.') == dot);
	int up[2] = {}, low[2] = {};

	memset(e, ' ', 11);
	e[12] = 0;
	for (i = 0; i < len; i++) {
		char c = s[i];
		int part = dot && s + i > dot;

		if (s + i == dot)
			continue;
		if (!short_char(c)) {
			fits = false;
			if (c == ' ' || c == '.')
				continue;
			c = '_';
		}
		up[part] |= !!isupper(c);
		low[part] |= !!islower(c);
		if (part)
			n = n < 8 ? 8 : n;
		if (part ? n < 11 : n < 8)
			e[n++] = toupper(c);
	}
	if (up[0] && low[0])
		fits = false;
	if (up[1] && low[1])
		fits = false;
	if (e[0] == ' ')
		e[0] = '_';
	if (e[0] == 0xe5)
		e[0] = 0x05;
	if (fits)
		e[12] = (low[0] ? 0x08 : 0) | (low[1] ? 0x10 : 0);
	return fits;
}

static bool short_exists(struct fls_inode *dir, const u8 *name11, int *err)
{
	struct fls_find *f __free(kfree) = fls_find(dir, NULL, 0, name11);

	if (IS_ERR(f)) {
		*err = PTR_ERR(no_free_ptr(f));
		return true;
	}
	return f->hit;
}

/* A short name no other entry of dir has, from the basis in e. */
static int unique_short(struct fls_inode *dir, u8 *e)
{
	u8 basis[8];
	char tail[8];
	int i, n, len = 8, err = 0;

	memcpy(basis, e, 8);
	while (len > 1 && basis[len - 1] == ' ')
		len--;
	for (i = 1; i < 1000000; i++) {
		n = snprintf(tail, sizeof(tail), "~%d", i);
		memcpy(e, basis, 8);
		memcpy(e + min(8 - n, len), tail, n);
		if (!short_exists(dir, e, &err))
			return 0;
		if (err)
			return err;
	}
	return -EEXIST;
}

static int check_name(const struct qstr *q)
{
	unsigned int i;

	if (!q->len || q->len > 255 || q->name[q->len - 1] == '.' ||
	    q->name[q->len - 1] == ' ')
		return -EINVAL;
	for (i = 0; i < q->len; i++)
		if (q->name[i] < 0x20 || q->name[i] > 0x7e ||
		    strchr(fat_bad, q->name[i]))
			return -EINVAL;
	return 0;
}

/*
 * Add an entry named q to dir: tmpl gives its short entry but for the
 * name. *ino is where that ends up.
 */
static int fls_add_entry(struct fls_inode *dir, const struct qstr *q,
			 u8 *tmpl, unsigned long *ino)
{
	unsigned int nlfn = 0, i, k, run = 0;
	loff_t pos, first = 0;
	u8 e[32];
	u32 sec;
	int ret = check_name(q);

	if (ret)
		return ret;
	if (!make_short(q, tmpl)) {
		nlfn = DIV_ROUND_UP(q->len, 13);
		ret = unique_short(dir, tmpl);
	} else if (short_exists(dir, tmpl, &ret)) {
		ret = ret ?: -EEXIST;
	}
	if (ret)
		return ret;

	/* nlfn + 1 free entries in a row, growing the directory if need be */
	for (pos = 0; run < nlfn + 1; pos++) {
		if (dir_entry(dir, pos, e, &sec)) {
			if (is_root(dir))
				return -ENOSPC;
			ret = fls_grow(dir, dir->nr_chain + 1, true);
			if (ret)
				return ret;
			dir_entry(dir, pos, e, &sec);
		}
		if (e[0] && e[0] != 0xe5) {
			run = 0;
			continue;
		}
		if (!run++)
			first = pos;
	}

	for (i = nlfn; i >= 1; i--, first++) {
		memset(e, 0, 32);
		e[0] = i | (i == nlfn ? 0x40 : 0);
		e[11] = ATTR_LFN;
		e[13] = lfn_checksum(tmpl);
		for (k = 0; k < 13; k++) {
			unsigned int at = (i - 1) * 13 + k;

			put_unaligned_le16(at < q->len ? q->name[at] :
					   at == q->len ? 0 : 0xffff,
					   e + lfn_at[k]);
		}
		ret = write_disk(fls_bmap(dir, first / 16), first % 16 * 32, e, 32);
		if (ret)
			return ret;
	}
	sec = fls_bmap(dir, first / 16);
	*ino = (unsigned long)sec << 4 | (first % 16);
	return write_disk(sec, first % 16 * 32, tmpl, 32);
}

/* ----------------------------------------------------------- inodes */

static const struct inode_operations fls_dir_iops;
static const struct inode_operations fls_file_iops;
static const struct file_operations fls_dir_fops;
static const struct file_operations fls_file_fops;

/* The inode of the file whose short entry e is at ino (NULL: root). */
static struct inode *fls_iget(struct super_block *sb, unsigned long ino,
			      const u8 *e)
{
	struct inode *inode = iget_locked(sb, e ? ino : ROOT_INO);
	struct timespec64 ts = {};
	struct fls_inode *fi;
	u16 time, date;

	if (!inode)
		return ERR_PTR(-ENOMEM);
	if (!(inode_state_read_once(inode) & I_NEW))
		return inode;
	fi = FLS_I(inode);
	fi->start = e ? get_unaligned_le16(e + 26) : 0;
	if (!e || e[11] & ATTR_DIR) {
		inode->i_mode = S_IFDIR | 0755;
		inode->i_op = &fls_dir_iops;
		inode->i_fop = &fls_dir_fops;
		set_nlink(inode, 2);
	} else {
		inode->i_mode = S_IFREG | 0755;	/* FAT has no x bit: allow exec */
		inode->i_op = &fls_file_iops;
		inode->i_fop = &fls_file_fops;
		inode->i_size = get_unaligned_le32(e + 28);
		set_nlink(inode, 1);
	}
	if (e && e[11] & ATTR_RO)
		inode->i_mode &= ~0222;
	time = e ? get_unaligned_le16(e + 22) : 0;
	date = e ? get_unaligned_le16(e + 24) : 0;
	if (date)
		ts.tv_sec = mktime64(1980 + (date >> 9), (date >> 5) & 15,
				     date & 31, time >> 11,
				     (time >> 5) & 63, (time & 31) * 2);
	inode_set_mtime_to_ts(inode, inode_set_atime_to_ts(inode,
				     inode_set_ctime_to_ts(inode, ts)));
	inode->i_blocks = (inode->i_size + 511) >> 9;
	unlock_new_inode(inode);
	return inode;
}

/* Write the first cluster, size and time of a file to its entry. */
static int fls_sync_entry(struct inode *inode)
{
	struct fls_inode *fi = FLS_I(inode);
	u32 sec = inode->i_ino >> 4;
	unsigned int off = (inode->i_ino & 15) * 32;
	u8 e[32];

	if (is_root(fi) || !inode->i_nlink)
		return 0;
	read_disk(sec, off, e, 32);
	put_unaligned_le16(fi->start, e + 26);
	if (!S_ISDIR(inode->i_mode)) {
		put_unaligned_le32(inode->i_size, e + 28);
		e[11] |= ATTR_ARCH;
	}
	fat_time(inode_get_mtime(inode), e + 22);
	inode->i_blocks = (inode->i_size + 511) >> 9;
	return write_disk(sec, off, e, 32);
}

/*
 * File data goes straight from the flash to the reader. A page cache
 * would only hold copies of what the flash already has at hand, and on
 * a 2 MiB nommu system its pages scattered over the RAM leave no room
 * for the next program's stack. Programs run from here get their private
 * copy through this as well (nommu mmap).
 */
static ssize_t fls_read_iter(struct kiocb *iocb, struct iov_iter *to)
{
	struct inode *inode = file_inode(iocb->ki_filp);
	struct fls_inode *fi = FLS_I(inode);
	loff_t pos = iocb->ki_pos, size = i_size_read(inode);
	ssize_t done = 0;
	int ret;

	mutex_lock(&fxcg_flash_lock);
	ret = fls_chain(fi);
	while (!ret && iov_iter_count(to) && pos < size) {
		u32 sec = fls_bmap(fi, pos / 512);
		unsigned int off = pos % 512;
		size_t n = min_t(size_t, min_t(loff_t, 512 - off, size - pos),
				 iov_iter_count(to));
		const u8 *src = sec ? disk_sector(sec) : NULL;

		if (!sec) {
			ret = -EIO;	/* the chain is shorter than the size */
			break;
		}
		if (copy_to_iter((src ?: ff_sector) + off, n, to) != n) {
			ret = -EFAULT;
			break;
		}
		pos += n;
		done += n;
	}
	mutex_unlock(&fxcg_flash_lock);
	iocb->ki_pos = pos;
	return done ?: ret;
}

/* Zero the file from byte from to byte to (both inside its clusters). */
static int fls_zero(struct fls_inode *fi, loff_t from, loff_t to)
{
	int ret = 0;

	while (from < to && !ret) {
		unsigned int off = from % 512, n = min_t(loff_t, 512 - off, to - from);
		u8 *p = buf_sector(fls_bmap(fi, from / 512));

		if (IS_ERR(p))
			return PTR_ERR(p);
		memset(p + off, 0, n);
		from += n;
	}
	return ret;
}

static int fls_resize(struct inode *inode, loff_t size)
{
	struct fls_inode *fi = FLS_I(inode);
	unsigned int csize = vol.spc * 512;
	loff_t old = i_size_read(inode);
	int ret;

	if (size > old) {
		ret = fls_grow(fi, DIV_ROUND_UP_ULL(size, csize), false);
		if (!ret)
			ret = fls_zero(fi, old, size);
	} else {
		ret = fls_shrink(fi, DIV_ROUND_UP_ULL(size, csize));
	}
	if (ret)
		return ret;
	i_size_write(inode, size);
	inode_set_mtime_to_ts(inode, inode_set_ctime_current(inode));
	return fls_sync_entry(inode);
}

static ssize_t fls_write_iter(struct kiocb *iocb, struct iov_iter *from)
{
	struct inode *inode = file_inode(iocb->ki_filp);
	struct fls_inode *fi = FLS_I(inode);
	unsigned int csize = vol.spc * 512;
	ssize_t done = 0, ret;
	loff_t pos, end;

	inode_lock(inode);
	ret = generic_write_checks(iocb, from);
	if (ret <= 0)
		goto out;
	mutex_lock(&fxcg_flash_lock);
	pos = iocb->ki_pos;
	end = pos + iov_iter_count(from);
	ret = 0;
	if (pos > i_size_read(inode))
		ret = fls_resize(inode, pos);
	if (!ret)
		ret = fls_grow(fi, DIV_ROUND_UP_ULL(end, csize), false);
	if (ret == -ENOSPC)	/* write what fits */
		end = min_t(loff_t, end, (loff_t)fi->nr_chain * csize);
	if (ret == -ENOSPC && end > pos)
		ret = 0;
	while (!ret && pos < end) {
		unsigned int off = pos % 512, n = min_t(loff_t, 512 - off, end - pos);
		u8 *p = buf_sector(fls_bmap(fi, pos / 512));
		size_t k;

		if (IS_ERR(p)) {
			ret = PTR_ERR(p);
			break;
		}
		k = copy_from_iter(p + off, n, from);
		pos += k;
		done += k;
		if (k != n)
			ret = -EFAULT;
	}
	if (pos > i_size_read(inode))
		i_size_write(inode, pos);
	if (done)
		inode_set_mtime_to_ts(inode, inode_set_ctime_current(inode));
	ret = fls_sync_entry(inode) ?: ret;	/* size, first cluster */
	mutex_unlock(&fxcg_flash_lock);
	iocb->ki_pos = pos;
out:
	inode_unlock(inode);
	return done ?: ret;
}

static int fls_fsync(struct file *file, loff_t start, loff_t end,
		     int datasync)
{
	int ret;

	mutex_lock(&fxcg_flash_lock);
	ret = fls_flush();
	mutex_unlock(&fxcg_flash_lock);
	return ret;
}

static int fls_release(struct inode *inode, struct file *file)
{
	if (file->f_mode & FMODE_WRITE)
		fls_fsync(file, 0, LLONG_MAX, 0);
	return 0;
}

/* nommu: only private copies, filled by fls_read_iter() */
static int fls_mmap_prepare(struct vm_area_desc *desc)
{
	return -ENOSYS;
}

static int fls_setattr(struct mnt_idmap *idmap, struct dentry *dentry,
		       struct iattr *attr)
{
	struct inode *inode = d_inode(dentry);
	int ret;

	/* FAT keeps no owner or mode: ignore those */
	attr->ia_valid &= ATTR_SIZE | ATTR_MTIME | ATTR_MTIME_SET | ATTR_ATIME |
			  ATTR_ATIME_SET | ATTR_CTIME | ATTR_TIMES_SET | ATTR_FILE |
			  ATTR_OPEN;
	ret = setattr_prepare(idmap, dentry, attr);
	if (ret)
		return ret;
	mutex_lock(&fxcg_flash_lock);
	if (attr->ia_valid & ATTR_SIZE && attr->ia_size != i_size_read(inode))
		ret = fls_resize(inode, attr->ia_size);
	if (!ret) {
		setattr_copy(idmap, inode, attr);
		ret = fls_sync_entry(inode);
	}
	ret = fls_flush() ?: ret;
	mutex_unlock(&fxcg_flash_lock);
	return ret;
}

static const struct file_operations fls_file_fops = {
	.llseek		= generic_file_llseek,
	.read_iter	= fls_read_iter,
	.write_iter	= fls_write_iter,
	.splice_write	= iter_file_splice_write,
	.mmap_prepare	= fls_mmap_prepare,
	.fsync		= fls_fsync,
	.release	= fls_release,
};

static const struct inode_operations fls_file_iops = {
	.setattr	= fls_setattr,
};

static struct dentry *fls_lookup(struct inode *dir, struct dentry *dentry,
				 unsigned int flags)
{
	struct fls_find *f __free(kfree) = NULL;
	struct inode *inode = NULL;

	mutex_lock(&fxcg_flash_lock);
	f = fls_find(FLS_I(dir), &dentry->d_name, 0, NULL);
	mutex_unlock(&fxcg_flash_lock);
	if (IS_ERR(f))
		return ERR_CAST(no_free_ptr(f));
	if (f->hit)
		inode = fls_iget(dir->i_sb, f->found.ino, f->found.e);
	return d_splice_alias(inode, dentry);
}

/* A short entry for a new file or directory. */
static void new_entry(u8 *e, u8 attr, u16 start)
{
	struct timespec64 now;

	ktime_get_real_ts64(&now);
	memset(e, 0, 32);
	e[11] = attr;
	fat_time(now, e + 14);			/* created */
	memcpy(e + 18, e + 16, 2);		/* accessed */
	memcpy(e + 22, e + 14, 4);		/* modified */
	put_unaligned_le16(start, e + 26);
}

static int fls_create(struct mnt_idmap *idmap, struct inode *dir,
		      struct dentry *dentry, umode_t mode, bool excl)
{
	struct inode *inode;
	unsigned long ino;
	u8 e[32];
	int ret;

	mutex_lock(&fxcg_flash_lock);
	new_entry(e, ATTR_ARCH, 0);
	ret = fls_add_entry(FLS_I(dir), &dentry->d_name, e, &ino);
	ret = fls_flush() ?: ret;
	mutex_unlock(&fxcg_flash_lock);
	if (ret)
		return ret;
	inode = fls_iget(dir->i_sb, ino, e);
	if (IS_ERR(inode))
		return PTR_ERR(inode);
	d_instantiate(dentry, inode);
	return 0;
}

static struct dentry *fls_mkdir(struct mnt_idmap *idmap, struct inode *dir,
				struct dentry *dentry, umode_t mode)
{
	struct fls_inode *fd = FLS_I(dir);
	struct inode *inode;
	unsigned long ino;
	u8 e[32];
	int c, ret;

	mutex_lock(&fxcg_flash_lock);
	c = fat_alloc(0);
	ret = c < 0 ? c : zero_cluster(c);
	if (!ret) {
		/* "." and ".." */
		new_entry(e, ATTR_DIR, c);
		memcpy(e, ".          ", 11);
		ret = write_disk(cluster_sector(c), 0, e, 32);
		memcpy(e, "..         ", 11);
		put_unaligned_le16(is_root(fd) ? 0 : fd->start, e + 26);
		ret = ret ?: write_disk(cluster_sector(c), 32, e, 32);
		new_entry(e, ATTR_DIR, c);
		ret = ret ?: fls_add_entry(fd, &dentry->d_name, e, &ino);
		if (ret)
			fat_set(c, 0);
	}
	ret = fls_flush() ?: ret;
	mutex_unlock(&fxcg_flash_lock);
	if (ret)
		return ERR_PTR(ret);
	inode = fls_iget(dir->i_sb, ino, e);
	if (IS_ERR(inode))
		return ERR_CAST(inode);
	d_instantiate(dentry, inode);
	return NULL;
}

/* Take away the entry of a file; its clusters go when the inode does. */
static int fls_remove(struct inode *dir, struct inode *inode)
{
	int ret;

	ret = fls_del_entry(FLS_I(dir), inode->i_ino);
	ret = fls_flush() ?: ret;
	if (ret)
		return ret;
	clear_nlink(inode);
	remove_inode_hash(inode);	/* its place may be reused */
	return 0;
}

static int fls_unlink(struct inode *dir, struct dentry *dentry)
{
	int ret;

	mutex_lock(&fxcg_flash_lock);
	ret = fls_remove(dir, d_inode(dentry));
	mutex_unlock(&fxcg_flash_lock);
	return ret;
}

static int fls_rmdir(struct inode *dir, struct dentry *dentry)
{
	struct inode *inode = d_inode(dentry);
	int ret;

	mutex_lock(&fxcg_flash_lock);
	ret = fls_dir_empty(FLS_I(inode));
	if (!ret)
		ret = fls_remove(dir, inode);
	mutex_unlock(&fxcg_flash_lock);
	return ret;
}

static int fls_rename(struct mnt_idmap *idmap, struct inode *old_dir,
		      struct dentry *old_dentry, struct inode *new_dir,
		      struct dentry *new_dentry, unsigned int flags)
{
	struct inode *inode = d_inode(old_dentry);
	struct inode *target = d_inode(new_dentry);
	struct fls_inode *fi = FLS_I(inode);
	unsigned long ino;
	u8 e[32];
	int ret = 0;

	if (flags & ~RENAME_NOREPLACE)
		return -EINVAL;
	mutex_lock(&fxcg_flash_lock);
	read_disk(inode->i_ino >> 4, (inode->i_ino & 15) * 32, e, 32);
	if (target && target != inode) {
		if (S_ISDIR(target->i_mode))
			ret = fls_dir_empty(FLS_I(target));
		ret = ret ?: fls_del_entry(FLS_I(new_dir), target->i_ino);
		if (!ret) {
			clear_nlink(target);
			remove_inode_hash(target);
		}
	}
	/* a new name for the same file (case only): its old place first */
	if (!ret && target == inode)
		ret = fls_del_entry(FLS_I(old_dir), inode->i_ino);
	ret = ret ?: fls_add_entry(FLS_I(new_dir), &new_dentry->d_name, e, &ino);
	if (!ret && target != inode)
		ret = fls_del_entry(FLS_I(old_dir), inode->i_ino);
	if (!ret && S_ISDIR(inode->i_mode) && old_dir != new_dir) {
		__le16 parent = cpu_to_le16(is_root(FLS_I(new_dir)) ? 0 :
					    FLS_I(new_dir)->start);

		ret = fls_chain(fi) ?:
		      write_disk(fls_bmap(fi, 0), 32 + 26, &parent, 2);
	}
	if (!ret) {
		remove_inode_hash(inode);
		inode->i_ino = ino;
		insert_inode_hash(inode);
	}
	ret = fls_flush() ?: ret;
	mutex_unlock(&fxcg_flash_lock);
	return ret;
}

static bool fls_emit(void *arg, struct fls_dirent *d)
{
	return dir_emit(arg, d->name, d->len, d->ino,
			d->e[11] & ATTR_DIR ? DT_DIR : DT_REG);
}

static int fls_readdir(struct file *file, struct dir_context *ctx)
{
	loff_t pos;
	int ret;

	if (!dir_emit_dots(file, ctx))
		return 0;
	pos = ctx->pos - 2;
	mutex_lock(&fxcg_flash_lock);
	ret = fls_walk(FLS_I(file_inode(file)), &pos, fls_emit, ctx);
	mutex_unlock(&fxcg_flash_lock);
	ctx->pos = pos + 2;
	return ret;
}

static const struct inode_operations fls_dir_iops = {
	.lookup		= fls_lookup,
	.create		= fls_create,
	.mkdir		= fls_mkdir,
	.unlink		= fls_unlink,
	.rmdir		= fls_rmdir,
	.rename		= fls_rename,
};

static const struct file_operations fls_dir_fops = {
	.llseek		= generic_file_llseek,
	.read		= generic_read_dir,
	.iterate_shared	= fls_readdir,
	.fsync		= fls_fsync,
};

/* FAT names do not care about case. */
static int fls_d_hash(const struct dentry *dentry, struct qstr *q)
{
	unsigned long hash = init_name_hash(dentry);
	unsigned int i;

	for (i = 0; i < q->len; i++)
		hash = partial_name_hash(tolower(q->name[i]), hash);
	q->hash = end_name_hash(hash);
	return 0;
}

static int fls_d_compare(const struct dentry *dentry, unsigned int len,
			 const char *str, const struct qstr *name)
{
	return len != name->len || strncasecmp(str, name->name, len);
}

static const struct dentry_operations fls_dentry_ops = {
	.d_hash		= fls_d_hash,
	.d_compare	= fls_d_compare,
};

/* ------------------------------------------------------------ super */

static struct inode *fls_alloc_inode(struct super_block *sb)
{
	struct fls_inode *fi = kzalloc(sizeof(*fi), GFP_KERNEL);

	if (!fi)
		return NULL;
	inode_init_once(&fi->vfs);
	return &fi->vfs;
}

static void fls_evict_inode(struct inode *inode)
{
	struct fls_inode *fi = FLS_I(inode);

	truncate_inode_pages_final(&inode->i_data);
	if (!inode->i_nlink && !is_root(fi)) {
		/* removed: free its clusters */
		mutex_lock(&fxcg_flash_lock);
		if (fls_shrink(fi, 0) || fls_flush())
			pr_warn("fls: freeing the clusters of a file failed\n");
		mutex_unlock(&fxcg_flash_lock);
	}
	clear_inode(inode);
}

static void fls_free_inode(struct inode *inode)
{
	struct fls_inode *fi = FLS_I(inode);

	kfree(fi->chain);
	kfree(fi);
}

static int fls_statfs(struct dentry *dentry, struct kstatfs *buf)
{
	unsigned int c, free = 0;

	mutex_lock(&fxcg_flash_lock);
	for (c = 2; c < vol.clusters + 2; c++)
		free += fat_entry(c) == 0;
	mutex_unlock(&fxcg_flash_lock);
	buf->f_type = FLS_MAGIC;
	buf->f_bsize = vol.spc * 512;
	buf->f_blocks = vol.clusters;
	buf->f_bfree = buf->f_bavail = free;
	buf->f_namelen = 255;
	return 0;
}

static int fls_sync_fs(struct super_block *sb, int wait)
{
	int ret;

	mutex_lock(&fxcg_flash_lock);
	ret = fls_flush();
	mutex_unlock(&fxcg_flash_lock);
	return ret;
}

static const struct super_operations fls_sops = {
	.alloc_inode	= fls_alloc_inode,
	.evict_inode	= fls_evict_inode,
	.free_inode	= fls_free_inode,
	.statfs		= fls_statfs,
	.sync_fs	= fls_sync_fs,
};

static int fls_read_volume(void)
{
	u8 *b __free(kfree) = kmalloc(512, GFP_KERNEL);
	unsigned int roots;
	u32 part = 0, total;

	if (!b)
		return -ENOMEM;
	/* the disk has an MBR with one FAT16 partition */
	read_disk(0, 0, b, 512);
	if (get_unaligned_le16(b + 510) == 0xaa55 && memcmp(b + 3, "MSDOS", 5))
		part = get_unaligned_le32(b + 0x1c6);
	read_disk(part, 0, b, 512);
	total = get_unaligned_le16(b + 19) ?: get_unaligned_le32(b + 32);
	roots = get_unaligned_le16(b + 17);
	vol.spf = get_unaligned_le16(b + 22);
	vol.nr_fats = b[16];
	vol.spc = b[13];
	if (get_unaligned_le16(b + 11) != 512 || !vol.spc || !vol.nr_fats ||
	    !vol.spf || get_unaligned_le16(b + 510) != 0xaa55) {
		pr_warn("fls: no FAT16 file system in the storage memory\n");
		return -EINVAL;
	}
	vol.fat = part + get_unaligned_le16(b + 14);
	vol.root = vol.fat + vol.nr_fats * vol.spf;
	vol.root_sectors = roots * 32 / 512;
	vol.data = vol.root + vol.root_sectors;
	vol.clusters = min3((part + total - vol.data) / vol.spc, 0xffefU,
			    vol.spf * 256 - 2);
	vol.next_free = 2;
	if ((part + total + 7) / 8 > MAX_PAGES) {
		pr_warn("fls: the disk is larger than expected\n");
		return -EINVAL;
	}
	pr_info("fls: FAT16, %u clusters of %u KiB\n", vol.clusters, vol.spc / 2);
	return 0;
}

static int fls_fill_super(struct super_block *sb, struct fs_context *fc)
{
	struct inode *root;
	int ret;

	if (!page_map) {
		mutex_lock(&fxcg_flash_lock);
		ret = build_page_map();
		if (!ret)
			ret = fls_read_volume();
		if (ret) {
			free_pages_exact(page_map, MAX_PAGES * sizeof(*page_map));
			page_map = NULL;
		}
		mutex_unlock(&fxcg_flash_lock);
		if (ret)
			return ret;
	}
	if (!(sb->s_flags & SB_RDONLY)) {
		ret = fls_rw_init();
		if (ret)
			return ret;
	}
	sb->s_flags |= SB_NOATIME;
	sb->s_magic = FLS_MAGIC;
	sb->s_op = &fls_sops;
	sb->s_blocksize = 512;
	sb->s_blocksize_bits = 9;
	sb->s_maxbytes = 0xffffffff;
	sb->s_time_gran = 2 * NSEC_PER_SEC;
	set_default_d_op(sb, &fls_dentry_ops);
	root = fls_iget(sb, ROOT_INO, NULL);
	if (IS_ERR(root))
		return PTR_ERR(root);
	sb->s_root = d_make_root(root);
	return sb->s_root ? 0 : -ENOMEM;
}

static int fls_get_tree(struct fs_context *fc)
{
	return get_tree_single(fc, fls_fill_super);
}

/* mount -o remount,rw / ro */
static int fls_reconfigure(struct fs_context *fc)
{
	sync_filesystem(fc->root->d_sb);
	return fc->sb_flags & SB_RDONLY ? 0 : fls_rw_init();
}

/* no options of its own: the others (relatime...) are ignored, as ramfs does */
static int fls_parse_param(struct fs_context *fc, struct fs_parameter *param)
{
	int ret = vfs_parse_fs_param_source(fc, param);

	return ret == -ENOPARAM ? 0 : ret;
}

static const struct fs_context_operations fls_context_ops = {
	.parse_param	= fls_parse_param,
	.get_tree	= fls_get_tree,
	.reconfigure	= fls_reconfigure,
};

static int fls_init_fs_context(struct fs_context *fc)
{
	fc->ops = &fls_context_ops;
	return 0;
}

static struct file_system_type fls_fs_type = {
	.name		= "fls",
	.init_fs_context = fls_init_fs_context,
	.kill_sb	= kill_anon_super,
};

static int __init fls_init(void)
{
	return register_filesystem(&fls_fs_type);
}
fs_initcall(fls_init);
