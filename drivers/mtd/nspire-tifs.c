// SPDX-License-Identifier: GPL-2.0-only
/*
 * Block device backed by a file of the TI-Nspire OS filesystem
 *
 * The TI-Nspire OS keeps its documents in Datalight Reliance on top of
 * Datalight FlashFX Pro, in the "filesystem" NAND partition. This driver
 * finds one preallocated file there (by default the Linux root filesystem
 * image created by linuxloader2) and exposes its contents as /dev/tifs0.
 * Reliance metadata is never modified: the file keeps its size and blocks,
 * only the contents of its blocks are rewritten, the way FlashFX itself
 * does it (append a newer copy of the logical page, reclaim full units).
 *
 * Formats, as documented by Goplat and on Hackspire:
 *
 * FlashFX: the partition is made of units of unitTotalBlocks pages. The
 * first page of a unit is a header whose spare area holds 0x48E2; the
 * other pages hold logical pages of a region (clientAddress), the spare
 * area holding 0x4000 | the page index within the region. The latest copy
 * of a logical page is the one in the unit with the highest sequence
 * number, then the highest page within the unit.
 *
 * Reliance: MAST header 0x40 bytes into the volume, two META copies (the
 * higher counter wins) pointing to the inode of the index file, INOD
 * blocks with inline data, direct, single or double indirect pointers,
 * directory entries naming inode indexes.
 *
 * What the documentation leaves open is learned or verified, never
 * assumed: the spare offset of the allocation info comes from the unit
 * headers, the page ECC (algorithm and placement) is matched against the
 * pages on flash, spare bytes of new pages are copied from the previous
 * copy of the same page, and the file's block list is checked against the
 * tag the loader writes at the start of every 4 KiB chunk. Writing is only
 * enabled when all of this checks out; otherwise the device is read-only.
 */

#define pr_fmt(fmt) "nspire-tifs: " fmt

#include <linux/blk-mq.h>
#include <linux/blkdev.h>
#include <linux/bvec.h>
#include <linux/crc32.h>
#include <linux/ctype.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand-ecc-sw-hamming.h>
#include <linux/mutex.h>
#include <linux/random.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/unaligned.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>

static char *tifs_path = "/documents/linux/rootfs.img.tns";
module_param_named(path, tifs_path, charp, 0444);
MODULE_PARM_DESC(path, "File of the TI-Nspire filesystem to expose");

static char *tifs_partition = "filesystem";
module_param_named(partition, tifs_partition, charp, 0444);
MODULE_PARM_DESC(partition, "MTD partition holding the TI-Nspire filesystem");

static bool tifs_allow_write = true;
module_param_named(write, tifs_allow_write, bool, 0444);
MODULE_PARM_DESC(write, "Allow writing when all safety checks pass (default: yes)");

#define FFX_UNIT_MAGIC		0x48e2
#define FFX_ALLOC_PAGE		0x4000
#define FFX_ALLOC_STATUS	0xf000
#define FFX_ALLOC_INDEX		0x0fff
#define FFX_NO_PAGE		U32_MAX
/*
 * Pages a real Touchpad's FlashFX writes with this allocation info (status
 * 5, index 0xff0) here and there among data pages: not data pages, and
 * not known otherwise, so kept as they are (their units are never erased).
 */
#define FFX_ALLOC_KEEP		0x5ff0

/*
 * linuxloader2 starts every 4 KiB chunk of a new image with a tag:
 * "NSPLXIMG", le32 chunk index, le32 0; and the last one with
 * "NSPLXEND", le32 number of chunks, le32 format version (1).
 */
#define TAG_CHUNK		4096
#define TAG_MAGIC		"NSPLXIMG"
#define END_MAGIC		"NSPLXEND"
/* After the tag of the last chunk: the block list, once fully verified */
#define SEAL_MAGIC		"TIFSSEAL"
#define SEAL_OFFSET		16

#define ECC_MAX_CODES		16	/* 2048 byte page, 128 byte... steps */
#define ECC_SAMPLES		2048	/* pages looked at */
#define ECC_MIN_SAMPLES		16	/* of which must tell the codes apart */

/* Unit header field offsets; Goplat's layout, or the one on Hackspire */
struct ffx_hdr_layout {
	const char *name;
	unsigned int client, erase, seq, block_size, total, client_blocks,
		     per_region, checksum;
};

static const struct ffx_hdr_layout ffx_layouts[] = {
	{ "Goplat", 0x10, 0x14, 0x1c, 0x2c, 0x32, 0x34, 0x2e, 0x38 },
	{ "Hackspire", 0x10, 0x14, 0x1c, 0x2a, 0x30, 0x32, 0x2c, 0x36 },
};

struct ffx_unit {
	u32 seq;		/* 0: free, U32_MAX: not usable */
	u32 region;
	u32 erase_count;
	u16 next_free;		/* first free page, unit_pages when full */
	u16 live;		/* pages holding the latest copy */
	u16 keep;		/* FFX_ALLOC_KEEP pages: never erase the unit */
};

struct tifs_ecc {
	bool learned;
	unsigned int step;
	bool sm_order;
	unsigned int ncodes;
	unsigned int off[ECC_MAX_CODES];
};

struct rel_inode {
	u32 block;
	u64 size;
	unsigned int mode;
};

struct tifs {
	struct mtd_info *mtd;		/* the partition */
	struct mtd_info *master;	/* written through: the partition is read-only */
	u64 master_ofs;
	struct mutex lock;

	unsigned int page, oob, pages;	/* NAND geometry of the partition */
	unsigned int alloc_off;		/* spare offset of the allocation info */
	const struct ffx_hdr_layout *hdr;
	unsigned int unit_pages, nunits, region_pages, nregions;
	struct ffx_unit *units;
	u32 max_seq;
	u32 *map;			/* logical page -> physical page */
	u32 *map_unit_seq;
	u32 logical_pages;
	u8 *hdr_template, *hdr_spare;	/* a valid unit header page */
	struct tifs_ecc ecc;
	bool writable;
	const char *ro_reason;
	bool space_reported;		/* once, when reclaiming finds no room */

	/* Reliance */
	u32 rbs;			/* block size */
	struct rel_inode index;		/* inode of the index file */
	u32 *blocks;			/* the file's blocks */
	u32 nblocks;
	u64 file_size, disk_size;
	bool fresh;			/* every chunk still has its tag */

	/*
	 * One page each: pbuf/pspare for reads and read-back checks,
	 * wbuf/wspare for unit headers, reclaim copies and the spare area of
	 * new pages, dbuf for the data of the page being rewritten.
	 */
	u8 *pbuf, *pspare, *wbuf, *wspare, *dbuf;

	struct blk_mq_tag_set tag_set;
	struct gendisk *disk;
	int major;
};

static struct tifs *tifs_dev;

/* Raw NAND access, through the master: no ECC applied by the NAND layer */

static int tifs_raw_read(struct tifs *t, u32 phys, u8 *data, u8 *spare)
{
	struct mtd_oob_ops ops = {
		.mode = MTD_OPS_RAW,
		.len = data ? t->page : 0,
		.datbuf = data,
		.ooblen = spare ? t->oob : 0,
		.oobbuf = spare,
	};
	int ret = mtd_read_oob(t->master, t->master_ofs + (u64)phys * t->page, &ops);

	if (ret < 0 && !mtd_is_bitflip(ret)) {
		pr_err_ratelimited("reading page %u failed: %d\n", phys, ret);
		return ret;
	}
	return 0;
}

static int tifs_raw_write(struct tifs *t, u32 phys, const u8 *data, const u8 *spare)
{
	struct mtd_oob_ops ops = {
		.mode = MTD_OPS_RAW,
		.len = t->page,
		.datbuf = (u8 *)data,
		.ooblen = t->oob,
		.oobbuf = (u8 *)spare,
	};

	return mtd_write_oob(t->master, t->master_ofs + (u64)phys * t->page, &ops);
}

static bool all_ff(const u8 *p, size_t len)
{
	return !memchr_inv(p, 0xff, len);
}

static u16 spare_alloc(struct tifs *t, const u8 *spare)
{
	return get_unaligned_le16(spare + t->alloc_off);
}

/* The two allocation bytes are followed by the inverse of their XOR */
static bool spare_alloc_valid(struct tifs *t, const u8 *spare)
{
	const u8 *a = spare + t->alloc_off;

	return a[2] == (u8)~(a[0] ^ a[1]);
}

/* ECC of the page data, as learned from the pages on flash */

static void tifs_ecc_codes(struct tifs *t, const struct tifs_ecc *e, const u8 *data,
			   u8 codes[][3])
{
	unsigned int i;

	for (i = 0; i < e->ncodes; i++)
		ecc_sw_hamming_calculate(data + i * e->step, e->step, codes[i], e->sm_order);
}

static void tifs_ecc_place(struct tifs *t, const u8 *data, u8 *spare)
{
	u8 codes[ECC_MAX_CODES][3];
	unsigned int i;

	tifs_ecc_codes(t, &t->ecc, data, codes);
	for (i = 0; i < t->ecc.ncodes; i++)
		memcpy(spare + t->ecc.off[i], codes[i], 3);
}

/* Check (and correct single bit errors in) a page read raw */
static int tifs_ecc_check(struct tifs *t, u8 *data, u8 *spare)
{
	u8 codes[ECC_MAX_CODES][3];
	unsigned int i;
	int ret;

	if (!t->ecc.learned || all_ff(spare + t->alloc_off, 2))
		return 0;

	tifs_ecc_codes(t, &t->ecc, data, codes);
	for (i = 0; i < t->ecc.ncodes; i++) {
		u8 *stored = spare + t->ecc.off[i];

		if (!memcmp(stored, codes[i], 3))
			continue;
		ret = ecc_sw_hamming_correct(data + i * t->ecc.step, stored, codes[i],
					     t->ecc.step, t->ecc.sm_order);
		if (ret < 0) {
			pr_err_ratelimited("uncorrectable ECC error in step %u\n", i);
			return -EBADMSG;
		}
	}

	return 0;
}

/*
 * Find the ECC: for every candidate, every code must sit at the same spare
 * offset in all sample pages. Then everything else in the spare area, apart
 * from the allocation info, has to be the same on every page, so copying it
 * from the previous copy of a page produces a valid spare area.
 */
static void tifs_learn_ecc(struct tifs *t, u32 *samples, unsigned int nsamples)
{
	static const struct { unsigned int step; bool sm; } cand[] = {
		{ 256, false }, { 256, true }, { 512, false }, { 512, true },
	};
	u8 (*codes)[3] = kmalloc_array(ECC_MAX_CODES, 3, GFP_KERNEL);
	u8 *ref_spare = kmalloc(t->oob, GFP_KERNEL);
	bool *covered = kzalloc(t->oob, GFP_KERNEL);
	unsigned int c, i, s, j, used[ECC_MAX_CODES];

	if (!codes || !ref_spare || !covered)
		goto out;

	for (c = 0; c < ARRAY_SIZE(cand); c++) {
		struct tifs_ecc e = { .step = cand[c].step, .sm_order = cand[c].sm,
				      .ncodes = t->page / cand[c].step };
		unsigned long *possible;

		if (e.ncodes > ECC_MAX_CODES)
			continue;
		possible = bitmap_alloc(e.ncodes * t->oob, GFP_KERNEL);
		if (!possible)
			goto out;
		bitmap_fill(possible, e.ncodes * t->oob);
		for (i = 0; i < e.ncodes; i++)	/* a code does not fit there */
			bitmap_clear(possible, i * t->oob + t->oob - 2, 2);

		/*
		 * A code only says something when it is not all ones: that is
		 * also the code of blank data, and what unused spare bytes hold.
		 */
		memset(used, 0, sizeof(used));
		for (s = 0; s < nsamples; s++) {
			if (tifs_raw_read(t, samples[s], t->pbuf, t->pspare))
				break;
			tifs_ecc_codes(t, &e, t->pbuf, codes);
			for (i = 0; i < e.ncodes; i++) {
				if (codes[i][0] == 0xff && codes[i][1] == 0xff && codes[i][2] == 0xff)
					continue;
				used[i]++;
				for (j = 0; j + 3 <= t->oob; j++)
					if (memcmp(t->pspare + j, codes[i], 3))
						clear_bit(i * t->oob + j, possible);
			}
		}

		/* every code seen often enough, at exactly one offset */
		for (i = 0; i < e.ncodes; i++) {
			if (used[i] < ECC_MIN_SAMPLES)
				break;
			j = find_next_bit(possible, (i + 1) * t->oob, i * t->oob);
			if (j >= (i + 1) * t->oob ||
			    find_next_bit(possible, (i + 1) * t->oob, j + 1) < (i + 1) * t->oob)
				break;
			e.off[i] = j - i * t->oob;
		}
		bitmap_free(possible);

		if (i == e.ncodes) {
			t->ecc = e;
			t->ecc.learned = true;
			break;
		}
	}

	if (!t->ecc.learned) {
		t->ro_reason = "the page ECC on flash is not one of the known codes";
		goto out;
	}

	/* The rest of the spare area must not depend on the page */
	for (j = 0; j < 4; j++)
		covered[t->alloc_off + j] = true;
	for (i = 0; i < t->ecc.ncodes; i++)
		for (j = 0; j < 3; j++)
			covered[t->ecc.off[i] + j] = true;

	for (s = 0; s < nsamples; s++) {
		if (tifs_raw_read(t, samples[s], NULL, t->pspare))
			goto out;
		if (!s) {
			memcpy(ref_spare, t->pspare, t->oob);
			continue;
		}
		for (j = 0; j < t->oob; j++) {
			if (!covered[j] && t->pspare[j] != ref_spare[j]) {
				t->ecc.learned = false;
				t->ro_reason = "the spare area holds unknown page dependent data";
				goto out;
			}
		}
	}

	{
		char offs[ECC_MAX_CODES * 4 + 1] = "";
		unsigned int len = 0;

		for (i = 0; i < t->ecc.ncodes; i++)
			len += scnprintf(offs + len, sizeof(offs) - len, " %u", t->ecc.off[i]);
		pr_info("page ECC: Hamming over %u bytes%s, codes at spare offsets%s\n",
			t->ecc.step, t->ecc.sm_order ? " (SmartMedia order)" : "", offs);
	}
out:
	kfree(covered);
	kfree(ref_spare);
	kfree(codes);
}

/* FlashFX */

static u32 hdr_le32(struct tifs *t, const u8 *h, unsigned int off)
{
	return get_unaligned_le32(h + off);
}

static bool tifs_hdr_valid(const struct ffx_hdr_layout *l, const u8 *h, unsigned int page)
{
	u16 sum = 0;
	unsigned int i;

	for (i = 0; i < l->checksum; i++)
		sum += h[i];

	return sum == get_unaligned_le16(h + l->checksum) &&
	       get_unaligned_le16(h + l->block_size) == page;
}

static int tifs_find_layout(struct tifs *t)
{
	unsigned int eb_pages = t->mtd->erasesize / t->page;
	unsigned int best = 0, best_hits = 0, off, b;
	unsigned int hits[64] = { };

	/* Unit headers start erase blocks: find where their 0x48E2 sits */
	for (b = 0; b * eb_pages < t->pages && b < 256; b++) {
		if (tifs_raw_read(t, b * eb_pages, NULL, t->pspare))
			continue;
		for (off = 0; off + 4 <= t->oob && off < ARRAY_SIZE(hits); off++)
			if (get_unaligned_le16(t->pspare + off) == FFX_UNIT_MAGIC &&
			    t->pspare[off + 2] == (u8)~(t->pspare[off] ^ t->pspare[off + 1]))
				hits[off]++;
	}
	for (off = 0; off < ARRAY_SIZE(hits); off++) {
		if (hits[off] > best_hits) {
			best_hits = hits[off];
			best = off;
		}
	}
	if (!best_hits) {
		pr_err("no FlashFX unit header found\n");
		return -ENODEV;
	}
	t->alloc_off = best;

	/* Then which header layout the headers follow */
	for (b = 0; b * eb_pages < t->pages && b < 256; b++) {
		const struct ffx_hdr_layout *l;

		if (tifs_raw_read(t, b * eb_pages, t->pbuf, t->pspare) ||
		    spare_alloc(t, t->pspare) != FFX_UNIT_MAGIC)
			continue;
		for (l = ffx_layouts; l < ffx_layouts + ARRAY_SIZE(ffx_layouts); l++) {
			if (!tifs_hdr_valid(l, t->pbuf, t->page))
				continue;
			t->hdr = l;
			t->unit_pages = get_unaligned_le16(t->pbuf + l->total);
			t->region_pages = get_unaligned_le16(t->pbuf + l->client_blocks) *
					  get_unaligned_le16(t->pbuf + l->per_region);
			break;
		}
		if (t->hdr)
			break;
	}
	if (!t->hdr || !t->unit_pages || t->unit_pages % eb_pages || !t->region_pages ||
	    t->region_pages > FFX_ALLOC_INDEX + 1) {
		pr_err("no consistent FlashFX unit header found\n");
		return -ENODEV;
	}

	t->nunits = t->pages / t->unit_pages;
	pr_info("FlashFX: %u units of %u pages, regions of %u pages, %s header layout, allocation info at spare offset %u\n",
		t->nunits, t->unit_pages, t->region_pages, t->hdr->name, t->alloc_off);
	return 0;
}

static void tifs_map_page(struct tifs *t, u32 logical, u32 phys, u32 seq)
{
	u32 old = t->map[logical];

	if (old != FFX_NO_PAGE) {
		if (t->map_unit_seq[logical] > seq ||
		    (t->map_unit_seq[logical] == seq && old > phys))
			return;
		t->units[old / t->unit_pages].live--;
	}
	t->map[logical] = phys;
	t->map_unit_seq[logical] = seq;
	t->units[phys / t->unit_pages].live++;
}

/*
 * A unit FlashFX does not use the way this driver knows: its first page
 * (data and spare), then the spare of its second, 8 bytes each.
 */
/*
 * A unit without a valid header, as a real Touchpad's last one: 1 when no
 * page after the first carries allocation info (no data page in it).
 */
static int tifs_unit_unused(struct tifs *t, unsigned int u)
{
	unsigned int p;
	int ret;

	for (p = 1; p < t->unit_pages; p++) {
		ret = tifs_raw_read(t, u * t->unit_pages + p, NULL, t->wspare);
		if (ret)
			return ret;
		if (spare_alloc(t, t->wspare) != 0xffff)
			return 0;
	}
	return 1;
}

/* The first FFX_ALLOC_KEEP page: its data's start and its spare area */
static void tifs_show_keep(struct tifs *t, unsigned int u, unsigned int p)
{
	unsigned int off;

	if (tifs_raw_read(t, u * t->unit_pages + p, t->pbuf, t->pspare))
		return;
	pr_info("%04x page: u%u p%u\n", FFX_ALLOC_KEEP, u, p);
	for (off = 0; off < 32; off += 8)
		pr_info("kd %02x %*phN\n", off, 8, t->pbuf + off);
	for (off = 0; off < min(16U, t->oob); off += 8)
		pr_info("ks %02x %*phN\n", off, 8, t->pspare + off);
}

static void tifs_show_unit(struct tifs *t, unsigned int u, const char *why)
{
	pr_warn("unit %u: %s\n", u, why);
	pr_warn("d %*phN\n", 8, t->pbuf);
	pr_warn("s %*phN\n", 8, t->pspare);
	if (!tifs_raw_read(t, u * t->unit_pages + 1, NULL, t->pspare))
		pr_warn("s1 %*phN\n", 8, t->pspare);
}

static int tifs_scan(struct tifs *t, u32 *samples, unsigned int *nsamples)
{
	unsigned int u, p, regions = 0, seen = 0, bad_units = 0, bad_pages = 0;
	unsigned int bad_status[16] = { }, foreign = 0, keep = 0;
	int ret;

	t->units = vcalloc(t->nunits, sizeof(*t->units));
	if (!t->units)
		return -ENOMEM;

	/* Unit headers first: they tell the size of the logical space */
	for (u = 0; u < t->nunits; u++) {
		struct ffx_unit *unit = &t->units[u];
		u32 phys = u * t->unit_pages;

		/*
		 * Bad blocks are FlashFX's business: a unit is only used when it
		 * has a valid header, or when it is entirely blank (free).
		 */
		if (tifs_raw_read(t, phys, t->pbuf, t->pspare))
			return -EIO;
		if (all_ff(t->pspare, t->oob) && all_ff(t->pbuf, t->page))
			continue;	/* free */
		if (spare_alloc(t, t->pspare) != FFX_UNIT_MAGIC ||
		    !tifs_hdr_valid(t->hdr, t->pbuf, t->page)) {
			unit->seq = U32_MAX;
			ret = tifs_unit_unused(t, u);
			if (ret < 0)
				return ret;
			if (ret) {
				/* holds no data page: left alone */
				foreign++;
				continue;
			}
			if (bad_units++ < 2)
				tifs_show_unit(t, u, "no header");
			continue;
		}
		unit->seq = hdr_le32(t, t->pbuf, t->hdr->seq);
		unit->erase_count = hdr_le32(t, t->pbuf, t->hdr->erase);
		unit->region = hdr_le32(t, t->pbuf, t->hdr->client) / t->page / t->region_pages;
		if (hdr_le32(t, t->pbuf, t->hdr->client) % (t->page * t->region_pages) ||
		    !unit->seq || unit->seq == U32_MAX) {
			unit->seq = U32_MAX;
			if (bad_units++ < 2)
				tifs_show_unit(t, u, "client or sequence");
			continue;
		}
		regions = max(regions, unit->region + 1);
		t->max_seq = max(t->max_seq, unit->seq);
		if (!t->hdr_template) {
			t->hdr_template = kmemdup(t->pbuf, t->page, GFP_KERNEL);
			t->hdr_spare = kmemdup(t->pspare, t->oob, GFP_KERNEL);
			if (!t->hdr_template || !t->hdr_spare)
				return -ENOMEM;
		}
	}

	t->nregions = regions;
	t->logical_pages = regions * t->region_pages;
	if (!regions) {
		pr_err("no FlashFX unit in use\n");
		return -ENODEV;
	}
	t->map = vmalloc_array(t->logical_pages, sizeof(u32));
	t->map_unit_seq = vcalloc(t->logical_pages, sizeof(u32));
	if (!t->map || !t->map_unit_seq)
		return -ENOMEM;
	memset(t->map, 0xff, t->logical_pages * sizeof(u32));

	*nsamples = 0;
	for (u = 0; u < t->nunits; u++) {
		struct ffx_unit *unit = &t->units[u];

		if (!unit->seq || unit->seq == U32_MAX)
			continue;
		unit->next_free = t->unit_pages;
		for (p = 1; p < t->unit_pages; p++) {
			u32 phys = u * t->unit_pages + p;
			u16 alloc;

			if (tifs_raw_read(t, phys, NULL, t->pspare))
				return -EIO;
			alloc = spare_alloc(t, t->pspare);
			if (alloc == 0xffff) {
				/* pages are written in order: the rest is free */
				unit->next_free = p;
				break;
			}
			if (alloc == FFX_ALLOC_KEEP && spare_alloc_valid(t, t->pspare)) {
				if (!keep++)
					tifs_show_keep(t, u, p);
				unit->keep++;
				continue;
			}
			if ((alloc & FFX_ALLOC_STATUS) != FFX_ALLOC_PAGE ||
			    !spare_alloc_valid(t, t->pspare) ||
			    (alloc & FFX_ALLOC_INDEX) >= t->region_pages) {
				bad_status[alloc >> 12] += spare_alloc_valid(t, t->pspare);
				if (bad_pages++ < 3)
					pr_warn("unknown: u%u p%u %*phN\n", u, p, 8, t->pspare);
				continue;
			}
			tifs_map_page(t, unit->region * t->region_pages + (alloc & FFX_ALLOC_INDEX),
				      phys, unit->seq);
			/*
			 * Sample pages from all over the flash (reservoir
			 * sampling): a big image file of zeros written in one
			 * go says nothing about the ECC.
			 */
			if (*nsamples < ECC_SAMPLES) {
				samples[(*nsamples)++] = phys;
			} else {
				u32 r = get_random_u32_below(seen + 1);

				if (r < ECC_SAMPLES)
					samples[r] = phys;
			}
			seen++;
		}
	}

	if (foreign || keep)
		pr_info("left alone: %u units without data pages or header, %u pages marked %04x\n",
			foreign, keep, FFX_ALLOC_KEEP);
	if (bad_units || bad_pages) {
		pr_warn("%u units and %u pages in an unknown state\n", bad_units, bad_pages);
		/* the unknown pages with a valid check, by allocation status */
		for (u = 0; u < ARRAY_SIZE(bad_status); u++)
			if (bad_status[u])
				pr_warn("status %xxxx: %u pages\n", u, bad_status[u]);
		t->ro_reason = "pages or units in an unknown state";
	}
	return 0;
}

/* Logical reads (through the FlashFX map) */

static int tifs_read_logical_page(struct tifs *t, u32 logical, u8 *buf)
{
	u32 phys;
	int ret;

	if (logical >= t->logical_pages)
		return -EINVAL;
	phys = t->map[logical];
	if (phys == FFX_NO_PAGE) {
		memset(buf, 0xff, t->page);
		return 0;
	}
	ret = tifs_raw_read(t, phys, buf, t->pspare);
	if (ret)
		return ret;
	return tifs_ecc_check(t, buf, t->pspare);
}

static int tifs_read_logical(struct tifs *t, u64 ofs, void *buf, size_t len)
{
	while (len) {
		u32 logical = div_u64(ofs, t->page);
		unsigned int in = ofs - (u64)logical * t->page;
		size_t n = min_t(size_t, len, t->page - in);
		int ret = tifs_read_logical_page(t, logical, t->pbuf);

		if (ret)
			return ret;
		memcpy(buf, t->pbuf + in, n);
		buf += n;
		ofs += n;
		len -= n;
	}
	return 0;
}

/* Reliance */

#define REL_INODE_DATA	0x40

static int rel_read_block(struct tifs *t, u32 block, void *buf, size_t len)
{
	if (len > t->rbs)
		return -EINVAL;
	return tifs_read_logical(t, (u64)block * t->rbs, buf, len);
}

static int rel_get_inode(struct tifs *t, u32 block, struct rel_inode *ino, u8 *buf)
{
	int ret = rel_read_block(t, block, buf, t->rbs);

	if (ret)
		return ret;
	if (memcmp(buf, "INOD", 4))
		return -EUCLEAN;
	ino->block = block;
	ino->size = get_unaligned_le32(buf + 8);
	ino->mode = get_unaligned_le32(buf + 0x28) & 3;
	return 0;
}

static int rel_index_lookup(struct tifs *t, const struct rel_inode *index, u32 idx,
			    u32 *block, u8 *buf);

/*
 * An indirect block (INDI, DBLI) into buf: it is named by its index, like
 * an inode, through the index file. A real Touchpad's file points to
 * indexes 867 and 868, next to its own inode's, while block 867 is free
 * space holding other data.
 */
static int rel_get_node(struct tifs *t, u32 idx, const char *sig, u8 *buf)
{
	u32 block;
	int ret = rel_index_lookup(t, &t->index, idx, &block, buf) ?:
		  rel_read_block(t, block, buf, t->rbs);

	if (ret)
		return ret;
	return memcmp(buf, sig, 4) ? -EUCLEAN : 0;
}

/*
 * Number of the data block holding byte n * rbs of an inode. Indirect
 * blocks have the inode's 0x40-byte header too: on a real Touchpad, a
 * file of 13096 blocks of 512 bytes is double indirect through two
 * pointers, as 112 pointers a block give 12544 blocks single indirect.
 * Data blocks are named by their number.
 */
static int rel_data_block(struct tifs *t, const struct rel_inode *ino, u32 n,
			  u8 *buf, u32 *block)
{
	u32 per = (t->rbs - REL_INODE_DATA) / 4;
	u32 ptr;
	int ret;

	/* The index file's own indirect blocks would need itself */
	if (ino->mode > 1 && ino->block == t->index.block)
		return -EUCLEAN;
	ret = rel_read_block(t, ino->block, buf, t->rbs);
	if (ret)
		return ret;

	switch (ino->mode) {
	case 1:
		if (n >= per)
			return -EUCLEAN;
		break;
	case 2:
		if (n / per >= per)
			return -EUCLEAN;
		ptr = get_unaligned_le32(buf + REL_INODE_DATA + 4 * (n / per));
		ret = rel_get_node(t, ptr, "INDI", buf);
		if (ret)
			return ret;
		break;
	case 3:
		if (n / per / per >= per)
			return -EUCLEAN;
		ptr = get_unaligned_le32(buf + REL_INODE_DATA + 4 * (n / per / per));
		ret = rel_get_node(t, ptr, "DBLI", buf);
		if (ret)
			return ret;
		ptr = get_unaligned_le32(buf + REL_INODE_DATA + 4 * (n / per % per));
		ret = rel_get_node(t, ptr, "INDI", buf);
		if (ret)
			return ret;
		break;
	default:
		return -EUCLEAN;
	}
	*block = get_unaligned_le32(buf + REL_INODE_DATA + 4 * (n % per));
	return 0;
}

/*
 * All the data blocks of a file, walking its indirect blocks once: one
 * index lookup an indirect block, not three reads a data block. *done
 * tells how many were found when it fails.
 */
static int rel_file_blocks(struct tifs *t, const struct rel_inode *ino, u32 *blocks,
			   u32 nblocks, u32 *done, u8 *buf)
{
	u32 per = (t->rbs - REL_INODE_DATA) / 4, i, j, k;
	u32 *top, *mid;
	int ret;

	*done = 0;
	if (ino->mode < 1 || ino->mode > 3)
		return -EUCLEAN;
	top = kmalloc_array(2 * per, sizeof(u32), GFP_KERNEL);
	if (!top)
		return -ENOMEM;
	mid = top + per;
	ret = rel_read_block(t, ino->block, buf, t->rbs);
	if (ret)
		goto out;
	for (i = 0; i < per; i++)
		top[i] = get_unaligned_le32(buf + REL_INODE_DATA + 4 * i);

	for (i = 0; i < per && *done < nblocks; i++) {
		if (ino->mode == 1) {
			blocks[(*done)++] = top[i];
			continue;
		}
		ret = rel_get_node(t, top[i], ino->mode == 3 ? "DBLI" : "INDI", buf);
		if (ret)
			goto out;
		if (ino->mode == 2) {
			for (k = 0; k < per && *done < nblocks; k++)
				blocks[(*done)++] = get_unaligned_le32(buf + REL_INODE_DATA + 4 * k);
			continue;
		}
		for (j = 0; j < per; j++)
			mid[j] = get_unaligned_le32(buf + REL_INODE_DATA + 4 * j);
		for (j = 0; j < per && *done < nblocks; j++) {
			ret = rel_get_node(t, mid[j], "INDI", buf);
			if (ret)
				goto out;
			for (k = 0; k < per && *done < nblocks; k++)
				blocks[(*done)++] = get_unaligned_le32(buf + REL_INODE_DATA + 4 * k);
		}
	}
	if (*done < nblocks)
		ret = -EUCLEAN;
out:
	kfree(top);
	return ret;
}

static int rel_read_inode_data(struct tifs *t, const struct rel_inode *ino, u64 ofs,
			       void *out, size_t len, u8 *buf)
{
	if (ofs + len > ino->size)
		return -EINVAL;

	if (ino->mode == 0) {
		int ret;

		if (REL_INODE_DATA + ofs + len > t->rbs)
			return -EUCLEAN;
		ret = rel_read_block(t, ino->block, buf, t->rbs);
		if (ret)
			return ret;
		memcpy(out, buf + REL_INODE_DATA + ofs, len);
		return 0;
	}

	while (len) {
		u32 n = div_u64(ofs, t->rbs), block;
		unsigned int in = ofs - (u64)n * t->rbs;
		size_t chunk = min_t(size_t, len, t->rbs - in);
		int ret = rel_data_block(t, ino, n, buf, &block);

		if (ret)
			return ret;
		ret = tifs_read_logical(t, (u64)block * t->rbs + in, out, chunk);
		if (ret)
			return ret;
		out += chunk;
		ofs += chunk;
		len -= chunk;
	}
	return 0;
}

/* Index table: inode block of an inode index */
static int rel_index_lookup(struct tifs *t, const struct rel_inode *index, u32 idx,
			    u32 *block, u8 *buf)
{
	__le32 v;
	int ret;

	if (4ULL * idx + 4 > index->size)
		return -EUCLEAN;
	ret = rel_read_inode_data(t, index, 4ULL * idx, &v, 4, buf);
	*block = le32_to_cpu(v);
	return ret;
}

/*
 * Directory entries, as on a real TI-Nspire: units of 16 bytes. The first
 * holds 0x80, a checksum (?), the length of the entry (u32 at 3), the
 * length of the name in bytes (u16 at 7), attributes at 9 and the inode
 * index (u32 at 11). Each next unit starts with the number of units left
 * (u16: 2, 1 for a name in two units), then holds 7 characters of the
 * UTF-16 name.
 */
#define REL_DIRENT_UNIT		16
#define REL_DIRENT_LEN		3
#define REL_DIRENT_NAMELEN	7
#define REL_DIRENT_ATTR		9
#define REL_DIRENT_ATTR_USED	BIT(0)
#define REL_DIRENT_ATTR_DIR	BIT(1)
#define REL_DIRENT_INDEX	11
#define REL_DIRENT_NAME_SKIP	2
#define REL_DIRENT_NAME_BYTES	(REL_DIRENT_UNIT - REL_DIRENT_NAME_SKIP)

/*
 * Read an entry's header (its first unit) and check its lengths; returns 1
 * past the last entry.
 */
static int rel_dirent(struct tifs *t, const struct rel_inode *dir, u64 pos,
		      u8 hdr[REL_DIRENT_UNIT], u32 *len, u16 *m, u8 *buf)
{
	int ret;

	if (pos + REL_DIRENT_UNIT > dir->size)
		return 1;
	ret = rel_read_inode_data(t, dir, pos, hdr, REL_DIRENT_UNIT, buf);
	if (ret)
		return ret;
	if (hdr[0] != 0x80)
		return 1;
	*len = get_unaligned_le32(hdr + REL_DIRENT_LEN);
	*m = get_unaligned_le16(hdr + REL_DIRENT_NAMELEN);
	if (*m % 2 || *m > 2 * NAME_MAX || *len > dir->size - pos ||
	    *len < REL_DIRENT_UNIT * (1 + DIV_ROUND_UP(*m, REL_DIRENT_NAME_BYTES)))
		return -EUCLEAN;
	return 0;
}

/* The m bytes of an entry's UTF-16 name, out of the units after the first */
static int rel_dirent_name(struct tifs *t, const struct rel_inode *dir, u64 pos, u16 m,
			   u8 *wname, u8 *buf)
{
	u16 done, n;
	int ret;

	for (done = 0; done < m; done += n) {
		n = min_t(u16, m - done, REL_DIRENT_NAME_BYTES);
		ret = rel_read_inode_data(t, dir, pos + REL_DIRENT_UNIT *
					  (1 + done / REL_DIRENT_NAME_BYTES) +
					  REL_DIRENT_NAME_SKIP, wname + done, n, buf);
		if (ret)
			return ret;
	}
	return 0;
}

/* Look a name up in a directory; returns the inode index */
static int rel_dir_lookup(struct tifs *t, const struct rel_inode *dir, const char *name,
			  size_t namelen, u32 *idx, bool *is_dir, u8 *buf)
{
	u8 hdr[REL_DIRENT_UNIT], wname[2 * NAME_MAX];
	u64 pos;
	u32 len;
	u16 m;
	int ret;

	for (pos = 0; !(ret = rel_dirent(t, dir, pos, hdr, &len, &m, buf)); pos += len) {
		bool match = true;
		size_t i;

		if (!(hdr[REL_DIRENT_ATTR] & REL_DIRENT_ATTR_USED) || m != 2 * namelen)
			continue;
		ret = rel_dirent_name(t, dir, pos, m, wname, buf);
		if (ret)
			return ret;
		for (i = 0; i < namelen && match; i++)
			match = !wname[2 * i + 1] && tolower(wname[2 * i]) == tolower(name[i]);
		if (match) {
			*idx = get_unaligned_le32(hdr + REL_DIRENT_INDEX);
			*is_dir = hdr[REL_DIRENT_ATTR] & REL_DIRENT_ATTR_DIR;
			return 0;
		}
	}
	return ret < 0 ? ret : -ENOENT;
}

/*
 * When a lookup fails: the directory's entries, one a line (position,
 * attributes, inode index, name), and the bytes where the walk stopped.
 */
static void rel_dump_dir(struct tifs *t, const struct rel_inode *dir, u8 *buf)
{
	u8 hdr[REL_DIRENT_UNIT], wname[2 * 24];
	char name[25];
	u64 pos;
	u32 len;
	u16 m;
	int ret;

	for (pos = 0; !(ret = rel_dirent(t, dir, pos, hdr, &len, &m, buf)); pos += len) {
		size_t i, n = min_t(size_t, m, sizeof(wname));

		if (rel_dirent_name(t, dir, pos, n, wname, buf))
			return;
		for (i = 0; i < n / 2; i++)
			name[i] = wname[2 * i + 1] || !isprint(wname[2 * i]) ? '?' : wname[2 * i];
		name[i] = '\0';
		pr_err("ent %03llx a%02x i%u %s\n", pos, hdr[REL_DIRENT_ATTR],
		       get_unaligned_le32(hdr + REL_DIRENT_INDEX), name);
	}
	if (pos + REL_DIRENT_UNIT <= dir->size &&
	    !rel_read_inode_data(t, dir, pos, hdr, REL_DIRENT_UNIT, buf))
		pr_err("end %03llx (%d): %*ph\n", pos, ret, REL_DIRENT_UNIT, hdr);
}

/* Bytes on the console, 8 a line to fit the calculator's screen */
static void tifs_dump(const char *what, const u8 *p, unsigned int len)
{
	unsigned int off;

	for (off = 0; off < len; off += 8)
		pr_err("%s %02x %*phN\n", what, off, min(8U, len - off), p + off);
}

/*
 * When a file's blocks cannot be found: its inode's header and first
 * pointers, then down the chain of first pointers, each indirect block's
 * index, block, signature and first pointers.
 */
static void rel_dump_file(struct tifs *t, const struct rel_inode *ino, u8 *buf)
{
	unsigned int level;
	u32 idx, block;

	pr_err("inode at block %u: mode %u, %llu bytes, blocks of %u\n", ino->block,
	       ino->mode, ino->size, t->rbs);
	if (rel_read_block(t, ino->block, buf, t->rbs))
		return;
	tifs_dump("i", buf, 8);
	tifs_dump("ip", buf + REL_INODE_DATA, 16);
	for (level = 1; level < ino->mode; level++) {
		idx = get_unaligned_le32(buf + REL_INODE_DATA);
		if (rel_index_lookup(t, &t->index, idx, &block, buf) ||
		    rel_read_block(t, block, buf, t->rbs)) {
			pr_err("index %u: not found\n", idx);
			return;
		}
		pr_err("index %u: block %u\n", idx, block);
		tifs_dump("n", buf, 8);
		tifs_dump("np", buf + REL_INODE_DATA, 16);
	}
}

static int tifs_find_file(struct tifs *t)
{
	struct rel_inode *index = &t->index, cur;
	u8 *buf = kmalloc(max_t(u32, t->rbs, 512), GFP_KERNEL);
	u8 mast[0x30], meta[2][0x40];
	const char *p = tifs_path;
	u32 meta_ptr[2], block;
	bool is_dir = true;
	int ret, best;
	u32 i;

	if (!buf)
		return -ENOMEM;

	ret = tifs_read_logical(t, 0x40, mast, sizeof(mast));
	if (ret)
		goto out;
	if (memcmp(mast, "MAST", 4)) {
		pr_err("no Reliance volume (MAST) found\n");
		ret = -ENODEV;
		goto out;
	}
	t->rbs = get_unaligned_le32(mast + 0x08);
	if (t->rbs < 256 || t->rbs > 65536 || !is_power_of_2(t->rbs)) {
		pr_err("Reliance: unexpected block size %u\n", t->rbs);
		ret = -EUCLEAN;
		goto out;
	}
	kfree(buf);
	buf = kmalloc(t->rbs, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	meta_ptr[0] = get_unaligned_le32(mast + 0x10);
	meta_ptr[1] = get_unaligned_le32(mast + 0x14);
	best = -1;
	for (i = 0; i < 2; i++) {
		if (rel_read_block(t, meta_ptr[i], meta[i], sizeof(meta[i])) ||
		    memcmp(meta[i], "META", 4))
			continue;
		if (best < 0 || get_unaligned_le32(meta[i] + 4) > get_unaligned_le32(meta[best] + 4))
			best = i;
	}
	if (best < 0) {
		pr_err("no Reliance META block\n");
		ret = -EUCLEAN;
		goto out;
	}

	ret = rel_get_inode(t, get_unaligned_le32(meta[best] + 8), index, buf);
	if (ret) {
		pr_err("Reliance: index file inode (block %u): %d\n",
		       get_unaligned_le32(meta[best] + 8), ret);
		goto out;
	}
	ret = rel_index_lookup(t, index, 2, &block, buf) ?:
	      rel_get_inode(t, block, &cur, buf);
	if (ret) {
		pr_err("Reliance: root directory: %d\n", ret);
		goto out;
	}

	/* Walk the path */
	while (*p) {
		const char *end;
		u32 idx;

		while (*p == '/')
			p++;
		if (!*p)
			break;
		if (!is_dir) {
			ret = -ENOTDIR;
			goto out;
		}
		end = strchrnul(p, '/');
		ret = rel_dir_lookup(t, &cur, p, end - p, &idx, &is_dir, buf);
		if (ret) {
			pr_err("%s: %.*s: %d in a directory of %llu bytes (mode %u)\n",
			       tifs_path, (int)(end - p), p, ret, cur.size, cur.mode);
			rel_dump_dir(t, &cur, buf);
			goto out;
		}
		ret = rel_index_lookup(t, index, idx, &block, buf) ?:
		      rel_get_inode(t, block, &cur, buf);
		if (ret) {
			pr_err("%s: %.*s: inode index %u, block %u: %d\n", tifs_path,
			       (int)(end - p), p, idx, block, ret);
			goto out;
		}
		p = end;
	}
	if (is_dir || cur.mode == 0) {
		pr_err("%s is not a preallocated file\n", tifs_path);
		ret = -EINVAL;
		goto out;
	}

	t->file_size = cur.size;
	t->nblocks = div_u64(cur.size + t->rbs - 1, t->rbs);
	t->blocks = vmalloc_array(t->nblocks, sizeof(u32));
	if (!t->blocks) {
		ret = -ENOMEM;
		goto out;
	}
	ret = rel_file_blocks(t, &cur, t->blocks, t->nblocks, &i, buf);
	for (; !ret && i < t->nblocks; i++)
		if ((u64)(t->blocks[i] + 1) * t->rbs > (u64)t->logical_pages * t->page)
			ret = -EUCLEAN;
	if (ret) {
		pr_err("%s: block %u of %u: %d\n", tifs_path, i, t->nblocks, ret);
		rel_dump_file(t, &cur, buf);
		goto out;
	}
	pr_info("%s: %llu bytes in %u blocks of %u bytes\n", tifs_path, t->file_size,
		t->nblocks, t->rbs);
out:
	kfree(buf);
	return ret;
}

/* File contents */

static int tifs_file_io(struct tifs *t, u64 ofs, void *buf, size_t len, bool write);

static bool tifs_tag_ok(struct tifs *t, u32 chunk, u32 chunks)
{
	u8 tag[16];

	if (tifs_file_io(t, (u64)chunk * TAG_CHUNK, tag, sizeof(tag), false))
		return false;
	if (chunk == chunks - 1)
		return !memcmp(tag, END_MAGIC, 8) && get_unaligned_le32(tag + 8) == chunks &&
		       get_unaligned_le32(tag + 12) == 1;
	return !memcmp(tag, TAG_MAGIC, 8) && get_unaligned_le32(tag + 8) == chunk &&
	       !get_unaligned_le32(tag + 12);
}

static u32 tifs_blocks_crc(struct tifs *t)
{
	return crc32_le(~0, (const u8 *)t->blocks, t->nblocks * sizeof(u32));
}

/*
 * The loader tags every 4 KiB chunk when it creates the image; Linux leaves
 * the last chunk alone. While every chunk still has its tag, they are all
 * checked, which proves the block list right; the list is then sealed in
 * the last chunk. Later on, the list found must be the sealed one.
 */
static int tifs_check_tags(struct tifs *t)
{
	u32 chunks = div_u64(t->file_size, TAG_CHUNK), i;
	u8 seal[16];

	if (chunks < 2 || !tifs_tag_ok(t, chunks - 1, chunks)) {
		pr_err("%s: not an image created by linuxloader2\n", tifs_path);
		return -EINVAL;
	}
	t->disk_size = (u64)(chunks - 1) * TAG_CHUNK;

	t->fresh = tifs_tag_ok(t, 0, chunks);
	for (i = 1; t->fresh && i < chunks - 1; i++)
		t->fresh = tifs_tag_ok(t, i, chunks);
	if (t->fresh) {
		pr_info("%s: fresh image, all %u chunks verified\n", tifs_path, chunks);
		return 0;
	}

	if (tifs_file_io(t, (u64)(chunks - 1) * TAG_CHUNK + SEAL_OFFSET, seal, sizeof(seal), false))
		return -EIO;
	if (memcmp(seal, SEAL_MAGIC, 8) || get_unaligned_le32(seal + 8) != t->nblocks ||
	    get_unaligned_le32(seal + 12) != tifs_blocks_crc(t)) {
		if (!memcmp(seal, SEAL_MAGIC, 8))
			pr_err("%s: the blocks found differ from the verified ones\n", tifs_path);
		t->ro_reason = "the image's block list was never verified";
	}
	return 0;
}

/* Record the verified block list; called once writing is known to work */
static int tifs_seal(struct tifs *t)
{
	u8 seal[16];

	memcpy(seal, SEAL_MAGIC, 8);
	put_unaligned_le32(t->nblocks, seal + 8);
	put_unaligned_le32(tifs_blocks_crc(t), seal + 12);
	return tifs_file_io(t, t->disk_size + SEAL_OFFSET, seal, sizeof(seal), true);
}

/* Writing */

static int tifs_write_checked(struct tifs *t, u32 phys, const u8 *data, const u8 *spare)
{
	int ret = tifs_raw_write(t, phys, data, spare);

	if (!ret)
		ret = tifs_raw_read(t, phys, t->pbuf, t->pspare);
	if (!ret && (memcmp(t->pbuf, data, t->page) || memcmp(t->pspare, spare, t->oob)))
		ret = -EIO;
	if (ret)
		pr_err_ratelimited("writing page %u failed: %d\n", phys, ret);
	return ret;
}

static int tifs_free_units(struct tifs *t)
{
	unsigned int u, n = 0;

	for (u = 0; u < t->nunits; u++)
		n += !t->units[u].seq;
	return n;
}

/*
 * Room for writes: free units; pages free at the end of units; stale pages
 * (copies that are not the latest) in units reclaiming may erase, and in
 * the units holding FFX_ALLOC_KEEP pages, which it does not.
 */
static void tifs_space_report(struct tifs *t)
{
	unsigned int u, units = 0, free_pages = 0, stale = 0, kunits = 0, kstale = 0, klive = 0;

	for (u = 0; u < t->nunits; u++) {
		const struct ffx_unit *unit = &t->units[u];
		unsigned int s;

		if (!unit->seq || unit->seq == U32_MAX)
			continue;
		units++;
		free_pages += t->unit_pages - unit->next_free;
		s = unit->next_free - 1 - unit->live - unit->keep;
		if (unit->keep) {
			kunits++;
			kstale += s;
			klive += unit->live;
		} else {
			stale += s;
		}
	}
	pr_info("space: %d free units, %u in use, %u free pages in them\n",
		tifs_free_units(t), units, free_pages);
	pr_info("space: %u stale pages to reclaim; %u units with %04x pages: %u stale, %u live\n",
		stale, kunits, FFX_ALLOC_KEEP, kstale, klive);
}

/* Start a new unit for a region, with the highest sequence number */
static int tifs_new_unit(struct tifs *t, u32 region, u32 *unit_out)
{
	unsigned int u;
	int ret;

	for (u = 0; u < t->nunits; u++)
		if (!t->units[u].seq)
			break;
	if (u == t->nunits)
		return -ENOSPC;

	memcpy(t->wbuf, t->hdr_template, t->page);
	put_unaligned_le32(region * t->region_pages * t->page, t->wbuf + t->hdr->client);
	put_unaligned_le32(++t->max_seq, t->wbuf + t->hdr->seq);
	/* a unit erased before this boot has no known erase count left */
	if (!t->units[u].erase_count)
		t->units[u].erase_count = hdr_le32(t, t->hdr_template, t->hdr->erase);
	put_unaligned_le32(t->units[u].erase_count + 1, t->wbuf + t->hdr->erase);
	{
		u16 sum = 0;
		unsigned int i;

		for (i = 0; i < t->hdr->checksum; i++)
			sum += t->wbuf[i];
		put_unaligned_le16(sum, t->wbuf + t->hdr->checksum);
	}
	memcpy(t->wspare, t->hdr_spare, t->oob);
	tifs_ecc_place(t, t->wbuf, t->wspare);

	ret = tifs_write_checked(t, u * t->unit_pages, t->wbuf, t->wspare);
	if (ret) {
		t->units[u].seq = U32_MAX;	/* never again */
		return ret;
	}
	t->units[u].seq = t->max_seq;
	t->units[u].region = region;
	t->units[u].erase_count++;
	t->units[u].next_free = 1;
	t->units[u].live = 0;
	*unit_out = u;
	return 0;
}

/* The unit taking new copies of a region's pages: its newest one */
static unsigned int tifs_newest_unit(struct tifs *t, u32 region)
{
	unsigned int u, cur = t->nunits;

	for (u = 0; u < t->nunits; u++) {
		struct ffx_unit *unit = &t->units[u];

		if (unit->seq && unit->seq != U32_MAX && unit->region == region &&
		    (cur == t->nunits || unit->seq > t->units[cur].seq))
			cur = u;
	}
	return cur;
}

static int tifs_erase_unit(struct tifs *t, unsigned int u)
{
	struct erase_info ei = {
		.addr = t->master_ofs + (u64)u * t->unit_pages * t->page,
		.len = (u64)t->unit_pages * t->page,
	};
	int ret = mtd_erase(t->master, &ei);

	if (ret) {
		t->units[u].seq = U32_MAX;	/* never again */
		return ret;
	}
	t->units[u].seq = 0;
	t->units[u].live = 0;
	t->units[u].next_free = 0;
	return 0;
}

/* Copy the latest pages of unit src to the free pages of unit dst */
static int tifs_move_live(struct tifs *t, unsigned int src, unsigned int dst)
{
	unsigned int p;
	int ret;

	for (p = 1; p < t->units[src].next_free; p++) {
		u32 phys = src * t->unit_pages + p;
		u32 to = dst * t->unit_pages + t->units[dst].next_free;
		u32 logical;

		ret = tifs_raw_read(t, phys, t->wbuf, t->wspare);
		if (ret)
			return ret;
		if ((spare_alloc(t, t->wspare) & FFX_ALLOC_INDEX) >= t->region_pages)
			continue;
		logical = t->units[src].region * t->region_pages +
			  (spare_alloc(t, t->wspare) & FFX_ALLOC_INDEX);
		if (t->map[logical] != phys)
			continue;
		if (t->units[dst].next_free >= t->unit_pages)
			return -ENOSPC;
		ret = tifs_write_checked(t, to, t->wbuf, t->wspare);
		t->units[dst].next_free++;
		if (ret)
			return ret;
		tifs_map_page(t, logical, to, t->units[dst].seq);
	}
	return 0;
}

/*
 * Make room. Until a unit is erased, its pages and their copies are both
 * there (the copies win), so an interruption loses nothing.
 */
static int tifs_reclaim(struct tifs *t)
{
	unsigned int u, best = t->nunits, cur, most = 0;
	int ret;

	/* A unit without any latest copy can just go */
	for (u = 0; u < t->nunits; u++) {
		if (t->units[u].seq && t->units[u].seq != U32_MAX && !t->units[u].live &&
		    !t->units[u].keep)
			return tifs_erase_unit(t, u);
	}

	/* Or its latest pages fit into its region's newest unit */
	for (u = 0; u < t->nunits; u++) {
		struct ffx_unit *unit = &t->units[u];

		if (!unit->seq || unit->seq == U32_MAX || unit->keep)
			continue;
		cur = tifs_newest_unit(t, unit->region);
		if (cur == u || t->unit_pages - t->units[cur].next_free < unit->live)
			continue;
		if (best == t->nunits || unit->live < t->units[best].live)
			best = u;
	}
	if (best != t->nunits) {
		ret = tifs_move_live(t, best, tifs_newest_unit(t, t->units[best].region));
		return ret ?: tifs_erase_unit(t, best);
	}

	/* Otherwise compact the unit with the most stale pages into a spare unit */
	for (u = 0; u < t->nunits; u++) {
		struct ffx_unit *unit = &t->units[u];
		unsigned int stale;

		if (!unit->seq || unit->seq == U32_MAX || unit->keep)
			continue;
		stale = unit->next_free - 1 - unit->live - unit->keep;
		if (stale > most) {
			most = stale;
			best = u;
		}
	}
	if (best == t->nunits) {
		if (!t->space_reported) {
			pr_err("FlashFX: no space left to reclaim\n");
			tifs_space_report(t);
			t->space_reported = true;
		}
		return -ENOSPC;
	}
	ret = tifs_new_unit(t, t->units[best].region, &cur);
	if (ret)
		return ret;
	ret = tifs_move_live(t, best, cur);
	return ret ?: tifs_erase_unit(t, best);
}

/* Where the next copy of a page of this region goes */
static int tifs_alloc_page(struct tifs *t, u32 region, u32 *phys)
{
	unsigned int u, cur;
	int ret, tries;

	for (tries = 0; tries < 64; tries++) {
		/* Only the region's newest unit may take new copies */
		cur = tifs_newest_unit(t, region);
		if (cur != t->nunits && t->units[cur].next_free < t->unit_pages) {
			*phys = cur * t->unit_pages + t->units[cur].next_free;
			return 0;
		}
		/* Keep one free unit in reserve for reclaiming */
		if (tifs_free_units(t) > 1)
			ret = tifs_new_unit(t, region, &u);
		else
			ret = tifs_reclaim(t);
		if (ret)
			return ret;
	}
	return -ENOSPC;
}

static int tifs_write_logical_page(struct tifs *t, u32 logical, const u8 *data)
{
	u32 old = t->map[logical], phys, region = logical / t->region_pages;
	int ret;

	if (old == FFX_NO_PAGE) {
		/* the loader wrote every page of the file */
		pr_err_ratelimited("logical page %u was never written\n", logical);
		return -EIO;
	}

	ret = tifs_alloc_page(t, region, &phys);
	if (ret)
		return ret;
	/* the old copy may have moved while reclaiming */
	old = t->map[logical];

	/* Spare area: the previous copy's, with this data's ECC */
	ret = tifs_raw_read(t, old, NULL, t->wspare);
	if (ret)
		return ret;
	tifs_ecc_place(t, data, t->wspare);

	ret = tifs_write_checked(t, phys, data, t->wspare);
	if (ret) {
		/* skip the page; it stays unused */
		t->units[phys / t->unit_pages].next_free++;
		return ret;
	}
	t->units[phys / t->unit_pages].next_free++;
	tifs_map_page(t, logical, phys, t->units[phys / t->unit_pages].seq);
	return 0;
}

static int tifs_file_io(struct tifs *t, u64 ofs, void *buf, size_t len, bool write)
{
	while (len) {
		u32 n = div_u64(ofs, t->rbs);
		unsigned int in = ofs - (u64)n * t->rbs, pin;
		u64 lofs;
		u32 logical;
		size_t chunk;
		int ret;

		if (n >= t->nblocks)
			return -EINVAL;
		lofs = (u64)t->blocks[n] * t->rbs + in;
		logical = div_u64(lofs, t->page);
		pin = lofs - (u64)logical * t->page;
		chunk = min3(len, (size_t)(t->rbs - in), (size_t)(t->page - pin));
		ret = tifs_read_logical_page(t, logical, t->dbuf);
		if (ret)
			return ret;
		if (write) {
			if (!memcmp(t->dbuf + pin, buf, chunk))
				goto next;	/* unchanged: spare the flash */
			memcpy(t->dbuf + pin, buf, chunk);
			ret = tifs_write_logical_page(t, logical, t->dbuf);
			if (ret)
				return ret;
		} else {
			memcpy(buf, t->dbuf + pin, chunk);
		}
next:
		buf += chunk;
		ofs += chunk;
		len -= chunk;
	}
	return 0;
}

/* Block device */

static blk_status_t tifs_queue_rq(struct blk_mq_hw_ctx *hctx, const struct blk_mq_queue_data *bd)
{
	struct tifs *t = hctx->queue->queuedata;
	struct request *rq = bd->rq;
	bool write = rq_data_dir(rq) == WRITE;
	u64 pos = (u64)blk_rq_pos(rq) << SECTOR_SHIFT;
	struct req_iterator iter;
	struct bio_vec bv;
	int ret = 0;

	blk_mq_start_request(rq);

	switch (req_op(rq)) {
	case REQ_OP_READ:
	case REQ_OP_WRITE:
		break;
	case REQ_OP_FLUSH:
		blk_mq_end_request(rq, BLK_STS_OK);
		return BLK_STS_OK;
	default:
		blk_mq_end_request(rq, BLK_STS_NOTSUPP);
		return BLK_STS_OK;
	}

	if (write && !t->writable) {
		blk_mq_end_request(rq, BLK_STS_PROTECTION);
		return BLK_STS_OK;
	}

	mutex_lock(&t->lock);
	rq_for_each_segment(bv, rq, iter) {
		void *p = bvec_kmap_local(&bv);

		if (pos + bv.bv_len > t->disk_size)
			ret = -EIO;
		else
			ret = tifs_file_io(t, pos, p, bv.bv_len, write);
		kunmap_local(p);
		if (ret)
			break;
		if (!write)
			flush_dcache_page(bv.bv_page);
		pos += bv.bv_len;
	}
	mutex_unlock(&t->lock);

	if (ret)
		pr_err_ratelimited("%s at %llu: %d\n", write ? "write" : "read", pos, ret);
	blk_mq_end_request(rq, errno_to_blk_status(ret));
	return BLK_STS_OK;
}

static const struct blk_mq_ops tifs_mq_ops = {
	.queue_rq = tifs_queue_rq,
};

static const struct block_device_operations tifs_fops = {
	.owner = THIS_MODULE,
};

static int tifs_add_disk(struct tifs *t)
{
	struct queue_limits lim = {
		.logical_block_size = 512,
		.physical_block_size = t->page,
		.max_hw_sectors = 64,
	};
	struct gendisk *disk;
	int ret;

	t->major = register_blkdev(0, "tifs");
	if (t->major < 0)
		return t->major;

	t->tag_set.ops = &tifs_mq_ops;
	t->tag_set.nr_hw_queues = 1;
	t->tag_set.queue_depth = 16;
	t->tag_set.numa_node = NUMA_NO_NODE;
	t->tag_set.flags = BLK_MQ_F_BLOCKING;
	ret = blk_mq_alloc_tag_set(&t->tag_set);
	if (ret)
		goto err_blkdev;

	disk = blk_mq_alloc_disk(&t->tag_set, &lim, t);
	if (IS_ERR(disk)) {
		ret = PTR_ERR(disk);
		goto err_tags;
	}
	disk->major = t->major;
	disk->first_minor = 0;
	disk->minors = 1;
	disk->fops = &tifs_fops;
	disk->private_data = t;
	disk->flags |= GENHD_FL_NO_PART;
	strscpy(disk->disk_name, "tifs0");
	set_capacity(disk, t->disk_size >> SECTOR_SHIFT);
	set_disk_ro(disk, !t->writable);
	t->disk = disk;

	ret = add_disk(disk);
	if (ret)
		goto err_disk;
	return 0;

err_disk:
	put_disk(disk);
err_tags:
	blk_mq_free_tag_set(&t->tag_set);
err_blkdev:
	unregister_blkdev(t->major, "tifs");
	return ret;
}

static void tifs_free(struct tifs *t)
{
	if (!t)
		return;
	vfree(t->blocks);
	vfree(t->map);
	vfree(t->map_unit_seq);
	vfree(t->units);
	kfree(t->hdr_template);
	kfree(t->hdr_spare);
	kfree(t->pbuf);
	kfree(t->pspare);
	kfree(t->wbuf);
	kfree(t->wspare);
	kfree(t->dbuf);
	if (t->mtd)
		put_mtd_device(t->mtd);
	kfree(t);
}

static void tifs_attach(struct work_struct *work)
{
	u32 *samples = kmalloc_array(ECC_SAMPLES, sizeof(u32), GFP_KERNEL);
	unsigned int nsamples = 0, i;
	struct mtd_info *mtd;
	struct tifs *t;
	int ret;

	mtd = get_mtd_device_nm(tifs_partition);
	if (IS_ERR(mtd)) {
		kfree(samples);
		return;
	}

	t = kzalloc(sizeof(*t), GFP_KERNEL);
	if (!t) {
		put_mtd_device(mtd);
		kfree(samples);
		return;
	}
	mutex_init(&t->lock);
	t->mtd = mtd;
	t->master = mtd_get_master(mtd);
	t->master_ofs = mtd_get_master_ofs(mtd, 0);
	t->page = mtd->writesize;
	t->oob = mtd->oobsize;
	t->pages = div_u64(mtd->size, t->page);
	t->pbuf = kmalloc(t->page, GFP_KERNEL);
	t->pspare = kmalloc(t->oob, GFP_KERNEL);
	t->wbuf = kmalloc(t->page, GFP_KERNEL);
	t->wspare = kmalloc(t->oob, GFP_KERNEL);
	t->dbuf = kmalloc(t->page, GFP_KERNEL);
	ret = -ENOMEM;
	if (!samples || !t->pbuf || !t->pspare || !t->wbuf || !t->wspare || !t->dbuf)
		goto err;

	ret = tifs_find_layout(t);
	if (ret)
		goto err;
	ret = tifs_scan(t, samples, &nsamples);
	if (ret)
		goto err;
	tifs_learn_ecc(t, samples, nsamples);
	ret = tifs_find_file(t);
	if (ret)
		goto err;
	ret = tifs_check_tags(t);
	if (ret)
		goto err;

	if (!tifs_allow_write)
		t->ro_reason = "disabled with nspire_tifs.write=0";
	else if (!t->ecc.learned && !t->ro_reason)
		t->ro_reason = "the page ECC could not be learned";
	else if (!(t->master->flags & MTD_WRITEABLE))
		t->ro_reason = "the NAND is not writable";

	/* Writing needs a spare unit to reclaim with, plus one */
	for (i = 0; !t->ro_reason && tifs_free_units(t) < 2 && i < 16; i++)
		if (tifs_reclaim(t))
			break;
	if (!t->ro_reason && tifs_free_units(t) < 2)
		t->ro_reason = "no free FlashFX units";
	t->writable = !t->ro_reason;
	if (t->writable && t->fresh) {
		ret = tifs_seal(t);
		if (ret) {
			pr_err("sealing the verified block list: %d\n", ret);
			t->writable = false;
			t->ro_reason = "sealing failed";
		}
	}
	if (t->writable)
		pr_info("read-write, %d free units\n", tifs_free_units(t));
	else
		pr_warn("read-only: %s\n", t->ro_reason);
	tifs_space_report(t);

	ret = tifs_add_disk(t);
	if (ret)
		goto err;
	tifs_dev = t;
	kfree(samples);
	return;
err:
	pr_err("not attached: %d\n", ret);
	tifs_free(t);
	kfree(samples);
}

static DECLARE_WORK(tifs_attach_work, tifs_attach);

static void tifs_mtd_add(struct mtd_info *mtd)
{
	if (!tifs_dev && !strcmp(mtd->name, tifs_partition))
		schedule_work(&tifs_attach_work);
}

static void tifs_mtd_remove(struct mtd_info *mtd)
{
}

static struct mtd_notifier tifs_notifier = {
	.add = tifs_mtd_add,
	.remove = tifs_mtd_remove,
};

static int __init tifs_init(void)
{
	register_mtd_user(&tifs_notifier);
	return 0;
}
module_init(tifs_init);

static void __exit tifs_exit(void)
{
	struct tifs *t = tifs_dev;

	unregister_mtd_user(&tifs_notifier);
	flush_work(&tifs_attach_work);
	if (t) {
		del_gendisk(t->disk);
		put_disk(t->disk);
		blk_mq_free_tag_set(&t->tag_set);
		unregister_blkdev(t->major, "tifs");
		tifs_free(t);
	}
}
module_exit(tifs_exit);

MODULE_DESCRIPTION("Block device on a file of the TI-Nspire OS filesystem");
MODULE_LICENSE("GPL");
