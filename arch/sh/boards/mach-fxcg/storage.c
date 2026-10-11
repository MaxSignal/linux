// SPDX-License-Identifier: GPL-2.0
/*
 * Casio fx-CG10/fx-CG20 storage memory file system ("fls"), read only
 *
 * The Casio OS keeps its storage memory (\\fls0, the FAT16 disk that the
 * calculator also offers over USB) in the NOR flash behind its own flash
 * translation layer. What this driver relies on was worked out from flash
 * dumps of an fx-CG20 with OS 02.02 and checked against the disk read
 * over USB:
 *
 *  - the storage area is made of 128 KiB flash sectors, each with a 4 KiB
 *    header, 30 slots of 4 KiB from offset 0x1000 and 64 records of 64
 *    bytes from offset 0x1f000;
 *  - record n + 1 describes slot n. A data record has type 0x11, a
 *    big-endian counter in bytes 2-5 that is lower for newer copies, and
 *    in bytes 6-7 the logical 4 KiB page of the disk the slot holds;
 *  - the newest copy of a page is the valid one, a page that was never
 *    written reads as 0xff.
 *
 * At mount the driver rebuilds the page map from the records and then
 * reads the FAT16 partition of that disk, so the files of the storage
 * memory appear as they do on the calculator and over USB:
 *
 *	mount -t fls fls /mnt/fls
 *
 * Long file names are shown (non-ASCII characters as '?'). The flash is
 * only ever read.
 */
#include <linux/ctype.h>
#include <linux/fs.h>
#include <linux/fs_context.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/minmax.h>
#include <linux/mutex.h>
#include <linux/uio.h>
#include <linux/slab.h>
#include <linux/statfs.h>
#include <linux/string.h>
#include <linux/time.h>
#include <linux/unaligned.h>
#include <mach/fxcg.h>

/* ---------------------------------------------- flash translation layer */

#define FLASH		0x80000000UL	/* P1: cached view of the NOR flash */
#define AREA_START	0x00c00000UL	/* storage sectors live up to the end */
#define AREA_END	0x02000000UL
#define FLASH_SECTOR	0x20000UL
#define NR_SECTORS	((AREA_END - AREA_START) / FLASH_SECTOR)
#define SLOT_SIZE	0x1000UL
#define NR_SLOTS	30
#define RECORDS		0x1f000UL
#define RECORD_SIZE	64
#define REC_DATA	0x11

#define MAX_PAGES	0x1100		/* the 16 MiB disk has 0x1013 pages */
#define UNMAPPED	0xffff

static u16 *page_map;			/* page -> sector << 5 | slot */

static const u8 *slot_addr(u16 loc)
{
	return (const u8 *)(FLASH + AREA_START + (loc >> 5) * FLASH_SECTOR +
			    SLOT_SIZE + (loc & 31) * SLOT_SIZE);
}

static const u8 *record_addr(u16 loc)
{
	return (const u8 *)(FLASH + AREA_START + (loc >> 5) * FLASH_SECTOR +
			    RECORDS + ((loc & 31) + 1) * RECORD_SIZE);
}

static bool sector_header_ok(const u8 *h)
{
	int i;

	/*
	 * three copies of { ~seq, seq }, seq numbering the sectors in the
	 * order they were taken into use: 32 bits, it goes past 255
	 */
	for (i = 0; i < 24; i += 8)
		if (get_unaligned_be32(h + i) != ~get_unaligned_be32(h + i + 4))
			return false;
	return true;
}

static int build_page_map(void)
{
	unsigned int sec, slot, pages = 0;

	page_map = kmalloc_array(MAX_PAGES, sizeof(*page_map), GFP_KERNEL);
	if (!page_map)
		return -ENOMEM;
	memset(page_map, 0xff, MAX_PAGES * sizeof(*page_map));

	for (sec = 0; sec < NR_SECTORS; sec++) {
		const u8 *base = (const u8 *)(FLASH + AREA_START + sec * FLASH_SECTOR);

		if (!sector_header_ok(base))
			continue;
		for (slot = 0; slot < NR_SLOTS; slot++) {
			u16 loc = sec << 5 | slot;
			const u8 *r = record_addr(loc);
			unsigned int page = get_unaligned_be16(r + 6);

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
	pr_info("fls: %u pages of the storage memory in use\n", pages);
	return 0;
}

static const u8 ff_sector[512] = { [0 ... 511] = 0xff };

/* Where 512-byte sector n of the disk is, NULL if never written (0xff). */
static const u8 *disk_sector(u32 n)
{
	u32 page = n / 8;

	if (page >= MAX_PAGES || page_map[page] == UNMAPPED)
		return NULL;
	return slot_addr(page_map[page]) + (n % 8) * 512;
}

static void read_disk(u32 n, unsigned int off, void *buf, unsigned int len)
{
	memcpy(buf, (disk_sector(n) ?: ff_sector) + off, len);
}

/* ------------------------------------------------------------ FAT16 */

#define FLS_MAGIC	0x464c5330	/* "FLS0" */
#define ROOT_INO	1
#define FAT_EOC		0xfff8
#define ATTR_DIR	0x10
#define ATTR_VOLUME	0x08
#define ATTR_LFN	0x0f

static struct fls_volume {
	u32 fat, root, data;		/* disk sectors */
	unsigned int spc;		/* sectors per cluster */
	unsigned int root_sectors;
	unsigned int clusters;		/* data clusters */
} vol;

struct fls_inode {
	u16 start;			/* first cluster, 0: the root directory */
	u16 nr_chain;
	u16 *chain;			/* its clusters, read on first use */
	struct inode vfs;
};

static inline struct fls_inode *FLS_I(struct inode *inode)
{
	return container_of(inode, struct fls_inode, vfs);
}

static u16 fat_entry(u16 cluster)
{
	__le16 v;

	read_disk(vol.fat + cluster / 256, cluster % 256 * 2, &v, 2);
	return le16_to_cpu(v);
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
	return 0;
}

/* Disk sector of sector idx of a file or directory, 0 past its end. */
static u32 fls_bmap(struct fls_inode *fi, u32 idx)
{
	u32 c;

	if (!fi->start)
		return idx < vol.root_sectors ? vol.root + idx : 0;
	c = idx / vol.spc;
	if (c >= fi->nr_chain)
		return 0;
	return vol.data + (fi->chain[c] - 2) * vol.spc + idx % vol.spc;
}

/* One directory entry, with its long name if it has one. */
struct fls_dirent {
	char name[256];
	int len;
	u8 attr;
	u16 start;
	u32 size;
	u16 time, date;
	unsigned long ino;		/* where the entry is on the disk */
};

static u8 lfn_checksum(const u8 *e)
{
	u8 sum = 0;
	int i;

	for (i = 0; i < 11; i++)
		sum = (sum >> 1 | sum << 7) + e[i];
	return sum;
}

static void short_name(const u8 *e, struct fls_dirent *d)
{
	int i, n = 0;

	for (i = 0; i < 8 && e[i] != ' '; i++)
		d->name[n++] = (e[12] & 0x08) ? tolower(e[i]) : e[i];
	if (e[8] != ' ')
		d->name[n++] = '.';
	for (i = 8; i < 11 && e[i] != ' '; i++)
		d->name[n++] = (e[12] & 0x10) ? tolower(e[i]) : e[i];
	if (n && d->name[0] == 0x05)
		d->name[0] = 0xe5;
	d->len = n;
}

/* Collect part seq of a long name from entry e. */
static void lfn_part(const u8 *e, int seq, struct fls_dirent *d)
{
	static const u8 at[13] = { 1, 3, 5, 7, 9, 14, 16, 18, 20, 22, 24, 28, 30 };
	int i;

	for (i = 0; i < 13; i++) {
		u16 ch = get_unaligned_le16(e + at[i]);

		if (ch == 0 || ch == 0xffff)
			break;
		d->name[(seq - 1) * 13 + i] = ch < 0x80 ? ch : '?';
	}
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
	u8 lfn_sum = 0, e[32];

	if (!d)
		return -ENOMEM;
	ret = fls_chain(dir);
	if (ret)
		return ret;

	for (;; (*pos)++) {
		u32 sec = fls_bmap(dir, *pos / 16);
		int seq;

		if (!sec)
			return 0;
		read_disk(sec, *pos % 16 * 32, e, 32);
		if (e[0] == 0)
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
		if (lfn_seq == 1 && lfn_checksum(e) == lfn_sum)
			d->len = strnlen(d->name, 255);
		else
			short_name(e, d);
		lfn_seq = 0;
		d->attr = e[11];
		d->start = get_unaligned_le16(e + 26);
		d->size = get_unaligned_le32(e + 28);
		d->time = get_unaligned_le16(e + 22);
		d->date = get_unaligned_le16(e + 24);
		d->ino = (unsigned long)sec << 4 | (*pos % 16);
		if (d->len && !fn(arg, d))
			return 0;
	}
}

/* ----------------------------------------------------------- inodes */

static const struct inode_operations fls_dir_iops;
static const struct file_operations fls_dir_fops;
static const struct file_operations fls_file_fops;

static struct inode *fls_iget(struct super_block *sb, struct fls_dirent *d)
{
	struct inode *inode = iget_locked(sb, d ? d->ino : ROOT_INO);
	struct timespec64 ts = {};
	struct fls_inode *fi;

	if (!inode)
		return ERR_PTR(-ENOMEM);
	if (!(inode_state_read_once(inode) & I_NEW))
		return inode;
	fi = FLS_I(inode);
	fi->start = d ? d->start : 0;
	if (!d || d->attr & ATTR_DIR) {
		inode->i_mode = S_IFDIR | 0555;
		inode->i_op = &fls_dir_iops;
		inode->i_fop = &fls_dir_fops;
		set_nlink(inode, 2);
	} else {
		inode->i_mode = S_IFREG | 0555;	/* FAT has no x bit: allow exec */
		inode->i_fop = &fls_file_fops;
		inode->i_size = d->size;
		set_nlink(inode, 1);
	}
	if (d && d->date)
		ts.tv_sec = mktime64(1980 + (d->date >> 9), (d->date >> 5) & 15,
				     d->date & 31, d->time >> 11,
				     (d->time >> 5) & 63, (d->time & 31) * 2);
	inode_set_mtime_to_ts(inode, inode_set_atime_to_ts(inode,
				     inode_set_ctime_to_ts(inode, ts)));
	inode->i_blocks = (inode->i_size + 511) >> 9;
	unlock_new_inode(inode);
	return inode;
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

/* nommu: only private copies, filled by fls_read_iter() */
static int fls_mmap_prepare(struct vm_area_desc *desc)
{
	return -ENOSYS;
}

static const struct file_operations fls_file_fops = {
	.llseek		= generic_file_llseek,
	.read_iter	= fls_read_iter,
	.mmap_prepare	= fls_mmap_prepare,
};

struct fls_find {
	const struct qstr *name;
	struct fls_dirent found;
	bool hit;
};

static bool fls_match(void *arg, struct fls_dirent *d)
{
	struct fls_find *f = arg;

	if (d->len != f->name->len ||
	    strncasecmp(d->name, f->name->name, d->len))
		return true;
	f->found = *d;
	f->hit = true;
	return false;
}

static struct dentry *fls_lookup(struct inode *dir, struct dentry *dentry,
				 unsigned int flags)
{
	struct fls_find *f __free(kfree) = kmalloc(sizeof(*f), GFP_KERNEL);
	struct inode *inode = NULL;
	loff_t pos = 0;
	int ret;

	if (!f)
		return ERR_PTR(-ENOMEM);
	f->name = &dentry->d_name;
	f->hit = false;
	mutex_lock(&fxcg_flash_lock);
	ret = fls_walk(FLS_I(dir), &pos, fls_match, f);
	mutex_unlock(&fxcg_flash_lock);
	if (ret)
		return ERR_PTR(ret);
	if (f->hit)
		inode = fls_iget(dir->i_sb, &f->found);
	return d_splice_alias(inode, dentry);
}

static bool fls_emit(void *arg, struct fls_dirent *d)
{
	return dir_emit(arg, d->name, d->len, d->ino,
			d->attr & ATTR_DIR ? DT_DIR : DT_REG);
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
};

static const struct file_operations fls_dir_fops = {
	.llseek		= generic_file_llseek,
	.read		= generic_read_dir,
	.iterate_shared	= fls_readdir,
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

static const struct super_operations fls_sops = {
	.alloc_inode	= fls_alloc_inode,
	.free_inode	= fls_free_inode,
	.statfs		= fls_statfs,
};

static int fls_read_volume(void)
{
	u8 *b __free(kfree) = kmalloc(512, GFP_KERNEL);
	unsigned int roots, spf;
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
	spf = get_unaligned_le16(b + 22);
	vol.spc = b[13];
	if (get_unaligned_le16(b + 11) != 512 || !vol.spc || !b[16] || !spf ||
	    get_unaligned_le16(b + 510) != 0xaa55) {
		pr_warn("fls: no FAT16 file system in the storage memory\n");
		return -EINVAL;
	}
	vol.fat = part + get_unaligned_le16(b + 14);
	vol.root = vol.fat + b[16] * spf;
	vol.root_sectors = roots * 32 / 512;
	vol.data = vol.root + vol.root_sectors;
	vol.clusters = min((part + total - vol.data) / vol.spc, 0xffefU);
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
			kfree(page_map);
			page_map = NULL;
		}
		mutex_unlock(&fxcg_flash_lock);
		if (ret)
			return ret;
	}
	sb->s_flags |= SB_RDONLY | SB_NOATIME;
	sb->s_magic = FLS_MAGIC;
	sb->s_op = &fls_sops;
	sb->s_blocksize = 512;
	sb->s_blocksize_bits = 9;
	sb->s_maxbytes = 0xffffffff;
	sb->s_time_gran = 2 * NSEC_PER_SEC;
	root = fls_iget(sb, NULL);
	if (IS_ERR(root))
		return PTR_ERR(root);
	sb->s_root = d_make_root(root);
	return sb->s_root ? 0 : -ENOMEM;
}

static int fls_get_tree(struct fs_context *fc)
{
	return get_tree_single(fc, fls_fill_super);
}

static const struct fs_context_operations fls_context_ops = {
	.get_tree	= fls_get_tree,
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
