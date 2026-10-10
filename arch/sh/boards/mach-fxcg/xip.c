// SPDX-License-Identifier: GPL-2.0
/*
 * Casio fx-CG: the root file system that runs in place from the flash
 *
 * With CONFIG_XIP_KERNEL the kernel and a romfs image live in the NOR
 * flash between the end of the Casio OS image and the language data at
 * 0xb40000 (erased on the fx-CG10/fx-CG20 and the fx-CG50). They are
 * packed downwards from 0xb40000: the romfs, then the kernel, then a
 * header in the last 256 bytes that the loader and this code read:
 *
 *	0	"fxcgXIP1"
 *	8	header size (64)
 *	12	kernel start, size, CRC-32
 *	24	romfs start, size, CRC-32
 *	36	kernel entry point
 *	60	CRC-32 of bytes 0-59
 *
 * (big-endian). The romfs becomes the first MTD device, a ROM, so that
 * programs run straight from it ("root=mtd0"). physmap maps a power of
 * two: the map is rounded up (over the kernel above it, read-only), and
 * romfs knows its own size. Not partitioned: a ROM map's erase size is
 * the whole map, which partitions would be rounded to.
 */
#include <linux/crc32.h>
#include <linux/init.h>
#include <linux/log2.h>
#include <linux/mtd/physmap.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/string.h>
#include <linux/unaligned.h>

#define FXCG_XIP_HEADER		0x00b3ff00	/* P0: cached view, MMU off */
#define FXCG_XIP_HEADER_SIZE	64

static struct physmap_flash_data fxcg_rootfs_data = {
	.width		= 2,
	.probe_type	= "map_rom",
};

static struct resource fxcg_rootfs_res = {
	.name		= "romfs",
	.flags		= IORESOURCE_MEM,
};

static struct platform_device fxcg_rootfs_dev = {
	.name		= "physmap-flash",
	.id		= 0,
	.resource	= &fxcg_rootfs_res,
	.num_resources	= 1,
	.dev		= {
		.platform_data	= &fxcg_rootfs_data,
	},
};

static int __init fxcg_xip_init(void)
{
	const u8 *h = (const u8 *)FXCG_XIP_HEADER;
	u32 kstart, start, size;

	if (memcmp(h, "fxcgXIP1", 8) ||
	    get_unaligned_be32(h + 8) != FXCG_XIP_HEADER_SIZE ||
	    ~crc32_le(~0, h, 60) != get_unaligned_be32(h + 60)) {
		pr_warn("fxcg: no XIP header at %#x\n", FXCG_XIP_HEADER);
		return 0;
	}
	kstart = get_unaligned_be32(h + 12);
	start = get_unaligned_be32(h + 24);
	size = get_unaligned_be32(h + 28);
	if (!size || start < 0x20000 || start + size > kstart ||
	    start + roundup_pow_of_two(size) > 0x02000000) {
		pr_warn("fxcg: bad romfs place in the XIP header\n");
		return 0;
	}
	fxcg_rootfs_res.start = start;
	fxcg_rootfs_res.end = start + roundup_pow_of_two(size) - 1;
	pr_info("fxcg: romfs at %#x, %u KiB\n", start, size >> 10);
	return platform_device_register(&fxcg_rootfs_dev);
}
device_initcall(fxcg_xip_init);
