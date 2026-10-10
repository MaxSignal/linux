// SPDX-License-Identifier: GPL-2.0
/*
 * Casio fx-CG10/fx-CG20 (Prizm) support
 *
 * SH7305 @ 58 MHz, 2 MiB of RAM at 0x08000000, 16 MiB NOR flash
 * (owned by the Casio OS, left untouched), R61524 384x216 LCD.
 *
 * The kernel is started by the fxcg-linux loader add-in, which takes
 * the machine over from the Casio OS: interrupts and the MMU are off
 * and the zImage has been copied to RAM. Rebooting goes through a CPU
 * reset, which brings the Casio OS back.
 */
#include <linux/init.h>
#include <linux/memblock.h>
#include <linux/printk.h>
#include <asm/machvec.h>
#include <mach/fxcg.h>
#include <mach/debug-marker.h>

static void __init fxcg_setup(char **cmdline_p)
{
	if (IS_ENABLED(CONFIG_FXCG_DEBUG_MARKERS))
		fxcg_debug_mark(2, FXCG_MARK_WHITE);
	fxcg_console_init();
	pr_info("Casio fx-CG10/fx-CG20 (Prizm)\n");
}

/*
 * Early allocations (memmap, per-cpu area, hash tables) go right after
 * the kernel instead of at the top of RAM, so the upper half stays one
 * free block: nommu needs the BusyBox binary physically contiguous in
 * the initramfs to execute it in place.
 */
static void __init fxcg_mem_reserve(void)
{
	memblock_set_bottom_up(true);
}

static struct sh_machine_vector mv_fxcg __initmv = {
	.mv_name	= "Casio fx-CG20",
	.mv_setup	= fxcg_setup,
	.mv_mem_reserve	= fxcg_mem_reserve,
};
