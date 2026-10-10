// SPDX-License-Identifier: GPL-2.0
/*
 * Casio fx-CG10/fx-CG20 (Prizm) support
 *
 * SH7305 @ 58 MHz, 2 MiB of RAM at 0x08000000, 32 MiB NOR flash
 * (the Casio OS's; the free part after its image can hold a kernel that
 * runs in place, see xip.c), R61524 384x216 LCD.
 *
 * The kernel is started by the fxcg-linux loader add-in, which takes
 * the machine over from the Casio OS: interrupts and the MMU are off
 * and the zImage has been copied to RAM, or the kernel runs from the
 * flash. Rebooting goes through a CPU reset, which brings the Casio OS
 * back.
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/memblock.h>
#include <linux/mutex.h>
#include <linux/printk.h>
#include <linux/string.h>
#include <asm/machvec.h>
#include <asm/setup.h>
#include <mach/fxcg.h>
#include <mach/debug-marker.h>

/* see mach/fxcg.h */
DEFINE_MUTEX(fxcg_flash_lock);

/*
 * What the loader asks for, at the end of the IL memory: "fxcgMODE" and
 * 1 to install the kernel that runs in place from the flash, 2 to remove
 * it. It becomes "fxcg.boot=install" or "fxcg.boot=remove" on the command
 * line (for the init scripts), once: the word is cleared.
 */
#define FXCG_BOOT_MODE	0xe5200ff0

static void __init fxcg_boot_mode(char *cmdline)
{
	static const char * const modes[] = { NULL, "install", "remove" };
	volatile u32 *p = (volatile u32 *)FXCG_BOOT_MODE;
	const char *mode;

	if (p[0] != 0x66786367 || p[1] != 0x4d4f4445 ||	/* "fxcgMODE" */
	    p[2] >= ARRAY_SIZE(modes) || !modes[p[2]]) {
		p[0] = 0;
		return;
	}
	mode = modes[p[2]];
	p[0] = 0;
	strlcat(cmdline, " fxcg.boot=", COMMAND_LINE_SIZE);
	strlcat(cmdline, mode, COMMAND_LINE_SIZE);
	strlcat(boot_command_line, " fxcg.boot=", COMMAND_LINE_SIZE);
	strlcat(boot_command_line, mode, COMMAND_LINE_SIZE);
	pr_info("fxcg: the loader asks to %s\n", mode);
}

static void __init fxcg_setup(char **cmdline_p)
{
	if (IS_ENABLED(CONFIG_FXCG_DEBUG_MARKERS))
		fxcg_debug_mark(2, FXCG_MARK_WHITE);
	fxcg_console_init();
	pr_info("Casio fx-CG10/fx-CG20 (Prizm)\n");
	fxcg_boot_mode(*cmdline_p);
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
