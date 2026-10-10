/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ld script for an SH kernel that runs in place from ROM or NOR flash
 * (CONFIG_XIP_KERNEL, nommu): text and read-only data stay at
 * CONFIG_XIP_PHYS_ADDR, everything written at run time is copied to RAM
 * by the startup code (head_32.S) from where it follows them in the image.
 */
OUTPUT_ARCH(sh)
/* No __ro_after_init data in the read-only part: it is written at boot */
#define RO_AFTER_INIT_DATA
#define RUNTIME_DISCARD_EXIT
#include <asm/thread_info.h>
#include <asm/cache.h>
#include <asm/vmlinux.lds.h>

ENTRY(_stext)
SECTIONS
{
	. = CONFIG_XIP_PHYS_ADDR;
	_text = .;			/* in ROM: text and read-only data */

	.text : {
		HEAD_TEXT
		TEXT_TEXT
		SCHED_TEXT
		LOCK_TEXT
		KPROBES_TEXT
		IRQENTRY_TEXT
		SOFTIRQENTRY_TEXT
		*(.fixup)
		*(.gnu.warning)
	} = 0x0009

	RO_DATA(PAGE_SIZE)
	DWARF_EH_FRAME
	_etext = .;

	INIT_TEXT_SECTION(PAGE_SIZE)
	.exit.text : { EXIT_TEXT }

	.init.rodata : {
		INIT_SETUP(16)
		INIT_CALLS
		CON_INITCALL
		INIT_RAM_FS
		. = ALIGN(4);
		__machvec_start = .;
		KEEP(*(.machvec.init))
		__machvec_end = .;
	}
	_exiprom = .;			/* end of what stays in ROM */

	/*
	 * In RAM from here on, loaded from the image right after the
	 * read-only part; head_32.S copies __data_loc.._edata_loc to _sdata.
	 */
	__data_loc = ALIGN(4);
	. = PAGE_OFFSET + __MEMORY_START + PHYSICAL_OFFSET + CONFIG_ZERO_PAGE_OFFSET;
#undef LOAD_OFFSET
#define LOAD_OFFSET (PAGE_OFFSET + __MEMORY_START + PHYSICAL_OFFSET + \
		     CONFIG_ZERO_PAGE_OFFSET - __data_loc)
	_sdata = .;

	.empty_zero_page : AT(ADDR(.empty_zero_page) - LOAD_OFFSET) {
		KEEP(*(.empty_zero_page))
	} = 0

	RW_DATA(L1_CACHE_BYTES, PAGE_SIZE, THREAD_SIZE)
	.data.ro_after_init : AT(ADDR(.data.ro_after_init) - LOAD_OFFSET) {
		*(.data..ro_after_init)
	}
	/* sorted at boot (no build time sort for SH): writable */
	EXCEPTION_TABLE(16)
	_edata = .;

	. = ALIGN(PAGE_SIZE);
	__init_begin = .;
	.init.data : AT(ADDR(.init.data) - LOAD_OFFSET) {
		INIT_DATA
	}
	.exit.data : AT(ADDR(.exit.data) - LOAD_OFFSET) { EXIT_DATA }
	PERCPU_SECTION(L1_CACHE_BYTES)

	/* end of what is copied (an empty section, for its load address) */
	.data.endmark : AT(ADDR(.data.endmark) - LOAD_OFFSET) { }
	_edata_loc = LOADADDR(.data.endmark);

	. = ALIGN(PAGE_SIZE);
	__init_end = .;
	__bss_start = .;
	SBSS(0)
	. = ALIGN(PAGE_SIZE);
	.bss : AT(ADDR(.bss) - LOAD_OFFSET) {
		BSS_FIRST_SECTIONS
		. = ALIGN(PAGE_SIZE);
		*(.bss..page_aligned)
		. = ALIGN(PAGE_SIZE);
		*(.dynbss)
		*(BSS_MAIN)
		*(COMMON)
		. = ALIGN(8);
	}
	__bss_stop = .;
	_end = . ;

	STABS_DEBUG
	DWARF_DEBUG
	MODINFO
	ELF_DETAILS

	DISCARDS
}
