// SPDX-License-Identifier: GPL-2.0
/*
 * arch/sh/boot/compressed/misc.c
 *
 * This is a collection of several routines from gzip-1.0.3
 * adapted for Linux.
 *
 * malloc by Hannu Savolainen 1993 and Matthias Urlichs 1994
 *
 * Adapted for SH by Stuart Menefy, Aug 1999
 *
 * Modified to use standard LinuxSH BIOS by Greg Banks 7Jul2000
 */

#include <linux/uaccess.h>
#include <asm/addrspace.h>
#include <asm/page.h>

#include "misc.h"

#ifdef CONFIG_FXCG_DEBUG_MARKERS
#include <mach/debug-marker.h>
#else
#define fxcg_debug_mark(slot, color) do { } while (0)
#endif

/*
 * gzip declarations
 */

#define STATIC static

#undef memset
#undef memcpy
#define memzero(s, n)     memset ((s), 0, (n))

extern char input_data[];
extern int input_len;
static unsigned char *output;

static void error(char *m);

int puts(const char *);

extern int _text;		/* Defined in vmlinux.lds.S */
extern int _end;
static unsigned long free_mem_ptr;
static unsigned long free_mem_end_ptr;

#ifdef CONFIG_HAVE_KERNEL_BZIP2
#define HEAP_SIZE	0x400000
#else
#define HEAP_SIZE	0x10000
#endif

#ifdef CONFIG_KERNEL_GZIP
#include "../../../../lib/decompress_inflate.c"
#endif

#ifdef CONFIG_KERNEL_BZIP2
#include "../../../../lib/decompress_bunzip2.c"
#endif

#ifdef CONFIG_KERNEL_LZMA
#include "../../../../lib/decompress_unlzma.c"
#endif

#ifdef CONFIG_KERNEL_XZ
#include "../../../../lib/decompress_unxz.c"
#endif

#ifdef CONFIG_KERNEL_LZO
#include "../../../../lib/decompress_unlzo.c"
#endif

int puts(const char *s)
{
	/* This should be updated to use the sh-sci routines */
	return 0;
}

void* memset(void* s, int c, size_t n)
{
	int i;
	char *ss = (char*)s;

	for (i=0;i<n;i++) ss[i] = c;
	return s;
}

void* memcpy(void* __dest, __const void* __src,
			    size_t __n)
{
	int i;
	char *d = (char *)__dest, *s = (char *)__src;

	for (i=0;i<__n;i++) d[i] = s[i];
	return __dest;
}

static void error(char *x)
{
	fxcg_debug_mark(0, FXCG_MARK_RED);
	puts("\n\n");
	puts(x);
	puts("\n\n -- System halted");

	while(1);	/* Halt */
}

const unsigned long __stack_chk_guard = 0x000a0dff;

void __stack_chk_fail(void)
{
	error("stack-protector: Kernel stack is corrupted\n");
}

/* Needed because vmlinux.lds.h references this */
void ftrace_stub(void)
{
}
void arch_ftrace_ops_list_func(void)
{
}

#define stackalign	4

#define STACK_SIZE (4096)
long __attribute__ ((aligned(stackalign))) user_stack[STACK_SIZE];
long *stack_start = &user_stack[STACK_SIZE];

void decompress_kernel(void)
{
	unsigned long output_addr;

	output_addr = __pa((unsigned long)&_text+PAGE_SIZE);
#if defined(CONFIG_29BIT)
	output_addr |= P2SEG;
#endif

	output = (unsigned char *)output_addr;
	free_mem_ptr = (unsigned long)&_end;
	free_mem_end_ptr = free_mem_ptr + HEAP_SIZE;

	fxcg_debug_mark(0, FXCG_MARK_YELLOW);
	puts("Uncompressing Linux... ");
	__decompress(input_data, input_len, NULL, NULL, output, 0, NULL, error);
#if defined(CONFIG_CPU_SH4A) && defined(CONFIG_32BIT)
	/*
	 * Without the 29-bit P2 alias the kernel is written through the
	 * operand cache. Push it to memory and drop any instruction cache
	 * line that still holds what used to be there (e.g. a boot loader
	 * or this image before it moved itself) before jumping to it.
	 */
	if ((unsigned long)input_data > output_addr) {
		unsigned long a;

		for (a = output_addr & ~31UL; a < (unsigned long)input_data; a += 32)
			__asm__ __volatile__("ocbwb @%0\n\t"
					     "icbi @%0" : : "r" (a) : "memory");
	}
#endif
	fxcg_debug_mark(1, FXCG_MARK_GREEN);
	puts("Ok, booting the kernel.\n");
}
