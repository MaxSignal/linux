// SPDX-License-Identifier: GPL-2.0
/*
 * SH7305 Setup
 *
 * The SH7305 is the SH4AL-DSP based SoC of the Casio fx-CG and
 * fx-9860GIII calculators. Renesas never published a manual for it;
 * the register layout below is the subset that matches the SH7724 and
 * has been confirmed by the calculator community (gint, casio-cg50
 * emulator, fx-9750GIII Linux port).
 */
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/sh_timer.h>
#include <linux/sh_intc.h>
#include <linux/irq.h>
#include <linux/usb/r8a66597.h>
#include <asm/hw_irq.h>
#include <asm/platform_early.h>

#define INTC_IPR(n)	(0xa4080000 + 4 * (n))	/* IPRA..IPRL, 16-bit */
#define INTC_IMR(n)	(0xa4080080 + 4 * (n))	/* IMR0..IMR12, 8-bit */
#define MSTPCR0		0xa4150030

/* channel 2 makes the tones of mach-fxcg/sound.c */
static struct sh_timer_config tmu0_platform_data = {
	.channels_mask = 3,
};

static struct resource tmu0_resources[] = {
	DEFINE_RES_MEM(0xa4490000, 0x2c),
	DEFINE_RES_IRQ(evt2irq(0x400)),
	DEFINE_RES_IRQ(evt2irq(0x420)),
	DEFINE_RES_IRQ(evt2irq(0x440)),
};

static struct platform_device tmu0_device = {
	.name		= "sh-tmu",
	.id		= 0,
	.dev = {
		.platform_data	= &tmu0_platform_data,
	},
	.resource	= tmu0_resources,
	.num_resources	= ARRAY_SIZE(tmu0_resources),
};

/* USB 2.0 function controller, r8a66597 compatible (clock "usb0") */
static struct r8a66597_platdata usb0_platform_data = {
	.on_chip	= 1,
	.endian		= 1,		/* big-endian FIFO access */
	.buswait	= 5,		/* what gint uses */
};

static struct resource usb0_resources[] = {
	DEFINE_RES_MEM(0xa4d80000, 0x100),
	DEFINE_RES_IRQ(evt2irq(0xa20)),
};

static struct platform_device usb0_device = {
	.name		= "r8a66597_udc",
	.id		= 0,
	.dev = {
		.platform_data	= &usb0_platform_data,
	},
	.resource	= usb0_resources,
	.num_resources	= ARRAY_SIZE(usb0_resources),
};

static struct platform_device *sh7305_devices[] __initdata = {
	&tmu0_device,
	&usb0_device,
};

static int __init sh7305_devices_setup(void)
{
	return platform_add_devices(sh7305_devices,
				    ARRAY_SIZE(sh7305_devices));
}
arch_initcall(sh7305_devices_setup);

static struct platform_device *sh7305_early_devices[] __initdata = {
	&tmu0_device,
};

void __init plat_early_device_setup(void)
{
	/* Make sure the TMU module clock is running (MSTP015). */
	__raw_writel(__raw_readl(MSTPCR0) & ~(1 << 15), MSTPCR0);

	sh_early_platform_add_devices(sh7305_early_devices,
				   ARRAY_SIZE(sh7305_early_devices));
}

enum {
	UNUSED = 0,

	/* interrupt sources */
	TMU0_TUNI0, TMU0_TUNI1, TMU0_TUNI2,
	USB0,
};

static struct intc_vect vectors[] __initdata = {
	INTC_VECT(TMU0_TUNI0, 0x400), INTC_VECT(TMU0_TUNI1, 0x420),
	INTC_VECT(TMU0_TUNI2, 0x440),
	INTC_VECT(USB0, 0xa20),
};

static struct intc_mask_reg mask_registers[] __initdata = {
	{ 0xa4080090, 0xa40800d0, 8, /* IMR4 / IMCR4 */
	  { 0, TMU0_TUNI2, TMU0_TUNI1, TMU0_TUNI0, 0, 0, 0, 0 } },
	{ 0xa40800a4, 0xa40800e4, 8, /* IMR9 / IMCR9 */
	  { 0, 0, 0, 0, 0, 0, USB0, 0 } },
};

static struct intc_prio_reg prio_registers[] __initdata = {
	{ 0xa4080000, 0, 16, 4, /* IPRA */
	  { TMU0_TUNI0, TMU0_TUNI1, TMU0_TUNI2, 0 } },
	{ 0xa4080014, 0, 16, 4, /* IPRF */
	  { 0, 0, USB0, 0 } },
};

static DECLARE_INTC_DESC(intc_desc, "sh7305", vectors, NULL,
			 mask_registers, prio_registers, NULL);

void __init plat_irq_setup(void)
{
	int i;

	/*
	 * The Casio OS leaves its own timers, the RTC, the key scanner
	 * and the USB module configured. Silence every source so that
	 * only what Linux registers below can reach the CPU.
	 */
	for (i = 0; i < 12; i++)
		__raw_writew(0, INTC_IPR(i));
	for (i = 0; i < 13; i++)
		__raw_writeb(0xff, INTC_IMR(i));

	register_intc_controller(&intc_desc);
}
