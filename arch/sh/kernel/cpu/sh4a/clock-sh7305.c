// SPDX-License-Identifier: GPL-2.0
/*
 * SH7305 clock framework support
 *
 * The SH7305 CPG is derived from the SH-Mobile parts, but the PLL ratio
 * and the dividers are encoded differently: the PLL multiplies by
 * (STC + 1) and every divider field n divides by 2^(n + 1). The FLL is
 * fed from the 32.768 kHz RTC crystal. Register semantics follow the
 * community documentation of the Casio fx-CG/fx-9860GIII calculators
 * (gint, and the fx-9750GIII Linux port by Artem Novak).
 */
#include <linux/clkdev.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/sh_clk.h>
#include <asm/clock.h>

#define FRQCRA		0xa4150000
#define PLLCR		0xa4150024
#define FLLFRQ		0xa4150050

#define USBCLKCR	0xa4150014
#define MSTPCR2		0xa4150038
#define MSELCRA		0xa4050180
#define MSELCRB		0xa4050182
#define UPONCR		0xa40501d4
#define USB_SYSCFG	0xa4d80000
#define USB_REG_C2	0xa4d800c2

#define USBCLKCR_CLKSTP	(1 << 8)
#define MSTP_USB0	(1 << 20)
#define SYSCFG_SCKE	(1 << 10)

#define PLLCR_PLLE	(1 << 14)
#define PLLCR_FLLE	(1 << 12)

static unsigned long sh7305_pll_out(void)
{
	u32 frqcr = __raw_readl(FRQCRA);
	u32 pllcr = __raw_readl(PLLCR);
	u32 fllfrq = __raw_readl(FLLFRQ);
	unsigned long rate = 32768;
	unsigned int fll = fllfrq & 0x7ff;

	if (((fllfrq >> 14) & 3) == 1)
		fll >>= 1;
	if (pllcr & PLLCR_FLLE)
		rate *= fll;
	if (pllcr & PLLCR_PLLE)
		rate *= ((frqcr >> 24) & 0x3f) + 1;

	return rate;
}

static unsigned long sh7305_div(struct clk *clk, unsigned int shift)
{
	return clk->parent->rate >> (((__raw_readl(FRQCRA) >> shift) & 0xf) + 1);
}

static void master_clk_init(struct clk *clk)
{
	clk->rate = sh7305_pll_out();
}

static struct sh_clk_ops sh7305_master_clk_ops = {
	.init		= master_clk_init,
};

static unsigned long module_clk_recalc(struct clk *clk)
{
	return sh7305_div(clk, 0);		/* P1FC */
}

static struct sh_clk_ops sh7305_module_clk_ops = {
	.recalc		= module_clk_recalc,
};

static unsigned long bus_clk_recalc(struct clk *clk)
{
	return sh7305_div(clk, 8);		/* BFC */
}

static struct sh_clk_ops sh7305_bus_clk_ops = {
	.recalc		= bus_clk_recalc,
};

static unsigned long cpu_clk_recalc(struct clk *clk)
{
	return sh7305_div(clk, 20);		/* IFC */
}

static struct sh_clk_ops sh7305_cpu_clk_ops = {
	.recalc		= cpu_clk_recalc,
};

static struct sh_clk_ops *sh7305_clk_ops[] = {
	&sh7305_master_clk_ops,
	&sh7305_module_clk_ops,
	&sh7305_bus_clk_ops,
	&sh7305_cpu_clk_ops,
};

/*
 * USB0: the function controller (r8a66597 compatible). Its clock, module
 * stop bit, pin function and power control are only known from the
 * calculator community (gint); the Casio OS uses this same sequence.
 */
static int usb0_clk_enable(struct clk *clk)
{
	__raw_writew(__raw_readw(MSELCRA) & ~0x00c0, MSELCRA);
	__raw_writew(__raw_readw(MSELCRB) & ~0xc000, MSELCRB);
	__raw_writel(__raw_readl(USBCLKCR) & ~USBCLKCR_CLKSTP, USBCLKCR);
	mdelay(1);
	__raw_writel(__raw_readl(MSTPCR2) & ~MSTP_USB0, MSTPCR2);
	__raw_writew(0x0600, UPONCR);

	/* SCKE gates every other register of the module */
	__raw_writew(__raw_readw(USB_SYSCFG) | SYSCFG_SCKE, USB_SYSCFG);
	udelay(1);
	__raw_writew(0x0020, USB_REG_C2);
	return 0;
}

static void usb0_clk_disable(struct clk *clk)
{
	__raw_writew(0, UPONCR);
	mdelay(1);
	__raw_writel(__raw_readl(MSTPCR2) | MSTP_USB0, MSTPCR2);
	__raw_writel(__raw_readl(USBCLKCR) | USBCLKCR_CLKSTP, USBCLKCR);
	mdelay(1);
	/* what the Casio OS leaves when USB is off */
	__raw_writew(__raw_readw(MSELCRB) | 0xc000, MSELCRB);
	__raw_writew(__raw_readw(MSELCRA) | 0x0040, MSELCRA);
}

static struct sh_clk_ops usb0_clk_ops = {
	.enable		= usb0_clk_enable,
	.disable	= usb0_clk_disable,
};

static struct clk usb0_clk = {
	.ops		= &usb0_clk_ops,
};

static struct clk_lookup sh7305_lookups[] = {
	CLKDEV_CON_ID("usb0", &usb0_clk),
};

void __init arch_init_clk_ops(struct sh_clk_ops **ops, int idx)
{
	if (idx < ARRAY_SIZE(sh7305_clk_ops))
		*ops = sh7305_clk_ops[idx];
}

int __init arch_clk_init(void)
{
	int ret = cpg_clk_init();

	ret |= clk_register(&usb0_clk);
	clkdev_add_table(sh7305_lookups, ARRAY_SIZE(sh7305_lookups));
	return ret;
}
