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
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/sh_clk.h>
#include <asm/clock.h>

#define FRQCRA		0xa4150000
#define PLLCR		0xa4150024
#define FLLFRQ		0xa4150050

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

void __init arch_init_clk_ops(struct sh_clk_ops **ops, int idx)
{
	if (idx < ARRAY_SIZE(sh7305_clk_ops))
		*ops = sh7305_clk_ops[idx];
}

int __init arch_clk_init(void)
{
	return cpg_clk_init();
}
