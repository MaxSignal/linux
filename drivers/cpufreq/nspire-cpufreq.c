// SPDX-License-Identifier: GPL-2.0-only
/*
 * CPU frequency scaling for TI-Nspire calculators
 *
 * The power management block derives the CPU clock from a base clock
 * through a base/CPU divider, and the AHB clock from the CPU clock through
 * a CPU/AHB divider (the APB runs at half the AHB clock). Only the split
 * of the total base/AHB ratio between these two dividers is changed here:
 * the AHB and APB clocks, which feed every peripheral and whose rates the
 * clock framework determined at boot, stay exactly as they are. The CPU is
 * also never clocked faster than the boot environment configured it.
 *
 * Copyright (C) 2012 Daniel Tang <tangrs@tangrs.id.au>
 * Ported from the legacy arch/arm/mach-nspire kernel.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cpufreq.h>
#include <linux/err.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/units.h>

#define NSPIRE_PWR_CLK_LOAD	0x00	/* clock setting to apply */
#define NSPIRE_PWR_CLK_CTRL	0x0c
#define NSPIRE_PWR_INT_STATUS	0x14
#define NSPIRE_PWR_CLK_CURRENT	0x24	/* clock setting in effect */

#define NSPIRE_PWR_CLK_CTRL_APPLY	4
#define NSPIRE_PWR_INT_CLOCK		BIT(1)

#define NSPIRE_CLK_BASE_CPU	GENMASK(7, 1)
#define NSPIRE_CLK_FIXED_BASE	BIT(8)
#define NSPIRE_CLK_CPU_AHB	GENMASK(14, 12)
#define NSPIRE_CLK_CLASSIC_BASE	GENMASK(20, 16)
#define NSPIRE_CLK_CX_BASE	GENMASK(20, 15)
#define NSPIRE_CLK_CX_MULT	GENMASK(22, 21)

/* The 3-bit CPU/AHB field encodes dividers 1 to 8. */
#define NSPIRE_CPU_AHB_MAX	8

#define NSPIRE_CLK_TIMEOUT_US	(100 * USEC_PER_MSEC)

struct nspire_clk_setting {
	u32 base_hz;
	u32 base_cpu;		/* base/CPU divider */
	u32 cpu_ahb;		/* CPU/AHB divider */
	u32 mult;		/* encoding of base_cpu in the register */
};

static struct regmap *nspire_pwr;
static bool nspire_is_cx;
/* driver_data holds the register value selecting each frequency. */
static struct cpufreq_frequency_table nspire_freq_table[NSPIRE_CPU_AHB_MAX + 1];

static int nspire_clk_decode(u32 val, struct nspire_clk_setting *s)
{
	/* The fixed 27/48 MHz modes use different divider semantics. */
	if (val & NSPIRE_CLK_FIXED_BASE)
		return -EOPNOTSUPP;

	if (nspire_is_cx) {
		s->base_hz = 6 * FIELD_GET(NSPIRE_CLK_CX_BASE, val) * HZ_PER_MHZ;
		s->mult = FIELD_GET(NSPIRE_CLK_CX_MULT, val);
	} else {
		s->base_hz = (300 - 6 * FIELD_GET(NSPIRE_CLK_CLASSIC_BASE, val)) *
			     HZ_PER_MHZ;
		s->mult = 2;
	}

	s->base_cpu = FIELD_GET(NSPIRE_CLK_BASE_CPU, val) * s->mult;
	s->cpu_ahb = FIELD_GET(NSPIRE_CLK_CPU_AHB, val) + 1;

	if (!s->base_hz || !s->base_cpu)
		return -EINVAL;

	return 0;
}

static unsigned int nspire_cpufreq_get(unsigned int cpu)
{
	struct nspire_clk_setting s;
	u32 val;

	if (regmap_read(nspire_pwr, NSPIRE_PWR_CLK_CURRENT, &val) ||
	    nspire_clk_decode(val, &s))
		return 0;

	return s.base_hz / s.base_cpu / HZ_PER_KHZ;
}

static int nspire_cpufreq_target(struct cpufreq_policy *policy,
				 unsigned int index)
{
	u32 val = nspire_freq_table[index].driver_data;
	u32 cur;
	int ret;

	ret = regmap_write(nspire_pwr, NSPIRE_PWR_CLK_LOAD, val);
	if (ret)
		return ret;

	ret = regmap_write(nspire_pwr, NSPIRE_PWR_CLK_CTRL,
			   NSPIRE_PWR_CLK_CTRL_APPLY);
	if (ret)
		return ret;

	/* The switch completes asynchronously, so wait for it to show up. */
	ret = regmap_read_poll_timeout(nspire_pwr, NSPIRE_PWR_CLK_CURRENT, cur,
				       cur == val, 100, NSPIRE_CLK_TIMEOUT_US);
	if (ret)
		pr_err("clock change to %#x timed out (current %#x)\n", val, cur);

	/* Completion also raises an unmaskable interrupt; clear it. */
	regmap_write(nspire_pwr, NSPIRE_PWR_INT_STATUS, NSPIRE_PWR_INT_CLOCK);

	return ret;
}

static int nspire_cpufreq_init(struct cpufreq_policy *policy)
{
	cpufreq_generic_init(policy, nspire_freq_table, 100 * NSEC_PER_USEC);

	return 0;
}

static struct cpufreq_driver nspire_cpufreq_driver = {
	.name		= "nspire",
	.flags		= CPUFREQ_NEED_INITIAL_FREQ_CHECK,
	.init		= nspire_cpufreq_init,
	.verify		= cpufreq_generic_frequency_table_verify,
	.target_index	= nspire_cpufreq_target,
	.get		= nspire_cpufreq_get,
};

/*
 * Offer every way of splitting the current base/AHB ratio between the two
 * dividers that the register can encode, leaving the AHB rate untouched and
 * the CPU no faster than it is now.
 */
static int nspire_cpufreq_build_table(u32 cur_val)
{
	struct nspire_clk_setting cur;
	unsigned int n = 0;
	u32 total, cpu_ahb, template;
	int ret;

	ret = nspire_clk_decode(cur_val, &cur);
	if (ret)
		return ret;

	template = cur_val & ~(NSPIRE_CLK_BASE_CPU | NSPIRE_CLK_CPU_AHB);
	total = cur.base_cpu * cur.cpu_ahb;

	/* Largest CPU/AHB divider first, i.e. ascending CPU frequency. */
	for (cpu_ahb = NSPIRE_CPU_AHB_MAX; cpu_ahb >= 1; cpu_ahb--) {
		u32 base_cpu, field;

		if (total % cpu_ahb)
			continue;

		base_cpu = total / cpu_ahb;
		if (base_cpu < cur.base_cpu || base_cpu % cur.mult)
			continue;

		field = base_cpu / cur.mult;
		if (field > FIELD_MAX(NSPIRE_CLK_BASE_CPU))
			continue;

		nspire_freq_table[n].frequency = cur.base_hz / base_cpu / HZ_PER_KHZ;
		nspire_freq_table[n].driver_data = template |
			FIELD_PREP(NSPIRE_CLK_BASE_CPU, field) |
			FIELD_PREP(NSPIRE_CLK_CPU_AHB, cpu_ahb - 1);
		n++;
	}
	nspire_freq_table[n].frequency = CPUFREQ_TABLE_END;

	pr_info("base %lu MHz, AHB %lu MHz, %u CPU frequencies\n",
		cur.base_hz / HZ_PER_MHZ, cur.base_hz / total / HZ_PER_MHZ, n);

	/* Nothing to scale between. */
	return n > 1 ? 0 : -ENODEV;
}

static int __init nspire_cpufreq_module_init(void)
{
	u32 val;
	int ret;

	if (of_machine_is_compatible("ti,nspire-cx"))
		nspire_is_cx = true;
	else if (!of_machine_is_compatible("ti,nspire-tp") &&
		 !of_machine_is_compatible("ti,nspire-clp"))
		return -ENODEV;

	nspire_pwr = syscon_regmap_lookup_by_compatible("ti,nspire-pwr");
	if (IS_ERR(nspire_pwr))
		return PTR_ERR(nspire_pwr);

	ret = regmap_read(nspire_pwr, NSPIRE_PWR_CLK_CURRENT, &val);
	if (ret)
		return ret;

	ret = nspire_cpufreq_build_table(val);
	if (ret) {
		pr_info("unsupported clock setting %#x, not scaling\n", val);
		return ret;
	}

	return cpufreq_register_driver(&nspire_cpufreq_driver);
}
module_init(nspire_cpufreq_module_init);

static void __exit nspire_cpufreq_module_exit(void)
{
	cpufreq_unregister_driver(&nspire_cpufreq_driver);
}
module_exit(nspire_cpufreq_module_exit);

MODULE_AUTHOR("Daniel Tang <tangrs@tangrs.id.au>");
MODULE_DESCRIPTION("TI-Nspire CPU frequency scaling driver");
MODULE_LICENSE("GPL");
