// SPDX-License-Identifier: GPL-2.0-only
/*
 * RTC driver for TI-Nspire calculators
 *
 * The RTC looks like an ARM PL031 (and even reports the PL031 PrimeCell
 * ID), but the interrupt registers are laid out differently, so the
 * pl031 driver cannot be used.
 *
 * Copyright (C) 2012 Daniel Tang <tangrs@tangrs.id.au>
 * Ported to device tree / current RTC API from the legacy
 * arch/arm/mach-nspire kernel.
 */

#include <linux/bits.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeirq.h>
#include <linux/rtc.h>
#include <linux/spinlock.h>

#define NSPIRE_RTC_CURR		0x00	/* current time, seconds */
#define NSPIRE_RTC_ALRM		0x04	/* alarm value */
#define NSPIRE_RTC_SET		0x08	/* load value for the counter */
#define NSPIRE_RTC_INTMSK	0x0c	/* interrupt mask (1 bit) */
#define NSPIRE_RTC_INTSTS	0x10	/* masked status, write 1 to ack */
#define NSPIRE_RTC_STS		0x14	/* pending register writes */

#define NSPIRE_RTC_STS_SET	BIT(0)

/*
 * The interrupt mask has to stay set for the counter to tick, so the alarm
 * is disabled by programming a value the counter never reaches.
 */
#define NSPIRE_RTC_ALRM_OFF	U32_MAX

/* Loading a new time takes "a couple of seconds" to become visible. */
#define NSPIRE_RTC_SET_TIMEOUT_US	(5 * USEC_PER_SEC)

struct nspire_rtc {
	void __iomem *base;
	struct rtc_device *rtc;
	spinlock_t lock;	/* protects alarm and alarm_enabled */
	u32 alarm;
	bool alarm_enabled;
};

static int nspire_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct nspire_rtc *priv = dev_get_drvdata(dev);

	rtc_time64_to_tm(readl(priv->base + NSPIRE_RTC_CURR), tm);

	return 0;
}

static int nspire_rtc_wait_set_done(struct nspire_rtc *priv)
{
	u32 sts;

	return readl_poll_timeout(priv->base + NSPIRE_RTC_STS, sts,
				  !(sts & NSPIRE_RTC_STS_SET), 10 * USEC_PER_MSEC,
				  NSPIRE_RTC_SET_TIMEOUT_US);
}

static int nspire_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct nspire_rtc *priv = dev_get_drvdata(dev);
	int ret;

	/* Don't stack a new load on top of one still in flight. */
	ret = nspire_rtc_wait_set_done(priv);
	if (ret)
		return ret;

	writel(rtc_tm_to_time64(tm), priv->base + NSPIRE_RTC_SET);

	ret = nspire_rtc_wait_set_done(priv);
	if (ret)
		dev_err(dev, "timed out waiting for the new time to load\n");

	return ret;
}

/* Called with priv->lock held. */
static void nspire_rtc_program_alarm(struct nspire_rtc *priv, bool enable)
{
	/* Drop anything that fired against the previous setting. */
	writel(1, priv->base + NSPIRE_RTC_INTSTS);
	writel(enable ? priv->alarm : NSPIRE_RTC_ALRM_OFF,
	       priv->base + NSPIRE_RTC_ALRM);
	priv->alarm_enabled = enable;
}

static int nspire_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct nspire_rtc *priv = dev_get_drvdata(dev);
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	rtc_time64_to_tm(priv->alarm, &alrm->time);
	alrm->enabled = priv->alarm_enabled;
	alrm->pending = readl(priv->base + NSPIRE_RTC_INTSTS) & 1;
	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static int nspire_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct nspire_rtc *priv = dev_get_drvdata(dev);
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	priv->alarm = rtc_tm_to_time64(&alrm->time);
	nspire_rtc_program_alarm(priv, alrm->enabled);
	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static int nspire_rtc_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct nspire_rtc *priv = dev_get_drvdata(dev);
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	nspire_rtc_program_alarm(priv, enabled);
	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static const struct rtc_class_ops nspire_rtc_ops = {
	.read_time		= nspire_rtc_read_time,
	.set_time		= nspire_rtc_set_time,
	.read_alarm		= nspire_rtc_read_alarm,
	.set_alarm		= nspire_rtc_set_alarm,
	.alarm_irq_enable	= nspire_rtc_alarm_irq_enable,
};

static irqreturn_t nspire_rtc_irq(int irq, void *dev_id)
{
	struct nspire_rtc *priv = dev_id;
	bool fired;

	spin_lock(&priv->lock);
	if (!(readl(priv->base + NSPIRE_RTC_INTSTS) & 1)) {
		spin_unlock(&priv->lock);
		return IRQ_NONE;
	}

	fired = priv->alarm_enabled;
	/* One-shot: the RTC core re-arms the next alarm itself. */
	nspire_rtc_program_alarm(priv, false);
	spin_unlock(&priv->lock);

	if (fired)
		rtc_update_irq(priv->rtc, 1, RTC_IRQF | RTC_AF);

	return IRQ_HANDLED;
}

static int nspire_rtc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nspire_rtc *priv;
	int irq, ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	spin_lock_init(&priv->lock);
	platform_set_drvdata(pdev, priv);

	/* The interrupt mask needs to be always on for the clock to tick. */
	writel(1, priv->base + NSPIRE_RTC_INTMSK);
	nspire_rtc_program_alarm(priv, false);

	priv->rtc = devm_rtc_allocate_device(dev);
	if (IS_ERR(priv->rtc))
		return PTR_ERR(priv->rtc);

	priv->rtc->ops = &nspire_rtc_ops;
	priv->rtc->range_max = U32_MAX;

	irq = platform_get_irq_optional(pdev, 0);
	if (irq == -EPROBE_DEFER)
		return irq;

	if (irq > 0) {
		ret = devm_request_irq(dev, irq, nspire_rtc_irq, 0,
				       dev_name(dev), priv);
		if (ret)
			return dev_err_probe(dev, ret, "failed to request IRQ\n");

		device_init_wakeup(dev, true);
		ret = devm_pm_set_wake_irq(dev, irq);
		if (ret)
			return ret;
	} else {
		dev_info(dev, "no IRQ, alarms disabled\n");
		clear_bit(RTC_FEATURE_ALARM, priv->rtc->features);
	}

	return devm_rtc_register_device(priv->rtc);
}

static const struct of_device_id nspire_rtc_of_match[] = {
	{ .compatible = "ti,nspire-rtc" },
	{ }
};
MODULE_DEVICE_TABLE(of, nspire_rtc_of_match);

static struct platform_driver nspire_rtc_driver = {
	.probe	= nspire_rtc_probe,
	.driver	= {
		.name		= "nspire-rtc",
		.of_match_table	= nspire_rtc_of_match,
	},
};
module_platform_driver(nspire_rtc_driver);

MODULE_AUTHOR("Daniel Tang <tangrs@tangrs.id.au>");
MODULE_DESCRIPTION("RTC driver for TI-Nspire calculators");
MODULE_LICENSE("GPL");
