// SPDX-License-Identifier: GPL-2.0-only
/*
 * LCD backlight / contrast driver for TI-Nspire calculators
 *
 * The same register controls the backlight brightness on the colour CX
 * models and the contrast of the grayscale LCD on the classic models.
 *
 * Copyright (C) 2012 Daniel Tang <tangrs@tangrs.id.au>
 * Ported to device tree / backlight class from the legacy
 * arch/arm/mach-nspire kernel (/proc/backlight and /proc/contrast).
 */

#include <linux/backlight.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>

#define NSPIRE_BL_REG		0x20

struct nspire_bl_variant {
	const char *name;
	u32 reg_min;		/* register value for brightness 0 */
	u32 reg_max;
	u32 reg_default;
	/* Contrast should not drop to the minimum when the display blanks. */
	bool is_contrast;
};

struct nspire_bl {
	void __iomem *base;
	const struct nspire_bl_variant *variant;
};

static int nspire_bl_update_status(struct backlight_device *bl)
{
	struct nspire_bl *priv = bl_get_data(bl);
	int level;

	if (priv->variant->is_contrast)
		level = bl->props.brightness;
	else
		level = backlight_get_brightness(bl);

	writel(priv->variant->reg_min + level, priv->base + NSPIRE_BL_REG);

	return 0;
}

static const struct backlight_ops nspire_bl_ops = {
	.options	= BL_CORE_SUSPENDRESUME,
	.update_status	= nspire_bl_update_status,
};

static int nspire_bl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct backlight_properties props = { };
	const struct nspire_bl_variant *variant;
	struct backlight_device *bl;
	struct nspire_bl *priv;
	u32 cur;

	variant = device_get_match_data(dev);
	if (!variant)
		return -ENODEV;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->variant = variant;
	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	/* Keep whatever the boot environment set up if it is sane. */
	cur = readl(priv->base + NSPIRE_BL_REG);
	if (cur < variant->reg_min || cur > variant->reg_max)
		cur = variant->reg_default;

	props.type = BACKLIGHT_RAW;
	props.max_brightness = variant->reg_max - variant->reg_min;
	props.brightness = cur - variant->reg_min;
	props.power = BACKLIGHT_POWER_ON;

	bl = devm_backlight_device_register(dev, variant->name, dev, priv,
					    &nspire_bl_ops, &props);
	if (IS_ERR(bl))
		return dev_err_probe(dev, PTR_ERR(bl),
				     "failed to register backlight\n");

	return backlight_update_status(bl);
}

/* Ranges from the Hackspire documentation of register 0x900F0020. */
static const struct nspire_bl_variant nspire_cx_backlight = {
	.name		= "nspire-backlight",
	.reg_min	= 0x100,	/* backlight off */
	.reg_max	= 0x1d0,	/* about maximum brightness */
	.reg_default	= 0x174,
};

static const struct nspire_bl_variant nspire_classic_contrast = {
	.name		= "nspire-contrast",
	.reg_min	= 0x6b,
	.reg_max	= 0x95,
	.reg_default	= 0x80,
	.is_contrast	= true,
};

static const struct of_device_id nspire_bl_of_match[] = {
	{ .compatible = "ti,nspire-cx-backlight", .data = &nspire_cx_backlight },
	{ .compatible = "ti,nspire-classic-contrast", .data = &nspire_classic_contrast },
	{ }
};
MODULE_DEVICE_TABLE(of, nspire_bl_of_match);

static struct platform_driver nspire_bl_driver = {
	.probe	= nspire_bl_probe,
	.driver	= {
		.name		= "nspire-backlight",
		.of_match_table	= nspire_bl_of_match,
	},
};
module_platform_driver(nspire_bl_driver);

MODULE_AUTHOR("Daniel Tang <tangrs@tangrs.id.au>");
MODULE_DESCRIPTION("TI-Nspire LCD backlight/contrast driver");
MODULE_LICENSE("GPL");
