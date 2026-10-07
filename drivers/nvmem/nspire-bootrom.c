// SPDX-License-Identifier: GPL-2.0-only
/*
 * TI-Nspire boot1 ROM reader
 *
 * Exposes the 512 KiB mask ROM at physical address 0 (boot1) read-only
 * through nvmem, as /sys/bus/nvmem/devices/nspire-bootrom/nvmem.
 *
 * Copyright (C) 2012 Daniel Tang <tangrs@tangrs.id.au>
 * Ported from /proc/boot1_rom of the legacy arch/arm/mach-nspire kernel.
 */

#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/nvmem-provider.h>
#include <linux/platform_device.h>

static int nspire_bootrom_read(void *context, unsigned int offset, void *val,
			       size_t bytes)
{
	void __iomem *base = context;

	memcpy_fromio(val, base + offset, bytes);

	return 0;
}

static int nspire_bootrom_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nvmem_config config = { };
	struct resource *res;
	void __iomem *base;

	base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	config.name = "nspire-bootrom";
	config.id = NVMEM_DEVID_NONE;
	config.dev = dev;
	config.owner = THIS_MODULE;
	config.read_only = true;
	config.root_only = true;
	config.type = NVMEM_TYPE_UNKNOWN;
	config.word_size = 1;
	config.stride = 1;
	config.size = resource_size(res);
	config.reg_read = nspire_bootrom_read;
	config.priv = base;

	return PTR_ERR_OR_ZERO(devm_nvmem_register(dev, &config));
}

static const struct of_device_id nspire_bootrom_of_match[] = {
	{ .compatible = "ti,nspire-bootrom" },
	{ }
};
MODULE_DEVICE_TABLE(of, nspire_bootrom_of_match);

static struct platform_driver nspire_bootrom_driver = {
	.probe	= nspire_bootrom_probe,
	.driver	= {
		.name		= "nspire-bootrom",
		.of_match_table	= nspire_bootrom_of_match,
	},
};
module_platform_driver(nspire_bootrom_driver);

MODULE_AUTHOR("Daniel Tang <tangrs@tangrs.id.au>");
MODULE_DESCRIPTION("TI-Nspire boot1 ROM reader");
MODULE_LICENSE("GPL");
