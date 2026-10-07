// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Driver for the ADC in TI-Nspire calculators
 *
 * The ADC has seven channels. Channels 1 ("VBATT"), 2 ("VSYS") and
 * 4 ("B12") are used to monitor the batteries, channel 3 tells which
 * keypad is attached.
 *
 * Author: Fabian Vogt <fabian@ritter-vogt.de>
 * Ported to device tree / current IIO API from the legacy
 * arch/arm/mach-nspire kernel.
 */

#include <linux/bits.h>
#include <linux/completion.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iio/iio.h>
#include <linux/jiffies.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/units.h>

#define NSPIRE_ADC_INT_MASKED_STATUS	0x000
#define NSPIRE_ADC_INT_RAW_STATUS	0x004	/* write 1s to acknowledge */
#define NSPIRE_ADC_INT_ENABLE		0x008

#define NSPIRE_ADC_CHANNEL_BASE		0x100
#define NSPIRE_ADC_CHANNEL_STRIDE	0x020
#define NSPIRE_ADC_CHANNEL_CMD		0x000
#define NSPIRE_ADC_CHANNEL_VALUE	0x010

#define NSPIRE_ADC_CMD_MEASURE		BIT(0)

#define NSPIRE_ADC_NUM_CHANNELS		7
/* Four interrupt status bits per channel */
#define NSPIRE_ADC_INT_ALL		GENMASK(4 * NSPIRE_ADC_NUM_CHANNELS - 1, 0)

#define NSPIRE_ADC_TIMEOUT		msecs_to_jiffies(100)

struct nspire_adc {
	void __iomem *base;
	struct completion done;
	struct mutex lock;	/* one conversion at a time */
};

static irqreturn_t nspire_adc_isr(int irq, void *dev_id)
{
	struct nspire_adc *adc = dev_id;
	u32 status;

	status = readl(adc->base + NSPIRE_ADC_INT_MASKED_STATUS);
	if (!status)
		return IRQ_NONE;

	writel(status, adc->base + NSPIRE_ADC_INT_RAW_STATUS);
	complete(&adc->done);

	return IRQ_HANDLED;
}

static int nspire_adc_convert(struct nspire_adc *adc,
			      const struct iio_chan_spec *chan, int *val)
{
	void __iomem *regs = adc->base + chan->address;

	guard(mutex)(&adc->lock);

	reinit_completion(&adc->done);
	writel(NSPIRE_ADC_CMD_MEASURE, regs + NSPIRE_ADC_CHANNEL_CMD);

	if (!wait_for_completion_timeout(&adc->done, NSPIRE_ADC_TIMEOUT))
		return -ETIMEDOUT;

	*val = readl(regs + NSPIRE_ADC_CHANNEL_VALUE);

	return 0;
}

static int nspire_adc_read_raw(struct iio_dev *indio_dev,
			       const struct iio_chan_spec *chan,
			       int *val, int *val2, long mask)
{
	struct nspire_adc *adc = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = nspire_adc_convert(adc, chan, val);
		if (ret)
			return ret;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		/*
		 * 310 units per volt, except for the two battery channels
		 * which read 155 units per volt. IIO wants millivolts.
		 */
		*val = MILLI;
		*val2 = (chan->channel == 1 || chan->channel == 2) ? 155 : 310;
		return IIO_VAL_FRACTIONAL;
	}

	return -EINVAL;
}

#define NSPIRE_ADC_CHANNEL(nr, name) {					\
	.type			= IIO_VOLTAGE,				\
	.indexed		= 1,					\
	.channel		= (nr),					\
	.address		= NSPIRE_ADC_CHANNEL_BASE +		\
				  (nr) * NSPIRE_ADC_CHANNEL_STRIDE,	\
	.datasheet_name		= (name),				\
	.info_mask_separate	= BIT(IIO_CHAN_INFO_RAW) |		\
				  BIT(IIO_CHAN_INFO_SCALE),		\
}

static const struct iio_chan_spec nspire_adc_channels[] = {
	NSPIRE_ADC_CHANNEL(0, NULL),
	NSPIRE_ADC_CHANNEL(1, "VBATT"),
	NSPIRE_ADC_CHANNEL(2, "VSYS"),
	NSPIRE_ADC_CHANNEL(3, "KEYPAD"),
	NSPIRE_ADC_CHANNEL(4, "B12"),
	NSPIRE_ADC_CHANNEL(5, NULL),
	NSPIRE_ADC_CHANNEL(6, NULL),
};

static const struct iio_info nspire_adc_info = {
	.read_raw = nspire_adc_read_raw,
};

static void nspire_adc_disable_irqs(void *data)
{
	struct nspire_adc *adc = data;

	writel(0, adc->base + NSPIRE_ADC_INT_ENABLE);
}

static int nspire_adc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iio_dev *indio_dev;
	struct nspire_adc *adc;
	int irq, ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*adc));
	if (!indio_dev)
		return -ENOMEM;

	adc = iio_priv(indio_dev);
	init_completion(&adc->done);
	ret = devm_mutex_init(dev, &adc->lock);
	if (ret)
		return ret;

	adc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(adc->base))
		return PTR_ERR(adc->base);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	/* Start from a clean state before the handler can run. */
	writel(0, adc->base + NSPIRE_ADC_INT_ENABLE);
	writel(NSPIRE_ADC_INT_ALL, adc->base + NSPIRE_ADC_INT_RAW_STATUS);

	ret = devm_request_irq(dev, irq, nspire_adc_isr, 0, dev_name(dev), adc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request IRQ\n");

	writel(NSPIRE_ADC_INT_ALL, adc->base + NSPIRE_ADC_INT_ENABLE);
	ret = devm_add_action_or_reset(dev, nspire_adc_disable_irqs, adc);
	if (ret)
		return ret;

	indio_dev->name = "nspire-adc";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &nspire_adc_info;
	indio_dev->channels = nspire_adc_channels;
	indio_dev->num_channels = ARRAY_SIZE(nspire_adc_channels);

	return devm_iio_device_register(dev, indio_dev);
}

static const struct of_device_id nspire_adc_of_match[] = {
	{ .compatible = "ti,nspire-adc" },
	{ }
};
MODULE_DEVICE_TABLE(of, nspire_adc_of_match);

static struct platform_driver nspire_adc_driver = {
	.probe	= nspire_adc_probe,
	.driver	= {
		.name		= "nspire-adc",
		.of_match_table	= nspire_adc_of_match,
	},
};
module_platform_driver(nspire_adc_driver);

MODULE_AUTHOR("Fabian Vogt <fabian@ritter-vogt.de>");
MODULE_DESCRIPTION("TI-Nspire ADC driver");
MODULE_LICENSE("GPL");
