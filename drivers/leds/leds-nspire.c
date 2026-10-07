// SPDX-License-Identifier: GPL-2.0-only
/*
 * Status LED of TI-Nspire calculators
 *
 * A green and a red LED, used by the TI-Nspire OS for its press-to-test mode
 * (both on: yellow). Each can be forced on or off, or driven from a 32 bit
 * blink pattern; only the force bits are used here.
 */

#include <linux/bits.h>
#include <linux/container_of.h>
#include <linux/io.h>
#include <linux/leds.h>
#include <linux/mfd/syscon.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/spinlock.h>

#define NSPIRE_LED_CTRL			0xb00
#define NSPIRE_LED_GREEN_BLINK		(BIT(0) | BIT(1) | BIT(6))
#define NSPIRE_LED_GREEN_OFF		BIT(2)
#define NSPIRE_LED_RED_OFF		BIT(3)
#define NSPIRE_LED_GREEN_ON		BIT(4)
#define NSPIRE_LED_RED_ON		BIT(5)
#define NSPIRE_LED_RED_BLINK		(BIT(9) | BIT(10) | BIT(12) | BIT(13))

/* Peripheral bus access gate in the power management block */
#define NSPIRE_PWR_BUS_DISABLE		0x18
#define NSPIRE_PWR_BUS_DISABLE_LED	BIT(22)

#define NSPIRE_LED_GREEN		0
#define NSPIRE_LED_RED			1
#define NSPIRE_LED_COUNT		2

struct nspire_leds;

struct nspire_led {
	struct led_classdev cdev;
	struct nspire_leds *priv;
	u32 on, off;
};

struct nspire_leds {
	void __iomem *base;
	spinlock_t lock;	/* serializes control register updates */
	struct nspire_led leds[NSPIRE_LED_COUNT];
};

static void nspire_led_set(struct led_classdev *cdev,
			   enum led_brightness brightness)
{
	struct nspire_led *led = container_of(cdev, struct nspire_led, cdev);
	struct nspire_leds *priv = led->priv;
	unsigned long flags;
	u32 ctrl;

	spin_lock_irqsave(&priv->lock, flags);
	ctrl = readl(priv->base + NSPIRE_LED_CTRL);
	ctrl &= ~(led->on | led->off);
	ctrl |= brightness ? led->on : led->off;
	writel(ctrl, priv->base + NSPIRE_LED_CTRL);
	spin_unlock_irqrestore(&priv->lock, flags);
}

static enum led_brightness nspire_led_get(struct led_classdev *cdev)
{
	struct nspire_led *led = container_of(cdev, struct nspire_led, cdev);
	u32 ctrl = readl(led->priv->base + NSPIRE_LED_CTRL);

	return (ctrl & led->on) && !(ctrl & led->off) ? LED_ON : LED_OFF;
}

static int nspire_leds_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nspire_leds *priv;
	struct regmap *pwr;
	u32 ctrl;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	spin_lock_init(&priv->lock);

	/* The OS may have cut the LED off the bus */
	pwr = syscon_regmap_lookup_by_compatible("ti,nspire-pwr");
	if (!IS_ERR(pwr))
		regmap_clear_bits(pwr, NSPIRE_PWR_BUS_DISABLE,
				  NSPIRE_PWR_BUS_DISABLE_LED);

	/* Take over from any blink pattern, keeping what is forced on */
	ctrl = readl(priv->base + NSPIRE_LED_CTRL);
	ctrl &= ~(NSPIRE_LED_GREEN_BLINK | NSPIRE_LED_RED_BLINK);
	writel(ctrl, priv->base + NSPIRE_LED_CTRL);

	priv->leds[NSPIRE_LED_GREEN].on = NSPIRE_LED_GREEN_ON;
	priv->leds[NSPIRE_LED_GREEN].off = NSPIRE_LED_GREEN_OFF;
	priv->leds[NSPIRE_LED_RED].on = NSPIRE_LED_RED_ON;
	priv->leds[NSPIRE_LED_RED].off = NSPIRE_LED_RED_OFF;

	device_for_each_child_node_scoped(dev, child) {
		struct led_init_data init_data = { .fwnode = child };
		struct nspire_led *led;
		u32 reg;

		ret = fwnode_property_read_u32(child, "reg", &reg);
		if (ret || reg >= NSPIRE_LED_COUNT)
			return dev_err_probe(dev, -EINVAL, "invalid LED %pfw\n", child);

		led = &priv->leds[reg];
		led->priv = priv;
		led->cdev.max_brightness = LED_ON;
		led->cdev.brightness_set = nspire_led_set;
		led->cdev.brightness_get = nspire_led_get;

		switch (led_init_default_state_get(child)) {
		case LEDS_DEFSTATE_ON:
			led->cdev.brightness = LED_ON;
			break;
		case LEDS_DEFSTATE_KEEP:
			led->cdev.brightness = nspire_led_get(&led->cdev);
			break;
		default:
			led->cdev.brightness = LED_OFF;
			break;
		}
		nspire_led_set(&led->cdev, led->cdev.brightness);

		ret = devm_led_classdev_register_ext(dev, &led->cdev, &init_data);
		if (ret)
			return dev_err_probe(dev, ret, "failed to register %pfw\n", child);
	}

	return 0;
}

static const struct of_device_id nspire_leds_of_match[] = {
	{ .compatible = "ti,nspire-led" },
	{ }
};
MODULE_DEVICE_TABLE(of, nspire_leds_of_match);

static struct platform_driver nspire_leds_driver = {
	.probe	= nspire_leds_probe,
	.driver	= {
		.name		= "nspire-led",
		.of_match_table	= nspire_leds_of_match,
	},
};
module_platform_driver(nspire_leds_driver);

MODULE_DESCRIPTION("TI-Nspire status LED driver");
MODULE_LICENSE("GPL");
