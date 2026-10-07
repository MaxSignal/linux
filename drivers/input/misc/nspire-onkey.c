// SPDX-License-Identifier: GPL-2.0-only
/*
 * ON key driver for TI-Nspire calculators
 *
 * The ON key is not part of the keypad matrix. It is wired to the power
 * management block, which raises an interrupt when the key is pressed
 * but not when it is released, so the key state is polled until release.
 *
 * Copyright (C) 2012 Daniel Tang <tangrs@tangrs.id.au>
 * Ported to device tree / syscon from the legacy arch/arm/mach-nspire
 * kernel.
 */

#include <linux/bits.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/mfd/syscon.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

/* Power management block registers */
#define NSPIRE_PWR_ON_INT_ENABLE	0x10
#define NSPIRE_PWR_INT_STATUS		0x14	/* write 1s to acknowledge */
#define NSPIRE_PWR_ON_STATE		0x28

#define NSPIRE_PWR_INT_ON		BIT(0)
/*
 * Set once a clock speed change has taken effect. It cannot be masked and
 * shares the interrupt line with the ON key, so it is acknowledged here too.
 */
#define NSPIRE_PWR_INT_CLOCK		BIT(1)
#define NSPIRE_PWR_ON_RELEASED		BIT(4)

#define NSPIRE_ONKEY_POLL_MS		10

struct nspire_onkey {
	struct input_dev *input;
	struct regmap *regmap;
	struct delayed_work poll_work;
};

static void nspire_onkey_update(struct nspire_onkey *onkey)
{
	unsigned int state;
	bool pressed;

	if (regmap_read(onkey->regmap, NSPIRE_PWR_ON_STATE, &state))
		return;

	pressed = !(state & NSPIRE_PWR_ON_RELEASED);
	input_report_key(onkey->input, KEY_POWER, pressed);
	input_sync(onkey->input);

	/* There is no release interrupt, so keep watching the key. */
	if (pressed)
		mod_delayed_work(system_dfl_wq, &onkey->poll_work,
				 msecs_to_jiffies(NSPIRE_ONKEY_POLL_MS));
}

static void nspire_onkey_poll(struct work_struct *work)
{
	struct nspire_onkey *onkey = container_of(to_delayed_work(work),
						  struct nspire_onkey,
						  poll_work);

	nspire_onkey_update(onkey);
}

static irqreturn_t nspire_onkey_irq(int irq, void *dev_id)
{
	struct nspire_onkey *onkey = dev_id;
	unsigned int status;

	if (regmap_read(onkey->regmap, NSPIRE_PWR_INT_STATUS, &status))
		return IRQ_NONE;

	status &= NSPIRE_PWR_INT_ON | NSPIRE_PWR_INT_CLOCK;
	if (!status)
		return IRQ_NONE;

	regmap_write(onkey->regmap, NSPIRE_PWR_INT_STATUS, status);

	if (status & NSPIRE_PWR_INT_ON)
		nspire_onkey_update(onkey);

	return IRQ_HANDLED;
}

static void nspire_onkey_disable(void *data)
{
	struct nspire_onkey *onkey = data;

	regmap_write(onkey->regmap, NSPIRE_PWR_ON_INT_ENABLE, 0);
	cancel_delayed_work_sync(&onkey->poll_work);
}

static int nspire_onkey_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nspire_onkey *onkey;
	int irq, ret;

	if (!dev->parent)
		return -ENODEV;

	onkey = devm_kzalloc(dev, sizeof(*onkey), GFP_KERNEL);
	if (!onkey)
		return -ENOMEM;

	onkey->regmap = syscon_node_to_regmap(dev->parent->of_node);
	if (IS_ERR(onkey->regmap))
		return dev_err_probe(dev, PTR_ERR(onkey->regmap),
				     "failed to get power management regmap\n");

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	INIT_DELAYED_WORK(&onkey->poll_work, nspire_onkey_poll);

	onkey->input = devm_input_allocate_device(dev);
	if (!onkey->input)
		return -ENOMEM;

	onkey->input->name = "nspire-onkey";
	onkey->input->phys = "nspire-onkey/input0";
	onkey->input->id.bustype = BUS_HOST;
	input_set_capability(onkey->input, EV_KEY, KEY_POWER);

	ret = input_register_device(onkey->input);
	if (ret)
		return ret;

	/*
	 * Registered before the IRQ so that on teardown the handler is gone
	 * before the poll work is cancelled.
	 */
	ret = devm_add_action_or_reset(dev, nspire_onkey_disable, onkey);
	if (ret)
		return ret;

	/* Discard a stale press before the interrupt can be delivered. */
	regmap_write(onkey->regmap, NSPIRE_PWR_INT_STATUS, NSPIRE_PWR_INT_ON);

	ret = devm_request_irq(dev, irq, nspire_onkey_irq, 0, "nspire-onkey",
			       onkey);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request IRQ\n");

	return regmap_write(onkey->regmap, NSPIRE_PWR_ON_INT_ENABLE,
			    NSPIRE_PWR_INT_ON);
}

static const struct of_device_id nspire_onkey_of_match[] = {
	{ .compatible = "ti,nspire-power-button" },
	{ }
};
MODULE_DEVICE_TABLE(of, nspire_onkey_of_match);

static struct platform_driver nspire_onkey_driver = {
	.probe	= nspire_onkey_probe,
	.driver	= {
		.name		= "nspire-onkey",
		.of_match_table	= nspire_onkey_of_match,
	},
};
module_platform_driver(nspire_onkey_driver);

MODULE_AUTHOR("Daniel Tang <tangrs@tangrs.id.au>");
MODULE_DESCRIPTION("TI-Nspire ON key driver");
MODULE_LICENSE("GPL");
