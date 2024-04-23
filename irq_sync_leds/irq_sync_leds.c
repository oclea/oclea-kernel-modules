/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * IRQ driven LEDs driver
 *
 * Copyright (c) 2019 Teknique Limited
 *
 * This file is subject to the terms and conditions of version 2 of
 * the GNU General Public License.  See the file COPYING in the main
 * directory of this archive for more details.
 *
*/

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/timer.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include <linux/of_gpio.h>
#include <linux/interrupt.h>

#define NIGHT_LED_MAX_TIME_MS 30000 // 30 seconds max possible shutter time
#define IRQ_MAX_ID 256

struct irq_sync_leds_data {
	struct device *dev;
	enum hrtimer_restart timer_state;
	struct hrtimer hr_timer;
	int irq;
	int virt_irq;
	const char *irq_name;
	int night_led_gpio;
	int night_led_gpio_active;
	int night_led_time_ms;
	int timer_inited;
};

static const struct platform_device_id irq_sync_leds_ids[] = {
	{ "irq_sync_leds", 0 },
	{}
};

enum hrtimer_restart timer_trigger_func(struct hrtimer *timer)
{
	struct irq_sync_leds_data *data =
		container_of(timer, struct irq_sync_leds_data, hr_timer);

	gpio_set_value(data->night_led_gpio, !data->night_led_gpio_active);
	return data->timer_state;
}

static irqreturn_t irq_sync_leds_interrupt_handler(int irq, void *dev_id)
{
	struct irq_sync_leds_data *data = dev_id;
	ktime_t kt = ktime_set(0, data->night_led_time_ms * NSEC_PER_MSEC);

	gpio_set_value(data->night_led_gpio, !!data->night_led_gpio_active);
	data->timer_state = HRTIMER_NORESTART;
	if (!data->timer_inited) {
		hrtimer_init(&data->hr_timer, CLOCK_MONOTONIC,
			     HRTIMER_MODE_REL);
		data->timer_inited = 1;
	}

	data->hr_timer.function = &timer_trigger_func;
	hrtimer_start(&data->hr_timer, kt, HRTIMER_MODE_REL);

	return IRQ_HANDLED;
}

static int irq_sync_leds_init(struct irq_sync_leds_data *data)
{
	int ret;
	struct irq_data *irq_data;
	int hw_irq;
	int i;

	data->virt_irq = -1;

	for (i = 0; i < IRQ_MAX_ID; i++) {
		irq_data = irq_get_irq_data(i);
		if (!irq_data) {
			continue;
		}

		hw_irq = irqd_to_hwirq(irq_data);
		if (data->irq == hw_irq) {
			data->virt_irq = i;
			break;
		}
	}

	if (data->virt_irq < 0) {
		dev_err(data->dev, "IRQ %d not valid\n", data->irq);
		return -EINVAL;
	}

	if (gpio_is_valid(data->night_led_gpio)) {
		gpio_direction_output(data->night_led_gpio, 0);
	} else {
		dev_err(data->dev, "gpio %d is invalid\n",
			data->night_led_gpio);
		return -EINVAL;
	}

	if (data->night_led_time_ms < 0 ||
	    data->night_led_time_ms > NIGHT_LED_MAX_TIME_MS) {
		dev_err(data->dev,
			"invalid night led time %d ms. Must be 0-%d ms\n",
			data->night_led_time_ms, NIGHT_LED_MAX_TIME_MS);
	}

	dev_info(data->dev, "Requesting irq: %d (%d)\n", data->virt_irq,
		 data->irq);

	ret = request_irq(data->virt_irq, &irq_sync_leds_interrupt_handler,
			  IRQF_TRIGGER_RISING | IRQF_SHARED,
			  data->irq_name ? data->irq_name : "irq-sync-leds",
			  data);
	if (ret) {
		dev_err(data->dev, "can't request IRQ(%d) %s. Err: %d!\n",
			data->virt_irq,
			data->irq_name ? data->irq_name : "NULL", ret);
		return -EINVAL;
	}

	return 0;
}

#ifdef CONFIG_OF
static int irq_sync_leds_parse_dt(struct device *dev,
				  struct irq_sync_leds_data *data)
{
	struct device_node *node = dev->of_node;
	int ret;
	int gpio, time_ms, irq;
	const char *name;
	enum of_gpio_flags flags;

	if (!node) {
		return -ENODEV;
	}

	gpio = of_get_named_gpio_flags(dev->of_node, "night-led-gpio", 0,
				       &flags);
	if (gpio == -EPROBE_DEFER) {
		return gpio;
	} else if (gpio < 0) {
		dev_err(data->dev, "missing gpio property\n");
		return -EINVAL;
	}

	if (of_property_read_s32(dev->of_node, "irq", &irq) < 0) {
		dev_err(data->dev, "missing irq property\n");
		return -EINVAL;
	}

	if (of_property_read_s32(dev->of_node, "night-led-time-ms", &time_ms) <
	    0) {
		dev_err(data->dev, "missing night-led-time-ms property\n");
		return -EINVAL;
	}

	if (gpio_is_valid(gpio)) {
		if (gpio_cansleep(gpio)) {
			dev_info(dev, "gpio %d operation may sleep\n", gpio);
		}

		ret = devm_gpio_request(dev, gpio, dev_name(dev));
		if (ret) {
			dev_err(dev, "failed to request gpio: %d\n", gpio);
			return ret;
		}
	}

	if (of_property_read_string(dev->of_node, "irq-name", &name) < 0) {
		name = NULL;
	}

	data->irq = irq;
	data->night_led_gpio = gpio;
	data->night_led_gpio_active = !!(flags & OF_GPIO_ACTIVE_LOW);
	data->night_led_time_ms = time_ms;
	data->irq_name = name;
	data->timer_inited = 0;

	return 0;
}

static const struct of_device_id irq_sync_leds_of_match[] = {
	{ .compatible = "oclea,irq_sync_leds", .data = &irq_sync_leds_ids[0] },
	{}
};

MODULE_DEVICE_TABLE(of, irq_sync_leds_of_match);
#else
static int irq_sync_leds_parse_dt(struct device *dev,
				  struct irq_sync_leds_data *data)
{
	return -ENODEV;
}
#endif

static int irq_sync_leds_probe(struct platform_device *pdev)
{
	struct irq_sync_leds_data *data;
	struct device *dev = &pdev->dev;
	const struct of_device_id *of_id =
		of_match_device(of_match_ptr(irq_sync_leds_of_match), dev);
	const struct platform_device_id *pdev_id;
	int rval;

	data = devm_kzalloc(dev, sizeof(struct irq_sync_leds_data), GFP_KERNEL);
	if (!data) {
		return -ENOMEM;
	}

	data->dev = dev;
	platform_set_drvdata(pdev, data);

	rval = irq_sync_leds_parse_dt(&pdev->dev, data);
	if (rval < 0) {
		dev_err(dev, "failed to parse dt: %d\n", rval);
		return -EINVAL;
	}

	rval = irq_sync_leds_init(data);
	if (rval < 0) {
		dev_err(&pdev->dev, "failed to init gpios: %d\n", rval);
		return -EINVAL;
	}

	pdev_id = of_id ? of_id->data : platform_get_device_id(pdev);

	dev_info(dev, "irq_sync_leds: %s successfully probed.\n",
		 pdev_id->name);
	dev_info(dev, "\tnight-led-gpio: %d\n", data->night_led_gpio);
	dev_info(dev, "\tnight-led-gpio-active: %d\n",
		 data->night_led_gpio_active);
	dev_info(dev, "\tnight-led-time-ms: %d\n", data->night_led_time_ms);
	dev_info(dev, "\tirq: %d\n", data->irq);

	return 0;
}

static int irq_sync_leds_remove(struct platform_device *pdev)
{
	struct irq_sync_leds_data *data = platform_get_drvdata(pdev);
	if (!data) {
		dev_err(&pdev->dev, "failed to get drvdata\n");
		return -EINVAL;
	}

	free_irq(data->virt_irq, data);

	return 0;
}

static struct platform_driver irq_sync_leds_driver = {
    .driver = {
        .name = "irq_sync_leds",
#ifdef CONFIG_OF
        .of_match_table = of_match_ptr(irq_sync_leds_of_match),
#endif
    },
    .probe = irq_sync_leds_probe,
    .remove = irq_sync_leds_remove,
    .id_table = irq_sync_leds_ids,
};

module_platform_driver(irq_sync_leds_driver);

MODULE_AUTHOR("Gym Ok Cho <kimogi@teknique.com>");
MODULE_DESCRIPTION("IRQ driven LEDs driver");
MODULE_LICENSE("GPL v2");
