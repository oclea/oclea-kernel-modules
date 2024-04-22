/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * IRQ monitor and GPIO toggle driver
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

#define US_TO_NS_DIVIDE (1000)

enum irq_modes {
    IRQ_MODE_PULSE_HIGH,
    IRQ_MODE_PULSE_LOW,
    IRQ_MODE_TOGGLE,
};

struct irq_toggle_gpio_data {
    struct device *dev;
    enum hrtimer_restart timer_state;
    struct hrtimer hr_timer;
    int irq;
    int virt_irq;
    const char *irq_name;
    int gpio;
    enum irq_modes mode;
    int duration_us;
    enum of_gpio_flags begin_state;
    int last_value;
    int inited;
};

static const struct platform_device_id irq_toggle_gpio_ids[] = {
    {"irq_toggle_gpio", 0},
    {}
};

enum hrtimer_restart timer_trigger_func(struct hrtimer *timer)
{
    struct irq_toggle_gpio_data temp_struct;
    struct irq_toggle_gpio_data *data = container_of(&temp_struct.hr_timer,
                                                     struct irq_toggle_gpio_data,
                                                     hr_timer);

    if (IRQ_MODE_PULSE_HIGH == data->mode) {
        gpio_set_value(data->gpio, 0);
    } else {
        gpio_set_value(data->gpio, 1);
    }

    return data->timer_state;
}

static irqreturn_t irq_toggle_gpio_interrupt_handler(int irq, void *dev_id)
{
    struct irq_toggle_gpio_data *data = dev_id;
    int change_to;
    ktime_t kt;

    switch(data->mode) {
    case IRQ_MODE_PULSE_HIGH:
        change_to = 1;
        break;
    case IRQ_MODE_PULSE_LOW:
        change_to = 0;
        break;
    default:
        change_to = -1;
    }

    if (change_to < 0) {
        gpio_set_value(data->gpio, !!data->last_value);
        data->last_value = !data->last_value;
    } else {
        gpio_set_value(data->gpio, change_to);
        data->timer_state = HRTIMER_NORESTART;
        kt = ktime_set(0, 10);
        if (!data->inited) {
            hrtimer_init(&data->hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
            data->inited = 1;
        }
        hrtimer_set_expires(&data->hr_timer, kt);
        data->hr_timer.function = &timer_trigger_func;
        hrtimer_start(&data->hr_timer, kt, HRTIMER_MODE_ABS);
    }

    return IRQ_HANDLED;
}

static int irq_toggle_gpio_init(struct irq_toggle_gpio_data *data)
{
    int ret;
    struct irq_data *irq_data;
    int hw_irq;
    int i;

    data->virt_irq = -1;

    for (i = 0; i < 256; i++) {
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

    if (gpio_is_valid(data->gpio)) {
        if (OF_GPIO_ACTIVE_LOW == data->begin_state) {
            gpio_direction_output(data->gpio, 1);
            data->last_value = 1;
        } else {
            gpio_direction_output(data->gpio, 0);
            data->last_value = 0;
        }
    } else {
        dev_err(data->dev, "gpio %d is invalid\n", data->gpio);
        return -EINVAL;
    }

    dev_info(data->dev, "Requesting irq: %d (%d)\n", data->virt_irq, data->irq);

    ret = request_irq(data->virt_irq,
                      &irq_toggle_gpio_interrupt_handler,
                      IRQF_TRIGGER_RISING | IRQF_SHARED,
                      data->irq_name ? data->irq_name : "irq-gpio-toggle",
                      data);
    if (ret) {
        dev_err(data->dev, "can't request IRQ(%d) %s. Err: %d!\n",
                data->virt_irq, data->irq_name ? data->irq_name : "NULL", ret);
        return -EINVAL;
    }

    return 0;
}

#ifdef CONFIG_OF
static int irq_toggle_gpio_parse_dt(struct device *dev, struct irq_toggle_gpio_data *data)
{
    struct device_node *node = dev->of_node;
    int ret;
    int gpio, duration_us, irq;
    const char *name;
    const char *mode;
    enum of_gpio_flags flags;

    if (!node) {
        return -ENODEV;
    }

    dev_dbg(dev, "irq_toggle_gpio_parse_dt\n");

    gpio = of_get_named_gpio_flags(dev->of_node, "gpio", 0, &flags);
    if (gpio == -EPROBE_DEFER) {
        return gpio;
    } else if (gpio < 0) {
        dev_err(data->dev, "missing gpio property\n");
        return -EINVAL;
    }

    data->begin_state = flags;

    if (of_property_read_s32(dev->of_node, "irq", &irq) < 0) {
        dev_err(data->dev, "missing irq property\n");
        return -EINVAL;
    }

/*  Pulse mode is not working due to the timer function, needs looking into
    if (of_property_read_string(dev->of_node, "mode", &mode) < 0) {
        dev_err(data->dev, "missing mode property\n");
        return -EINVAL;
    }
*/
    mode = "toggle";

    if (strncmp("pulse-high", mode, strlen("pulse-high")) == 0) {
        data->mode = IRQ_MODE_PULSE_HIGH;
    } else if (strncmp("pulse-low", mode, strlen("pulse-low")) == 0) {
        data->mode = IRQ_MODE_PULSE_LOW;
    } else if (strncmp("toggle", mode, strlen("toggle")) == 0) {
        data->mode = IRQ_MODE_TOGGLE;
    } else {
        dev_err(data->dev, "invalid mode specified\n");
        return -EINVAL;
    }

/*  Only used in pulsed mode
    if (of_property_read_s32(dev->of_node, "duration-us", &duration_us) < 0) {
        if (IRQ_MODE_PULSE_HIGH == data->mode ||
            IRQ_MODE_PULSE_LOW == data->mode) {
            dev_err(data->dev, "missing duration-us property\n");
            return -EINVAL;
        }

        duration_us = -1;
    }
*/

    if (gpio_is_valid(gpio)) {
        if (gpio_cansleep(gpio)) {
            dev_info(dev, "gpio %d operation may sleep\n", gpio);
        }

        ret = devm_gpio_request(dev,
                                gpio,
                                dev_name(dev));
        if (ret) {
            dev_err(dev, "failed to request gpio: %d\n", gpio);
            return ret;
        }
    }

    if (of_property_read_string(dev->of_node, "irq-name", &name) < 0) {
        name = NULL;
    }

    data->irq = irq;
    data->gpio = gpio;
    data->irq_name = name;
    data->duration_us = duration_us;
    data->inited = 0;

    return 0;
}

static const struct of_device_id irq_toggle_gpio_of_match[] = {
    { .compatible = "oclea,irq_toggle_gpio",
        .data = &irq_toggle_gpio_ids[0] },
    { }
};

MODULE_DEVICE_TABLE(of, irq_toggle_gpio_of_match);
#else
static int irq_toggle_gpio_parse_dt(struct device *dev, struct irq_toggle_gpio_data *data)
{
    return -ENODEV;
}
#endif

static int irq_toggle_gpio_probe(struct platform_device *pdev)
{
    struct irq_toggle_gpio_data *data;
    struct device *dev = &pdev->dev;
    const struct of_device_id *of_id = of_match_device(of_match_ptr(irq_toggle_gpio_of_match), dev);
    const struct platform_device_id *pdev_id;  
    int rval;

    data = devm_kzalloc(dev, sizeof(struct irq_toggle_gpio_data), GFP_KERNEL);
    if (!data){
        return -ENOMEM;
    }

    data->dev = dev;

    platform_set_drvdata(pdev, data);

    rval = irq_toggle_gpio_parse_dt(&pdev->dev, data);
    if (rval < 0) {
        dev_err(dev, "failed to parse dt: %d\n", rval);
        return -EINVAL;
    }

    rval = irq_toggle_gpio_init(data);
    if (rval < 0) {
        dev_err(&pdev->dev, "failed to init gpios: %d\n", rval);
        return -EINVAL;
    }

    pdev_id = of_id ? of_id->data : platform_get_device_id(pdev);

    dev_info(dev, "irq_toggle_gpio: %s successfully probed.\n",
             pdev_id->name);

    return 0;
}

static int irq_toggle_gpio_remove(struct platform_device *pdev)
{
    struct irq_toggle_gpio_data* data = platform_get_drvdata(pdev);
    if (!data) {
        dev_err(&pdev->dev, "failed to get drvdata\n");
        return -EINVAL;
    }

    free_irq(data->virt_irq, data);

    return 0;
}

static struct platform_driver irq_toggle_gpio_driver = {
    .driver = {
        .name = "irq_toggle_gpio",
#ifdef CONFIG_OF
        .of_match_table = of_match_ptr(irq_toggle_gpio_of_match),
#endif
    },
    .probe = irq_toggle_gpio_probe,
    .remove = irq_toggle_gpio_remove,
    .id_table = irq_toggle_gpio_ids,
};

module_platform_driver(irq_toggle_gpio_driver);

MODULE_AUTHOR("Stephen Ye <stephen.ye@teknique.com>");
MODULE_DESCRIPTION("IRQ monitor and GPIO toggle driver");
MODULE_LICENSE("GPL v2");
