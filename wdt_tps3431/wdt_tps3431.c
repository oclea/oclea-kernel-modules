/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * TPS3431 Watchdog Driver
 *
 * Copyright (c) 2020 Teknique Limited
 *
 * This file is subject to the terms and conditions of version 2 of
 * the GNU General Public License.  See the file COPYING in the main
 * directory of this archive for more details.
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/slab.h>

#define DEFAULT_TRIGGER_MSECS 40

struct wdt_tps3431_data {
    enum hrtimer_restart timer_state;
    struct hrtimer hr_timer;
    u32 en_gpio;
    u32 trigger_gpio;
    long long trigger_interval_ns;
    int trigger_state;
};

static struct wdt_tps3431_data *data = NULL;

enum hrtimer_restart timer_trigger_func(struct hrtimer *timer)
{
    hrtimer_forward(timer, hrtimer_cb_get_time(timer), ktime_set(0, data->trigger_interval_ns));

    if (0 == data->trigger_state) {
        gpio_set_value(data->trigger_gpio, 1);
        data->trigger_state = 1;
    } else {
        gpio_set_value(data->trigger_gpio, 0);
        data->trigger_state = 0;
    }

    return data->timer_state;
}

static void wdt_tps3431_enable(void)
{
    ktime_t kt;
    data->timer_state = HRTIMER_RESTART;

    kt = ktime_set(0, data->trigger_interval_ns);
    hrtimer_init(&data->hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
    hrtimer_set_expires(&data->hr_timer, kt);
    data->hr_timer.function = &timer_trigger_func;
    hrtimer_start(&data->hr_timer, kt, HRTIMER_MODE_ABS);

    gpio_set_value_cansleep(data->trigger_gpio, 1);
    data->trigger_state = 1;
    gpio_set_value_cansleep(data->en_gpio, 1);
}

static void wdt_tps3431_disable(void)
{
    gpio_set_value_cansleep(data->en_gpio, 0);
    hrtimer_cancel(&data->hr_timer);
    data->timer_state = HRTIMER_NORESTART;
}

static int wdt_tps3431_init(struct platform_device *pdev)
{
    int err;

    if (gpio_is_valid(data->en_gpio)) {
        err = devm_gpio_request_one(&pdev->dev, data->en_gpio, GPIOF_OUT_INIT_LOW, "wdt en");
        if (err < 0) {
            dev_err(&pdev->dev,
                    "failed to request GPIO %d, error %d\n",
                    data->en_gpio, err);
            return err;
        }
    } else {
        dev_err(&pdev->dev, "Invalid gpio %d, error: %d\n", data->en_gpio, err);
        return err;
    }

    if (gpio_is_valid(data->trigger_gpio)) {
        err = devm_gpio_request_one(&pdev->dev, data->trigger_gpio, GPIOF_OUT_INIT_LOW, "wdt trigger");
        if (err < 0) {
            dev_err(&pdev->dev,
                    "failed to request GPIO %d, error %d\n",
                    data->trigger_gpio, err);
            return err;
        }
    } else {
        dev_err(&pdev->dev, "Invalid gpio %d, error: %d\n", data->trigger_gpio, err);
        devm_gpio_free(&pdev->dev, data->trigger_gpio);
        return err;
    }

    wdt_tps3431_enable();

    return 0;
}

static void wdt_tps3431_release(struct platform_device *pdev)
{
    wdt_tps3431_disable();
    devm_gpio_free(&pdev->dev, data->en_gpio);
    devm_gpio_free(&pdev->dev, data->trigger_gpio);
}

#ifdef CONFIG_OF
static int wdt_tps3431_parse_dt(struct device *dev)
{
    struct device_node *node = dev->of_node;
    enum of_gpio_flags flags;
    u32 interval_ms;

    if (!node)
        return -ENODEV;

    data->en_gpio = of_get_named_gpio_flags(node, "en-gpio", 0, &flags);
    data->trigger_gpio = of_get_named_gpio_flags(node, "trigger-gpio", 0, &flags);

    if (of_property_read_u32(node, "trigger-interval", &interval_ms) != 0) {
        interval_ms = DEFAULT_TRIGGER_MSECS;
    }

    interval_ms /= 2;

    data->trigger_interval_ns = (long long)interval_ms * 1000000;

    if (!gpio_is_valid(data->en_gpio) || !gpio_is_valid(data->trigger_gpio)) {
        dev_err(dev, "Invalid gpios\n");
        return -EINVAL;
    }

    return 0;
}

static const struct of_device_id wdt_tps3431_of_match[] = {
    { .compatible = "wdt_tps3431" },
    { }
};

MODULE_DEVICE_TABLE(of, wdt_tps3431_of_match);
#else
static int wdt_tps3431_parse_dt(struct device *dev)
{
    return -ENODEV;
}
#endif

static int wdt_tps3431_probe(struct platform_device *pdev)
{
    struct wdt_tps3431_data *wdt_data;
    int rval;

    wdt_data = kmalloc(sizeof(struct wdt_tps3431_data), GFP_KERNEL);
    if (!wdt_data) {
        return -ENOMEM;
    }

    data = wdt_data;

    rval = wdt_tps3431_parse_dt(&pdev->dev);
    if (rval < 0) {
        dev_err(&pdev->dev, "Failed to parse dt %d!\n", rval);
        return -EINVAL;
    }

    rval = wdt_tps3431_init(pdev);
    if (rval < 0)
        return rval;

    dev_set_name(&pdev->dev, "wdt_tps3431");

    return 0;
}

static int wdt_tps3431_remove(struct platform_device *pdev)
{
    kfree(data);
    wdt_tps3431_release(pdev);
    return 0;
}

static void wdt_tps3431_shutdown(struct platform_device *pdev)
{
    wdt_tps3431_disable();
}

static const struct platform_device_id wdt_tps3431_ids[] = {
    {"wdt_tps3431", 0},
    {}
};

MODULE_DEVICE_TABLE(platform, wdt_tps3431_ids);

static struct platform_driver wdt_tps3431_driver = {
    .probe = wdt_tps3431_probe,
    .remove = wdt_tps3431_remove,
    .shutdown = wdt_tps3431_shutdown,
    .id_table = wdt_tps3431_ids,
    .driver = {
        .name = "wdt_tps3431",
        .of_match_table = of_match_ptr(wdt_tps3431_of_match),
    },
};

module_platform_driver(wdt_tps3431_driver);

MODULE_AUTHOR("Caleb J <caleb@teknique.com>");
MODULE_DESCRIPTION("TPS3431 Watchdog Timer Driver");
MODULE_LICENSE("GPL v2");
