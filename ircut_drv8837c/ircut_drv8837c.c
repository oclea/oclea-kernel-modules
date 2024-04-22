/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * DRV8837C gpio driver
 *
 * Copyright (c) 2017 Teknique Limited
 *
 * This file is subject to the terms and conditions of version 2 of
 * the GNU General Public License.  See the file COPYING in the main
 * directory of this archive for more details.
 *
 * IIO driver for DRV8837C gpio driver.
*/

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/timer.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/iio/iio.h>
#include <linux/iio/types.h>
#include <linux/iio/sysfs.h>
#include <linux/version.h>

typedef enum ir_cut_state_e {
    IR_CUT_STATE_DAY = 0,
    IR_CUT_STATE_NIGHT
} ir_cut_state;

struct drv8837c_data {
    const char *dev_name;
    ir_cut_state state;
    struct timer_list timer;
    u32 sw1_gpio;
    u32 sw2_gpio;
};

static const struct iio_chan_spec drv8837c_channels[] = {
    {
        .type = IIO_IRCUT,
        .output = 1,
        .info_mask_separate = BIT(IIO_CHAN_INFO_ENABLE),
    }
};

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
void drv_timer_func(unsigned long ptr)
{
    struct drv8837c_data *data = (struct drv8837c_data *)ptr;
#else
void drv_timer_func(struct timer_list *t)
{
    struct drv8837c_data *data = from_timer(data, t, timer);
#endif
    gpio_set_value(data->sw1_gpio, 0);
    gpio_set_value(data->sw2_gpio, 0);
}

static void drv8837c_drive_foward(struct drv8837c_data *data)
{
    gpio_set_value(data->sw1_gpio, 0);
    gpio_set_value(data->sw2_gpio, 1);

    data->state = IR_CUT_STATE_DAY;
}

static void drv8837c_drive_reverse(struct drv8837c_data *data)
{
    gpio_set_value(data->sw1_gpio, 1);
    gpio_set_value(data->sw2_gpio, 0);

    data->state = IR_CUT_STATE_NIGHT;
}

static void drv8837c_ircut_control(struct drv8837c_data *data, int state)
{
    if (state == IR_CUT_STATE_DAY)
        drv8837c_drive_foward(data);
    else if (state == IR_CUT_STATE_NIGHT)
        drv8837c_drive_reverse(data);

    mod_timer(&data->timer, jiffies + msecs_to_jiffies(3000));
}

static int drv8837c_read_raw(struct iio_dev *indio_dev,
                struct iio_chan_spec const *chan, int *val,
                int *val2, long mask)
{
    struct drv8837c_data *data = iio_priv(indio_dev);

    switch (mask) {
    case IIO_CHAN_INFO_ENABLE:
        *val = data->state;
        return IIO_VAL_INT;
    }

    return -EINVAL;
}

static int drv8837c_write_raw(struct iio_dev *indio_dev,
                 struct iio_chan_spec const *chan, int val,
                 int val2, long mask)
{
    struct drv8837c_data *data = iio_priv(indio_dev);

    switch (mask) {
    case IIO_CHAN_INFO_ENABLE:
        if (val == IR_CUT_STATE_NIGHT || val == IR_CUT_STATE_DAY)
        {
            drv8837c_ircut_control(data, val);
            return 0;
        }
    }

    return -EINVAL;
}

static const struct iio_info drv8837c_info = {
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
    .driver_module  = THIS_MODULE,
#endif
    .read_raw   = drv8837c_read_raw,
    .write_raw  = drv8837c_write_raw,
};

static int drv8837c_init(struct platform_device *pdev)
{
    int err;
    struct iio_dev *indio_dev = platform_get_drvdata(pdev);
    struct drv8837c_data *data = iio_priv(indio_dev);

    if (gpio_is_valid(data->sw1_gpio)) {
        err = devm_gpio_request_one(&indio_dev->dev, data->sw1_gpio, GPIOF_OUT_INIT_LOW, "sw1");
        if (err < 0) {
            dev_err(&indio_dev->dev,
                "failed to request GPIO %d, error %d\n",
                data->sw1_gpio, err);
            return err;
        }
    }

    if (gpio_is_valid(data->sw2_gpio)) {
        err = devm_gpio_request_one(&indio_dev->dev, data->sw2_gpio, GPIOF_OUT_INIT_LOW, "sw2");
        if (err < 0) {
            dev_err(&indio_dev->dev,
                "failed to request GPIO %d, error %d\n",
                data->sw2_gpio, err);
            return err;
        }
    }

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
    setup_timer(&data->timer, drv_timer_func, (unsigned long) data);
#else
    timer_setup(&data->timer, drv_timer_func, 0);
#endif
    drv8837c_ircut_control(data, 0);
    return 0;
}

static void drv8837c_release(struct platform_device *pdev)
{
    struct iio_dev *indio_dev = platform_get_drvdata(pdev);
    struct drv8837c_data *data = iio_priv(indio_dev);

    devm_gpio_free(&indio_dev->dev, data->sw1_gpio);
    devm_gpio_free(&indio_dev->dev, data->sw2_gpio);

    del_timer(&data->timer);
}

#ifdef CONFIG_OF
static int drv8837c_parse_dt(struct device *dev, struct drv8837c_data *data)
{
    struct device_node *node = dev->of_node;
    enum of_gpio_flags flags;

    if (!node)
        return -ENODEV;

    data->sw1_gpio = of_get_named_gpio_flags(node, "sw1-gpio", 0, &flags);
    data->sw2_gpio = of_get_named_gpio_flags(node, "sw2-gpio", 0, &flags);

    if (!gpio_is_valid(data->sw1_gpio) || !gpio_is_valid(data->sw2_gpio))
        return -EINVAL;

    if (of_property_read_string(node, "dev-name", &data->dev_name) != 0)
        data->dev_name = NULL;

    return 0;
}

static const struct of_device_id drv8837c_of_match[] = {
    { .compatible = "ti,drv8837c" },
    { }
};

MODULE_DEVICE_TABLE(of, drv8837c_of_match);
#else
static int drv8837c_parse_dt(struct device *dev, struct drv8837c_data *data)
{
    return -ENODEV;
}
#endif

static int drv8837c_probe(struct platform_device *pdev)
{
    struct drv8837c_data *data;
    struct iio_dev *indio_dev = NULL;
    int rval;

    indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(struct drv8837c_data));
    if (!indio_dev) {
        dev_err(&pdev->dev, "failed to allocate iio device\n");
        return -ENOMEM;
    }

    data = iio_priv(indio_dev);

    platform_set_drvdata(pdev, indio_dev);

    rval = drv8837c_parse_dt(&pdev->dev, data);
    if (rval < 0) {
        dev_err(&pdev->dev, "failed to parse dt %d!\n", rval);
        return -EINVAL;
    }

    rval = drv8837c_init(pdev);
    if (rval < 0)
        return rval;

    if (data->dev_name)
        dev_set_name(&indio_dev->dev, data->dev_name);
    else
        dev_set_name(&indio_dev->dev, "iio:drv8837c");

    indio_dev->name = dev_name(&pdev->dev);
    indio_dev->dev.parent = &pdev->dev;
    indio_dev->dev.of_node = pdev->dev.of_node;
    indio_dev->info = &drv8837c_info;
    indio_dev->channels = drv8837c_channels;
    indio_dev->num_channels = ARRAY_SIZE(drv8837c_channels);
    indio_dev->modes = INDIO_DIRECT_MODE;

    rval = iio_device_register(indio_dev);
    if (rval < 0) {
        dev_err(&pdev->dev, "failed to register iio device %d!\n", rval);
        return -ENXIO;
    }

    dev_info(&pdev->dev, "%d channels\n", indio_dev->num_channels);

    return 0;
}

static int drv8837c_remove(struct platform_device *pdev)
{
    drv8837c_release(pdev);
    iio_device_unregister(platform_get_drvdata(pdev));
    return 0;
}

static const struct platform_device_id drv8837c_ids[] = {
    {"drv8837c", 0},
    {}
};

MODULE_DEVICE_TABLE(platform, drv8837c_ids);

static struct platform_driver drv8837c_driver = {
    .probe = drv8837c_probe,
    .remove = drv8837c_remove,
    .id_table = drv8837c_ids,
    .driver = {
        .name = "drv8837c",
        .of_match_table = of_match_ptr(drv8837c_of_match),
    },
};

module_platform_driver(drv8837c_driver);

MODULE_AUTHOR("Gym Ok Cho <kimogi@teknique.com>");
MODULE_DESCRIPTION("DRV8837 ircut Driver");
MODULE_LICENSE("GPL v2");
