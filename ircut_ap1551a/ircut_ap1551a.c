/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * AP1551A driver
 *
 * Copyright (c) 2017 Teknique Limited
 *
 * This file is subject to the terms and conditions of version 2 of
 * the GNU General Public License.  See the file COPYING in the main
 * directory of this archive for more details.
 *
 * IIO driver for ap1551a gpio driver.
*/

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/delay.h>
#include <linux/version.h>

typedef enum ir_cut_state_e {
    IR_CUT_STATE_OFF = 0,
    IR_CUT_STATE_ON
} ir_cut_state;

struct ap1551a_data {
    const char *dev_name;
    ir_cut_state state;
    int fbc_gpio; //sw1-gpio
    int enb_gpio; //sw2-gpio
    int fbc_gpio_active;
    int enb_gpio_active;
};

static const struct iio_chan_spec ap1551a_channels[] = {
    {
        .type = IIO_IRCUT,
        .output = 1,
        .info_mask_separate = BIT(IIO_CHAN_INFO_ENABLE),
    }
};

static void ap1551a_drive_foward(struct ap1551a_data *data)
{
    gpio_set_value_cansleep(data->fbc_gpio, !data->fbc_gpio_active);
    gpio_set_value_cansleep(data->enb_gpio, !!data->enb_gpio_active);

    data->state = IR_CUT_STATE_OFF;
}

static void ap1551a_drive_reverse(struct ap1551a_data *data)
{
    gpio_set_value_cansleep(data->fbc_gpio, !!data->fbc_gpio_active);
    gpio_set_value_cansleep(data->enb_gpio, !!data->enb_gpio_active);

    data->state = IR_CUT_STATE_ON;
}

static void ap1551a_ircut_control(struct ap1551a_data *data, int state)
{
    if (state == IR_CUT_STATE_OFF)
        ap1551a_drive_foward(data);
    else if (state == IR_CUT_STATE_ON)
        ap1551a_drive_reverse(data);
    msleep(1);
    gpio_set_value_cansleep(data->enb_gpio, !data->enb_gpio_active);
}

static int ap1551a_read_raw(struct iio_dev *indio_dev,
                struct iio_chan_spec const *chan, int *val,
                int *val2, long mask)
{
    struct ap1551a_data *data = iio_priv(indio_dev);

    switch (mask) {
    case IIO_CHAN_INFO_ENABLE:
        *val = data->state;
        return IIO_VAL_INT;
    }

    return -EINVAL;
}

static int ap1551a_write_raw(struct iio_dev *indio_dev,
                 struct iio_chan_spec const *chan, int val,
                 int val2, long mask)
{
    struct ap1551a_data *data = iio_priv(indio_dev);

    switch (mask) {
    case IIO_CHAN_INFO_ENABLE:
        if (val == IR_CUT_STATE_ON || val == IR_CUT_STATE_OFF)
        {
            ap1551a_ircut_control(data, val);
            return 0;
        }
    }

    return -EINVAL;
}

static const struct iio_info ap1551a_info = {
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
    .driver_module  = THIS_MODULE,
#endif
    .read_raw   = ap1551a_read_raw,
    .write_raw  = ap1551a_write_raw,
};

static int ap1551a_init(struct platform_device *pdev)
{
    int err;
    struct iio_dev *indio_dev = platform_get_drvdata(pdev);
    struct     ap1551a_data *data = iio_priv(indio_dev);

    if (gpio_is_valid(data->fbc_gpio)) {
        err = devm_gpio_request_one(&indio_dev->dev, data->fbc_gpio, !data->fbc_gpio_active ? GPIOF_OUT_INIT_LOW : GPIOF_OUT_INIT_HIGH, "fbc");
        if (err < 0) {
            dev_err(&indio_dev->dev,
                "failed to request GPIO %d, error %d\n",
                data->fbc_gpio, err);
            return err;
        }
    }

    if (gpio_is_valid(data->enb_gpio)) {
        err = devm_gpio_request_one(&indio_dev->dev, data->enb_gpio, !data->enb_gpio_active ? GPIOF_OUT_INIT_LOW : GPIOF_OUT_INIT_HIGH, "enb");
        if (err < 0) {
            dev_err(&indio_dev->dev,
                "failed to request GPIO %d, error %d\n",
                data->enb_gpio, err);
            return err;
        }
    }

    return 0;
}

static void ap1551a_release(struct platform_device *pdev)
{
    struct iio_dev *indio_dev = platform_get_drvdata(pdev);
    struct     ap1551a_data *data = iio_priv(indio_dev);

    devm_gpio_free(&indio_dev->dev, data->fbc_gpio);
    devm_gpio_free(&indio_dev->dev, data->enb_gpio);
}

#ifdef CONFIG_OF
static int ap1551a_parse_dt(struct device *dev, struct ap1551a_data *data)
{
    struct device_node *node = dev->of_node;
    enum of_gpio_flags flags;

    if (!node)
        return -ENODEV;

    data->fbc_gpio = of_get_named_gpio_flags(node, "sw1-gpio", 0, &flags);
    data->fbc_gpio_active = !!(flags & OF_GPIO_ACTIVE_LOW);

    data->enb_gpio = of_get_named_gpio_flags(node, "sw2-gpio", 0, &flags);
    data->enb_gpio_active = !!(flags & OF_GPIO_ACTIVE_LOW);

    if (!gpio_is_valid(data->fbc_gpio) || !gpio_is_valid(data->enb_gpio))
        return -EINVAL;

    if (of_property_read_string(node, "dev-name", &data->dev_name) != 0)
        data->dev_name = NULL;

    return 0;
}

static const struct of_device_id ap1551a_of_match[] = {
    { .compatible = "ap1551a" },
    { }
};

MODULE_DEVICE_TABLE(of, ap1551a_of_match);
#else
static int ap1551a_parse_dt(struct device *dev, struct     ap1551a_data *data)
{
    return -ENODEV;
}
#endif

static int ap1551a_probe(struct platform_device *pdev)
{
    struct     ap1551a_data *data;
    struct iio_dev *indio_dev = NULL;
    int rval;

    indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(struct ap1551a_data));
    if (!indio_dev) {
        dev_err(&pdev->dev, "failed to allocate iio device\n");
        return -ENOMEM;
    }

    data = iio_priv(indio_dev);

    platform_set_drvdata(pdev, indio_dev);

    rval = ap1551a_parse_dt(&pdev->dev, data);
    if (rval < 0) {
        dev_err(&pdev->dev, "failed to parse dt %d!\n", rval);
        return -EINVAL;
    }

    rval = ap1551a_init(pdev);
    if (rval < 0)
        return rval;

    if (data->dev_name)
        dev_set_name(&indio_dev->dev, data->dev_name);
    else
        dev_set_name(&indio_dev->dev, "iio:    ap1551a");

    indio_dev->name = dev_name(&pdev->dev);
    indio_dev->dev.parent = &pdev->dev;
    indio_dev->dev.of_node = pdev->dev.of_node;
    indio_dev->info = &ap1551a_info;
    indio_dev->channels = ap1551a_channels;
    indio_dev->num_channels = ARRAY_SIZE(ap1551a_channels);
    indio_dev->modes = INDIO_DIRECT_MODE;

    rval = iio_device_register(indio_dev);
    if (rval < 0) {
        dev_err(&pdev->dev, "failed to register iio device %d!\n", rval);
        return -ENXIO;
    }

    dev_info(&pdev->dev, "%d channels\n", indio_dev->num_channels);

    return 0;
}

static int ap1551a_remove(struct platform_device *pdev)
{
    ap1551a_release(pdev);
    iio_device_unregister(platform_get_drvdata(pdev));
    return 0;
}

static const struct platform_device_id ap1551a_ids[] = {
    {"ap1551a", 0},
    {}
};

MODULE_DEVICE_TABLE(platform, ap1551a_ids);

static struct platform_driver ap1551a_driver = {
    .probe = ap1551a_probe,
    .remove = ap1551a_remove,
    .id_table = ap1551a_ids,
    .driver = {
        .name = "ap1551a",
        .of_match_table = of_match_ptr(ap1551a_of_match),
    },
};

module_platform_driver(ap1551a_driver);

MODULE_AUTHOR("Gym Ok Cho <kimogi@teknique.com>");
MODULE_DESCRIPTION("AP1551A ircut Driver");
MODULE_LICENSE("GPL v2");
