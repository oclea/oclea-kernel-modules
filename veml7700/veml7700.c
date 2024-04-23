// SPDX-License-Identifier: GPL-2.0-only
/*
 * IIO driver for VEML7700 (7-bit I2C slave address 0x10)
 *
 * TODO: interrupts, white channel, persistance protect
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/delay.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#define VEML7700_I2C_ADDR 0x10

#define VEML7700_ALS_CFG_REG 0x00
#define VEML7700_ALS_WH_REG 0x01
#define VEML7700_ALS_WL_REG 0x02
#define VEML7700_ALS_PSM_REG 0x03 //reserved
#define VEML7700_ALS_DATA_REG 0x04
#define VEML7700_ALS_WHITE_REG 0x05
#define VEML7700_ALS_INT_REG 0x06

#define VEML7700_ALS_SD_M BIT(0)
#define VEML7700_ALS_IT_M 0x3C0
#define VEML7700_ALS_IT_P 6
#define VEML7700_ALS_GAIN_M 0x1800
#define VEML7700_ALS_GAIN_P 11

#define VEML7700_ALS_STARTUP_MS 5

#define ALS_IT_NUM 6
#define ALS_GAIN_NUM 4

typedef enum {
    ALS_IT_25ms = 0b1100,
    ALS_IT_50ms = 0b1000,
    ALS_IT_100ms = 0b0000,
    ALS_IT_200ms = 0b0001,
    ALS_IT_400ms = 0b0010,
    ALS_IT_800ms = 0b0011,
} ALS_IT;

typedef enum {
    ALS_GAIN_x1 = 0b00,
    ALS_GAIN_x2 = 0b01,
    ALS_GAIN_div_8 = 0b10,
    ALS_GAIN_div_4 = 0b11,
} ALS_GAIN;

typedef struct veml7700_10K_lux_scale {
    ALS_IT it;
    ALS_GAIN gain;
    int scale;
} veml7700_10K_lux_scale;

static veml7700_10K_lux_scale veml7700_scales[ALS_IT_NUM * ALS_GAIN_NUM] = {
    { .it = ALS_IT_25ms, .gain = ALS_GAIN_x1, .scale = 1152 },
    { .it = ALS_IT_25ms, .gain = ALS_GAIN_x2, .scale = 2304 },
    { .it = ALS_IT_25ms, .gain = ALS_GAIN_div_4, .scale = 9216 },
    { .it = ALS_IT_25ms, .gain = ALS_GAIN_div_8, .scale = 8432 },
    { .it = ALS_IT_50ms, .gain = ALS_GAIN_x1, .scale = 576 },
    { .it = ALS_IT_50ms, .gain = ALS_GAIN_x2, .scale = 1152 },
    { .it = ALS_IT_50ms, .gain = ALS_GAIN_div_4, .scale = 4608 },
    { .it = ALS_IT_50ms, .gain = ALS_GAIN_div_8, .scale = 9216 },
    { .it = ALS_IT_100ms, .gain = ALS_GAIN_x1, .scale = 288 },
    { .it = ALS_IT_100ms, .gain = ALS_GAIN_x2, .scale = 576 },
    { .it = ALS_IT_100ms, .gain = ALS_GAIN_div_4, .scale = 2304 },
    { .it = ALS_IT_100ms, .gain = ALS_GAIN_div_8, .scale = 4608 },
    { .it = ALS_IT_200ms, .gain = ALS_GAIN_x1, .scale = 144 },
    { .it = ALS_IT_200ms, .gain = ALS_GAIN_x2, .scale = 288 },
    { .it = ALS_IT_200ms, .gain = ALS_GAIN_div_4, .scale = 1152 },
    { .it = ALS_IT_200ms, .gain = ALS_GAIN_div_8, .scale = 2304 },
    { .it = ALS_IT_400ms, .gain = ALS_GAIN_x1, .scale = 72 },
    { .it = ALS_IT_400ms, .gain = ALS_GAIN_x2, .scale = 144 },
    { .it = ALS_IT_400ms, .gain = ALS_GAIN_div_4, .scale = 576 },
    { .it = ALS_IT_400ms, .gain = ALS_GAIN_div_8, .scale = 1152 },
    { .it = ALS_IT_800ms, .gain = ALS_GAIN_x1, .scale = 36 },
    { .it = ALS_IT_800ms, .gain = ALS_GAIN_x2, .scale = 72 },
    { .it = ALS_IT_800ms, .gain = ALS_GAIN_div_4, .scale = 288 },
    { .it = ALS_IT_800ms, .gain = ALS_GAIN_div_8, .scale = 576 },
};

struct veml7700_data {
    struct i2c_client * client;
    struct device * dev;
    struct mutex lock;
};

static int veml7700_gain_to_reg(int value)
{
    switch(value) {
    case 1:
        return ALS_GAIN_div_8;
    case 2:
        return ALS_GAIN_div_4;
    case 3:
        return ALS_GAIN_x1;
    case 4:
        return ALS_GAIN_x2;
    default:
        return -EINVAL;
    }
}

static int veml7700_reg_to_gain(ALS_GAIN gain)
{
    switch(gain) {
    case ALS_GAIN_div_8:
        return 1;
    case ALS_GAIN_div_4:
        return 2;
    case ALS_GAIN_x1:
        return 3;
    case ALS_GAIN_x2:
        return 4;
    default:
        return -EINVAL;
    }
}

static int veml7700_it_to_reg(int value)
{
    switch(value) {
    case -2:
        return ALS_IT_25ms;
    case -1:
        return ALS_IT_50ms;
    case 0:
        return ALS_IT_100ms;
    case 1:
        return ALS_IT_200ms;
    case 2:
        return ALS_IT_400ms;
    case 3:
        return ALS_IT_800ms;
    default:
        return -EINVAL;
    }
}

static int veml7700_reg_to_it(ALS_IT it)
{
    switch(it) {
    case ALS_IT_25ms:
        return -2;
    case ALS_IT_50ms:
        return -1;
    case ALS_IT_100ms:
        return 0;
    case ALS_IT_200ms:
        return 1;
    case ALS_IT_400ms:
        return 2;
    case ALS_IT_800ms:
        return 3;
    default:
        return -EINVAL;
    }
}

static int veml7700_shutdown(struct veml7700_data * data)
{
    int ret;
    u16 cfg_reg = 0;

    ret = i2c_smbus_read_word_data(data->client, VEML7700_ALS_CFG_REG);
    if (ret < 0) {
        return ret;
    }

    cfg_reg = ret;
    cfg_reg |= VEML7700_ALS_SD_M;

    return i2c_smbus_write_word_data(data->client, VEML7700_ALS_CFG_REG, cfg_reg);
}

static int veml7700_startup(struct veml7700_data * data)
{
    int ret;
    u16 cfg_reg = 0;

    ret = i2c_smbus_read_word_data(data->client, VEML7700_ALS_CFG_REG);
    if (ret < 0) {
        return ret;
    }

    cfg_reg = ret;
    cfg_reg &= ~VEML7700_ALS_SD_M;

    return i2c_smbus_write_word_data(data->client, VEML7700_ALS_CFG_REG, cfg_reg);
}

static int veml7700_get_scale(ALS_GAIN gain, ALS_IT it)
{
    int i = 0;
    int scale = 0;

    for (i = 0; i < ALS_IT_NUM*ALS_GAIN_NUM; i++) {
        if (veml7700_scales[i].gain == gain && veml7700_scales[i].it == it) {
            return veml7700_scales[i].scale;
        }
    }

    return -1;
}

static ALS_GAIN veml7700_read_gain(struct veml7700_data * data)
{
    int ret;
    u16 cfg_reg = 0;

    ret = i2c_smbus_read_word_data(data->client, VEML7700_ALS_CFG_REG);
    if (ret < 0) {
        return ret;
    }

    cfg_reg = ret;
    return (cfg_reg & VEML7700_ALS_GAIN_M) >> VEML7700_ALS_GAIN_P;
}

static ALS_IT veml7700_read_it(struct veml7700_data * data)
{
    int ret;
    u16 cfg_reg = 0;

    ret = i2c_smbus_read_word_data(data->client, VEML7700_ALS_CFG_REG);
    if (ret < 0) {
        return ret;
    }

    cfg_reg = ret;
    return (cfg_reg & VEML7700_ALS_IT_M) >> VEML7700_ALS_IT_P;
}

static int veml7700_write_gain(struct veml7700_data * data, ALS_GAIN gain)
{
    int ret;
    u16 cfg_reg = 0;

    ret = i2c_smbus_read_word_data(data->client, VEML7700_ALS_CFG_REG);
    if (ret < 0) {
        return ret;
    }

    cfg_reg = ret;
    cfg_reg &= ~VEML7700_ALS_GAIN_M;
    cfg_reg |= (gain << VEML7700_ALS_GAIN_P);

    return i2c_smbus_write_word_data(data->client, VEML7700_ALS_CFG_REG, cfg_reg);
}

static int veml7700_write_it(struct veml7700_data * data, ALS_IT it)
{
    int ret;
    u16 cfg_reg = 0;

    ret = i2c_smbus_read_word_data(data->client, VEML7700_ALS_CFG_REG);
    if (ret < 0) {
        return ret;
    }

    cfg_reg = ret;
    cfg_reg &= ~VEML7700_ALS_IT_M;
    cfg_reg |= (it << VEML7700_ALS_IT_P);

    return i2c_smbus_write_word_data(data->client, VEML7700_ALS_CFG_REG, cfg_reg);
}

static int veml7700_it_to_ms(ALS_IT it)
{
    switch(it) {
    case ALS_IT_25ms:
        return 25;
    case ALS_IT_50ms:
        return 50;
    case ALS_IT_100ms:
        return 100;
    case ALS_IT_200ms:
        return 200;
    case ALS_IT_400ms:
        return 400;
    case ALS_IT_800ms:
        return 800;
    default:
        return -EINVAL;
    }
}

static int veml7700_read_data(struct veml7700_data * data)
{
    int ret, it;
    int als_it_ms;
    int reg_data;

    ret = veml7700_read_it(data);
    if (ret < 0) {
        return ret;
    }

    it = ret;

    ret = veml7700_it_to_ms((ALS_IT)it);
    if (ret < 0) {
        return ret;
    }

    als_it_ms = ret + VEML7700_ALS_STARTUP_MS;

    ret = veml7700_startup(data);
    if (ret < 0) {
        return ret;
    }

    msleep(als_it_ms);

    ret = i2c_smbus_read_word_data(data->client, VEML7700_ALS_DATA_REG);
    if (ret < 0) {
        return ret;
    }

    reg_data = ret;

    ret = veml7700_shutdown(data);
    if (ret < 0) {
        return ret;
    }

    return reg_data;
}

static int veml7700_read_lux(struct veml7700_data * data)
{
    int scale, cnts, ret;
    ALS_GAIN gain;
    ALS_IT it;

    ret = veml7700_read_data(data);
    if (ret < 0) {
        return ret;
    }

    cnts = ret;

    it = veml7700_read_it(data);
    gain = veml7700_read_gain(data);

    scale = veml7700_get_scale(gain, it);

    return cnts * scale / 10000;
}

static const struct iio_chan_spec veml7700_channels[] = {
    {
        .type = IIO_LIGHT,
        .info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED) |
            BIT(IIO_CHAN_INFO_INT_TIME) |
            BIT(IIO_CHAN_INFO_HARDWAREGAIN),
    },
};

static int veml7700_write_raw(struct iio_dev *indio_dev,
                   struct iio_chan_spec const *chan,
                   int val, int val2, long mask)
{
    struct veml7700_data *data = iio_priv(indio_dev);
    int ret;

    mutex_lock(&data->lock);

    switch (mask) {
    case IIO_CHAN_INFO_INT_TIME:
        ret = veml7700_it_to_reg(val);
        if (ret < 0) {
            goto out;
        }
        ret = veml7700_write_it(data, (ALS_IT)ret);
        break;
    case IIO_CHAN_INFO_HARDWAREGAIN:
        ret = veml7700_gain_to_reg(val);
        if (ret < 0) {
            goto out;
        }
        ret = veml7700_write_gain(data, (ALS_GAIN)ret);
        break;
    default:
        ret = -EINVAL;
        break;
    }

out:
    mutex_unlock(&data->lock);
    return ret;
}

static int veml7700_read_raw(struct iio_dev *indio_dev,
                struct iio_chan_spec const *chan,
                int *val, int *val2, long mask)
{
    struct veml7700_data *data = iio_priv(indio_dev);
    int ret;

    mutex_lock(&data->lock);

    switch (mask) {
    case IIO_CHAN_INFO_PROCESSED:
        ret = veml7700_read_lux(data);
        if (ret < 0) {
            goto out;
        }
        *val = ret;
        ret = IIO_VAL_INT;
        break;
    case IIO_CHAN_INFO_HARDWAREGAIN:
        ret = veml7700_read_gain(data);
        if (ret < 0) {
            goto out;
        }
        *val = veml7700_reg_to_gain((u16)ret);
        ret = IIO_VAL_INT;
        break;
    case IIO_CHAN_INFO_INT_TIME:
        ret = veml7700_read_it(data);
        if (ret < 0) {
            goto out;
        }
        *val = veml7700_reg_to_it((u16)ret);
        ret = IIO_VAL_INT;
        break;
    default:
        ret = -EINVAL;
        break;
    }

out:
    mutex_unlock(&data->lock);
    return ret;
}

static IIO_CONST_ATTR(int_time_available, "-2 : 25 ms, -1 : 50 ms, 0 : 100 ms, 1 : 200 ms, 2 : 400 ms, 3 : 800 ms");
static IIO_CONST_ATTR(gain_available, "4 : gain x2, 3 : gain x1, 2 : gain x1/4, 1 : gain x1/8");

static struct attribute *veml7700_attributes[] = {
    &iio_const_attr_int_time_available.dev_attr.attr,
    &iio_const_attr_gain_available.dev_attr.attr,
    NULL
};

static const struct attribute_group veml7700_attribute_group = {
    .attrs = veml7700_attributes,
};

static const struct iio_info veml7700_info = {
    .read_raw = veml7700_read_raw,
    .write_raw = veml7700_write_raw,
    .attrs = &veml7700_attribute_group,
};

static int veml7700_probe(struct i2c_client *client,
              const struct i2c_device_id *id)
{
    struct device *dev = &client->dev;
    struct iio_dev *iio;
    struct veml7700_data *data;
    int ret;

    iio = devm_iio_device_alloc(dev, sizeof(*data));
    if (!iio) {
        return -ENOMEM;
    }

    data = iio_priv(iio);
    data->client = client;
    data->dev = dev;

    mutex_init(&data->lock);

    dev_set_name(&iio->dev, "iio:als");

    i2c_set_clientdata(client, iio);

    iio->name = client->name;
    iio->channels = veml7700_channels;
    iio->num_channels = ARRAY_SIZE(veml7700_channels);
    iio->dev.parent = dev;
    iio->modes = INDIO_DIRECT_MODE;
    iio->info = &veml7700_info;

    ret = veml7700_shutdown(data);
    if (ret < 0) {
        dev_err(dev, "Failed to shutdowm VEML7700\n");
        return ret;
    }

    ret = veml7700_write_it(data, ALS_IT_25ms);
    if (ret < 0) {
        dev_err(dev, "Failed to set intergration to VEML7700\n");
        return ret;
    }

    ret = veml7700_write_gain(data, ALS_GAIN_div_4);
    if (ret < 0) {
        dev_err(dev, "Failed to set als gain to VEML7700\n");
        return ret;
    }

    ret = devm_iio_device_register(dev, iio);
    if (ret) {
        dev_err(dev, "Failed to register IIO device\n");
        return ret;
    }

    return 0;
}

static int veml7700_remove(struct i2c_client *client)
{
    return 0;
}

static const struct i2c_device_id veml7700_id[] = {
    { "veml7700", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, veml7700_id);

static const struct of_device_id veml7700_of_match[] = {
    { .compatible = "vi,veml7700" },
    { }
};
MODULE_DEVICE_TABLE(of, veml7700_of_match);

static struct i2c_driver veml7700_driver = {
    .driver = {
        .name   = "veml7700",
        .of_match_table = of_match_ptr(veml7700_of_match),
    },
    .probe  = veml7700_probe,
    .remove  = veml7700_remove,
    .id_table = veml7700_id,
};

module_i2c_driver(veml7700_driver);

MODULE_AUTHOR("Gym Ok Cho <kimogi@teknique.com>");
MODULE_DESCRIPTION("Vishay VEML7700 light sensor driver");
MODULE_LICENSE("GPL");
