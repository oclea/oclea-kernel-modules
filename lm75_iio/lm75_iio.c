/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * lm75_iio.c - Support for LM75 temperature sensor
 *
 */

#include <linux/init.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/version.h>

#include <linux/module.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>

#include <linux/i2c.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/regmap.h>

#define LM75_TEMP_MIN (-55000)
#define LM75_TEMP_MAX 125000
#define LM75_SHUTDOWN 0x01

// The LM75 registers
#define LM75_REG_TEMP		0x00
#define LM75_REG_CONF		0x01
#define LM75_REG_HYST		0x02
#define LM75_REG_MAX		0x03

// Each client has this additional data
struct lm75_data {
    const char          *dev_name;
    struct i2c_client   *client;
    struct regmap       *regmap;
    u8                   orig_conf;
    u8                   resolution;	// In bits, between 9 and 12 /
    u8                   resolution_limits;
    unsigned int         sample_time;	// In ms
};

static bool lm75_is_writeable_reg(struct device *dev, unsigned int reg)
{
    return reg != LM75_REG_TEMP;
}

static bool lm75_is_volatile_reg(struct device *dev, unsigned int reg)
{
    return reg == LM75_REG_TEMP;
}

static const struct regmap_config lm75_regmap_config = {
    .reg_bits = 8,
    .val_bits = 16,
    .max_register = LM75_REG_MAX,
    .writeable_reg = lm75_is_writeable_reg,
    .volatile_reg = lm75_is_volatile_reg,
    .val_format_endian = REGMAP_ENDIAN_BIG,
    .cache_type = REGCACHE_RBTREE,
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
    .use_single_rw = true,
#else
    .use_single_read = true,
    .use_single_write = true,
#endif
};

static const struct iio_chan_spec lm75_iio_channels[] = {
    {
        .type = IIO_TEMP,
        .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_PROCESSED),
        .info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),
    }
};

static ssize_t lm75_show_type(struct device *dev,
                              struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "4\n");
}

static inline long lm75_reg_to_mc(s16 temp, u8 resolution)
{
    return (temp * 1000) >> 8 ;
}

static ssize_t lm75_show_temp(struct device *dev,
                              struct device_attribute *attr, char *buf)
{
    struct lm75_data *data = iio_priv(dev_get_drvdata(dev));
    int err;
    long mc;
    unsigned int regval;

    if(!data->regmap) {
        printk("null regmap %s, %s\n", dev->init_name, data->dev_name);
        return -1;
    }

    err = regmap_read(data->regmap, LM75_REG_TEMP, &regval);
    if (err < 0) {
        return err;
    }

    mc = lm75_reg_to_mc(regval, data->resolution);

    return sprintf(buf, "%ld\n", mc);
}

static SENSOR_DEVICE_ATTR(temp_type, S_IRUGO, lm75_show_type, NULL, 0);
static SENSOR_DEVICE_ATTR(temp_input, S_IRUGO, lm75_show_temp, NULL, 0);

static struct attribute *lm75_iio_attrs[] = {
    &sensor_dev_attr_temp_type.dev_attr.attr,
    &sensor_dev_attr_temp_input.dev_attr.attr,
    NULL,
};
ATTRIBUTE_GROUPS(lm75_iio);  //defines lm75_iio_group(s), refer to sysfs.h

static const struct iio_info lm75_iio_info = {
    .attrs = &lm75_iio_group,
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
    .driver_module = THIS_MODULE,
#endif
};

static int lm75_iio_parse_dt(struct device *dev, struct lm75_data *data)
{
    struct device_node *node = dev->of_node;

    if (!node) {
        return -ENODEV;
    }

    if (of_property_read_string(node, "dev-name", &data->dev_name) != 0) {
        data->dev_name = NULL;
    }

    return 0;
}

/////////////////////////////////////////////////////////////////
static int lm75_iio_probe(struct i2c_client *client,
             const struct i2c_device_id *id)
{
    struct device *dev = &client->dev;
    struct iio_dev *indio_dev;
    struct device *hwmon_dev;
    struct lm75_data *dev_data;
    int rval;
    int status;
    u8 set_mask, clr_mask;
    int new;
    int i;
    const char *name_buf;

    if (!i2c_check_functionality(client->adapter,
                     I2C_FUNC_SMBUS_WRITE_BYTE_DATA |
                     I2C_FUNC_SMBUS_WRITE_BYTE |
                     I2C_FUNC_SMBUS_READ_I2C_BLOCK)) {
        dev_err(&client->dev,
            "Adapter does not support some i2c transaction\n");
        return -EOPNOTSUPP;
    }

    indio_dev = devm_iio_device_alloc(dev, sizeof(*dev_data));

    if (!indio_dev) {
        return -ENOMEM;
    }

    dev_data = iio_priv(indio_dev);
    dev_data->client = client;

    rval = lm75_iio_parse_dt(dev, dev_data);
    if (rval < 0) {
        dev_err(dev, "failed to parse dt %d!\n", rval);
        return -EINVAL;
    }

    if (dev_data->dev_name) {
        dev_set_name(&indio_dev->dev, dev_data->dev_name);
    } else {
        dev_set_name(&indio_dev->dev, "iio:lm75_iio");
    }

    dev_data->regmap = devm_regmap_init_i2c(client, &lm75_regmap_config);
    if (IS_ERR(dev_data->regmap)) {
        dev_err(dev, "failed to init regmap %ld!\n", PTR_ERR(dev_data->regmap));
        return PTR_ERR(dev_data->regmap);
    }

    //HERE it comes the rest of lm75 config
    set_mask = 0;
    clr_mask = LM75_SHUTDOWN;

    dev_data->resolution = 9;  //for lm75, lm75a
    dev_data->resolution_limits = 99;//for debug
    dev_data->sample_time = MSEC_PER_SEC / 2; //for lm75, lm75a

    //read the existing config, save, then apply the new config
    status = i2c_smbus_read_byte_data(client, LM75_REG_CONF);
    if (status < 0) {
        dev_err(dev, "Can't read config? %d\n", status);
        return status;
    }
    dev_data->orig_conf = status;
    new = status & ~clr_mask;
    new |= set_mask;
    if (status != new) {
        i2c_smbus_write_byte_data(client, LM75_REG_CONF, new);
    }

    //now, IIO:
    indio_dev->info = &lm75_iio_info;
    indio_dev->name = dev_data->dev_name;
    indio_dev->dev.parent = dev;
    indio_dev->modes = INDIO_DIRECT_MODE;
    indio_dev->channels = lm75_iio_channels;
    indio_dev->num_channels = ARRAY_SIZE(lm75_iio_channels);

    i2c_set_clientdata(client, indio_dev);

    i = devm_iio_device_register(dev, indio_dev);
    if (i < 0) {
        dev_err(dev, "failed to register iio device %d!\n", i);
        return -ENXIO;
    }

    // Pass the IIO device as the data to hwmon register, to match what is
    // passed as data for IIO device (so lm75_show_temp works the same).
    hwmon_dev = devm_hwmon_device_register_with_groups(dev,
                                                       dev_data->dev_name,
                                                       indio_dev,
                                                       lm75_iio_groups);
    if (IS_ERR(hwmon_dev)) {
        dev_err(dev, "Can't create hwmon device %ld\n", PTR_ERR(hwmon_dev));
        return PTR_ERR(hwmon_dev);
    }

    if (of_property_read_string(dev->of_node, "alias-name", &name_buf) == 0) {
        if (sysfs_create_link(&dev->kobj, &hwmon_dev->kobj, name_buf) != 0) {
            dev_err(dev, "failed to set alias for %s\n", name_buf);
        }
    }

    dev_info(dev, "%s: sensor '%s'\n", dev_name(hwmon_dev), indio_dev->name);

    return i;
}

static const struct i2c_device_id lm75_iio_id[] = {
    {"lm75a", 0},
    {}
};
MODULE_DEVICE_TABLE(i2c, lm75_iio_id);

static const struct of_device_id lm75_iio_of_match[] = {
    {
        .compatible = "ti,lm75a",
        .data = &lm75_iio_id[0]
    },
    {}
};
MODULE_DEVICE_TABLE(of, lm75_iio_of_match);

static struct i2c_driver lm75_iio_driver = {
    .probe = lm75_iio_probe,
    .id_table = lm75_iio_id,
    .driver = {
           .name = "lm75_iio",
           .of_match_table = of_match_ptr(lm75_iio_of_match)
           },
};

module_i2c_driver(lm75_iio_driver);

MODULE_DESCRIPTION("LM75 temperature sensor iio-based driver");
MODULE_AUTHOR("");
MODULE_LICENSE("GPL v2");
