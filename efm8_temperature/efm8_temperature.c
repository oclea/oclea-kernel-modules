/*
 * efm8_temperature.c - Support for Oclea MCU temperature sensor
 */

#include <linux/init.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/version.h>

#include <linux/module.h>

#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>

#include <linux/i2c.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/thermal.h>

#define OCLEA_TEMP_REG_TEMP		0x10
#define OCLEA_TEMP_REG_CRITICAL		0x36

struct efm8_temperature_data {
    struct i2c_client          *client;
    struct thermal_zone_device *tz;
    struct device              *hwmon_dev;
    int                        resume_temp_mc;
};

static struct efm8_temperature_data *notify_data;

static int efm8_temperature_read_temp(void *dev, int *temp)
{
    int value;
    s16 s_temp;

    struct efm8_temperature_data *data = (struct efm8_temperature_data *)dev;
    if (IS_ERR(data)) {
        return PTR_ERR(data);
    }

    value = i2c_smbus_read_word_data(data->client, OCLEA_TEMP_REG_TEMP);
    if (value < 0) {
        return -EIO;
    }

    s_temp = (s16)(value & 0xffff);
    *temp = s_temp;
    *temp *= 100;

    return 0;
}

static int efm8_temperature_notify(struct thermal_zone_device *tzd, int temp,  enum thermal_trip_type type)
{
    u16 wake_temp_dc;

    if (!notify_data) {
        printk("Warning! Missing thermal wakeup data");
        return -1;
    }

    if (notify_data->resume_temp_mc < 0) {
        // No resume temperature specified, MCU will use default
        printk("No resume temperature specified!");
        return 0;
    }

    wake_temp_dc = (u16)(notify_data->resume_temp_mc / 100);

    if (THERMAL_TRIP_CRITICAL == type) {
        return i2c_smbus_write_word_data(notify_data->client, OCLEA_TEMP_REG_CRITICAL, wake_temp_dc);
    }

    return 0;
}

static umode_t efm8_temperature_is_visible(const void *data, enum hwmon_sensor_types type,
                                    u32 attr, int channel)
{
    switch (type) {
    case hwmon_chip:
        break;
    case hwmon_temp:
        switch (attr) {
        case hwmon_temp_input:
            return 0444;
        }

        break;
    default:
        break;
    }

    return 0;
}

static int efm8_temperature_read(struct device *dev, enum hwmon_sensor_types type,
                           u32 attr, int channel, long *val)
{
    struct efm8_temperature_data *data = dev_get_drvdata(dev);
    long regval;
    s16 s_temp;

    if (IS_ERR(data)) {
        return PTR_ERR(data);
    }

    switch (type) {
    case hwmon_chip:
        return -EINVAL;
    case hwmon_temp:
        switch (attr) {
        case hwmon_temp_input:
            regval = i2c_smbus_read_word_data(data->client, OCLEA_TEMP_REG_TEMP);
            break;
        default:
            return -EINVAL;
        }

        if (regval < 0) {
            return -EINVAL;
        }

        s_temp = (s16)(regval & 0xffff);

        *val = s_temp;
        *val *= 100;

        break;
    default:
        return -EINVAL;
    }

    return 0;
}

static int efm8_temperature_write(struct device *dev, enum hwmon_sensor_types type,
                            u32 attr, int channel, long val)
{
    return -EINVAL;
}

static const struct hwmon_channel_info *efm8_temperature_info[] = {
    HWMON_CHANNEL_INFO(chip, HWMON_C_REGISTER_TZ),
    HWMON_CHANNEL_INFO(temp, HWMON_T_INPUT),
    NULL
};

static const struct hwmon_ops efm8_temperature_hwmon_ops = {
    .is_visible = efm8_temperature_is_visible,
    .read = efm8_temperature_read,
    .write = efm8_temperature_write,
};

static const struct hwmon_chip_info efm8_temperature_chip_info = {
    .ops = &efm8_temperature_hwmon_ops,
    .info = efm8_temperature_info,
};

static const struct thermal_zone_of_device_ops efm8_temperature_thermal_ops = {
    .get_temp = efm8_temperature_read_temp,
};

static int efm8_temperature_probe(struct i2c_client *client,
             const struct i2c_device_id *id)
{
    struct device *dev = &client->dev;
    struct efm8_temperature_data *dev_data;
    int status;
    u32 wake_temp_mc;

    dev_data = devm_kzalloc(dev, sizeof(struct efm8_temperature_data), GFP_KERNEL);
    if (!dev_data) {
        return -ENOMEM;
    }

    dev_data->client = client;

    status = i2c_smbus_read_word_data(client, OCLEA_TEMP_REG_TEMP);
    if (status < 0) {
        dev_err(dev, "Can't read temperature, error: %d\n", status);
        return status;
    }

    if (of_property_read_u32(dev->of_node, "resume-temp-mc", &wake_temp_mc)) {
        dev_data->resume_temp_mc = -1;
    } else {
        dev_data->resume_temp_mc = (int)wake_temp_mc;
    }

    i2c_set_clientdata(client, dev_data);
    dev_set_drvdata(dev, dev_data);

    dev_data->hwmon_dev = devm_hwmon_device_register_with_info(dev,
                                                               "efm8_temperature",
                                                               dev_data,
                                                               &efm8_temperature_chip_info,
                                                               NULL);
    if (IS_ERR(dev_data->hwmon_dev)) {
        dev_err(dev, "Can't create hwmon device %ld\n", PTR_ERR(dev_data->hwmon_dev));
        return PTR_ERR(dev_data->hwmon_dev);
    }

    dev_data->tz = thermal_zone_of_sensor_register(dev,
                                                   0,
                                                   dev_data,
                                                   &efm8_temperature_thermal_ops);
    if (IS_ERR(dev_data->tz)) {
        dev_err(dev, "Failed to register thermal zone\n");
        devm_hwmon_device_unregister(dev_data->hwmon_dev);
        return PTR_ERR(dev_data->tz);
    }

    notify_data = dev_data;
    dev_data->tz->ops->notify = efm8_temperature_notify;

    dev_info(dev, "%s: sensor '%s'\n", dev_name(dev_data->hwmon_dev), client->name);

    return 0;
}

static int efm8_temperature_remove(struct i2c_client *client)
{
    struct efm8_temperature_data *temp_data = i2c_get_clientdata(client);

    if (!temp_data) {
        return 0;
    }

    if (temp_data->hwmon_dev) {
        devm_hwmon_device_unregister(temp_data->hwmon_dev);
    }

    if (temp_data->tz) {
        devm_thermal_zone_of_sensor_unregister(&client->dev, temp_data->tz);
    }

    return 0;
}

static const struct i2c_device_id efm8_temperature_id[] = {
    {"efm8-temperature", 0},
    {}
};
MODULE_DEVICE_TABLE(i2c, efm8_temperature_id);

static const struct of_device_id efm8_temperature_of_match[] = {
    {
        .compatible = "oclea,efm8-temperature",
        .data = &efm8_temperature_id[0]
    },
    {}
};
MODULE_DEVICE_TABLE(of, efm8_temperature_of_match);

static struct i2c_driver efm8_temperature_driver = {
    .probe = efm8_temperature_probe,
    .remove = efm8_temperature_remove,
    .id_table = efm8_temperature_id,
    .driver = {
           .name = "efm8-temperature",
           .of_match_table = of_match_ptr(efm8_temperature_of_match)
    },
};

module_i2c_driver(efm8_temperature_driver);

MODULE_DESCRIPTION("Oclea MCU temperature sensor driver");
MODULE_AUTHOR("");
MODULE_LICENSE("GPL");
