/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * adc_version.c - HW version check by ADC
 *
 *
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/math64.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include <linux/iio/iio.h>
#include <linux/iio/machine.h>
#include <linux/iio/driver.h>
#include <linux/iio/consumer.h>

#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>

#include "adc_version.h"

struct adc_level {
    int	max_mv;
    int	version;
};

/* Order matters, adc_ver_match references the entries by index
 * new types to be added for future platforms
 */
static const struct platform_device_id adc_version_id[] = {
    { "adcver_s5l", TYPE_ADCVER_S5L },
    { "adcver_cv22", TYPE_ADCVER_CV22 },
    { "adcver_cv22_usom", TYPE_ADCVER_CV22_USOM },
    { "adcver_s6lm", TYPE_ADCVER_S6LM },
    { "adcver_cv25", TYPE_ADCVER_CV25 },
};

/*
 * A conversion table should be sorted by the values of .max_mv
 * in ascending order.
 * 4% tolerance is given either way (actual resistors have 1% tolerance)
 */
static const struct adc_level s5l_uSOM[] = {
    { .max_mv = 1017, .version = 97 },  //invalid range
    { .max_mv = 1103, .version = 3 },   //EVT3 = 1.06V
    { .max_mv = 1584, .version = 0 },
    { .max_mv = 1716, .version = 1 },   //EVT1 = 1.65V
    { .max_mv = 2237, .version = 98 },  //invalid range
    { .max_mv = 2423, .version = 2 },   //EVT2 = 2.33V
    { .max_mv = 2611, .version = 96 },
    { .max_mv = 2828, .version = 4 },   //DVT1 = 2.72V
    { .max_mv = 3300, .version = 99 }
};

static const struct adc_level cv22_SOM[] = {
    { .max_mv = 1584, .version = 0 },
    { .max_mv = 1716, .version = 1 },   //EVT0 = 1.65V
    { .max_mv = 2237, .version = 97 },  //invalid range
    { .max_mv = 2423, .version = 2 },   //EVT1 = 2.33V
    { .max_mv = 2592, .version = 98 },  //invalid range
    { .max_mv = 2808, .version = 3 },   //EVT1* = 2.7V
    { .max_mv = 3300, .version = 91 }   //prototype
};

// uSOM has a 1.8V pullup but still has internal 3.3V reference
// so needs a new table

static const struct adc_level cv22_uSOM[] = {
    { .max_mv = 330, .version = 1 },	// EVT0 = 0.315V
    { .max_mv = 1424, .version = 98 },   // Invalid
    { .max_mv = 1529, .version = 2 }, 	// EVT1 = 1.484V
    { .max_mv = 1637, .version = 3 },   // EVT2 = 1.574V
    { .max_mv = 1800, .version = 99 },
};

// TODO: Check S6Lm version is correct
static const struct adc_level s6lm_uSOM[] = {
    { .max_mv = 1529, .version = 1 }, 	// EVT1 = 1.484V
    { .max_mv = 1637, .version = 2 },   // EVT2 = 1.574V
    { .max_mv = 1800, .version = 99 },
};

static const struct adc_level cv25_uSOM[] = {
    { .max_mv = 864, .version = 98 },   // Invalid
    { .max_mv = 936, .version = 1 },    // EVT0 = 0.9V
    { .max_mv = 1800, .version = 99 },   // Invalid
};

struct adc_ver_data {
    struct adc_version_platform_data *pdata;
    const struct adc_level *conv;
    int n_conv;
};

#if defined(CONFIG_OF) && IS_ENABLED(CONFIG_IIO)

static int adc_ver_iio_read(struct adc_version_platform_data *pdata)
{
    struct iio_channel *channel = pdata->chan;
    int raw, mv, ret;

    ret = iio_read_channel_raw(channel, &raw);
    if (ret < 0) {
        pr_err("read channel() error: %d\n", ret);
        return ret;
    }

    ret = iio_convert_raw_to_processed(channel, raw, &mv, 1);
    if (ret < 0) {
        /* Assume 12 bit ADC with vref at pullup_uv */
        mv = (pdata->pullup_mv * (s64)raw) >> 12;
    }

    return mv;
}

static const struct of_device_id adc_ver_match[] = {
    { .compatible = "oclea,s5l_uSOM",
        .data = &adc_version_id[0] },
    { .compatible = "oclea,cv22_SOM",
        .data = &adc_version_id[1] },
    { .compatible = "oclea,cv22_uSOM",
        .data = &adc_version_id[2] },
    { .compatible = "oclea,s6lm_uSOM",
        .data = &adc_version_id[3] },
    { .compatible = "oclea,cv25_uSOM",
        .data = &adc_version_id[4] },
    { },
};
MODULE_DEVICE_TABLE(of, adc_ver_match);

static struct adc_version_platform_data *
adc_version_parse_dt(struct device *dev)
{
    struct iio_channel *chan;
    enum iio_chan_type type;
    struct device_node *np = dev->of_node;
    struct adc_version_platform_data *pdata;
    int ret;

    if (!np)
        return NULL;

    pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
    if (!pdata)
        return ERR_PTR(-ENOMEM);

    chan = devm_iio_channel_get(dev, NULL);
    if (IS_ERR(chan))
        return ERR_CAST(chan);

    ret = iio_get_channel_type(chan, &type);
    if (ret < 0)
        return ERR_PTR(ret);

    if (type != IIO_VOLTAGE)
        return ERR_PTR(-EINVAL);

    if (of_property_read_u32(np, "pullup-mv", &pdata->pullup_mv))
        return ERR_PTR(-ENODEV);

    pdata->chan = chan;
    pdata->read_mv = adc_ver_iio_read;

    return pdata;
}
#else
static struct adc_version_platform_data *
adc_version_parse_dt(struct device *dev)
{
    return NULL;
}

#define adc_ver_match	NULL

#endif



static int lookup_conv(struct adc_ver_data *data, unsigned int mv)
{
    int i;
    for(i=0; i<data->n_conv; i++){
        if(mv < data->conv[i].max_mv){
            return data->conv[i].version;
        }
    }
    return -EINVAL;
}

static int get_adc_ver(struct adc_ver_data *data, unsigned int mv)
{
    return lookup_conv(data, mv);
}

static int adc_ver_get_mv(struct adc_ver_data *data)
{
    if (data->pdata->read_mv) {
        return data->pdata->read_mv(data->pdata);
    }
    return -EINVAL;
}

static ssize_t adc_ver_show_type(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "4\n");
}

static ssize_t adc_ver_show_version(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct adc_ver_data *data = dev_get_drvdata(dev);

    int mv;
    mv = adc_ver_get_mv(data);
    if (mv < 0)
        return mv;

    return sprintf(buf, "%d\n", get_adc_ver(data, mv));
}

static SENSOR_DEVICE_ATTR(ver1_type, S_IRUGO, adc_ver_show_type, NULL, 0);
static SENSOR_DEVICE_ATTR(ver1_input, S_IRUGO, adc_ver_show_version, NULL, 0);

static struct attribute *adc_ver_attrs[] = {
    &sensor_dev_attr_ver1_type.dev_attr.attr,
    &sensor_dev_attr_ver1_input.dev_attr.attr,
    NULL,
};
ATTRIBUTE_GROUPS(adc_ver);  //defines adc_ver_groups[], refer to sysfs.h

static int adc_version_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    const struct of_device_id *of_id =
            of_match_device(of_match_ptr(adc_ver_match), dev);
    const struct platform_device_id *pdev_id;
    struct adc_version_platform_data *pdata;
    struct device *hwmon_dev;
    struct adc_ver_data *data;
    struct device_node *np = dev->of_node;
    const char *name_buf;
    pdata = adc_version_parse_dt(dev);
    if (IS_ERR(pdata))
        return PTR_ERR(pdata);
    else if (pdata == NULL)
        pdata = dev_get_platdata(dev);

    if (!pdata) {
        return -ENODEV;
    }

    /* Either one of the two is required. */
    if (!pdata->read_mv) {
        return -EINVAL;
    }

    data = devm_kzalloc(dev, sizeof(struct adc_ver_data), GFP_KERNEL);
    if (!data){
        return -ENOMEM;
    }

    pdev_id = of_id ? of_id->data : platform_get_device_id(pdev);

    data->pdata = pdata;

    switch (pdev_id->driver_data) {
    case TYPE_ADCVER_S5L:
        data->conv = s5l_uSOM;
        data->n_conv = ARRAY_SIZE(s5l_uSOM);
        break;
    case TYPE_ADCVER_CV22:
        data->conv = cv22_SOM;
        data->n_conv = ARRAY_SIZE(cv22_SOM);
        break;
    case TYPE_ADCVER_CV22_USOM:
        data->conv = cv22_uSOM;
        data->n_conv = ARRAY_SIZE(cv22_uSOM);
        break;
    case TYPE_ADCVER_S6LM:
        data->conv = s6lm_uSOM;
        data->n_conv = ARRAY_SIZE(s6lm_uSOM);
        break;
    case TYPE_ADCVER_CV25:
        data->conv = cv25_uSOM;
        data->n_conv = ARRAY_SIZE(cv25_uSOM);
        break;
    default:
        dev_err(dev, "Unknown device type: %lu(%s)\n",
                pdev_id->driver_data, pdev_id->name);
        return -EINVAL;
    }
    hwmon_dev = devm_hwmon_device_register_with_groups(dev, pdev_id->name,
                               data, adc_ver_groups);
    if (IS_ERR(hwmon_dev)) {
        dev_err(dev, "unable to register as hwmon device.\n");
        return PTR_ERR(hwmon_dev);
    }

    dev_info(dev, "adc_ver type: %s successfully probed.\n",
             pdev_id->name);

    if (of_property_read_string(np, "alias-name", &name_buf) == 0) {
        if (sysfs_create_link(&dev->kobj, &hwmon_dev->kobj, name_buf) != 0) {
            dev_err(dev, "failed to set alias for %s\n", name_buf);
        }
    }

    return 0;
}

static struct platform_driver adc_version_driver = {
    .driver = {
        .name = "adc_version",
        .of_match_table = of_match_ptr(adc_ver_match),
    },
    .probe = adc_version_probe,
    .id_table = adc_version_id,
};

module_platform_driver(adc_version_driver);

MODULE_DESCRIPTION("ADC version Driver");
MODULE_AUTHOR("Oclea <info@oclea.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:adc-driver");
