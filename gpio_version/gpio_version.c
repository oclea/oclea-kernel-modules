/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * gpio_version.c - HW version check by gpios
 *
 *
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>

#include "gpio_version.h"

static const struct of_device_id gpio_ver_match[] = {
	{
		.compatible = "oclea,gpio_version",
	},
	{},
};
MODULE_DEVICE_TABLE(of, gpio_ver_match);

static struct gpio_version_platform_data *
gpio_version_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct gpio_version_platform_data *pdata;
	int ret;

	if (!np)
		return NULL;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	pdata->input_gpios = devm_gpiod_get_array(dev, NULL, GPIOD_IN);
	if (IS_ERR(pdata->input_gpios)) {
		dev_err(dev, "unable to acquire input gpios\n");
		return PTR_ERR(pdata->input_gpios);
	}
	if (pdata->input_gpios->ndescs < 2) {
		dev_err(dev, "not enough gpios found\n");
		return -EINVAL;
	}

	pdata->max_versions = of_property_count_strings(np, "versions");
	if (pdata->max_versions < 1) {
		dev_err(dev, "At least one version must be listed\n");
		return pdata->max_versions;
	}

	int i;
	pdata->version_names = devm_kzalloc(
		dev, pdata->max_versions * sizeof(*pdata->version_names),
		GFP_KERNEL);
	for (i = 0; i < pdata->max_versions; i++) {
		pdata->version_names[i] = devm_kzalloc(
			dev,
			MAX_VERSION_NAME_SIZE * sizeof(**pdata->version_names),
			GFP_KERNEL);
	}
	ret = of_property_read_string_array(
		np, "versions", pdata->version_names, pdata->max_versions);
	if (ret < 0) {
		dev_err(dev, "Could not read versions\n");
		return ret;
	}

	return pdata;
}

static int gpio_ver_get_gpios_state(struct device *dev)
{
	struct gpio_version_platform_data *pdata = dev_get_drvdata(dev);
	struct gpio_descs *gpios = pdata->input_gpios;
	unsigned int ret = 0;
	int i, val;

	for (i = 0; i < gpios->ndescs; i++) {
		val = gpiod_get_value(gpios->desc[i]);
		if (val < 0) {
			dev_err(dev, "Error reading gpio %d: %d\n",
				desc_to_gpio(gpios->desc[i]), val);
			return val;
		}

		val = !!val;
		ret = (ret << 1) | val;
	}

	return ret;
}

static ssize_t gpio_ver_show_type(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "4\n");
}

static ssize_t gpio_ver_show_version(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	int gpio_states = gpio_ver_get_gpios_state(dev);
	struct gpio_version_platform_data *pdata = dev_get_drvdata(dev);

	char ver;
	if (gpio_states >= pdata->max_versions) {
		dev_err(dev, "No version name registered for gpio version %d\n",
			gpio_states);
		return -EINVAL;
	}

	return sprintf(buf, "%s\n", pdata->version_names[gpio_states]);
}

static SENSOR_DEVICE_ATTR(ver1_type, S_IRUGO, gpio_ver_show_type, NULL, 0);
static SENSOR_DEVICE_ATTR(ver1_input, S_IRUGO, gpio_ver_show_version, NULL, 0);

static struct attribute *gpio_ver_attrs[] = {
	&sensor_dev_attr_ver1_type.dev_attr.attr,
	&sensor_dev_attr_ver1_input.dev_attr.attr,
	NULL,
};
ATTRIBUTE_GROUPS(gpio_ver); //defines gpio_ver_groups[], refer to sysfs.h

static int gpio_version_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct of_device_id *of_id =
		of_match_device(of_match_ptr(gpio_ver_match), dev);
	struct gpio_version_platform_data *pdata;
	struct device *hwmon_dev;
	struct device_node *np = dev->of_node;
	const char *name_buf;

	pdata = gpio_version_parse_dt(dev);
	if (IS_ERR(pdata))
		return PTR_ERR(pdata);
	else if (pdata == NULL)
		pdata = dev_get_platdata(dev);

	if (!pdata) {
		return -ENODEV;
	}

	hwmon_dev = devm_hwmon_device_register_with_groups(
		dev, "gpio_version", pdata, gpio_ver_groups);
	if (IS_ERR(hwmon_dev)) {
		dev_err(dev, "unable to register as hwmon device.\n");
		return PTR_ERR(hwmon_dev);
	}

	if (of_property_read_string(np, "alias-name", &name_buf) == 0) {
		if (sysfs_create_link(&dev->kobj, &hwmon_dev->kobj, name_buf) !=
		    0) {
			dev_err(dev, "failed to set alias for %s\n", name_buf);
		}
	}

	return 0;
}

static struct platform_driver gpio_version_driver = {
    .driver = {
        .name = "gpio_version",
        .of_match_table = of_match_ptr(gpio_ver_match),
    },
    .probe = gpio_version_probe,
};

module_platform_driver(gpio_version_driver);

MODULE_DESCRIPTION("GPIO driver using pins to form a HW version");
MODULE_AUTHOR("Vinuli Jayasekera <vinuli.j@teknique.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:gpio-version-driver");
