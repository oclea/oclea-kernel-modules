// SPDX-License-Identifier: GPL-2.0-only
/*
 * 3-axis accelerometer driver for the BMI088, using the i2c interface
 *
 * Copyright (c) 2023, Teknique Limited.
 * Copyright (c) 2014, Intel Corporation.
 */

#include <linux/device.h>
#include <linux/mod_devicetable.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/acpi.h>
#include <linux/regmap.h>

#include "bmi088-accel.h"

static int bmi088_accel_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	struct regmap *regmap;
	const char *name = NULL;
	int ret;

	bool block_supported =
		i2c_check_functionality(client->adapter, I2C_FUNC_I2C) ||
		i2c_check_functionality(client->adapter,
					I2C_FUNC_SMBUS_READ_I2C_BLOCK);

	regmap = devm_regmap_init_i2c(client, &bmi088_regmap_conf);
	if (IS_ERR(regmap)) {
		dev_err(&client->dev, "Failed to initialize i2c regmap\n");
		return PTR_ERR(regmap);
	}

	if (id)
		name = id->name;

	ret = bmi088_accel_core_probe(&client->dev, regmap, client->irq, name, block_supported);
	if (ret)
		return ret;


	dev_info(&client->dev, "Initialised bmi088-accel-i2c");

	return 0;
}

static int bmi088_accel_remove(struct i2c_client *client)
{
	bmi088_accel_core_remove(&client->dev);

	return 0;
}

static const struct acpi_device_id bmi088_accel_acpi_match[] = {
	{"BMI088"},
	{ },
};
MODULE_DEVICE_TABLE(acpi, bmi088_accel_acpi_match);

static const struct i2c_device_id bmi088_accel_id[] = {
	{"bmi088_accel"},
	{}
};

MODULE_DEVICE_TABLE(i2c, bmi088_accel_id);

static const struct of_device_id bmi088_accel_of_match[] = {
	{ .compatible = "bosch,bmi088_accel" },
	{ },
};
MODULE_DEVICE_TABLE(of, bmi088_accel_of_match);

static struct i2c_driver bmi088_accel_driver = {
	.driver = {
		.name	= "bmi088_accel_i2c",
		.of_match_table = bmi088_accel_of_match,
		.acpi_match_table = ACPI_PTR(bmi088_accel_acpi_match),
		.pm	= &bmi088_accel_pm_ops,
	},
	.probe		= bmi088_accel_probe,
	.remove		= bmi088_accel_remove,
	.id_table	= bmi088_accel_id,
};
module_i2c_driver(bmi088_accel_driver);

MODULE_AUTHOR("Brychan Dempsey-Jensen <brychan.d@teknique.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("BMI088 I2C accelerometer driver");