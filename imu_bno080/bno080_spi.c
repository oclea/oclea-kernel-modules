/*
 * BMI160 - Bosch IMU, SPI bits
 *
 * Copyright (c) 2016, Intel Corporation.
 *
 * This file is subject to the terms and conditions of version 2 of
 * the GNU General Public License.  See the file COPYING in the main
 * directory of this archive for more details.
 */
#include <linux/module.h>
#include <linux/spi/spi.h>

#include "bno080.h"

int bno080_spi_read_multiple_byte(struct device *dev, uint8_t *data, int len)
{
	return 0;
}

int bno080_spi_write_multiple_byte(struct device *dev, uint8_t *data, int len)
{
	return 0;
}

static struct bno080_api_hal_ops bno080_spi_hal_ops = {
	.read_multiple_byte = bno080_spi_read_multiple_byte,
	.write_multiple_byte = bno080_spi_write_multiple_byte,
};

static int bno080_spi_probe(struct spi_device *spi)
{
	const struct spi_device_id *id = spi_get_device_id(spi);

	return bno080_core_probe(&spi->dev, &bno080_spi_hal_ops, id->name);
}

static int bno080_spi_remove(struct spi_device *spi)
{
	bno080_core_remove(&spi->dev);

	return 0;
}

static const struct spi_device_id bno080_spi_id[] = { { "bno080", 0 }, {} };
MODULE_DEVICE_TABLE(spi, bno080_spi_id);

static const struct of_device_id bno080_dt_ids[] = {
	{
		.compatible = "oclea,bno080",
	},
	{},
};

static struct spi_driver bno080_spi_driver = {
	.driver =
		{
			.name = "bno080",
			.of_match_table = bno080_dt_ids,
		},
	.probe = bno080_spi_probe,
	.remove = bno080_spi_remove,
	.id_table = bno080_spi_id,
};
module_spi_driver(bno080_spi_driver);

MODULE_AUTHOR("Gym Ok Cho <kimogi@teknique.com");
MODULE_DESCRIPTION("Hillcrest Labs BNO080 SPI driver");
MODULE_LICENSE("GPL v2");
