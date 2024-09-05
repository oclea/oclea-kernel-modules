// SPDX-License-Identifier: GPL-2.0+
/*
 * Device driver for monitoring ambient light intensity in (lux) of cg5162tc device.
 *
 * Copyright (c) 2021 Teknique Limited <will@teknique.com>
 */

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/regulator/consumer.h>

/* Lux calculation constants */
#define CG5162TC_LUX_CALC_OVER_FLOW 65535

/*
 * Register definitions
 */

/* Register offsets */
#define CG5162TC_MAX_CONFIG_REG 0x25

/* Device Registers and Masks */
#define CG5162TC_PRODUCT_NO_1 0x00
#define CG5162TC_PRODUCT_NO_2 0x01
#define CG5162TC_OP_MODE 0X03
#define CG5162TC_TIG_SEL 0x04
#define CG5162TC_CURRENT_GAIN 0x05
#define CG5162TC_UPDATE 0x20
#define CG5162TC_ALS_CHAN0L 0x21
#define CG5162TC_ALS_CHAN0H 0x22
#define CG5162TC_ALS_CHAN1L 0x23
#define CG5162TC_ALS_CHAN1H 0x24

/* reg masks */
#define CG5162TC_OP_DEF 0x04
#define CG5162TC_OP_POWER_DOWN 0x02
#define CG5162TC_UPDATE_READY 0x01

enum { CG5162TC_CHIP_UNKNOWN = 0,
       CG5162TC_CHIP_WORKING = 1,
       CG5162TC_CHIP_SUSPENDED = 2 };

/* Per-device data */
struct cg5162tc_als_info {
	u16 als_ch0;
	u16 als_ch1;
	u16 lux;
};

static IIO_CONST_ATTR_INT_TIME_AVAIL("2.7 5.4 51.3 100 200 400 688.5");

/*
 * Lookup tables for integration times, the datasheet doesn't explicitly
 * state we can support any value from 0x01-0xff * 2.7ms, so we just map
 * their supplied table.
 */
#define TIG_VALUE_COUNT 7

/*
 * IIO module values
 */
static int tig_values_int[TIG_VALUE_COUNT] = { 2, 5, 51, 100, 200, 400, 688 };
static int tig_values_frac[TIG_VALUE_COUNT] = { 700000, 400000, 300000, 0,
						0,	0,	500000 };

/*
 * Device specific register values.
 */
static unsigned char chip_tig[TIG_VALUE_COUNT] = { 0x01, 0x02, 0x13, 0x25,
						   0x4A, 0x94, 0xFF };

/*
 * IIO defined attribute for available integration times.
 */
static struct attribute *cg5162tc_attributes[] = {
	&iio_const_attr_integration_time_available.dev_attr.attr, NULL
};

static const struct attribute_group cg5162tc_attribute_group = {
	.attrs = cg5162tc_attributes,
};

/*
 * Channels available on our device, note IIO_INTENSITY channels
 * are for raw ADC debug only and require a read of the processed value at
 * in_illuminance0_input to populated them.
 */
static const struct iio_chan_spec cg5162tc_channels[] = {
	{
		.type = IIO_LIGHT,
		.indexed = 1,
		.channel = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED) |
				      BIT(IIO_CHAN_INFO_HARDWAREGAIN) |
				      BIT(IIO_CHAN_INFO_SCALE),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_INT_TIME),
	},
	{
		.type = IIO_INTENSITY,
		.indexed = 1,
		.channel = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
	{
		.type = IIO_INTENSITY,
		.indexed = 1,
		.channel = 1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
};

/*
 * Struct to hold our als data.
 */
struct cg5162tc {
	struct i2c_client *client;
	struct device *dev;
	u8 cg5162tc_config[CG5162TC_MAX_CONFIG_REG];
	struct mutex als_mutex;
	struct cg5162tc_als_info als_cur_info;
	int cgain0;
	int int_time_raw;
	int int_time;
	int int_time_frac;
	int scale;
};

/*
 * cg5162tc_enable() - Power the device up/down.
 * @chip:	pointer to our device data
 * @enable:   0 to disable, 1 to enable
 */
static int cg5162tc_enable(struct cg5162tc *chip, int enable)
{
	int reg = CG5162TC_OP_DEF;
	int ret;

	if (!enable) {
		reg |= CG5162TC_OP_POWER_DOWN;
	}

	// get back into a known state by powering down and back up
	ret = i2c_smbus_write_byte_data(chip->client, CG5162TC_OP_MODE, reg);

	if (ret < 0) {
		dev_err(&chip->client->dev, "%s: failed to power %s\n",
			__func__, enable ? "up" : "down");
		return ret;
	}

	return 0;
}

/*
 * cg5162tc_read_status() - Read the ALS status register, triggers, if non-zero
 * then a read is triggered, we must sleep minimum integration time before
 * reading out ADC registers.
 * @chip:	pointer to our device data
 */
static int cg5162tc_read_status(struct cg5162tc *chip)
{
	int ret;

	ret = i2c_smbus_read_byte_data(chip->client, CG5162TC_UPDATE);
	if (ret < 0) {
		dev_err(&chip->client->dev,
			"%s: failed to read STATUS register: %d\n", __func__,
			ret);
	}

	return ret;
}

/*
 * Set the integration time.
 * cg5162tc_set_tig() - Set the integration time.
 * @chip:	    pointer to our device data
 * @int_part:   integer part of the number
 * @frac_part:  fraction part of the number
 */
static int cg5162tc_set_tig(struct cg5162tc *chip, int int_part, int frac_part)
{
	int i, ret;
	for (i = 0; i < TIG_VALUE_COUNT; i++) {
		if (int_part == tig_values_int[i] &&
		    frac_part == tig_values_frac[i]) {
			ret = i2c_smbus_write_byte_data(
				chip->client, CG5162TC_TIG_SEL, chip_tig[i]);
			if (ret < 0) {
				dev_err(&chip->client->dev,
					"%s: failed to set integration time\n",
					__func__);
				return ret;
			} else {
				chip->int_time_raw = chip_tig[i];
				chip->int_time = int_part;
				chip->int_time_frac = frac_part;
				return 0;
			}
		}
	}
	return -EINVAL;
}

/*
 * cg5162tc_set_gain() - Set the channel gain to increase light sensitivity.
 * @chip:	pointer to out device data
 * @gain:   gain between 1-15
 */
static int cg5162tc_set_gain(struct cg5162tc *chip, int gain)
{
	int g, ret;
	if (gain < 0 || gain > 0xf) {
		return -EINVAL;
	}

	// gain needs to match on both channels
	g = gain | (gain << 4);
	ret = i2c_smbus_write_byte_data(chip->client, CG5162TC_CURRENT_GAIN, g);
	if (ret < 0) {
		dev_err(&chip->client->dev, "%s: failed to set gain\n",
			__func__);
		return ret;
	} else {
		chip->cgain0 = gain;
	}

	return 0;
}

/**
 * cg5162tc_get_lux() - Reads and calculates current lux value.
 * @dev:	pointer to IIO device
 *
 * A read to the status register triggers ambient light sensor integration
 * cycle, then ch0 and ch1 values are read and lux is calculated based on
 * integration time, gain and device specific scaling factor. See datasheet
 * for calculation.
 */
static int cg5162tc_get_lux(struct iio_dev *dev)
{
	struct cg5162tc *chip = iio_priv(dev);
	int chan0, chan1, ret;
	int64_t calc;

	// get status to trigger a read
	ret = cg5162tc_read_status(chip);
	if (ret < 0) {
		dev_err(&chip->client->dev, "%s: failed to read status\n",
			__func__);
		goto done;
	}

	//printk("chip status: %02x\n", ret);

	if (!(ret & CG5162TC_UPDATE_READY)) {
		dev_err(&chip->client->dev, "%s: device not powered on\n",
			__func__);
		goto done;
	}

	// sleep time is integration time * 2.7ms per step
	// min time we add on 1 ms
	// max time we can stretch out to TIG+(TIG/2)
	unsigned long sleep_us_min = (chip->int_time_raw * 2700);
	unsigned long sleep_us_max = sleep_us_min + (sleep_us_min / 2);

	// add at least a millisecond onto the minimum integration time.
	// 5ms if the integration time is larger than 10ms
	sleep_us_min += sleep_us_min < 10000 ? 1000 : 5000;
	usleep_range(sleep_us_min, sleep_us_max);

	// read channel registers
	ret = i2c_smbus_read_byte_data(chip->client, CG5162TC_ALS_CHAN0L);
	if (ret < 0) {
		dev_err(&chip->client->dev,
			"%s: failed to read CG5162TC_ALS_CHAN0L register: %d\n",
			__func__, ret);
		goto done;
	}
	chan0 = ret & 0xff;

	ret = i2c_smbus_read_byte_data(chip->client, CG5162TC_ALS_CHAN0H);
	if (ret < 0) {
		dev_err(&chip->client->dev,
			"%s: failed to read CG5162TC_ALS_CHAN0L register: %d\n",
			__func__, ret);
		goto done;
	}
	chan0 |= ((ret << 8) & 0xff00);

	ret = i2c_smbus_read_byte_data(chip->client, CG5162TC_ALS_CHAN1L);
	if (ret < 0) {
		dev_err(&chip->client->dev,
			"%s: failed to read CG5162TC_ALS_CHAN0L register: %d\n",
			__func__, ret);
		goto done;
	}
	chan1 = ret & 0xff;

	ret = i2c_smbus_read_byte_data(chip->client, CG5162TC_ALS_CHAN1H);
	if (ret < 0) {
		dev_err(&chip->client->dev,
			"%s: failed to read CG5162TC_ALS_CHAN0L register: %d\n",
			__func__, ret);
		goto done;
	}
	chan1 |= ((ret << 8) & 0xff00);

	chip->als_cur_info.als_ch0 = chan0;
	chip->als_cur_info.als_ch1 = chan1;

	// work out lux
	if (chan0 < 65535) {
		// from datasheet
		//lux = (CH0 - CH1) x (15 / CGAIN_0) x (148 / TIG_SEL) x 0.022

		// not sure why chan1 is larger chan chan0 at low light, documentation
		// isn't really clear how exact timing has to be, but seems to be okay
		calc = chan0 - chan1;
		if (calc < 0)
			calc = 0;

		// multiple ADC by gain/TIG/multipler up scaling
		calc *= 15 * 148 * 22;

		// scale up depending on our application side scale factor
		// i.e. our mechanicals might block some light so we need to scale
		// up before we scale down the chip fixed amount of 0.022
		calc *= chip->scale;

		// and downscale gain, TIG and multipler
		calc /= chip->cgain0;
		calc /= chip->int_time_raw;
		calc /= 1000;
	} else {
		// from datasheet
		//lux = (CH0) x (15 / CGAIN_0) x (148/ TIG_SEL) x 0.022;

		// so as not to overflow transfer chan0 to calc
		calc = chan0;
		// multiple ADC by gain/TIG/multipler up scaling
		calc *= 15 * 148 * 22;

		// scale up depending on our application side scale factor
		// i.e. our mechanicals might block some light so we need to scale
		// up before we scale down the chip fixed amount of 0.022
		calc *= chip->scale;

		// and downscale gain, TIG and multipler
		calc /= chip->cgain0;
		calc /= chip->int_time_raw;
		calc /= 1000;
	}

	dev_dbg(&chip->client->dev, "ALS val: %d, chan0: %d, chan1: %d\n",
		(int)calc, chan0, chan1);
	chip->als_cur_info.lux = (int)calc;
	ret = chip->als_cur_info.lux;
done:
	return ret;
}

static int cg5162tc_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan, int *val,
			     int *val2, long mask)
{
	struct cg5162tc *chip = iio_priv(indio_dev);
	int ret;

	mutex_lock(&chip->als_mutex);

	switch (mask) {
	case IIO_CHAN_INFO_PROCESSED:
		*val = cg5162tc_get_lux(indio_dev);
		if (*val < 0) {
			ret = -EINVAL;
		} else {
			ret = IIO_VAL_INT;
		}
		break;
	case IIO_CHAN_INFO_RAW:
		// just read the previously capture values
		if (chan->channel == 0) {
			*val = chip->als_cur_info.als_ch0;
		} else {
			*val = chip->als_cur_info.als_ch1;
		}
		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_SCALE:
		*val = chip->scale;
		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_HARDWAREGAIN:
		// TODO read gain
		*val = chip->cgain0;
		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_INT_TIME:
		*val = chip->int_time;
		*val2 = chip->int_time_frac;
		ret = IIO_VAL_INT_PLUS_MICRO;
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&chip->als_mutex);
	return ret;
}

static int cg5162tc_write_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan, int val,
			      int val2, long mask)
{
	struct cg5162tc *chip = iio_priv(indio_dev);
	int ret;

	mutex_lock(&chip->als_mutex);

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		if (val >= 1 && val <= 255) {
			chip->scale = val;
			ret = 0;
		} else {
			ret = -EINVAL;
		}
		break;
	case IIO_CHAN_INFO_HARDWAREGAIN:
		ret = cg5162tc_set_gain(chip, val);
		break;
	case IIO_CHAN_INFO_INT_TIME:
		ret = cg5162tc_set_tig(chip, val, val2);
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&chip->als_mutex);
	return ret;
}

static const struct iio_info cg5162tc_info = {
	.read_raw = cg5162tc_read_raw,
	.write_raw = cg5162tc_write_raw,
	.attrs = &cg5162tc_attribute_group,
};

static int cg5162tc_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct iio_dev *iio;
	struct cg5162tc *chip;
	int ret;

	iio = devm_iio_device_alloc(&client->dev, sizeof(*chip));
	if (!iio)
		return -ENOMEM;

	chip = iio_priv(iio);
	chip->client = client;
	chip->dev = dev;

	mutex_init(&chip->als_mutex);
	i2c_set_clientdata(client, iio);

	dev_set_name(&iio->dev, "iio:als");

	// device is enabled by default

	iio->name = client->name;
	iio->channels = cg5162tc_channels;
	iio->num_channels = ARRAY_SIZE(cg5162tc_channels);
	iio->dev.parent = dev;
	iio->modes = INDIO_DIRECT_MODE;
	iio->info = &cg5162tc_info;

	// set some defaults
	chip->cgain0 = 0xf;
	chip->int_time_raw = 0x94;
	chip->int_time = 400;
	chip->int_time_frac = 0;
	chip->scale = 1;

	// disable/enable to reset registers
	ret = cg5162tc_enable(chip, 0);

	if (ret < 0) {
		return ret;
	}

	ret = cg5162tc_enable(chip, 1);

	if (ret < 0) {
		return ret;
	}

	return devm_iio_device_register(dev, iio);
}

static const struct i2c_device_id cg5162tc_idtable[] = { { "cg5162tc", 0 },
							 {} };

MODULE_DEVICE_TABLE(i2c, cg5162tc_idtable);

static const struct of_device_id cg5162tc_of_match[] = {
	{ .compatible = "chipgoal,cg5162tc" },
	{}
};
MODULE_DEVICE_TABLE(of, cg5162tc_of_match);

static struct i2c_driver cg5162tc_driver = {
	.driver = {
		.name = "cg5162tc",
		.of_match_table = cg5162tc_of_match,
	},
	.id_table = cg5162tc_idtable,
	.probe = cg5162tc_probe,
};

module_i2c_driver(cg5162tc_driver);

MODULE_AUTHOR("Will Bodley <will@teknique.com>");
MODULE_DESCRIPTION("Chipgoal cg5162tc ambient light sensor driver");
MODULE_LICENSE("GPL");
