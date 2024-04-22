#include <linux/module.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/version.h>

#include "amba_eis.h"
#include "bno080.h"

extern int get_hwtimer_output_ticks(u64 *out_tick);

#define BNO080_IRQ_NAME "IMU Data ready IRQ"
#define BNO080_DEV_NAME_DEFAULT "iio:imu"

#define BNO080_OUT_CHANNEL(_type, _axis)                                       \
	{                                                                      \
		.type = _type, .modified = 1, .channel2 = IIO_MOD_##_axis,     \
		.output = 1, .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),     \
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),      \
	}

enum bno080_accuracy {
	UNRELIABLE = 0,
	LOW,
	MEDIUM,
	HIGH,
};

enum bno080_calib_status {
	OFF = 0,
	ON,
	FAILED,
};

struct bno080_eis_desc {
	void *arg;
	GYRO_EIS_CALLBACK callback;
	int sample_id;
	int rate_in_hz;
};

static struct bno080_eis_desc *eis_desc;

static const char *bno080_calib_state_to_str(enum bno080_calib_status status)
{
	switch (status) {
	case OFF:
		return "Off";
	case ON:
		return "On";
	case FAILED:
		return "Failed";
	default:
		return "Invalid";
	};
}

static const char *bno080_accuracy_to_str(enum bno080_accuracy accuracy)
{
	switch (accuracy) {
	case UNRELIABLE:
		return "Unreliable";
	case LOW:
		return "Low";
	case MEDIUM:
		return "Medium";
	case HIGH:
		return "High";
	default:
		return "Invalid";
	};
}

void gyro_register_eis_callback(GYRO_EIS_CALLBACK cb, void *arg)
{
	eis_desc->callback = cb;
	eis_desc->arg = arg;
}
EXPORT_SYMBOL(gyro_register_eis_callback);

void gyro_unregister_eis_callback(void)
{
	eis_desc->callback = NULL;
	eis_desc->arg = NULL;
}
EXPORT_SYMBOL(gyro_unregister_eis_callback);

void gyro_dev_get_info(gyro_dev_info_t *gyro_dev_info)
{
	gyro_dev_info->accel_full_scale_range =
		16; // +-8g from BNO080 datasheet also can be read from Metadata
	gyro_dev_info->accel_lsb = 16384; // TODO TBD
	gyro_dev_info->gyro_full_scale_range =
		2000; // +- 2000 deg/s from BNO080 datasheet also can be read from Metadata
	gyro_dev_info->gyro_lsb = 131; // TODO TBD
	gyro_dev_info->gyro_sample_rate_in_hz = eis_desc->rate_in_hz;
}
EXPORT_SYMBOL(gyro_dev_get_info);

static ssize_t bno080_calib_status_show(struct iio_dev *indio_dev,
					uintptr_t private,
					const struct iio_chan_spec *chan,
					char *buf)
{
	struct bno080_data *data = iio_priv(indio_dev);

	switch (chan->type) {
	case IIO_ACCEL:
		return snprintf(
			buf, PAGE_SIZE, "%s\n",
			bno080_calib_state_to_str(data->acc_calib_status));
	case IIO_ANGL_VEL:
		return snprintf(
			buf, PAGE_SIZE, "%s\n",
			bno080_calib_state_to_str(data->gyr_calib_status));
	case IIO_MAGN:
		return snprintf(
			buf, PAGE_SIZE, "%s\n",
			bno080_calib_state_to_str(data->mag_calib_status));
	case IIO_ROT:
		return snprintf(
			buf, PAGE_SIZE, "%s\n",
			bno080_calib_state_to_str(data->rot_calib_status));
	default:
		break;
	}

	return -EINVAL;
}

static ssize_t bno080_calib_save_and_stop(struct iio_dev *indio_dev,
					  uintptr_t private,
					  const struct iio_chan_spec *chan,
					  const char *buf, size_t len)
{
	unsigned long val;
	int ret;
	struct bno080_data *data = iio_priv(indio_dev);
	struct bno080_api_desc *desc = data->api_desc;

	if (data->acc_calib_status != ON && data->gyr_calib_status != ON &&
	    data->mag_calib_status != ON && data->rot_calib_status != ON)
		return -EINVAL;

	if (len <= 0 || buf == NULL)
		return -EINVAL;

	ret = kstrtoul(buf, 10, &val);
	if (ret != 0)
		return ret;

	if (val == 0)
		return -EINVAL;

	bno080_api_saveCalibration(desc);
	bno080_api_requestCalibrationStatus(desc);

	schedule_delayed_work(&data->post_calib_work, msecs_to_jiffies(200));
	return len;
}

static void bno080_post_calib_work(struct work_struct *work)
{
	struct bno080_data *data =
		container_of(work, struct bno080_data, post_calib_work.work);
	struct bno080_api_desc *desc = data->api_desc;
	bool calib_failed = bno080_api_calibrationFailed(desc);

	if (data->acc_calib_status == ON) {
		if (calib_failed)
			data->acc_calib_status = FAILED;
		else
			data->acc_calib_status = OFF;
	}
	if (data->gyr_calib_status == ON) {
		if (calib_failed)
			data->gyr_calib_status = FAILED;
		else
			data->gyr_calib_status = OFF;
	}
	if (data->mag_calib_status == ON) {
		if (calib_failed)
			data->mag_calib_status = FAILED;
		else
			data->mag_calib_status = OFF;
	}
	if (data->rot_calib_status == ON) {
		if (calib_failed)
			data->rot_calib_status = FAILED;
		else
			data->rot_calib_status = OFF;
	}
}

static ssize_t bno080_calib_start(struct iio_dev *indio_dev, uintptr_t private,
				  const struct iio_chan_spec *chan,
				  const char *buf, size_t len)
{
	unsigned long val;
	int ret;
	struct bno080_data *data = iio_priv(indio_dev);
	struct bno080_api_desc *desc = data->api_desc;

	if (len <= 0 || buf == NULL)
		return -EINVAL;

	ret = kstrtoul(buf, 10, &val);
	if (ret != 0)
		return ret;

	if (val == 0)
		return -EINVAL;

	switch (chan->type) {
	case IIO_ACCEL:
		bno080_api_calibrateAccelerometer(desc);
		data->acc_calib_status = true;
		break;
	case IIO_ANGL_VEL:
		bno080_api_calibrateGyro(desc);
		data->gyr_calib_status = true;
		break;
	case IIO_MAGN:
		bno080_api_calibrateMagnetometer(desc);
		data->mag_calib_status = true;
		break;
	case IIO_ROT:
		bno080_api_calibrateAll(desc);
		data->rot_calib_status = true;
		break;
	default:
		break;
	}

	return len;
}

static ssize_t bno080_accuracy_show(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan, char *buf)
{
	struct bno080_data *data = iio_priv(indio_dev);
	struct bno080_api_sample_data *samples = data->api_desc->sample_data;

	switch (chan->type) {
	case IIO_ROT:
		return snprintf(buf, PAGE_SIZE, "%s\n",
				bno080_accuracy_to_str(samples->quatAccuracy));
	case IIO_ACCEL:
		return snprintf(buf, PAGE_SIZE, "%s\n",
				bno080_accuracy_to_str(samples->accelAccuracy));
	case IIO_ANGL_VEL:
		return snprintf(buf, PAGE_SIZE, "%s\n",
				bno080_accuracy_to_str(samples->gyroAccuracy));
	case IIO_MAGN:
		return snprintf(buf, PAGE_SIZE, "%s\n",
				bno080_accuracy_to_str(samples->magAccuracy));
	default:
		break;
	};

	return -EINVAL;
}

static ssize_t bno080_rad_accuracy_show(struct iio_dev *indio_dev,
					uintptr_t private,
					const struct iio_chan_spec *chan,
					char *buf)
{
	struct bno080_data *data = iio_priv(indio_dev);
	struct bno080_api_sample_data *samples = data->api_desc->sample_data;

	return snprintf(buf, PAGE_SIZE, "%d\n", samples->rawQuatRadianAccuracy);
}

static const struct iio_chan_spec_ext_info bno080_calib_ext_info[] = {
	{
		.name = "calib_status",
		.shared = IIO_SEPARATE,
		.read = bno080_calib_status_show,
	},
	{
		.name = "calibrate",
		.shared = IIO_SEPARATE,
		.write = bno080_calib_start,
	},
	{
		.name = "calib_save_and_stop",
		.shared = IIO_SEPARATE,
		.write = bno080_calib_save_and_stop,
	},
	{}
};

static const struct iio_chan_spec_ext_info bno080_accuracy_ext_info[] = {
	{
		.name = "accuracy",
		.shared = IIO_SEPARATE,
		.read = bno080_accuracy_show,
	},
	{}
};

static const struct iio_chan_spec_ext_info bno080_rad_accuracy_ext_info[] = {
	{
		.name = "rad_accuracy",
		.shared = IIO_SEPARATE,
		.read = bno080_rad_accuracy_show,
	},
	{}
};

static const struct iio_chan_spec bno080_accel_channels[] = {
	BNO080_OUT_CHANNEL(IIO_ACCEL, X),
	BNO080_OUT_CHANNEL(IIO_ACCEL, Y),
	BNO080_OUT_CHANNEL(IIO_ACCEL, Z),
	{
		.type = IIO_ACCEL,
		.ext_info = bno080_calib_ext_info,
	},
	{
		.type = IIO_ACCEL,
		.ext_info = bno080_accuracy_ext_info,
		.output = 1,
	},
};

static const struct iio_chan_spec bno080_gyro_channels[] = {
	BNO080_OUT_CHANNEL(IIO_ANGL_VEL, X),
	BNO080_OUT_CHANNEL(IIO_ANGL_VEL, Y),
	BNO080_OUT_CHANNEL(IIO_ANGL_VEL, Z),
	{
		.type = IIO_ANGL_VEL,
		.ext_info = bno080_calib_ext_info,
	},
	{
		.type = IIO_ANGL_VEL,
		.ext_info = bno080_accuracy_ext_info,
		.output = 1,
	},
};

static const struct iio_chan_spec bno080_mag_channels[] = {
	BNO080_OUT_CHANNEL(IIO_MAGN, X),
	BNO080_OUT_CHANNEL(IIO_MAGN, Y),
	BNO080_OUT_CHANNEL(IIO_MAGN, Z),
	{
		.type = IIO_MAGN,
		.ext_info = bno080_calib_ext_info,
	},
	{
		.type = IIO_MAGN,
		.ext_info = bno080_accuracy_ext_info,
		.output = 1,
	},
};

static const struct iio_chan_spec bno080_rot_channels[] = {
	{
		.type = IIO_ROT,
		.modified = 1,
		.channel2 = IIO_MOD_QUATERNION,
		.output = 1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
	},
	{
		.type = IIO_ROT,
		.ext_info = bno080_calib_ext_info,
	},
	{
		.type = IIO_ROT,
		.ext_info = bno080_accuracy_ext_info,
		.output = 1,
	},
	{
		.type = IIO_ROT,
		.ext_info = bno080_rad_accuracy_ext_info,
		.output = 1,
	},
};

static struct iio_chan_spec *bno080_channels = NULL;

static int bno080_get_axis_data(struct bno080_api_sample_data *samples,
				int chan_type, int axis, int *vals)
{
	if (chan_type == IIO_ACCEL) {
		if (axis == IIO_MOD_X)
			vals[0] = samples->rawAccelX;
		else if (axis == IIO_MOD_Y)
			vals[0] = samples->rawAccelY;
		else if (axis == IIO_MOD_Z)
			vals[0] = samples->rawAccelZ;
		else
			return -EINVAL;
	} else if (chan_type == IIO_ANGL_VEL) {
		if (axis == IIO_MOD_X)
			vals[0] = samples->rawGyroX;
		else if (axis == IIO_MOD_Y)
			vals[0] = samples->rawGyroY;
		else if (axis == IIO_MOD_Z)
			vals[0] = samples->rawGyroZ;
		else
			return -EINVAL;
	} else if (chan_type == IIO_MAGN) {
		if (axis == IIO_MOD_X)
			vals[0] = samples->rawMagX;
		else if (axis == IIO_MOD_Y)
			vals[0] = samples->rawMagY;
		else if (axis == IIO_MOD_Z)
			vals[0] = samples->rawMagZ;
		else
			return -EINVAL;
	} else {
		return -EINVAL;
	}

	return 0;
}

static int bno080_get_rot_quat(struct bno080_api_sample_data *samples,
			       int *vals)
{
	vals[0] = samples->rawQuatI;
	vals[1] = samples->rawQuatJ;
	vals[2] = samples->rawQuatK;
	vals[3] = samples->rawQuatReal;
	return 4;
}

static int bno080_get_freq(struct bno080_data *data, int chan_type, int *vals)
{
	switch (chan_type) {
	case IIO_ACCEL:
		vals[0] = data->acc_freq;
		break;
	case IIO_ANGL_VEL:
		vals[0] = data->gyr_freq;
		break;
	case IIO_MAGN:
		vals[0] = data->mag_freq;
		break;
	case IIO_ROT:
		vals[0] = data->rot_vect_freq;
		break;
	default:
		break;
	}

	return 0;
}

static int bno080_read_raw_multi(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan, int size,
				 int *vals, int *val_len, long mask)
{
	struct bno080_data *data = iio_priv(indio_dev);
	struct bno080_api_sample_data *samples = data->api_desc->sample_data;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (chan->type == IIO_ROT) {
			if (size < 4)
				return -EINVAL;

			*val_len = bno080_get_rot_quat(samples, vals);
			return IIO_VAL_INT_MULTIPLE;
		} else {
			bno080_get_axis_data(samples, chan->type,
					     chan->channel2, vals);
			return IIO_VAL_INT;
		}
	case IIO_CHAN_INFO_SAMP_FREQ:
		bno080_get_freq(data, chan->type, vals);
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}

	return 0;
}

//These Q values are defined in the datasheet but can also be obtained by querying the meta data records
static IIO_CONST_ATTR(out_accel_q1, "8");
static IIO_CONST_ATTR(out_anglvel_q1, "10");
static IIO_CONST_ATTR(out_magn_q1, "4");
static IIO_CONST_ATTR(out_rot_q1, "14");
static IIO_CONST_ATTR(out_rot_rad_accuracy_q1, "12");

static struct attribute *bno080_attrs[6];

static const struct attribute_group bno080_attrs_group = {
	.attrs = bno080_attrs,
};

static const struct iio_info bno080_info = {
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0)
	.driver_module = THIS_MODULE,
#endif
	.read_raw_multi = bno080_read_raw_multi,
	.attrs = &bno080_attrs_group,
};

static int bno080_chip_init(struct bno080_data *data)
{
	if (!bno080_api_begin(data->api_desc)) {
		return -EINVAL;
	}

	if (data->rot_vect_freq > 0)
		bno080_api_enableRotationVector(data->api_desc,
						data->rot_vect_freq);
	if (data->acc_freq > 0)
		bno080_api_enableAccelerometer(data->api_desc, data->acc_freq);
	if (data->gyr_freq > 0)
		bno080_api_enableGyro(data->api_desc, data->gyr_freq);
	if (data->mag_freq > 0)
		bno080_api_enableMagnetometer(data->api_desc, data->mag_freq);

	return 0;
}

static irqreturn_t bno080_irq(int irq, void *private)
{
	struct bno080_data *data = private;
	struct bno080_api_sample_data *samples = data->api_desc->sample_data;
	gyro_data_t gyro_data;

	bno080_api_getReadings(data->api_desc);

	gyro_data.sample_id = (eis_desc->sample_id)++;
	gyro_data.xg = (s16)samples->memsRawGyroX;
	gyro_data.yg = (s16)samples->memsRawGyroY;
	gyro_data.zg = (s16)samples->memsRawGyroZ;
	gyro_data.xa = (s16)samples->memsRawAccelX;
	gyro_data.ya = (s16)samples->memsRawAccelY;
	gyro_data.za = (s16)samples->memsRawAccelZ;
	gyro_data.xm = (s16)samples->memsRawMagX;
	gyro_data.ym = (s16)samples->memsRawMagY;
	gyro_data.zm = (s16)samples->memsRawMagZ;
	get_hwtimer_output_ticks(&gyro_data.pts);

	if (eis_desc->callback) {
		eis_desc->callback(&gyro_data, eis_desc->arg);
	}

	return IRQ_HANDLED;
}

static void bno080_post_init_work(struct work_struct *work)
{
	struct bno080_data *data;
	data = container_of(work, struct bno080_data, post_init_work.work);

	if (!gpio_get_value(data->drdy_gpio))
		bno080_api_getReadings(data->api_desc);
}

static int bno080_parse_dt(struct bno080_data *data)
{
	struct device_node *np = data->api_desc->dev->of_node;
	enum of_gpio_flags flags;

	data->drdy_gpio =
		of_get_named_gpio_flags(np, "ocl,drdy-gpio", 0, &flags);
	if (!gpio_is_valid(data->drdy_gpio))
		return -EINVAL;

	if (of_property_read_u32(np, "ocl,rot-vect-freq",
				 &data->rot_vect_freq) < 0)
		data->rot_vect_freq = 0;

	if (of_property_read_u32(np, "ocl,acc-freq", &data->acc_freq) < 0)
		data->acc_freq = 0;

	if (of_property_read_u32(np, "ocl,gyr-freq", &data->gyr_freq) < 0)
		data->gyr_freq = 0;
	else
		eis_desc->rate_in_hz = (1000 / data->gyr_freq);

	if (of_property_read_u32(np, "ocl,mag-freq", &data->mag_freq) < 0)
		data->mag_freq = 0;

	if (of_property_read_string(np, "ocl,dev-name", &data->dev_name) != 0)
		data->dev_name = BNO080_DEV_NAME_DEFAULT;

	return 0;
}

static int bno080_get_channels_num(struct bno080_data *data)
{
	int chan_num = 0;

	if (data->rot_vect_freq > 0)
		chan_num += ARRAY_SIZE(bno080_rot_channels);
	if (data->acc_freq > 0)
		chan_num += ARRAY_SIZE(bno080_accel_channels);
	if (data->gyr_freq)
		chan_num += ARRAY_SIZE(bno080_gyro_channels);
	if (data->mag_freq)
		chan_num += ARRAY_SIZE(bno080_mag_channels);

	return chan_num;
}

static int bno080_channels_init(struct bno080_data *data)
{
	struct iio_chan_spec *tail = bno080_channels;

	if (data->acc_freq > 0) {
		memcpy(tail, bno080_accel_channels,
		       sizeof(bno080_accel_channels));
		tail += ARRAY_SIZE(bno080_accel_channels);
	}

	if (data->gyr_freq > 0) {
		memcpy(tail, bno080_gyro_channels,
		       sizeof(bno080_gyro_channels));
		tail += ARRAY_SIZE(bno080_gyro_channels);
	}

	if (data->mag_freq > 0) {
		memcpy(tail, bno080_mag_channels, sizeof(bno080_mag_channels));
		tail += ARRAY_SIZE(bno080_mag_channels);
	}

	if (data->rot_vect_freq > 0) {
		memcpy(tail, bno080_rot_channels, sizeof(bno080_rot_channels));
		tail += ARRAY_SIZE(bno080_rot_channels);
	}

	return 0;
}

static int bno080_attr_init(struct bno080_data *data)
{
	int index = 0;

	if (data->acc_freq > 0) {
		bno080_attrs[index] =
			&iio_const_attr_out_accel_q1.dev_attr.attr;
		index++;
	}

	if (data->gyr_freq > 0) {
		bno080_attrs[index] =
			&iio_const_attr_out_anglvel_q1.dev_attr.attr;
		index++;
	}

	if (data->mag_freq) {
		bno080_attrs[index] = &iio_const_attr_out_magn_q1.dev_attr.attr;
		index++;
	}

	if (data->rot_vect_freq) {
		bno080_attrs[index] = &iio_const_attr_out_rot_q1.dev_attr.attr;
		index++;
		bno080_attrs[index] =
			&iio_const_attr_out_rot_rad_accuracy_q1.dev_attr.attr;
		index++;
	}

	bno080_attrs[index] = NULL;
	return 0;
}

int bno080_core_probe(struct device *dev, struct bno080_api_hal_ops *hal_ops,
		      const char *name)
{
	struct iio_dev *indio_dev;
	struct bno080_data *data;
	int ret;
	int bno080_chan_num;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	dev_set_drvdata(dev, indio_dev);

	eis_desc = kzalloc(sizeof(struct bno080_eis_desc), GFP_KERNEL);
	if (!eis_desc)
		return -ENOMEM;

	data->api_desc = bno080_api_init(dev, hal_ops);
	if (data->api_desc == NULL) {
		ret = -ENOMEM;
		goto free_eis;
	}

	ret = bno080_parse_dt(data);
	if (ret != 0)
		return ret;

	dev_set_name(&indio_dev->dev, data->dev_name);

	bno080_chan_num = bno080_get_channels_num(data);
	if (bno080_chan_num <= 0) {
		dev_err(dev,
			"Invalid channels number %d! Must be more then zero\n",
			bno080_chan_num);
		ret = -EINVAL;
		goto free_api;
	}

	bno080_channels = kzalloc(
		sizeof(struct iio_chan_spec) * bno080_chan_num, GFP_KERNEL);
	if (!bno080_channels) {
		ret = -ENOMEM;
		goto free_api;
	}

	bno080_channels_init(data);
	bno080_attr_init(data);

	ret = bno080_chip_init(data);
	if (ret < 0) {
		dev_err(dev, "Failed to init BNO080 %d\n", ret);
		goto free_channels;
	}

	INIT_DELAYED_WORK(&data->post_init_work, bno080_post_init_work);
	INIT_DELAYED_WORK(&data->post_calib_work, bno080_post_calib_work);

	indio_dev->dev.parent = dev;
	indio_dev->channels = bno080_channels;
	indio_dev->num_channels = bno080_chan_num;
	indio_dev->name = name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &bno080_info;

	ret = iio_device_register(indio_dev);
	if (ret < 0) {
		dev_err(dev, "Failed to register iio device %d!\n", ret);
		ret = -ENXIO;
		goto free_works;
	}

	ret = gpio_to_irq(data->drdy_gpio);
	if (ret < 0) {
		dev_err(dev, "Failed convert gpio to irq %d!\n", ret);
		goto free_device;
	}

	data->drdy_irq = ret;

	ret = devm_request_threaded_irq(dev, data->drdy_irq, NULL, bno080_irq,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					BNO080_IRQ_NAME, data);

	if (ret < 0) {
		dev_err(dev, "Failed to request irq %d\n", data->drdy_irq);
		goto free_irq;
	}

	dev_info(dev, "gpio resource, no:%d irq:%d\n", data->drdy_gpio,
		 data->drdy_irq);

	schedule_delayed_work(&data->post_init_work, msecs_to_jiffies(10));
	return 0;

free_irq:
	devm_free_irq(dev, data->drdy_irq, data);
free_device:
	iio_device_unregister(indio_dev);
free_works:
	cancel_delayed_work(&data->post_init_work);
	cancel_delayed_work(&data->post_calib_work);
	flush_scheduled_work();
free_channels:
	kfree(bno080_channels);
free_api:
	bno080_api_release(data->api_desc);
free_eis:
	kfree(eis_desc);

	return ret;
}
EXPORT_SYMBOL_GPL(bno080_core_probe);

void bno080_core_remove(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct bno080_data *data = iio_priv(indio_dev);

	cancel_delayed_work(&data->post_init_work);
	cancel_delayed_work(&data->post_calib_work);
	flush_scheduled_work();

	if (data->drdy_irq > 0)
		devm_free_irq(dev, data->drdy_irq, data);

	if (data->api_desc)
		bno080_api_release(data->api_desc);

	iio_device_unregister(indio_dev);

	if (bno080_channels)
		kfree(bno080_channels);

	if (eis_desc)
		kfree(eis_desc);
}
EXPORT_SYMBOL_GPL(bno080_core_remove);

MODULE_AUTHOR("Gym Ok Cho <kimogi@teknique.com");
MODULE_DESCRIPTION("Hillcrest Labs BNO080 driver");
MODULE_LICENSE("GPL v2");
