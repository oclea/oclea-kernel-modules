// SPDX-License-Identifier: GPL-2.0
/*
 * Terasillc senseboost radar sensor driver
 *
 */

#include <asm/unaligned.h>
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/jiffies.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/serdev.h>

#define TS_SENSEBOOST_DRIVER_NAME                 "ts-senseboost-ph12"
#define TS_SENSEBOOST_FRAME_START                 (0x55AA)
#define TS_SENSEBOOST_FRAME_CMD_RGF               (0x31)
#define TS_SENSEBOOST_FRAME_CMD_SYS               (0x00)
#define TS_SENSEBOOST_FRAME_START_SIZE            (2)
#define TS_SENSEBOOST_FRAME_CMD_SIZE              (2)
#define TS_SENSEBOOST_FRAME_LEN_SIZE              (1)
#define TS_SENSEBOOST_FRAME_DATA_SIZE             (8)
#define TS_SENSEBOOST_FRAME_CHECKSUM_SIZE         (1)
#define TS_SENSEBOOST_FRAME_BODY_SIZE             (                                  \
                                                   TS_SENSEBOOST_FRAME_CMD_SIZE +    \
                                                   TS_SENSEBOOST_FRAME_LEN_SIZE +    \
                                                   TS_SENSEBOOST_FRAME_DATA_SIZE +   \
                                                   TS_SENSEBOOST_FRAME_CHECKSUM_SIZE \
										          )
#define TS_SENSEBOOST_FRAME_SIZE                  (14)
#define TS_SENSEBOOST_SENSING_TARGET_ID_OFFSET    (3)
#define TS_SENSEBOOST_SENSING_ANGLE_OFFSET        (3)
#define TS_SENSEBOOST_SENSING_AMPLITUDE_OFFSET    (5)
#define TS_SENSEBOOST_SENSING_DISTANCE_OFFSET     (7)
#define TS_SENSEBOOST_SENSING_VELOCITY_OFFSET     (9)
#define TS_SENSEBOOST_TIMEOUT                     msecs_to_jiffies(2000)
#define TS_SENSEBOOST_ATTRIBUTES_SIZE_MAX         (48)

enum ts_senseboost_scan {
	SCAN_ID,
	SCAN_ANGLE,
	SCAN_AMPLITUDE,
	SCAN_DISTANCE,
	SCAN_VELOCITY
};

enum ts_senseboost_query {
	QUERY_NOP = -1,
	QUERY_SYS_STATUS,
	QUERY_SYS_VERSION,
	QUERY_RGF_MODE_GAIN,
	QUERY_RGF_MODE_CFAR,
	QUERY_RGF_MOTION,
	QUERY_RGF_TRACK_TIME,
	QUERY_RGF_MODE_DUTY,
	QUERY_RGF_MODE_STATUS,
	QUERY_RGF_SETTING_ENABLE,
	QUERY_RGF_ROI_RANGE,
	QUERY_RGF_ROI_ZONE_1,
	QUERY_RGF_ROI_ZONE_2,
	QUERY_RGF_ANGLE,
	QUERY_RGF_GPIO,
	QUERY_NUM
};

enum ts_senseboost_set {
	SET_NOP = -1,
	SET_SYS_STATUS,
	SET_SYS_CONFIG_RESET,
	SET_SYS_CONFIG_REBOOT,
	SET_SYS_CONFIG_DEFAULT,
	SET_RGF_MODE_GAIN,
	SET_RGF_MODE_CFAR,
	SET_RGF_MOTION,
	SET_RGF_TRACK_TIME,
	SET_RGF_MODE_DUTY,
	SET_RGF_MODE_STATUS,
	SET_RGF_SETTING_ENABLE,
	SET_RGF_ROI_RANGE,
	SET_RGF_ROI_ZONE_1,
	SET_RGF_ROI_ZONE_2,
	SET_RGF_ANGLE,
	SET_RGF_GPIO,
	SET_NUM
};

enum ts_senseboost_response {
	RESPONSE_NOP = -1,
	RESPONSE_SYS_STATUS,
	RESPONSE_SYS_VERSION,
	RESPONSE_RGF_MODE_GAIN,
	RESPONSE_RGF_SENSING_DATA,
	RESPONSE_RGF_MODE_CFAR,
	RESPONSE_RGF_MOTION,
	RESPONSE_RGF_TRACK_TIME,
	RESPONSE_RGF_MODE_DUTY,
	RESPONSE_RGF_MODE_STATUS,
	RESPONSE_RGF_SETTING_ENABLE,
	RESPONSE_RGF_ROI_RANGE,
	RESPONSE_RGF_ROI_ZONE_1,
	RESPONSE_RGF_ROI_ZONE_2,
	RESPONSE_RGF_ANGLE,
	RESPONSE_RGF_GPIO,
	RESPONSE_NUM
};

enum ts_senseboost_attribute_type {
	TS_SENSEBOOST_ATTR_TYPE_SYS_STATUS = 0,
	TS_SENSEBOOST_ATTR_TYPE_SYS_CONFIG,
	TS_SENSEBOOST_ATTR_TYPE_VERSION,
	TS_SENSEBOOST_ATTR_TYPE_MODE_GAIN,
	TS_SENSEBOOST_ATTR_TYPE_MODE_CFAR,
	TS_SENSEBOOST_ATTR_TYPE_MOTION,
	TS_SENSEBOOST_ATTR_TYPE_TRACK_TIME,
	TS_SENSEBOOST_ATTR_TYPE_MODE_DUTY,
	TS_SENSEBOOST_ATTR_TYPE_MODE_STATUS,
	TS_SENSEBOOST_ATTR_TYPE_SETTING_ENABLE,
	TS_SENSEBOOST_ATTR_TYPE_ROI_RANGE,
	TS_SENSEBOOST_ATTR_TYPE_ROI_ZONE_1,
	TS_SENSEBOOST_ATTR_TYPE_ROI_ZONE_2,
	TS_SENSEBOOST_ATTR_TYPE_ANGLE,
	TS_SENSEBOOST_ATTR_TYPE_GPIO,
	TS_SENSEBOOST_ATTR_TYPE_NUM
};

enum ts_senseboost_attribute_num {
	TS_SENSEBOOST_ATTR_NUM_1 = 1,
	TS_SENSEBOOST_ATTR_NUM_2,
	TS_SENSEBOOST_ATTR_NUM_3,
	TS_SENSEBOOST_ATTR_NUM_4,
	TS_SENSEBOOST_ATTR_NUM_10 = 10,
	TS_SENSEBOOST_ATTR_NUM_MAX
};

#define TS_SENSEBOOST_MSG_TEMPLATE \
                { 0x55, 0xAA, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }

#define TS_SENSEBOOST_MSG_WITH_FILLING                                                                \
                { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,                                     \
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                  0xFF, 0xFF, 0xFF, 0xFF,                                                             \
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,                                     \
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                  0xFF, 0xFF, 0xFF, 0xFF,                                                             \
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }

#define TS_SENSEBOOST_CHAN(_type, _index, _sign, _realbits, _storagebits) \
                {                                                         \
                .type = _type,                                            \
                .info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),       \
                .scan_index = _index,                                     \
                .scan_type =                                              \
                    {                                                     \
                    .sign = _sign,                                        \
                    .realbits = _realbits,                                \
                    .storagebits = _storagebits,                          \
                    .endianness = IIO_CPU,                                \
                    },                                                    \
                }

struct ts_senseboost_attribute_map {
	s8 query_idx;
	s8 set_idx;
	s8 response_idx;
	s8 num;
};

struct ts_senseboost_attribute {
	struct device_attribute dev_attr;
	enum ts_senseboost_attribute_type type;
	union {
		struct {
			u8 data_out_on:1;
			u8 plot_on:1;
			u8 plot_rx1_on:1;
			u8 plot_rx2_on:1;
		} status;

		struct {
			u8 major;
			u8 minor;
			u8 patch_high;
			u8 patch_low;
		} version;

		struct {
			u8 tx_sentry;
			u8 rx01_sentry:4;
			u8 rx02_sentry:4;
			u8 hpf01_sentry;
			u8 hpf02_sentry;
			u8 tx_2d;
			u8 rx01_2d:4;
			u8 rx02_2d:4;
			u8 hpf01_2d;
			u8 hpf02_2d;
		} mode_gain;

		struct {
			u16 threshold_sentry;
			u16 threshold_2d;
		} mode_cfar;

		struct {
			u16 range_min;
			u16 range_max;
			u8 angle_max;
		} motion;

		struct {
			u8 init;
			u8 delete;
		} track_time;

		struct {
			u8 enabled;
			u8 rate_sentry;
			u8 rate_2d;
		} mode_duty;

		struct {
			u8 mode_select;
			u8 debounce_2d;
			u8 debounce_sentry;
		} mode_status;

		struct {
			u8 enabled;
		} setting_enable;

		struct {
			u16 range_min;
			u16 range_max;
			u8 angle_max;
		} roi_range;

		struct {
			u16 ignored[4];
		} roi_zone1;

		struct {
			u16 ignored;
		} roi_zone2;

		struct {
			u8 reverse;
			s16 offset;
			s32 rate;
		} angle;

		struct {
			u8 gpio01_low:1;
			u8 gpio02_low:1;
		} gpio;
	};
};

struct ts_senseboost_priv {
	struct completion frame_ready[RESPONSE_NUM];
	u8 buf[RESPONSE_NUM][TS_SENSEBOOST_FRAME_BODY_SIZE];
	u8 buf_idx;
	u16 expected_length;
	u16 length;
};

struct ts_senseboost_state {
	const char *dev_name;
	struct serdev_device *serdev;
	struct ts_senseboost_priv *priv;
	struct mutex lock; /* must be held whenever state gets touched */
	/* Used to construct scan to push to the IIO buffer */
	struct {
		u16 target_id;
		s16 angle;
		u16 amplitude;
		u16 distance;
		s16 velocity;
		s64 ts;
	} scan;
};

/*
 * commands have following format:
 * +------+------+------+------+------+----------------------------+--------------+
 * |     Start   | CMD  | CMD2 | LEN  |      DATA(8Bytes)          |   Checkksum  |
 * +------+------+------+------+------+----------------------------+--------------+
 * | 0x55 | 0xAA | 0x31 |  XX  | 0x08 |  XX XX XX XX XX XX XX XX   |2's Complement|
 * +------+------+------+------+------+----------------------------+--------------+
 */
static const struct ts_senseboost_attribute_map ts_senseboost_attribute_map_tbl[TS_SENSEBOOST_ATTR_TYPE_NUM] = {
	[TS_SENSEBOOST_ATTR_TYPE_SYS_STATUS]     = {QUERY_SYS_STATUS, SET_SYS_STATUS, RESPONSE_SYS_STATUS, TS_SENSEBOOST_ATTR_NUM_4},
	[TS_SENSEBOOST_ATTR_TYPE_SYS_CONFIG]     = {QUERY_NOP, SET_SYS_CONFIG_RESET, RESPONSE_NOP, TS_SENSEBOOST_ATTR_NUM_1},
	[TS_SENSEBOOST_ATTR_TYPE_VERSION]        = {QUERY_SYS_VERSION, SET_NOP, RESPONSE_SYS_VERSION, TS_SENSEBOOST_ATTR_NUM_3},
	[TS_SENSEBOOST_ATTR_TYPE_MODE_GAIN]      = {QUERY_RGF_MODE_GAIN, SET_RGF_MODE_GAIN, RESPONSE_RGF_MODE_GAIN, TS_SENSEBOOST_ATTR_NUM_10},
	[TS_SENSEBOOST_ATTR_TYPE_MODE_CFAR]      = {QUERY_RGF_MODE_CFAR, SET_RGF_MODE_CFAR, RESPONSE_RGF_MODE_CFAR, TS_SENSEBOOST_ATTR_NUM_2},
	[TS_SENSEBOOST_ATTR_TYPE_MOTION]         = {QUERY_RGF_MOTION, SET_RGF_MOTION, RESPONSE_RGF_MOTION, TS_SENSEBOOST_ATTR_NUM_3},
	[TS_SENSEBOOST_ATTR_TYPE_TRACK_TIME]     = {QUERY_RGF_TRACK_TIME, SET_RGF_TRACK_TIME, RESPONSE_RGF_TRACK_TIME, TS_SENSEBOOST_ATTR_NUM_2},
	[TS_SENSEBOOST_ATTR_TYPE_MODE_DUTY]      = {QUERY_RGF_MODE_DUTY, SET_RGF_MODE_DUTY, RESPONSE_RGF_MODE_DUTY, TS_SENSEBOOST_ATTR_NUM_3},
	[TS_SENSEBOOST_ATTR_TYPE_MODE_STATUS]    = {QUERY_RGF_MODE_STATUS, SET_RGF_MODE_STATUS, RESPONSE_RGF_MODE_STATUS, TS_SENSEBOOST_ATTR_NUM_3},
	[TS_SENSEBOOST_ATTR_TYPE_SETTING_ENABLE] = {QUERY_RGF_SETTING_ENABLE, SET_RGF_SETTING_ENABLE, RESPONSE_RGF_SETTING_ENABLE, TS_SENSEBOOST_ATTR_NUM_1},
	[TS_SENSEBOOST_ATTR_TYPE_ROI_RANGE]      = {QUERY_RGF_ROI_RANGE, SET_RGF_ROI_RANGE, RESPONSE_RGF_ROI_RANGE, TS_SENSEBOOST_ATTR_NUM_3},
	[TS_SENSEBOOST_ATTR_TYPE_ROI_ZONE_1]     = {QUERY_RGF_ROI_ZONE_1, SET_RGF_ROI_ZONE_1, RESPONSE_RGF_ROI_ZONE_1, TS_SENSEBOOST_ATTR_NUM_4},
	[TS_SENSEBOOST_ATTR_TYPE_ROI_ZONE_2]     = {QUERY_RGF_ROI_ZONE_2, SET_RGF_ROI_ZONE_2, RESPONSE_RGF_ROI_ZONE_2, TS_SENSEBOOST_ATTR_NUM_1},
	[TS_SENSEBOOST_ATTR_TYPE_ANGLE]          = {QUERY_RGF_ANGLE, SET_RGF_ANGLE, RESPONSE_RGF_ANGLE, TS_SENSEBOOST_ATTR_NUM_3},
	[TS_SENSEBOOST_ATTR_TYPE_GPIO]           = {QUERY_RGF_GPIO, SET_RGF_GPIO, RESPONSE_RGF_GPIO, TS_SENSEBOOST_ATTR_NUM_2},
};

static const u8 ts_senseboost_query_tbl[QUERY_NUM][TS_SENSEBOOST_FRAME_CMD_SIZE] = {
	[QUERY_SYS_STATUS]         = {0x00, 0x01},
	[QUERY_SYS_VERSION]        = {0x00, 0x07},
	[QUERY_RGF_MODE_GAIN]      = {0x31, 0x00},
	[QUERY_RGF_MODE_CFAR]      = {0x31, 0x16},
	[QUERY_RGF_MOTION]         = {0x31, 0x19},
	[QUERY_RGF_TRACK_TIME]     = {0x31, 0x22},
	[QUERY_RGF_MODE_DUTY]      = {0x31, 0x25},
	[QUERY_RGF_MODE_STATUS]    = {0x31, 0x28},
	[QUERY_RGF_SETTING_ENABLE] = {0x31, 0x31},
	[QUERY_RGF_ROI_RANGE]      = {0x31, 0x34},
	[QUERY_RGF_ROI_ZONE_1]     = {0x31, 0x37},
	[QUERY_RGF_ROI_ZONE_2]     = {0x31, 0x40},
	[QUERY_RGF_ANGLE]          = {0x31, 0x43},
	[QUERY_RGF_GPIO]           = {0x31, 0x46},
};

static const u8 ts_senseboost_set_tbl[SET_NUM][TS_SENSEBOOST_FRAME_CMD_SIZE] = {
	[SET_SYS_STATUS]         = {0x00, 0x02},
	[SET_SYS_CONFIG_RESET]   = {0x00, 0x04},
	[SET_SYS_CONFIG_REBOOT]  = {0x00, 0x05},
	[SET_SYS_CONFIG_DEFAULT] = {0x00, 0x06},
	[SET_RGF_MODE_GAIN]      = {0x31, 0x01},
	[SET_RGF_MODE_CFAR]      = {0x31, 0x17},
	[SET_RGF_MOTION]         = {0x31, 0x20},
	[SET_RGF_TRACK_TIME]     = {0x31, 0x23},
	[SET_RGF_MODE_DUTY]      = {0x31, 0x26},
	[SET_RGF_MODE_STATUS]    = {0x31, 0x29},
	[SET_RGF_SETTING_ENABLE] = {0x31, 0x32},
	[SET_RGF_ROI_RANGE]      = {0x31, 0x35},
	[SET_RGF_ROI_ZONE_1]     = {0x31, 0x38},
	[SET_RGF_ROI_ZONE_2]     = {0x31, 0x41},
	[SET_RGF_ANGLE]          = {0x31, 0x44},
	[SET_RGF_GPIO]           = {0x31, 0x47},
};

static const u8 ts_senseboost_response_tbl[RESPONSE_NUM][TS_SENSEBOOST_FRAME_CMD_SIZE] = {
	[RESPONSE_SYS_STATUS]         = {0x00, 0x03},
	[RESPONSE_SYS_VERSION]        = {0x00, 0x09},
	[RESPONSE_RGF_MODE_GAIN]      = {0x31, 0x02},
	[RESPONSE_RGF_SENSING_DATA]   = {0x31, 0x15},
	[RESPONSE_RGF_MODE_CFAR]      = {0x31, 0x18},
	[RESPONSE_RGF_MOTION]         = {0x31, 0x21},
	[RESPONSE_RGF_TRACK_TIME]     = {0x31, 0x24},
	[RESPONSE_RGF_MODE_DUTY]      = {0x31, 0x27},
	[RESPONSE_RGF_MODE_STATUS]    = {0x31, 0x30},
	[RESPONSE_RGF_SETTING_ENABLE] = {0x31, 0x33},
	[RESPONSE_RGF_ROI_RANGE]      = {0x31, 0x36},
	[RESPONSE_RGF_ROI_ZONE_1]     = {0x31, 0x39},
	[RESPONSE_RGF_ROI_ZONE_2]     = {0x31, 0x42},
	[RESPONSE_RGF_ANGLE]          = {0x31, 0x45},
	[RESPONSE_RGF_GPIO]           = {0x31, 0x48},
};

static const unsigned long ts_senseboost_scan_masks[] = { BIT(SCAN_ID) | BIT(SCAN_ANGLE) | BIT(SCAN_AMPLITUDE) | BIT(SCAN_DISTANCE) | BIT(SCAN_VELOCITY), 0x00 };

static u8 ts_senseboost_calc_checksum(u8 *data, u16 length)
{
	u8 checksum = 0;
	int i = TS_SENSEBOOST_FRAME_CMD_SIZE + TS_SENSEBOOST_FRAME_LEN_SIZE;

	while (i < length - TS_SENSEBOOST_FRAME_CHECKSUM_SIZE) {
		checksum += data[i++];
	}

	return (~checksum + 1);
}

static int ts_senseboost_xfer_cmd(struct ts_senseboost_state *state, u8 msg[], int response_idx)
{
	u8 tx[] = TS_SENSEBOOST_MSG_WITH_FILLING;
	u8 start_pos1 = 8, start_pos2 = 26, start_pos3 = 48, start_pos4 = 66;
	int ret;

	memcpy(&tx[start_pos1], msg, TS_SENSEBOOST_FRAME_SIZE);
	memcpy(&tx[start_pos2], msg, TS_SENSEBOOST_FRAME_SIZE);
	memcpy(&tx[start_pos3], msg, TS_SENSEBOOST_FRAME_SIZE);
	memcpy(&tx[start_pos4], msg, TS_SENSEBOOST_FRAME_SIZE);

	if (response_idx != RESPONSE_NOP) {
		init_completion(&state->priv->frame_ready[response_idx]);
	}

	ret = serdev_device_write(state->serdev, tx, sizeof(tx), TS_SENSEBOOST_TIMEOUT);
	if (ret < sizeof(tx))
		return ret < 0 ? ret : -EIO;

	if (response_idx != RESPONSE_NOP) {
		ret = wait_for_completion_interruptible_timeout(&state->priv->frame_ready[response_idx], TS_SENSEBOOST_TIMEOUT);
		if (!ret)
			ret = -ETIMEDOUT;
	}

	return ret < 0 ? ret : 0;
}

static int ts_senseboost_get_data(const u8 *data, enum iio_chan_type type)
{
	int val = 0;
	int offset;

	switch (type) {
	case IIO_INDEX:
		offset = TS_SENSEBOOST_SENSING_TARGET_ID_OFFSET;
		val = data[offset] & 0x0F;
		break;
	case IIO_ANGL:
		offset = TS_SENSEBOOST_SENSING_ANGLE_OFFSET;
		val |= data[offset + 1];
		val = (data[offset] & 0x80) ? -val : val;
		break; 
	case IIO_ALTVOLTAGE:
		offset = TS_SENSEBOOST_SENSING_AMPLITUDE_OFFSET;
		val |= data[offset + 1];
		val = (val << 8) | data[offset];
		break;
	case IIO_DISTANCE:
		offset = TS_SENSEBOOST_SENSING_DISTANCE_OFFSET;
		val |= data[offset + 1];
		val = (val << 8) | data[offset];
		break;
	case IIO_VELOCITY:
		offset = TS_SENSEBOOST_SENSING_VELOCITY_OFFSET;
		val = (s8)data[offset + 1];
		val = (val << 8) | data[offset];
		break;
	default:
		break;
	}
	return val;
}

static irqreturn_t ts_senseboost_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ts_senseboost_state *state = iio_priv(indio_dev);
	struct ts_senseboost_priv *priv = state->priv;

	mutex_lock(&state->lock);
	state->scan.target_id =
		ts_senseboost_get_data(priv->buf[RESPONSE_RGF_SENSING_DATA], IIO_INDEX);
	state->scan.angle =
		ts_senseboost_get_data(priv->buf[RESPONSE_RGF_SENSING_DATA], IIO_ANGL);
	state->scan.amplitude =
		ts_senseboost_get_data(priv->buf[RESPONSE_RGF_SENSING_DATA], IIO_ALTVOLTAGE);
	state->scan.distance =
		ts_senseboost_get_data(priv->buf[RESPONSE_RGF_SENSING_DATA], IIO_DISTANCE);
	state->scan.velocity =
		ts_senseboost_get_data(priv->buf[RESPONSE_RGF_SENSING_DATA], IIO_VELOCITY);
	mutex_unlock(&state->lock);

	iio_push_to_buffers_with_timestamp(indio_dev, &state->scan,
					   iio_get_time_ns(indio_dev));

	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static int ts_senseboost_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct ts_senseboost_state *state = iio_priv(indio_dev);
	struct ts_senseboost_priv *priv = state->priv;

	switch (mask) {
	case IIO_CHAN_INFO_PROCESSED:
		mutex_lock(&state->lock);
		*val = ts_senseboost_get_data(priv->buf[RESPONSE_RGF_SENSING_DATA], chan->type);
		mutex_unlock(&state->lock);
		return IIO_VAL_INT;
	}

	return -EINVAL;
}

static inline struct ts_senseboost_attribute *
to_ts_senseboost_attr(struct device_attribute *attr)
{
	return container_of(attr, struct ts_senseboost_attribute, dev_attr);
}

static ssize_t ts_senseboost_unpack_attr(struct ts_senseboost_attribute *attr, u8 rx[], char *buf)
{
	u8 i = TS_SENSEBOOST_FRAME_CMD_SIZE + TS_SENSEBOOST_FRAME_LEN_SIZE;

	if (attr->type == TS_SENSEBOOST_ATTR_TYPE_SYS_STATUS) {
		attr->status.data_out_on = (rx[i] & 0x02) >> 1;
		attr->status.plot_on = rx[i] & 0x01;
		attr->status.plot_rx1_on = rx[i + 2] & 0x01; 
		attr->status.plot_rx2_on = (rx[i + 2] & 0x02) >> 1;

		return sysfs_emit(buf, "%d %d %d %d\n", attr->status.data_out_on, attr->status.plot_on, attr->status.plot_rx1_on, attr->status.plot_rx2_on);
	} else if (attr->type == TS_SENSEBOOST_ATTR_TYPE_VERSION) {
		attr->version.major = rx[i];
		attr->version.minor = rx[i + 1];
		attr->version.patch_high = rx[i + 2];
		attr->version.patch_low = rx[i + 3];

		return sysfs_emit(buf, "%d.%d.%c%c\n", attr->version.major, attr->version.minor, attr->version.patch_high, attr->version.patch_low);
	} else if (attr->type == TS_SENSEBOOST_ATTR_TYPE_MODE_GAIN) {
		attr->mode_gain.tx_sentry = rx[i];
		attr->mode_gain.rx01_sentry = (rx[i + 1] >> 4) & 0x0F;
		attr->mode_gain.rx02_sentry = rx[i + 1] & 0x0F;
		attr->mode_gain.hpf01_sentry = rx[i + 2];
		attr->mode_gain.hpf02_sentry = rx[i + 3];
		attr->mode_gain.tx_2d = rx[i + 4];
		attr->mode_gain.rx01_2d = (rx[i + 5] >> 4) & 0x0F;
		attr->mode_gain.rx02_2d = rx[i + 5] & 0x0F;
		attr->mode_gain.hpf01_2d = rx[i + 6];
		attr->mode_gain.hpf02_2d = rx[i + 7];

		return sysfs_emit(buf, "%d %d %d %d %d %d %d %d %d %d\n", \
							attr->mode_gain.tx_sentry, attr->mode_gain.rx01_sentry, \
							attr->mode_gain.rx02_sentry, attr->mode_gain.hpf01_sentry, \
							attr->mode_gain.hpf02_sentry, attr->mode_gain.tx_2d, \
							attr->mode_gain.rx01_2d, attr->mode_gain.rx02_2d, \
							attr->mode_gain.hpf01_2d, attr->mode_gain.hpf02_2d);
	} else if (attr->type == TS_SENSEBOOST_ATTR_TYPE_MODE_CFAR) {
		attr->mode_cfar.threshold_sentry = ((u16)rx[i + 1] << 8 | rx[i]);
		attr->mode_cfar.threshold_2d = ((u16)rx[i + 3] << 8 | rx[i + 2]);

		return sysfs_emit(buf, "%d %d\n", attr->mode_cfar.threshold_sentry, attr->mode_cfar.threshold_2d);
	} else if (attr->type == TS_SENSEBOOST_ATTR_TYPE_MOTION) {
		attr->motion.range_min = ((u16)rx[i + 1] << 8 | rx[i]);
		attr->motion.range_max = ((u16)rx[i + 3] << 8 | rx[i + 2]);
		attr->motion.angle_max = rx[i + 4];

		return sysfs_emit(buf, "%d %d %d\n", attr->motion.range_min, attr->motion.range_max, attr->motion.angle_max);
	} else if (attr->type == TS_SENSEBOOST_ATTR_TYPE_TRACK_TIME) {
		attr->track_time.init = rx[i];
		attr->track_time.delete = rx[i + 1];

		return sysfs_emit(buf, "%d %d\n", attr->track_time.init, attr->track_time.delete);
	} else if (attr->type == TS_SENSEBOOST_ATTR_TYPE_MODE_DUTY) {
		attr->mode_duty.enabled = rx[i];
		attr->mode_duty.rate_sentry = rx[i + 1];
		attr->mode_duty.rate_2d = rx[i + 2];

		return sysfs_emit(buf, "%d %d %d\n", attr->mode_duty.enabled, attr->mode_duty.rate_sentry, attr->mode_duty.rate_2d);
	} else if (attr->type == TS_SENSEBOOST_ATTR_TYPE_MODE_STATUS) {
		attr->mode_status.mode_select = rx[i];
		attr->mode_status.debounce_2d = rx[i + 1];
		attr->mode_status.debounce_sentry = rx[i + 2];

		return sysfs_emit(buf, "%d %d %d\n", attr->mode_status.mode_select, attr->mode_status.debounce_2d, attr->mode_status.debounce_sentry);
	} else if (attr->type == TS_SENSEBOOST_ATTR_TYPE_SETTING_ENABLE) {
		attr->setting_enable.enabled = rx[i];

		return sysfs_emit(buf, "%d\n", attr->setting_enable.enabled);
	} else if (attr->type == TS_SENSEBOOST_ATTR_TYPE_ROI_RANGE) {
		attr->roi_range.range_min = ((u16)rx[i + 1] << 8 | rx[i]);
		attr->roi_range.range_max = ((u16)rx[i + 3] << 8 | rx[i + 2]);
		attr->roi_range.angle_max = rx[i + 4];

		return sysfs_emit(buf, "%d %d %d\n", attr->roi_range.range_min, attr->roi_range.range_max, attr->roi_range.angle_max);
	} else if (attr->type == TS_SENSEBOOST_ATTR_TYPE_ROI_ZONE_1) {
		attr->roi_zone1.ignored[0] = ((u16)rx[i] << 8) | rx[i + 1];
		attr->roi_zone1.ignored[1] = ((u16)rx[i + 2] << 8) | rx[i + 3];
		attr->roi_zone1.ignored[2] = ((u16)rx[i + 4] << 8) | rx[i + 5];
		attr->roi_zone1.ignored[3] = ((u16)rx[i + 6] << 8) | rx[i + 7];

		return sysfs_emit(buf, "0x%04x 0x%04x 0x%04x 0x%04x\n", attr->roi_zone1.ignored[0], attr->roi_zone1.ignored[1], attr->roi_zone1.ignored[2], attr->roi_zone1.ignored[3]);
	} else if (attr->type == TS_SENSEBOOST_ATTR_TYPE_ROI_ZONE_2) {
		attr->roi_zone2.ignored = ((u16)rx[i] << 8 | rx[i + 1]);

		return sysfs_emit(buf, "0x%04x\n", attr->roi_zone2.ignored);
	} else if (attr->type == TS_SENSEBOOST_ATTR_TYPE_ANGLE) {
		attr->angle.reverse = rx[i];
		attr->angle.offset = ((u16)rx[i + 3] << 8 | rx[i + 2]);
		attr->angle.rate = ((u32)rx[i + 7] << 24 | (u32)rx[i + 6] << 16 | (u32)rx[i + 5] << 8 | rx[i + 4]);

		return sysfs_emit(buf, "%d %d 0x%x\n", attr->angle.reverse, attr->angle.offset, attr->angle.rate);
	} else if (attr->type == TS_SENSEBOOST_ATTR_TYPE_GPIO) {
		attr->gpio.gpio01_low = rx[i] & 0x01;
		attr->gpio.gpio02_low = (rx[i] >> 1) & 0x01;

		return sysfs_emit(buf, "%d %d\n", attr->gpio.gpio01_low, attr->gpio.gpio02_low);
	} else {
		return -EINVAL;
	}
}

static ssize_t ts_senseboost_pack_attr(struct ts_senseboost_attribute *attr, char *buf, u8 msg[])
{
	u32 val[TS_SENSEBOOST_ATTR_NUM_MAX] = {0};
	char *token;
	s8 set_idx = ts_senseboost_attribute_map_tbl[attr->type].set_idx;
	s8 num = ts_senseboost_attribute_map_tbl[attr->type].num;
	u8 i;

	for (i = 0; (token = strsep(&buf, " ")) != NULL && i < num; i++) {
		if (kstrtou32(token, 0, &val[i])) {
			return -EINVAL;
		}
	}

	if (attr->type == TS_SENSEBOOST_ATTR_TYPE_SYS_STATUS) {		
		attr->status.data_out_on = val[0];
		attr->status.plot_on = val[1];
		attr->status.plot_rx1_on = val[2];
		attr->status.plot_rx2_on = val[3];

		msg[5] = (msg[5] & ~(1 << 1)) | (attr->status.data_out_on << 1);
		msg[5] = (msg[5] & ~(1)) | (attr->status.plot_on);
		msg[7] = (msg[7] & ~(1)) | (attr->status.plot_rx1_on);
		msg[7] = (msg[7] & ~(1 << 1)) | (attr->status.plot_rx2_on << 1);
	} else if (attr->type == TS_SENSEBOOST_ATTR_TYPE_SYS_CONFIG) {
		set_idx += val[0] > (SET_SYS_CONFIG_DEFAULT - SET_SYS_CONFIG_RESET) ? 0 : val[0];
	} else if (attr->type == TS_SENSEBOOST_ATTR_TYPE_MODE_GAIN) {
		attr->mode_gain.tx_sentry = val[0];
		attr->mode_gain.rx01_sentry = val[1];
		attr->mode_gain.rx02_sentry = val[2];
		attr->mode_gain.hpf01_sentry = val[3];
		attr->mode_gain.hpf02_sentry = val[4];
		attr->mode_gain.tx_2d = val[5];
		attr->mode_gain.rx01_2d = val[6];
		attr->mode_gain.rx02_2d = val[7];
		attr->mode_gain.hpf01_2d = val[8];
		attr->mode_gain.hpf02_2d = val[9];

		msg[5]  = attr->mode_gain.tx_sentry;
		msg[6]  = attr->mode_gain.rx01_sentry << 4 | attr->mode_gain.rx02_sentry;
		msg[7]  = attr->mode_gain.hpf01_sentry;
		msg[8]  = attr->mode_gain.hpf02_sentry;
		msg[9]  = attr->mode_gain.tx_2d;
		msg[10] = attr->mode_gain.rx01_2d << 4 | attr->mode_gain.rx02_2d;
		msg[11] = attr->mode_gain.hpf01_2d;
		msg[12] = attr->mode_gain.hpf02_2d;
	} else if (attr->type == TS_SENSEBOOST_ATTR_TYPE_MODE_CFAR) {
		attr->mode_cfar.threshold_sentry = val[0];
		attr->mode_cfar.threshold_2d = val[1];

		msg[5] = attr->mode_cfar.threshold_sentry & 0xFF;
		msg[6] = (attr->mode_cfar.threshold_sentry >> 8) & 0xFF;
		msg[7] = attr->mode_cfar.threshold_2d & 0xFF;
		msg[8] = (attr->mode_cfar.threshold_2d >> 8) & 0xFF;
	} else if (attr->type == TS_SENSEBOOST_ATTR_TYPE_MOTION) {
		attr->motion.range_min = val[0];
		attr->motion.range_max = val[1];
		attr->motion.angle_max = val[2];

		msg[5] = attr->motion.range_min & 0xFF;
		msg[6] = (attr->motion.range_min >> 8) & 0xFF;
		msg[7] = attr->motion.range_max & 0xFF;
		msg[8] = (attr->motion.range_max >> 8) & 0xFF;
		msg[9] = attr->motion.angle_max;
	} else if (attr->type == TS_SENSEBOOST_ATTR_TYPE_TRACK_TIME) {
		attr->track_time.init = val[0];
		attr->track_time.delete = val[1];

		msg[5] = attr->track_time.init;
		msg[6] = attr->track_time.delete;
	} else if (attr->type == TS_SENSEBOOST_ATTR_TYPE_MODE_DUTY) {
		attr->mode_duty.enabled = val[0];
		attr->mode_duty.rate_sentry = val[1];
		attr->mode_duty.rate_2d = val[2];

		msg[5] = attr->mode_duty.enabled;
		msg[6] = attr->mode_duty.rate_sentry;
		msg[7] = attr->mode_duty.rate_2d;
	} else if (attr->type == TS_SENSEBOOST_ATTR_TYPE_MODE_STATUS) {
		attr->mode_status.mode_select = val[0];
		attr->mode_status.debounce_2d = val[1];
		attr->mode_status.debounce_sentry = val[2];

		msg[5] = attr->mode_status.mode_select;
		msg[6] = attr->mode_status.debounce_2d;
		msg[7] = attr->mode_status.debounce_sentry;
		msg[10] = 0x04;
	} else if (attr->type == TS_SENSEBOOST_ATTR_TYPE_SETTING_ENABLE) {
		attr->setting_enable.enabled = val[0];
		
		msg[5] = attr->setting_enable.enabled;
	} else if (attr->type == TS_SENSEBOOST_ATTR_TYPE_ROI_RANGE) {
		attr->roi_range.range_min = val[0];
		attr->roi_range.range_max = val[1];
		attr->roi_range.angle_max = val[2];

		msg[5] = attr->roi_range.range_min & 0xFF;
		msg[6] = (attr->roi_range.range_min >> 8) & 0xFF;
		msg[7] = attr->roi_range.range_max & 0xFF;
		msg[8] = (attr->roi_range.range_max >> 8) & 0xFF;
		msg[9] = attr->roi_range.angle_max;
	} else if (attr->type == TS_SENSEBOOST_ATTR_TYPE_ROI_ZONE_1) {
		attr->roi_zone1.ignored[0] = val[0];
		attr->roi_zone1.ignored[1] = val[1];
		attr->roi_zone1.ignored[2] = val[2];
		attr->roi_zone1.ignored[3] = val[3];

		msg[5] = (attr->roi_zone1.ignored[0] >> 8) & 0xFF;
		msg[6] = attr->roi_zone1.ignored[0] & 0xFF;
		msg[7] = (attr->roi_zone1.ignored[1] >> 8) & 0xFF;
		msg[8] = attr->roi_zone1.ignored[1] & 0xFF;
		msg[9] = (attr->roi_zone1.ignored[2] >> 8) & 0xFF;
		msg[10] = attr->roi_zone1.ignored[2] & 0xFF;
		msg[11] = (attr->roi_zone1.ignored[3] >> 8) & 0xFF;
		msg[12] = attr->roi_zone1.ignored[3] & 0xFF;
	} else if (attr->type == TS_SENSEBOOST_ATTR_TYPE_ROI_ZONE_2) {
		attr->roi_zone2.ignored = val[0];
		
		msg[5] = (attr->roi_zone2.ignored >> 8) & 0xFF;
		msg[6] = attr->roi_zone2.ignored & 0xFF;
	} else if (attr->type == TS_SENSEBOOST_ATTR_TYPE_ANGLE) {
		attr->angle.reverse = val[0];
		attr->angle.offset = val[1];
		attr->angle.rate = val[2];

		msg[5] = attr->angle.reverse;
		msg[7] = attr->angle.offset & 0xFF;
		msg[8] = (attr->angle.offset >> 8) & 0xFF;
		msg[9] = attr->angle.rate & 0xFF;
		msg[10] = (attr->angle.rate >> 8) & 0xFF;
		msg[11] = (attr->angle.rate >> 16) & 0xFF;
		msg[12] = (attr->angle.rate >> 24) & 0xFF;
	} else if (attr->type == TS_SENSEBOOST_ATTR_TYPE_GPIO) {
		attr->gpio.gpio01_low = val[0];
		attr->gpio.gpio02_low = val[1];

		msg[5] = attr->gpio.gpio01_low | (attr->gpio.gpio02_low << 1);
	}

	memcpy(msg + TS_SENSEBOOST_FRAME_START_SIZE, ts_senseboost_set_tbl[set_idx], TS_SENSEBOOST_FRAME_CMD_SIZE);
	msg[TS_SENSEBOOST_FRAME_SIZE - 1] = ts_senseboost_calc_checksum(&msg[TS_SENSEBOOST_FRAME_START_SIZE], TS_SENSEBOOST_FRAME_BODY_SIZE);

	return 0;
}

static ssize_t ts_senseboost_show_attr(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ts_senseboost_state *state = iio_priv(indio_dev);
	struct ts_senseboost_attribute *ts_senseboost_attr = to_ts_senseboost_attr(attr);
	u8 rx[TS_SENSEBOOST_FRAME_BODY_SIZE];
	u8 msg[] = TS_SENSEBOOST_MSG_TEMPLATE;
	s8 query_idx, response_idx;
	int ret = 0;

	if (ts_senseboost_attr->type < TS_SENSEBOOST_ATTR_TYPE_NUM && ts_senseboost_attribute_map_tbl[ts_senseboost_attr->type].query_idx != QUERY_NOP) {
		query_idx = ts_senseboost_attribute_map_tbl[ts_senseboost_attr->type].query_idx;
		response_idx = ts_senseboost_attribute_map_tbl[ts_senseboost_attr->type].response_idx;
	} else {
		return -ENXIO;
	}

	memcpy(msg + TS_SENSEBOOST_FRAME_START_SIZE, ts_senseboost_query_tbl[query_idx], TS_SENSEBOOST_FRAME_CMD_SIZE);
	mutex_lock(&state->lock);
	ret = ts_senseboost_xfer_cmd(state, msg, response_idx);
	memcpy(rx, state->priv->buf[response_idx], TS_SENSEBOOST_FRAME_BODY_SIZE);
	mutex_unlock(&state->lock);

	if (ret)
		return ret;

	return ts_senseboost_unpack_attr(ts_senseboost_attr, rx, buf);
}

static ssize_t ts_senseboost_store_attr(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ts_senseboost_state *state = iio_priv(indio_dev);
	struct ts_senseboost_attribute *ts_senseboost_attr = to_ts_senseboost_attr(attr);
	u8 msg[] = TS_SENSEBOOST_MSG_TEMPLATE;
	char dup[TS_SENSEBOOST_ATTRIBUTES_SIZE_MAX];
	s8 set_idx, response_idx;
	int ret;

	if (ts_senseboost_attr->type < TS_SENSEBOOST_ATTR_TYPE_NUM && ts_senseboost_attribute_map_tbl[ts_senseboost_attr->type].set_idx != SET_NOP) {
		set_idx = ts_senseboost_attribute_map_tbl[ts_senseboost_attr->type].set_idx;
		response_idx = ts_senseboost_attribute_map_tbl[ts_senseboost_attr->type].response_idx;
	} else {
		return -ENXIO;
	}

	if (len >= TS_SENSEBOOST_ATTRIBUTES_SIZE_MAX) {
		return -ENOMEM;
	}
	memcpy(dup, buf, len);
	dup[len] = 0;

	ret = ts_senseboost_pack_attr(ts_senseboost_attr, dup, msg);
	if (ret)
		return ret;

	mutex_lock(&state->lock);
	ts_senseboost_xfer_cmd(state, msg, response_idx);
	mutex_unlock(&state->lock);
	
	return len;
}

#define TS_ATTR(_name, _mode, _show, _store, _type)	{ \
	.dev_attr = __ATTR(_name, _mode, _show, _store), \
	.type = _type, \
}

#define TS_SENSEBOOST_ATTR(_name, _mode, _type) \
	struct ts_senseboost_attribute ts_senseboost_attr_##_name =	\
		TS_ATTR(_name, _mode, ts_senseboost_show_attr, ts_senseboost_store_attr, _type)

static TS_SENSEBOOST_ATTR(system_status,  S_IRUGO | S_IWUSR, TS_SENSEBOOST_ATTR_TYPE_SYS_STATUS);
static TS_SENSEBOOST_ATTR(system_config,  S_IWUSR,           TS_SENSEBOOST_ATTR_TYPE_SYS_CONFIG);
static TS_SENSEBOOST_ATTR(version,        S_IRUGO,           TS_SENSEBOOST_ATTR_TYPE_VERSION);
static TS_SENSEBOOST_ATTR(mode_gain,      S_IRUGO | S_IWUSR, TS_SENSEBOOST_ATTR_TYPE_MODE_GAIN);
static TS_SENSEBOOST_ATTR(mode_cfar,      S_IRUGO | S_IWUSR, TS_SENSEBOOST_ATTR_TYPE_MODE_CFAR);
static TS_SENSEBOOST_ATTR(motion,         S_IRUGO | S_IWUSR, TS_SENSEBOOST_ATTR_TYPE_MOTION);
static TS_SENSEBOOST_ATTR(track_time,     S_IRUGO | S_IWUSR, TS_SENSEBOOST_ATTR_TYPE_TRACK_TIME);
static TS_SENSEBOOST_ATTR(mode_duty,      S_IRUGO | S_IWUSR, TS_SENSEBOOST_ATTR_TYPE_MODE_DUTY);
static TS_SENSEBOOST_ATTR(mode_status,    S_IRUGO | S_IWUSR, TS_SENSEBOOST_ATTR_TYPE_MODE_STATUS);
static TS_SENSEBOOST_ATTR(setting_enable, S_IRUGO | S_IWUSR, TS_SENSEBOOST_ATTR_TYPE_SETTING_ENABLE);
static TS_SENSEBOOST_ATTR(roi_range,      S_IRUGO | S_IWUSR, TS_SENSEBOOST_ATTR_TYPE_ROI_RANGE);
static TS_SENSEBOOST_ATTR(roi_zone_1,     S_IRUGO | S_IWUSR, TS_SENSEBOOST_ATTR_TYPE_ROI_ZONE_1);
static TS_SENSEBOOST_ATTR(roi_zone_2,     S_IRUGO | S_IWUSR, TS_SENSEBOOST_ATTR_TYPE_ROI_ZONE_2);
static TS_SENSEBOOST_ATTR(angle,          S_IRUGO | S_IWUSR, TS_SENSEBOOST_ATTR_TYPE_ANGLE);
static TS_SENSEBOOST_ATTR(gpio,           S_IRUGO | S_IWUSR, TS_SENSEBOOST_ATTR_TYPE_GPIO);

static struct attribute *ts_senseboost_attributes[] = {
	&ts_senseboost_attr_system_status.dev_attr.attr,
	&ts_senseboost_attr_system_config.dev_attr.attr,
	&ts_senseboost_attr_version.dev_attr.attr,
	&ts_senseboost_attr_mode_gain.dev_attr.attr,
	&ts_senseboost_attr_mode_cfar.dev_attr.attr,
	&ts_senseboost_attr_motion.dev_attr.attr,
	&ts_senseboost_attr_track_time.dev_attr.attr,
	&ts_senseboost_attr_mode_duty.dev_attr.attr,
	&ts_senseboost_attr_mode_status.dev_attr.attr,
	&ts_senseboost_attr_setting_enable.dev_attr.attr,
	&ts_senseboost_attr_roi_range.dev_attr.attr,
	&ts_senseboost_attr_roi_zone_1.dev_attr.attr,
	&ts_senseboost_attr_roi_zone_2.dev_attr.attr,
	&ts_senseboost_attr_angle.dev_attr.attr,
	&ts_senseboost_attr_gpio.dev_attr.attr,
	NULL,
};

static const struct attribute_group ts_senseboost_group = {
	.attrs = ts_senseboost_attributes,
};

static const struct iio_info ts_senseboost_info = {
	.read_raw = ts_senseboost_read_raw,
	.attrs    = &ts_senseboost_group,
};

static const struct iio_chan_spec ts_senseboost_channels[] = {
	TS_SENSEBOOST_CHAN(IIO_INDEX,      0, 'u', 16, 16),
	TS_SENSEBOOST_CHAN(IIO_ANGL,       1, 's', 16, 16),
	TS_SENSEBOOST_CHAN(IIO_ALTVOLTAGE, 2, 'u', 16, 16),
	TS_SENSEBOOST_CHAN(IIO_DISTANCE,   3, 'u', 16, 16),
	TS_SENSEBOOST_CHAN(IIO_VELOCITY,   4, 's', 16, 16),
	IIO_CHAN_SOFT_TIMESTAMP(5),
};

static bool ts_senseboost_frame_is_okay(struct ts_senseboost_priv *priv)
{
	int offset = priv->length - TS_SENSEBOOST_FRAME_CHECKSUM_SIZE;
	u8 checksum = *(priv->buf[priv->buf_idx] + offset);

	return checksum == ts_senseboost_calc_checksum(priv->buf[priv->buf_idx], priv->length);
}

static u8 ts_senseboost_lookup_response_pos(const u8 response_tbl[][TS_SENSEBOOST_FRAME_CMD_SIZE], u8 cmd1, u8 cmd2)
{
	u8 pos;

	for (pos = 0; pos < RESPONSE_NUM; pos++) {
		if (response_tbl[pos][0] == cmd1 && response_tbl[pos][1] == cmd2)
			break;
	}

	return pos;
}

static int ts_senseboost_receive_buf(struct serdev_device *serdev, const unsigned char *buf, size_t size)
{
	struct iio_dev *indio_dev = serdev_device_get_drvdata(serdev);
	struct ts_senseboost_state *state = iio_priv(indio_dev);
	struct ts_senseboost_priv *priv = state->priv;
	int num;

	if (!priv->expected_length) {
		u16 start;

		if (size < TS_SENSEBOOST_FRAME_START_SIZE)
			return 0;

		start = get_unaligned_be16(buf);
		if (start != TS_SENSEBOOST_FRAME_START)
			return 1;

		priv->expected_length = TS_SENSEBOOST_FRAME_BODY_SIZE;
		priv->length = 0;

		return TS_SENSEBOOST_FRAME_START_SIZE;
	}

	if (!priv->length) {
		if (size < TS_SENSEBOOST_FRAME_CMD_SIZE)
			return 0;

		if (*buf != TS_SENSEBOOST_FRAME_CMD_RGF && *buf != TS_SENSEBOOST_FRAME_CMD_SYS) {
			priv->expected_length = 0;
			return 1;
		}

		priv->buf_idx = ts_senseboost_lookup_response_pos(ts_senseboost_response_tbl, *buf, *(buf + 1));
		if (priv->buf_idx >= RESPONSE_NUM) {
			priv->expected_length = 0;
			return 1;
		}
	}

	num = min(size, (size_t)(priv->expected_length - priv->length));
	memcpy(priv->buf[priv->buf_idx] + priv->length, buf, num);
	priv->length += num;

	if (priv->length == priv->expected_length) {
		if (ts_senseboost_frame_is_okay(priv)) {
			complete(&priv->frame_ready[priv->buf_idx]);
		}
		priv->expected_length = 0;
	}

	return num;
}

static const struct serdev_device_ops ts_senseboost_serdev_ops = {
	.receive_buf = ts_senseboost_receive_buf,
	.write_wakeup = serdev_device_write_wakeup,
};

static int ts_senseboost_iio_parse_dt(struct device *dev, struct ts_senseboost_state *state)
{
    struct device_node *node = dev->of_node;

    if (!node) {
        return -ENODEV;
    }

    if (of_property_read_string(node, "dev-name", &state->dev_name) != 0) {
        state->dev_name = NULL;
    }

    return 0;
}

static int ts_senseboost_probe(struct serdev_device *serdev)
{
	struct ts_senseboost_state *state;
	struct iio_dev *indio_dev;
	struct ts_senseboost_priv *priv;
	int i, ret;

	indio_dev = devm_iio_device_alloc(&serdev->dev, sizeof(*state));
	if (!indio_dev)
		return -ENOMEM;

	state = iio_priv(indio_dev);

    ret = ts_senseboost_iio_parse_dt(&serdev->dev, state);
    if (ret < 0) {
        dev_err(&serdev->dev, "failed to parse dt %d!\n", ret);
        return -EINVAL;
    }

    if (state->dev_name) {
        dev_set_name(&indio_dev->dev, state->dev_name);
    } else {
        dev_set_name(&indio_dev->dev, "iio:ts_senseboost_ph12");
    }

	serdev_device_set_drvdata(serdev, indio_dev);
	state->serdev = serdev;

	indio_dev->dev.parent = &serdev->dev;
	indio_dev->info = &ts_senseboost_info;
	indio_dev->name = TS_SENSEBOOST_DRIVER_NAME;
	indio_dev->channels = ts_senseboost_channels,
	indio_dev->num_channels = ARRAY_SIZE(ts_senseboost_channels);
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->available_scan_masks = ts_senseboost_scan_masks;

	priv = devm_kzalloc(&serdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->length = 0;
	state->priv = priv;
	mutex_init(&state->lock);

	for (i = 0; i < RESPONSE_NUM; i++) {
		init_completion(&priv->frame_ready[i]);
	}

	serdev_device_set_client_ops(serdev, &ts_senseboost_serdev_ops);
	ret = devm_serdev_device_open(&serdev->dev, serdev);
	if (ret)
		return ret;

	serdev_device_set_baudrate(serdev, 115200);
	serdev_device_set_flow_control(serdev, false);
	ret = serdev_device_set_parity(serdev, SERDEV_PARITY_NONE);
	if (ret)
		return ret;

	ret = devm_iio_triggered_buffer_setup(&serdev->dev, indio_dev, NULL,
					      ts_senseboost_trigger_handler, NULL);
	if (ret)
		return ret;

	return devm_iio_device_register(&serdev->dev, indio_dev);
}

static const struct of_device_id ts_senseboost_of_match[] = {
	{ .compatible = "ts,senseboost", },
	{ }
};
MODULE_DEVICE_TABLE(of, ts_senseboost_of_match);

static struct serdev_device_driver ts_senseboost_driver = {
	.driver = {
		.name = TS_SENSEBOOST_DRIVER_NAME,
		.of_match_table = ts_senseboost_of_match,
	},
	.probe = ts_senseboost_probe,
};
module_serdev_device_driver(ts_senseboost_driver);

MODULE_AUTHOR("");
MODULE_DESCRIPTION("Terasillic senseboost radar sensor driver");
MODULE_LICENSE("GPL v2");
