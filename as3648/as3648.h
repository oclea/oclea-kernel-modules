#ifndef UAPI_AS3648_H_
#define UAPI_AS3648_H_

#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/videodev2.h>

enum as3648_LED
{
	as3648_LED_1 = 0,
	as3648_LED_2
};

enum as3648_projector_type
{
	as3648_dot = 0,
	as3648_flood,
	as3648_max_types
};

enum as3648_mode
{
	as3648_mode_shutdown		= 0x00,
	as3648_mode_indicator		= 0x01,
	as3648_mode_assist_light	= 0x02,
	as3648_mode_flash		= 0x03,
	as3648_mode_max			= 0x04,
};

enum as3648_output_state
{
	as3648_output_off		= 0x00,
	as3648_output_on		= 0x08,
};

struct as3648_control
{
	uint32_t mode;
	uint32_t output_state;
} __attribute__((packed));

enum as3648_fault
{
	as3648_no_fault			= 0x00,
	as3648_undervoltage_err		= 0x01,
	as3648_txmask_err		= 0x08,
	as3648_timeout_err		= 0x10,
	as3648_overtemp_err		= 0x20,
	as3648_led_short_err		= 0x40,
	as3648_overvolt_prot_err	= 0x80
};

#define AS3648_GET_ILLUMINATOR_TYPE_IOCTL_CMD \
	_IOR('V', BASE_VIDIOC_PRIVATE + 52, uint32_t)

#define AS3648_GET_LED_CURRENT1_MA_IOCTL_CMD \
	_IOR('V', BASE_VIDIOC_PRIVATE + 53, uint32_t)

#define AS3648_GET_LED_CURRENT2_MA_IOCTL_CMD \
	_IOR('V', BASE_VIDIOC_PRIVATE + 54, uint32_t)

#define AS3648_SET_LED_CURRENT1_MA_IOCTL_CMD \
	_IOW('V', BASE_VIDIOC_PRIVATE + 55, uint32_t)

#define AS3648_SET_LED_CURRENT2_MA_IOCTL_CMD \
	_IOW('V', BASE_VIDIOC_PRIVATE + 56, uint32_t)

#define AS3648_SET_CTRL_IOCTL_CMD \
	_IOW('V', BASE_VIDIOC_PRIVATE + 57, struct as3648_control)

#define AS3648_GET_CTRL_IOCTL_CMD \
	_IOR('V', BASE_VIDIOC_PRIVATE + 58, struct as3648_control)

#define AS3648_GET_TIMEOUT_MS_IOCTL_CMD \
	_IOR('V', BASE_VIDIOC_PRIVATE + 59, uint32_t)

#define AS3648_SET_TIMEOUT_MS_IOCTL_CMD \
	_IOW('V', BASE_VIDIOC_PRIVATE + 60, uint32_t)

#define AS3648_GET_MAX_LED_CURRENT_MA_IOCTL_CMD \
	_IOR('V', BASE_VIDIOC_PRIVATE + 61, uint32_t)

#define AS3648_GET_MAX_TIMEOUT_MS_IOCTL_CMD \
	_IOR('V', BASE_VIDIOC_PRIVATE + 62, uint32_t)

#define AS3648_ARM_SELECT_IOCTL_CMD \
	_IOR('V', BASE_VIDIOC_PRIVATE + 63, uint32_t)

#endif
