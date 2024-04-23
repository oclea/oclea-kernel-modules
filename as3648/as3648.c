/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Flash driver for ams AS3648 flash chip.
 *
 * Copyright (C) 2020, ams AG, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed .as is. WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/mutex.h>
#include <linux/printk.h>
#include "as3648.h"
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/fs.h>

#define AS3648_DRV_VERSION_STR      "0.16"
#define AS3648_MAX_CDEV_MINORS      10
#define AS3648_MODULE_NAME          "as3648"
#define AS3648_FIRST_MINOR          0

#define CHIP_ID_ADDR                0x00
#define LED_CURRENT_1_REG           0x01
#define LED_CURRENT_2_REG           0x02
#define TX_MASK_REG                 0x03
#define LOW_VOLTAGE_REG             0x04
#define FLASH_TIMER_REG             0x05
#define CTRL_REG                    0x06
#define STROBE_REG                  0x07
#define FAULT_REG                   0x08
#define PWM_IND_REG                 0x09
#define MIN_LED_CURRENT_REG         0x0E
#define ACTUAL_LED_CURRENT_REG      0x0F
#define TIMEOUT_THRESHOLD_REG       0x7F
#define PASSWORD_REG                0x80
#define CURRENT_BOOST_REG           0x81

#define CURRENT_RATIO_NUMERATOR     3530
#define CURRENT_RATIO_DENOMINATOR   1000

#define TIMEOUT_THRESHOLD_VALUE     256
#define TIMEOUT_INCREMENT_SLOW      2
#define TIMEOUT_INCREMENT_FAST      8
#define TIMEOUT_OFFSET_MS           2

#define DEV_ID_MASK                 0xF8
#define EXPECTED_DEV_ID             0xB0

#define MAX_CURRENT_MA              900
#define MAX_TIMEOUT_MS              1280

#define CTRL_REG_MODE_MASK          0x03
#define CTRL_REG_OUT_ON_MASK        0x08

#define SYNC_FRAME_TS_DIFF          2000000   // 2 ms
#define FRAME_INTV_NS               33000000  // 33 ms
#define FRAME_INTV_TWICE_NS         (2 * FRAME_INTV_NS)

#define FALI_EXIT_ON_AS3648_NOT_ACQUIRED(flash_dev) \
    if (flash_dev->state != as3648_acquired) \
    { \
        AS3648_WARN("Device not acquired\n"); \
        return -EIO; \
    }

enum as3648_state
{
    as3648_released,
    as3648_acquired
};

struct as3648_device
{
    struct i2c_client          *i2c_client;
    struct cdev                 cdev;
    struct device               dev;
    int                         minor;
    struct mutex                mutex;
    struct device_node          *of_node;
    enum as3648_state           state;

    enum as3648_projector_type  type;
    uint32_t                    max_current_ma;
    uint32_t                    max_timeout_ms;
    uint32_t                    default_timeout_ms;
    uint32_t                    led_1_current_ma;
    uint32_t                    led_2_current_ma;
    uint32_t                    timeout_ms;    
};

#define PRJ_REC_LEN 64
struct projector_work_record
{
    struct {
        u64 ts_ns;
        u32 type;
    } prj_rec [PRJ_REC_LEN];

    int store_idx;
    int first_frame_flag;
    u64 first_frame_ts_ns;
};

#define GET_AS3648_DEVICE_FROM_CDEV(chardev) \
          container_of(chardev, struct as3648_device, cdev)
#define AS3648_LOG(level, ...) printk(level "AS3648: " __VA_ARGS__)
#define AS3648_DEBUG(...) AS3648_LOG(KERN_DEBUG, __VA_ARGS__)
#define AS3648_INFO(...) AS3648_LOG(KERN_INFO, __VA_ARGS__)
#define AS3648_WARN(...) AS3648_LOG(KERN_WARNING, __VA_ARGS__)
#define AS3648_ERROR(...) AS3648_LOG(KERN_ERR, __VA_ARGS__)

static int as3648_major;
static DEFINE_IDA(as3648_minor_ida);
static struct class *as3648_class = NULL;
static struct mutex flash_dev_switch_mutex;
static struct as3648_device* g_flash_dev_armed = NULL;
static struct as3648_device* g_dots_dev = NULL;
static struct as3648_device* g_flood_dev = NULL;
static struct task_struct *g_flash_current_arm_task;
static struct semaphore g_flash_arm_sem;

/* 1 means let dots/flood projector works in alternate mode */
static int alternate_mode = 0;
struct projector_work_record g_prj_types;

static u64 frame_intv_ns, frame_intv_twice_ns;

/**
 * Read number of bytes starting at a specific address over I2C
 *
 * @client: the i2c client
 * @reg: the i2c register address
 * @buf: pointer to a buffer that will contain the received data
 * @len: number of bytes to read
 */
static int as3648_i2c_read(struct i2c_client *client,
    char reg, char *buf, int len)
{
  struct i2c_msg msgs[2];
  int ret;

  msgs[0].flags = 0;
  msgs[0].addr  = client->addr;
  msgs[0].len   = 1;
  msgs[0].buf   = &reg;

  msgs[1].flags = I2C_M_RD;
  msgs[1].addr  = client->addr;
  msgs[1].len   = len;
  msgs[1].buf   = buf;

  ret = i2c_transfer(client->adapter, msgs, 2);

  return ret < 0 ? ret : (ret != ARRAY_SIZE(msgs) ? -EIO : 0);
}

/**
 * as3648_i2c_write - Write nuber of bytes starting at a specific 
 *                    address over I2C
 *
 * @client: the i2c client
 * @reg: the i2c register address
 * @buf: pointer to a buffer that will contain the data to write
 * @len: number of bytes to write
 */
static int as3648_i2c_write(struct i2c_client *client,
    char reg, const char *buf, int len)
{
  u8 *addr_buf;
  struct i2c_msg msg;
  int ret;

  addr_buf = kmalloc(len + 1, GFP_KERNEL);
  if (!addr_buf)
    return -ENOMEM;

  addr_buf[0] = reg;
  memcpy(&addr_buf[1], buf, len);
  msg.flags = 0;
  msg.addr = client->addr;
  msg.buf = addr_buf;
  msg.len = len + 1;

  ret = i2c_transfer(client->adapter, &msg, 1);
  
  if (ret != 1) 
  {
      AS3648_ERROR("i2c_transfer failed: %d msg_len: %u", ret, len);
  }

  kfree(addr_buf);
  return ret < 0 ? ret : (ret != 1 ? -EIO : 0);
}

static int cam_as3648_register_read(struct as3648_device* flash_dev,
    uint8_t reg, uint8_t *value)
{
    int rc = 0;

    if (NULL == flash_dev)
    {
        AS3648_ERROR("Flash device is NULL\n");
        return -EINVAL;
    }

    rc = as3648_i2c_read(flash_dev->i2c_client, reg, value, 1);
    if (rc < 0)
    {
        AS3648_ERROR("Error performing cci read\n");
    }

    return rc;
}

static int cam_as3648_register_write(struct as3648_device* flash_dev,
    uint8_t reg, uint8_t value)
{
    uint8_t data = value;

    return as3648_i2c_write(flash_dev->i2c_client, reg, &data, 1);
}

static int cam_as3648_power_up(struct as3648_device* flash_dev)
{
    int32_t                 rc = 0;

    // wxl, to be added for specific hw if necessary
    return rc;
}

static int cam_as3648_power_down(struct as3648_device *flash_dev)
{
    int  rc = 0;

    // wxl, to be added for specific hw if necessary
    return rc;
}

static uint8_t cam_as3648_current_to_reg(struct as3648_device* flash_dev,
    uint32_t current_ma)
{
    uint32_t reg_value = 0;

    if (NULL == flash_dev)
    {
        // Since we cannot figure out the maximum current 
        // for this flash dev, return the lowest.
        return 0;
    }

    // saturate at maximum current;
    if (current_ma > flash_dev->max_current_ma)
    {
        current_ma = flash_dev->max_current_ma;
    }

    reg_value = ((current_ma * CURRENT_RATIO_DENOMINATOR) + 
        (CURRENT_RATIO_NUMERATOR >> 1)) / CURRENT_RATIO_NUMERATOR;

    if (reg_value > 0xFF)
    {
        reg_value = 0xFF;
    }

    return (uint8_t)reg_value;
}

static inline uint32_t cam_as3648_reg_to_current(uint8_t reg_value)
{
    return ((uint32_t)reg_value * CURRENT_RATIO_NUMERATOR) /
           CURRENT_RATIO_DENOMINATOR;
}

static uint8_t cam_as3648_timeout_to_reg(struct as3648_device* flash_dev,
    uint32_t timeout_ms)
{
    uint8_t reg_value = 0;

    if (timeout_ms > flash_dev->max_timeout_ms)
    {
        timeout_ms = flash_dev->max_timeout_ms;
    }

    if (timeout_ms < TIMEOUT_OFFSET_MS)
    {
        return 0;
    }

    if (timeout_ms > TIMEOUT_THRESHOLD_VALUE)
    {
        timeout_ms -= TIMEOUT_THRESHOLD_VALUE;
        reg_value = TIMEOUT_THRESHOLD_REG +
                    (timeout_ms / TIMEOUT_INCREMENT_FAST);
    }
    else
    {
        reg_value = (timeout_ms - TIMEOUT_OFFSET_MS) /
                    TIMEOUT_INCREMENT_SLOW;
    }

    return reg_value;
}

static uint32_t cam_as3648_reg_to_timeout(uint8_t reg_value)
{
    uint32_t timeout_ms = 0;
    if (reg_value > TIMEOUT_THRESHOLD_REG)
    {
        reg_value -= TIMEOUT_THRESHOLD_REG;
        timeout_ms = TIMEOUT_THRESHOLD_VALUE + 
                     (reg_value * TIMEOUT_INCREMENT_FAST);
    }
    else
    {
        timeout_ms = reg_value * TIMEOUT_INCREMENT_SLOW +
                     TIMEOUT_OFFSET_MS;
    }

    return timeout_ms;
}

static int as3648_validate_ctrl_structure(struct as3648_control* ctrl)
{
    if (NULL == ctrl) 
    {
        return -EINVAL;
    }

    if (ctrl->mode >= as3648_mode_max) 
    {
        return -EINVAL;
    }

    if ((ctrl->output_state) & ~as3648_output_on) 
    {
        return -EINVAL;
    }

    return 0;
/*
    // Allow only flash mode or shutdown for eye safety
    if ((ctrl->mode != as3648_mode_shutdown) &&
        (ctrl->mode != as3648_mode_flash))
    {
        return -EINVAL;
    }

    return 0;
*/
}

static int cam_as3648_acquire(struct as3648_device* flash_dev)
{
    int rc = 0;

    if (NULL == flash_dev)
    {
        AS3648_ERROR("Flash device is NULL\n");
        return -EINVAL;
    }

    mutex_lock(&flash_dev->mutex);

    if (flash_dev->state == as3648_acquired) 
    {
        AS3648_WARN("Device already acquired\n");
        rc = -EALREADY;
        goto unlock_mutex;
    }

    rc = cam_as3648_power_up(flash_dev);
    if (rc < 0) 
    {
        AS3648_ERROR("Error Powering up as3648\n");
        goto unlock_mutex;
    }

    flash_dev->state = as3648_acquired;

unlock_mutex:
    mutex_unlock(&flash_dev->mutex);
    return rc;
}

static int cam_as3648_release(struct as3648_device*  flash_dev)
{
    int rc = 0;

    if (NULL == flash_dev)
    {
        AS3648_ERROR("Flash device is NULL\n");
        return -EINVAL;
    }

    mutex_lock(&flash_dev->mutex);

    if (flash_dev->state == as3648_released)
    {
        AS3648_WARN("Device already released\n");
        rc = -EALREADY;
        goto unlock_mutex;
    }

    rc = cam_as3648_power_down(flash_dev);
    if (rc < 0) 
    {
        AS3648_ERROR("Error Powering down as3648\n");
        goto unlock_mutex;
    }

    flash_dev->state  = as3648_released;

unlock_mutex:
    mutex_unlock(&flash_dev->mutex);
    return rc;
}

static int cam_as3648_get_device_identifier(
    struct as3648_device *flash_dev, uint8_t* id)
{
    int rc = 0;

    if ((NULL == flash_dev) || (id == NULL))
    {
        AS3648_ERROR("flash device or id is NULL\n");
        return -EINVAL;
    }

    mutex_lock(&flash_dev->mutex);
    rc = cam_as3648_register_read(flash_dev, CHIP_ID_ADDR, id);
    mutex_unlock(&flash_dev->mutex);
    return rc;
}

static int cam_as3648_identify_device(struct as3648_device *flash_dev)
{
    int rc = -ENODEV;
    uint8_t id = 0;

    FALI_EXIT_ON_AS3648_NOT_ACQUIRED(flash_dev);
    AS3648_INFO("Probing as3648 at address 0x%x ...\n", flash_dev->i2c_client->addr);
    rc = cam_as3648_get_device_identifier(flash_dev, &id);
    if ( (rc == 0) && ((id & DEV_ID_MASK) == EXPECTED_DEV_ID))
    {
        rc = 0;
        AS3648_INFO("as3648 probed ok\n");
    }

    return rc;
}

static int cam_as3648_set_led_current(struct as3648_device*  flash_dev,
    enum as3648_LED led, uint32_t  current_ma)
{
    int rc = 0;
    uint8_t target_register = 0x00;
    uint8_t target_value = 0x00;

    if (NULL == flash_dev)
    {
        AS3648_ERROR("flash device is NULL\n");
        return -EINVAL;
    }

    mutex_lock(&flash_dev->mutex);

    switch (led)
    {
        case as3648_LED_1:
            target_register = LED_CURRENT_1_REG;
            break;
        case as3648_LED_2:
            target_register = LED_CURRENT_2_REG;
            break;
        default:
            AS3648_ERROR("Invalid value for LED index: %d\n", led);
            rc = -EINVAL;
            goto unlock_mutex;
    }

    target_value = cam_as3648_current_to_reg(flash_dev, current_ma);

    rc = cam_as3648_register_write(flash_dev,
                                   target_register, target_value);
    if (rc < 0)
    {
        AS3648_ERROR("Error writing register 0x%02x with value 0x%02x\n",
                      target_register, target_value);
        goto unlock_mutex;
    }

unlock_mutex:
    mutex_unlock(&flash_dev->mutex);
    return rc;
}

static int cam_as3648_get_led_current(struct as3648_device* flash_dev,
    enum as3648_LED  led, uint32_t*  current_ma)
{
    int rc = 0;
    uint8_t target_register = 0x00;
    uint8_t target_value = 0x00;

    if ((NULL == flash_dev) || (NULL == current_ma))
    {
        AS3648_ERROR("flash device or current_ma is NULL\n");
        return -EINVAL;
    }

    mutex_lock(&flash_dev->mutex);

    switch (led)
    {
        case as3648_LED_1:
            target_register = LED_CURRENT_1_REG;
            break;
        case as3648_LED_2:
            target_register = LED_CURRENT_2_REG;
            break;
        default:
            AS3648_ERROR("Invalid value for LED index: %d\n", led);
            rc = -EINVAL;
            goto unlock_mutex;
    }

    rc = cam_as3648_register_read(flash_dev, target_register, &target_value);
    if (rc < 0)
    {
        AS3648_ERROR("Error reading register 0x%02x\n", target_register);
        goto unlock_mutex;
    }

    *current_ma = cam_as3648_reg_to_current(target_value);

unlock_mutex:
    mutex_unlock(&flash_dev->mutex);
    return rc;
}

static int cam_as3648_get_projector_type(
    struct as3648_device* flash_dev, uint32_t* type)
{
    int rc = 0;

    if ((NULL == flash_dev) || (NULL == type))
    {
        AS3648_ERROR("flash device or type is NULL\n");
        return -EINVAL;
    }

    mutex_lock(&flash_dev->mutex);

    *type = (uint32_t)flash_dev->type;

    mutex_unlock(&flash_dev->mutex);
    return rc;
}

static int cam_as3648_get_state(struct as3648_device* flash_dev,
    enum as3648_state *state)
{
    int rc = 0;

    if ((NULL == flash_dev) || (NULL == state))
    {
        AS3648_ERROR("flash device or state is NULL\n");
        return -EINVAL;
    }

    mutex_lock(&flash_dev->mutex);

    *state = flash_dev->state;

    mutex_unlock(&flash_dev->mutex);
    return rc;
}

static int cam_as3648_set_ctrl(struct as3648_device* flash_dev,
    struct as3648_control* control)
{
    int rc = 0;
    uint8_t target_value = 0x00;

    if ((NULL == flash_dev) || (NULL == control))
    {
        AS3648_ERROR("flash device or control is NULL\n");
        return -EINVAL;
    }

    rc = as3648_validate_ctrl_structure(control);
    if (rc < 0) 
    {
        AS3648_ERROR("Malformed CTRL structure\n");
        return -EINVAL;
    }

    target_value = control->mode | control->output_state;
    mutex_lock(&flash_dev->mutex);

    /* Set low voltage dropout value first */
    rc = cam_as3648_register_write(flash_dev, LOW_VOLTAGE_REG, 0x0);
    if (rc < 0)
    {
        AS3648_ERROR("Error writing CTRL register\n");
        goto unlock_mutex;
    }

    rc = cam_as3648_register_write(flash_dev, CTRL_REG, target_value);
    if (rc < 0)
    {
        AS3648_ERROR("Error writing CTRL register\n");
        goto unlock_mutex;
    }

unlock_mutex:
    mutex_unlock(&flash_dev->mutex);
    return rc;
}

static int cam_as3648_get_ctrl(struct as3648_device* flash_dev,
    struct as3648_control* control)
{
    int rc = 0;
    uint8_t ctrl_reg = 0x00;

    if ((NULL == flash_dev) || (NULL == control))
    {
        AS3648_ERROR("flash device or control is NULL\n");
        return -EINVAL;
    }

    mutex_lock(&flash_dev->mutex);
    rc = cam_as3648_register_read(flash_dev, CTRL_REG, &ctrl_reg);
    if (rc < 0)
    {
        AS3648_ERROR("Error reading CTRL register\n");
        goto unlock_mutex;
    }

    control->mode = (enum as3648_mode)(ctrl_reg & CTRL_REG_MODE_MASK);
    control->output_state = (enum as3648_output_state)(ctrl_reg &
                             CTRL_REG_OUT_ON_MASK);

unlock_mutex:
    mutex_unlock(&flash_dev->mutex);
    return rc;
}

static int cam_as3648_set_flash_timeout(
    struct as3648_device*  flash_dev, uint32_t  timeout_ms)
{
    int rc = 0;
    uint8_t target_value = 0x00;

    if (NULL == flash_dev)
    {
        AS3648_ERROR("flash device is NULL\n");
        return -EINVAL;
    }

    mutex_lock(&flash_dev->mutex);
    target_value = cam_as3648_timeout_to_reg(flash_dev, timeout_ms);
    rc = cam_as3648_register_write(flash_dev, 
                                   FLASH_TIMER_REG, target_value);
    if (rc < 0)
    {
        AS3648_ERROR("Error writing register 0x%02x with value 0x%02x\n",
            FLASH_TIMER_REG, target_value);
        goto unlock_mutex;
    }

unlock_mutex:
    mutex_unlock(&flash_dev->mutex);
    return rc;
}

static int cam_as3648_get_flash_timeout(
    struct as3648_device*  flash_dev, uint32_t*  timeout_ms)
{
    int rc = 0;
    uint8_t target_value = 0x00;

    if ((NULL == flash_dev) || (NULL == timeout_ms))
    {
        AS3648_ERROR("flash device or timeout_ms is NULL\n");
        return -EINVAL;
    }

    mutex_lock(&flash_dev->mutex);
    rc = cam_as3648_register_read(flash_dev,
                                  FLASH_TIMER_REG, &target_value);
    if (rc < 0)
    {
        AS3648_ERROR("Error reading register 0x%02x\n",
                     FLASH_TIMER_REG);
        goto unlock_mutex;
    }

    *timeout_ms = cam_as3648_reg_to_timeout(target_value);

unlock_mutex:
    mutex_unlock(&flash_dev->mutex);
    return rc;
}

static void store_prj_type(int type)
{
    /* save latest timestamp and type, older will be overwriten with newer
       after store index rolling back */
    g_prj_types.prj_rec[g_prj_types.store_idx].ts_ns = ktime_get_ns();
    g_prj_types.prj_rec[g_prj_types.store_idx].type = type;

    //printk("S [ %d ] = %lld, %d\n",
    //    g_prj_types.store_idx,
    //    g_prj_types.prj_rec[g_prj_types.store_idx].ts_ns,
    //    g_prj_types.prj_rec[g_prj_types.store_idx].type);

    g_prj_types.store_idx ++;
    if (g_prj_types.store_idx >= PRJ_REC_LEN) g_prj_types.store_idx = 0;
}

u32 get_prj_type(u64 ts_ns)
{
    u32 type = 2; // means unknown
    int i;
    u64 ts_diff;

    /* timestamp diff between first frame and projector arm will be
       above 200ms depending on the sensor init time, directly return
       first record type for it */
    if (g_prj_types.first_frame_flag)
    {
        if (g_prj_types.prj_rec[0].ts_ns > 0)
            type = g_prj_types.prj_rec[0].type;

        g_prj_types.first_frame_ts_ns = ts_ns;
        g_prj_types.first_frame_flag = 0;
    }
    else
    {
        /* check if the ts close to the first frame ts */
        ts_diff = ts_ns - g_prj_types.first_frame_ts_ns;
        if (ts_diff < SYNC_FRAME_TS_DIFF)
        {
            i = 0;
            type = g_prj_types.prj_rec[0].type;
        }
        else
        {
            /* find the record entry with timestamp earlier 1~2 frame interval */
            for (i = 0; i < PRJ_REC_LEN; i ++)
            {
                if (g_prj_types.prj_rec[i].ts_ns > 0)
                {
                    ts_diff = ts_ns - g_prj_types.prj_rec[i].ts_ns;
                    if ((ts_diff > frame_intv_ns) &&
                        (ts_diff < frame_intv_twice_ns))
                    {
                        type = g_prj_types.prj_rec[i].type;
                        break;
                    }
                }
            }
        }
    }

    //printk("G < %lld > = [ %d ], %d, \n", ts_ns, i, type);
    return type;
}
EXPORT_SYMBOL(get_prj_type);

static void set_led_work_paras(struct as3648_device* pdev)
{
    struct as3648_control ctrl;

    mutex_lock(&flash_dev_switch_mutex);
    g_flash_dev_armed = pdev;
    if (g_flash_dev_armed)
    {
        cam_as3648_set_led_current(g_flash_dev_armed,
            as3648_LED_1, g_flash_dev_armed->led_1_current_ma);

        cam_as3648_set_led_current(g_flash_dev_armed,
            as3648_LED_2, g_flash_dev_armed->led_2_current_ma);

        cam_as3648_set_flash_timeout(g_flash_dev_armed,
            g_flash_dev_armed->timeout_ms);

        ctrl.mode = as3648_mode_flash;
        ctrl.output_state = as3648_output_on;
        cam_as3648_set_ctrl(g_flash_dev_armed, &ctrl);
        store_prj_type(g_flash_dev_armed->type);
    }
    mutex_unlock(&flash_dev_switch_mutex);
}

static int cam_as3648_set_arm_select(struct as3648_device*  flash_dev)
{
    frame_intv_ns = 33000000;
    frame_intv_twice_ns = frame_intv_ns * 2;

    /* reset prj type record on arm select */
    memset(&g_prj_types, 0, sizeof(g_prj_types));
    g_prj_types.first_frame_flag = 1;

    set_led_work_paras(flash_dev);
    return 0;
}

static int cam_as3648_get_fault_status(
    struct as3648_device*  flash_dev, enum as3648_fault*  faults)
{
    int rc = 0;
    uint8_t target_value = 0x00;

    if ((NULL == flash_dev) || (NULL == faults))
    {
        AS3648_ERROR("flash device or faults is NULL\n");
        return -EINVAL;
    }

    mutex_lock(&flash_dev->mutex);
    rc = cam_as3648_register_read(flash_dev, FAULT_REG, &target_value);
    if (rc < 0)
    {
        AS3648_ERROR("Error reading register 0x%02x\n", FLASH_TIMER_REG);
        goto unlock_mutex;
    }

    *faults = (enum as3648_fault) target_value;

unlock_mutex:
    mutex_unlock(&flash_dev->mutex);
    return rc;
}

static int as3648_handle_get_led_current_event(
    struct as3648_device* flash_dev,
    enum as3648_LED led, uint32_t *current_mA)
{
    int rc = 0;
    uint32_t local_current_mA = 0;

    if (current_mA == NULL)
        return -EINVAL;

    FALI_EXIT_ON_AS3648_NOT_ACQUIRED(flash_dev);
    rc = cam_as3648_get_led_current(flash_dev, led, &local_current_mA);
    if (rc < 0) 
    {
        AS3648_ERROR("Error processing get current request\n");
        return rc;
    }

    rc = copy_to_user(current_mA, &local_current_mA, sizeof(uint32_t));
    if (rc < 0) 
    {
        return -EFAULT;
    }

    return rc;
}

static int as3648_handle_set_led_current_event(
    struct as3648_device* flash_dev, enum as3648_LED led,
    uint32_t *current_mA)
{
    int rc = 0;
    uint32_t local_current_mA = 0;

    if (current_mA == NULL)
        return -EINVAL;

    rc = copy_from_user(&local_current_mA, current_mA, sizeof(uint32_t));
    if (rc < 0) 
    {
        AS3648_ERROR("Error handling set led current event, "
            "can't read current from userspace\n");
        return rc;
    }

    FALI_EXIT_ON_AS3648_NOT_ACQUIRED(flash_dev);
    rc = cam_as3648_set_led_current(flash_dev, led, local_current_mA);
    if (rc < 0)
    {
        AS3648_ERROR("Error handling set led current event, "
            "can't set current\n");
    }

    return rc;
}

static int as3648_handle_get_projector_type_event(
    struct as3648_device* flash_dev, uint32_t* type)
{
    int rc = 0;
    uint32_t local_type;

    if (type == NULL)
        return -EINVAL;

    rc = cam_as3648_get_projector_type(flash_dev, &local_type);
    if (rc < 0)
    {
        AS3648_ERROR("Error getting projector type\n");
        return rc;
    }

    return copy_to_user(type, &local_type, 
                        sizeof(enum as3648_projector_type));
}

static int as3648_handle_set_ctrl_event(
    struct as3648_device* flash_dev, struct as3648_control* ctrl)
{
    int rc = 0;
    struct as3648_control local_ctrl;

    if (ctrl == NULL)
        return -EINVAL;

    rc = copy_from_user(&local_ctrl, ctrl,
                        sizeof(struct as3648_control));
    if (rc < 0)
    {
        AS3648_ERROR("Error handling set ctrl event, "
            "can't read ctrl from userspace\n");
        return rc;
    }

    FALI_EXIT_ON_AS3648_NOT_ACQUIRED(flash_dev);
    rc = cam_as3648_set_ctrl(flash_dev, &local_ctrl);
    if (rc < 0)
    {
        AS3648_ERROR("Error handling set ctrl event, "
            "can't set ctrl\n");
    }

    return rc;
}

static int as3648_handle_get_ctrl_event(
    struct as3648_device* flash_dev, struct as3648_control* ctrl)
{
    int rc = 0;
    struct as3648_control local_ctrl;

    if (ctrl == NULL)
        return -EINVAL;

    FALI_EXIT_ON_AS3648_NOT_ACQUIRED(flash_dev);
    rc = cam_as3648_get_ctrl(flash_dev, &local_ctrl);
    if (rc < 0) 
    {
        AS3648_ERROR("Error handling get ctrl event, "
            "can't get ctrl\n");
        return rc;
    }

    rc = copy_to_user(ctrl, &local_ctrl, sizeof(struct as3648_control));
    if (rc < 0) 
    {
        AS3648_ERROR("Error handling get ctrl event, "
            "can't write ctrl to userspace\n");
    }

    return rc;
}

static int as3648_handle_get_timeout_event(
    struct as3648_device* flash_dev, uint32_t *timeout_ms)
{
    int rc = 0;
    uint32_t local_timeout_ms = 0;

    if (timeout_ms == NULL)
        return -EINVAL;

    FALI_EXIT_ON_AS3648_NOT_ACQUIRED(flash_dev);
    rc = cam_as3648_get_flash_timeout(flash_dev, &local_timeout_ms);
    if (rc < 0)
    {
        AS3648_ERROR("Error processing get current request\n");
        return rc;
    }

    rc = copy_to_user(timeout_ms, &local_timeout_ms, sizeof(uint32_t));
    if (rc < 0)
    {
        return -EFAULT;
    }

    return rc;
}

static int as3648_handle_set_timeout_event(
    struct as3648_device* flash_dev, uint32_t *timeout_ms)
{
    int rc = 0;
    uint32_t local_timeout_ms = 0;

    if (timeout_ms == NULL)
        return -EINVAL;

    rc = copy_from_user(&local_timeout_ms, timeout_ms,
                        sizeof(uint32_t));
    if (rc < 0)
    {
        AS3648_ERROR("Error handling set led current event, "
            "can't read current from userspace\n");
        return rc;
    }

    FALI_EXIT_ON_AS3648_NOT_ACQUIRED(flash_dev);
    rc = cam_as3648_set_flash_timeout(flash_dev, local_timeout_ms);
    if (rc < 0)
    {
        AS3648_ERROR("Error handling set led current event, "
            "can't set current\n");
    }

    return rc;
}

static int as3648_handle_get_max_led_current_event(
    struct as3648_device* flash_dev, uint32_t *current_mA)
{
    int rc = 0;

    if (current_mA == NULL)
        return -EINVAL;

    rc = copy_to_user(current_mA, &flash_dev->max_current_ma,
                      sizeof(uint32_t));
    if (rc < 0)
    {
        return -EFAULT;
    }

    return rc;
}

static int as3648_handle_get_max_timeout_event(
    struct as3648_device* flash_dev, uint32_t *timeout_ms)
{
    int rc = 0;

    if (timeout_ms == NULL)
        return -EINVAL;

    rc = copy_to_user(timeout_ms, &flash_dev->max_timeout_ms,
                      sizeof(uint32_t));
    if (rc < 0)
    {
        return -EFAULT;
    }

    return rc;
}

static int as3648_handle_arm_select_event(struct as3648_device* flash_dev)
{
    cam_as3648_set_arm_select(flash_dev);
    return 0;
}

static ssize_t as3648_id_show(
    struct device *dev, struct device_attribute *attr, char *buf)
{
    uint8_t id = 0;
    int rc = 0;
    struct as3648_device *flash_dev =
        (struct as3648_device*) dev_get_drvdata(dev);

    FALI_EXIT_ON_AS3648_NOT_ACQUIRED(flash_dev);

    rc = cam_as3648_get_device_identifier(flash_dev, &id);    
    if (rc < 0)
    {
        return scnprintf(buf, PAGE_SIZE, "Error getting ID\n");
    }

    return scnprintf(buf, PAGE_SIZE, "ID=0x%02x\n", id);
}

static ssize_t as3648_projector_type_show(
    struct device *dev, struct device_attribute *attr, char *buf)
{
    int rc = 0;
    struct as3648_device *flash_dev =
        (struct as3648_device*) dev_get_drvdata(dev);
    uint32_t type;

    rc = cam_as3648_get_projector_type(flash_dev, &type);
    if (rc < 0) 
    {
        return scnprintf(buf, PAGE_SIZE,
                         "Error getting projector type\n");
    }
    
    switch (type)
    {
    case as3648_dot:
        return scnprintf(buf, PAGE_SIZE, "Dot projector\n");
    case as3648_flood:
        return scnprintf(buf, PAGE_SIZE, "Flood projector\n");
    default:
        return scnprintf(buf, PAGE_SIZE, "Unknown\n");
    }
}

static ssize_t as3648_led_current_show(
    struct device *dev, struct device_attribute *attr, char *buf)
{
    int             rc = 0;
    uint32_t         current_ma = 0;
    enum as3648_LED     led = as3648_LED_1;
    struct as3648_device*    flash_dev =
        (struct as3648_device*) dev_get_drvdata(dev);

    if (!strcmp("led_current1", attr->attr.name)) 
    {
        led = as3648_LED_1;
    }
    else if (!strcmp("led_current2", attr->attr.name))
    {
        led = as3648_LED_2;
    }
    else 
    {
        return scnprintf(buf, PAGE_SIZE, "Unknown current setting\n");
    }

    FALI_EXIT_ON_AS3648_NOT_ACQUIRED(flash_dev);
    rc = cam_as3648_get_led_current(flash_dev, led, &current_ma);
    if (rc < 0)
    {
        return scnprintf(buf, PAGE_SIZE, "Error reading current\n");
    }

    return scnprintf(buf, PAGE_SIZE, "%d mA\n", current_ma);
}

static ssize_t as3648_led_current_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    int rc = 0;
    long int current_ma = 0;
    enum as3648_LED led = as3648_LED_1;
    struct as3648_device*    flash_dev =
        (struct as3648_device*) dev_get_drvdata(dev);

    rc = kstrtol(buf, 0, &current_ma);
    if (rc < 0) 
    {
        AS3648_ERROR("Error getting value for led current\n");
        return -EINVAL;
    }

    if (!strcmp("led_current1", attr->attr.name))
    {
        led = as3648_LED_1;
        flash_dev->led_1_current_ma = current_ma;
    }
    else if (!strcmp("led_current2", attr->attr.name))
    {
        led = as3648_LED_2;
        flash_dev->led_2_current_ma = current_ma;
    }
    else
    {
        AS3648_ERROR("Unknown attribute\n");
        return -EINVAL;
    }

    FALI_EXIT_ON_AS3648_NOT_ACQUIRED(flash_dev);
    rc = cam_as3648_set_led_current(flash_dev, led, current_ma);

    return count;
}

static ssize_t as3648_status_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    int rc = 0;
    enum as3648_state state;
    struct as3648_device*    flash_dev =
        (struct as3648_device*) dev_get_drvdata(dev);

    rc = cam_as3648_get_state(flash_dev, &state);
    if (rc < 0)
    {
        AS3648_ERROR("Error obtaining state\n");
        return -EINVAL;
    }

    switch (state)
    {
    case as3648_released:
        return scnprintf(buf, PAGE_SIZE, "Released\n");
    case as3648_acquired:
        return scnprintf(buf, PAGE_SIZE, "Acquired\n");
    default:
        return scnprintf(buf, PAGE_SIZE, "Unknown state\n");
    }
}

static ssize_t as3648_status_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    int rc, ret;
    const static char acquire_str[] = "acquire";
    const static char release_str[] = "release";

    struct as3648_device*    flash_dev =
        (struct as3648_device*) dev_get_drvdata(dev);

    if (!strncmp(buf, acquire_str, strlen(acquire_str)))
    {
        ret = cam_as3648_acquire(flash_dev);
        rc = (ret == 0)? count : ret;
    }
    else if (!strncmp(buf, release_str, strlen(acquire_str)))
    {
        ret = cam_as3648_release(flash_dev);
        rc = (ret == 0)? count : ret;
    }
    else
    {
        rc = -EINVAL;
    }

    return rc;
}

static ssize_t as3648_ctrl_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    int rc = 0;
    struct as3648_control ctrl;
    struct as3648_device*    flash_dev =
        (struct as3648_device*) dev_get_drvdata(dev);

    FALI_EXIT_ON_AS3648_NOT_ACQUIRED(flash_dev);
    rc = cam_as3648_get_ctrl(flash_dev, &ctrl);
    if (rc < 0)
    {
        return scnprintf(buf, PAGE_SIZE,
                        "Error obtaining ctrl register\n");
    }

    return scnprintf(buf, PAGE_SIZE, "Mode 0x%02x, out_on: %d\n",
        ctrl.mode, ctrl.output_state);
}

static ssize_t as3648_ctrl_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    const static char shutdown_str[]    = "shutdown";
    const static char indicator_str[]   = "indicator";
    const static char assist_str[]      = "assist";
    const static char flash_str[]       = "flash";
    const static char out_on_str[]      = "out_on";
    const static char out_off_str[]     = "out_off";

    struct as3648_control ctrl;
    struct as3648_device* flash_dev =
        (struct as3648_device*) dev_get_drvdata(dev);
    char *substr;

    substr = strstr(buf, ",");
    if (!substr)
    {
        AS3648_ERROR("Missing separator in as3648_ctrl_store\n");
        return -EINVAL;
    }
    substr++;

    if (!strncmp(buf, shutdown_str, strlen(shutdown_str)))
    {
        ctrl.mode = as3648_mode_shutdown;
    }
    else if (!strncmp(buf, indicator_str, strlen(indicator_str)))
    {
        ctrl.mode = as3648_mode_indicator;
    }
    else if (!strncmp(buf, assist_str, strlen(assist_str)))
    {
        ctrl.mode = as3648_mode_assist_light;
    }
    else if (!strncmp(buf, flash_str, strlen(flash_str)))
    {
        ctrl.mode = as3648_mode_flash;
    }
    else
    {
        AS3648_ERROR("Unknown mode\n");
        return -EINVAL;
    }

    if (!strncmp(substr, out_on_str, strlen(out_on_str)))
    {
        ctrl.output_state = as3648_output_on;
    }
    else if (!strncmp(substr, out_off_str, strlen(out_off_str)))
    {
        ctrl.output_state = as3648_output_off;
    }
    else
    {
        AS3648_ERROR("Unknown output state\n");
        return -EINVAL;
    }

    FALI_EXIT_ON_AS3648_NOT_ACQUIRED(flash_dev);
    if (cam_as3648_set_ctrl(flash_dev, &ctrl) < 0)
    {
        AS3648_ERROR("Error setting ctrl\n");
        return -EINVAL;
    }

    return count;
}

static ssize_t as3648_timeout_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    int rc = 0;
    uint32_t timeout_ms = 0;
    struct as3648_device* flash_dev =
        (struct as3648_device*) dev_get_drvdata(dev);

    FALI_EXIT_ON_AS3648_NOT_ACQUIRED(flash_dev);
    rc = cam_as3648_get_flash_timeout(flash_dev, &timeout_ms);
    if (rc < 0)
    {
        return scnprintf(buf, PAGE_SIZE, "Error reading timeout\n");
    }

    return scnprintf(buf, PAGE_SIZE, "%d ms\n", timeout_ms);
}

static ssize_t as3648_timeout_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    int rc = 0;
    long int timeout_ms = 0;
    struct as3648_device* flash_dev =
        (struct as3648_device*) dev_get_drvdata(dev);

    rc = kstrtol(buf, 0, &timeout_ms);
    if (rc < 0) 
    {
        AS3648_ERROR("Error getting value for flash timeout\n");
        return -EINVAL;
    }

    flash_dev->timeout_ms = timeout_ms;
    FALI_EXIT_ON_AS3648_NOT_ACQUIRED(flash_dev);
    rc = cam_as3648_set_flash_timeout(flash_dev, timeout_ms);

    return count;
}

static ssize_t as3648_max_timeout_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    struct as3648_device *flash_dev =
        (struct as3648_device*) dev_get_drvdata(dev);

    if (flash_dev == NULL)
    {
        return scnprintf(buf, PAGE_SIZE, "Error reading max timeout\n");
    }

    return scnprintf(buf, PAGE_SIZE, "%d ms\n", flash_dev->max_timeout_ms);
}

static ssize_t as3648_fault_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    int rc = 0;
    struct as3648_device *flash_dev =
        (struct as3648_device*) dev_get_drvdata(dev);
    enum as3648_fault faults = as3648_no_fault;
    ssize_t length = 0;

    FALI_EXIT_ON_AS3648_NOT_ACQUIRED(flash_dev);
    rc = cam_as3648_get_fault_status(flash_dev, &faults);
    if (rc < 0)
    {
        return scnprintf(buf, PAGE_SIZE, "Error reading fault reg\n");
    }

    length = scnprintf(buf, PAGE_SIZE, "Faults:\n");

    length = scnprintf(buf, PAGE_SIZE, "%s[%01d] Undervoltage\n", 
        buf, (faults & as3648_undervoltage_err) != 0);

    length = scnprintf(buf, PAGE_SIZE, "%s[%01d] TXMask\n",
        buf, (faults & as3648_txmask_err) != 0);

    length = scnprintf(buf, PAGE_SIZE, "%s[%01d] Timeout\n",
        buf, (faults & as3648_timeout_err) != 0);

    length = scnprintf(buf, PAGE_SIZE, "%s[%01d] Overtemp\n",
        buf, (faults & as3648_overtemp_err) != 0);

    length = scnprintf(buf, PAGE_SIZE, "%s[%01d] LED Short\n",
        buf, (faults & as3648_led_short_err) != 0);

    length = scnprintf(buf, PAGE_SIZE, "%s[%01d] Overvoltage\n",
        buf, (faults & as3648_overvolt_prot_err) != 0);

    return length;
}

static ssize_t as3648_arm_select_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    int rc = 0;
    long int selection = 0;

    struct as3648_device* flash_dev =
        (struct as3648_device*) dev_get_drvdata(dev);

    rc = kstrtol(buf, 0, &selection);
    if (rc < 0)
    {
        AS3648_ERROR("Error getting value for arm selection\n");
        return -EINVAL;
    }

    alternate_mode = 0;
    if (selection == 0)
    {
        AS3648_INFO("projector arm disabled\n");
        flash_dev = NULL;
    }
    else if (selection == 1)
    {
        AS3648_INFO("projector (%s) arm enabled \n",
            (flash_dev->type == as3648_dot) ? "dots" : "flood");
    }
    else if (selection == 2)
    {
        AS3648_INFO("alternate dots/flood arm enabled\n");
        alternate_mode = 1;
    }
    else
    {
        AS3648_ERROR("Unknow led arm selection (%ld)\n", selection);
        return -EINVAL;
    }

    cam_as3648_set_arm_select(flash_dev);
    return count;
}

static DEVICE_ATTR(status, S_IRUSR | S_IWUSR, as3648_status_show, as3648_status_store);
static DEVICE_ATTR(id, S_IRUSR, as3648_id_show, NULL);
static DEVICE_ATTR(projector_type, S_IRUSR, as3648_projector_type_show, NULL);
static DEVICE_ATTR(led_current1, S_IRUSR | S_IWUSR, as3648_led_current_show, as3648_led_current_store);
static DEVICE_ATTR(led_current2, S_IRUSR | S_IWUSR, as3648_led_current_show, as3648_led_current_store);
static DEVICE_ATTR(control, S_IRUSR | S_IWUSR, as3648_ctrl_show, as3648_ctrl_store);
static DEVICE_ATTR(timeout, S_IRUSR | S_IWUSR, as3648_timeout_show, as3648_timeout_store);
static DEVICE_ATTR(max_timeout, S_IRUSR , as3648_max_timeout_show, NULL);
static DEVICE_ATTR(fault, S_IRUSR , as3648_fault_show, NULL);
static DEVICE_ATTR(arm_select, S_IWUSR, NULL, as3648_arm_select_store);

static struct attribute *as3648_attrs[] = {
    &dev_attr_status.attr,
    &dev_attr_id.attr,
    &dev_attr_projector_type.attr,
    &dev_attr_led_current1.attr,
    &dev_attr_led_current2.attr,
    &dev_attr_control.attr,
    &dev_attr_timeout.attr,
    &dev_attr_max_timeout.attr,
    &dev_attr_fault.attr,
    &dev_attr_arm_select.attr,
    NULL
};

static struct attribute_group as3648_group = {
    .attrs = as3648_attrs
};

static const struct attribute_group *as3648_groups[] = {
    &as3648_group,
    NULL
};

static int cam_as3648_parse_dt(struct as3648_device* dev)
{
    int rc = 0;

    const char *type_str = NULL;
    struct device_node *of_node = NULL;

    if (dev == NULL) 
    {
        AS3648_ERROR("Device is NULL\n");
        return -EINVAL;
    }

    of_node = dev->of_node;

    /* property of type */
    rc = of_property_read_string(of_node, "type", &type_str);
    if (rc < 0)
    {
        AS3648_ERROR("Unspecified device type\n");
        return rc;
    }

    AS3648_INFO("Identified %s projector type\n", type_str);

    if (0 == strcmp(type_str, "dot"))
    {
        dev->type = as3648_dot;
    }
    else if (0 == strcmp(type_str, "flood"))
    {
        dev->type = as3648_flood;
    }
    else
    {
        AS3648_ERROR("Unknown device type\n");
        return -EINVAL;
    }

    /* property of type max-current-mA */
    rc = of_property_read_u32(of_node, "max-current-mA",
        &dev->max_current_ma);
    if (rc <0)
    {
        AS3648_ERROR("Cannot read maximum current from dt\n");
        return rc;
    }
    else if (dev->max_current_ma > MAX_CURRENT_MA)
    {
        dev->max_current_ma = 0;
        AS3648_ERROR("Max current is larger than the available current.\n");
        return -EINVAL;
    }

    /* property of type max-timeout-ms */
    rc = of_property_read_u32(of_node, "max-timeout-ms", 
        &dev->max_timeout_ms);
    if (rc <0)
    {
        AS3648_ERROR("Cannot read maximum timeout from dt\n");
        return rc;
    }
    else if (dev->max_timeout_ms > MAX_TIMEOUT_MS)
    {
        dev->max_timeout_ms = 0;
        AS3648_ERROR("Max timeout is larger than available in driver.\n");
        return -EINVAL;
    }

    /* property of type default-timeout-ms */
    rc = of_property_read_u32(of_node, "default-timeout-ms",
        &dev->default_timeout_ms);
    if (rc <0)
    {
        AS3648_ERROR("Cannot read maximum timeout from dt\n");
        return rc;
    }
    else if (dev->default_timeout_ms > dev->max_timeout_ms)
    {
        dev->default_timeout_ms = 0;
        AS3648_ERROR("default timeout is larger than max timeout.\n");
        return -EINVAL;
    }

    dev->timeout_ms = dev->default_timeout_ms;
    dev->led_1_current_ma = dev->max_current_ma / 10;
    dev->led_1_current_ma = dev->max_current_ma / 10;

    return rc;
}

static int as3648_dev_open(struct inode *inode, struct file *filp)
{
    int rc = 0;
    struct as3648_device *flash_dev = 
        GET_AS3648_DEVICE_FROM_CDEV(inode->i_cdev);

    rc = cam_as3648_acquire(flash_dev);
    if (rc == 0)
    {
        filp->private_data = flash_dev;
    }

    return rc;
}

static int as3648_dev_release (struct inode *inode, struct file *filp)
{
    int rc = 0;
    struct as3648_device *flash_dev = filp->private_data;

    if (flash_dev != NULL)
    {
        rc = cam_as3648_release(flash_dev);
    }

    return rc;
}

static long as3648_dev_ioctl(struct file *filp,
    unsigned int cmd, unsigned long arg)
{
    int rc = 0;
    struct as3648_device *flash_dev =
        (struct as3648_device*) filp->private_data;

    if (flash_dev == NULL)
    {
        AS3648_ERROR("Flash device is null in filp struct!\n");
        return -EINVAL;
    }

    switch (cmd)
    {
    case AS3648_GET_ILLUMINATOR_TYPE_IOCTL_CMD:
        return as3648_handle_get_projector_type_event(flash_dev,
            (uint32_t*) arg);

    case AS3648_GET_LED_CURRENT1_MA_IOCTL_CMD:
        return as3648_handle_get_led_current_event(
            flash_dev,as3648_LED_1, (uint32_t*) arg);

    case AS3648_GET_LED_CURRENT2_MA_IOCTL_CMD:
        return as3648_handle_get_led_current_event(
            flash_dev, as3648_LED_2, (uint32_t*) arg);

    case AS3648_SET_LED_CURRENT1_MA_IOCTL_CMD:
        return as3648_handle_set_led_current_event(
            flash_dev, as3648_LED_1, (uint32_t*) arg);

    case AS3648_SET_LED_CURRENT2_MA_IOCTL_CMD:
        return as3648_handle_set_led_current_event(
            flash_dev, as3648_LED_2, (uint32_t*) arg);

    case AS3648_SET_CTRL_IOCTL_CMD:
        return as3648_handle_set_ctrl_event(flash_dev, 
            (struct as3648_control*) arg);

    case AS3648_GET_CTRL_IOCTL_CMD:
        return as3648_handle_get_ctrl_event(flash_dev, 
            (struct as3648_control*) arg);

    case AS3648_SET_TIMEOUT_MS_IOCTL_CMD:
        return as3648_handle_set_timeout_event(flash_dev,
            (uint32_t*) arg);

    case AS3648_GET_TIMEOUT_MS_IOCTL_CMD:
        return as3648_handle_get_timeout_event(flash_dev,
            (uint32_t*) arg);

    case AS3648_GET_MAX_LED_CURRENT_MA_IOCTL_CMD:
        return as3648_handle_get_max_led_current_event(flash_dev,
            (uint32_t*) arg);

    case AS3648_GET_MAX_TIMEOUT_MS_IOCTL_CMD:
        return as3648_handle_get_max_timeout_event(flash_dev,
            (uint32_t*) arg);

    case AS3648_ARM_SELECT_IOCTL_CMD:
        return as3648_handle_arm_select_event(flash_dev);
    
    default:
        AS3648_ERROR("COMMAND %d not implemented", cmd);
        return -EINVAL;
    }

    return rc;
}

static const struct file_operations as3648_fops =
{
    .owner = THIS_MODULE,
    .open = as3648_dev_open,
    .release = as3648_dev_release,
    .unlocked_ioctl = as3648_dev_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = as3648_dev_ioctl,
#endif
};

static int cam_as3648_initialize_cdev(struct as3648_device* flash_dev)
{
    int rc = 0;

    cdev_init(&flash_dev->cdev, &as3648_fops);
    flash_dev->cdev.owner = THIS_MODULE;
    flash_dev->minor = ida_simple_get(&as3648_minor_ida,
        AS3648_FIRST_MINOR, AS3648_MAX_CDEV_MINORS, GFP_KERNEL);
    if (flash_dev->minor < 0)
    {
        rc = -EINVAL;
        AS3648_ERROR("Unable to obtain minor for device\n");
        goto ida_failed;
    }

    device_initialize(&flash_dev->dev);
    flash_dev->dev.parent = &flash_dev->i2c_client->dev;
    flash_dev->dev.class = as3648_class;
    flash_dev->dev.id = flash_dev->minor;
    flash_dev->dev.devt = MKDEV(as3648_major, flash_dev->minor);
    dev_set_drvdata(&flash_dev->dev, flash_dev);
    dev_set_name(&flash_dev->dev, "as3648_%d", flash_dev->minor);

    rc = cdev_add(&flash_dev->cdev, flash_dev->dev.devt, 1);
    if (rc < 0)
    {
        AS3648_ERROR("Unable to add character device\n");
        goto cdev_add_error;
    }

    rc = device_add(&flash_dev->dev);
    if (rc < 0)
    {
        AS3648_ERROR("Unable to register device\n");
        put_device(&flash_dev->dev);
        goto device_add_error;
    }

    return rc;

device_add_error:
    cdev_del(&flash_dev->cdev);
cdev_add_error:
    ida_simple_remove(&as3648_minor_ida, flash_dev->minor);
ida_failed:
    return rc;
}

static int cam_as3648_initialize_internal_registers(
    struct as3648_device* flash_dev)
{
    int rc = 0;

    FALI_EXIT_ON_AS3648_NOT_ACQUIRED(flash_dev);
    rc = cam_as3648_set_led_current(flash_dev, as3648_LED_1, 0);
    if (rc < 0)
    {
        AS3648_ERROR("Init led_current1 failed\n");
        return rc;
    }

    rc = cam_as3648_set_led_current(flash_dev, as3648_LED_2, 0);
    if (rc < 0)
    {
        AS3648_ERROR("Init led_current2 failed\n");
        return rc;
    }

    rc = cam_as3648_set_flash_timeout(flash_dev, 
        flash_dev->default_timeout_ms);
    if (rc < 0)
    {
        AS3648_ERROR("Init flash timeout failed\n");
        return rc;
    }

    return rc;
}

int do_arm_led_current(void *p)
{
    struct as3648_device* pdev;

    while (!kthread_should_stop())
    {
        /* get the sem and will sleep if not available */
        if (down_interruptible(&g_flash_arm_sem))
            return -EINTR;

        if (alternate_mode)
        {
            pdev = (g_flash_dev_armed->type == as3648_dot) ?
                g_flood_dev : g_dots_dev;
            set_led_work_paras(pdev);
        }
        else
        {
            set_led_work_paras(g_flash_dev_armed);
        }
    }

    return 0;
}

static int as3648_i2c_probe(struct i2c_client *client,
    const struct i2c_device_id *idp)
{
    int rc;
    struct device *dev = &client->dev;
    struct as3648_device *flash_dev = NULL;

    flash_dev = devm_kzalloc(dev, sizeof(*flash_dev), GFP_KERNEL);
    if (flash_dev == NULL)
    {
        rc = -ENOMEM;
        AS3648_ERROR("Error allocating as3648 device\n");
        goto flash_dev_alloc_fail;
    }

    /* data structures setup and init */
    flash_dev->i2c_client = client;
    flash_dev->of_node = dev->of_node;
    mutex_init(&flash_dev->mutex);
    flash_dev->state = as3648_released;
    i2c_set_clientdata(client, flash_dev);

    rc = cam_as3648_initialize_cdev(flash_dev);
    if (rc < 0)
    {
        AS3648_ERROR("Failed to initialize cdev\n");
        // goto free_flash_dev; //temp disable this error return
    }

    rc = cam_as3648_parse_dt(flash_dev);
    if (rc < 0)
    {
        AS3648_ERROR("Error while parsing device tree\n");
        goto free_flash_dev;
    }

    rc = cam_as3648_acquire(flash_dev);
    if (rc < 0)
    {
        AS3648_ERROR("Device acquisition failed\n");
        goto free_flash_dev;
    }

    rc = cam_as3648_identify_device(flash_dev);
    if (rc < 0)
    {
        AS3648_ERROR("Device identification failed\n");
        goto release_flash_dev;
    }

    rc = cam_as3648_initialize_internal_registers(flash_dev);
    if (rc < 0)
    {
        AS3648_ERROR("Device initialization failed\n");
        goto release_flash_dev;
    }

    rc = cam_as3648_release(flash_dev);
    if (rc < 0)
    {
        AS3648_ERROR("Error releasing flash device\n");
        goto free_flash_dev;
    }

    rc = sysfs_create_groups(&client->dev.kobj, as3648_groups);
    if (rc) 
    {
        AS3648_ERROR("Error creating sysfs attribute group.\n");
        goto release_flash_dev;
    }

    /* store the pointer of dots and flood device */
    if (flash_dev->type == as3648_dot) g_dots_dev = flash_dev;
    if (flash_dev->type == as3648_flood) g_flood_dev = flash_dev;

    return rc;

release_flash_dev:
    cam_as3648_release(flash_dev);

free_flash_dev:
    devm_kfree(dev, flash_dev);

flash_dev_alloc_fail:
    return rc;
}

static int as3648_i2c_remove(struct i2c_client *client)
{
    struct as3648_device *flash_dev = 
        (struct as3648_device *) i2c_get_clientdata(client);

    sysfs_remove_groups(&client->dev.kobj, as3648_groups);
    cam_as3648_release(flash_dev);
    ida_simple_remove(&as3648_minor_ida, flash_dev->minor);
    cdev_del(&flash_dev->cdev);
    device_del(&flash_dev->dev);
    devm_kfree(&client->dev, flash_dev);
    return 0;
}

void trigger_arm_as3648_led_current(void)
{
    if (g_flash_dev_armed)
    {
        up(&g_flash_arm_sem);
    }
}
EXPORT_SYMBOL(trigger_arm_as3648_led_current);

static const struct i2c_device_id as3648_id[] = {
    {"as3648", 0},
    {}
};

static const struct of_device_id as3648_of_match[] = {
    { .compatible = "ams,as3648" },
    {},
};

MODULE_DEVICE_TABLE(of, as3648_of_match);
MODULE_DEVICE_TABLE(i2c, as3648_id);
static struct i2c_driver as3648_i2c_driver = {
    .driver = {
        .of_match_table = of_match_ptr(as3648_of_match),
        .name = "as3648",
    },
    .probe = as3648_i2c_probe,
    .remove = as3648_i2c_remove,
    .id_table = as3648_id,
};

static int __init as3648_driver_init(void)
{
    int rc;
    dev_t as3648_first_major;
    
    AS3648_INFO("as3648 drv version: %s\n", AS3648_DRV_VERSION_STR);

    memset(&g_prj_types, 0, sizeof(g_prj_types));
    mutex_init(&flash_dev_switch_mutex);
    rc = alloc_chrdev_region(&as3648_first_major,
                             AS3648_FIRST_MINOR,
                             AS3648_MAX_CDEV_MINORS,
                             AS3648_MODULE_NAME);
    if (rc < 0)
    {
        AS3648_ERROR("can't allocate chardev region: %d\n", rc);
        goto alloc_chrdev_region_failed;
    }

    as3648_major = MAJOR(as3648_first_major);
    as3648_class = class_create(THIS_MODULE, "as3648");
    if (IS_ERR(as3648_class))
    {
        rc = PTR_ERR(as3648_class);
        pr_err("class_create failed: %d\n", rc);
        goto class_create_failed;
    }

    sema_init(&g_flash_arm_sem, 0);
    g_flash_current_arm_task = kthread_run(do_arm_led_current,
        (void *)NULL, "arm-led-current");
    if (IS_ERR(g_flash_current_arm_task))
    {
        AS3648_ERROR("Error starting  arm-led-current thread.\n");
        rc = PTR_ERR(g_flash_current_arm_task);
        goto start_thread_failed;
    }

    rc = i2c_add_driver(&as3648_i2c_driver);
    if (rc)
    {
        AS3648_ERROR("i2c_add_driver failed rc = %d", rc);
        goto add_driver_failed;
    }

    AS3648_INFO("as3648 driver init sucessfully");
    return 0;

add_driver_failed:
    (void)kthread_stop(g_flash_current_arm_task);
start_thread_failed:
    class_destroy(as3648_class);
class_create_failed:
    unregister_chrdev_region(MKDEV(as3648_major, AS3648_FIRST_MINOR),
                             AS3648_MAX_CDEV_MINORS);
alloc_chrdev_region_failed:
    return rc;
}

static void __init as3648_driver_exit(void)
{
    AS3648_INFO("Removing as3648\n");
    (void)kthread_stop(g_flash_current_arm_task);
    i2c_del_driver(&as3648_i2c_driver);
    class_destroy(as3648_class);
    unregister_chrdev_region(MKDEV(as3648_major, AS3648_FIRST_MINOR),
                             AS3648_MAX_CDEV_MINORS);
    g_flash_dev_armed = NULL;
    g_dots_dev = NULL;
    g_flood_dev = NULL;
    AS3648_INFO("as3648 i2c driver removed\n");
}

module_init(as3648_driver_init);
module_exit(as3648_driver_exit);

MODULE_DESCRIPTION("as3648 Flash Driver Module");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Javier Alvarez,xulong re-orgnize");
MODULE_VERSION(AS3648_DRV_VERSION_STR);
