#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/sysfs.h>

#define LED_MODE         0x01
#define RED_BRIGHTNESS   0x10
#define GREEN_BRIGHTNESS 0x11
#define BLUE_BRIGHTNESS  0x12
#define LED_DELAY        0x20
#define BLINK_ON_TIME    0x30
#define BLINK_OFF_TIME   0x31
#define APPLY_SETTINGS   0xA0

#define CHECK_INPUT() {                   \
    unsigned int i_val;                   \
    if (sscanf(buf, "%d", &i_val) != 1) { \
        return -EINVAL;                   \
    }                                     \
                                          \
    if (i_val < 0 || i_val > 255) {       \
        return -EINVAL;                   \
    }                                     \
                                          \
    c_val = (unsigned char)i_val;         \
}

static struct i2c_client *ledring_i2c_client;

static void i2c_init(void)
{
    printk("LED Ring initialised");
}

static ssize_t ledring_i2c_red_brightness_write(struct device *dev,
                                                struct device_attribute *attr,
                                                const char *buf,
                                                size_t size)
{
    unsigned char c_val;
    CHECK_INPUT();
    i2c_smbus_write_byte_data(ledring_i2c_client, RED_BRIGHTNESS, c_val);
    return size;
}

static struct device_attribute attr_red_brightness = {
    .attr = {
        .name = "red_brightness",
        .mode = S_IWUSR,
    },
    .store = ledring_i2c_red_brightness_write,
};

static ssize_t ledring_i2c_green_brightness_write(struct device *dev,
                                                  struct device_attribute *attr,
                                                  const char *buf,
                                                  size_t size)
{
    unsigned char c_val;
    CHECK_INPUT();
    i2c_smbus_write_byte_data(ledring_i2c_client, GREEN_BRIGHTNESS, c_val);
    return size;
}

static struct device_attribute attr_green_brightness = {
    .attr = {
        .name = "green_brightness",
        .mode = S_IWUSR,
    },
    .store = ledring_i2c_green_brightness_write,
};

static ssize_t ledring_i2c_blue_brightness_write(struct device *dev,
                                                 struct device_attribute *attr,
                                                 const char *buf,
                                                 size_t size)
{
    unsigned char c_val;
    CHECK_INPUT();
    i2c_smbus_write_byte_data(ledring_i2c_client, BLUE_BRIGHTNESS, c_val);
    return size;
}

static struct device_attribute attr_blue_brightness = {
    .attr = {
        .name = "blue_brightness",
        .mode = S_IWUSR,
    },
    .store = ledring_i2c_blue_brightness_write,
};

static ssize_t ledring_i2c_led_update_delay_write(struct device *dev,
                                                  struct device_attribute *attr,
                                                  const char *buf,
                                                  size_t size)
{
    unsigned char c_val;
    CHECK_INPUT();
    i2c_smbus_write_byte_data(ledring_i2c_client, LED_DELAY, c_val);
    return size;
}

static struct device_attribute attr_led_update_delay = {
    .attr = {
        .name = "led_update_delay",
        .mode = S_IWUSR,
    },
    .store = ledring_i2c_led_update_delay_write,
};

static ssize_t ledring_i2c_blink_on_time_write(struct device *dev,
                                               struct device_attribute *attr,
                                               const char *buf,
                                               size_t size)
{
    unsigned char c_val;
    CHECK_INPUT();
    i2c_smbus_write_byte_data(ledring_i2c_client, BLINK_ON_TIME, c_val);
    return size;
}

static struct device_attribute attr_blink_on_time = {
    .attr = {
        .name = "blink_on_time",
        .mode = S_IWUSR,
    },
    .store = ledring_i2c_blink_on_time_write,
};

static ssize_t ledring_i2c_blink_off_time_write(struct device *dev,
                                                struct device_attribute *attr,
                                                const char *buf,
                                                size_t size)
{
    unsigned char c_val;
    CHECK_INPUT();
    i2c_smbus_write_byte_data(ledring_i2c_client, BLINK_OFF_TIME, c_val);
    return size;
}

static struct device_attribute attr_blink_off_time = {
    .attr = {
        .name = "blink_off_time",
        .mode = S_IWUSR,
    },
    .store = ledring_i2c_blink_off_time_write,
};

static ssize_t ledring_i2c_led_mode_write(struct device *dev,
                                          struct device_attribute *attr,
                                          const char *buf,
                                          size_t size)
{
    unsigned char c_val;
    CHECK_INPUT();
    i2c_smbus_write_byte_data(ledring_i2c_client, LED_MODE, c_val);
    return size;
}

static struct device_attribute attr_led_mode = {
    .attr = {
        .name = "led_mode",
        .mode = S_IWUSR,
    },
    .store = ledring_i2c_led_mode_write,
};

static ssize_t ledring_i2c_apply_settings_write(struct device *dev,
                                                struct device_attribute *attr,
                                                const char *buf,
                                                size_t size)
{
    unsigned char c_val;
    CHECK_INPUT();
    i2c_smbus_write_byte_data(ledring_i2c_client, APPLY_SETTINGS, c_val);
    return size;
}

static struct device_attribute attr_apply_settings = {
    .attr = {
        .name = "apply_settings",
        .mode = S_IWUSR,
    },
    .store = ledring_i2c_apply_settings_write,
};

static int ledring_i2c_probe(struct i2c_client *client,
                             const struct i2c_device_id *id)
{
    int ret;

    ledring_i2c_client = client;

    ret = device_create_file(&client->dev, &attr_red_brightness);
    if (ret < 0) {
        printk("Error creating sysfs entry: %d\n", ret);
    }

    ret = device_create_file(&client->dev, &attr_green_brightness);
    if (ret < 0) {
        printk("Error creating sysfs entry: %d\n", ret);
    }

    ret = device_create_file(&client->dev, &attr_blue_brightness);
    if (ret < 0) {
        printk("Error creating sysfs entry: %d\n", ret);
    }

    ret = device_create_file(&client->dev, &attr_led_update_delay);
    if (ret < 0) {
        printk("Error creating sysfs entry: %d\n", ret);
    }

    ret = device_create_file(&client->dev, &attr_blink_on_time);
    if (ret < 0) {
        printk("Error creating sysfs entry: %d\n", ret);
    }

    ret = device_create_file(&client->dev, &attr_blink_off_time);
    if (ret < 0) {
        printk("Error creating sysfs entry: %d\n", ret);
    }

    ret = device_create_file(&client->dev, &attr_led_mode);
    if (ret < 0) {
        printk("Error creating sysfs entry: %d\n", ret);
    }

    ret = device_create_file(&client->dev, &attr_apply_settings);
    if (ret < 0) {
        printk("Error creating sysfs entry: %d\n", ret);
    }

    i2c_init();

    return ret;
}

static int ledring_i2c_remove(struct i2c_client *client)
{
    device_remove_file(&client->dev, &attr_red_brightness);
    device_remove_file(&client->dev, &attr_green_brightness);
    device_remove_file(&client->dev, &attr_blue_brightness);
    device_remove_file(&client->dev, &attr_led_update_delay);
    device_remove_file(&client->dev, &attr_blink_on_time);
    device_remove_file(&client->dev, &attr_blink_off_time);
    device_remove_file(&client->dev, &attr_led_mode);
    device_remove_file(&client->dev, &attr_apply_settings);

    return 0;
}

static struct i2c_device_id ledring_i2c_idtable[] = {
      { "ledring_i2c", 0 },
      { },
};
MODULE_DEVICE_TABLE(i2c, ledring_i2c_idtable);

static const struct of_device_id ledring_i2c_of_match[] = {
    {
        .compatible = "ledring_i2c",
        .data = &ledring_i2c_idtable[0]
    },
    {}
};
MODULE_DEVICE_TABLE(of, ledring_i2c_of_match);

static struct i2c_driver ledring_i2c_driver = {
      .driver = {
              .name   = "ledring_i2c",
              .owner  = THIS_MODULE,
              .of_match_table = of_match_ptr(ledring_i2c_of_match),
      },

      .id_table       = ledring_i2c_idtable,
      .probe          = ledring_i2c_probe,
      .remove         = ledring_i2c_remove,
};

static int __init ledring_i2c_init(void)
{
    return i2c_add_driver(&ledring_i2c_driver);
}

static void __exit ledring_i2c_exit(void)
{
    return i2c_del_driver(&ledring_i2c_driver);
}

module_init(ledring_i2c_init);
module_exit(ledring_i2c_exit);

MODULE_AUTHOR("Caleb J <caleb@teknique.com>");
MODULE_DESCRIPTION("Driver for i2c LED ring");
MODULE_LICENSE("GPL");
