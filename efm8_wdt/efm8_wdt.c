#include <linux/device.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#define WATCHDOG_REGISTER      0x20
#define WATCHDOG_DISABLE       0x0000
#define WATCHDOG_ENABLE        0x0001
#define WATCHDOG_SECONDS_SHIFT 8

struct efm8_wdt_data {
    struct i2c_client *i2c_client;
    struct device_node *of_node;
    int wdt_expire_s;
    int64_t wdt_kick_ns;
    int wdt_gpio;
    int wdt_gpio_state;
    enum hrtimer_restart timer_state;
    struct hrtimer hr_timer;
};

enum hrtimer_restart timer_trigger_func(struct hrtimer *timer)
{
    struct efm8_wdt_data *wdt_data = container_of(timer, struct efm8_wdt_data, hr_timer);
    hrtimer_forward(timer, hrtimer_cb_get_time(timer), ktime_set(0, wdt_data->wdt_kick_ns));

    if (0 == wdt_data->wdt_gpio_state) {
        gpio_set_value(wdt_data->wdt_gpio, 1);
        wdt_data->wdt_gpio_state = 1;
    } else {
        gpio_set_value(wdt_data->wdt_gpio, 0);
        wdt_data->wdt_gpio_state = 0;
    }

    return wdt_data->timer_state;
}

static int efm8_wdt_i2c_write_word(struct i2c_client *client,
                                    const char addr,
                                    uint16_t data)
{
    return i2c_smbus_write_word_data(client, addr, data);
}

static int efm8_wdt_init(struct i2c_client *client)
{
    ktime_t kt;
    int err;
    struct efm8_wdt_data *wdt_data = (struct efm8_wdt_data *)i2c_get_clientdata(client);

    if (gpio_is_valid(wdt_data->wdt_gpio)) {
        err = devm_gpio_request_one(&client->dev, wdt_data->wdt_gpio, GPIOF_OUT_INIT_LOW, "oclea-wdt trigger");
        if (err < 0) {
            dev_err(&client->dev,
                    "failed to request GPIO %d, error %d\n",
                    wdt_data->wdt_gpio, err);
            return err;
        }
    } else {
        dev_err(&client->dev, "Invalid gpio %d, error: %d\n", wdt_data->wdt_gpio, err);
        devm_gpio_free(&client->dev, wdt_data->wdt_gpio);
        return err;
    }

    err = efm8_wdt_i2c_write_word(client, WATCHDOG_REGISTER, WATCHDOG_ENABLE | (wdt_data->wdt_expire_s << WATCHDOG_SECONDS_SHIFT));
    if (err < 0) {
        dev_err(&client->dev, "Failed to start watchdog, error: %d\n", err);
        devm_gpio_free(&client->dev, wdt_data->wdt_gpio);
        return err;
    }

    gpio_set_value(wdt_data->wdt_gpio, 1);
    wdt_data->wdt_gpio_state = 1;

    wdt_data->timer_state = HRTIMER_RESTART;

    kt = ktime_set(0, wdt_data->wdt_kick_ns);
    hrtimer_init(&wdt_data->hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
    hrtimer_set_expires(&wdt_data->hr_timer, kt);
    wdt_data->hr_timer.function = &timer_trigger_func;
    hrtimer_start(&wdt_data->hr_timer, kt, HRTIMER_MODE_ABS);

    return 0;
}

static int efm8_wdt_i2c_probe(struct i2c_client *client,
                             const struct i2c_device_id *id)
{
    struct device *dev = &client->dev;
    const __be32 *prop;
    struct efm8_wdt_data *wdt_data;

    wdt_data = devm_kzalloc(dev, sizeof(*wdt_data), GFP_KERNEL);
    if (!wdt_data) {
        return -ENOMEM;
    };

    wdt_data->i2c_client = client;
    wdt_data->of_node = dev->of_node;
    i2c_set_clientdata(client, wdt_data);

    prop = of_get_property(wdt_data->of_node, "wdt-timeout-s", NULL);
    if (!prop) {
        wdt_data->wdt_expire_s = 64;
    } else {
        wdt_data->wdt_expire_s = be32_to_cpup(prop);
        if (wdt_data->wdt_expire_s < 8 || wdt_data->wdt_expire_s > 255) {
            return -EINVAL;
        }
    }

    prop = of_get_property(wdt_data->of_node, "wdt-kick-time-s", NULL);
    if (!prop) {
        wdt_data->wdt_kick_ns = (int64_t)16 * (1000 * 1000 * 1000);
    } else {
        wdt_data->wdt_kick_ns = (int64_t)be32_to_cpup(prop);
        if (wdt_data->wdt_kick_ns < 1 || wdt_data->wdt_kick_ns > 255) {
            return -EINVAL;
        }

        wdt_data->wdt_kick_ns = wdt_data->wdt_kick_ns * (1000 * 1000 * 1000);
    }

    wdt_data->wdt_gpio = of_get_named_gpio(wdt_data->of_node, "wdt-gpio", 0);
    if (wdt_data->wdt_gpio < 0) {
        return -EINVAL;
    }

    return efm8_wdt_init(client);
}

static int efm8_wdt_i2c_remove(struct i2c_client *client)
{
    struct efm8_wdt_data *wdt_data = (struct efm8_wdt_data *)i2c_get_clientdata(client);
    hrtimer_cancel(&wdt_data->hr_timer);
    wdt_data->timer_state = HRTIMER_NORESTART;

    return efm8_wdt_i2c_write_word(client, WATCHDOG_REGISTER, WATCHDOG_DISABLE);
}

static struct i2c_device_id efm8_wdt_i2c_idtable[] = {
      { "efm8-wdt", 0 },
      { },
};
MODULE_DEVICE_TABLE(i2c, efm8_wdt_i2c_idtable);

static const struct of_device_id efm8_wdt_i2c_of_match[] = {
    {
        .compatible = "oclea,efm8_wdt",
        .data = &efm8_wdt_i2c_idtable[0]
    },
    {}
};
MODULE_DEVICE_TABLE(of, efm8_wdt_i2c_of_match);

static struct i2c_driver efm8_wdt_i2c_driver = {
      .driver = {
              .name   = "efm8-wdt",
              .owner  = THIS_MODULE,
              .of_match_table = of_match_ptr(efm8_wdt_i2c_of_match),
      },

      .id_table       = efm8_wdt_i2c_idtable,
      .probe          = efm8_wdt_i2c_probe,
      .remove         = efm8_wdt_i2c_remove,
};

static int __init efm8_wdt_i2c_init(void)
{
    return i2c_add_driver(&efm8_wdt_i2c_driver);
}

static void __exit efm8_wdt_i2c_exit(void)
{
    return i2c_del_driver(&efm8_wdt_i2c_driver);
}

module_init(efm8_wdt_i2c_init);
module_exit(efm8_wdt_i2c_exit);

MODULE_AUTHOR("Caleb J <caleb@teknique.com>");
MODULE_DESCRIPTION("Driver for Oclea EFM8 Watchdog");
MODULE_LICENSE("GPL");
