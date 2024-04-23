/* SPDX-License-Identifier: GPL-2.0-only */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/mutex.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/iio/iio.h>
#include <linux/iio/types.h>
#include <linux/iio/sysfs.h>
#include <linux/version.h>

#define PYD1698_DETECT_IRQ_NAME "PIR Detect IRQ"

#define PYD1698_TCLK_US 4       // clock time > 2 us
#define PYD1698_SIN_TSLT_US 680 // data load time > 580 us
#define PYD1698_SIN_TSHD_US 76  // data in hold time > 72 us

#define PYD1698_DL_TDS_FRC_US 120   // data set-up time > 110 ms for force mode
#define PYD1698_DL_TDS_WU_US 80     // data set-up time > 75 ms for intr mode
#define PYD1698_DL_TBS_US 10        // data bit settling time 2 us under Cload < 10pF
#define PYD1698_DL_TBIT_US 22       // bit time < 22 us
#define PYD1698_DL_TCL_US 145       // readout cancel time > 145 us
#define PYD1698_DL_TRST_US 70       // > 35 ms

#define PYD1698_THRESHOLD_DEFAULT 50    // [24:17] (0 .. 255)
#define PYD1698_BLIND_TIME_DEFAULT 15   // [16:13] 0.5s + [Reg Val] * 0.5s - 8 seconds
#define PYD1698_PULSE_COUNT_DEFAULT 1   // [12:11] 1 + [Reg Val] - 2 pulses to trigger
#define PYD1698_WINDOW_TIME_DEFAULT 0   // [10:9] 2s + [Reg Val] * 2s - 2 second window for pulse triggers
#define PYD1698_OP_MODE_DEFAULT 2       // [8:7] 0 Force, 1 Interrupt, 2 Wakeup
#define PYD1698_FILTER_SOURCE_DEFAULT 0 // [6:5] Filter source: 0: PIR (BPF) 1: PIR(LPF) 2: Reserved 3: Temperature sensor
#define PYD1698_RESERVED_1 2            // [4:3] must be 2
#define PYD1698_HPF_CUT_OFF_DEFAULT 0   // [2] 0: 0.4 Hz 1: 0.2 Hz
#define PYD1698_RESERVED_0 0            // [1] must be 0
#define PYD1698_PD_MODE_DEAULT 0        // [0] 0: with BPF sign change 1: without
#define PYD1698_RESET_TIME_MS_DEFAULT 1000

#define PYD1698_CFG_REG_BIT_LEN 25
#define PYD1698_DATA_REG_BIT_LEN 40

typedef union pyd1698_cfg_reg {
    uint32_t value;
    struct pyd1698_cfg_reg_bits {
        uint32_t pd_mode : 1;
        uint32_t rsrv_0 : 1;
        uint32_t hpf_co : 1;
        uint32_t rsrv_1 : 2;
        uint32_t sig_src : 2;
        uint32_t op_mode : 2;
        uint32_t win_time : 2;
        uint32_t pls_cnt : 2;
        uint32_t blind_time : 4;
        uint32_t threshold : 8;
    } msb_fields;
} PYD1698_CFG_REG;

typedef union pyd1698_data_reg {
    uint64_t value;
    struct pyd1698_data_reg_bits {
        uint64_t pd_mode : 1;
        uint64_t rsrv_0 : 1;
        uint64_t hpf_co : 1;
        uint64_t rsrv_1 : 2;
        uint64_t sig_src : 2;
        uint64_t op_mode : 2;
        uint64_t win_time : 2;
        uint64_t pls_cnt : 2;
        uint64_t blind_time : 4;
        uint64_t threshold : 8;
        uint64_t adc_cnt : 14;
        uint64_t out_of_range : 1;
    } msb_fields;
} PYD1698_DATA_REG;

struct pyd1698_data {
    const char *dev_name;
    int sin_gpio;
    int dl_gpio;
    PYD1698_DATA_REG data_reg;
    PYD1698_CFG_REG cfg_reg;
    int detected;
    struct mutex lock;
    int irq_number;
    struct timer_list rst_timer;
    int rst_time_ms;
};

static IIO_CONST_ATTR(threshold_available, "0 .. 255 detection threshold on BPF value");
static IIO_CONST_ATTR(blind_time_available, "0.5s + [Reg Val] * 0.5s");
static IIO_CONST_ATTR(pulse_count_available, "1 + [Reg Val]");
static IIO_CONST_ATTR(window_time_available, "2s + [Reg Val] * 2s");

static struct attribute *pyd1698_attributes[] = {
    &iio_const_attr_threshold_available.dev_attr.attr,
    &iio_const_attr_blind_time_available.dev_attr.attr,
    &iio_const_attr_pulse_count_available.dev_attr.attr,
    &iio_const_attr_window_time_available.dev_attr.attr,
    NULL
};

static const struct attribute_group pyd1698_attribute_group = {
    .attrs = pyd1698_attributes,
};

/*
 * Write config to PYD1698 serial in
 */
static int pyd1698_set_config(struct pyd1698_data *data)
{
    int bit_count = PYD1698_CFG_REG_BIT_LEN;

    mutex_lock(&data->lock);
    gpio_direction_output(data->sin_gpio, 0);
    gpio_set_value(data->sin_gpio, 0);
    udelay(PYD1698_SIN_TSLT_US); // Hold low for 16 PIR clock cycles before starting transfer

    while (bit_count)
    {
        bit_count--;
        gpio_set_value(data->sin_gpio, 0);
        udelay(PYD1698_TCLK_US);
        gpio_set_value(data->sin_gpio, 1);
        udelay(PYD1698_TCLK_US);
        if ((data->cfg_reg.value >> bit_count) & 0x01)
        {
            // Pin is already in correct bit state
        }
        else
        {
            gpio_set_value(data->sin_gpio, 0);
        }
        udelay(PYD1698_SIN_TSHD_US); // Sleep at least two PIR clock cycles (72uS)
    }

    gpio_set_value(data->sin_gpio, 0);
    udelay(PYD1698_SIN_TSLT_US); // Sleep at least 16 PIR clock cycles (580uS)

    mutex_unlock(&data->lock);
    return 0;
}

/*
 * Debug only
 */
static void pyd1698_data_debug_print(struct pyd1698_data *data)
{
    printk("%s:%d: out of range [%d]\n", __func__, __LINE__, data->data_reg.msb_fields.out_of_range);
    printk("%s:%d: adc counts [%d]\n", __func__, __LINE__, data->data_reg.msb_fields.adc_cnt);
    printk("%s:%d: threshold [%d]\n", __func__, __LINE__, data->data_reg.msb_fields.threshold);
    printk("%s:%d: pulse count [%d]\n", __func__, __LINE__, data->data_reg.msb_fields.pls_cnt);
    printk("%s:%d: op mode [%d]\n", __func__, __LINE__, data->data_reg.msb_fields.op_mode);
    printk("%s:%d: blind time [%d]\n", __func__, __LINE__, data->data_reg.msb_fields.blind_time);
    printk("%s:%d: win time [%d]\n", __func__, __LINE__, data->data_reg.msb_fields.win_time);
    printk("%s:%d: signal source [%d]\n", __func__, __LINE__, data->data_reg.msb_fields.sig_src);
    printk("%s:%d: hpf cut off [%d]\n", __func__, __LINE__, data->data_reg.msb_fields.hpf_co);
    printk("%s:%d: pd mode [%d]\n", __func__, __LINE__, data->data_reg.msb_fields.pd_mode);
}

/*
 * Read out data bytes in wake up mode, called after start condition
 */
static int pyd1698_read_out(struct pyd1698_data *data)
{
    int bit_count = PYD1698_DATA_REG_BIT_LEN;

    mutex_lock(&data->lock);

    while (bit_count)
    {
        bit_count--;
        gpio_direction_output(data->dl_gpio, 0);
        gpio_set_value(data->dl_gpio, 0);
        udelay(PYD1698_TCLK_US);
        gpio_set_value(data->dl_gpio, 1);
        udelay(PYD1698_TCLK_US);
        gpio_direction_input(data->dl_gpio);
        udelay(PYD1698_DL_TBS_US);
        if (gpio_get_value(data->dl_gpio))
        {
            data->data_reg.value |= 1 << bit_count;
        }
        else
        {
            data->data_reg.value &= ~(1 << bit_count);
        }
    }

    data->cfg_reg.msb_fields.threshold = data->data_reg.msb_fields.threshold;
    data->cfg_reg.msb_fields.pls_cnt = data->data_reg.msb_fields.pls_cnt;
    data->cfg_reg.msb_fields.op_mode = data->data_reg.msb_fields.op_mode;
    data->cfg_reg.msb_fields.blind_time = data->data_reg.msb_fields.blind_time;
    data->cfg_reg.msb_fields.win_time = data->data_reg.msb_fields.win_time;
    data->cfg_reg.msb_fields.sig_src = data->data_reg.msb_fields.sig_src;
    data->cfg_reg.msb_fields.hpf_co = data->data_reg.msb_fields.hpf_co;
    data->cfg_reg.msb_fields.pd_mode = data->data_reg.msb_fields.pd_mode;

    mutex_unlock(&data->lock);
    return 0;
}

/*
 * Direct Link line reset timer handler
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
static void pyd1698_rst_timer_func(unsigned long p_data)
{
    struct pyd1698_data *data = (struct pyd1698_data *) p_data;
#else
static void pyd1698_rst_timer_func(struct timer_list *t)
{
    struct pyd1698_data *data = from_timer(data, t, rst_timer);
#endif

    gpio_direction_output(data->dl_gpio, 0);
    gpio_set_value(data->dl_gpio, 0);
    udelay(PYD1698_DL_TRST_US);
    mutex_lock(&data->lock);
    data->detected = 0;
    mutex_unlock(&data->lock);
    gpio_direction_input(data->dl_gpio);
}

/*
 * Direct Link Line interrupt handler
 */
static irqreturn_t pyd1698_gpio_interrupt_handler(int irq, void *irq_data)
{
    struct pyd1698_data *data = irq_data;

    mutex_lock(&data->lock);
    data->detected = 1;
    mutex_unlock(&data->lock);

    mod_timer(&data->rst_timer, jiffies + data->rst_time_ms * (HZ / 1000));
    return IRQ_HANDLED;
}

static ssize_t pyd1698_read_threshold(struct iio_dev *indio_dev, uintptr_t private,
    const struct iio_chan_spec *chan, char *buf)
{
    struct pyd1698_data *data = iio_priv(indio_dev);
    return snprintf(buf, PAGE_SIZE, "%d\n", data->cfg_reg.msb_fields.threshold);
}

static ssize_t pyd1698_write_threshold(struct iio_dev *indio_dev, uintptr_t private,
    const struct iio_chan_spec *chan, const char *buf, size_t len)
{
    struct pyd1698_data *data = iio_priv(indio_dev);
    unsigned int threshold;
    int ret;

    ret = kstrtouint(buf, 0, &threshold);
    if (ret)
        return ret;

    mutex_lock(&data->lock);
    data->cfg_reg.msb_fields.threshold = threshold;
    mutex_unlock(&data->lock);
    pyd1698_set_config(data);

    return len;
}

static ssize_t pyd1698_read_pulse_count(struct iio_dev *indio_dev, uintptr_t private,
    const struct iio_chan_spec *chan, char *buf)
{
    struct pyd1698_data *data = iio_priv(indio_dev);
    return snprintf(buf, PAGE_SIZE, "%d\n", data->cfg_reg.msb_fields.pls_cnt);
}

static ssize_t pyd1698_write_pulse_count(struct iio_dev *indio_dev, uintptr_t private,
    const struct iio_chan_spec *chan, const char *buf, size_t len)
{
    struct pyd1698_data *data = iio_priv(indio_dev);
    unsigned int pulse_count;
    int ret;

    ret = kstrtouint(buf, 0, &pulse_count);
    if (ret)
        return ret;

    mutex_lock(&data->lock);
    data->cfg_reg.msb_fields.pls_cnt = pulse_count;
    mutex_unlock(&data->lock);
    pyd1698_set_config(data);

    return len;
}

static ssize_t pyd1698_read_blind_time(struct iio_dev *indio_dev, uintptr_t private,
    const struct iio_chan_spec *chan, char *buf)
{
    struct pyd1698_data *data = iio_priv(indio_dev);
    return snprintf(buf, PAGE_SIZE, "%d\n", data->cfg_reg.msb_fields.blind_time);
}

static ssize_t pyd1698_write_blind_time(struct iio_dev *indio_dev, uintptr_t private,
    const struct iio_chan_spec *chan, const char *buf, size_t len)
{
    struct pyd1698_data *data = iio_priv(indio_dev);
    unsigned int blind_time;
    int ret;

    ret = kstrtouint(buf, 0, &blind_time);
    if (ret)
        return ret;

    mutex_lock(&data->lock);
    data->cfg_reg.msb_fields.blind_time = blind_time;
    mutex_unlock(&data->lock);
    pyd1698_set_config(data);

    return len;
}

static ssize_t pyd1698_read_window_time(struct iio_dev *indio_dev, uintptr_t private,
    const struct iio_chan_spec *chan, char *buf)
{
    struct pyd1698_data *data = iio_priv(indio_dev);
    return snprintf(buf, PAGE_SIZE, "%d\n", data->cfg_reg.msb_fields.win_time);
}

static ssize_t pyd1698_write_window_time(struct iio_dev *indio_dev, uintptr_t private,
    const struct iio_chan_spec *chan, const char *buf, size_t len)
{
    struct pyd1698_data *data = iio_priv(indio_dev);
    unsigned int window_time;
    int ret;

    ret = kstrtouint(buf, 0, &window_time);
    if (ret)
        return ret;

    mutex_lock(&data->lock);
    data->cfg_reg.msb_fields.win_time = window_time;
    mutex_unlock(&data->lock);
    pyd1698_set_config(data);

    return len;
}

static ssize_t pyd1698_read_reset_time(struct iio_dev *indio_dev, uintptr_t private,
    const struct iio_chan_spec *chan, char *buf)
{
    struct pyd1698_data *data = iio_priv(indio_dev);
    return snprintf(buf, PAGE_SIZE, "%d\n", data->rst_time_ms);
}

static ssize_t pyd1698_write_reset_time(struct iio_dev *indio_dev, uintptr_t private,
    const struct iio_chan_spec *chan, const char *buf, size_t len)
{
    struct pyd1698_data *data = iio_priv(indio_dev);
    unsigned int reset_time;
    int ret;

    ret = kstrtouint(buf, 0, &reset_time);
    if (ret)
        return ret;

    mutex_lock(&data->lock);
    data->rst_time_ms = reset_time;
    mutex_unlock(&data->lock);

    return len;
}

static ssize_t pyd1698_read_detect(struct iio_dev *indio_dev, uintptr_t private,
    const struct iio_chan_spec *chan, char *buf)
{
    struct pyd1698_data *data = iio_priv(indio_dev);
    return snprintf(buf, PAGE_SIZE, "%d\n", data->detected);
}

static const struct iio_chan_spec_ext_info pyd1698_threshold_ext_info[] = {
    {
        .name = "threshold",
        .shared = IIO_SHARED_BY_DIR,
        .read = pyd1698_read_threshold,
        .write = pyd1698_write_threshold
    },
    {}
};

static const struct iio_chan_spec_ext_info pyd1698_blind_time_ext_info[] = {
    {
        .name = "blind_time",
        .shared = IIO_SHARED_BY_DIR,
        .read = pyd1698_read_blind_time,
        .write = pyd1698_write_blind_time
    },
    {}
};

static const struct iio_chan_spec_ext_info pyd1698_pulse_count_ext_info[] = {
    {
        .name = "pulse_count",
        .shared = IIO_SHARED_BY_DIR,
        .read = pyd1698_read_pulse_count,
        .write = pyd1698_write_pulse_count
    },
    {}
};

static const struct iio_chan_spec_ext_info pyd1698_window_time_ext_info[] = {
    {
        .name = "window_time",
        .shared = IIO_SHARED_BY_DIR,
        .read = pyd1698_read_window_time,
        .write = pyd1698_write_window_time
    },
    {}
};

static const struct iio_chan_spec_ext_info pyd1698_reset_time_ext_info[] = {
    {
        .name = "reset_time",
        .shared = IIO_SHARED_BY_DIR,
        .read = pyd1698_read_reset_time,
        .write = pyd1698_write_reset_time
    },
    {}
};

static const struct iio_chan_spec_ext_info pyd1698_detect_ext_info[] = {
    {
        .name = "detect",
        .shared = IIO_SHARED_BY_DIR,
        .read = pyd1698_read_detect,
    },
    {}
};

static const struct iio_chan_spec pyd1698_channels[] = {
    {
        .type = IIO_PIR,
        .indexed = 1,
        .channel = 0,
        .ext_info = pyd1698_detect_ext_info,
    },
    {
        .type = IIO_PIR,
        .indexed = 1,
        .channel = 1,
        .ext_info = pyd1698_threshold_ext_info,
        .output = 1,
    },
    {
        .type = IIO_PIR,
        .indexed = 1,
        .channel = 2,
        .ext_info = pyd1698_blind_time_ext_info,
        .output = 1,
    },
    {
        .type = IIO_PIR,
        .indexed = 1,
        .channel = 3,
        .ext_info = pyd1698_pulse_count_ext_info,
        .output = 1,
    },
    {
        .type = IIO_PIR,
        .indexed = 1,
        .channel = 4,
        .ext_info = pyd1698_window_time_ext_info,
        .output = 1,
    },
    {
        .type = IIO_PIR,
        .indexed = 1,
        .channel = 5,
        .ext_info = pyd1698_reset_time_ext_info,
        .output = 1,
    }
};

static const struct iio_info pyd1698_info = {
    .attrs = &pyd1698_attribute_group,
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
    .driver_module = THIS_MODULE,
#endif
};

#ifdef CONFIG_OF
static int pyd1698_parse_dt(struct device *dev, struct pyd1698_data *data)
{
    struct device_node *node = dev->of_node;
    enum of_gpio_flags flags;

    if (!node)
        return -ENODEV;

    data->sin_gpio = of_get_named_gpio_flags(node, "sin-gpio", 0, &flags);
    data->dl_gpio = of_get_named_gpio_flags(node, "dl-gpio", 0, &flags);

    if (!gpio_is_valid(data->sin_gpio) || !gpio_is_valid(data->dl_gpio))
        return -EINVAL;

    if (of_property_read_string(node, "dev-name", &data->dev_name) != 0)
        data->dev_name = NULL;

    return 0;
}

static const struct of_device_id pyd1698_of_match[] = {
    { .compatible = "oclea,pyd1698" },
    { }
};

MODULE_DEVICE_TABLE(of, pyd1698_of_match);
#else
static int pyd1698_parse_dt(struct device *dev, struct pyd1698_data *data)
{
    return -ENODEV;
}
#endif

static int pyd1698_probe(struct platform_device *pdev)
{
    struct pyd1698_data *data;
    struct iio_dev *indio_dev = NULL;
    int rval;

    indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(struct pyd1698_data));
    if (!indio_dev) {
        dev_err(&pdev->dev, "failed to allocate iio device\n");
        return -ENOMEM;
    }

    data = iio_priv(indio_dev);

    platform_set_drvdata(pdev, indio_dev);

    rval = pyd1698_parse_dt(&pdev->dev, data);
    if (rval < 0) {
        dev_err(&pdev->dev, "failed to parse dt %d!\n", rval);
        return -EINVAL;
    }

    if (data->dev_name)
        dev_set_name(&indio_dev->dev, data->dev_name);
    else
        dev_set_name(&indio_dev->dev, "iio:pyd1698");

    data->cfg_reg.value = 0;
    data->cfg_reg.msb_fields.threshold = PYD1698_THRESHOLD_DEFAULT;
    data->cfg_reg.msb_fields.blind_time = PYD1698_BLIND_TIME_DEFAULT;
    data->cfg_reg.msb_fields.pls_cnt = PYD1698_PULSE_COUNT_DEFAULT;
    data->cfg_reg.msb_fields.win_time = PYD1698_WINDOW_TIME_DEFAULT;
    data->cfg_reg.msb_fields.op_mode = PYD1698_OP_MODE_DEFAULT;
    data->cfg_reg.msb_fields.sig_src = PYD1698_FILTER_SOURCE_DEFAULT;
    data->cfg_reg.msb_fields.rsrv_1 = PYD1698_RESERVED_1;
    data->cfg_reg.msb_fields.hpf_co = PYD1698_HPF_CUT_OFF_DEFAULT; 
    data->cfg_reg.msb_fields.rsrv_0 = PYD1698_RESERVED_0;
    data->cfg_reg.msb_fields.pd_mode = PYD1698_PD_MODE_DEAULT;
    data->rst_time_ms = PYD1698_RESET_TIME_MS_DEFAULT;

    data->detected = 0;
    mutex_init(&data->lock);

    pyd1698_set_config(data);

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
    init_timer(&data->rst_timer);
    data->rst_timer.function = pyd1698_rst_timer_func;
    data->rst_timer.data = (unsigned long) data;
#else
    timer_setup(&data->rst_timer, pyd1698_rst_timer_func, 0);
#endif
    data->rst_timer.expires = jiffies + HZ / 1000;

    data->irq_number = gpio_to_irq(data->dl_gpio);
    rval = request_irq(data->irq_number, &pyd1698_gpio_interrupt_handler,
        IRQF_TRIGGER_RISING, PYD1698_DETECT_IRQ_NAME, data);

    if (rval < 0)
        return -EINVAL;

    indio_dev->name = dev_name(&pdev->dev);
    indio_dev->dev.parent = &pdev->dev;
    indio_dev->dev.of_node = pdev->dev.of_node;
    indio_dev->info = &pyd1698_info;
    indio_dev->channels = pyd1698_channels;
    indio_dev->num_channels = ARRAY_SIZE(pyd1698_channels);
    indio_dev->modes = INDIO_DIRECT_MODE;

    rval = iio_device_register(indio_dev);
    if (rval < 0)
    {
        dev_err(&pdev->dev, "failed to register iio device %d!\n", rval);
        return -ENXIO;
    }

    dev_info(&pdev->dev, "%d channels\n", indio_dev->num_channels);
    add_timer(&data->rst_timer);
    return 0;
}

static int pyd1698_remove(struct platform_device *pdev)
{
    struct iio_dev *indio_dev = platform_get_drvdata(pdev);
    struct pyd1698_data *data = iio_priv(indio_dev);

    free_irq(data->irq_number, data);
    del_timer(&data->rst_timer);
    iio_device_unregister(platform_get_drvdata(pdev));
    iio_device_free(indio_dev);
    return 0;
}

static const struct platform_device_id pyd1698_ids[] = {
    {"pyd1698", 0},
    {}
};

MODULE_DEVICE_TABLE(platform, pyd1698_ids);

static struct platform_driver pyd1698_driver = {
    .probe = pyd1698_probe,
    .remove = pyd1698_remove,
    .id_table = pyd1698_ids,
    .driver = {
        .name = "pyd1698",
        .of_match_table = of_match_ptr(pyd1698_of_match),
    },
};

module_platform_driver(pyd1698_driver);

MODULE_AUTHOR("Gym Ok Cho <kimogi@teknique.com>");
MODULE_DESCRIPTION("PYD1698 PIR sensor driver");
MODULE_LICENSE("GPL");
