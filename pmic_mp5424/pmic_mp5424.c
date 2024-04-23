/* SPDX-License-Identifier: GPL-2.0-only */

#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/sysfs.h>

#define PM_CTL0      0x00
#define PM_BUCK1_1   0x04
#define PM_BUCK2_1   0x08
#define PM_BUCK3_1   0x0C
#define PM_BUCK4_1   0x10
#define PM_CTL3      0x13
#define PM_ID2       0x29

static struct i2c_client *pmic_mp5424_client;

static ssize_t pmic_mp5424_info(struct device *dev,
                                struct device_attribute *attr,
                                char *buf)
{
    unsigned char v_ctl0, v_buck1_1, v_buck2_1, v_buck3_1, v_buck4_1, v_ctl3;

    v_ctl0 = i2c_smbus_read_byte_data(pmic_mp5424_client, PM_CTL0);
    v_buck1_1 = i2c_smbus_read_byte_data(pmic_mp5424_client, PM_BUCK1_1);
    v_buck2_1 = i2c_smbus_read_byte_data(pmic_mp5424_client, PM_BUCK2_1);
    v_buck3_1 = i2c_smbus_read_byte_data(pmic_mp5424_client, PM_BUCK3_1);
    v_buck4_1 = i2c_smbus_read_byte_data(pmic_mp5424_client, PM_BUCK4_1);
    v_ctl3 = i2c_smbus_read_byte_data(pmic_mp5424_client, PM_CTL3);
    return snprintf(buf, PAGE_SIZE,
             "Slew rate: %s\n"
             "Frequency: %s\n"
             "Power on debounce timer: %s\n"
             "Buck 1 discharge mode: %s\n"
             "Buck 1 PWM mode: %s\n"
             "Buck 2 discharge mode: %s\n"
             "Buck 2 PWM mode: %s\n"
             "Buck 3 discharge mode: %s\n"
             "Buck 3 PWM mode: %s\n"
             "Buck 4 discharge mode: %s\n"
             "Buck 4 PWM mode: %s\n"
             "Power down mode: %s\n",

             /* Slew rate */
             ((v_ctl0 & 0b11000000) == 0b11000000) ? "4mV/us" :
             ((v_ctl0 & 0b11000000) == 0b10000000) ? "8mV/us" :
             "Invalid",

             /* Frequency */
             ((v_ctl0 & 0b00110000) == 0b00000000) ? "1.1Mhz" :
             ((v_ctl0 & 0b00110000) == 0b00010000) ? "1.65Mhz" :
             ((v_ctl0 & 0b00110000) == 0b00100000) ? "2.2Mhz" :
             "2.75Mhz",

             /* Power on debounce timer */
             ((v_ctl0 & 0b00000110) == 0b00000000) ? "0.5ms" :
             ((v_ctl0 & 0b00000110) == 0b00000010) ? "10ms" :
             ((v_ctl0 & 0b00000110) == 0b00000100) ? "40ms" :
             "160ms",

             /* Buck 1 discharge mode */
             ((v_buck1_1 & 0b00100000) == 0b00100000) ? "Enabled" : "Disabled",
             /* Buck 1 PWM mode */
             ((v_buck1_1 & 0b00010000) == 0b00010000) ? "Forced PWM" : "Auto PFM/PWM",

             /* Buck 2 discharge mode */
             ((v_buck2_1 & 0b00100000) == 0b00100000) ? "Enabled" : "Disabled",
             /* Buck 2 PWM mode */
             ((v_buck2_1 & 0b00010000) == 0b00010000) ? "Forced PWM" : "Auto PFM/PWM",

             /* Buck 3 discharge mode */
             ((v_buck3_1 & 0b00100000) == 0b00100000) ? "Enabled" : "Disabled",
             /* Buck 3 PWM mode */
             ((v_buck3_1 & 0b00010000) == 0b00010000) ? "Forced PWM" : "Auto PFM/PWM",

             /* Buck 4 discharge mode */
             ((v_buck4_1 & 0b00100000) == 0b00100000) ? "Enabled" : "Disabled",
             /* Buck 4 PWM mode */
             ((v_buck4_1 & 0b00010000) == 0b00010000) ? "Forced PWM" : "Auto PFM/PWM",

             /* Power down mode */
             ((v_ctl3 & 0b00001000) == 0b00001000) ? "Instant" : "Sequenced"

             );
}

static struct device_attribute attr_info = {
    .attr = {
        .name = "status",
        .mode = S_IRUGO,
    },
    .show = pmic_mp5424_info,
    .store = NULL,
};

static ssize_t pmic_mp5424_slewrate_read(struct device *dev,
                                         struct device_attribute *attr,
                                         char *buf)
{
    unsigned char v_ctl0 = i2c_smbus_read_byte_data(pmic_mp5424_client, PM_CTL0);
    return snprintf(buf, PAGE_SIZE, "%s\nAllowed values: 4, 8\n",
                    ((v_ctl0 & 0b11000000) == 0b11000000) ? "4mV/us" :
                    ((v_ctl0 & 0b11000000) == 0b10000000) ? "8mV/us" :
                    "Invalid");
}

static ssize_t pmic_mp5424_slewrate_write(struct device *dev,
                                          struct device_attribute *attr,
                                          const char *buf,
                                          size_t size)
{
    int ret, val;
    unsigned char v_ctl0 = i2c_smbus_read_byte_data(pmic_mp5424_client, PM_CTL0);

    if ((ret = sscanf(buf, "%d", &val)) != 1) {
        return -EINVAL;
    }

    v_ctl0 &= 0b00111111;
    if (4 == val) {
        v_ctl0 |= 0b11000000;
        i2c_smbus_write_byte_data(pmic_mp5424_client, PM_CTL0, v_ctl0);
        return size;
    } else if (8 == val) {
        v_ctl0 |= 0b10000000;
        i2c_smbus_write_byte_data(pmic_mp5424_client, PM_CTL0, v_ctl0);
        return size;
    }

    return -EINVAL;
}

static struct device_attribute attr_slewrate = {
    .attr = {
        .name = "slew_rate",
        .mode = S_IRUGO | S_IWUSR,
    },
    .show = pmic_mp5424_slewrate_read,
    .store = pmic_mp5424_slewrate_write,
};

static ssize_t pmic_mp5424_pon_debounce_read(struct device *dev,
                                             struct device_attribute *attr,
                                             char *buf)
{
    unsigned char v_ctl0 = i2c_smbus_read_byte_data(pmic_mp5424_client, PM_CTL0);
    return snprintf(buf, PAGE_SIZE, "%s\nAllowed values: 0.5, 10, 40, 160\n",
                    ((v_ctl0 & 0b00000110) == 0b00000000) ? "0.5ms" :
                    ((v_ctl0 & 0b00000110) == 0b00000010) ? "10ms" :
                    ((v_ctl0 & 0b00000110) == 0b00000100) ? "40ms" :
                    "160ms");
}

static ssize_t pmic_mp5424_pon_debounce_write(struct device *dev,
                                              struct device_attribute *attr,
                                              const char *buf,
                                              size_t size)
{
    int ret, val1, val2;
    unsigned char v_ctl0 = i2c_smbus_read_byte_data(pmic_mp5424_client, PM_CTL0);

    ret = sscanf(buf, "%d.%d", &val1, &val2);

    if (ret != 1 && ret != 2) {
        return -EINVAL;
    }

    v_ctl0 &= 0b11111001;
    if (0 == val1 && 5 == val2) {
        i2c_smbus_write_byte_data(pmic_mp5424_client, PM_CTL0, v_ctl0);
        return size;
    } else if (10 == val1) {
        v_ctl0 |= 0b00000010;
        i2c_smbus_write_byte_data(pmic_mp5424_client, PM_CTL0, v_ctl0);
        return size;
    } else if (40 == val1) {
        v_ctl0 |= 0b00000100;
        i2c_smbus_write_byte_data(pmic_mp5424_client, PM_CTL0, v_ctl0);
        return size;
    } else if (160 == val1) {
        v_ctl0 |= 0b00000110;
        i2c_smbus_write_byte_data(pmic_mp5424_client, PM_CTL0, v_ctl0);
        return size;
    }

    return -EINVAL;
}

static struct device_attribute attr_pon_debounce = {
    .attr = {
        .name = "power_on_debounce",
        .mode = S_IRUGO | S_IWUSR,
    },
    .show = pmic_mp5424_pon_debounce_read,
    .store = pmic_mp5424_pon_debounce_write,
};

static ssize_t pmic_mp5424_frequency_read(struct device *dev,
                                          struct device_attribute *attr,
                                          char *buf)
{
    unsigned char v_ctl0 = i2c_smbus_read_byte_data(pmic_mp5424_client, PM_CTL0);
    return snprintf(buf, PAGE_SIZE, "%s\nAllowed values:1.1, 1.65, 2.2, 2.75\n",
                    ((v_ctl0 & 0b00110000) == 0b00000000) ? "1.1Mhz" :
                    ((v_ctl0 & 0b00110000) == 0b00010000) ? "1.65Mhz" :
                    ((v_ctl0 & 0b00110000) == 0b00100000) ? "2.2Mhz" :
                    "2.75Mhz");
}

static ssize_t pmic_mp5424_frequency_write(struct device *dev,
                                           struct device_attribute *attr,
                                           const char *buf,
                                           size_t size)
{
    int ret, val1, val2;
    unsigned char v_ctl0 = i2c_smbus_read_byte_data(pmic_mp5424_client, PM_CTL0);

    if ((ret = sscanf(buf, "%d.%d", &val1, &val2)) != 2) {
        return -EINVAL;
    }

    v_ctl0 &= 0b11001111;
    if (1 == val1 && 1 == val2) {
        i2c_smbus_write_byte_data(pmic_mp5424_client, PM_CTL0, v_ctl0);
        return size;
    } else if (1 == val1 && 65 == val2) {
        v_ctl0 |= 0b00010000;
        i2c_smbus_write_byte_data(pmic_mp5424_client, PM_CTL0, v_ctl0);
        return size;
    } else if (2 == val1 && 2 == val2) {
        v_ctl0 |= 0b00100000;
        i2c_smbus_write_byte_data(pmic_mp5424_client, PM_CTL0, v_ctl0);
        return size;
    } else if (2 == val1 && 75 == val2) {
        v_ctl0 |= 0b00110000;
        i2c_smbus_write_byte_data(pmic_mp5424_client, PM_CTL0, v_ctl0);
        return size;
    }

    return -EINVAL;
}

static struct device_attribute attr_frequency = {
    .attr = {
        .name = "frequency",
        .mode = S_IRUGO | S_IWUSR,
    },
    .show = pmic_mp5424_frequency_read,
    .store = pmic_mp5424_frequency_write,
};

static ssize_t pmic_mp5424_buck1_discharge_read(struct device *dev,
                                          struct device_attribute *attr,
                                          char *buf)
{
    unsigned char v_buck1_1 = i2c_smbus_read_byte_data(pmic_mp5424_client, PM_BUCK1_1);
    return snprintf(buf, PAGE_SIZE, "%s\nAllowed values: Enabled, Disabled\n",
                    ((v_buck1_1 & 0b00100000) == 0b00100000) ? "Enabled" : "Disabled");
}

static ssize_t pmic_mp5424_buck1_discharge_write(struct device *dev,
                                           struct device_attribute *attr,
                                           const char *buf,
                                           size_t size)
{
    unsigned char v_buck1_1 = i2c_smbus_read_byte_data(pmic_mp5424_client, PM_BUCK1_1);

    v_buck1_1 &= 0b11011111;
    if (strcmp("Enabled", buf) == 0 || strcmp("Enabled\n", buf) == 0) {
        v_buck1_1 |= 0b00100000;
        i2c_smbus_write_byte_data(pmic_mp5424_client, PM_BUCK1_1, v_buck1_1);
        return size;
    } else if (strcmp("Disabled", buf) == 0 || strcmp("Disabled\n", buf) == 0) {
        i2c_smbus_write_byte_data(pmic_mp5424_client, PM_BUCK1_1, v_buck1_1);
        return size;
    }

    return -EINVAL;
}

static struct device_attribute attr_buck1_discharge = {
    .attr = {
        .name = "buck1_discharge",
        .mode = S_IRUGO | S_IWUSR,
    },
    .show = pmic_mp5424_buck1_discharge_read,
    .store = pmic_mp5424_buck1_discharge_write,
};

static ssize_t pmic_mp5424_buck2_discharge_read(struct device *dev,
                                          struct device_attribute *attr,
                                          char *buf)
{
    unsigned char v_buck2_1 = i2c_smbus_read_byte_data(pmic_mp5424_client, PM_BUCK2_1);
    return snprintf(buf, PAGE_SIZE, "%s\nAllowed values: Enabled, Disabled\n",
                    ((v_buck2_1 & 0b00100000) == 0b00100000) ? "Enabled" : "Disabled");
}

static ssize_t pmic_mp5424_buck2_discharge_write(struct device *dev,
                                           struct device_attribute *attr,
                                           const char *buf,
                                           size_t size)
{
    unsigned char v_buck2_1 = i2c_smbus_read_byte_data(pmic_mp5424_client, PM_BUCK2_1);

    v_buck2_1 &= 0b11011111;
    if (strcmp("Enabled", buf) == 0 || strcmp("Enabled\n", buf) == 0) {
        v_buck2_1 |= 0b00100000;
        i2c_smbus_write_byte_data(pmic_mp5424_client, PM_BUCK2_1, v_buck2_1);
        return size;
    } else if (strcmp("Disabled", buf) == 0 || strcmp("Disabled\n", buf) == 0) {
        i2c_smbus_write_byte_data(pmic_mp5424_client, PM_BUCK2_1, v_buck2_1);
        return size;
    }

    return -EINVAL;
}

static struct device_attribute attr_buck2_discharge = {
    .attr = {
        .name = "buck2_discharge",
        .mode = S_IRUGO | S_IWUSR,
    },
    .show = pmic_mp5424_buck2_discharge_read,
    .store = pmic_mp5424_buck2_discharge_write,
};

static ssize_t pmic_mp5424_buck3_discharge_read(struct device *dev,
                                          struct device_attribute *attr,
                                          char *buf)
{
    unsigned char v_buck3_1 = i2c_smbus_read_byte_data(pmic_mp5424_client, PM_BUCK3_1);
    return snprintf(buf, PAGE_SIZE, "%s\nAllowed values: Enabled, Disabled\n",
                    ((v_buck3_1 & 0b00100000) == 0b00100000) ? "Enabled" : "Disabled");
}

static ssize_t pmic_mp5424_buck3_discharge_write(struct device *dev,
                                           struct device_attribute *attr,
                                           const char *buf,
                                           size_t size)
{
    unsigned char v_buck3_1 = i2c_smbus_read_byte_data(pmic_mp5424_client, PM_BUCK3_1);

    v_buck3_1 &= 0b11011111;
    if (strcmp("Enabled", buf) == 0 || strcmp("Enabled\n", buf) == 0) {
        v_buck3_1 |= 0b00100000;
        i2c_smbus_write_byte_data(pmic_mp5424_client, PM_BUCK3_1, v_buck3_1);
        return size;
    } else if (strcmp("Disabled", buf) == 0 || strcmp("Disabled\n", buf) == 0) {
        i2c_smbus_write_byte_data(pmic_mp5424_client, PM_BUCK3_1, v_buck3_1);
        return size;
    }

    return -EINVAL;
}

static struct device_attribute attr_buck3_discharge = {
    .attr = {
        .name = "buck3_discharge",
        .mode = S_IRUGO | S_IWUSR,
    },
    .show = pmic_mp5424_buck3_discharge_read,
    .store = pmic_mp5424_buck3_discharge_write,
};

static ssize_t pmic_mp5424_buck4_discharge_read(struct device *dev,
                                          struct device_attribute *attr,
                                          char *buf)
{
    unsigned char v_buck4_1 = i2c_smbus_read_byte_data(pmic_mp5424_client, PM_BUCK4_1);
    return snprintf(buf, PAGE_SIZE, "%s\nAllowed values: Enabled, Disabled\n",
                    ((v_buck4_1 & 0b00100000) == 0b00100000) ? "Enabled" : "Disabled");
}

static ssize_t pmic_mp5424_buck4_discharge_write(struct device *dev,
                                           struct device_attribute *attr,
                                           const char *buf,
                                           size_t size)
{
    unsigned char v_buck4_1 = i2c_smbus_read_byte_data(pmic_mp5424_client, PM_BUCK4_1);

    v_buck4_1 &= 0b11011111;
    if (strcmp("Enabled", buf) == 0 || strcmp("Enabled\n", buf) == 0) {
        v_buck4_1 |= 0b00100000;
        i2c_smbus_write_byte_data(pmic_mp5424_client, PM_BUCK4_1, v_buck4_1);
        return size;
    } else if (strcmp("Disabled", buf) == 0 || strcmp("Disabled\n", buf) == 0) {
        i2c_smbus_write_byte_data(pmic_mp5424_client, PM_BUCK4_1, v_buck4_1);
        return size;
    }

    return -EINVAL;
}

static struct device_attribute attr_buck4_discharge = {
    .attr = {
        .name = "buck4_discharge",
        .mode = S_IRUGO | S_IWUSR,
    },
    .show = pmic_mp5424_buck4_discharge_read,
    .store = pmic_mp5424_buck4_discharge_write,
};

static ssize_t pmic_mp5424_buck1_pwm_read(struct device *dev,
                                          struct device_attribute *attr,
                                          char *buf)
{
    unsigned char v_buck1_1 = i2c_smbus_read_byte_data(pmic_mp5424_client, PM_BUCK1_1);
    return snprintf(buf, PAGE_SIZE, "%s\nAllowed values: Forced PWM, Auto PFM/PWM\n",
                    ((v_buck1_1 & 0b00010000) == 0b00010000) ? "Forced PWM" : "Auto PFM/PWM");
}

static ssize_t pmic_mp5424_buck1_pwm_write(struct device *dev,
                                           struct device_attribute *attr,
                                           const char *buf,
                                           size_t size)
{
    unsigned char v_buck1_1 = i2c_smbus_read_byte_data(pmic_mp5424_client, PM_BUCK1_1);

    v_buck1_1 &= 0b11101111;
    if (strcmp("Forced PWM", buf) == 0 || strcmp("Forced PWM\n", buf) == 0) {
        v_buck1_1 |= 0b00010000;
        i2c_smbus_write_byte_data(pmic_mp5424_client, PM_BUCK1_1, v_buck1_1);
        return size;
    } else if (strcmp("Auto PFM/PWM", buf) == 0 || strcmp("Auto PFM/PWM\n", buf) == 0) {
        i2c_smbus_write_byte_data(pmic_mp5424_client, PM_BUCK1_1, v_buck1_1);
        return size;
    }

    return -EINVAL;
}

static struct device_attribute attr_buck1_pwm = {
    .attr = {
        .name = "buck1_pwm",
        .mode = S_IRUGO | S_IWUSR,
    },
    .show = pmic_mp5424_buck1_pwm_read,
    .store = pmic_mp5424_buck1_pwm_write,
};

static ssize_t pmic_mp5424_buck2_pwm_read(struct device *dev,
                                          struct device_attribute *attr,
                                          char *buf)
{
    unsigned char v_buck2_1 = i2c_smbus_read_byte_data(pmic_mp5424_client, PM_BUCK2_1);
    return snprintf(buf, PAGE_SIZE, "%s\nAllowed values: Forced PWM, Auto PFM/PWM\n",
                    ((v_buck2_1 & 0b00010000) == 0b00010000) ? "Forced PWM" : "Auto PFM/PWM");
}

static ssize_t pmic_mp5424_buck2_pwm_write(struct device *dev,
                                           struct device_attribute *attr,
                                           const char *buf,
                                           size_t size)
{
    unsigned char v_buck2_1 = i2c_smbus_read_byte_data(pmic_mp5424_client, PM_BUCK2_1);

    v_buck2_1 &= 0b11101111;
    if (strcmp("Forced PWM", buf) == 0 || strcmp("Forced PWM\n", buf) == 0) {
        v_buck2_1 |= 0b00010000;
        i2c_smbus_write_byte_data(pmic_mp5424_client, PM_BUCK2_1, v_buck2_1);
        return size;
    } else if (strcmp("Auto PFM/PWM", buf) == 0 || strcmp("Auto PFM/PWM\n", buf) == 0) {
        i2c_smbus_write_byte_data(pmic_mp5424_client, PM_BUCK2_1, v_buck2_1);
        return size;
    }

    return -EINVAL;
}

static struct device_attribute attr_buck2_pwm = {
    .attr = {
        .name = "buck2_pwm",
        .mode = S_IRUGO | S_IWUSR,
    },
    .show = pmic_mp5424_buck2_pwm_read,
    .store = pmic_mp5424_buck2_pwm_write,
};

static ssize_t pmic_mp5424_buck3_pwm_read(struct device *dev,
                                          struct device_attribute *attr,
                                          char *buf)
{
    unsigned char v_buck3_1 = i2c_smbus_read_byte_data(pmic_mp5424_client, PM_BUCK3_1);
    return snprintf(buf, PAGE_SIZE, "%s\nAllowed values: Forced PWM, Auto PFM/PWM\n",
                    ((v_buck3_1 & 0b00010000) == 0b00010000) ? "Forced PWM" : "Auto PFM/PWM");
}

static ssize_t pmic_mp5424_buck3_pwm_write(struct device *dev,
                                           struct device_attribute *attr,
                                           const char *buf,
                                           size_t size)
{
    unsigned char v_buck3_1 = i2c_smbus_read_byte_data(pmic_mp5424_client, PM_BUCK3_1);

    v_buck3_1 &= 0b11101111;
    if (strcmp("Forced PWM", buf) == 0 || strcmp("Forced PWM\n", buf) == 0) {
        v_buck3_1 |= 0b00010000;
        i2c_smbus_write_byte_data(pmic_mp5424_client, PM_BUCK3_1, v_buck3_1);
        return size;
    } else if (strcmp("Auto PFM/PWM", buf) == 0 || strcmp("Auto PFM/PWM\n", buf) == 0) {
        i2c_smbus_write_byte_data(pmic_mp5424_client, PM_BUCK3_1, v_buck3_1);
        return size;
    }

    return -EINVAL;
}

static struct device_attribute attr_buck3_pwm = {
    .attr = {
        .name = "buck3_pwm",
        .mode = S_IRUGO | S_IWUSR,
    },
    .show = pmic_mp5424_buck3_pwm_read,
    .store = pmic_mp5424_buck3_pwm_write,
};

static ssize_t pmic_mp5424_buck4_pwm_read(struct device *dev,
                                          struct device_attribute *attr,
                                          char *buf)
{
    unsigned char v_buck4_1 = i2c_smbus_read_byte_data(pmic_mp5424_client, PM_BUCK4_1);
    return snprintf(buf, PAGE_SIZE, "%s\nAllowed values: Forced PWM, Auto PFM/PWM\n",
                    ((v_buck4_1 & 0b00010000) == 0b00010000) ? "Forced PWM" : "Auto PFM/PWM");
}

static ssize_t pmic_mp5424_buck4_pwm_write(struct device *dev,
                                           struct device_attribute *attr,
                                           const char *buf,
                                           size_t size)
{
    unsigned char v_buck4_1 = i2c_smbus_read_byte_data(pmic_mp5424_client, PM_BUCK4_1);

    v_buck4_1 &= 0b11101111;
    if (strcmp("Forced PWM", buf) == 0 || strcmp("Forced PWM\n", buf) == 0) {
        v_buck4_1 |= 0b00010000;
        i2c_smbus_write_byte_data(pmic_mp5424_client, PM_BUCK4_1, v_buck4_1);
        return size;
    } else if (strcmp("Auto PFM/PWM", buf) == 0 || strcmp("Auto PFM/PWM\n", buf) == 0) {
        i2c_smbus_write_byte_data(pmic_mp5424_client, PM_BUCK4_1, v_buck4_1);
        return size;
    }

    return -EINVAL;
}

static struct device_attribute attr_buck4_pwm = {
    .attr = {
        .name = "buck4_pwm",
        .mode = S_IRUGO | S_IWUSR,
    },
    .show = pmic_mp5424_buck4_pwm_read,
    .store = pmic_mp5424_buck4_pwm_write,
};

static ssize_t pmic_mp5424_pdown_mode_read(struct device *dev,
                                           struct device_attribute *attr,
                                           char *buf)
{
    unsigned char v_ctl3 = i2c_smbus_read_byte_data(pmic_mp5424_client, PM_CTL3);
    return snprintf(buf, PAGE_SIZE, "%s\nAllowed values: Instant, Sequenced\n",
                    ((v_ctl3 & 0b00001000) == 0b00001000) ? "Instant" :
                    "Sequenced");
}

static ssize_t pmic_mp5424_pdown_mode_write(struct device *dev,
                                            struct device_attribute *attr,
                                            const char *buf,
                                            size_t size)
{
    unsigned char v_ctl3 = i2c_smbus_read_byte_data(pmic_mp5424_client, PM_CTL3);

    v_ctl3 &= 0b11110111;
    if (strcmp("Instant", buf) == 0 || strcmp("Instant\n", buf) == 0) {
        v_ctl3 |= 0b00001000;
        i2c_smbus_write_byte_data(pmic_mp5424_client, PM_CTL3, v_ctl3);
        return size;
    } else if (strcmp("Sequenced", buf) == 0 || strcmp("Sequenced\n", buf) == 0) {
        i2c_smbus_write_byte_data(pmic_mp5424_client, PM_CTL3, v_ctl3);
        return size;
    }

    return -EINVAL;
}

static struct device_attribute attr_powerdown_mode = {
    .attr = {
        .name = "powerdown_mode",
        .mode = S_IRUGO | S_IWUSR,
    },
    .show = pmic_mp5424_pdown_mode_read,
    .store = pmic_mp5424_pdown_mode_write,
};

static void i2c_init(void)
{
    unsigned char whoami;
    whoami = i2c_smbus_read_byte_data(pmic_mp5424_client, PM_ID2);
    printk("Pmic mp5424 Vendor ID: %hhd\n", (whoami >> 4) & 0xF);
}

static int pmic_mp5424_probe(struct i2c_client *client,
                             const struct i2c_device_id *id)
{
    int ret;

    pmic_mp5424_client = client;

    ret = device_create_file(&client->dev, &attr_info);
    if (ret < 0) {
        printk("Error creating sysfs entry: %d\n", ret);
    }

    ret = device_create_file(&client->dev, &attr_slewrate);
    if (ret < 0) {
        printk("Error creating sysfs entry: %d\n", ret);
    }

    ret = device_create_file(&client->dev, &attr_frequency);
    if (ret < 0) {
        printk("Error creating sysfs entry: %d\n", ret);
    }

    ret = device_create_file(&client->dev, &attr_pon_debounce);
    if (ret < 0) {
        printk("Error creating sysfs entry: %d\n", ret);
    }

    ret = device_create_file(&client->dev, &attr_buck1_discharge);
    if (ret < 0) {
        printk("Error creating sysfs entry: %d\n", ret);
    }

    ret = device_create_file(&client->dev, &attr_buck2_discharge);
    if (ret < 0) {
        printk("Error creating sysfs entry: %d\n", ret);
    }

    ret = device_create_file(&client->dev, &attr_buck3_discharge);
    if (ret < 0) {
        printk("Error creating sysfs entry: %d\n", ret);
    }

    ret = device_create_file(&client->dev, &attr_buck4_discharge);
    if (ret < 0) {
        printk("Error creating sysfs entry: %d\n", ret);
    }

    ret = device_create_file(&client->dev, &attr_buck1_pwm);
    if (ret < 0) {
        printk("Error creating sysfs entry: %d\n", ret);
    }

    ret = device_create_file(&client->dev, &attr_buck2_pwm);
    if (ret < 0) {
        printk("Error creating sysfs entry: %d\n", ret);
    }

    ret = device_create_file(&client->dev, &attr_buck3_pwm);
    if (ret < 0) {
        printk("Error creating sysfs entry: %d\n", ret);
    }

    ret = device_create_file(&client->dev, &attr_buck4_pwm);
    if (ret < 0) {
        printk("Error creating sysfs entry: %d\n", ret);
    }

    ret = device_create_file(&client->dev, &attr_powerdown_mode);
    if (ret < 0) {
        printk("Error creating sysfs entry: %d\n", ret);
    }

    i2c_init();

    return ret;
}

static int pmic_mp5424_remove(struct i2c_client *client)
{
    device_remove_file(&client->dev, &attr_info);
    device_remove_file(&client->dev, &attr_slewrate);
    device_remove_file(&client->dev, &attr_frequency);
    device_remove_file(&client->dev, &attr_pon_debounce);
    device_remove_file(&client->dev, &attr_buck1_discharge);
    device_remove_file(&client->dev, &attr_buck2_discharge);
    device_remove_file(&client->dev, &attr_buck3_discharge);
    device_remove_file(&client->dev, &attr_buck4_discharge);
    device_remove_file(&client->dev, &attr_buck1_pwm);
    device_remove_file(&client->dev, &attr_buck2_pwm);
    device_remove_file(&client->dev, &attr_buck3_pwm);
    device_remove_file(&client->dev, &attr_buck4_pwm);
    device_remove_file(&client->dev, &attr_powerdown_mode);

    return 0;
}

static struct i2c_device_id pmic_mp5424_idtable[] = {
      { "pmic_mp5424", 0 },
      { },
};
MODULE_DEVICE_TABLE(i2c, pmic_mp5424_idtable);

static const struct of_device_id pmic_mp5424_of_match[] = {
    {
        .compatible = "pmic_mp5424",
        .data = &pmic_mp5424_idtable[0]
    },
    {}
};
MODULE_DEVICE_TABLE(of, pmic_mp5424_of_match);

static struct i2c_driver pmic_mp5424_driver = {
      .driver = {
              .name   = "pmic_mp5424",
              .owner  = THIS_MODULE,
              .of_match_table = of_match_ptr(pmic_mp5424_of_match),
      },

      .id_table       = pmic_mp5424_idtable,
      .probe          = pmic_mp5424_probe,
      .remove         = pmic_mp5424_remove,
};

static int __init pmic_mp5424_init(void)
{
    return i2c_add_driver(&pmic_mp5424_driver);
}

static void __exit pmic_mp5424_exit(void)
{
    return i2c_del_driver(&pmic_mp5424_driver);
}

module_init(pmic_mp5424_init);
module_exit(pmic_mp5424_exit);

MODULE_AUTHOR("Caleb J <caleb@teknique.com>");
MODULE_DESCRIPTION("Driver for MPS mp5424 pmic");
MODULE_LICENSE("GPL");
