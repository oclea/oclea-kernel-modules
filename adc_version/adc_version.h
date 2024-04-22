/*
 * adc_version.h - ADC version
 */
#ifndef _LINUX_ADC_VER_H
#define _LINUX_ADC_VER_H

struct iio_channel;

enum adc_version_type {
    TYPE_ADCVER_S5L,
    TYPE_ADCVER_CV22,
    TYPE_ADCVER_CV22_USOM,
    TYPE_ADCVER_S6LM,
    TYPE_ADCVER_CV25,
};

struct adc_version_platform_data {
    /*
     * chan: iio_channel pointer to communicate with the ADC which
     * is used for conversion of the analog values.
     */
    int (*read_mv)(struct adc_version_platform_data *);
    unsigned int pullup_mv;
    struct iio_channel *chan;
};

#endif /* _LINUX_ADC_VER_H */
