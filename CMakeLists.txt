cmake_minimum_required(VERSION 3.5)

project(kernel_modules)

set(KERNEL_MODULE_LIST
    hello_world
    ircut_ap1551a
    ircut_drv8837c
    ntc_thermistor
    adc_version
    bmi088-accel
    lm75_iio
    imu_bno080
    irq_toggle_gpio
    irq_sync_leds
    pir_pyd_1698
    pmic_mp5424
    pwm-gpio
    wdt_tps3431
    ledring_i2c
    as3648
    efm8_wdt
    efm8_temperature
    ts_senseboost_ph12
)

# Get kernel version string
if(NOT DEFINED KERNEL_VERSION)
    file(READ ${KERNELHEADERS_DIR}/include/config/kernel.release KERNEL_VERSION)
    string(REGEX REPLACE "\n$" "" KERNEL_VERSION "${KERNEL_VERSION}")
endif()

if(NOT DEFINED FAKEROOT_DIR)
    set(FAKEROOT_DIR ${CMAKE_INSTALL_PREFIX})
endif()

# Set kernel architecture
if(NOT DEFINED KERNEL_ARCH)
if (PLATFORM STREQUAL cv22 OR
    PLATFORM STREQUAL s5l OR
    PLATFORM STREQUAL cv25 OR
    PLATFORM STREQUAL s6lm OR
    PLATFORM STREQUAL cv28 OR
    PLATFORM STREQUAL cv2)
    set(KERNEL_ARCH arm64)
elseif(PLATFORM STREQUAL s2l)
    set(KERNEL_ARCH arm)
else()
    message(FATAL_ERROR "Invalid kernel architecture for modules")
endif()
endif()

foreach(KERNEL_MODULE ${KERNEL_MODULE_LIST})
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/${KERNEL_MODULE})
endforeach()
