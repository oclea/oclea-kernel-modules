# oclea-kernel-module-als-cg5162tc

Kernel module for cg5162tc i2c ALS sensor, device tree binding should look like:

```
    i2c3: i2c@e400b000 {
        status = "ok";
        cg5162tc: als@10 {
            compatible = "chipgoal,cg5162tc";
            reg = <0x10>;
        };
    };
```

The driver will create the following sysfs nodes:

```
# lux value calculated from ADC channels 0 and 1
/sys/bus/iio/devices/iio:device{X}/in_illuminance_input

# ADC readings for channels 0,1 debug, must read in_illuminance_input to populate
/sys/bus/iio/devices/iio:device{X}/in_intensity0_raw
/sys/bus/iio/devices/iio:device{X}/in_intensity1_raw

# integration time in ms
/sys/bus/iio/devices/iio:device{X}/integration_time

# available integration times
/sys/bus/iio/devices/iio:device{X}/integration_time_available

# gain from 1-15 for increased light sensitivity
/sys/bus/iio/devices/iio:device{X}/in_illuminance0_hardwaregain

# user scaling of output lux, 1-255
/sys/bus/iio/devices/iio:device{X}/in_illuminance0_scale
```
