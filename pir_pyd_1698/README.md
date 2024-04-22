## pir_pyd_1698
PIR PYD 1698 driver

**Note**

### DTS configuration example
pir0: pir0 {
    compatible = "oclea,pyd1698";
    sin-gpio = <&gpio 36 0x0>;
    dl-gpio = <&gpio 35 0x0>;
    dev-name = "iio:pir0";
};
