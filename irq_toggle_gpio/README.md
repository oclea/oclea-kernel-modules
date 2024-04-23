## irq_toggle_gpio 
This driver can be used for monitoring a shared IRQ and when interrupts happen, it will toggle the associated GPIOs.

**Note**

The monitored IRQ must be shared by all drivers that request it. Otherwise, it will not be accessible.

S5L Interrupt IDs:
idsp_vin_mvsync		64
idsp_vin_vsync		65
idsp_vin_sof		66
idsp_vin_dvsync		67
idsp_vin_last_pixel	68
gdma			69
idsp_pip_mvsync		70
idsp_pip_vsync		71
idsp_pip_sof		72
idsp_pip_dvsync		73
idsp_pip_last_pixel	74
vout_tv_sync		75
vout_lcd_sync		76
vin			85
orc_vout0		86
vout			87
vdsp_pip_coding		88
adc_level		89
hdmi			90
wdt			91
uart0			93
pwc_alarm		96
uart1			112
idc2			113
idc1			114
idc0			115
i2stx			117
i2srx			118
usbvbus			119
usb_digital_id_change	120
usb_connect_change	121
usb_charge		122
sdxc_cd			123
sd_cd			124
uart2			125
idc3			127

CV22/CV25/S6LM Interrupt IDs:
adc_level		45
idc0			47
idc1			48
idc2			49
idc3			50
idcs			51
uart0			53
wdt			74
usb_digital_id_change	83
usbvbus			84
usb_connect_change	85
usb_charge		86
sdio0_cd		87
sd_cd			88
uart1			91
uart2			92
uart3			93
uart4			94
gdma			102
i2stx			113
i2srx			114
vout_tv_sync		120
vout_lcd_sync		121
rolling_shutter		140
idsp_vin_mvsync		141
idsp_vin_vsync		142
idsp_sof		143
idsp_last_pixel		144
idsp_vin_dvsync		145
idsp_pip2_vsync		146 (excluding CV22)
idsp_pip2_sof		147 (excluding CV22)
idsp_pip2_last_pixel	148 (excluding CV22)
idsp_pip2_dvsync	149 (excluding CV22)
idsp_pip_mvsync		166
idsp_pip_vsync		167
idsp_pip_sof		168
idsp_pip_last_pixel	169
idsp_pip_dvsync		170
vin			172
vout_lcd		177
vout_tv			178

### DTS configuration example on S5L
1. monitor IRQ 66 which is actually *idsp_vin_sof*
2. toggle gpio 8

- When the first IRQ is triggered, GPIO 8 will toggle, beginning in a low state.

```
  irq_toggle_gpio {
    compatible = "oclea,irq_toggle_gpio";
    gpio = <&gpio 8 0x0>; /* use 0x1 to begin in high state */
    irq = <66>;
    irqname = "start_of_frame_toggle"; /* name if for your reference only, can be anything */
  };

```
