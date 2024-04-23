## bno080
BNO080 SiP driver

**Note**

BNO080 driver implements SHTP interfaces to accelerometer, magnetometer, gyroscope and attitude sensors
Each sensor can be enabled separetely
If sensor's update rate DT property is set to 0 or not specified the sensor is disabled
Assign the corresponding update rate DT property to a value greater than zero to enable the sensor
NOTE: Use update rates less then 10 at your own risk, there are complains in the web about random failures for this case
NOTE: Enabling gyroscope and rotatation vector at the same time will increase the latency of rotation vector

### DTS configuration example

imu0: imu@4b {
    compatible = "oclea,bno080";
    reg = <0x4b>;
    ocl,drdy-gpio = <&gpio 39 0x2>; -- interrupt gpio for triggering data read
    ocl,rot-vect-freq = <10>; -- optional, rotation vector update rate in ms
    ocl,acc-freq = <10>; -- optional, accelerometer update rate in ms
    ocl,gyr-freq = <10>; -- optional, gyroscope update rate in ms
    ocl,mag-freq = <10>; -- optional, magnetometer update rate in ms
    dev-name = "iio:imu0"; -- optional
};

DEV-NOTE: spi driver is not tested
DEV-NOTE: there should be a way to get shorter gyro latency if needed

### Sysfs interface

name -- device name
in_accel_calib_save_and_stop -- Stop accelerometer calibration and save calibration data [0/1]
in_accel_calib_status -- Show accelerometer calibration status [On/Off]
in_accel_calibrate -- Start accelerometer calibration [0/1]
in_anglvel_calib_save_and_stop -- Stop gyroscope calibration and save calibration data [0/1]
in_anglvel_calib_status -- Show gyroscope calibration status [On/Off]
in_anglvel_calibrate -- Start gyroscope calibration [0/1]
in_magn_calib_save_and_stop -- Stop magnetometer calibration and save calibration data [0/1]
in_magn_calib_status -- Show magnetometer calibration status [On/Off]
in_magn_calibrate -- Start magnetometer calibration [0/1]
in_rot_calib_save_and_stop -- Stop rotation vector calibration and save calibration data [0/1]
in_rot_calib_status -- Show rotation vector calibration status [On/Off]
in_rot_calibrate -- Start rotation vector calibration [0/1]
out_accel_accuracy -- Show accelerometer accuracy [Low/Medium/High]
out_accel_q1 -- Show accelerometer Q format number [0-16]
out_accel_sampling_frequency -- Show accelerometer report update rate in milliseconds
out_accel_x_raw -- Show accelerometer x axis value [fix point value of q1 format]
out_accel_y_raw -- Show accelerometer y axis value [fix point value of q1 format]
out_accel_z_raw -- Show accelerometer z axis value [fix point value of q1 format]
out_anglvel_accuracy -- Show gyroscope accuracy [Low/Medium/High]
out_anglvel_q1 -- Show gyroscope Q format number [0-16]
out_anglvel_sampling_frequency -- Show gyroscope report update rate in milliseconds
out_anglvel_x_raw -- Show gyroscope x axis value [fix point value of q1 format]
out_anglvel_y_raw -- Show gyroscope y axis value [fix point value of q1 format]
out_anglvel_z_raw -- Show gyroscope z axis value [fix point value of q1 format]
out_magn_accuracy -- Show magnetometer accuracy [Low/Medium/High]
out_magn_q1 -- Show magnetometer Q format number [0-16]
out_magn_sampling_frequency -- Show magnetometer report update rate in milliseconds
out_magn_x_raw -- Show magnetometer x axis value [fix point value of q1 format]
out_magn_y_raw -- Show magnetometer y axis value [fix point value of q1 format]
out_magn_z_raw -- Show magnetometer z axis value [fix point value of q1 format]
out_rot_accuracy -- Show rotation vector accuracy [Low/Medium/High]
out_rot_q1 -- Show rotation vector Q format number [0-16]
out_rot_quaternion_raw -- Show rotation vector quaternion [i j k r]
    i - fix point value of q1 format
    j - fix point value of q1 format
    k - fix point value of q1 format
    r - fix point value of q1 format
out_rot_rad_accuracy -- Show rotation vector radian accuracy [fix point value of q1 format]
out_rot_rad_accuracy_q1 -- Show rotation vector radian accuracy Q format number [0-16]
out_rot_sampling_frequency -- Show rotation vector report update rate in milliseconds
