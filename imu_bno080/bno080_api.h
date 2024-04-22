#pragma once
#ifndef BNO080_API_H_
#define BNO080_API_H_

struct bno080_api_hal_ops {
	uint32_t read_len_limit; // aplicable if > 0
	int (*read_multiple_byte)(struct device *, uint8_t *, int);
	int (*write_multiple_byte)(struct device *, uint8_t *, int);
};

struct bno080_api_sample_data {
	uint16_t rawAccelX, rawAccelY, rawAccelZ, accelAccuracy;
	uint16_t rawLinAccelX, rawLinAccelY, rawLinAccelZ, accelLinAccuracy;
	uint16_t rawGyroX, rawGyroY, rawGyroZ, gyroAccuracy;
	uint16_t rawMagX, rawMagY, rawMagZ, magAccuracy;
	uint16_t rawQuatI, rawQuatJ, rawQuatK, rawQuatReal,
		rawQuatRadianAccuracy, quatAccuracy;
	uint16_t rawFastGyroX, rawFastGyroY, rawFastGyroZ;
	uint8_t tapDetector;
	uint16_t stepCount;
	uint32_t timeStamp;
	uint8_t stabilityClassifier;
	uint8_t activityClassifier;
	uint8_t *_activityConfidences;
	uint8_t calibrationStatus; //Byte R0 of ME Calibration Response
	uint16_t memsRawAccelX, memsRawAccelY,
		memsRawAccelZ; //Raw readings from MEMS sensor
	uint16_t memsRawGyroX, memsRawGyroY,
		memsRawGyroZ; //Raw readings from MEMS sensor
	uint16_t memsRawMagX, memsRawMagY,
		memsRawMagZ; //Raw readings from MEMS sensor
};

struct bno080_api_desc {
	struct bno080_api_hal_ops *hal_ops;
	struct device *dev;
	struct bno080_api_sample_data *sample_data;
};

//Registers
#define CHANNEL_COMMAND 0
#define CHANNEL_EXECUTABLE 1
#define CHANNEL_CONTROL 2
#define CHANNEL_REPORTS 3
#define CHANNEL_WAKE_REPORTS 4
#define CHANNEL_GYRO 5

//All the ways we can configure or talk to the BNO080, figure 34, page 36 reference manual
//These are used for low level communication with the sensor, on channel 2
#define SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define SHTP_REPORT_COMMAND_REQUEST 0xF2
#define SHTP_REPORT_FRS_READ_RESPONSE 0xF3
#define SHTP_REPORT_FRS_READ_REQUEST 0xF4
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_PRODUCT_ID_REQUEST 0xF9
#define SHTP_REPORT_BASE_TIMESTAMP 0xFB
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD

//All the different sensors and features we can get reports from
//These are used when enabling a given sensor
#define SENSOR_REPORTID_ACCELEROMETER 0x01
#define SENSOR_REPORTID_GYROSCOPE 0x02
#define SENSOR_REPORTID_MAGNETIC_FIELD 0x03
#define SENSOR_REPORTID_LINEAR_ACCELERATION 0x04
#define SENSOR_REPORTID_ROTATION_VECTOR 0x05
#define SENSOR_REPORTID_GRAVITY 0x06
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR 0x08
#define SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR 0x09
#define SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR 0x2A
#define SENSOR_REPORTID_TAP_DETECTOR 0x10
#define SENSOR_REPORTID_STEP_COUNTER 0x11
#define SENSOR_REPORTID_STABILITY_CLASSIFIER 0x13
#define SENSOR_REPORTID_RAW_ACCELEROMETER 0x14
#define SENSOR_REPORTID_RAW_GYROSCOPE 0x15
#define SENSOR_REPORTID_RAW_MAGNETOMETER 0x16
#define SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER 0x1E
#define SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR 0x28
#define SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR 0x29

//Record IDs from figure 29, page 29 reference manual
//These are used to read the metadata for each sensor type
#define FRS_RECORDID_ACCELEROMETER 0xE302
#define FRS_RECORDID_GYROSCOPE_CALIBRATED 0xE306
#define FRS_RECORDID_MAGNETIC_FIELD_CALIBRATED 0xE309
#define FRS_RECORDID_ROTATION_VECTOR 0xE30B

//Command IDs from section 6.4, page 42
//These are used to calibrate, initialize, set orientation, tare etc the sensor
#define COMMAND_ERRORS 1
#define COMMAND_COUNTER 2
#define COMMAND_TARE 3
#define COMMAND_INITIALIZE 4
#define COMMAND_DCD 6
#define COMMAND_ME_CALIBRATE 7
#define COMMAND_DCD_PERIOD_SAVE 9
#define COMMAND_OSCILLATOR 10
#define COMMAND_CLEAR_DCD 11

#define CALIBRATE_ACCEL 0
#define CALIBRATE_GYRO 1
#define CALIBRATE_MAG 2
#define CALIBRATE_PLANAR_ACCEL 3
#define CALIBRATE_ACCEL_GYRO_MAG 4
#define CALIBRATE_STOP 5

#define MAX_PACKET_SIZE                                                        \
	128 //Packets can be up to 32k but we don't have that much RAM.
#define MAX_METADATA_SIZE                                                      \
	9 //This is in words. There can be many but we mostly only care about the first 9 (Qs, range, etc)
#define HEADER_SIZE 4

struct bno080_api_desc *bno080_api_init(struct device *dev,
					struct bno080_api_hal_ops *hal_ops);
void bno080_api_release(struct bno080_api_desc *desc);
bool bno080_api_begin(struct bno080_api_desc *desc);
uint16_t bno080_api_getReadings(struct bno080_api_desc *desc);

void bno080_api_enableRotationVector(struct bno080_api_desc *desc,
				     uint16_t timeBetweenReports);
void bno080_api_enableAccelerometer(struct bno080_api_desc *desc,
				    uint16_t timeBetweenReports);
void bno080_api_enableLinearAccelerometer(struct bno080_api_desc *desc,
					  uint16_t timeBetweenReports);
void bno080_api_enableGyro(struct bno080_api_desc *desc,
			   uint16_t timeBetweenReports);
void bno080_api_enableMagnetometer(struct bno080_api_desc *desc,
				   uint16_t timeBetweenReports);
void bno080_api_enableRawAccelerometer(struct bno080_api_desc *desc,
				       uint16_t timeBetweenReports);
void bno080_api_enableRawGyro(struct bno080_api_desc *desc,
			      uint16_t timeBetweenReports);
void bno080_api_enableRawMagnetometer(struct bno080_api_desc *desc,
				      uint16_t timeBetweenReports);

void softReset(struct bno080_api_desc *desc); //Try to reset the IMU via software
uint8_t resetReason(struct bno080_api_desc
			    *desc); //Query the IMU for the reason it last reset
void modeOn(struct bno080_api_desc
		    *desc); //Use the executable channel to turn the BNO on
void modeSleep(
	struct bno080_api_desc
		*desc); //Use the executable channel to put the BNO to sleep

bool receivePacket(struct bno080_api_desc *desc);
bool getData(struct bno080_api_desc *desc, uint16_t bytesRemaining);
bool sendPacket(struct bno080_api_desc *desc, uint8_t channelNumber,
		uint8_t dataLength);
void printPacket(
	struct device *dev); //Prints the current shtp header and data packets
void printHeader(struct device *dev); //Prints the current shtp header (only)

uint16_t parseInputReport(
	struct bno080_api_desc *desc); //Parse sensor readings out of report
uint16_t parseCommandReport(
	struct bno080_api_desc *desc); //Parse command responses out of report

void bno080_api_calibrateAccelerometer(struct bno080_api_desc *desc);
void bno080_api_calibrateGyro(struct bno080_api_desc *desc);
void bno080_api_calibrateMagnetometer(struct bno080_api_desc *desc);
void bno080_api_calibratePlanarAccelerometer(struct bno080_api_desc *desc);
void bno080_api_calibrateAll(struct bno080_api_desc *desc);
void bno080_api_endCalibration(struct bno080_api_desc *desc);
void bno080_api_saveCalibration(struct bno080_api_desc *desc);
void bno080_api_requestCalibrationStatus(
	struct bno080_api_desc *desc); //Sends command to get status
bool bno080_api_calibrationFailed(
	struct bno080_api_desc
		*desc); //Checks ME Cal response for byte 5, R0 - Status

void setFeatureCommand(struct bno080_api_desc *desc, uint8_t reportID,
		       uint16_t timeBetweenReports);
void setFeatureCommandWithConfig(struct bno080_api_desc *desc, uint8_t reportID,
				 uint16_t timeBetweenReports,
				 uint32_t specificConfig);
void sendCommand(struct bno080_api_desc *desc, uint8_t command);
void sendCalibrateCommand(struct bno080_api_desc *desc,
			  uint8_t thingToCalibrate);

//Metadata functions
int16_t bno080_api_getQ1(struct bno080_api_desc *desc, uint16_t recordID);
int16_t bno080_api_getQ2(struct bno080_api_desc *desc, uint16_t recordID);
int16_t bno080_api_getQ3(struct bno080_api_desc *desc, uint16_t recordID);
uint32_t bno080_api_getResolution(struct bno080_api_desc *desc,
				  uint16_t recordID);
uint32_t bno080_api_getRange(struct bno080_api_desc *desc, uint16_t recordID);
uint32_t readFRSword(struct bno080_api_desc *desc, uint16_t recordID,
		     uint8_t wordNumber);
void frsReadRequest(struct bno080_api_desc *desc, uint16_t recordID,
		    uint16_t readOffset, uint16_t blockSize);
bool readFRSdata(struct bno080_api_desc *desc, uint16_t recordID,
		 uint8_t startLocation, uint8_t wordsToRead);

#endif // BNO080_API_H_
