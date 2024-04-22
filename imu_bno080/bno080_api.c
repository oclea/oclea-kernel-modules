/*
BNO080 api based on Sparkfun Arduino Library
https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library
*/

#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/device.h>

#include "bno080_api.h"

//Global Variables
static uint8_t shtpHeader[4]; //Each packet has a header of 4 bytes
static uint8_t shtpData[MAX_PACKET_SIZE];
static uint8_t sequenceNumber[6] = {
	0, 0, 0, 0, 0, 0
}; //There are 6 com channels. Each channel has its own seqnum
static uint8_t commandSequenceNumber =
	0; //Commands have a seqNum as well. These are inside command packet, the header uses its own seqNum per channel
static uint32_t metaData
	[MAX_METADATA_SIZE]; //There is more than 10 words in a metadata record but we'll stop at Q point 3

struct bno080_api_desc *bno080_api_init(struct device *dev,
					struct bno080_api_hal_ops *hal_ops)
{
	struct bno080_api_sample_data *sample_data;
	struct bno080_api_desc *desc;

	sample_data =
		kzalloc(sizeof(struct bno080_api_sample_data), GFP_KERNEL);
	if (sample_data == NULL)
		return NULL;

	desc = kzalloc(sizeof(struct bno080_api_desc), GFP_KERNEL);
	if (desc == NULL) {
		kfree(sample_data);
		return NULL;
	}

	desc->sample_data = sample_data;
	desc->dev = dev;
	desc->hal_ops = hal_ops;

	return desc;
}

void bno080_api_release(struct bno080_api_desc *desc)
{
	if (desc) {
		if (desc->sample_data)
			kfree(desc->sample_data);
		kfree(desc);
	}
}

//Attempt communication with the device
bool bno080_api_begin(struct bno080_api_desc *desc)
{
	bool ret = false;
	struct device *dev;
	uint32_t SW_Part_Number;
	uint32_t SW_Build_Number;
	uint16_t SW_Version_Patch;

	//Begin by resetting the IMU
	softReset(desc);

	//Check communication with device
	shtpData[0] =
		SHTP_REPORT_PRODUCT_ID_REQUEST; //Request the product ID and reset info
	shtpData[1] = 0; //Reserved

	//Transmit packet on channel 2, 2 bytes
	sendPacket(desc, CHANNEL_CONTROL, 2);

	dev = desc->dev;

	//Now we wait for response
	if (receivePacket(desc) == true) {
		if (shtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE) {
			dev_dbg(dev, "SW Version Major: 0x%02x\n",
				 shtpData[2]);
			dev_dbg(dev, "SW Version Minor: 0x%02x\n",
				 shtpData[3]);
			SW_Part_Number = ((uint32_t)shtpData[7] << 24) |
					 ((uint32_t)shtpData[6] << 16) |
					 ((uint32_t)shtpData[5] << 8) |
					 ((uint32_t)shtpData[4]);
			dev_dbg(dev, "SW Part Number: 0x%08x\n",
				 SW_Part_Number);
			SW_Build_Number = ((uint32_t)shtpData[11] << 24) |
					  ((uint32_t)shtpData[10] << 16) |
					  ((uint32_t)shtpData[9] << 8) |
					  ((uint32_t)shtpData[8]);
			dev_dbg(dev, "SW Build Number: 0x%08x\n",
				 SW_Build_Number);
			SW_Version_Patch = ((uint16_t)shtpData[13] << 8) |
					   ((uint16_t)shtpData[12]);
			dev_dbg(dev, "SW Version Patch: 0x%04x\n",
				 SW_Version_Patch);
			ret = true;
		}
	}

	return ret;
}

uint16_t bno080_api_getReadings(struct bno080_api_desc *desc)
{
	if (receivePacket(desc) == true) {
		//Check to see if this packet is a sensor reporting its data to us
		if (shtpHeader[2] == CHANNEL_REPORTS &&
		    shtpData[0] == SHTP_REPORT_BASE_TIMESTAMP) {
			return parseInputReport(
				desc); //This will update the rawAccelX, etc variables depending on which feature report is found
		} else if (shtpHeader[2] == CHANNEL_CONTROL) {
			return parseCommandReport(
				desc); //This will update responses to commands, calibrationStatus, etc.
		} else if (shtpHeader[2] == CHANNEL_GYRO) {
			return parseInputReport(
				desc); //This will update the rawAccelX, etc variables depending on which feature report is found
		}
	}
	return 0;
}

//This function pulls the data from the command response report

//Unit responds with packet that contains the following:
//shtpHeader[0:3]: First, a 4 byte header
//shtpData[0]: The Report ID
//shtpData[1]: Sequence number (See 6.5.18.2)
//shtpData[2]: Command
//shtpData[3]: Command Sequence Number
//shtpData[4]: Response Sequence Number
//shtpData[5 + 0]: R0
//shtpData[5 + 1]: R1
//shtpData[5 + 2]: R2
//shtpData[5 + 3]: R3
//shtpData[5 + 4]: R4
//shtpData[5 + 5]: R5
//shtpData[5 + 6]: R6
//shtpData[5 + 7]: R7
//shtpData[5 + 8]: R8
uint16_t parseCommandReport(struct bno080_api_desc *desc)
{
	if (shtpData[0] == SHTP_REPORT_COMMAND_RESPONSE) {
		//The BNO080 responds with this report to command requests. It's up to use to remember which command we issued.
		uint8_t command =
			shtpData[2]; //This is the Command byte of the response

		if (command == COMMAND_ME_CALIBRATE) {
			desc->sample_data->calibrationStatus = shtpData
				[5 +
				 0]; //R0 - Status (0 = success, non-zero = fail)
		}
		return shtpData[0];
	} else {
		//This sensor report ID is unhandled.
		//See reference manual to add additional feature reports as needed
	}

	//TODO additional feature reports may be strung together. Parse them all.
	return 0;
}

//This function pulls the data from the input report
//The input reports vary in length so this function stores the various 16-bit values as globals

//Unit responds with packet that contains the following:
//shtpHeader[0:3]: First, a 4 byte header
//shtpData[0:4]: Then a 5 byte timestamp of microsecond clicks since reading was taken
//shtpData[5 + 0]: Then a feature report ID (0x01 for Accel, 0x05 for Rotation Vector)
//shtpData[5 + 1]: Sequence number (See 6.5.18.2)
//shtpData[5 + 2]: Status
//shtpData[3]: Delay
//shtpData[4:5]: i/accel x/gyro x/etc
//shtpData[6:7]: j/accel y/gyro y/etc
//shtpData[8:9]: k/accel z/gyro z/etc
//shtpData[10:11]: real/gyro temp/etc
//shtpData[12:13]: Accuracy estimate
uint16_t parseInputReport(struct bno080_api_desc *desc)
{
	uint8_t status;
	uint8_t command;
	uint16_t data1;
	uint16_t data2;
	uint16_t data3;
	uint16_t data4;
	uint16_t data5;
	int i;
	int16_t dataLength; //Calculate the number of data bytes in this packet

	dataLength = ((uint16_t)shtpHeader[1] << 8 | shtpHeader[0]);
	dataLength &= ~(
		1
		<< 15); //Clear the MSbit. This bit indicates if this package is a continuation of the last.
	//Ignore it for now. TODO catch this as an error and exit

	dataLength -= HEADER_SIZE; //Remove the header bytes from the data count

	desc->sample_data->timeStamp = ((uint32_t)shtpData[4] << (8 * 3)) |
				       ((uint32_t)shtpData[3] << (8 * 2)) |
				       ((uint32_t)shtpData[2] << (8 * 1)) |
				       ((uint32_t)shtpData[1] << (8 * 0));

	// The gyro-integrated input reports are sent via the special gyro channel and do no include the usual ID, sequence, and status fields
	if (shtpHeader[2] == CHANNEL_GYRO) {
		desc->sample_data->rawQuatI =
			(uint16_t)shtpData[1] << 8 | shtpData[0];
		desc->sample_data->rawQuatJ =
			(uint16_t)shtpData[3] << 8 | shtpData[2];
		desc->sample_data->rawQuatK =
			(uint16_t)shtpData[5] << 8 | shtpData[4];
		desc->sample_data->rawQuatReal =
			(uint16_t)shtpData[7] << 8 | shtpData[6];
		desc->sample_data->rawFastGyroX =
			(uint16_t)shtpData[9] << 8 | shtpData[8];
		desc->sample_data->rawFastGyroY =
			(uint16_t)shtpData[11] << 8 | shtpData[10];
		desc->sample_data->rawFastGyroZ =
			(uint16_t)shtpData[13] << 8 | shtpData[12];

		return SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR;
	}

	status = shtpData[5 + 2] & 0x03; //Get status bits
	data1 = (uint16_t)shtpData[5 + 5] << 8 | shtpData[5 + 4];
	data2 = (uint16_t)shtpData[5 + 7] << 8 | shtpData[5 + 6];
	data3 = (uint16_t)shtpData[5 + 9] << 8 | shtpData[5 + 8];
	data4 = 0;
	data5 = 0; //We would need to change this to uin32_t to capture time stamp value on Raw Accel/Gyro/Mag reports

	if (dataLength - 5 > 9) {
		data4 = (uint16_t)shtpData[5 + 11] << 8 | shtpData[5 + 10];
	}
	if (dataLength - 5 > 11) {
		data5 = (uint16_t)shtpData[5 + 13] << 8 | shtpData[5 + 12];
	}

	//Store these generic values to their proper global variable
	if (shtpData[5] == SENSOR_REPORTID_ACCELEROMETER) {
		desc->sample_data->accelAccuracy = status;
		desc->sample_data->rawAccelX = data1;
		desc->sample_data->rawAccelY = data2;
		desc->sample_data->rawAccelZ = data3;
	} else if (shtpData[5] == SENSOR_REPORTID_LINEAR_ACCELERATION) {
		desc->sample_data->accelLinAccuracy = status;
		desc->sample_data->rawLinAccelX = data1;
		desc->sample_data->rawLinAccelY = data2;
		desc->sample_data->rawLinAccelZ = data3;
	} else if (shtpData[5] == SENSOR_REPORTID_GYROSCOPE) {
		desc->sample_data->gyroAccuracy = status;
		desc->sample_data->rawGyroX = data1;
		desc->sample_data->rawGyroY = data2;
		desc->sample_data->rawGyroZ = data3;
	} else if (shtpData[5] == SENSOR_REPORTID_MAGNETIC_FIELD) {
		desc->sample_data->magAccuracy = status;
		desc->sample_data->rawMagX = data1;
		desc->sample_data->rawMagY = data2;
		desc->sample_data->rawMagZ = data3;
	} else if (shtpData[5] == SENSOR_REPORTID_ROTATION_VECTOR ||
		   shtpData[5] == SENSOR_REPORTID_GAME_ROTATION_VECTOR ||
		   shtpData[5] ==
			   SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR ||
		   shtpData[5] ==
			   SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR) {
		desc->sample_data->quatAccuracy = status;
		desc->sample_data->rawQuatI = data1;
		desc->sample_data->rawQuatJ = data2;
		desc->sample_data->rawQuatK = data3;
		desc->sample_data->rawQuatReal = data4;

		//Only available on rotation vector and ar/vr stabilized rotation vector,
		// not game rot vector and not ar/vr stabilized rotation vector
		desc->sample_data->rawQuatRadianAccuracy = data5;
	} else if (shtpData[5] == SENSOR_REPORTID_TAP_DETECTOR) {
		desc->sample_data->tapDetector = shtpData[5 + 4]; //Byte 4 only
	} else if (shtpData[5] == SENSOR_REPORTID_STEP_COUNTER) {
		desc->sample_data->stepCount = data3; //Bytes 8/9
	} else if (shtpData[5] == SENSOR_REPORTID_STABILITY_CLASSIFIER) {
		desc->sample_data->stabilityClassifier =
			shtpData[5 + 4]; //Byte 4 only
	} else if (shtpData[5] ==
		   SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER) {
		desc->sample_data->activityClassifier =
			shtpData[5 + 5]; //Most likely state

		//Load activity classification confidences into the array
		for (i = 0; i < 9;
		     i++) //Hardcoded to max of 9. TODO - bring in array size
			desc->sample_data->_activityConfidences[i] = shtpData
				[5 + 6 +
				 i]; //5 bytes of timestamp, byte 6 is first confidence byte
	} else if (shtpData[5] == SENSOR_REPORTID_RAW_ACCELEROMETER) {
		desc->sample_data->memsRawAccelX = data1;
		desc->sample_data->memsRawAccelY = data2;
		desc->sample_data->memsRawAccelZ = data3;
	} else if (shtpData[5] == SENSOR_REPORTID_RAW_GYROSCOPE) {
		desc->sample_data->memsRawGyroX = data1;
		desc->sample_data->memsRawGyroY = data2;
		desc->sample_data->memsRawGyroZ = data3;
	} else if (shtpData[5] == SENSOR_REPORTID_RAW_MAGNETOMETER) {
		desc->sample_data->memsRawMagX = data1;
		desc->sample_data->memsRawMagY = data2;
		desc->sample_data->memsRawMagZ = data3;
	} else if (shtpData[5] == SHTP_REPORT_COMMAND_RESPONSE) {
		//The BNO080 responds with this report to command requests. It's up to use to remember which command we issued.
		command =
			shtpData[5 +
				 2]; //This is the Command byte of the response

		if (command == COMMAND_ME_CALIBRATE) {
			desc->sample_data->calibrationStatus = shtpData
				[5 +
				 5]; //R0 - Status (0 = success, non-zero = fail)
		}
	} else {
		//This sensor report ID is unhandled.
		//See reference manual to add additional feature reports as needed
		return 0;
	}

	//TODO additional feature reports may be strung together. Parse them all.
	return shtpData[5];
}

//Given a record ID, read the Q1 value from the metaData record in the FRS (ya, it's complicated)
//Q1 is used for all sensor data calculations
int16_t bno080_api_getQ1(struct bno080_api_desc *desc, uint16_t recordID)
{
	//Q1 is always the lower 16 bits of word 7
	uint16_t q = readFRSword(desc, recordID, 7) &
		     0xFFFF; //Get word 7, lower 16 bits
	return q;
}

//Given a record ID, read the Q2 value from the metaData record in the FRS
//Q2 is used in sensor bias
int16_t bno080_api_getQ2(struct bno080_api_desc *desc, uint16_t recordID)
{
	//Q2 is always the upper 16 bits of word 7
	uint16_t q = readFRSword(desc, recordID, 7) >>
		     16; //Get word 7, upper 16 bits
	return q;
}

//Given a record ID, read the Q3 value from the metaData record in the FRS
//Q3 is used in sensor change sensitivity
int16_t bno080_api_getQ3(struct bno080_api_desc *desc, uint16_t recordID)
{
	//Q3 is always the upper 16 bits of word 8
	uint16_t q = readFRSword(desc, recordID, 8) >>
		     16; //Get word 8, upper 16 bits
	return q;
}

//Given a record ID, read the resolution value from the metaData record in the FRS for a given sensor
uint32_t bno080_api_getResolution(struct bno080_api_desc *desc,
				  uint16_t recordID)
{
	//Resolution is always word 2
	return readFRSword(desc, recordID, 2); //Get word 2
}

//Given a record ID, read the range value from the metaData record in the FRS for a given sensor
uint32_t bno080_api_getRange(struct bno080_api_desc *desc, uint16_t recordID)
{
	//Range is always word 1
	return readFRSword(desc, recordID, 1); //Get word 1
}

//Given a record ID and a word number, look up the word data
//Helpful for pulling out a Q value, range, etc.
//Use readFRSdata for pulling out multi-word objects for a sensor (Vendor data for example)
uint32_t readFRSword(struct bno080_api_desc *desc, uint16_t recordID,
		     uint8_t wordNumber)
{
	if (readFRSdata(desc, recordID, wordNumber, 1) ==
	    true) //Get word number, just one word in length from FRS
		return (metaData[0]); //Return this one word

	return 0; //Error
}

//Ask the sensor for data from the Flash Record System
//See 6.3.6 page 40, FRS Read Request
void frsReadRequest(struct bno080_api_desc *desc, uint16_t recordID,
		    uint16_t readOffset, uint16_t blockSize)
{
	shtpData[0] = SHTP_REPORT_FRS_READ_REQUEST; //FRS Read Request
	shtpData[1] = 0; //Reserved
	shtpData[2] = (readOffset >> 0) & 0xFF; //Read Offset LSB
	shtpData[3] = (readOffset >> 8) & 0xFF; //Read Offset MSB
	shtpData[4] = (recordID >> 0) & 0xFF; //FRS Type LSB
	shtpData[5] = (recordID >> 8) & 0xFF; //FRS Type MSB
	shtpData[6] = (blockSize >> 0) & 0xFF; //Block size LSB
	shtpData[7] = (blockSize >> 8) & 0xFF; //Block size MSB

	//Transmit packet on channel 2, 8 bytes
	sendPacket(desc, CHANNEL_CONTROL, 8);
}

//Given a sensor or record ID, and a given start/stop bytes, read the data from the Flash Record System (FRS) for this sensor
//Returns true if metaData array is loaded successfully
//Returns false if failure
bool readFRSdata(struct bno080_api_desc *desc, uint16_t recordID,
		 uint8_t startLocation, uint8_t wordsToRead)
{
	uint8_t spot = 0;
	uint8_t counter = 0;
	uint8_t dataLength = 0;
	uint8_t frsStatus = 0;
	uint32_t data0, data1;

	//First we send a Flash Record System (FRS) request
	frsReadRequest(
		desc, recordID, startLocation,
		wordsToRead); //From startLocation of record, read a # of words

	//Read bytes until FRS reports that the read is complete
	while (1) {
		//Now we wait for response
		while (1) {
			counter = 0;
			while (receivePacket(desc) == false) {
				if (counter++ > 100)
					return false; //Give up
				mdelay(1);
			}

			//We have the packet, inspect it for the right contents
			//See page 40. Report ID should be 0xF3 and the FRS types should match the thing we requested
			if (shtpData[0] == SHTP_REPORT_FRS_READ_RESPONSE)
				if (((((uint16_t)shtpData[13]) << 8) |
				     shtpData[12]) == recordID)
					break; //This packet is one we are looking for
		}

		dataLength = shtpData[1] >> 4;
		frsStatus = shtpData[1] & 0x0F;

		data0 = (uint32_t)shtpData[7] << 24 |
			(uint32_t)shtpData[6] << 16 |
			(uint32_t)shtpData[5] << 8 | (uint32_t)shtpData[4];
		data1 = (uint32_t)shtpData[11] << 24 |
			(uint32_t)shtpData[10] << 16 |
			(uint32_t)shtpData[9] << 8 | (uint32_t)shtpData[8];

		//Record these words to the metaData array
		if (dataLength > 0) {
			metaData[spot++] = data0;
		}
		if (dataLength > 1) {
			metaData[spot++] = data1;
		}

		if (spot >= MAX_METADATA_SIZE) {
			dev_dbg(desc->dev,
				"metaData array over run. Returning.\n");
			return true; //We have run out of space in our array. Bail.
		}

		if (frsStatus == 3 || frsStatus == 6 || frsStatus == 7) {
			return true; //FRS status is read completed! We're done!
		}
	}
}

//Send command to reset IC
//Read all advertisement packets from sensor
//The sensor has been seen to reset twice if we attempt too much too quickly.
//This seems to work reliably.
void softReset(struct bno080_api_desc *desc)
{
	shtpData[0] = 1; //Reset

	//Attempt to start communication with sensor
	sendPacket(desc, CHANNEL_EXECUTABLE,
		   1); //Transmit packet on channel 1, 1 byte

	//Read all incoming data and flush it
	mdelay(200);
	while (receivePacket(desc) == true)
		;
	mdelay(200);
	while (receivePacket(desc) == true)
		;
}

//Set the operating mode to "On"
void modeOn(struct bno080_api_desc *desc)
{
	shtpData[0] = 2; //On

	//Attempt to start communication with sensor
	sendPacket(desc, CHANNEL_EXECUTABLE,
		   1); //Transmit packet on channel 1, 1 byte

	//Read all incoming data and flush it
	mdelay(200);
	while (receivePacket(desc) == true)
		;
	mdelay(200);
	while (receivePacket(desc) == true)
		;
}

//Set the operating mode to "Sleep"
void modeSleep(struct bno080_api_desc *desc)
{
	shtpData[0] = 3; //Sleep

	//Attempt to start communication with sensor
	sendPacket(desc, CHANNEL_EXECUTABLE,
		   1); //Transmit packet on channel 1, 1 byte

	//Read all incoming data and flush it
	mdelay(200);
	while (receivePacket(desc) == true)
		;
	mdelay(200);
	while (receivePacket(desc) == true)
		;
}

//Get the reason for the last reset
//1 = POR, 2 = Internal reset, 3 = Watchdog, 4 = External reset, 5 = Other
uint8_t resetReason(struct bno080_api_desc *desc)
{
	shtpData[0] =
		SHTP_REPORT_PRODUCT_ID_REQUEST; //Request the product ID and reset info
	shtpData[1] = 0; //Reserved

	//Transmit packet on channel 2, 2 bytes
	sendPacket(desc, CHANNEL_CONTROL, 2);

	//Now we wait for response
	if (receivePacket(desc) == true) {
		if (shtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE) {
			return shtpData[1];
		}
	}

	return 0;
}

//Sends the packet to enable the rotation vector
void bno080_api_enableRotationVector(struct bno080_api_desc *desc,
				     uint16_t timeBetweenReports)
{
	setFeatureCommand(desc, SENSOR_REPORTID_ROTATION_VECTOR,
			  timeBetweenReports);
}

//Sends the packet to enable the accelerometer
void bno080_api_enableAccelerometer(struct bno080_api_desc *desc,
				    uint16_t timeBetweenReports)
{
	setFeatureCommand(desc, SENSOR_REPORTID_ACCELEROMETER,
			  timeBetweenReports);
}

//Sends the packet to enable the accelerometer
void bno080_api_enableLinearAccelerometer(struct bno080_api_desc *desc,
					  uint16_t timeBetweenReports)
{
	setFeatureCommand(desc, SENSOR_REPORTID_LINEAR_ACCELERATION,
			  timeBetweenReports);
}

//Sends the packet to enable the gyro
void bno080_api_enableGyro(struct bno080_api_desc *desc,
			   uint16_t timeBetweenReports)
{
	setFeatureCommand(desc, SENSOR_REPORTID_GYROSCOPE, timeBetweenReports);
}

//Sends the packet to enable the magnetometer
void bno080_api_enableMagnetometer(struct bno080_api_desc *desc,
				   uint16_t timeBetweenReports)
{
	setFeatureCommand(desc, SENSOR_REPORTID_MAGNETIC_FIELD,
			  timeBetweenReports);
}

//Sends the packet to enable the raw accel readings
//Note you must enable basic reporting on the sensor as well
void bno080_api_enableRawAccelerometer(struct bno080_api_desc *desc,
				       uint16_t timeBetweenReports)
{
	setFeatureCommand(desc, SENSOR_REPORTID_RAW_ACCELEROMETER,
			  timeBetweenReports);
}

//Sends the packet to enable the raw accel readings
//Note you must enable basic reporting on the sensor as well
void bno080_api_enableRawGyro(struct bno080_api_desc *desc,
			      uint16_t timeBetweenReports)
{
	setFeatureCommand(desc, SENSOR_REPORTID_RAW_GYROSCOPE,
			  timeBetweenReports);
}

//Sends the packet to enable the raw accel readings
//Note you must enable basic reporting on the sensor as well
void bno080_api_enableRawMagnetometer(struct bno080_api_desc *desc,
				      uint16_t timeBetweenReports)
{
	setFeatureCommand(desc, SENSOR_REPORTID_RAW_MAGNETOMETER,
			  timeBetweenReports);
}

//Sends the commands to begin calibration of the accelerometer
void bno080_api_calibrateAccelerometer(struct bno080_api_desc *desc)
{
	sendCalibrateCommand(desc, CALIBRATE_ACCEL);
}

//Sends the commands to begin calibration of the gyro
void bno080_api_calibrateGyro(struct bno080_api_desc *desc)
{
	sendCalibrateCommand(desc, CALIBRATE_GYRO);
}

//Sends the commands to begin calibration of the magnetometer
void bno080_api_calibrateMagnetometer(struct bno080_api_desc *desc)
{
	sendCalibrateCommand(desc, CALIBRATE_MAG);
}

//Sends the commands to begin calibration of the planar accelerometer
void bno080_api_calibratePlanarAccelerometer(struct bno080_api_desc *desc)
{
	sendCalibrateCommand(desc, CALIBRATE_PLANAR_ACCEL);
}

//See 2.2 of the Calibration Procedure document 1000-4044
void bno080_api_calibrateAll(struct bno080_api_desc *desc)
{
	sendCalibrateCommand(desc, CALIBRATE_ACCEL_GYRO_MAG);
}

void bno080_api_endCalibration(struct bno080_api_desc *desc)
{
	sendCalibrateCommand(desc, CALIBRATE_STOP); //Disables all calibrations
}

//See page 51 of reference manual - ME Calibration Response
//Byte 5 is parsed during the readPacket and stored in calibrationStatus
bool bno080_api_calibrationFailed(struct bno080_api_desc *desc)
{
	return (desc->sample_data->calibrationStatus != 0);
}

//Given a sensor's report ID, this tells the BNO080 to begin reporting the values
void setFeatureCommand(struct bno080_api_desc *desc, uint8_t reportID,
		       uint16_t timeBetweenReports)
{
	setFeatureCommandWithConfig(desc, reportID, timeBetweenReports,
				    0); //No specific config
}

//Given a sensor's report ID, this tells the BNO080 to begin reporting the values
//Also sets the specific config word. Useful for personal activity classifier
void setFeatureCommandWithConfig(struct bno080_api_desc *desc, uint8_t reportID,
				 uint16_t timeBetweenReports,
				 uint32_t specificConfig)
{
	long microsBetweenReports = (long)timeBetweenReports * 1000L;

	shtpData[0] =
		SHTP_REPORT_SET_FEATURE_COMMAND; //Set feature command. Reference page 55
	shtpData[1] =
		reportID; //Feature Report ID. 0x01 = Accelerometer, 0x05 = Rotation vector
	shtpData[2] = 0; //Feature flags
	shtpData[3] = 0; //Change sensitivity (LSB)
	shtpData[4] = 0; //Change sensitivity (MSB)
	shtpData[5] =
		(microsBetweenReports >> 0) &
		0xFF; //Report interval (LSB) in microseconds. 0x7A120 = 500ms
	shtpData[6] = (microsBetweenReports >> 8) & 0xFF; //Report interval
	shtpData[7] = (microsBetweenReports >> 16) & 0xFF; //Report interval
	shtpData[8] =
		(microsBetweenReports >> 24) & 0xFF; //Report interval (MSB)
	shtpData[9] = 0; //Batch Interval (LSB)
	shtpData[10] = 0; //Batch Interval
	shtpData[11] = 0; //Batch Interval
	shtpData[12] = 0; //Batch Interval (MSB)
	shtpData[13] =
		(specificConfig >> 0) & 0xFF; //Sensor-specific config (LSB)
	shtpData[14] = (specificConfig >> 8) & 0xFF; //Sensor-specific config
	shtpData[15] = (specificConfig >> 16) & 0xFF; //Sensor-specific config
	shtpData[16] =
		(specificConfig >> 24) & 0xFF; //Sensor-specific config (MSB)

	//Transmit packet on channel 2, 17 bytes
	sendPacket(desc, CHANNEL_CONTROL, 17);
}

//Tell the sensor to do a command
//See 6.3.8 page 41, Command request
//The caller is expected to set P0 through P8 prior to calling
void sendCommand(struct bno080_api_desc *desc, uint8_t command)
{
	shtpData[0] = SHTP_REPORT_COMMAND_REQUEST; //Command Request
	shtpData[1] =
		commandSequenceNumber++; //Increments automatically each function call
	shtpData[2] = command; //Command

	//Caller must set these
	/*shtpData[3] = 0; //P0
	shtpData[4] = 0; //P1
	shtpData[5] = 0; //P2
	shtpData[6] = 0;
	shtpData[7] = 0;
	shtpData[8] = 0;
	shtpData[9] = 0;
	shtpData[10] = 0;
	shtpData[11] = 0;*/

	//Transmit packet on channel 2, 12 bytes
	sendPacket(desc, CHANNEL_CONTROL, 12);
}

//This tells the BNO080 to begin calibrating
//See page 50 of reference manual and the 1000-4044 calibration doc
void sendCalibrateCommand(struct bno080_api_desc *desc,
			  uint8_t thingToCalibrate)
{
	/*shtpData[3] = 0; //P0 - Accel Cal Enable
	shtpData[4] = 0; //P1 - Gyro Cal Enable
	shtpData[5] = 0; //P2 - Mag Cal Enable
	shtpData[6] = 0; //P3 - Subcommand 0x00
	shtpData[7] = 0; //P4 - Planar Accel Cal Enable
	shtpData[8] = 0; //P5 - Reserved
	shtpData[9] = 0; //P6 - Reserved
	shtpData[10] = 0; //P7 - Reserved
	shtpData[11] = 0; //P8 - Reserved*/

	uint8_t x;

	for (x = 3; x < 12; x++) //Clear this section of the shtpData array
		shtpData[x] = 0;

	if (thingToCalibrate == CALIBRATE_ACCEL)
		shtpData[3] = 1;
	else if (thingToCalibrate == CALIBRATE_GYRO)
		shtpData[4] = 1;
	else if (thingToCalibrate == CALIBRATE_MAG)
		shtpData[5] = 1;
	else if (thingToCalibrate == CALIBRATE_PLANAR_ACCEL)
		shtpData[7] = 1;
	else if (thingToCalibrate == CALIBRATE_ACCEL_GYRO_MAG) {
		shtpData[3] = 1;
		shtpData[4] = 1;
		shtpData[5] = 1;
	} else if (thingToCalibrate == CALIBRATE_STOP)
		; //Do nothing, bytes are set to zero

	//Make the internal calStatus variable non-zero (operation failed) so that user can test while we wait
	desc->sample_data->calibrationStatus = 1;

	//Using this shtpData packet, send a command
	sendCommand(desc, COMMAND_ME_CALIBRATE);
}

//Request ME Calibration Status from BNO080
//See page 51 of reference manual
void bno080_api_requestCalibrationStatus(struct bno080_api_desc *desc)
{
	/*shtpData[3] = 0; //P0 - Reserved
	shtpData[4] = 0; //P1 - Reserved
	shtpData[5] = 0; //P2 - Reserved
	shtpData[6] = 0; //P3 - 0x01 - Subcommand: Get ME Calibration
	shtpData[7] = 0; //P4 - Reserved
	shtpData[8] = 0; //P5 - Reserved
	shtpData[9] = 0; //P6 - Reserved
	shtpData[10] = 0; //P7 - Reserved
	shtpData[11] = 0; //P8 - Reserved*/

	uint8_t x;

	for (x = 3; x < 12; x++) //Clear this section of the shtpData array
		shtpData[x] = 0;

	shtpData[6] = 0x01; //P3 - 0x01 - Subcommand: Get ME Calibration

	//Using this shtpData packet, send a command
	sendCommand(desc, COMMAND_ME_CALIBRATE);
}

//This tells the BNO080 to save the Dynamic Calibration Data (DCD) to flash
//See page 49 of reference manual and the 1000-4044 calibration doc
void bno080_api_saveCalibration(struct bno080_api_desc *desc)
{
	/*shtpData[3] = 0; //P0 - Reserved
	shtpData[4] = 0; //P1 - Reserved
	shtpData[5] = 0; //P2 - Reserved
	shtpData[6] = 0; //P3 - Reserved
	shtpData[7] = 0; //P4 - Reserved
	shtpData[8] = 0; //P5 - Reserved
	shtpData[9] = 0; //P6 - Reserved
	shtpData[10] = 0; //P7 - Reserved
	shtpData[11] = 0; //P8 - Reserved*/

	uint8_t x;

	for (x = 3; x < 12; x++) //Clear this section of the shtpData array
		shtpData[x] = 0;

	//Using this shtpData packet, send a command
	sendCommand(desc, COMMAND_DCD); //Save DCD command
}

//Check to see if there is any new data available
//Read the contents of the incoming packet into the shtpData array
bool receivePacket(struct bno080_api_desc *desc)
{
	struct device *dev;
	struct bno080_api_hal_ops *ops;
	uint8_t packetLSB;
	uint8_t packetMSB;
	uint16_t dataLength;

	memset(shtpHeader, 0x0, HEADER_SIZE);

	dev = desc->dev;
	ops = desc->hal_ops;

	if (ops->read_multiple_byte(dev, shtpHeader, HEADER_SIZE) !=
	    HEADER_SIZE) {
		return false;
	}

	//Get the first four bytes, aka the packet header
	packetLSB = shtpHeader[0];
	packetMSB = shtpHeader[1];

	//Calculate the number of data bytes in this packet
	dataLength = (((uint16_t)packetMSB) << 8) | ((uint16_t)packetLSB);
	dataLength &= ~(1 << 15); //Clear the MSbit.

	if (dataLength == 0) {
		return false; //All done
	}
	dataLength -= HEADER_SIZE; //Remove the header bytes from the data count

	getData(desc, dataLength);
	dev_dbg(desc->dev, "Received packet:\n");
	printPacket(desc->dev);

	return true;
}

bool getData(struct bno080_api_desc *desc, uint16_t bytesRemaining)
{
	uint16_t dataSpot = 0; //Start at the beginning of shtpData array
	uint8_t buffer[MAX_PACKET_SIZE + HEADER_SIZE] = { 0 };
	uint32_t read_len_limit = desc->hal_ops->read_len_limit;
	struct bno080_api_hal_ops *ops = desc->hal_ops;
	struct device *dev = desc->dev;

	//Setup a series of chunked 32 byte reads
	while (bytesRemaining > 0) {
		uint16_t numberOfBytesToRead = bytesRemaining;
		if (read_len_limit > 0) {
			if (numberOfBytesToRead >
			    (read_len_limit - HEADER_SIZE))
				numberOfBytesToRead =
					(read_len_limit - HEADER_SIZE);
		}

		if (ops->read_multiple_byte(
			    dev, buffer, numberOfBytesToRead + HEADER_SIZE) !=
		    (numberOfBytesToRead + HEADER_SIZE)) {
			return false;
		}

		if ((dataSpot + numberOfBytesToRead) < MAX_PACKET_SIZE) {
			memcpy(shtpData + dataSpot, buffer + HEADER_SIZE,
			       numberOfBytesToRead);
			dataSpot += numberOfBytesToRead;
		}

		bytesRemaining -= numberOfBytesToRead;
	}

	return true;
}

//Given the data packet, send the header then the data
//Returns false if sensor does not ACK
bool sendPacket(struct bno080_api_desc *desc, uint8_t channelNumber,
		uint8_t dataLength)
{
	uint8_t packetLength =
		dataLength + HEADER_SIZE; //Add four bytes for the header
	uint8_t buffer[HEADER_SIZE + MAX_PACKET_SIZE] = { 0 };
	struct bno080_api_hal_ops *ops = desc->hal_ops;
	struct device *dev = desc->dev;

	shtpHeader[0] = packetLength & 0xFF;
	shtpHeader[1] = packetLength >> 8;
	shtpHeader[2] = channelNumber;
	shtpHeader[3] = sequenceNumber[channelNumber]++;

	memcpy(buffer, shtpHeader, HEADER_SIZE);
	memcpy(buffer + HEADER_SIZE, shtpData, dataLength);

	if (ops->write_multiple_byte(dev, buffer, dataLength + HEADER_SIZE) !=
	    (dataLength + HEADER_SIZE)) {
		return false;
	}

	dev_dbg(dev, "Sending packet:\n");
	printPacket(dev);

	return true;
}

//Pretty prints the contents of the current shtp header and data packets
void printPacket(struct device *dev)
{
	uint16_t packetLength;
	uint16_t printLength;

	packetLength =
		(((uint16_t)shtpHeader[1] << 8) | ((uint16_t)shtpHeader[0]));

	if (packetLength & (1 << 15)) {
		dev_dbg(dev, "[Continued packet]\n");
		packetLength &= ~(1 << 15);
	}

	if (packetLength == 0) {
		dev_dbg(dev, "Empty packet\n");
		return;
	}

	//Print the four byte header
	dev_dbg(dev, "Header: [%02x %02x %02x %02x]\n", shtpHeader[0],
		shtpHeader[1], shtpHeader[2], shtpHeader[3]);

	dev_dbg(dev, "Length: %d\n", packetLength);

	printLength = packetLength - HEADER_SIZE;

	if (printLength >= MAX_PACKET_SIZE)
		printLength = MAX_PACKET_SIZE;

	if (printLength > 0)
		dev_dbg(dev, "Body %d bytes [%02x ... %02x]\n", printLength,
			shtpData[0], shtpData[printLength - 1]);

	if (shtpHeader[2] == 0)
		dev_dbg(dev, "Channel: Command\n");
	else if (shtpHeader[2] == 1)
		dev_dbg(dev, "Channel: Executable\n");
	else if (shtpHeader[2] == 2)
		dev_dbg(dev, "Channel: Control\n");
	else if (shtpHeader[2] == 3)
		dev_dbg(dev, "Channel: Sensor-report\n");
	else if (shtpHeader[2] == 4)
		dev_dbg(dev, "Channel: Wake-report\n");
	else if (shtpHeader[2] == 5)
		dev_dbg(dev, "Channel: Gyro-vector\n");
	else
		dev_dbg(dev, "Channel: 0x%02x\n", shtpHeader[2]);
}

//Pretty prints the contents of the current shtp header (only)
void printHeader(struct device *dev)
{
	dev_dbg(dev, "Header: [%02x %02x %02x %02x]\n", shtpHeader[0],
		shtpHeader[1], shtpHeader[2], shtpHeader[3]);
}
