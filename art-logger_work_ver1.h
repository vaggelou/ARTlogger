/*
 * art-logger_work_ver1.h
 *
 *  Created on: 11 Ïêô 2016
 *      Author: Vaggelou
 */

#ifndef ART_LOGGER_WORK_VER1_H_
#define ART_LOGGER_WORK_VER1_H_


//ANALOG ITEM STRUCT
typedef struct
{
	//Raw data from the ADC buffer
	uint16_t *ui16AnalogDigits;

	//Signed value written in log file
	int32_t i32AnalogValue;

	//Floating point analog value
	double dAnalogValue;

	//Minimum bit value
	uint16_t ui16MinBitAnalogValue;

	//Multiplicator
	float fAnalogMult;

	//Precision in decimal digits
	uint16_t ui16Precision;

	//Offset
	int32_t i32AnalogOffset;

	//Minimum sensor value
	float fMinAnalogValue;

	//Maximum sensor value
	float fMaxAnalogValue;

	//Variable that enables the recording of the analog channel
	bool analogRec;

	//Is this channel used to start the acquisition?
	bool isTrig;

	//Analog channel name
	char *analogName;
}tAnalogItem;

//CAN ITEM STRUCT
typedef struct
{
	//CAN message number
	uint8_t ui8CANMsgNum;

	//CAN message ID
	uint32_t ui32CANMsgID;

	//CAN message mask
	uint32_t ui32CANMsgMask;

	//CAN message byte array
	uint8_t pui8MsgData[8];

	//Array with the 16bit raw data of the message
	uint16_t ui16RawCANData[4];

	//Array with the processed data
	int32_t i32ProcessedCANData[4];

	//Array with processed data in double format
	double dProcessedCANData[4];

	//Multiplier
	float fCANMult[4];

	//Offset
	int32_t i32CANOffset[4];

	//Precision
	uint16_t ui16CANPrecision[4];

	//Minimum sensor value
	float fMinCANValue[4];

	//Maximum sensor value
	float fMaxCANValue[4];

	//Minimum bit value
	uint16_t ui16MinBitCANValue[4];

	//CAN channel 1 name
	char CANName1[15];

	//CAN channel 2 name
	char CANName2[15];

	//CAN channel 3 name
	char CANName3[15];

	//CAN channel 4 name
	char CANName4[15];

	//Variable that enables the recording of the CAN channel
	bool CANRec;

	//Is this channel used to start the acquisition?
	bool CANIsTrig[4];

	//CAN channel interrupt flag
	bool bCANIntFlag;
}tCANItem;

//GPS struct
typedef struct
{
	char start[10];
	char timestamp[10];
	char validity[2];
	char lat[9];
	char latDir[2];
	char lon[10];
	char lonDir[2];
	char speed[4];
	char trueCourse[10];
	char datestamp[10];
	char variation[6];
	char eastWestCheck[6];
}GPSStruct;

//LOG RECORD STRUCT
typedef struct
{
	uint32_t ui32Seconds; //Logging seconds

	uint16_t ui16SubSeconds; //Logging milliseconds

	uint8_t ui8NumRecAnalogItems; //Number of recorded analog channels

	uint8_t ui8NumRecCANItems; //Number of recorded CAN channels

	int32_t *i32TriggerValue; //Pointer to the value used as the acquisition trigger

	int32_t i32Threshold; //Threshold value to start acquisition

	int32_t i32ThresholdValue;

	char logFileName[12];
}tLogRecord;


#endif /* ART_LOGGER_WORK_VER1_H_ */
