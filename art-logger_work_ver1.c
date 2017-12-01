/*
 * ART-LOGGER
 *
 * This is the main body of the ART Logger firmware
 * 
 * Author: Angelou Evangelos
 *
 */ 


#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/i2c.h"
#include "driverlib/adc.h"
#include "driverlib/can.h"
#include "driverlib/uart.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_can.h"
#include "fatfs/src/ff.h"
#include "sensorlib/hw_mpu9150.h"
#include "sensorlib/hw_ak8975.h"
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/ak8975.h"
#include "sensorlib/mpu9150.h"
#include "sensorlib/comp_dcm.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include "utils/cmdline.h"
#include "drivers/pinout.h"
#include "art-logger_work_ver1.h"


//********************************************************************
//--------------------------LOGGING STATES----------------------------
//********************************************************************
typedef enum
{
	LOGGING,
	NOT_LOGGING,
	STOP_LOGGING,
}tLoggerState;

static tLoggerState loggerState = NOT_LOGGING;
uint32_t ui32SystemClock;
tLogRecord demoRec;

//*********************************************************************
//--------------------------GUI VARIABLES------------------------------
//*********************************************************************
bool startLogging;

//********************************************************************
//---------------------FATFS VARIABLES--------------------------------
//********************************************************************
static FATFS driveObj;
static FIL fileObj;

//********************************************************************
//---------------------SYSTICK VARIABLES------------------------------
//********************************************************************
static volatile uint32_t g_pui32TimeStamp[2];
static volatile uint32_t ui32SysTickCount;
static volatile uint32_t ui32LastSysTickCount;

//********************************************************************
//------------------------ADC VARIABLES-------------------------------
//********************************************************************
//ADC buffer matrix
uint32_t ui32ADCBuffer[16];

//Vector of analog channels available
tAnalogItem analogChannelVector[16];

//*********************************************************************
//----------------------------CAN VARIABLES----------------------------
//*********************************************************************
//Vector with CAN1 channels available
tCANItem CAN1ItemsVector[16];

//CAN message object
tCANMsgObject CANMsgObj;

//CAN message error flag
bool bCANErrorFlag;

//*********************************************************************
//-------------------------GPS VARIABLES-------------------------------
//*********************************************************************
//UART6 interrupt flag
bool GPSIntFlag;

//GPS string
char GPSString[100];

//Log file GPS headers
char cGPSHeaders[] = "Latitude,Longitude,GPS Speed(knots),";

//Useful matrices for GPS data
char prevLati[9], longi[9], prevLongi[10], prevSpeed[4];


//*********************************************************************
//-----------------------MPU-9150 VARIABLES----------------------------
//*********************************************************************
//MPU-9150 I2C slave address
#define MPU9150_ADDR		0x68

//Floating point data from SensorLib code
float g_pfAccel[3];

//16bit value post translation from floating point
int16_t g_i16Accel[3];

//Global instance structure for the I2C master driver.
tI2CMInstance g_sI2CInst;

//Global instance structure for the MPU9150 sensor driver.
tMPU9150 g_sMPU9150Inst;

//Global flags to alert main that MPU9150 I2C transaction is complete
volatile uint_fast8_t g_vui8I2CDoneFlag;

//Global flags to alert main that MPU9150 I2C transaction error has occurred.
volatile uint_fast8_t g_vui8ErrorFlag;

//Accelerometer headers for .csv logging
char cAccelHeaders[] = "ACC_X(G),ACC_Y(G),ACC_Z(G),";


//*********************************************************************
//-------------------------UART FUNCTIONS------------------------------
//*********************************************************************
void ConfigureUART(void)
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    ROM_UARTClockSourceSet(UART0_BASE, UART_CLOCK_SYSTEM);
    UARTStdioConfig(0, 115200, ui32SystemClock);
}


//*********************************************************************
//---------------------------GPS FUNCTIONS-----------------------------
//*********************************************************************
//UART6 initialization function
void UART6Init(void)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART6);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);

	ROM_GPIOPinConfigure(GPIO_PP0_U6RX);
	ROM_GPIOPinConfigure(GPIO_PP1_U6TX);
	ROM_GPIOPinTypeUART(GPIO_PORTP_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	ROM_UARTConfigSetExpClk(UART6_BASE, ui32SystemClock, 9600, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
							UART_CONFIG_PAR_NONE));
}

//Initialize GPS struct variables
void GPSInit(GPSStruct *gps)
{
	int i;

	for(i = 0; i < 10; i++)
	{
		gps->start[i] = 0;
		gps->timestamp[i] = 0;
		gps->lon[i] = 0;
		gps->lat[i] = 0;
		gps->datestamp[i] = 0;
		gps->trueCourse[i] = 0;
	}
	for(i = 0; i < 6; i++)
	{
		gps->speed[i] = 0;
		gps->variation[i] = 0;
		gps->eastWestCheck[i] = 0;
	}
	for(i = 0; i < 2; i++)
	{
		gps->latDir[i] = 0;
		gps->lonDir[i] = 0;
		gps->validity[i] = 0;
	}
}

void PreviousToCurrentGPSData(GPSStruct *gps)
{
	int i;

	for(i = 0; i < sizeof(gps->lat); i++)
	{
		gps->lat[i] = prevLati[i];
	}
	for(i = 0; i < sizeof(gps->lon); i++)
	{
		gps->lon[i] = prevLongi[i];
	}
	for(i = 0; i < sizeof(gps->speed); i++)
	{
		gps->speed[i] = prevSpeed[i];
	}
}

void CurrentToPreviousGPSData(GPSStruct *gps)
{
	int i;

	for(i = 0; i < sizeof(gps->lat); i++)
	{
		prevLati[i] = gps->lat[i];
	}
	for(i = 0; i < sizeof(gps->lon); i++)
	{
		prevLongi[i] = gps->lon[i];
	}
	for(i = 0; i < sizeof(gps->speed); i++)
	{
		prevSpeed[i] = gps->speed[i];
	}
}

//Parse GPS into token of strings
void ParseTokenGPS(GPSStruct *gps, char *GPSData)
{
	int i = 0;
	int k = 0;
	int j;
	char temp[30];

	while(*GPSData)
	{
		while(*GPSData && *GPSData != ',')
		{
			temp[i++] = *GPSData++;
		}

		temp[i] = 0;
		if(*GPSData == ',')
		{
			GPSData++;
		}
		k++;

		if(i)
		{
			switch(k)
			{
				case 1:
					for(j = 0; j < i; j++)
					{
						gps->start[j] = temp[j];
					}
					break;
				case 2:
					for(j = 0; j < i; j++)
					{
						gps->timestamp[j] = temp[j];
					}
					break;
				case 3:
					for(j = 0; j < i; j++)
					{
						gps->validity[j] = temp[j];
					}
					break;
				case 4:
					for(j = 0; j < i; j++)
					{
						gps->lat[j] = temp[j];
					}
					break;
				case 5:
					for(j = 0; j < i; j++)
					{
						gps->latDir[j] = temp[j];
					}
					break;
				case 6:
					for(j = 0; j < i; j++)
					{
						gps->lon[j] = temp[j];
					}
					break;
				case 7:
					for(j = 0; j < i; j++)
					{
						gps->lonDir[j] = temp[j];
					}
					break;
				case 8:
					for(j = 0; j < i; j++)
					{
						gps->speed[j] = temp[j];
					}
					break;
				case 9:
					for(j = 0; j < i; j++)
					{
						gps->trueCourse[j] = temp[j];
					}
					break;
				case 10:
					for(j = 0; j < i; j++)
					{
						gps->datestamp[j] = temp[j];
					}
					break;
				case 11:
					for(j = 0; j < i; j++)
					{
						gps->variation[j] = temp[j];
					}
					break;
				case 12:
					for(j = 0; j < i; j++)
					{
						gps->eastWestCheck[j] = temp[j];
					}
					break;
				default:
					break;
			}
		}
		i = 0;
	}

	return;
}

void UARTIntHandler(void)
{
    uint32_t ui32Status;
    int i = 0;
    char GPSData;

    ui32Status = ROM_UARTIntStatus(UART6_BASE, true);
    ROM_UARTIntClear(UART6_BASE, ui32Status);

	GPSData = ROM_UARTCharGet(UART6_BASE);

	if(GPSData == '$')
	{
		do
		{
			GPSString[i] = GPSData;
			i++;
			GPSData = ROM_UARTCharGet(UART6_BASE);
		}while(GPSData != '\n');

		GPSString[i] = 0;
	}

    GPSIntFlag = 1;
}


//********************************************************************
//------------------------ADC FUNCTIONS-------------------------------
//
//ADC peripheral initialization
//The data logger uses 16 ADCs, 8 from each ADC peripheral
//It uses sequencer 0 of each peripheral
//
//********************************************************************
void InitADC(void)
{
	//Enable the ADC peripherals
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);

	//Enable the GPIO peripherals
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	//Assign the appropriate GPIO pins as ADC pins
	ROM_GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|
				GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);
	ROM_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|
				GPIO_PIN_4|GPIO_PIN_5);

	//Configure the ADC sequencers for each peripheral
	ROM_ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
	ROM_ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);

	//Configure the steps of the ADC0 peripheral data acquisition
	ROM_ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH10);
	ROM_ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH11);
	ROM_ADCSequenceStepConfigure(ADC0_BASE, 0, 2, ADC_CTL_CH15);
	ROM_ADCSequenceStepConfigure(ADC0_BASE, 0, 3, ADC_CTL_CH14);
	ROM_ADCSequenceStepConfigure(ADC0_BASE, 0, 4, ADC_CTL_CH13);
	ROM_ADCSequenceStepConfigure(ADC0_BASE, 0, 5, ADC_CTL_CH12);
	ROM_ADCSequenceStepConfigure(ADC0_BASE, 0, 6, ADC_CTL_CH3);
	ROM_ADCSequenceStepConfigure(ADC0_BASE, 0, 7, ADC_CTL_CH2|ADC_CTL_IE|ADC_CTL_END);

	//Configure the steps of the ADC1 peripheral data acquisition
	ROM_ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADC_CTL_CH1);
	ROM_ADCSequenceStepConfigure(ADC1_BASE, 0, 1, ADC_CTL_CH0);
	ROM_ADCSequenceStepConfigure(ADC1_BASE, 0, 2, ADC_CTL_CH9);
	ROM_ADCSequenceStepConfigure(ADC1_BASE, 0, 3, ADC_CTL_CH8);
	ROM_ADCSequenceStepConfigure(ADC1_BASE, 0, 4, ADC_CTL_CH16);
	ROM_ADCSequenceStepConfigure(ADC1_BASE, 0, 5, ADC_CTL_CH17);
	ROM_ADCSequenceStepConfigure(ADC1_BASE, 0, 6, ADC_CTL_CH18);
	ROM_ADCSequenceStepConfigure(ADC1_BASE, 0, 7, ADC_CTL_CH19|ADC_CTL_IE|ADC_CTL_END);

	//Setting the ADC reference to external 3V
	ROM_ADCReferenceSet(ADC0_BASE, ADC_REF_EXT_3V);
	ROM_ADCReferenceSet(ADC1_BASE, ADC_REF_EXT_3V);
}

void ADC0SS0Handler(void)
{
	//Clear the ADC interrupts
	ROM_ADCIntClear(ADC0_BASE, 0);

	//Acquisition of ADC data
	ROM_ADCSequenceDataGet(ADC0_BASE, 0, &ui32ADCBuffer[0]);
}

void ADC1SS0Handler(void)
{
	//Clear the ADC interrupts
	ROM_ADCIntClear(ADC1_BASE, 0);

	//Acquisition of ADC data
	ROM_ADCSequenceDataGet(ADC1_BASE, 0, &ui32ADCBuffer[8]);
}

//*******************************************************************************
//---------------------------SYSTICK FUNCTIONS-----------------------------------
//*******************************************************************************
void SysTickIntHandler(void)
{
	if(loggerState == LOGGING)
	{
		if(g_pui32TimeStamp[1] < 990)
		{
			g_pui32TimeStamp[1] += 10;
		}
		else
		{
			g_pui32TimeStamp[1] = 0;
			g_pui32TimeStamp[0]++;
		}
	}

    ui32SysTickCount++;
}


//*******************************************************************************
//--------------------------------CAN FUNCTIONS----------------------------------
//*******************************************************************************
void CAN1IntHandler(void)
{
    uint32_t ui32Status;
    int CANIdx;

    //Read the CAN interrupt status to find the cause of the interrupt
    ui32Status = ROM_CANIntStatus(CAN1_BASE, CAN_INT_STS_CAUSE);

    //If the cause is a controller status interrupt, then get the status
    if(ui32Status == CAN_INT_INTID_STATUS)
    {
        //Read the controller status to see if there is an error
        ui32Status = ROM_CANStatusGet(CAN1_BASE, CAN_STS_CONTROL);

        //Set a flag to indicate some errors may have occurred
        bCANErrorFlag = 1;
    }

    //Check which message caused the interrupt
    for(CANIdx = 0; CANIdx < 16; CANIdx++)
    {
    	if(CAN1ItemsVector[CANIdx].CANRec)
    	{
        	if(ui32Status == CAN1ItemsVector[CANIdx].ui8CANMsgNum)
        	{
        		ROM_CANIntClear(CAN1_BASE, CAN1ItemsVector[CANIdx].ui8CANMsgNum);
        		CAN1ItemsVector[CANIdx].bCANIntFlag = 1;
        		bCANErrorFlag = 0;
        	}
    	}
    }
}

//CAN bus initialization
void CANConfigure(void)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	ROM_GPIOPinConfigure(GPIO_PB0_CAN1RX);
	ROM_GPIOPinConfigure(GPIO_PB1_CAN1TX);
	GPIOPinTypeCAN(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	ROM_SysCtlPeripheralDisable(SYSCTL_PERIPH_CAN1);
	ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_CAN1);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN1);

	ROM_CANInit(CAN1_BASE);

	ROM_CANBitRateSet(CAN1_BASE, ui32SystemClock, 500000);

	ROM_CANIntEnable(CAN1_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);

	ROM_CANEnable(CAN1_BASE);
}

//Determine which CAN channels are to be recorded
void SetRecordingCANChannels(void)
{
	int canIdx;

	for(canIdx = 0; canIdx < 16; canIdx++)
	{
		if(CAN1ItemsVector[canIdx].CANRec)
		{
			CAN1ItemsVector[canIdx].ui8CANMsgNum = canIdx + 1;

			//Create a CAN message object
			CANMsgObj.ui32MsgID = CAN1ItemsVector[canIdx].ui32CANMsgID;
			CANMsgObj.ui32MsgIDMask = 0xfffff;
			CANMsgObj.ui32Flags = (MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER);
			CANMsgObj.ui32MsgLen = 8;
			CANMessageSet(CAN1_BASE, CAN1ItemsVector[canIdx].ui8CANMsgNum,
						&CANMsgObj, MSG_OBJ_TYPE_RX);
		}
	}
}

//Check if an interrupt from the CAN peripheral has occured
void GetCANMessage(void)
{
	int CANIdx, uIdx, Idx;
	uint16_t ui16Value1, ui16Value2;

	for(CANIdx = 0; CANIdx < 16; CANIdx++)
	{
		if(CAN1ItemsVector[CANIdx].CANRec)
		{
			if(CAN1ItemsVector[CANIdx].bCANIntFlag)
			{
				CANMsgObj.pui8MsgData = CAN1ItemsVector[CANIdx].pui8MsgData;
				ROM_CANMessageGet(CAN1_BASE, CAN1ItemsVector[CANIdx].ui8CANMsgNum,
									&CANMsgObj, 0);
				CAN1ItemsVector[CANIdx].bCANIntFlag = 0;

				//Convert 2 byte values to one 16-bit value
				uIdx = 0;
				Idx = 0;
				while(uIdx < CANMsgObj.ui32MsgLen)
				{
					ui16Value1 = (uint16_t)(CANMsgObj.pui8MsgData[uIdx]);
					ui16Value2 = (uint16_t)(CANMsgObj.pui8MsgData[uIdx + 1]);
					CAN1ItemsVector[CANIdx].ui16RawCANData[Idx] =
												(ui16Value1 << 8) | ui16Value2;
					uIdx += 2;
					Idx++;
				}
			}
		}
	}
}


//*****************************************************************************
//----------------------------MPU-9150 FUNCTIONS-------------------------------
//*****************************************************************************
//MPU-9150 callback function
//This function is called when the I2C data transaction is completed
void MPU9150AppCallback(void *pvCallbackData, uint_fast8_t ui8Status)
{
    //If the transaction succeeded set the data flag to indicate to
    //application that this transaction is complete and data may be ready.
    if(ui8Status == I2CM_STATUS_SUCCESS)
    {
        g_vui8I2CDoneFlag = 1;
    }

    //Store the most recent status in case it was an error condition
    g_vui8ErrorFlag = ui8Status;
}

//This function waits for the I2C transaction to complete
void MPU9150AppI2CWait(void)
{
    //Put the processor to sleep while we wait for the I2C driver to
    //indicate that the transaction is complete.
    while((g_vui8I2CDoneFlag == 0) && (g_vui8ErrorFlag == 0))
    {
    }

    //If an error occurred call the error handler immediately.
    if(g_vui8ErrorFlag)
    {
        UARTprintf("ERROR OCCURRED\n");
    }

    //Clear the data flag for next use.
    g_vui8I2CDoneFlag = 0;
}

//Called by the NVIC as a result of GPIO port B interrupt event. For this
//application GPIO port B pin 2 is the interrupt line for the MPU9150
void IntGPIOb(void)
{
    unsigned long ulStatus;

    ulStatus = GPIOIntStatus(GPIO_PORTF_BASE, true);

    //Clear all the pin interrupts that are set
    GPIOIntClear(GPIO_PORTF_BASE, ulStatus);

    if(ulStatus & GPIO_PIN_1)
    {
        //MPU9150 Data is ready for retrieval and processing.
        MPU9150DataRead(&g_sMPU9150Inst, MPU9150AppCallback, &g_sMPU9150Inst);
    }
}

//Called by the NVIC as a result of I2C3 Interrupt. I2C3 is the I2C connection
//to the MPU9150.
void MPU9150I2CIntHandler(void)
{
    //Pass through to the I2CM interrupt handler provided by sensor library.
    //This is required to be at application level so that I2CMIntHandler can
    //receive the instance structure pointer as an argument.
    I2CMIntHandler(&g_sI2CInst);
}

void InitializeMPU9150(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);

    GPIOPinConfigure(GPIO_PG0_I2C1SCL);
    GPIOPinConfigure(GPIO_PG1_I2C1SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTG_BASE, GPIO_PIN_0);
    GPIOPinTypeI2C(GPIO_PORTG_BASE, GPIO_PIN_1);
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_1);
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_1);
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_FALLING_EDGE);

    //Initialize I2C9 Peripheral
    I2CMInit(&g_sI2CInst, I2C1_BASE, INT_I2C1, 0xff, 0xff, ui32SystemClock);

    //Initialize the MPU9150 Driver.
    MPU9150Init(&g_sMPU9150Inst, &g_sI2CInst, MPU9150_ADDR,
                MPU9150AppCallback, &g_sMPU9150Inst);

    //Wait for transaction to complete
    MPU9150AppI2CWait();

    //Write application specific sensor configuration such as filter settings
    //and sensor range settings.
    g_sMPU9150Inst.pui8Data[0] = MPU9150_CONFIG_DLPF_CFG_94_98;
    g_sMPU9150Inst.pui8Data[1] = MPU9150_GYRO_CONFIG_FS_SEL_250;
    g_sMPU9150Inst.pui8Data[0] = (MPU9150_ACCEL_CONFIG_ACCEL_HPF_5HZ |
                                  MPU9150_ACCEL_CONFIG_AFS_SEL_2G);
    MPU9150Write(&g_sMPU9150Inst, MPU9150_O_CONFIG, g_sMPU9150Inst.pui8Data, 1,
                 MPU9150AppCallback, &g_sMPU9150Inst);

    //Wait for transaction to complete
    MPU9150AppI2CWait();

    //Configure the data ready interrupt pin output of the MPU9150.
    g_sMPU9150Inst.pui8Data[0] = MPU9150_INT_PIN_CFG_INT_LEVEL |
                                    MPU9150_INT_PIN_CFG_INT_RD_CLEAR |
                                    MPU9150_INT_PIN_CFG_LATCH_INT_EN;
    g_sMPU9150Inst.pui8Data[1] = MPU9150_INT_ENABLE_DATA_RDY_EN;
    MPU9150Write(&g_sMPU9150Inst, MPU9150_O_INT_PIN_CFG,
                 g_sMPU9150Inst.pui8Data, 2, MPU9150AppCallback,
                 &g_sMPU9150Inst);

    // Wait for transaction to complete
    MPU9150AppI2CWait();
}

//Print accelerometer data for debugging
void PrintAccelerometerData(int16_t *accelData)
{
	int16_t i16Accel;
	int i;

	if(g_vui8ErrorFlag)
	{
		UARTprintf("ERROR OCCURRED (AGAIN...)\n");
	}
	else
	{
	    for(i = 0; i < 3; i++)
	    {
	    	i16Accel = accelData[i];

	    	if(i == 0)
	    	{
	    		UARTprintf("X: ");
	    	}
	    	else if(i == 1)
	    	{
	    		UARTprintf("Y: ");
	    	}
	    	else
	    	{
	    		UARTprintf("Z: ");
	    	}

	    	if(i16Accel < 0)
	    	{
	    		i16Accel *= -1;
	    		UARTprintf("-");
	    	}
	    	else
	    	{
	    		i16Accel *= 1;
	    	}

	    	UARTprintf("%d.%03u G  ", i16Accel / 1000, i16Accel % 1000);
	    	if(i == 2)
	    	{
	    		UARTprintf("\n");
	    	}
	    }
	}
}

//Restart the accelerometer peripherals
void RestartMPU9150(void)
{
	ROM_IntDisable(INT_GPIOB);
	ROM_IntDisable(INT_I2C3);

	SysCtlPeripheralDisable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralDisable(SYSCTL_PERIPH_GPIOK);
	SysCtlPeripheralDisable(SYSCTL_PERIPH_I2C3);

	ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOB);
	ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOK);
	ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C3);
}


//*******************************************************************************
//-----------------------ACQUISITION FUNCTIONS-----------------------------------
//*******************************************************************************
void ProcessDataItems(tLogRecord *record, GPSStruct *gps)
{
	int recAnalogIdx, recCANIdx, CANIdx;
	uint16_t ui16AnalogMultPrec;
	int32_t i32AnalogDigits, i32AnalogOffset;
	uint16_t CANMultPrec;
	int32_t rawCANData[4], CANOffset;

	//Write the seconds and subseconds values on the record
	record->ui32Seconds = g_pui32TimeStamp[0];
	record->ui16SubSeconds = (uint16_t)g_pui32TimeStamp[1];

    //GPS information get
	if(GPSIntFlag)
	{
		GPSIntFlag = 0;

		GPSInit(gps);
		ParseTokenGPS(gps, GPSString);

		if(strcmp(gps->start, "$GPRMC") == 0)
		{
			if(strcmp(gps->validity, "A") == 0)
			{
				CurrentToPreviousGPSData(gps);
			}
			else
			{
				PreviousToCurrentGPSData(gps);
			}
		}
		else
		{
			PreviousToCurrentGPSData(gps);
		}
	}

	//Write the processed ADC values in the record
	for(recAnalogIdx = 0; recAnalogIdx < 16; recAnalogIdx++)
	{
		if(analogChannelVector[recAnalogIdx].analogRec)
		{
			i32AnalogOffset = analogChannelVector[recAnalogIdx].i32AnalogOffset;
			ui16AnalogMultPrec = (uint16_t)(analogChannelVector[recAnalogIdx].fAnalogMult*
					analogChannelVector[recAnalogIdx].ui16Precision);
			i32AnalogDigits = (int32_t)((*analogChannelVector[recAnalogIdx].ui16AnalogDigits) -
					analogChannelVector[recAnalogIdx].ui16MinBitAnalogValue);
			analogChannelVector[recAnalogIdx].i32AnalogValue =
					(int32_t)(ui16AnalogMultPrec*i32AnalogDigits + i32AnalogOffset);
			analogChannelVector[recAnalogIdx].dAnalogValue = (double)(analogChannelVector[recAnalogIdx].i32AnalogValue /
					((float)analogChannelVector[recAnalogIdx].ui16Precision));
		}
	}

    //Write the processed CAN values in the record
    for(recCANIdx = 0; recCANIdx < 16; recCANIdx++)
    {
    	if(CAN1ItemsVector[recCANIdx].CANRec)
    	{
        	for(CANIdx = 0; CANIdx < 4; CANIdx++)
        	{
            	CANOffset = CAN1ItemsVector[recCANIdx].i32CANOffset[CANIdx];
            	CANMultPrec = (uint16_t)(CAN1ItemsVector[recCANIdx].fCANMult[CANIdx]*
            			CAN1ItemsVector[recCANIdx].ui16CANPrecision[CANIdx]);
            	rawCANData[CANIdx] = (int32_t)(CAN1ItemsVector[recCANIdx].ui16RawCANData[CANIdx] -
            			CAN1ItemsVector[recCANIdx].ui16MinBitCANValue[CANIdx]);
            	CAN1ItemsVector[recCANIdx].i32ProcessedCANData[CANIdx] =
        					(int32_t)(CANMultPrec*rawCANData[CANIdx] + CANOffset);
            	CAN1ItemsVector[recCANIdx].dProcessedCANData[CANIdx] =
            			(double)(CAN1ItemsVector[recCANIdx].i32ProcessedCANData[CANIdx] /
            					(float)CAN1ItemsVector[recCANIdx].ui16CANPrecision[CANIdx]);
        	}
    	}
    }

    //Get floating point version of the Accel Data in m/s^2.
    MPU9150DataAccelGetFloat(&g_sMPU9150Inst, &g_pfAccel[0], &g_pfAccel[1],
                                 &g_pfAccel[2]);

    g_i16Accel[0]= (int16_t)((g_pfAccel[0] / 9.81f)*1000.f);
    g_i16Accel[1]= (int16_t)((g_pfAccel[1] / 9.81f)*1000.f);
    g_i16Accel[2]= (int16_t)((g_pfAccel[2] / 9.81f)*1000.f);

    //Print accelerometer to serial monitor for debugging
    PrintAccelerometerData(g_i16Accel);

    //Restart Accelerometer if an error occurred
//    if(g_vui8ErrorFlag)
//    {
//    	UARTprintf("RESTARTING ACCELEROMETER...\n");
//    	RestartMPU9150();
//    	InitializeMPU9150();
//    }
}

void DAQInit(tLogRecord *record)
{
	int analogIdx;

	//ADC peripheral initialization
	InitADC();

	//ADC channels-to-Analog item vector binding
	for(analogIdx = 0; analogIdx < 16; analogIdx++)
	{
		analogChannelVector[analogIdx].ui16AnalogDigits = (uint16_t *)&ui32ADCBuffer[analogIdx];
		analogChannelVector[analogIdx].ui16Precision = 1;
		analogChannelVector[analogIdx].fAnalogMult = 1;
	}

	//Initializing CAN
	CANConfigure();

	//Setting the SysTick period
	ROM_SysTickPeriodSet(ui32SystemClock/100);

	//Initialize UART6 port used for the GPS sensor
	UART6Init();

	//Initialize I2C3 port and the MPU9150 peripherals
	InitializeMPU9150();
}

void DAQStart(tLogRecord *record)
{
	//Initialize the time stamp variables
	g_pui32TimeStamp[0] = 0;
	g_pui32TimeStamp[1] = 0;

	//Flush the ADC buffers in case there are old data left
	ROM_ADCSequenceEnable(ADC0_BASE, 0);
	ROM_ADCSequenceEnable(ADC1_BASE, 0);
	ROM_ADCSequenceDataGet(ADC0_BASE, 0, &ui32ADCBuffer[0]);
	ROM_ADCSequenceDataGet(ADC1_BASE, 0, &ui32ADCBuffer[8]);

	//Enable the ADC interrupts
	ROM_ADCIntClear(ADC0_BASE, 0);
	ROM_ADCIntClear(ADC1_BASE, 0);
	ROM_ADCIntEnable(ADC0_BASE, 0);
	ROM_ADCIntEnable(ADC1_BASE, 0);
	ROM_IntEnable(INT_ADC0SS0_TM4C129);
	ROM_IntEnable(INT_ADC1SS0_TM4C129);

	//Enable the SysTick interrupts
	ROM_SysTickIntEnable();

	//Enable SysTick
	ROM_SysTickEnable();

	//Enable CAN1 interrupt
	ROM_IntEnable(INT_CAN1_TM4C129);

	//Enable UART6 for GPS module
	ROM_IntEnable(INT_UART6);
	ROM_UARTIntEnable(UART6_BASE, UART_INT_RX | UART_INT_RT);
}

int DAQRun(tLogRecord *record, GPSStruct *gps)
{
	//SystTick interrupt check
	if(ui32LastSysTickCount != ui32SysTickCount)
//	if((ui32LastSysTickCount != ui32SysTickCount) & (g_vui8I2CDoneFlag == 1))
	{
		ui32LastSysTickCount = ui32SysTickCount;

		ROM_ADCProcessorTrigger(ADC0_BASE, 0);
		ROM_ADCProcessorTrigger(ADC1_BASE, 0);

		//Check if an interrupt from the CAN peripheral has occured
		GetCANMessage();

		//Process the items and pass them to the log record
		ProcessDataItems(record, gps);

		return(0);
	}
	else
	{
		return(1);
	}
}

void DAQStop(void)
{
	ROM_IntDisable(INT_ADC0SS0_TM4C129);
	ROM_IntDisable(INT_ADC1SS0_TM4C129);

	ROM_ADCSequenceDisable(ADC0_BASE, 0);
	ROM_ADCSequenceDisable(ADC1_BASE, 0);
}

//Choose the channel to use as threshold
void SetThresholdValue(tLogRecord *record)
{
	int thresIdx, canIdx;

	//Initializing analog channel 1 --> ONLY FOR TESTING
	analogChannelVector[0].analogRec = 1;
	analogChannelVector[0].isTrig = 1;
	analogChannelVector[0].analogName = "AIN1";
	record->i32Threshold = 300;

	for(thresIdx = 0; thresIdx < 16; thresIdx++)
	{
		if(analogChannelVector[thresIdx].analogRec)
		{
			if(analogChannelVector[thresIdx].isTrig)
			{
				record->i32TriggerValue = &analogChannelVector[thresIdx].i32AnalogValue;
				record->i32ThresholdValue = (int64_t)(record->i32Threshold*
						analogChannelVector[thresIdx].ui16Precision);
			}
		}
	}

	for(thresIdx = 0; thresIdx < 16; thresIdx++)
	{
		if(CAN1ItemsVector[thresIdx].CANRec)
		{
			for(canIdx = 0; canIdx < 4; canIdx++)
			{
				if(CAN1ItemsVector[thresIdx].CANIsTrig[canIdx])
				{
					record->i32TriggerValue = &CAN1ItemsVector[thresIdx].i32ProcessedCANData[canIdx];
					record->i32ThresholdValue =
							record->i32Threshold*CAN1ItemsVector[thresIdx].ui16CANPrecision[canIdx];
				}
			}
		}
	}
}


//*******************************************************************
//----------------------SD CARD FUNCTIONS----------------------------
//*******************************************************************
void SDCardOpenLogFile(tLogRecord *record)
{
	FRESULT iFResult;
	int headerIdx = 0;
	uint8_t headerCount, commaCount, timeCount, breakCount;

	iFResult = f_mount(0, &driveObj);
	if(iFResult != FR_OK)
	{
		UARTprintf("COULD NOT MOUNT THE DRIVE\n");
	}

//	iFResult = f_open(&fileObj, record->logFileName, FA_WRITE|FA_OPEN_ALWAYS);
	iFResult = f_open(&fileObj, "dokimi2.csv", FA_WRITE|FA_OPEN_ALWAYS);
	if(iFResult != FR_OK)
	{
		UARTprintf("COULD NOT OPEN THE FILE\n");
	}

	iFResult = f_lseek(&fileObj, f_size(&fileObj));
	if(iFResult != FR_OK)
	{
		UARTprintf("COULD NOT FIND FREE SPACE\n");
	}

	//Write "Time" header
	iFResult = f_write(&fileObj, "Time,", 5, (UINT *)&timeCount);
	if(iFResult != FR_OK)
	{
		UARTprintf("COULD NOT WRITE TIME\n");
	}

	iFResult = f_write(&fileObj, cGPSHeaders, sizeof(cGPSHeaders), (UINT *)&headerCount);
	if(iFResult != FR_OK)
	{
		UARTprintf("COULD NOT WRITE GPS HEADERS\n");
	}

	iFResult = f_write(&fileObj, cAccelHeaders, sizeof(cAccelHeaders), (UINT *)&headerCount);
	if(iFResult != FR_OK)
	{
		UARTprintf("COULD NOT WRITE ACCELEROMETER HEADERS\n");
	}

	//Write analog channels' headers
	for(headerIdx = 0; headerIdx < 16; headerIdx++)
	{
		if(analogChannelVector[headerIdx].analogRec)
		{
			iFResult = f_write(&fileObj, analogChannelVector[headerIdx].analogName,
				sizeof(analogChannelVector[headerIdx].analogName),
						(UINT *)&headerCount);
			if(iFResult != FR_OK)
			{
				UARTprintf("COULD NOT WRITE ROW %i\n", headerIdx);
			}
			if(headerIdx < 15)
			{
				iFResult = f_write(&fileObj, ",", 1, (UINT *)&commaCount);
				if(iFResult != FR_OK)
				{
					UARTprintf("COULD NOT WRITE COMMA %i\n", headerIdx);
				}
			}
		}
	}

	//Write CAN message headers
	for(headerIdx = 0; headerIdx < 16; headerIdx++)
	{
		if(CAN1ItemsVector[headerIdx].CANRec)
		{
			iFResult = f_write(&fileObj, CAN1ItemsVector[headerIdx].CANName1,
					sizeof(CAN1ItemsVector[headerIdx].CANName1),
								(UINT *)&headerCount);
			if(iFResult != FR_OK)
			{
				UARTprintf("COULD NOT WRITE CAN ROW %i\n", headerIdx);
			}
			iFResult = f_write(&fileObj, ",", 1, (UINT *)&commaCount);
			if(iFResult != FR_OK)
			{
				UARTprintf("COULD NOT WRITE CAN COMMA %i\n", headerIdx);
			}

			iFResult = f_write(&fileObj, CAN1ItemsVector[headerIdx].CANName2,
					sizeof(CAN1ItemsVector[headerIdx].CANName2),
							(UINT *)&headerCount);
			if(iFResult != FR_OK)
			{
				UARTprintf("COULD NOT WRITE CAN ROW %i\n", headerIdx);
			}
			iFResult = f_write(&fileObj, ",", 1, (UINT *)&commaCount);
			if(iFResult != FR_OK)
			{
				UARTprintf("COULD NOT WRITE CAN COMMA %i\n", headerIdx);
			}

			iFResult = f_write(&fileObj, CAN1ItemsVector[headerIdx].CANName3,
					sizeof(CAN1ItemsVector[headerIdx].CANName3),
							(UINT *)&headerCount);
			if(iFResult != FR_OK)
			{
				UARTprintf("COULD NOT WRITE CAN ROW %i\n", headerIdx);
			}
			iFResult = f_write(&fileObj, ",", 1, (UINT *)&commaCount);
			if(iFResult != FR_OK)
			{
				UARTprintf("COULD NOT WRITE CAN COMMA %i\n", headerIdx);
			}

			iFResult = f_write(&fileObj, CAN1ItemsVector[headerIdx].CANName4,
					sizeof(CAN1ItemsVector[headerIdx].CANName4),
							(UINT *)&headerCount);
			if(iFResult != FR_OK)
			{
				UARTprintf("COULD NOT WRITE CAN ROW %i\n", headerIdx);
			}
			iFResult = f_write(&fileObj, ",", 1, (UINT *)&commaCount);
			if(iFResult != FR_OK)
			{
				UARTprintf("COULD NOT WRITE CAN COMMA %i\n", headerIdx);
			}
		}
	}
	iFResult = f_write(&fileObj, "\n", 1, (UINT *)&breakCount);
	if(iFResult != FR_OK)
	{
		UARTprintf("COULD NOT WRITE BREAK CHAR\n");
	}
}

void SDCardWriteLoggedData(tLogRecord *record, GPSStruct *gps)
{
	int dataIdx;
	int idx, accelIdx;
	int8_t printOK;
	FRESULT iFResult;
	uint8_t byteCount;
	uint8_t dataCount;
	uint8_t commaCount;
	uint16_t CANPrecision, analogPrecision;
	uint32_t ui32PositiveAnalogValue, ui32PositiveCANValue;
	int32_t i32Analog, i32CAN;
	int16_t i16Accel;

	//Write time data
	printOK = f_printf(&fileObj, "%u.%03u,", record->ui32Seconds, record->ui16SubSeconds);
	if(printOK == -1)
	{
		UARTprintf("COULD NOT WRITE TIME\n");
	}

    //Write GPS data into SD card
    iFResult = f_write(&fileObj, gps->lat, sizeof(gps->lat), (UINT *)&dataCount);
	if(iFResult != FR_OK)
	{
		UARTprintf("COULD NOT WRITE LATITUDE\n");
	}
	iFResult = f_write(&fileObj, ",", 1, (UINT *)&commaCount);
	if(iFResult != FR_OK)
	{
		UARTprintf("COULD NOT WRITE GPS COMMA\n");
	}

    for(idx = 0; idx < 9; idx++)
    {
    	longi[idx] = gps->lon[idx + 1];
    }
    iFResult = f_write(&fileObj, longi, sizeof(longi), (UINT *)&dataCount);
	if(iFResult != FR_OK)
	{
		UARTprintf("COULD NOT WRITE LONGITUDE\n");
	}
	iFResult = f_write(&fileObj, ",", 1, (UINT *)&commaCount);
	if(iFResult != FR_OK)
	{
		UARTprintf("COULD NOT WRITE GPS COMMA\n");
	}

    iFResult = f_write(&fileObj, gps->speed, sizeof(gps->speed), (UINT *)&dataCount);
	if(iFResult != FR_OK)
	{
		UARTprintf("COULD NOT WRITE GPS SPEED\n");
	}
	iFResult = f_write(&fileObj, ",", 1, (UINT *)&commaCount);
	if(iFResult != FR_OK)
	{
		UARTprintf("COULD NOT WRITE GPS COMMA\n");
	}

	//Write accelerometer data into SD card file
	for(accelIdx = 0; accelIdx < 3; accelIdx++)
	{
		i16Accel = g_i16Accel[accelIdx];

		if(i16Accel < 0)
		{
			i16Accel *= -1;
			iFResult = f_write(&fileObj, "-", 1, (UINT *)&commaCount);
			if(iFResult != FR_OK)
			{
				UARTprintf("COULD NOT WRITE PROSIMO\n");
			}
		}
		else
		{
			i16Accel *= 1;
		}

		printOK = f_printf(&fileObj, "%d.%03u,", i16Accel / 1000, i16Accel % 1000);
		if(printOK == -1)
		{
			UARTprintf("COULD NOT WRITE ACCELEROMETER DATA\n");
		}
	}

	//Write analog channel data
	for(dataIdx = 0; dataIdx < 16; dataIdx++)
	{
		if(analogChannelVector[dataIdx].analogRec)
		{
			analogPrecision = analogChannelVector[dataIdx].ui16Precision;
			i32Analog = analogChannelVector[dataIdx].i32AnalogValue;

			if(i32Analog < 0)
			{
				i32Analog *= -1;
				iFResult = f_write(&fileObj, "-", 1, (UINT *)&dataCount);
			}
			else
			{
				i32Analog *= 1;
			}

			ui32PositiveAnalogValue = (uint32_t)i32Analog;
			printOK = f_printf(&fileObj, "%u.%03u,", ui32PositiveAnalogValue / analogPrecision,
					ui32PositiveAnalogValue % analogPrecision);
		}
	}

    //Write CAN message data
	for(dataIdx = 0; dataIdx < 16; dataIdx++)
	{
		if(CAN1ItemsVector[dataIdx].CANRec)
		{
			for(idx = 0; idx < 4; idx++)
			{
				CANPrecision = CAN1ItemsVector[dataIdx].ui16CANPrecision[idx];
				i32CAN = CAN1ItemsVector[dataIdx].i32ProcessedCANData[idx];

				if(i32CAN < 0)
				{
					i32CAN *= -1;
					iFResult = f_write(&fileObj, "-", 1, (UINT *)&dataCount);
				}
				else
				{
					i32CAN *= 1;
				}

				ui32PositiveCANValue = (uint32_t)i32CAN;
				printOK = f_printf(&fileObj, "%u.%03u,", ui32PositiveCANValue / CANPrecision,
						ui32PositiveCANValue % CANPrecision);
				if(printOK == -1)
				{
					UARTprintf("COULD NOT WRITE CAN DATA ROW\n");
				}
			}
		}
	}

	f_write(&fileObj, "\n", 1, (UINT *)&byteCount);
}

void SDCardCloseFile(void)
{
	f_close(&fileObj);
	f_mount(0, NULL);
}


//********************************************************************
//-------------------------ART-LOGGER MAIN----------------------------
//********************************************************************
int main(void)
{
	tLogRecord *record = &demoRec;
	GPSStruct gps;
	ui32SysTickCount = 0;
	ui32LastSysTickCount = 0;
	startLogging = 0;
	GPSIntFlag = 0;

	//Enable lazy stacking
	ROM_FPULazyStackingEnable();

	//Configure the system clock to 25MHz
	ui32SystemClock = MAP_SysCtlClockFreqSet((SYSCTL_OSC_INT|SYSCTL_USE_PLL|SYSCTL_CFG_VCO_320), 16000000);

	ConfigureUART();

	while(1)
	{
		//Initialize the data acquisition module
		DAQInit(record);

		//Starting the acquisition of data
		DAQStart(record);

		//Enable interrupts to the processor
		ROM_IntMasterEnable();

//		while(!startLogging)
//		{
//			SetRecordingCANChannels();
//			DAQRun(record, &gps);
//		}
//		startLogging = 0;

		//Set the value that will be used as threshold to start logging
		SetThresholdValue(record);

		//Mount the microSD card and open a .csv file
		SDCardOpenLogFile(record);

		//PB2 pin for MPU9150 interrupt enable
		ROM_IntEnable(INT_GPIOF);
		ROM_IntEnable(INT_I2C1);

		//Main program loop
		while(1)
		{
			if(!DAQRun(record, &gps))
			{
				if((*record->i32TriggerValue < record->i32ThresholdValue) &&
							(loggerState == NOT_LOGGING))
				{
					UARTprintf("NOT LOGGING\n");
				}
				else if(*record->i32TriggerValue > record->i32ThresholdValue)
				{
					loggerState = LOGGING;

					UARTprintf("LOGGING\n");

					SDCardWriteLoggedData(record, &gps);
				}
				else if((*record->i32TriggerValue < 3*record->i32ThresholdValue) &&
							(loggerState == LOGGING))
				{
					loggerState = NOT_LOGGING;

					UARTprintf("NOT LOGGING\n");

					DAQStop();

					SDCardCloseFile();

					ROM_IntDisable(INT_GPIOF);
					ROM_IntDisable(INT_I2C1);

					break;
				}
			}
		}
	}
}
