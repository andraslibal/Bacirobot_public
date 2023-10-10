/*
 * I2CDevices.h
 *
 *  Created on: Feb 23, 2022
 *      Author: Arthur
 */

#ifndef INC_I2CDEVICES_H_
#define INC_I2CDEVICES_H_

// Settings
/*
 *
 *
 *
 */


//==================================================================================================================== USECASE START
/*
 *
 *
 *
 */
//==================================================================================================================== USECASE END


//#define I2CDEVICES_DEBUG_MODE

//#define I2CVEML3328_DEBUG_MODE
//#define I2CTSL24C512_DEBUG_MODE
#define I2CTSL25911_DEBUG_MODE

// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);	//  debug on PC13 HIGH
// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);  //	debug on PC13 LOW
//====================================================================================================================

// custom data structures
//==================================================================================================================== CUSTOM_DATA_STRUCTS_START




typedef enum
{
	UNDEFINED		= 0U,		/*  */
	RIGHT   		= 1U,		/*  */
	LEFT			= 2U		/*  */
}SIDE;

typedef enum
{
	UNKNOWN_DATASTATE			= 0U,		/* it is used initially, before the structure is initialized  */
	NEW_DATA_IS_AVAILABLE		= 1U,		/* new data is available, it is set if the sensor reading succeed */
	NO_NEW_DATA_IS_AVAILABLE	= 2U,		/* the last data was already read out */
}NEW_DATA_AVAILABLE;

typedef enum
{
	CALLBACK_NOT_RECEIVED			= 0U,			/*  */
	CALLBACK_RECEIVED				= 1U			/*  */
}I2C_CALLBACK_STATUS;


// ------------------------------ VEML3328 ------------------------------- START
#define ERROR_VEML3328_01		(0x0001<<0U)		/* error on the I2C reading loop:  HandleRightVEML3328Loop() or HandleLeftVEML3328Loop()*/
#define ERROR_VEML3328_02		(0x0001<<1U)		/* error on the I2C reading loop:  HandleRightVEML3328Loop() or HandleLeftVEML3328Loop()*/
#define ERROR_VEML3328_03		(0x0001<<2U)		/* error on the I2C reading loop:  HandleRightVEML3328Loop() or HandleLeftVEML3328Loop()*/
#define ERROR_VEML3328_04		(0x0001<<3U)		/* error on the I2C reading loop:  HandleRightVEML3328Loop() or HandleLeftVEML3328Loop()*/
#define ERROR_VEML3328_05		(0x0001<<4U)		/* error on the I2C reading loop:  HandleRightVEML3328Loop() or HandleLeftVEML3328Loop()*/
#define ERROR_VEML3328_06		(0x0001<<5U)		/* error on the I2C reading loop:  HandleRightVEML3328Loop() or HandleLeftVEML3328Loop()*/
#define ERROR_VEML3328_07		(0x0001<<6U)		/* error on the I2C reading loop:  HandleRightVEML3328Loop() or HandleLeftVEML3328Loop()*/
#define ERROR_VEML3328_08		(0x0001<<7U)		/* error on the I2C reading loop:  HandleRightVEML3328Loop() or HandleLeftVEML3328Loop()*/
#define ERROR_VEML3328_09		(0x0001<<8U)		/* error on the I2C reading timeout:  VEML3328RightTimeout100HZ() or VEML3328LeftTimeout100HZ()*/

typedef enum
{
	NoI2CReadingInProgress			= 0U,			/* the interrogation of the VEML sensor is not in progress */
	I2CReadingConfigRegister		= 1U,			/* it was sent the command to read the configuration register from address 0x00 */
	I2CSetConfigRegister			= 2U,			/* the write command was sent to POWER ON the sensor in case the configuration register has  SD1==1 or/and SD0==1 */
	I2CReadingClearData				= 3U,			/* it was sent the command to read the CLEAR data from the sensor and the RX callback is expected to confirm reception of the data from the sensor */
	I2CReadingRedData				= 4U,			/* it was sent the command to read the RED data from the sensor and the RX callback is expected to confirm reception of the data from the sensor */
	I2CReadingGreenData				= 5U,			/* it was sent the command to read the GREEN data from the sensor and the RX callback is expected to confirm reception of the data from the sensor */
	I2CReadingBlueData				= 6U,			/* it was sent the command to read the BLUE data from the sensor and the RX callback is expected to confirm reception of the data from the sensor */
	I2CReadingIRData				= 7U,			/* it was sent the command to read the IR data from the sensor and the RX callback is expected to confirm reception of the data from the sensor */
	//I2CReadingError					= 9U,			/* error occurred during the I2C reading operation */
	I2CReadingDone					= 10U			/* the interrogation of the sensor is complete for this round, the results are stored in the VEML3328Result structure and the NEW_DATA_AVAILABLE flag was set */
}VEML3328_I2C_STATE_MACHINE;

typedef struct
{
	uint16_t						uiError;				/* it is set if some I2C error code is detected during the I2C operations. all the results are also set to 0 in case of some I2C Error */
	uint16_t						uiConfiguration;		/* represents the register 0x00 content from VEML3328 */
	uint16_t						uiClear;				/* represents the CLEAR measurement result from the 0x04 command */
	uint16_t						uiRed;					/* represents the RED measurement result from the 0x05 command */
	uint16_t						uiGreen;				/* represents the GREEN measurement result from the 0x06 command */
	uint16_t						uiBlue;					/* represents the BLUE measurement result from the 0x07 command */
	uint16_t						uiIR;					/* represents the IR measurement result from the 0x08 command */
	NEW_DATA_AVAILABLE				NewMeasuredData;		/* it is set to "NEW_DATA_IS_AVAILABLE"  after the reading is done with all the registers */
}VEML3328Result;

typedef struct
{
	SIDE							SensorSide;				/* LEFT or RIGHT Side of the robot */
	uint16_t						uiI2CAddress;			/* used to store the I2C Address of the sensor */
	VEML3328_I2C_STATE_MACHINE		I2CReadingState;		/* used to keep track of the sensor reading steps for a complete set of data */
	VEML3328Result					MeasuredData;			/* structure to store the reading result */
	I2C_CALLBACK_STATUS				RXCallbackStatus;		/* used between the state machine steps to determine when to take the next step. or to know of the previous step was completed by the I2C hardware */
	I2C_CALLBACK_STATUS				TXCallbackStatus;		/* used between the state machine steps to determine when to take the next step. or to know of the previous step was completed by the I2C hardware */
	uint16_t						uiTimeout100HZ;			/* used to timeout the I2C operation in case something is going wrong, used in VEML3328RightTimeout100HZ() / VEML3328LeftTimeout100HZ() functions */
}VEML3328TypeDef;		/* used to measure the color on the back of the robot */
// ------------------------------ VEML3328 ------------------------------- END
//----------------------------------------------------------------------------------


// ------------------------------ 24C512 EE ------------------------------ START
typedef enum
{
	EEPROM_READ					= 0x00U,					/* used to indicate a read from the EEPROM */
	EEPROM_WRITE				= 0x01U						/* indicate a write operation */
}EEPROM_OPERATION;

typedef enum
{
	LIST_IS_EMPTY				= 0x00U,					/* the list do not contain any field and can be populated */
	LIST_IS_FULL				= 0x01U						/* the list is already populated and we are waiting for the I2C engine to access the EEPORM */
}EEPROM_LIST_STATUS;

typedef enum
{
	UNKNOWN_SECTION				= 0x00U,					/* if the callback is not needed or the section the request came was not specified */
	APPLICATION_SECTION			= 0x01U,					/* callback implemented in the Application.c -file */
	INTERACTION_SECTION			= 0x02U,					/* once the call the callback function for the Interaction section */
	WHEEL_SECTION				= 0x03U,					/* callback implemented in the wheel.c */
	AUDIO_SECTION				= 0x04U,					/* callback implemented in the Audio.c section */
	QRE_SECTION					= 0x05U,					/* callback implemented in the QRE.c file */


}EEPROM_DONE_CALLBACK_TO_CALL;

typedef struct
{
	uint16_t					ui16EEPROMAddress;			/* the internal address of the EEPROM where the data filed is intended to be written to or read from */
	uint32_t					ui32EEPROMData;				/* data to be read or write */
}EEPROMDataTypeDef;		// used to store or retrieve a single uint32 value from an EEPROM address.

// the maximum depth of the list is equivalent to 1 EEPROM page => 128/4=32 records/page =>  the maximum number of elements in the "*pEEDTList" is 32.
#define I2C_EEPROM_MAXIMUM_LIST_DEPTH			32		/* see the comment from above */
typedef struct
{
	// control fields
	//EEPROM_DONE_CALLBACK_TO_CALL	ucCallbackToCall;			/* once the list operation is done the I2C engine will call the callback function according to where the list is originating from */
	uint8_t							ucErrorCounter;				/* after the Main Loop EEPROM operation ends it will contain the number of errors encountered during the process. this can be checked inside the callback function to see if it was some error */
	EEPROM_LIST_STATUS				ucListIsPopulated;			/* if it is set to LIST_IS_FULL => an EEPROM operation is scheduled to be performed in the main loop of the I2CDevices.c => the list is already populated */
	EEPROM_OPERATION				ucReadWrite;				/* indicating what is the list for */
	uint8_t							ucSizeOfTheList;			/* the depth of the list containing "EEPROMDataTypeDef" data structures */

	// data fields
	volatile EEPROMDataTypeDef		*pEEDTList;					/* pointer of data fields to be read from or written into the EEPROM */
}EEPROMDataListTypeDef;		// used to write-to or read-from the EEPROM a data list structure using synchronous main loop access (not to be used @ initialization section, only in the main loop section)
// ------------------------------ 24C512 EE ------------------------------ END
//----------------------------------------------------------------------------------


// ------------------------------- TSL25911 ------------------------------ START

typedef enum
{
	TSL25911_UNKNOWN_STATE			= 0U,					/* BEFORE ANYTHING, TRY TO INITIALIZE THE SENSOR 	=> the initial state of the state-machine */
	TSL25911_ERROR					= 1U,					/* IF AN ERROR IS DETECTED			=> in case the sensor dose not answer to the I2C commands */
	//TSL25911_TRY_2_ENABLE			= 2U,					/* SET IN THE MANI LOOP				=> waiting for the write callback ==> WRITE OPERATION */
	TSL25911_ENABLED				= 3U,					/* SET IN THE WRITE CALLBACK		=> after the ENABLED register was set with: AEN and PON fields */
	//TSL25911_TRY_2_CONFIG			= 4U,					/* SET IN THE MAIN LOOP 			=> waiting for the write callback ==> WRITE OPERATION */
	TSL25911_CONFIGURED				= 5U,					/* SET IN THE WRITE CALLBACK		=> after the CONFIG register is used to set the GAIN, after this the gain is configured to MEDIUM_GAIN (01) */
	TSL25911_TRY_2_READ				= 6U,					/* SET IN THE MAIN LOOP				=> a channel read was triggered in the main loop to read both CH0 and CH1 data ==> READ OPERATION */
	TSL25911_CH_READ_DONE			= 7U,					/* SET INT THE READ CALLBACK		=> after the CH0 and CH1 channels were read out */
	TSL25911_WAIT_NEXT_READ			= 8U					/* SET IN THE MAIN LOOP				=> after the channel data is stored in the result variables */
}TSL25911_STATUS;

// ** for some, only by good known, reason the write interrupt dose not do the job. it dose not return any error but id dose not work in case of the TSL25911 ... => no mode : TSL25911_TRY_2_ENABLE, and TSL25911_TRY_2_CONFIG

typedef struct
{
	// settings and control variables
	SIDE 							ucSensorSide;				/* used to specify the on which side of the robot is the sensor */
	TSL25911_STATUS					ucOperationStatus;			/* used to control the reading steps of the sensor */

	// I2C
	uint8_t							ucI2CAddress;				/* the I2C address of the sensor */

	// result variables
	uint8_t							ucResultBuffer[4];			/* buffer to read out the result register: 0x14, 0x15 and 0x16, 0x17 */

	// operation result
	uint16_t						m_ui16OperationTimeoutMS;	/* used to insert some delay between the I2C operations */

}TSL25911TypeDef;		// used to handle the I2C access of the Sensor

typedef struct
{
	uint16_t						ui16CH0Result;				/* result from the CH0 channel */
	uint16_t						ui16CH1Result;				/* result from the CH1 channel */
	uint8_t							ucNewResultIsAvailable;		/* set each time a new result is available */
	TSL25911_STATUS					ucSensorStatus;				/* if error is detected this will be set to TSL25911_ERROR, if the read is ok this will be set to TSL25911_CH_READ_DONE */
}TSL25911Result;
// ------------------------------- TSL25911 ------------------------------ END
//----------------------------------------------------------------------------------


//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------


// I2C related variables
typedef struct
{
	uint8_t 			ucError;						/* used to hold the error detected on the I2C hardware. if(ucError & 0x01) => Error detected on I2C1, if(ucError & 0x02) => Error detected on I2C3 */
	uint8_t				ucI2CErrorCallbackTimer;		/* used to trigger the I2CErrorCallback() sending once every second as long as some error is detected */
}I2CErrorManagement;
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== CUSTOM_DATA_STRUCTS_END




// API
//==================================================================================================================== API_START

// VEML3328 Color Sensor
// the callback functions:
void I2CVEML3328DataAvailableCallback(SIDE SensorSide);			// can be used to notify the application that new VEML3328 data is available
//----------------------------------------------------------------------------------
void I2CVEML3328BothDataAvailableCallback(void);				// this callback will be called only after both VEML3328 sensors were read and the results are ok (in case of error, at least with one sensor this function will not be called)
//----------------------------------------------------------------------------------
// the regular API functions:
VEML3328Result I2CVEMLReadResult(SIDE SensorSide);				// used to read the sensor data. this function can be called inside the callback functions or outside of the callback function
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------

// 24C512 EEPROM section
// the callback functions:
// EEPROM operation done with callback
//----------------------------------------------------------------------------------
// the regular API functions:
// ------- FOR INITIALIZATION SECTION USE ONLY (NO MAIN LOOP) ---------
// EEPROM Read Initialization Section
HAL_StatusTypeDef 	I2CEEPROMReadIS(volatile EEPROMDataTypeDef *pData);		// function usable in the INITIALIZATION section of the code for read a uint32_t data from the EEPROM
// --------------------------------------------------------------------

// EEPROM Write Initialization Section
HAL_StatusTypeDef 	I2CEEPROMWriteIS(volatile EEPROMDataTypeDef *pData);		// function usable in the INITIALIZATION section only
// --------------------------------------------------------------------
// EEPROM Check Mani Loop
// ---- FOR MAIN LOOP SECTION USE ONLY (NO INITIALIZATION SECTION) ----
EEPROM_LIST_STATUS 	I2CEEPROMChekListStatusML(void);					// used to check the status of the list. if the list is empty one can read or write to the EEPROM using a function from below. if the list is not empty one need to wait to empty up before requesting anything to the EEPROM
// --------------------------------------------------------------------
// EEPROM Read Write Main Loop
HAL_StatusTypeDef	I2CEEPROMReadWriteListML(volatile EEPROMDataListTypeDef EEListData, void (*EEpromDoneCallback)(EEPROMDataListTypeDef) );		// -usable only in the main loop , not in initialization section- used to write data into the EEPROM. if everything is ok all the data from the list will be written into the EEPROM
// --------------------------------------------------------------------
// EEPROM callback -> operation finished
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------

// TSL25911
// callback
void I2CTSL25911DataAvailableCallback(TSL25911Result RightSesnor, TSL25911Result LeftSensor);		// must be implemented in the application code if the TSL25911 data is needed
//----------------------------------------------------------------------------------
//regular API
void I2CTSL25911SensorReset(SIDE ucSensorSide);		// used to reset a sensor if needed
//----------------------------------------------------------------------------------
TSL25911Result I2CTSL25911ReadSensorData(SIDE ucSensorSide);	// read the status and the available data on a sensor
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------


// I2C related functions
void I2CErrorCallback(uint8_t ucError);	// used to indicate that at least one of the two I2C engine has some error
//----------------------------------------------------------------------------------
uint8_t I2CGetErrorStatus(void);		// returns the error status of the 2 I2C engines. if(returned value & 0x01) => Error with I2C1, if (returned value & 0x02) => Error with I2C3
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== API_END



// SYSTEM
//==================================================================================================================== SYSTEM_START
void I2CDevicesInit(void);				// used to initialize the QRE Module. Called from main.c file @ init section
//----------------------------------------------------------------------------------
void I2CDevicesMainLoop(void);			// loop function of the QRE Module. Called from the main.c file @ main loop section
//----------------------------------------------------------------------------------
void I2CDevicesTimer1KHZISR(void);		// 1KHZ timer function called from System ISR from stm32f4xx_it.c file @ SysTick Function (!!! keep as short as possible !!!)
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
#ifdef I2CDEVICES_DEBUG_MODE
void I2CDevicesDebugRX(uint8_t *ucRXBuffer, uint8_t ucRXBufferLength);		// used to receive RX data from the debug interface
void I2CDevicesDebugRXChar(uint8_t ucRXBuffer);		// used to receive RX single char from the debug interface
#endif
//----------------------------------------------------------------------------------
//==================================================================================================================== SYSTEM_END


// STM32 HAL CALLBACK
//==================================================================================================================== HAL_START

//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== HAL_END


// STATIC (INTERN)
//==================================================================================================================== STATIC_START
// defined in .c file
//==================================================================================================================== STATIC_END


//main HAL functions used in this file
// HAL_StatusTypeDef HAL_I2C_Mem_Write_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
// HAL_StatusTypeDef HAL_I2C_Mem_Read_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
// void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c);
// void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);


// Resources
/*
I2C1
- PB6 = SCL1
- PB7 = SDA1
-- VEML3328 (RIGHT) (max 400KHZ) 	=> Add = 0x20(W), 0x21(R)
-- TSL25911 (RIGHT)	(max 400KHZ)	=> Add = 0x52(W), 0x53(R)	=> https://github.com/adafruit/Adafruit_TSL2591_Library/blob/master/Adafruit_TSL2591.cpp
-- 24C512           (max 400KHZ)	=> Add = 0xA0(W), 0xA1(R)
-- LSM9DS1 (IMU)    (max 400KHZ)	=> Add:
----- Accelerometer + Gyroscope		=> Add = 0xD4(W), 0xD5(R), 0xD6(W), 0xD7(R)
----- Magnetic sensor				=> Add = 0x38(W), 0x39(R), 0x3C(W), 0x3D(R)
-- INA219			(max 400KHZ)	=> Add = 0x80(W), 0x81(R)

I2C3
- PA8 = SCL2
- PC9 = SDA2
 *
 *
 *
 *
 */

#endif /* INC_I2CDEVICES_H_ */
