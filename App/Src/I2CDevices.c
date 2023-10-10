/*
 * I2CDevices.c
 *
 *  Created on: Feb 23, 2022
 *      Author: Arthur
 */
#include "main.h"

#include "stdint.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"

#include "I2CDevices.h"
#include "I2CEEPROMAddresses.h"

#include "Audio.h"	/* for playing the error sound in case it is I2C Error */




// STATIC (INTERN) prototypes
//==================================================================================================================== STATIC_START
// variables - external
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c3;
//----------------------------------------------------------------------------------


// variables - internal
#ifdef I2CDEVICES_DEBUG_MODE
uint8_t m_ucDebugBuffer[200];
uint8_t m_ucDebugChar1 = 1;		// used as general purpose var in debug
uint16_t m_uiDebugTemp1, m_uiDebugTemp2;
#endif
#define I2CDEVICES_SKIP_REAL_TIME		821
uint32_t m_ui32I2CDevicesSkipRealTime;	// used in the main loop to skip real time
//----------------------------------------------------------------------------------
uint8_t m_ucI2CRXDataBuffer[20] = {0};
uint8_t m_ucI2CTXDataBuffer[20] = {0};
HAL_StatusTypeDef HAL_ErrorCode;
//----------------------------------------------------------------------------------
// Timer sharing variable
#define I2C_TIMER_SHARING_1KHZ		9
uint8_t m_ucTimerSharing100HZCounter;		// used for the 1KHZ timer sharing in 100HZ shares
//----------------------------------------------------------------------------------
// VMEL3328 sensor
VEML3328TypeDef RightVEML3328 = {0};		// used to handle the RIGHT side sensor
VEML3328Result  RightVEML3328Result = {0};	// used to store the result of the reading from the RIGHT color sensor
VEML3328TypeDef LeftVEML3328 = {0};			// used to handle the LEFT side sensor
VEML3328Result  LeftVEML3328Result = {0};	// used to store the result of the reading from the LEFT color sensor
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
// EEPROM 24C512
#define I2CEEPROM_WRITE_TIME_DELAY_MS		12		/* used to delay after each eeprom write operation */
volatile uint8_t					m_ucEEpromAccessTimerCounterMS;		// used to be decremented @ 1 ms rate after each EEPROM Write operation
volatile EEPROMDataTypeDef			EEpromData[32];		/* the maximum number of elements a list can hold */
volatile EEPROMDataListTypeDef		EEpromDataList;		/* the data list that is used to operate the EEPROM in the main loop mode */
void 								(*EEDoneCallbackFunction)(EEPROMDataListTypeDef);	// callback function to return to caller
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
// TSL25911
static volatile TSL25911TypeDef		RightTSL25911 		= {0};		/* used to handle the I2C reading from the Right TSL25911 ALS sensor */
static volatile TSL25911Result		RightTSL25911Result = {0};		/* used to store the measurement result from the Right sensor */
static volatile TSL25911TypeDef		LeftTSL25911 		= {0};		/* used to handle the I2C reading from the Left TSL25911 ALS sensor */
static volatile TSL25911Result		LeftTSL25911Result 	= {0};		/* used to store the measurement result from the Left sensor */
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
// I2C related variables (mainly for error detection and management)
I2CErrorManagement I2CErrorStatus = {0};
//----------------------------------------------------------------------------------

// prototypes
//----------------------------------------------------------------------------------
// VEML3328 sensor
//static void HandleRightVEML3328Loop(void);
//----------------------------------------------------------------------------------
static void HandleVEML3328Loop(VEML3328TypeDef *VEML3328);
//----------------------------------------------------------------------------------
static void VEML3328ErrorDetected(VEML3328TypeDef *VEML3328, uint16_t uiError);	// handle VEML3328 data in case some error happened during the I2C reading operation.
//----------------------------------------------------------------------------------
static void StoreVEML3328Result(VEML3328TypeDef VEML3328);		// used to store the measurement result in the sensor variable to be available for the API read
//----------------------------------------------------------------------------------
static void VEML3328RightTimeout100HZ(void);			// called from the system timer @ 100HZ rate. it is used to timeout the reading of the RIGHT VEML3328 color sensors
//----------------------------------------------------------------------------------
static void VEML3328LeftTimeout100HZ(void);			// called from the system timer @ 100HZ rate. it is used to timeout the reading of the LEFT VEML3328 color sensors
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
// EEPROM 24C512 section (@I2C1 channel)
static void EEPROMInitialization(void);				// called from the initialization section
//----------------------------------------------------------------------------------
static void EEPROMMainLoop(void);					// called from the system's main loop to handle list related READ / WRITE operations
//----------------------------------------------------------------------------------
static void EEPROM32To8(uint32_t ui32DataIn, uint8_t *ucDataOut);			// used to convert 32 bit uint data into 4 bytes of 8 bit uint data
//----------------------------------------------------------------------------------
static void EEPROM8To32(uint8_t *ucDataIn, volatile uint32_t *ui32DataOut);			// used to convert buffer of 4 bytes of 8 bit uint data into single 32 bit uint data
//----------------------------------------------------------------------------------
static void EEPROMWriteDelay(void);			// used to wait for the write time after an EEPROM write operation
//----------------------------------------------------------------------------------
static void EEPROMTimer1KHZISR();			// EEPROM Write Access Time delay timer. called in the 1KHZ Timer ISR
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
// TSL25911
//----------------------------------------------------------------------------------
static void TSL25911Init(void);							// used @ initialization
//----------------------------------------------------------------------------------
static void TSL25911MainLoop(SIDE ucSensorSide);		// used in the main loop
//----------------------------------------------------------------------------------
static void TSL25911Timer1KHZISR(void);				// called from the 1KHZ Timer ISR
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------

// I2C general function and error management
static void I2CErrorChecker(void);		// used in the loop to check if there is some I2C error and trigger the callback
//----------------------------------------------------------------------------------
static void I2CErrorTimer100HZ(void);		// called @ 100HZ rate to handle error management
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== STATIC_END



// API
//==================================================================================================================== API_START

// VEML3328 Color Sensor
// the callback functions:
__attribute__((weak)) void I2CVEML3328DataAvailableCallback(SIDE SensorSide)	// can be used to notify the application that new VEML3328 data is available
{
	UNUSED(SensorSide);	// just to avoid warning. comment or delete for real implementation

	/*
	VEML3328Result myVEML3328Result;
	if(SensorSide == RIGHT)	myVEML3328Result = I2CVEMLReadResult(SensorSide); // right data is available
	if(SensorSide == LEFT)	myVEML3328Result = I2CVEMLReadResult(SensorSide); // left  data is available
	*/


	/*
	// EX:
	VEML3328Result myVEML3328Result;

	if(SensorSide == RIGHT)
	{
		myVEML3328Result = I2CVEMLReadResult(RIGHT); // right data is available
		sprintf((char *)m_ucDebugBuffer,"RE=%u, C=0x%X, CL=%u, R=%u, G=%u, B=%u, IR=%u\r\n",myVEML3328Result.uiError, myVEML3328Result.uiConfiguration, myVEML3328Result.uiClear, myVEML3328Result.uiRed,
				myVEML3328Result.uiGreen, myVEML3328Result.uiGreen, myVEML3328Result.uiIR);
		DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
	}

	if(SensorSide == LEFT)
	{
		myVEML3328Result = I2CVEMLReadResult(LEFT); // right data is available
		sprintf((char *)m_ucDebugBuffer,"LE=%u, C=0x%X, CL=%u, R=%u, G=%u, B=%u, IR=%u\r\n",myVEML3328Result.uiError, myVEML3328Result.uiConfiguration, myVEML3328Result.uiClear, myVEML3328Result.uiRed,
				myVEML3328Result.uiGreen, myVEML3328Result.uiGreen, myVEML3328Result.uiIR);
		DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
	}
	*/

	return;
}
//----------------------------------------------------------------------------------
__attribute__((weak)) void I2CVEML3328BothDataAvailableCallback(void)	// this callback will be called only after both VEML3328 sensors were read and the results are ok (in case of error, at least with one sensor this function will not be called)
{
	// to get the data from the 2 sensors just call the I2CVEMLReadResult(RIGHT) and I2CVEMLReadResult(LEFT) functions
	/*
	 * VEML3328Result myVEML3328ResultRight, myVEML3328ResultLeft;
	 * myVEML3328ResultRight = I2CVEMLReadResult(RIGHT);
	 * myVEML3328ResultLeft  = I2CVEMLReadResult(LEFT);
	 */

#define I2CVEML3328_DEBUG
#if defined(I2CDEVICES_DEBUG_MODE) && defined(I2CVEML3328_DEBUG) && defined(I2CVEML3328_DEBUG_MODE)
  // EX:
	if(m_ucDebugChar1)
	{
		VEML3328Result myVEML3328ResultRight, myVEML3328ResultLeft;

		myVEML3328ResultRight = I2CVEMLReadResult(RIGHT);
		myVEML3328ResultLeft = I2CVEMLReadResult(LEFT);
		//sprintf((char *)m_ucDebugBuffer,"E=%u, R=%u, G=%u, B=%u, E=%u, R=%u, G=%u, B=%u\r\n",myVEML3328ResultRight.uiError, myVEML3328ResultRight.uiRed,myVEML3328ResultRight.uiGreen,myVEML3328ResultRight.uiBlue,
		//																					myVEML3328ResultLeft.uiError, myVEML3328ResultLeft.uiRed,myVEML3328ResultLeft.uiGreen,myVEML3328ResultLeft.uiBlue);

		sprintf((char *)m_ucDebugBuffer,"n%u,%u,%u,%u,%u,%u\r\n", myVEML3328ResultRight.uiRed,myVEML3328ResultRight.uiGreen,myVEML3328ResultRight.uiBlue,
																 myVEML3328ResultLeft.uiRed,myVEML3328ResultLeft.uiGreen,myVEML3328ResultLeft.uiBlue);
		DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
	}
#endif

	return;
}
//----------------------------------------------------------------------------------
// the regular API functions:
VEML3328Result I2CVEMLReadResult(SIDE SensorSide)	// used to read the VEML3328 color sensor data
{
	VEML3328Result myVEML3328Result;

	if(SensorSide == RIGHT)
	{	// right sensor
		myVEML3328Result = RightVEML3328Result;
		RightVEML3328Result.NewMeasuredData = NO_NEW_DATA_IS_AVAILABLE;
	}
	else
	{	// left sensor
		myVEML3328Result = LeftVEML3328Result;
		LeftVEML3328Result.NewMeasuredData = NO_NEW_DATA_IS_AVAILABLE;
	}

	return myVEML3328Result;
}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------

// 24C512 EEPROM section
// ------- FOR INITIALIZATION SECTION USE ONLY (NO MAIN LOOP) ---------
HAL_StatusTypeDef 	I2CEEPROMReadIS(volatile EEPROMDataTypeDef *pData)		// function usable in the INITIALIZATION section of the code for read a uint32_t data from the EEPROM
{
	uint16_t	ui16EEAddress;
	//uint32_t	ui32EEData;
	uint8_t		ucEEData[4];
	//-------------------------------

	ui16EEAddress 	= (uint16_t)pData->ui16EEPROMAddress;

	// HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
	if(HAL_I2C_Mem_Read(&hi2c1, EEPROM_I2C_ADDRESS_ON_I2C1, ui16EEAddress, 2,ucEEData,4, HAL_MAX_DELAY) != HAL_OK)
	{
		//Error_Handler2(ERROR_CODE_I2CDEVICES_DEBUG_001);
		return HAL_ERROR;
	}
	EEPROMWriteDelay();	// the 10ms write delay (not critical for read operations)

	// DIGIT3 = MSD
	//ui32EEData = (uint32_t)ucEEData[3];
	//ui32EEData <<= (3*8);	// shift with 24 bits
	//pData->ui32EEPROMData = ui32EEData;		// 1/4
	// DIGIT2
	//ui32EEData = (uint32_t)ucEEData[2];
	//ui32EEData <<= (2*8);	// shift with 16 bits
	//pData->ui32EEPROMData |= ui32EEData;	// 2/4
	// DIGIT1
	//ui32EEData = (uint32_t)ucEEData[1];
	//ui32EEData <<= (1*8);	// shift with 8 bits
	//pData->ui32EEPROMData |= ui32EEData;	// 3/4
	// DIGIT0 = LSD
	//ui32EEData = (uint32_t)ucEEData[0];
	//ui32EEData <<= (0*8);	// shift with 0 bits
	//pData->ui32EEPROMData |= ui32EEData;	// 4/4
	EEPROM8To32(ucEEData, &(pData->ui32EEPROMData));


	return HAL_OK;
}
// --------------------------------------------------------------------
HAL_StatusTypeDef 	I2CEEPROMWriteIS(volatile EEPROMDataTypeDef *pData)		// function usable in the INITIALIZATION section only
{
	uint16_t	ui16EEAddress;
	uint32_t	ui32EEData;
	uint8_t		ucEEData[4];
	//-------------------------------

	ui16EEAddress 	= (uint16_t)pData->ui16EEPROMAddress;
	ui32EEData		= pData->ui32EEPROMData;

	//ucEEData[0]		= (uint8_t)((ui32EEData >> (0*8)) & 0x000000FF);	// shifted with 0  bits	LSD	= Less Significant Digit
	//ucEEData[1]		= (uint8_t)((ui32EEData >> (1*8)) & 0x000000FF);	// shifted with 8  bits
	//ucEEData[2]		= (uint8_t)((ui32EEData >> (2*8)) & 0x000000FF);	// shifted with 16 bits
	//ucEEData[3]		= (uint8_t)((ui32EEData >> (3*8)) & 0x000000FF);	// shifted with 24 bits	MSD = MostSignificant Digit
	EEPROM32To8(ui32EEData, ucEEData);

	// HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
	if(HAL_I2C_Mem_Write(&hi2c1, EEPROM_I2C_ADDRESS_ON_I2C1, ui16EEAddress, 2,ucEEData,4, HAL_MAX_DELAY) != HAL_OK)
	{
		//Error_Handler2(ERROR_CODE_I2CDEVICES_DEBUG_001);
		return HAL_ERROR;
	}
	EEPROMWriteDelay();	// the 10ms write delay

	return HAL_OK;
}
// --------------------------------------------------------------------
// ---- FOR MAIN LOOP SECTION USE ONLY (NO INITIALIZATION SECTION) ----
EEPROM_LIST_STATUS 	I2CEEPROMChekListStatusML(void)								// used to check the status of the list. if the list is empty one can read or write to the EEPROM using a function from below. if the list is not empty one need to wait to empty up before requesting anything to the EEPROM
{
	return EEpromDataList.ucListIsPopulated;
	//return LIST_IS_EMPTY;
}
// --------------------------------------------------------------------
//HAL_StatusTypeDef	I2CEEPROMReadWriteListML(volatile EEPROMDataListTypeDef EEListData)		// -usable only in the main loop , not in initialization section- used to write/read data into/from the EEPROM. if everything is ok all the data from the list will be written/read into/from the EEPROM => the callback will be called @ the end
HAL_StatusTypeDef	I2CEEPROMReadWriteListML(volatile EEPROMDataListTypeDef EEListData, void (*EEpromDoneCallback)(EEPROMDataListTypeDef) )
{
	uint8_t i;	// only for indexing

	// check if the list is not already populated
	if( (EEpromDataList.ucListIsPopulated == LIST_IS_FULL) )	return HAL_ERROR;

	// the list was not populated but it is now
	EEpromDataList.ucListIsPopulated 	= LIST_IS_FULL;
	EEpromDataList.ucReadWrite 			= EEListData.ucReadWrite;
	EEpromDataList.ucSizeOfTheList 		= EEListData.ucSizeOfTheList;

	// set the pointer of the callback function
	EEDoneCallbackFunction = EEpromDoneCallback;

	for(i=0; i<EEpromDataList.ucSizeOfTheList; i++)
	{	// copy the input list content into the local list
		EEpromDataList.pEEDTList[i].ui16EEPROMAddress 		= EEListData.pEEDTList[i].ui16EEPROMAddress;	// all the EEPROM addresses (important for both READ and WRITE)
		EEpromDataList.pEEDTList[i].ui32EEPROMData			= EEListData.pEEDTList[i].ui32EEPROMData;		// all the data what will be written into the EEPROM (only important for EEPROM_WRITE case)
	}

	return HAL_OK;
}
// --------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------

// TSL25911
// callback
__attribute__ ((weak)) void I2CTSL25911DataAvailableCallback(TSL25911Result RightSesnor, TSL25911Result LeftSensor)		// must be implemented in the application code if the TSL25911 data is needed
{	// it is called only if both sensor are working properly
	// in case at least one sensor dose not work => use the I2CTSL25911ReadSensorData() function and check the side containing the error
	// the sensor reading is available in the 2 input parameters
	//------------------------------

	#define I2CTSL25911_CALLBACK_PRINT
	#if defined(I2CDEVICES_DEBUG_MODE) && defined(I2CTSL25911_CALLBACK_PRINT) && defined(I2CTSL25911_DEBUG_MODE)
	sprintf((char *)m_ucDebugBuffer,"TSL25911: Right: CH0=%u\t CH1=%u\t\t Left: CH0=%u\t CH1=%u\r\n",
																		RightSesnor.ui16CH0Result,
																		RightSesnor.ui16CH1Result,
																		LeftSensor.ui16CH0Result,
																		LeftSensor.ui16CH1Result);
	DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
	#endif


	return;
}
//----------------------------------------------------------------------------------
//regular API
void I2CTSL25911SensorReset(SIDE ucSensorSide)		// used to reset a sensor if needed
{
	if(ucSensorSide == RIGHT)	RightTSL25911.ucOperationStatus = TSL25911_UNKNOWN_STATE;	// @I2C1
	if(ucSensorSide == LEFT)	LeftTSL25911.ucOperationStatus 	= TSL25911_UNKNOWN_STATE;	// @I2C3
	return;
}
//----------------------------------------------------------------------------------
TSL25911Result I2CTSL25911ReadSensorData(SIDE ucSensorSide)	// read the status and the available data on a sensor
{
	if(ucSensorSide == RIGHT) return RightTSL25911Result;
	return LeftTSL25911Result;
}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------

// I2C related functions
__attribute__ ((weak)) void I2CErrorCallback(uint8_t ucError)	// used to indicate that at least one of the two I2C engine has some error
{	// called each time a change in the I2C hardware state is detected
	// in case of some error during the error this function will be called once every second

	/*
	if( (ucError & 0x01) )	//=> I2C1 has error
	{

	}
	if( (ucError & 0x02) )	//=> I2C3 has error
	{

	}
	*/

#ifdef I2CDEVICES_DEBUG_MODE
	sprintf((char *)m_ucDebugBuffer,"I2C Error = 0x%02X\r\n",ucError);
	DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
#endif

	return;
}
//----------------------------------------------------------------------------------
uint8_t I2CGetErrorStatus(void)	// returns the error status of the 2 I2C engines. if(returned value & 0x01) => Error with I2C1, if (returned value & 0x02) => Error with I2C3
{
	return I2CErrorStatus.ucError;
}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== API_END


// SYSTEM
//==================================================================================================================== SYSTEM_START
void I2CDevicesInit(void)	// used to initialize the I2CDevices Module. Called from main.c file @ init section
{
	// VEML3328
	RightVEML3328.SensorSide = RIGHT;	// I2C1
	RightVEML3328.uiI2CAddress = 0x20;	// I2C1 VEML3328 Address (0x20 = 0x10<<1)
	RightVEML3328.I2CReadingState = NoI2CReadingInProgress;
	RightVEML3328Result.NewMeasuredData = NO_NEW_DATA_IS_AVAILABLE;	// no new data from the right color sensor
	LeftVEML3328.SensorSide = LEFT;		// I2C3
	LeftVEML3328.uiI2CAddress = 0x20;	// I2C3 VEML3328 Address (0x20 = 0x10<<1)
	LeftVEML3328.I2CReadingState = NoI2CReadingInProgress;
	LeftVEML3328Result.NewMeasuredData = NO_NEW_DATA_IS_AVAILABLE;	// no new data from the left color sensor

	// 24C512
	EEPROMInitialization();


	// TSL25911
	TSL25911Init();

	// some other I2c

	m_ui32I2CDevicesSkipRealTime = 100000;	// 100K, Max Value in 32 bit uint = 4,294,967,295
	return;
}
//----------------------------------------------------------------------------------
void I2CDevicesMainLoop(void)	// loop function of the I2CDevices Module. Called from the main.c file @ main loop section
{
	// skip real time
	if(m_ui32I2CDevicesSkipRealTime)
	{
		m_ui32I2CDevicesSkipRealTime --;
		return;
	}
	m_ui32I2CDevicesSkipRealTime = I2CDEVICES_SKIP_REAL_TIME;
	//--------------------------------------------------------
	// check the state of the I2C hardware interfaces and in case of error trigger the error callback function
	I2CErrorChecker();
	//--------------------------------------------------------

	// 24C512 @I2C1 driver
	//--------------------------------------------------------	24C512 START
	if( (EEpromDataList.ucListIsPopulated == LIST_IS_FULL) )
	{	// some data is available to be read-from or write-into the EEPROM
		// !!! the EEPROM function will access the EEPROM only if no other scheduled I2C1 operation is in progress: no waiting for I2C interrupt complete situation !!!
		// for further detail check the implementation of the EEPROM main loop function from below.
		EEPROMMainLoop();
	}
	//--------------------------------------------------------	24C512 END

	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);	//  debug on PC13 HIGH
	// VEML3328
	// ------------------------------------------------------- Right VEML START
	if( (RightVEML3328.I2CReadingState != I2CReadingDone) )
	{	//VEML3328 @I2C1 driver
		//HandleRightVEML3328Loop();
		HandleVEML3328Loop(&RightVEML3328);
		//if(m_ucDebugChar1) HandleVEML3328Loop(&RightVEML3328);
		return;
	}
	// ------------------------------------------------------- Right VEML END
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);  //	debug on PC13 LOW

	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);	//  debug on PC13 HIGH
	// ------------------------------------------------------- Left VEML START
	if( (LeftVEML3328.I2CReadingState != I2CReadingDone) )
	{	//VEML3328 @I2C3 driver
		HandleVEML3328Loop(&LeftVEML3328);
		return;
	}
	// ------------------------------------------------------- Left VEML END
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);  //	debug on PC13 LOW




	// ------- CALLBACK for both VEML3328 sensors done ------- START
	if( (LeftVEML3328.MeasuredData.NewMeasuredData == NEW_DATA_IS_AVAILABLE) && (RightVEML3328.MeasuredData.NewMeasuredData == NEW_DATA_IS_AVAILABLE))
	{
		LeftVEML3328.MeasuredData.NewMeasuredData  = NO_NEW_DATA_IS_AVAILABLE;
		RightVEML3328.MeasuredData.NewMeasuredData = NO_NEW_DATA_IS_AVAILABLE;
		// call the callback function
		// this callback will be called only after both VEML3328 sensors were read and the results are ok (in case of error at least with one sensor this function will not be called)
		I2CVEML3328BothDataAvailableCallback();
		return;
	}
	// ------- CALLBACK for both VEML3328 sensors done ------- END



	// TSL25911
	//--------------------------------------------------------	TSL25911 START
	// ---- TSL25911 Right @ I2C1 driver ----
	if( (RightTSL25911.ucOperationStatus != TSL25911_ERROR) )
	{
		//if( (RightTSL25911.ucOperationStatus == TSL25911_UNKNOWN_STATE) 	||
		//		(RightTSL25911.ucOperationStatus == TSL25911_ENABLED) 		||
		//		(RightTSL25911.ucOperationStatus == TSL25911_CONFIGURED) 	||
		//		(RightTSL25911.ucOperationStatus == TSL25911_CH_READ_DONE) 		)
		if( (RightTSL25911.ucOperationStatus != TSL25911_WAIT_NEXT_READ) )
		{
			TSL25911MainLoop(RIGHT);
			return;
		}
	}
	// --------------------------------------

	// ---- TSL25911 Left  @ I2C3 driver ----
	if( (LeftTSL25911.ucOperationStatus != TSL25911_ERROR) )
	{
		//if( (LeftTSL25911.ucOperationStatus == TSL25911_UNKNOWN_STATE) 		||
		//		(LeftTSL25911.ucOperationStatus == TSL25911_ENABLED) 		||
		//		(LeftTSL25911.ucOperationStatus == TSL25911_CONFIGURED) 	||
		//		(LeftTSL25911.ucOperationStatus == TSL25911_CH_READ_DONE) 		)
		if( (LeftTSL25911.ucOperationStatus != TSL25911_WAIT_NEXT_READ) )
		{
			TSL25911MainLoop(LEFT);
			return;
		}
	}
	// --------------------------------------

	// ---- check the callback condition ----
	if( (RightTSL25911Result.ucNewResultIsAvailable) && (LeftTSL25911Result.ucNewResultIsAvailable) )
	{
		I2CTSL25911DataAvailableCallback(RightTSL25911Result, LeftTSL25911Result);
		RightTSL25911Result.ucNewResultIsAvailable = 0;
		LeftTSL25911Result.ucNewResultIsAvailable = 0;
		return;
	}
	// --------------------------------------
	//--------------------------------------------------------	TSL25911 END



	// LSM9DS1	@ I2C1 driver
	//--------------------------------------------------------	LSM9DS1 STAR
	//--------------------------------------------------------	LSM9DS1 END



	// INA219 @ I2C1 driver
	//--------------------------------------------------------	INA219 STAR
	//--------------------------------------------------------	INA219 END








	// VEML3328
	// restart sensor reading
	RightVEML3328.I2CReadingState = NoI2CReadingInProgress;		// restart RIGHT VEML
	LeftVEML3328.I2CReadingState  = NoI2CReadingInProgress;		// restart LEFT VEML

	// TSL25911
	// restart reading
	if( (RightTSL25911.ucOperationStatus 	!= TSL25911_ERROR) && (RightTSL25911.ucOperationStatus 	== TSL25911_WAIT_NEXT_READ) ) 	RightTSL25911.ucOperationStatus = TSL25911_CONFIGURED;		// the sensor is supposed to be configured => ready for another read try
	if( (LeftTSL25911.ucOperationStatus 	!= TSL25911_ERROR) && (LeftTSL25911.ucOperationStatus 	== TSL25911_WAIT_NEXT_READ) ) 	LeftTSL25911.ucOperationStatus	= TSL25911_CONFIGURED;		// the sensor is supposed to be configured => ready for another read try




	return;
}
//----------------------------------------------------------------------------------
void I2CDevicesTimer1KHZISR(void)	// 1KHZ timer function called from System ISR from stm32f4xx_it.c file @ SysTick Function (!!! keep as short as possible !!!)
{	// used for timeout checking of the I2C operations

	// EEPROM 24C16
	EEPROMTimer1KHZISR();

	// TSL25911
	TSL25911Timer1KHZISR();


	// VEML3328
	if(m_ucTimerSharing100HZCounter < I2C_TIMER_SHARING_1KHZ)		m_ucTimerSharing100HZCounter ++;	// 0->1,1->2,2->3,3->4,4->5,5->6,6->7,7->8,8->9
	else 															m_ucTimerSharing100HZCounter = 0;	// 9->0  => total number of rounds = (I2C_TIMER_SHARING_1KHZ + 1) =in range of= [0 ... I2C_TIMER_SHARING_1KHZ]
																											// freq. rate = 1KHZ/(I2C_TIMER_SHARING_1KHZ + 1)

	// case 0 => RIGHT Side VEML3328 color sensor timeout checking
	if( (m_ucTimerSharing100HZCounter == 0) )
	{
		VEML3328RightTimeout100HZ();
		return;
	}

	// case 1 => LEFT Side VEML3328 color sensor timeout checking
	if( (m_ucTimerSharing100HZCounter == 1) )
	{
		VEML3328LeftTimeout100HZ();
		return;
	}


	// case 2 => TSL25911 Right
	if( (m_ucTimerSharing100HZCounter == 2) )
	{
		return;
	}
	// case 3 => TSL25911 Left
	if( (m_ucTimerSharing100HZCounter == 3) )
	{
		return;
	}

	// case 4 => LSM9DS1 Magnetometer
	if( (m_ucTimerSharing100HZCounter == 4) )
	{
		return;
	}
	// case 5 => LSM9DS1 Accelerometer
	if( (m_ucTimerSharing100HZCounter == 5) )
	{
		return;
	}

	// case 6 => INA219
	if( (m_ucTimerSharing100HZCounter == 6) )
	{
		return;
	}

	// case 7 =>
	// case 8 =>
	// case 9 => I2C Error management
	if( (m_ucTimerSharing100HZCounter == 9) )
	{
		I2CErrorTimer100HZ();
		return;
	}







	return;
}
//----------------------------------------------------------------------------------
#ifdef I2CDEVICES_DEBUG_MODE
void I2CDevicesDebugRX(uint8_t *ucRXBuffer, uint8_t ucRXBufferLength)	// used to receive RX data from the debug interface
{
	return;
}
void I2CDevicesDebugRXChar(uint8_t ucRXBuffer)	// used to receive RX data from the debug interface
{

	// I2CError ---------------------------------------------- START
	if(ucRXBuffer == 'v')
	{
		__HAL_RCC_I2C1_FORCE_RESET();
		return;
	}
	if(ucRXBuffer == 'V')
	{
		__HAL_RCC_I2C1_RELEASE_RESET();
		return;
	}

	if(ucRXBuffer == 'b')
	{
	  hi2c1.Instance = I2C1;
	  hi2c1.Init.ClockSpeed = 100000;
	  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	  hi2c1.Init.OwnAddress1 = 0;
	  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	  hi2c1.Init.OwnAddress2 = 0;
	  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	  {
		//Error_Handler();
	  }
		return;
	}

	if(ucRXBuffer == 'n')
	{
		m_ucDebugChar1 = 1;	// loop on
		return;
	}
	if(ucRXBuffer == 'm')
	{
		m_ucDebugChar1 = 0;	// loop on
		return;
	}
	if(ucRXBuffer == '.')
	{
		HAL_I2C_MspDeInit(&hi2c1);
		return;
	}
	if(ucRXBuffer == ':')
	{
		HAL_I2C_MspInit(&hi2c1);
		return;
	}

	if(ucRXBuffer == ',')
	{
		sprintf((char *)m_ucDebugBuffer,"some message\r\n");
		DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
		return;
	}
	// I2CError ---------------------------------------------- END



#ifdef I2CTSL25911_DEBUG_MODE
	// TSL25911 ---------------------------------------------- START
	if(ucRXBuffer == 'p')
	{
		// read ID from TSL25911
		// TSL25911 I2C Address = 0x52 (after shifting, in PDF is 0x29)
		// ID Register Address = 0x12
		// ID Expected Value = 0x50 (Device ID)
		if(HAL_I2C_Mem_Read(&hi2c1, 0x52, 0xB2, 1,m_ucI2CRXDataBuffer,1, HAL_MAX_DELAY) != HAL_OK)
		{
			//Error_Handler2(ERROR_CODE_I2CDEVICES_DEBUG_001);
			sprintf((char *)m_ucDebugBuffer,"TSL25911 ID Read Error\r\n");
		}
		else
		{
			sprintf((char *)m_ucDebugBuffer,"ID=0x%02X\r\n", m_ucI2CRXDataBuffer[0]);
		}

		DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
		return;
	}
	if(ucRXBuffer == 'o')
	{	// TSL25911
		// TSL25911 I2C Address = 0x52 (after shifting, in PDF is 0x29)
		// Enable Register Address = 0x00 + (Normal Command = 0xA0) => Register Address = 0xA0
		// Enable Register Data    = 0000 0011 = 0x03 (AEN + PON)
		m_ucI2CTXDataBuffer[0] = 0x03;		// (AEN + PON)

		// HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
		if(HAL_I2C_Mem_Write(&hi2c1, 0x52, 0xA0, 1,m_ucI2CTXDataBuffer,1, HAL_MAX_DELAY) != HAL_OK)
		{
			//Error_Handler2(ERROR_CODE_I2CDEVICES_DEBUG_001);
			sprintf((char *)m_ucDebugBuffer,"(02)TSL25911 ID Read Error\r\n");
			DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
		}

		return;
	}

	if(ucRXBuffer == 'i')
	{	// TSL25911
		// TSL25911 I2C Address = 0x52 (after shifting, in PDF is 0x29)
		// Control Register Address = 0x01 + (Normal Command = 0xA0) => Register Address = 0xA1
		// Control Register Data    = 0001 0000 = 0x10 (AGAIN = MEDIUM)
		m_ucI2CTXDataBuffer[0] = 0x10;		// (AGAIN = MEDIUM)

		// HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
		if(HAL_I2C_Mem_Write(&hi2c1, 0x52, 0xA1, 1,m_ucI2CTXDataBuffer,1, HAL_MAX_DELAY) != HAL_OK)
		{
			//Error_Handler2(ERROR_CODE_I2CDEVICES_DEBUG_001);
			sprintf((char *)m_ucDebugBuffer,"(03)TSL25911 ID Read Error\r\n");
			DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
		}

		return;
	}

	if(ucRXBuffer == 'u')
	{	// TSL25911
		// TSL25911 I2C Address = 0x52 (after shifting, in PDF is 0x29)
		// ALS Data Register Address = 0x14 + (Normal Command = 0xA0) => Register Address = 0xB4
		// ALS Data Register Data    = XX,YY
		if(HAL_I2C_Mem_Read(&hi2c1, 0x52, 0xB4, 1,m_ucI2CRXDataBuffer,4, HAL_MAX_DELAY) != HAL_OK)
		{
			//Error_Handler2(ERROR_CODE_I2CDEVICES_DEBUG_001);
			sprintf((char *)m_ucDebugBuffer,"(04)TSL25911 ID Read Error\r\n");
		}
		else
		{
			m_uiDebugTemp1   = (uint16_t)m_ucI2CRXDataBuffer[1];
			m_uiDebugTemp1 <<= 8;
			m_uiDebugTemp1  += (uint16_t)m_ucI2CRXDataBuffer[0];

			m_uiDebugTemp2   = (uint16_t)m_ucI2CRXDataBuffer[3];
			m_uiDebugTemp2 <<= 8;
			m_uiDebugTemp2  += (uint16_t)m_ucI2CRXDataBuffer[2];


			sprintf((char *)m_ucDebugBuffer,"CH0=%u, CH1=%u\r\n", m_uiDebugTemp1,m_uiDebugTemp2 );
		}

		DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
		return;
	}

	if(ucRXBuffer == 'y')
	{	// reset the TSL25911 reading engine on RIGHT side @ I2C1
		I2CTSL25911SensorReset(RIGHT);
		return;
	}

	//------------------------

	if(ucRXBuffer == 'P')
	{
		// read ID from TSL25911
		// TSL25911 I2C Address = 0x52 (after shifting, in PDF is 0x29)
		// ID Register Address = 0x12 + 0x20 (normal operation) + 0x80 (command mode) => 1011 0010 = 0xB2
		// ID Expected Value = 0x50 (Device ID)
		if(HAL_I2C_Mem_Read(&hi2c3, 0x52, 0xB2, 1,m_ucI2CRXDataBuffer,1, HAL_MAX_DELAY) != HAL_OK)
		{
			//Error_Handler2(ERROR_CODE_I2CDEVICES_DEBUG_001);
			sprintf((char *)m_ucDebugBuffer,"(01)TSL25911 ID Read Error\r\n");
		}
		else
		{
			sprintf((char *)m_ucDebugBuffer,"ID=0x%02X\r\n", m_ucI2CRXDataBuffer[0]);
		}

		DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
		return;
	}

	if(ucRXBuffer == 'O')
	{	// TSL25911
		// TSL25911 I2C Address = 0x52 (after shifting, in PDF is 0x29)
		// Enable Register Address = 0x00 + (Normal Command = 0xA0) => Register Address = 0xA0
		// Enable Register Data    = 0000 0011 = 0x03 (AEN + PON)
		m_ucI2CTXDataBuffer[0] = 0x03;		// (AEN + PON)

		// HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
		if(HAL_I2C_Mem_Write(&hi2c3, 0x52, 0xA0, 1,m_ucI2CTXDataBuffer,1, HAL_MAX_DELAY) != HAL_OK)
		{
			//Error_Handler2(ERROR_CODE_I2CDEVICES_DEBUG_001);
			sprintf((char *)m_ucDebugBuffer,"(02)TSL25911 ID Read Error\r\n");
			DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
		}

		return;
	}

	if(ucRXBuffer == 'I')
	{	// TSL25911
		// TSL25911 I2C Address = 0x52 (after shifting, in PDF is 0x29)
		// Control Register Address = 0x01 + (Normal Command = 0xA0) => Register Address = 0xA1
		// Control Register Data    = 0001 0000 = 0x10 (AGAIN = MEDIUM)
		m_ucI2CTXDataBuffer[0] = 0x10;		// (AGAIN = MEDIUM)

		// HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
		if(HAL_I2C_Mem_Write(&hi2c3, 0x52, 0xA1, 1,m_ucI2CTXDataBuffer,1, HAL_MAX_DELAY) != HAL_OK)
		{
			//Error_Handler2(ERROR_CODE_I2CDEVICES_DEBUG_001);
			sprintf((char *)m_ucDebugBuffer,"(03)TSL25911 ID Read Error\r\n");
			DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
		}

		return;
	}

	if(ucRXBuffer == 'U')
	{	// TSL25911
		// TSL25911 I2C Address = 0x52 (after shifting, in PDF is 0x29)
		// ALS Data Register Address = 0x14 + (Normal Command = 0xA0) => Register Address = 0xB4
		// ALS Data Register Data    = XX,YY
		if(HAL_I2C_Mem_Read(&hi2c3, 0x52, 0xB4, 1,m_ucI2CRXDataBuffer,4, HAL_MAX_DELAY) != HAL_OK)
		{
			//Error_Handler2(ERROR_CODE_I2CDEVICES_DEBUG_001);
			sprintf((char *)m_ucDebugBuffer,"(04)TSL25911 ID Read Error\r\n");
		}
		else
		{
			m_uiDebugTemp1   = (uint16_t)m_ucI2CRXDataBuffer[1];
			m_uiDebugTemp1 <<= 8;
			m_uiDebugTemp1  += (uint16_t)m_ucI2CRXDataBuffer[0];

			m_uiDebugTemp2   = (uint16_t)m_ucI2CRXDataBuffer[3];
			m_uiDebugTemp2 <<= 8;
			m_uiDebugTemp2  += (uint16_t)m_ucI2CRXDataBuffer[2];


			sprintf((char *)m_ucDebugBuffer,"CH0=%u, CH1=%u\r\n", m_uiDebugTemp1,m_uiDebugTemp2 );
		}

		DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
		return;
	}

	if(ucRXBuffer == 'Y')
	{	// reset the TSL25911 reading engine on LEFT side @ I2C3
		I2CTSL25911SensorReset(LEFT);
		return;
	}

	// TSL25911 ---------------------------------------------- END
#endif

#ifdef I2CTSL24C512_DEBUG_MODE
	// 24C512 ------------------------------------------------ START
	if(ucRXBuffer == 'z')
	{
		// read from the 24C512 EEPROM
		//m_ucTXDataBuffer[0] = 0x0c;	// command to get the config
		// HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
		if(HAL_I2C_Mem_Read(&hi2c1, 0xA0, 10, 2,m_ucI2CRXDataBuffer,20, HAL_MAX_DELAY) != HAL_OK)
		{
			//Error_Handler2(ERROR_CODE_I2CDEVICES_DEBUG_001);
		}
		else
		{
			sprintf((char *)m_ucDebugBuffer,"EE=%u,%u,%u,%u,%u,%u\r\n", m_ucI2CRXDataBuffer[0],m_ucI2CRXDataBuffer[1],m_ucI2CRXDataBuffer[2],m_ucI2CRXDataBuffer[3],m_ucI2CRXDataBuffer[4],m_ucI2CRXDataBuffer[5]);
			DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
		}


		return;
	}

	if(ucRXBuffer == 'x')
	{
		m_ucI2CTXDataBuffer[0] = 11;		//
		m_ucI2CTXDataBuffer[1] = 23;		//
		m_ucI2CTXDataBuffer[2] = 34;		//
		m_ucI2CTXDataBuffer[3] = 45;		//
		m_ucI2CTXDataBuffer[4] = 67;		//
		m_ucI2CTXDataBuffer[5] = 89;		//

		// HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
		if(HAL_I2C_Mem_Write(&hi2c1, 0xA0, 10, 2,m_ucI2CTXDataBuffer,6, HAL_MAX_DELAY) != HAL_OK)
		{
			//Error_Handler2(ERROR_CODE_I2CDEVICES_DEBUG_001);
		}

		return;
	}

	if(ucRXBuffer == 'c')
	{
		m_ucI2CTXDataBuffer[0] = 56;		//
		m_ucI2CTXDataBuffer[1] = 93;		//
		m_ucI2CTXDataBuffer[2] = 74;		//
		m_ucI2CTXDataBuffer[3] = 11;		//
		m_ucI2CTXDataBuffer[4] = 22;		//
		m_ucI2CTXDataBuffer[5] = 44;		//

		// HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
		if(HAL_I2C_Mem_Write(&hi2c1, 0xA0, 10, 2,m_ucI2CTXDataBuffer,6, HAL_MAX_DELAY) != HAL_OK)
		{
			//Error_Handler2(ERROR_CODE_I2CDEVICES_DEBUG_001);
		}

		return;
	}
	// 24C512 ------------------------------------------------ END
#endif






#ifdef I2CVEML3328_DEBUG_MODE
	// VEML3328 --------------------------------------------- START
	if(ucRXBuffer == 'e')
	{
		//m_ucTXDataBuffer[0] = 0x00;	// command to get the config
		// HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
		if(HAL_I2C_Mem_Read(&hi2c1, 0x20, 0x00, 1,m_ucI2CRXDataBuffer,2, HAL_MAX_DELAY) != HAL_OK)
		{
			//Error_Handler2(ERROR_CODE_I2CDEVICES_DEBUG_001);
		}


		return;
	}

	if(ucRXBuffer == 'r')
	{
		//m_ucTXDataBuffer[0] = 0x0c;	// command to get the config
		// HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
		if(HAL_I2C_Mem_Read(&hi2c1, 0x20, 0x0c, 1,m_ucI2CRXDataBuffer,2, HAL_MAX_DELAY) != HAL_OK)
		{
			//Error_Handler2(ERROR_CODE_I2CDEVICES_DEBUG_001);
		}


		return;
	}
	if(ucRXBuffer == 't')
	{
		m_ucI2CTXDataBuffer[0] = 0x00;	// command to get the config
		m_ucI2CTXDataBuffer[1] = 0x00;	// command to get the config
		// HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
		if(HAL_I2C_Mem_Write(&hi2c1, 0x20, 0x00, 1,m_ucI2CTXDataBuffer,2, HAL_MAX_DELAY) != HAL_OK)
		{
			//Error_Handler2(ERROR_CODE_I2CDEVICES_DEBUG_001);
		}


		return;
	}

	if(ucRXBuffer == 'a')
	{
		// HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
		if(HAL_I2C_Mem_Read(&hi2c1, 0x20, 0x04, 1,m_ucI2CRXDataBuffer,2, HAL_MAX_DELAY) != HAL_OK)
		{
			//Error_Handler2(ERROR_CODE_I2CDEVICES_DEBUG_001);
		}
		return;
	}

	if(ucRXBuffer == 's')
	{
		// HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
		if(HAL_I2C_Mem_Read(&hi2c1, 0x20, 0x05, 1,m_ucI2CRXDataBuffer,2, HAL_MAX_DELAY) != HAL_OK)
		{
			//Error_Handler2(ERROR_CODE_I2CDEVICES_DEBUG_001);
		}
		return;
	}

	if(ucRXBuffer == 'd')
	{
		// HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
		if(HAL_I2C_Mem_Read(&hi2c1, 0x20, 0x06, 1,m_ucI2CRXDataBuffer,2, HAL_MAX_DELAY) != HAL_OK)
		{
			//Error_Handler2(ERROR_CODE_I2CDEVICES_DEBUG_001);
		}
		return;
	}

	if(ucRXBuffer == 'f')
	{
		// HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
		if(HAL_I2C_Mem_Read(&hi2c1, 0x20, 0x07, 1,m_ucI2CRXDataBuffer,2, HAL_MAX_DELAY) != HAL_OK)
		{
			//Error_Handler2(ERROR_CODE_I2CDEVICES_DEBUG_001);
		}
		return;
	}

	if(ucRXBuffer == 'g')
	{
		// HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
		if(HAL_I2C_Mem_Read(&hi2c1, 0x20, 0x08, 1,m_ucI2CRXDataBuffer,2, HAL_MAX_DELAY) != HAL_OK)
		{
			//Error_Handler2(ERROR_CODE_I2CDEVICES_DEBUG_001);
		}
		return;
	}


	if(ucRXBuffer == 'E')
	{
		//m_ucTXDataBuffer[0] = 0x00;	// command to get the config
		// HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
		if(HAL_I2C_Mem_Read(&hi2c3, 0x20, 0x00, 1,m_ucI2CRXDataBuffer,2, HAL_MAX_DELAY) != HAL_OK)
		{
			//Error_Handler2(ERROR_CODE_I2CDEVICES_DEBUG_001);
		}


		return;
	}
	// VEML3328 --------------------------------------------- END
#endif





	return;
}
#endif
//----------------------------------------------------------------------------------
//==================================================================================================================== SYSTEM_END



// STM32 HAL CALLBACK
//==================================================================================================================== HAL_START
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)		// TX Complete callback
{	//UNUSED(hi2c);
  if(hi2c->Instance == I2C1)
  {	// I2C1 = RIGHT Side => TX done on I2C1

	  // ----------- VEML3328 -----------
	  // check if the VEML3328 configuration is in progress (command 0x00 to set the device to normal operation mode and not power down mode or standby SD0=0, SD1=0)
	  if( (RightVEML3328.I2CReadingState == I2CSetConfigRegister) )
	  {		// VEML3328 communication is in progress.
		  RightVEML3328.TXCallbackStatus = CALLBACK_RECEIVED;	// the last I2C operation is finished
		  return;
	  }
	  // --------------------------------

	  // ----------- TSL25911 -----------
	  //if( RightTSL25911.ucOperationStatus == TSL25911_TRY_2_ENABLE )
	  //{	// Enabling (write) DONE
	  //  	RightTSL25911.ucOperationStatus = TSL25911_ENABLED;
	  //	return;
	  //}
	  //if( RightTSL25911.ucOperationStatus == TSL25911_TRY_2_CONFIG )
	  //{	// Configuring (write) DONE
	//	  RightTSL25911.ucOperationStatus = TSL25911_CONFIGURED;
	//	  return;
	 // }
	  // --------------------------------

	  // ------------ 24C512 ------------
	  // --------------------------------

	  // ------------ LSM9DS1 -----------
	  // --------------------------------

	  // ------------ INA219 ------------
	  // --------------------------------







	  return;
  }

  if(hi2c->Instance == I2C3)
  {	// I2C3 = LEFT Side => TX done on I2C3

	  // ----------- VEML3328 -----------
	  // check if the VEML3328 configuration is in progress (command 0x00 to set the device to normal operation mode and not power down mode or standby SD0=0, SD1=0)
	  if( (LeftVEML3328.I2CReadingState == I2CSetConfigRegister) )
	  {		// VEML3328 communication is in progress.
		  LeftVEML3328.TXCallbackStatus = CALLBACK_RECEIVED;	// the last I2C operation is finished
		  return;
	  }
	  // --------------------------------

	  // ----------- TSL25911 -----------
	  //if( LeftTSL25911.ucOperationStatus == TSL25911_TRY_2_ENABLE )
	  //{	// Enabling (write) DONE
	//	  LeftTSL25911.ucOperationStatus = TSL25911_ENABLED;
	//	  return;
	 // }
	 //if( LeftTSL25911.ucOperationStatus == TSL25911_TRY_2_CONFIG )
	 //{	// Configuring (write) DONE
	//	  LeftTSL25911.ucOperationStatus = TSL25911_CONFIGURED;
	//	  return;
	  //}
	  // --------------------------------








	  return;
  }



  return;
}
//----------------------------------------------------------------------------------
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)		// RX Complete callback
{	//UNUSED(hi2c);
  if(hi2c->Instance == I2C1)
  {	// I2C1 = RIGHT Side => RX done on I2C1

	  // ----------- VEML3328 -----------
	  // check if the VEML3328 reading is in progress
	  if( (RightVEML3328.I2CReadingState != I2CReadingDone) && (RightVEML3328.I2CReadingState != NoI2CReadingInProgress) )
	  {		// VEML3328 communication is in progress, and the last reading is now done.
		  RightVEML3328.RXCallbackStatus = CALLBACK_RECEIVED;	// the last I2C operation is finished
		  return;
	  }
	  // --------------------------------

	  // ----------- TSL25911 -----------
	  if( RightTSL25911.ucOperationStatus == TSL25911_TRY_2_READ )
	  {	// Reading (read) DONE
		  RightTSL25911.ucOperationStatus = TSL25911_CH_READ_DONE;
		  return;
	  }
	  // --------------------------------

	  // ------------ 24C512 ------------
	  // --------------------------------

	  // ------------ LSM9DS1 -----------
	  // --------------------------------

	  // ------------ INA219 ------------
	  // --------------------------------




	  return;
  }

  if(hi2c->Instance == I2C3)
  {	// I2C3 = LEFT Side => RX done on I2C2

	  // ----------- VEML3328 -----------
	  // check if the VEML3328 reading is in progress
	  if( (LeftVEML3328.I2CReadingState != I2CReadingDone) && (LeftVEML3328.I2CReadingState != NoI2CReadingInProgress) )
	  {		// VEML3328 communication is in progress, and the last reading is now done.
		  LeftVEML3328.RXCallbackStatus = CALLBACK_RECEIVED;	// the last I2C operation is finished
		  return;
	  }
	  // --------------------------------

	  // ----------- TSL25911 -----------
	  if( LeftTSL25911.ucOperationStatus == TSL25911_TRY_2_READ )
	  {	// Reading (read) DONE
		  LeftTSL25911.ucOperationStatus = TSL25911_CH_READ_DONE;
		  return;
	  }
	  // --------------------------------





	  return;
  }



  return;

}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== HAL_END




// STATIC (INTERN) implementation
//==================================================================================================================== STATIC_START
// VEML3328 section
//----------------------------------------------------------------------------------
/*
static void HandleRightVEML3328Loop(void)
{
	// ----------- STEP 0 ------------
	if( (RightVEML3328.I2CReadingState == NoI2CReadingInProgress) )
	{	// no reading is in progress => start a new reading
		// HAL_StatusTypeDef HAL_I2C_Mem_Read_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
		HAL_ErrorCode = HAL_I2C_Mem_Read_IT(&hi2c1, RightVEML3328.uiI2CAddress, (uint16_t)0x00, (uint16_t)1, m_ucI2CRXDataBuffer, (uint16_t)2);
		//if(HAL_ErrorCode == HAL_BUSY)
		//{	// not yet finished the previous I2C operation
			// !!! some timeout may be useful !!! (there is the system timeout)
		//	return;
		//}
		if( (HAL_ErrorCode != HAL_OK) )
		{	// it can also be that the return was HAL_BUSY, but just treat it as an ERROR
			VEML3328ErrorDetected(&RightVEML3328,ERROR_VEML3328_01);		// set the error on the sensor data
			RightVEML3328.I2CReadingState = I2CReadingDone;	// indicate that the reading is done because of some I2CError
			return;
		}
		RightVEML3328.I2CReadingState = I2CReadingConfigRegister;	// indicate that the conf. register is expected to be read by the I2C engine (waiting for RXCallback)
		return;
	}
	// -------------------------------

	// ----------- STEP 1 ------------
	if( (RightVEML3328.I2CReadingState == I2CReadingConfigRegister) )
	{	// no reading is in progress => start a new reading
		// check first if the I2CRX complete callback was received after the previous step
		if( (RightVEML3328.RXCallbackStatus != CALLBACK_RECEIVED) )
		{	// previous operation not yet done
			// !!! some timeout may be useful !!!
			return;
		}
		// else => callback was received so get to the next step and clear the callback flag
		RightVEML3328.RXCallbackStatus = CALLBACK_NOT_RECEIVED;	// clear the callback flag
		// store the status register data in the sensor variable
		RightVEML3328.MeasuredData.uiConfiguration   = (uint16_t)m_ucI2CRXDataBuffer[1];	// high byte
		RightVEML3328.MeasuredData.uiConfiguration <<= 8;	// shift to high part
		RightVEML3328.MeasuredData.uiConfiguration  |= (uint16_t)m_ucI2CRXDataBuffer[0];	// low byte
		// check if the sensor is in power down mode or not
		if( (RightVEML3328.MeasuredData.uiConfiguration & (0x0001<<15U)) || (RightVEML3328.MeasuredData.uiConfiguration & (0x0001 << 0U)) )
		{	// at least one of the SD1 or SD0 are not set to 0 => both must be set to 0 to enable POWER ON mode of the sensor
			m_ucI2CRXDataBuffer[0] = 0;
			m_ucI2CRXDataBuffer[1] = 0;
			// HAL_StatusTypeDef HAL_I2C_Mem_Write_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
			HAL_ErrorCode = HAL_I2C_Mem_Write_IT(&hi2c1, RightVEML3328.uiI2CAddress, (uint16_t)0x00, (uint16_t)1, m_ucI2CRXDataBuffer, (uint16_t)2);
			if( (HAL_ErrorCode != HAL_OK) )
			{	// it can also be that the return was HAL_BUSY, but just treat it as an ERROR
				VEML3328ErrorDetected(&RightVEML3328,ERROR_VEML3328_02);		// set the error on the sensor data
				RightVEML3328.I2CReadingState = I2CReadingDone;	// indicate that the reading is done because of some I2CError
				return;
			}
			RightVEML3328.I2CReadingState = I2CSetConfigRegister;	// indicate that the conf. register is expected to be read by the I2C engine (now waiting for TXCallback)
			return;
		}
		else
		{	// SD0 and SD1 are both in logical 0 state => the sensor is in POWER ON mode => can read the CLEAR data
			// HAL_StatusTypeDef HAL_I2C_Mem_Read_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
			HAL_ErrorCode = HAL_I2C_Mem_Read_IT(&hi2c1, RightVEML3328.uiI2CAddress, (uint16_t)0x04, (uint16_t)1, m_ucI2CRXDataBuffer, (uint16_t)2);
			if( (HAL_ErrorCode != HAL_OK) )
			{	// it can also be that the return was HAL_BUSY, but just treat it as an ERROR
				VEML3328ErrorDetected(&RightVEML3328,ERROR_VEML3328_03);		// set the error on the sensor data
				RightVEML3328.I2CReadingState = I2CReadingDone;		// indicate that the reading is done because of some I2CError
				return;
			}
			RightVEML3328.I2CReadingState = I2CReadingClearData;	// indicate that the CLEAR. register is expected to be read by the I2C engine (waiting for RXCallback)
			return;
		}

	}
	// -------------------------------

	// ---------- STEP (2) ---------- (if the configuration register was previously configured this step is skipped)
	if( (RightVEML3328.I2CReadingState == I2CSetConfigRegister) )
	{	// the configuration register was set and now continue with reading the CLEAR data register
		if( (RightVEML3328.TXCallbackStatus != CALLBACK_RECEIVED) )
		{	// last operation is not yet done .. must wait longer
			// !!! some timeout may be useful !!!
			return;
		}
		// else => callback was received so get to the next step and clear the callback flag
		RightVEML3328.TXCallbackStatus = CALLBACK_NOT_RECEIVED;	// clear the callback flag

		// HAL_StatusTypeDef HAL_I2C_Mem_Read_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
		HAL_ErrorCode = HAL_I2C_Mem_Read_IT(&hi2c1, RightVEML3328.uiI2CAddress, (uint16_t)0x04, (uint16_t)1, m_ucI2CRXDataBuffer, (uint16_t)2);
		if( (HAL_ErrorCode != HAL_OK) )
		{	// it can also be that the return was HAL_BUSY, but just treat it as an ERROR
			VEML3328ErrorDetected(&RightVEML3328,ERROR_VEML3328_04);		// set the error on the sensor data
			RightVEML3328.I2CReadingState = I2CReadingDone;	// indicate that the reading is done because of some I2CError
			return;
		}
		RightVEML3328.I2CReadingState = I2CReadingClearData;	// indicate that the CLEAR. register is expected to be read by the I2C engine (waiting for RXCallback)
		return;
	}
	// -------------------------------

	// ----------- STEP 3 ------------
	if( (RightVEML3328.I2CReadingState == I2CReadingClearData) )
	{	// check if reading CLEAR register is done
		if( (RightVEML3328.RXCallbackStatus != CALLBACK_RECEIVED) )
		{	// not yet, last operation is not yet done .. must wait longer
			// !!! some timeout may be useful !!!
			return;
		}
		// else => callback was received so get to the next step and clear the callback flag
		RightVEML3328.RXCallbackStatus = CALLBACK_NOT_RECEIVED;	// clear the callback flag

		// store the CLEAR data in the Sensor variable
		RightVEML3328.MeasuredData.uiClear   = (uint16_t)m_ucI2CRXDataBuffer[1];	// high byte
		RightVEML3328.MeasuredData.uiClear <<= 8;	//(*256)
		RightVEML3328.MeasuredData.uiClear  += (uint16_t)m_ucI2CRXDataBuffer[0];	// low byte

		// start next I2C operation
		// HAL_StatusTypeDef HAL_I2C_Mem_Read_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
		HAL_ErrorCode = HAL_I2C_Mem_Read_IT(&hi2c1, RightVEML3328.uiI2CAddress, (uint16_t)0x05, (uint16_t)1, m_ucI2CRXDataBuffer, (uint16_t)2);
		if( (HAL_ErrorCode != HAL_OK) )
		{	// it can also be that the return was HAL_BUSY, but just treat it as an ERROR
			VEML3328ErrorDetected(&RightVEML3328,ERROR_VEML3328_05);		// set the error on the sensor data
			RightVEML3328.I2CReadingState = I2CReadingDone;	// indicate that the reading is done because of some I2CError
			return;
		}


		RightVEML3328.I2CReadingState = I2CReadingRedData;	// indicate that the RED. register is expected to be read by the I2C engine (waiting for RXCallback)
		return;
	}
	// -------------------------------

	// ----------- STEP 4 ------------
	if( (RightVEML3328.I2CReadingState == I2CReadingRedData) )
	{	// check if reading RED register is done
		if( (RightVEML3328.RXCallbackStatus != CALLBACK_RECEIVED) )
		{	// not yet, last operation is not yet done .. must wait longer
			// !!! some timeout may be useful !!!
			return;
		}
		// else => callback was received so get to the next step and clear the callback flag
		RightVEML3328.RXCallbackStatus = CALLBACK_NOT_RECEIVED;	// clear the callback flag

		// store the RED data in the Sensor variable
		RightVEML3328.MeasuredData.uiRed   = (uint16_t)m_ucI2CRXDataBuffer[1];	// high byte
		RightVEML3328.MeasuredData.uiRed <<= 8;	//(*256)
		RightVEML3328.MeasuredData.uiRed  += (uint16_t)m_ucI2CRXDataBuffer[0];	// low byte

		// start next I2C operation
		// HAL_StatusTypeDef HAL_I2C_Mem_Read_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
		HAL_ErrorCode = HAL_I2C_Mem_Read_IT(&hi2c1, RightVEML3328.uiI2CAddress, (uint16_t)0x06, (uint16_t)1, m_ucI2CRXDataBuffer, (uint16_t)2);
		if( (HAL_ErrorCode != HAL_OK) )
		{	// it can also be that the return was HAL_BUSY, but just treat it as an ERROR
			VEML3328ErrorDetected(&RightVEML3328,ERROR_VEML3328_06);		// set the error on the sensor data
			RightVEML3328.I2CReadingState = I2CReadingDone;	// indicate that the reading is done because of some I2CError
			return;
		}


		RightVEML3328.I2CReadingState = I2CReadingGreenData;	// indicate that the GREEN. register is expected to be read by the I2C engine (waiting for RXCallback)
		return;
	}
	// -------------------------------

	// ----------- STEP 5 ------------
	if( (RightVEML3328.I2CReadingState == I2CReadingGreenData) )
	{	// check if reading GREEN register is done
		if( (RightVEML3328.RXCallbackStatus != CALLBACK_RECEIVED) )
		{	// not yet, last operation is not yet done .. must wait longer
			// !!! some timeout may be useful !!!
			return;
		}
		// else => callback was received so get to the next step and clear the callback flag
		RightVEML3328.RXCallbackStatus = CALLBACK_NOT_RECEIVED;	// clear the callback flag

		// store the GREEN data in the Sensor variable
		RightVEML3328.MeasuredData.uiGreen   = (uint16_t)m_ucI2CRXDataBuffer[1];	// high byte
		RightVEML3328.MeasuredData.uiGreen <<= 8;	//(*256)
		RightVEML3328.MeasuredData.uiGreen  += (uint16_t)m_ucI2CRXDataBuffer[0];	// low byte

		// start next I2C operation
		// HAL_StatusTypeDef HAL_I2C_Mem_Read_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
		HAL_ErrorCode = HAL_I2C_Mem_Read_IT(&hi2c1, RightVEML3328.uiI2CAddress, (uint16_t)0x07, (uint16_t)1, m_ucI2CRXDataBuffer, (uint16_t)2);
		if( (HAL_ErrorCode != HAL_OK) )
		{	// it can also be that the return was HAL_BUSY, but just treat it as an ERROR
			VEML3328ErrorDetected(&RightVEML3328,ERROR_VEML3328_07);		// set the error on the sensor data
			RightVEML3328.I2CReadingState = I2CReadingDone;	// indicate that the reading is done because of some I2CError
			return;
		}


		RightVEML3328.I2CReadingState = I2CReadingBlueData;	// indicate that the BLUE. register is expected to be read by the I2C engine (waiting for RXCallback)
		return;
	}
	// -------------------------------

	// ----------- STEP 6 ------------
	if( (RightVEML3328.I2CReadingState == I2CReadingBlueData) )
	{	// check if reading BLUE register is done
		if( (RightVEML3328.RXCallbackStatus != CALLBACK_RECEIVED) )
		{	// not yet, last operation is not yet done .. must wait longer
			// !!! some timeout may be useful !!!
			return;
		}
		// else => callback was received so get to the next step and clear the callback flag
		RightVEML3328.RXCallbackStatus = CALLBACK_NOT_RECEIVED;	// clear the callback flag

		// store the BLUE data in the Sensor variable
		RightVEML3328.MeasuredData.uiBlue   = (uint16_t)m_ucI2CRXDataBuffer[1];	// high byte
		RightVEML3328.MeasuredData.uiBlue <<= 8;	//(*256)
		RightVEML3328.MeasuredData.uiBlue  += (uint16_t)m_ucI2CRXDataBuffer[0];	// low byte

		// start next I2C operation
		// HAL_StatusTypeDef HAL_I2C_Mem_Read_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
		HAL_ErrorCode = HAL_I2C_Mem_Read_IT(&hi2c1, RightVEML3328.uiI2CAddress, (uint16_t)0x08, (uint16_t)1, m_ucI2CRXDataBuffer, (uint16_t)2);
		if( (HAL_ErrorCode != HAL_OK) )
		{	// it can also be that the return was HAL_BUSY, but just treat it as an ERROR
			VEML3328ErrorDetected(&RightVEML3328,ERROR_VEML3328_08);		// set the error on the sensor data
			RightVEML3328.I2CReadingState = I2CReadingDone;	// indicate that the reading is done because of some I2CError
			return;
		}


		RightVEML3328.I2CReadingState = I2CReadingIRData;	// indicate that the IR. register is expected to be read by the I2C engine (waiting for RXCallback)
		return;
	}
	// -------------------------------

	// ----------- STEP 7 ------------
	if( (RightVEML3328.I2CReadingState == I2CReadingIRData) )
	{	// check if reading IR register is done
		if( (RightVEML3328.RXCallbackStatus != CALLBACK_RECEIVED) )
		{	// not yet, last operation is not yet done .. must wait longer
			// !!! some timeout may be useful !!!
			return;
		}
		// else => callback was received so get to the next step and clear the callback flag
		RightVEML3328.RXCallbackStatus = CALLBACK_NOT_RECEIVED;	// clear the callback flag

		// store the IR data in the Sensor variable
		RightVEML3328.MeasuredData.uiIR   = (uint16_t)m_ucI2CRXDataBuffer[1];	// high byte
		RightVEML3328.MeasuredData.uiIR <<= 8;	//(*256)
		RightVEML3328.MeasuredData.uiIR  += (uint16_t)m_ucI2CRXDataBuffer[0];	// low byte

		// All reading is done, now save the data and call the callback function
		RightVEML3328.MeasuredData.uiError = 0;	// no error detected
		StoreVEML3328Result(RightVEML3328);
		//RightVEML3328.MeasuredData.NewMeasuredData = NEW_DATA_IS_AVAILABLE;	// indicating that new data is available

		// CALLBACK
		// uncomment in StoreVEML3328Result() if the callback is used only with valid data (so callback will not be called if error occurred)
		//I2CVEML3328DataAvailableCallback(RIGHT);	// in this case also call if error occurred

		RightVEML3328.I2CReadingState = I2CReadingDone;	// indicate that the IR. register is expected to be read by the I2C engine (waiting for RXCallback)

		// CALLBACK for both sensors done
		if(LeftVEML3328.MeasuredData.NewMeasuredData == NEW_DATA_IS_AVAILABLE)
		{
			LeftVEML3328.MeasuredData.NewMeasuredData = NO_NEW_DATA_IS_AVAILABLE;
			// call the callback function
			// this callback will be called only after both VEML3328 sensors were read and the results are ok (in case of error at least with one sensor this function will not be called)
			I2CVEML3328BothDataAvailableCallback();
			return;
		}
		else 	RightVEML3328.MeasuredData.NewMeasuredData = NEW_DATA_IS_AVAILABLE;		// mark the data available properties for the RIGHT sensor

		return;
	}
	// -------------------------------



	return;
}
*/
//----------------------------------------------------------------------------------
static void HandleVEML3328Loop(VEML3328TypeDef *VEML3328)
{
	// ----------- STEP 0 ------------
	if( (VEML3328->I2CReadingState == NoI2CReadingInProgress) )
	{	// no reading is in progress => start a new reading
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);	//  debug on PC13 HIGH
		// HAL_StatusTypeDef HAL_I2C_Mem_Read_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
		// !!!!! in case of I2C Error this function "HAL_I2C_Mem_Read_IT()" takes up to 30ms => STUPID HALL LAYER !!!!!
		if( (VEML3328->SensorSide == RIGHT) )		HAL_ErrorCode = HAL_I2C_Mem_Read_IT(&hi2c1, VEML3328->uiI2CAddress, (uint16_t)0x00, (uint16_t)1, m_ucI2CRXDataBuffer, (uint16_t)2);		// RIGHT => I2C1
		else										HAL_ErrorCode = HAL_I2C_Mem_Read_IT(&hi2c3, VEML3328->uiI2CAddress, (uint16_t)0x00, (uint16_t)1, m_ucI2CRXDataBuffer, (uint16_t)2);		// LEFT  => I2C3
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);  //	debug on PC13 LOW
		/*
		if(HAL_ErrorCode == HAL_BUSY)
		{	// not yet finished the previous I2C operation
			// !!! some timeout may be useful !!! (there is the system timeout)
			return;
		}
		*/
		if( (HAL_ErrorCode != HAL_OK) )
		{	// it can also be that the return was HAL_BUSY, but just treat it as an ERROR
			VEML3328ErrorDetected(VEML3328,ERROR_VEML3328_01);		// set the error on the sensor data
			VEML3328->I2CReadingState = I2CReadingDone;	// indicate that the reading is done because of some I2CError
			return;
		}
		VEML3328->I2CReadingState = I2CReadingConfigRegister;	// indicate that the conf. register is expected to be read by the I2C engine (waiting for RXCallback)
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);  //	debug on PC13 LOW
		return;
	}
	// -------------------------------

	// ----------- STEP 1 ------------
	if( (VEML3328->I2CReadingState == I2CReadingConfigRegister) )
	{	// no reading is in progress => start a new reading
		// check first if the I2CRX complete callback was received after the previous step
		if( (VEML3328->RXCallbackStatus != CALLBACK_RECEIVED) )
		{	// previous operation not yet done
			// !!! some timeout may be useful !!!
			return;
		}
		// else => callback was received so get to the next step and clear the callback flag
		VEML3328->RXCallbackStatus = CALLBACK_NOT_RECEIVED;	// clear the callback flag
		// store the status register data in the sensor variable
		VEML3328->MeasuredData.uiConfiguration   = (uint16_t)m_ucI2CRXDataBuffer[1];	// high byte
		VEML3328->MeasuredData.uiConfiguration <<= 8;	// shift to high part
		VEML3328->MeasuredData.uiConfiguration  |= (uint16_t)m_ucI2CRXDataBuffer[0];	// low byte
		// check if the sensor is in power down mode or not
		if( (VEML3328->MeasuredData.uiConfiguration & (0x0001<<15U)) || (VEML3328->MeasuredData.uiConfiguration & (0x0001 << 0U)) )
		{	// at least one of the SD1 or SD0 are not set to 0 => both must be set to 0 to enable POWER ON mode of the sensor
			m_ucI2CRXDataBuffer[0] = 0;
			m_ucI2CRXDataBuffer[1] = 0;
			// HAL_StatusTypeDef HAL_I2C_Mem_Write_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
			if( (VEML3328->SensorSide == RIGHT) ) 		HAL_ErrorCode = HAL_I2C_Mem_Write_IT(&hi2c1, VEML3328->uiI2CAddress, (uint16_t)0x00, (uint16_t)1, m_ucI2CRXDataBuffer, (uint16_t)2);		// RIGHT => I2C1
			else 										HAL_ErrorCode = HAL_I2C_Mem_Write_IT(&hi2c3, VEML3328->uiI2CAddress, (uint16_t)0x00, (uint16_t)1, m_ucI2CRXDataBuffer, (uint16_t)2);		// LEFT  => I2C3
			if( (HAL_ErrorCode != HAL_OK) )
			{	// it can also be that the return was HAL_BUSY, but just treat it as an ERROR
				VEML3328ErrorDetected(VEML3328,ERROR_VEML3328_02);		// set the error on the sensor data
				VEML3328->I2CReadingState = I2CReadingDone;	// indicate that the reading is done because of some I2CError
				return;
			}
			VEML3328->I2CReadingState = I2CSetConfigRegister;	// indicate that the conf. register is expected to be read by the I2C engine (now waiting for TXCallback)
			return;
		}
		else
		{	// SD0 and SD1 are both in logical 0 state => the sensor is in POWER ON mode => can read the CLEAR data
			// HAL_StatusTypeDef HAL_I2C_Mem_Read_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
			if( (VEML3328->SensorSide == RIGHT) ) 		HAL_ErrorCode = HAL_I2C_Mem_Read_IT(&hi2c1, VEML3328->uiI2CAddress, (uint16_t)0x04, (uint16_t)1, m_ucI2CRXDataBuffer, (uint16_t)2);		// RIGHT => I2C1
			else 										HAL_ErrorCode = HAL_I2C_Mem_Read_IT(&hi2c3, VEML3328->uiI2CAddress, (uint16_t)0x04, (uint16_t)1, m_ucI2CRXDataBuffer, (uint16_t)2);		// LEFT  => I2C3
			if( (HAL_ErrorCode != HAL_OK) )
			{	// it can also be that the return was HAL_BUSY, but just treat it as an ERROR
				VEML3328ErrorDetected(VEML3328,ERROR_VEML3328_03);		// set the error on the sensor data
				VEML3328->I2CReadingState = I2CReadingDone;		// indicate that the reading is done because of some I2CError
				return;
			}
			VEML3328->I2CReadingState = I2CReadingClearData;	// indicate that the CLEAR. register is expected to be read by the I2C engine (waiting for RXCallback)
			return;
		}

	}
	// -------------------------------

	// ---------- STEP (2) ---------- (if the configuration register was previously configured this step is skipped)
	if( (VEML3328->I2CReadingState == I2CSetConfigRegister) )
	{	// the configuration register was set and now continue with reading the CLEAR data register
		if( (VEML3328->TXCallbackStatus != CALLBACK_RECEIVED) )
		{	// last operation is not yet done .. must wait longer
			// !!! some timeout may be useful !!!
			return;
		}
		// else => callback was received so get to the next step and clear the callback flag
		VEML3328->TXCallbackStatus = CALLBACK_NOT_RECEIVED;	// clear the callback flag

		// HAL_StatusTypeDef HAL_I2C_Mem_Read_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
		if( (VEML3328->SensorSide == RIGHT) ) 		HAL_ErrorCode = HAL_I2C_Mem_Read_IT(&hi2c1, VEML3328->uiI2CAddress, (uint16_t)0x04, (uint16_t)1, m_ucI2CRXDataBuffer, (uint16_t)2); 		// RIGHT => I2C1
		else 										HAL_ErrorCode = HAL_I2C_Mem_Read_IT(&hi2c3, VEML3328->uiI2CAddress, (uint16_t)0x04, (uint16_t)1, m_ucI2CRXDataBuffer, (uint16_t)2);			// LEFT  => I2C3
		if( (HAL_ErrorCode != HAL_OK) )
		{	// it can also be that the return was HAL_BUSY, but just treat it as an ERROR
			VEML3328ErrorDetected(VEML3328,ERROR_VEML3328_04);		// set the error on the sensor data
			VEML3328->I2CReadingState = I2CReadingDone;	// indicate that the reading is done because of some I2CError
			return;
		}
		VEML3328->I2CReadingState = I2CReadingClearData;	// indicate that the CLEAR. register is expected to be read by the I2C engine (waiting for RXCallback)
		return;
	}
	// -------------------------------

	// ----------- STEP 3 ------------
	if( (VEML3328->I2CReadingState == I2CReadingClearData) )
	{	// check if reading CLEAR register is done
		if( (VEML3328->RXCallbackStatus != CALLBACK_RECEIVED) )
		{	// not yet, last operation is not yet done .. must wait longer
			// !!! some timeout may be useful !!!
			return;
		}
		// else => callback was received so get to the next step and clear the callback flag
		VEML3328->RXCallbackStatus = CALLBACK_NOT_RECEIVED;	// clear the callback flag

		// store the CLEAR data in the Sensor variable
		VEML3328->MeasuredData.uiClear   = (uint16_t)m_ucI2CRXDataBuffer[1];	// high byte
		VEML3328->MeasuredData.uiClear <<= 8;	//(*256)
		VEML3328->MeasuredData.uiClear  += (uint16_t)m_ucI2CRXDataBuffer[0];	// low byte

		// start next I2C operation
		// HAL_StatusTypeDef HAL_I2C_Mem_Read_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
		if( (VEML3328->SensorSide == RIGHT) ) 		HAL_ErrorCode = HAL_I2C_Mem_Read_IT(&hi2c1, VEML3328->uiI2CAddress, (uint16_t)0x05, (uint16_t)1, m_ucI2CRXDataBuffer, (uint16_t)2); 		// RIGHT => I2C1
		else 										HAL_ErrorCode = HAL_I2C_Mem_Read_IT(&hi2c3, VEML3328->uiI2CAddress, (uint16_t)0x05, (uint16_t)1, m_ucI2CRXDataBuffer, (uint16_t)2);			// LEFT  => I2C3
		if( (HAL_ErrorCode != HAL_OK) )
		{	// it can also be that the return was HAL_BUSY, but just treat it as an ERROR
			VEML3328ErrorDetected(VEML3328,ERROR_VEML3328_05);		// set the error on the sensor data
			VEML3328->I2CReadingState = I2CReadingDone;	// indicate that the reading is done because of some I2CError
			return;
		}


		VEML3328->I2CReadingState = I2CReadingRedData;	// indicate that the RED. register is expected to be read by the I2C engine (waiting for RXCallback)
		return;
	}
	// -------------------------------

	// ----------- STEP 4 ------------
	if( (VEML3328->I2CReadingState == I2CReadingRedData) )
	{	// check if reading RED register is done
		if( (VEML3328->RXCallbackStatus != CALLBACK_RECEIVED) )
		{	// not yet, last operation is not yet done .. must wait longer
			// !!! some timeout may be useful !!!
			return;
		}
		// else => callback was received so get to the next step and clear the callback flag
		VEML3328->RXCallbackStatus = CALLBACK_NOT_RECEIVED;	// clear the callback flag

		// store the RED data in the Sensor variable
		VEML3328->MeasuredData.uiRed   = (uint16_t)m_ucI2CRXDataBuffer[1];	// high byte
		VEML3328->MeasuredData.uiRed <<= 8;	//(*256)
		VEML3328->MeasuredData.uiRed  += (uint16_t)m_ucI2CRXDataBuffer[0];	// low byte

		// start next I2C operation
		// HAL_StatusTypeDef HAL_I2C_Mem_Read_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
		if( (VEML3328->SensorSide == RIGHT) ) 		HAL_ErrorCode = HAL_I2C_Mem_Read_IT(&hi2c1, VEML3328->uiI2CAddress, (uint16_t)0x06, (uint16_t)1, m_ucI2CRXDataBuffer, (uint16_t)2); 		// RIGHT => I2C1
		else 										HAL_ErrorCode = HAL_I2C_Mem_Read_IT(&hi2c3, VEML3328->uiI2CAddress, (uint16_t)0x06, (uint16_t)1, m_ucI2CRXDataBuffer, (uint16_t)2);			// LEFT  => I2C3
		if( (HAL_ErrorCode != HAL_OK) )
		{	// it can also be that the return was HAL_BUSY, but just treat it as an ERROR
			VEML3328ErrorDetected(VEML3328,ERROR_VEML3328_06);		// set the error on the sensor data
			VEML3328->I2CReadingState = I2CReadingDone;	// indicate that the reading is done because of some I2CError
			return;
		}


		VEML3328->I2CReadingState = I2CReadingGreenData;	// indicate that the GREEN. register is expected to be read by the I2C engine (waiting for RXCallback)
		return;
	}
	// -------------------------------

	// ----------- STEP 5 ------------
	if( (VEML3328->I2CReadingState == I2CReadingGreenData) )
	{	// check if reading GREEN register is done
		if( (VEML3328->RXCallbackStatus != CALLBACK_RECEIVED) )
		{	// not yet, last operation is not yet done .. must wait longer
			// !!! some timeout may be useful !!!
			return;
		}
		// else => callback was received so get to the next step and clear the callback flag
		VEML3328->RXCallbackStatus = CALLBACK_NOT_RECEIVED;	// clear the callback flag

		// store the GREEN data in the Sensor variable
		VEML3328->MeasuredData.uiGreen   = (uint16_t)m_ucI2CRXDataBuffer[1];	// high byte
		VEML3328->MeasuredData.uiGreen <<= 8;	//(*256)
		VEML3328->MeasuredData.uiGreen  += (uint16_t)m_ucI2CRXDataBuffer[0];	// low byte

		// start next I2C operation
		// HAL_StatusTypeDef HAL_I2C_Mem_Read_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
		if( (VEML3328->SensorSide == RIGHT) ) 		HAL_ErrorCode = HAL_I2C_Mem_Read_IT(&hi2c1, VEML3328->uiI2CAddress, (uint16_t)0x07, (uint16_t)1, m_ucI2CRXDataBuffer, (uint16_t)2); 		// RIGHT => I2C1
		else 										HAL_ErrorCode = HAL_I2C_Mem_Read_IT(&hi2c3, VEML3328->uiI2CAddress, (uint16_t)0x07, (uint16_t)1, m_ucI2CRXDataBuffer, (uint16_t)2);			// LEFT  => I2C3
		if( (HAL_ErrorCode != HAL_OK) )
		{	// it can also be that the return was HAL_BUSY, but just treat it as an ERROR
			VEML3328ErrorDetected(VEML3328,ERROR_VEML3328_07);		// set the error on the sensor data
			VEML3328->I2CReadingState = I2CReadingDone;	// indicate that the reading is done because of some I2CError
			return;
		}


		VEML3328->I2CReadingState = I2CReadingBlueData;	// indicate that the BLUE. register is expected to be read by the I2C engine (waiting for RXCallback)
		return;
	}
	// -------------------------------

	// ----------- STEP 6 ------------
	if( (VEML3328->I2CReadingState == I2CReadingBlueData) )
	{	// check if reading BLUE register is done
		if( (VEML3328->RXCallbackStatus != CALLBACK_RECEIVED) )
		{	// not yet, last operation is not yet done .. must wait longer
			// !!! some timeout may be useful !!!
			return;
		}
		// else => callback was received so get to the next step and clear the callback flag
		VEML3328->RXCallbackStatus = CALLBACK_NOT_RECEIVED;	// clear the callback flag

		// store the BLUE data in the Sensor variable
		VEML3328->MeasuredData.uiBlue   = (uint16_t)m_ucI2CRXDataBuffer[1];	// high byte
		VEML3328->MeasuredData.uiBlue <<= 8;	//(*256)
		VEML3328->MeasuredData.uiBlue  += (uint16_t)m_ucI2CRXDataBuffer[0];	// low byte

		// start next I2C operation
		// HAL_StatusTypeDef HAL_I2C_Mem_Read_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
		if( (VEML3328->SensorSide == RIGHT) ) 		HAL_ErrorCode = HAL_I2C_Mem_Read_IT(&hi2c1, VEML3328->uiI2CAddress, (uint16_t)0x08, (uint16_t)1, m_ucI2CRXDataBuffer, (uint16_t)2); 		// RIGHT => I2C1
		else 										HAL_ErrorCode = HAL_I2C_Mem_Read_IT(&hi2c3, VEML3328->uiI2CAddress, (uint16_t)0x08, (uint16_t)1, m_ucI2CRXDataBuffer, (uint16_t)2);			// LEFT  => I2C3
		if( (HAL_ErrorCode != HAL_OK) )
		{	// it can also be that the return was HAL_BUSY, but just treat it as an ERROR
			VEML3328ErrorDetected(VEML3328,ERROR_VEML3328_08);		// set the error on the sensor data
			VEML3328->I2CReadingState = I2CReadingDone;	// indicate that the reading is done because of some I2CError
			return;
		}


		VEML3328->I2CReadingState = I2CReadingIRData;	// indicate that the IR. register is expected to be read by the I2C engine (waiting for RXCallback)
		return;
	}
	// -------------------------------

	// ----------- STEP 7 ------------
	if( (VEML3328->I2CReadingState == I2CReadingIRData) )
	{	// check if reading IR register is done
		if( (VEML3328->RXCallbackStatus != CALLBACK_RECEIVED) )
		{	// not yet, last operation is not yet done .. must wait longer
			// !!! some timeout may be useful !!!
			return;
		}
		// else => callback was received so get to the next step and clear the callback flag
		VEML3328->RXCallbackStatus = CALLBACK_NOT_RECEIVED;	// clear the callback flag

		// store the IR data in the Sensor variable
		VEML3328->MeasuredData.uiIR   = (uint16_t)m_ucI2CRXDataBuffer[1];	// high byte
		VEML3328->MeasuredData.uiIR <<= 8;	//(*256)
		VEML3328->MeasuredData.uiIR  += (uint16_t)m_ucI2CRXDataBuffer[0];	// low byte

		// All reading is done, now save the data and call the callback function
		VEML3328->MeasuredData.uiError = 0;	// no error detected
		StoreVEML3328Result(*VEML3328);
		VEML3328->uiTimeout100HZ = 0;		// I2C action completed => I2C engine is not jammed
		//RightVEML3328.MeasuredData.NewMeasuredData = NEW_DATA_IS_AVAILABLE;	// indicating that new data is available

		// CALLBACK
		// uncomment in StoreVEML3328Result() if the callback is used only with valid data (so callback will not be called if error occurred)
		//I2CVEML3328DataAvailableCallback(VEML3328->SensorSide);	// in this case also call if error occurred

		VEML3328->I2CReadingState = I2CReadingDone;	// indicate that the reading has finished for this round
		VEML3328->MeasuredData.NewMeasuredData = NEW_DATA_IS_AVAILABLE;		// mark the data available properties for the RIGHT sensor

		return;
	}
	// -------------------------------

	return;
}
//----------------------------------------------------------------------------------
static void VEML3328ErrorDetected(VEML3328TypeDef *VEML3328, uint16_t uiError)
{	// used to handle error cases during the VEML3328 Reading
	VEML3328->MeasuredData.uiError |= uiError;
	VEML3328->MeasuredData.uiConfiguration = 0xFFFF;	// some unreasonable value indicating the value is not real
	VEML3328->MeasuredData.uiClear = 0;
	VEML3328->MeasuredData.uiRed = 0;
	VEML3328->MeasuredData.uiGreen = 0;
	VEML3328->MeasuredData.uiBlue = 0;
	VEML3328->MeasuredData.uiIR = 0;
	//VEML3328->MeasuredData.NewMeasuredData = NEW_DATA_IS_AVAILABLE;	// new measurement is completed but with error

	VEML3328->RXCallbackStatus = CALLBACK_NOT_RECEIVED;
	VEML3328->TXCallbackStatus = CALLBACK_NOT_RECEIVED;

	StoreVEML3328Result(*VEML3328);

	return;
}
//----------------------------------------------------------------------------------
static void StoreVEML3328Result(VEML3328TypeDef VEML3328)	// used to store the result in the global variable to be available any time for the API Sensor Data Reading
{

	if(VEML3328.SensorSide == RIGHT)
	{	// RIGHT Side
		RightVEML3328Result.uiError = VEML3328.MeasuredData.uiError;
		RightVEML3328Result.uiConfiguration = VEML3328.MeasuredData.uiConfiguration;
		RightVEML3328Result.uiClear = VEML3328.MeasuredData.uiClear;
		RightVEML3328Result.uiRed = VEML3328.MeasuredData.uiRed;
		RightVEML3328Result.uiGreen = VEML3328.MeasuredData.uiGreen;
		RightVEML3328Result.uiBlue = VEML3328.MeasuredData.uiBlue;
		RightVEML3328Result.uiIR = VEML3328.MeasuredData.uiIR;
		RightVEML3328Result.NewMeasuredData = NEW_DATA_IS_AVAILABLE;

		// callback
		// if the callback should only work in case there is no error then uncomment in "HandleRightVEML3328Loop()"
		I2CVEML3328DataAvailableCallback(RIGHT);	// in this case also call if error occurred
	}
	else
	{	// LEFT Side
		LeftVEML3328Result.uiError = VEML3328.MeasuredData.uiError;
		LeftVEML3328Result.uiConfiguration = VEML3328.MeasuredData.uiConfiguration;
		LeftVEML3328Result.uiClear = VEML3328.MeasuredData.uiClear;
		LeftVEML3328Result.uiRed = VEML3328.MeasuredData.uiRed;
		LeftVEML3328Result.uiGreen = VEML3328.MeasuredData.uiGreen;
		LeftVEML3328Result.uiBlue = VEML3328.MeasuredData.uiBlue;
		LeftVEML3328Result.uiIR = VEML3328.MeasuredData.uiIR;
		LeftVEML3328Result.NewMeasuredData = NEW_DATA_IS_AVAILABLE;

		// callback
		// if the callback should only work in case there is no error then uncomment in "HandleLeftVEML3328Loop()"
		I2CVEML3328DataAvailableCallback(LEFT);		// in this case also call if error occurred
	}
	return;
}
//----------------------------------------------------------------------------------
static void VEML3328RightTimeout100HZ(void)		// called from the system timer @ 100HZ rate. it is used to timeout the reading of the RIGHT VEML3328 color sensors
{	// called every 10mS <=> 100HZ

	if( (RightVEML3328.I2CReadingState == I2CReadingDone) || (RightVEML3328.I2CReadingState == NoI2CReadingInProgress) )
	{	// I2C communication is done, everything seems ok, the reading loop did not jam during the reading.
		RightVEML3328.uiTimeout100HZ = 0;
		return;
	}
	// I2C reading of the sensor is still in progress, must check if the I2C loop is not hanged due to some I2C error or bug in the code
	RightVEML3328.uiTimeout100HZ ++;

	if( (RightVEML3328.uiTimeout100HZ > 100) )	// each unit represents 10ms => 100 units is 1s
	{
		VEML3328ErrorDetected(&RightVEML3328,ERROR_VEML3328_09);		// set the error on the sensor data
		RightVEML3328.I2CReadingState = I2CReadingDone;	// indicate that the reading is done because of some I2CError
		RightVEML3328.uiTimeout100HZ = 0;
	}

	return;
}
//----------------------------------------------------------------------------------
static void VEML3328LeftTimeout100HZ(void)		// called from the system timer @ 100HZ rate. it is used to timeout the reading of the LEFT VEML3328 color sensors
{	// called every 10mS <=> 100HZ

	if((LeftVEML3328.I2CReadingState == I2CReadingDone) || (LeftVEML3328.I2CReadingState == NoI2CReadingInProgress))
	{	// I2C communication is done, everything seems ok, the reading loop did not jam during the reading.
		LeftVEML3328.uiTimeout100HZ = 0;
		return;
	}
	// I2C reading of the sensor is still in progress, must check if the I2C loop is not hanged due to some I2C error or bug in the code
	LeftVEML3328.uiTimeout100HZ ++;

	if( (LeftVEML3328.uiTimeout100HZ > 100) )	// each unit represents 10ms => 100 units is 1s
	{
		VEML3328ErrorDetected(&LeftVEML3328,ERROR_VEML3328_09);		// set the error on the sensor data
		LeftVEML3328.I2CReadingState = I2CReadingDone;	// indicate that the reading is done because of some I2CError
		LeftVEML3328.uiTimeout100HZ = 0;
	}

	return;
}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
// 24C512 section (@I2C1 channel)
// EEPROM 24C512
//volatile EEPROMDataTypeDef			EEpromData[32];		/* the maximum number of elements a list can hold */
//volatile EEPROMDataListTypeDef		EEpromDataList;		/* the data list that is used to operate the EEPROM in the main loop mode */
static void EEPROMInitialization(void)				// called from the initialization section
{
	EEpromDataList.pEEDTList = EEpromData;
	return;
}
//----------------------------------------------------------------------------------
static void EEPROMMainLoop(void)					// called from the system's main loop to handle list related READ / WRITE operations
{
	// !!!! before accessing the 24C512 EEPROM from I2C1 driver make sure that no other device has launched an asynchronous I2C operation and is waiting for the I2C complete interrupt !!!
	// VEML3328 Right 	@ I2C1 driver
	if( (RightVEML3328.I2CReadingState != I2CReadingDone) )					return;		// some VEM3328 I2C1 operation is still in progress => can not use the I2C driver yet
	// TSL25911 Right 	@ I2C1 driver
	if( (RightTSL25911.ucOperationStatus != TSL25911_CH_READ_DONE) )		return;		// some TSL25911 I2C1 operation is still in progress => can not use the I2C driver yet (not yet implemented)
	// LSM9DS1			@ I2C1 driver
	////if( (LSM9DS1.I2CReadingState != I2CReadingDone) )			return;		// some LSM9DS1 I2C1 operation is still in progress => can not use the I2C driver yet (not yet implemented)
	// INA219 			@ I2C1 driver
	////if( (INA219.I2CReadingState != I2CReadingDone) )			return;		// some INA219 I2C1 operation is still in progress => can not use the I2C driver yet (not yet implemented)
	//----------------------------------------------

	uint8_t i;	// for indexing
	uint16_t	ui16EEpromAddress;
	uint32_t	ui32EEpromData;
	uint8_t		ucEEpromDataBuffer[4];
	uint8_t		ucErrorCounter;

	// -------------- perform the EEPROM operation  ---------------------
	// decide if need to read or to write
	if( (EEpromDataList.ucReadWrite == EEPROM_WRITE) )
	{	// write into the EEPROM

		for(i=0, ucErrorCounter = 0; i<EEpromDataList.ucSizeOfTheList; i++)
		{
			ui16EEpromAddress 	= EEpromDataList.pEEDTList[i].ui16EEPROMAddress;
			ui32EEpromData		= EEpromDataList.pEEDTList[i].ui32EEPROMData;
			EEPROM32To8(ui32EEpromData, ucEEpromDataBuffer);
			// HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
			if(HAL_I2C_Mem_Write(&hi2c1, EEPROM_I2C_ADDRESS_ON_I2C1, ui16EEpromAddress, 2, ucEEpromDataBuffer,4, HAL_MAX_DELAY) != HAL_OK)
			{
				// no error checking is done yet
				if( (ucErrorCounter < 255) )	ucErrorCounter ++;
			}
			EEPROMWriteDelay();	// the 10ms write delay
		}
	}
	else
	{	// read from the EEPRPM
		for(i=0, ucErrorCounter = 0; i<EEpromDataList.ucSizeOfTheList; i++)
		{
			ui16EEpromAddress 	= EEpromDataList.pEEDTList[i].ui16EEPROMAddress;
			// HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
			if(HAL_I2C_Mem_Read(&hi2c1, EEPROM_I2C_ADDRESS_ON_I2C1, ui16EEpromAddress, 2,ucEEpromDataBuffer,4, HAL_MAX_DELAY) != HAL_OK)
			{
				// no error checking is done yet
				if( (ucErrorCounter < 255) )	ucErrorCounter ++;
			}
			EEPROM8To32(ucEEpromDataBuffer, &ui32EEpromData);
			EEpromDataList.pEEDTList[i].ui32EEPROMData = ui32EEpromData;

			EEPROMWriteDelay();	// write delay ... used just to prevent too fast EEPROM access-time
		}
	}

	EEpromDataList.ucListIsPopulated 	= LIST_IS_EMPTY;
	EEpromDataList.ucErrorCounter		= ucErrorCounter;
	// ------------------------------------------------------------------

	// --------- trigger the EEPORM operation done callback -------------
	EEDoneCallbackFunction(EEpromDataList);	// call the proper callback function


	//if( (EEpromDataList.ucCallbackToCall == APPLICATION_SECTION) )
	//{	// the EEPROM operation was performed on the behalf of the Application
	//
	//	return;
	//}
	//else if( (EEpromDataList.ucCallbackToCall == INTERACTION_SECTION) )
	//{	// the EEPROM operation was performed on the behalf of the Interaction section
	//
	//	return;
	//}

	//else if( (EEpromDataList.ucCallbackToCall == WHEEL_SECTION) )
	//{	// the EEPROM operation was performed on the behalf of the Wheel section
	//
	//	return;
	//}
	//else if( (EEpromDataList.ucCallbackToCall == AUDIO_SECTION) )
	//{	// the EEPROM operation was performed on the behalf of the Audio section
	//
	//	return;
	//}
	//else if( (EEpromDataList.ucCallbackToCall == QRE_SECTION) )
	//{	// the EEPROM operation was performed on the behalf of the QRE section
	//
	//	return;
	//}
	// .... fill up with mode if/else steps if they are needed ...
	// ------------------------------------------------------------------

	return;
}
//----------------------------------------------------------------------------------
static void EEPROM32To8(uint32_t ui32DataIn, uint8_t *ucDataOut)			// used to convert 32 bit uint data into 4 bytes of 8 bit uint data
{
	ucDataOut[0]		= (uint8_t)((ui32DataIn >> (0*8)) & 0x000000FF);	// shifted with 0  bits	LSD	= Less Significant Digit
	ucDataOut[1]		= (uint8_t)((ui32DataIn >> (1*8)) & 0x000000FF);	// shifted with 8  bits
	ucDataOut[2]		= (uint8_t)((ui32DataIn >> (2*8)) & 0x000000FF);	// shifted with 16 bits
	ucDataOut[3]		= (uint8_t)((ui32DataIn >> (3*8)) & 0x000000FF);	// shifted with 24 bits	MSD = MostSignificant Digit

	return;
}
//----------------------------------------------------------------------------------
static void EEPROM8To32(uint8_t *ucDataIn, volatile uint32_t *ui32DataOut)			// used to convert buffer of 4 bytes of 8 bit uint data into single 32 bit uint data
{
	uint32_t ui32EEData;

	// DIGIT3 = MSD
	ui32EEData = (uint32_t)ucDataIn[3];
	ui32EEData <<= (3*8);	// shift with 24 bits
	(*ui32DataOut) = ui32EEData;		// 1/4
	// DIGIT2
	ui32EEData = (uint32_t)ucDataIn[2];
	ui32EEData <<= (2*8);	// shift with 16 bits
	(*ui32DataOut) |= ui32EEData;	// 2/4
	// DIGIT1
	ui32EEData = (uint32_t)ucDataIn[1];
	ui32EEData <<= (1*8);	// shift with 8 bits
	(*ui32DataOut) |= ui32EEData;	// 3/4
	// DIGIT0 = LSD
	ui32EEData = (uint32_t)ucDataIn[0];
	ui32EEData <<= (0*8);	// shift with 0 bits
	(*ui32DataOut) |= ui32EEData;	// 4/4

	return;
}
//----------------------------------------------------------------------------------
//#define I2CEEPROM_WRITE_TIME_DELAY_MS		12		/* used to delay after each eeprom write operation */
//volatile uint8_t					m_ucEEpromAccessTimerCounterMS;		// used to be decremented @ 1 ms rate after each EEPROM Write operation
static void EEPROMWriteDelay(void)			// used to wait for the write time after an EEPROM write operation
{
	m_ucEEpromAccessTimerCounterMS = I2CEEPROM_WRITE_TIME_DELAY_MS;
	while(m_ucEEpromAccessTimerCounterMS);	//m_ucEEpromAccessTimerCounterMS is decremented in the 1KHZ timer of the EEPROM
}
//----------------------------------------------------------------------------------
static void EEPROMTimer1KHZISR()			// EEPROM Write Access Time delay timer
{	// called from the 1KHZ ISR
	if(m_ucEEpromAccessTimerCounterMS)	m_ucEEpromAccessTimerCounterMS --;
	return;
}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
// TSL25911
//----------------------------------------------------------------------------------
static void TSL25911Init(void)							// used @ initialization
{
	// Right side
	RightTSL25911.ucSensorSide 				= RIGHT;
	RightTSL25911.ucI2CAddress				= 0x52;
	RightTSL25911.ucOperationStatus			= TSL25911_UNKNOWN_STATE;
	RightTSL25911.m_ui16OperationTimeoutMS 	= 100; // 20 ms timeout

	// Left Side
	LeftTSL25911.ucSensorSide 				= LEFT;
	LeftTSL25911.ucI2CAddress				= 0x52;
	LeftTSL25911.ucOperationStatus			= TSL25911_UNKNOWN_STATE;
	LeftTSL25911.m_ui16OperationTimeoutMS 	= 100; // 20 ms timeout

	return;
}
//----------------------------------------------------------------------------------
static void TSL25911MainLoop(SIDE ucSensorSide)		// used in the main loop
{
	uint8_t 	ucCommand;			// used for sensor command
	uint8_t	ucRegisterAddress;	// used for sensor internal address

	// ------------------ handle the RIGHT side sensor -----------------------
	if(ucSensorSide == RIGHT)
	{	// RIGHT => I2C1
		// check the operation timeout
		if(RightTSL25911.m_ui16OperationTimeoutMS) return;

		if( (RightTSL25911.ucOperationStatus == TSL25911_UNKNOWN_STATE) )
		{	// after reset => must ENABLE the ALS
			// Enable Register Address = 0x00 + (Normal Command = 0xA0) => Register Address = 0xA0
			// Enable Register Data    = 0000 0011 = 0x03 (AEN + PON)
			//----------------------------------------

			ucCommand = 0x03;			// (AEN + PON)
			ucRegisterAddress = 0xA0;	// add

			// HAL_StatusTypeDef HAL_I2C_Mem_Write_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
			//if(HAL_I2C_Mem_Write_IT(&hi2c1, RightTSL25911.ucI2CAddress, ucRegisterAddress, 1, &ucCommand,1) != HAL_OK)
			//if(HAL_I2C_Mem_Write_IT(&hi2c1, 0x52, 0xA0, 1, (uint8_t *)&ucCommand,1) != HAL_OK)
			if(HAL_I2C_Mem_Write(&hi2c1, 0x52, 0xA0, 1,&ucCommand,1, HAL_MAX_DELAY) != HAL_OK)
			{
				RightTSL25911.ucOperationStatus 	= TSL25911_ERROR;
				RightTSL25911Result.ucSensorStatus 	= TSL25911_ERROR;
				#if defined(I2CTSL25911_DEBUG_MODE) && defined(I2CDEVICES_DEBUG_MODE)
				sprintf((char *)m_ucDebugBuffer,"(01) RIGHT TSL25911: TSL25911_UNKNOWN_STATE Error\r\n");
				DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
				#endif
				//Error_Handler2(ERROR_CODE_I2CDEVICES_DEBUG_040);
			}
			else
			{
				//RightTSL25911.ucOperationStatus = TSL25911_TRY_2_ENABLE;
				RightTSL25911.ucOperationStatus = TSL25911_ENABLED;
				RightTSL25911.m_ui16OperationTimeoutMS = 20;	// 20ms timeout
			}

			return;
		}
		else if( (RightTSL25911.ucOperationStatus == TSL25911_ENABLED) )
		{	// Sensor is enabled => must configure the GAIN -> MEDIUM (10)
			// Control Register Address = 0x01 + (Normal Command = 0xA0) => Register Address = 0xA1
			// Control Register Data    = 0001 0000 = 0x10 (AGAIN = MEDIUM)
			//----------------------------------------

			ucCommand = 0x10;			// (AGAIN = MEDIUM)
			ucRegisterAddress = 0xA1;	// add

			// HAL_StatusTypeDef HAL_I2C_Mem_Write_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
			//if(HAL_I2C_Mem_Write_IT(&hi2c1, RightTSL25911.ucI2CAddress, ucRegisterAddress, 1, &ucCommand,1) != HAL_OK)
			//if(HAL_I2C_Mem_Write_IT(&hi2c1, 0x52, 0xA1, 1, (uint8_t *)&ucCommand,1) != HAL_OK)
			if(HAL_I2C_Mem_Write(&hi2c1, 0x52, 0xA1, 1,&ucCommand,1, HAL_MAX_DELAY) != HAL_OK)
			{
				RightTSL25911.ucOperationStatus 	= TSL25911_ERROR;
				RightTSL25911Result.ucSensorStatus 	= TSL25911_ERROR;
				#if defined(I2CTSL25911_DEBUG_MODE) && defined(I2CDEVICES_DEBUG_MODE)
				sprintf((char *)m_ucDebugBuffer,"(02) RIGHT TSL25911: TSL25911_ENABLED Error\r\n");
				DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
				#endif
				//Error_Handler2(ERROR_CODE_I2CDEVICES_DEBUG_041);
			}
			else
			{
				//RightTSL25911.ucOperationStatus = TSL25911_TRY_2_CONFIG;
				RightTSL25911.ucOperationStatus = TSL25911_CONFIGURED;
				RightTSL25911.m_ui16OperationTimeoutMS = 20;	// 20ms timeout
			}

			return;

		}
		else if( (RightTSL25911.ucOperationStatus == TSL25911_CONFIGURED) )
		{	// Sensor configured => must try to read the CH0 and CH1 data
			// ALS Data Register Address = 0x14 + (Normal Command = 0xA0) => Register Address = 0xB4
			// ALS Data Register Data    = XX,YY -> read 4 data
			//----------------------------------------

			ucCommand = 0x01;			// (AGAIN = MEDIUM)
			ucRegisterAddress = 0xB4;	// add

			// HAL_StatusTypeDef HAL_I2C_Mem_Read_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
			//if(HAL_I2C_Mem_Read_IT(&hi2c1, RightTSL25911.ucI2CAddress, ucRegisterAddress, 1, (uint8_t *)RightTSL25911.ucResultBuffer ,4) != HAL_OK)
			if(HAL_I2C_Mem_Read_IT(&hi2c1, 0x52, 0xB4, 1, (uint8_t *)RightTSL25911.ucResultBuffer ,4) != HAL_OK)
			{
				RightTSL25911.ucOperationStatus 	= TSL25911_ERROR;
				RightTSL25911Result.ucSensorStatus 	= TSL25911_ERROR;
				#if defined(I2CTSL25911_DEBUG_MODE) && defined(I2CDEVICES_DEBUG_MODE)
				sprintf((char *)m_ucDebugBuffer,"(03) RIGHT TSL25911: TSL25911_CONFIGURED Error\r\n");
				DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
				#endif
				//Error_Handler2(ERROR_CODE_I2CDEVICES_DEBUG_042);
			}
			else RightTSL25911.ucOperationStatus = TSL25911_TRY_2_READ;
			RightTSL25911.m_ui16OperationTimeoutMS = 5;	// 5ms timeout
			return;

		}
		else if( (RightTSL25911.ucOperationStatus == TSL25911_CH_READ_DONE) )
		{	// Data Read is done => calculate the result from the reading buffer
			//----------------------------------------

			// CH0
			RightTSL25911Result.ui16CH0Result	= (uint16_t)RightTSL25911.ucResultBuffer[1];
			RightTSL25911Result.ui16CH0Result <<= 8;
			RightTSL25911Result.ui16CH0Result  += (uint16_t)RightTSL25911.ucResultBuffer[0];

			// CH1
			RightTSL25911Result.ui16CH1Result	= (uint16_t)RightTSL25911.ucResultBuffer[3];
			RightTSL25911Result.ui16CH1Result <<= 8;
			RightTSL25911Result.ui16CH1Result  += (uint16_t)RightTSL25911.ucResultBuffer[2];

			RightTSL25911Result.ucNewResultIsAvailable = 1;

			RightTSL25911.ucOperationStatus 	= TSL25911_WAIT_NEXT_READ;
			RightTSL25911Result.ucSensorStatus 	= TSL25911_CH_READ_DONE;	// indicating that the result reading has finished as expected
			RightTSL25911.m_ui16OperationTimeoutMS = 50;	// 50ms timeout (allow some time-slice also for the EEPROM 24C512)

		}
		else
		{	// error
			RightTSL25911.ucOperationStatus = TSL25911_UNKNOWN_STATE;
		}
		return;
	}
	// -----------------------------------------------------------------------


	// ------------------- handle the LEFT side sensor -----------------------
	if(ucSensorSide == LEFT)
	{	// LEFT => I2C3
		// check the operation timeout
		if(LeftTSL25911.m_ui16OperationTimeoutMS) return;

		if( (LeftTSL25911.ucOperationStatus == TSL25911_UNKNOWN_STATE) )
		{	// after reset => must ENABLE the ALS
			// Enable Register Address = 0x00 + (Normal Command = 0xA0) => Register Address = 0xA0
			// Enable Register Data    = 0000 0011 = 0x03 (AEN + PON)
			//----------------------------------------

			ucCommand = 0x03;			// (AEN + PON)
			ucRegisterAddress = 0xA0;	// add

			// HAL_StatusTypeDef HAL_I2C_Mem_Write_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
			//if(HAL_I2C_Mem_Write_IT(&hi2c3, LeftTSL25911.ucI2CAddress, ucRegisterAddress, 1, &ucCommand,1) != HAL_OK)
			if(HAL_I2C_Mem_Write(&hi2c3, 0x52, 0xA0, 1,&ucCommand,1, HAL_MAX_DELAY) != HAL_OK)
			{
				LeftTSL25911.ucOperationStatus 		= TSL25911_ERROR;
				LeftTSL25911Result.ucSensorStatus 	= TSL25911_ERROR;
				#if defined(I2CTSL25911_DEBUG_MODE) && defined(I2CDEVICES_DEBUG_MODE)
				sprintf((char *)m_ucDebugBuffer,"(01) LEFT TSL25911: TSL25911_UNKNOWN_STATE Error\r\n");
				DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
				#endif
				//Error_Handler2(ERROR_CODE_I2CDEVICES_DEBUG_040);
			}
			else
			{
				//LeftTSL25911.ucOperationStatus = TSL25911_TRY_2_ENABLE;
				LeftTSL25911.ucOperationStatus = TSL25911_ENABLED;
				LeftTSL25911.m_ui16OperationTimeoutMS = 20; // 20 ms timeout
			}

			return;
		}
		else if( (LeftTSL25911.ucOperationStatus == TSL25911_ENABLED) )
		{	// Sensor is enabled => must configure the GAIN -> MEDIUM (10)
			// Control Register Address = 0x01 + (Normal Command = 0xA0) => Register Address = 0xA1
			// Control Register Data    = 0001 0000 = 0x10 (AGAIN = MEDIUM)
			//----------------------------------------

			ucCommand = 0x10;			// (AGAIN = MEDIUM)
			ucRegisterAddress = 0xA1;	// add

			// HAL_StatusTypeDef HAL_I2C_Mem_Write_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
			//if(HAL_I2C_Mem_Write_IT(&hi2c3, LeftTSL25911.ucI2CAddress, ucRegisterAddress, 1, &ucCommand,1) != HAL_OK)
			if(HAL_I2C_Mem_Write(&hi2c3, 0x52, 0xA1, 1,&ucCommand,1, HAL_MAX_DELAY) != HAL_OK)
			{
				LeftTSL25911.ucOperationStatus 		= TSL25911_ERROR;
				LeftTSL25911Result.ucSensorStatus 	= TSL25911_ERROR;
				#if defined(I2CTSL25911_DEBUG_MODE) && defined(I2CDEVICES_DEBUG_MODE)
				sprintf((char *)m_ucDebugBuffer,"(02) LEFT TSL25911: TSL25911_ENABLED Error\r\n");
				DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
				#endif
				//Error_Handler2(ERROR_CODE_I2CDEVICES_DEBUG_041);
			}
			else
			{
				//LeftTSL25911.ucOperationStatus = TSL25911_TRY_2_CONFIG;
				LeftTSL25911.ucOperationStatus = TSL25911_CONFIGURED;
				LeftTSL25911.m_ui16OperationTimeoutMS = 20; // 20 ms timeout
			}

			return;

		}
		else if( (LeftTSL25911.ucOperationStatus == TSL25911_CONFIGURED) )
		{	// Sensor configured => must try to read the CH0 and CH1 data
			// ALS Data Register Address = 0x14 + (Normal Command = 0xA0) => Register Address = 0xB4
			// ALS Data Register Data    = XX,YY -> read 4 data
			//----------------------------------------

			ucCommand = 0x01;			// (AGAIN = MEDIUM)
			ucRegisterAddress = 0xB4;	// add

			// HAL_StatusTypeDef HAL_I2C_Mem_Read_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
			if(HAL_I2C_Mem_Read_IT(&hi2c3, LeftTSL25911.ucI2CAddress, ucRegisterAddress, 1, (uint8_t *)LeftTSL25911.ucResultBuffer ,4) != HAL_OK)
			{
				LeftTSL25911.ucOperationStatus 		= TSL25911_ERROR;
				LeftTSL25911Result.ucSensorStatus 	= TSL25911_ERROR;
				#if defined(I2CTSL25911_DEBUG_MODE) && defined(I2CDEVICES_DEBUG_MODE)
				sprintf((char *)m_ucDebugBuffer,"(03) LEFT TSL25911: TSL25911_CONFIGURED Error\r\n");
				DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
				#endif
				//Error_Handler2(ERROR_CODE_I2CDEVICES_DEBUG_042);
			}
			else LeftTSL25911.ucOperationStatus = TSL25911_TRY_2_READ;
			LeftTSL25911.m_ui16OperationTimeoutMS = 5; // 5 ms timeout
			return;

		}
		else if( (LeftTSL25911.ucOperationStatus == TSL25911_CH_READ_DONE) )
		{	// Data Read is done => calculate the result from the reading buffer
			//----------------------------------------

			// CH0
			LeftTSL25911Result.ui16CH0Result	= (uint16_t)LeftTSL25911.ucResultBuffer[1];
			LeftTSL25911Result.ui16CH0Result <<= 8;
			LeftTSL25911Result.ui16CH0Result  += (uint16_t)LeftTSL25911.ucResultBuffer[0];

			// CH1
			LeftTSL25911Result.ui16CH1Result	= (uint16_t)LeftTSL25911.ucResultBuffer[3];
			LeftTSL25911Result.ui16CH1Result <<= 8;
			LeftTSL25911Result.ui16CH1Result  += (uint16_t)LeftTSL25911.ucResultBuffer[2];

			LeftTSL25911Result.ucNewResultIsAvailable = 1;

			LeftTSL25911.ucOperationStatus 		= TSL25911_WAIT_NEXT_READ;
			LeftTSL25911Result.ucSensorStatus 	= TSL25911_CH_READ_DONE;	// indicating that the result reading has finished as expected

		}
		else
		{	// error
			LeftTSL25911.ucOperationStatus = TSL25911_UNKNOWN_STATE;
		}
		return;
	}
	// -----------------------------------------------------------------------



	return;
}
//----------------------------------------------------------------------------------
static void TSL25911Timer1KHZISR(void)				// called from the 1KHZ Timer ISR
{
	// RIGHT
	if(RightTSL25911.m_ui16OperationTimeoutMS )		RightTSL25911.m_ui16OperationTimeoutMS --;

	//LEFT
	if(LeftTSL25911.m_ui16OperationTimeoutMS )		LeftTSL25911.m_ui16OperationTimeoutMS --;

	return;
}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
static void I2CErrorChecker(void)	// used in the loop to check if there is some I2C error and trigger the callback
{
	uint8_t ucError;
	ucError = 0;

	// check the error for I2C1
	if( (hi2c1.ErrorCode != 0) || (hi2c1.Instance->SR1 != 0) || (hi2c1.Instance->SR2 != 0) )	ucError |= 0x01;

	// check the error for I2C3
	if( (hi2c3.ErrorCode != 0) || (hi2c3.Instance->SR1 != 0) || (hi2c3.Instance->SR2 != 0) )	ucError |= 0x02;

	if(I2CErrorStatus.ucError != ucError)
	{
		I2CErrorStatus.ucError = ucError;
		I2CErrorStatus.ucI2CErrorCallbackTimer = 100;	// decremented @ 100HZ => 1Second is 100 decrements
		I2CErrorCallback(ucError);
		// audio playing error sound = partiture 0
		AudioPlayPartiture(0);
		return;
	}

	if( (I2CErrorStatus.ucError != 0) )
	{
		if(I2CErrorStatus.ucI2CErrorCallbackTimer == 0)
		{
			I2CErrorStatus.ucI2CErrorCallbackTimer = 100;	// trigger another callback sending
			I2CErrorCallback(I2CErrorStatus.ucError);
			// audio playing error sound = partiture 0
			AudioPlayPartiture(0);
		}
	}

	return;
}
//----------------------------------------------------------------------------------
static void I2CErrorTimer100HZ(void)	// called @ 100HZ rate to handle error management	// called from the system timer @ 100HZ rate. it is used to timeout the reading of the LEFT VEML3328 color sensors
{
	if( (I2CErrorStatus.ucError == 0) ) return;
	if( (I2CErrorStatus.ucI2CErrorCallbackTimer > 0) )	I2CErrorStatus.ucI2CErrorCallbackTimer --;
	return;
}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== STATIC_END

