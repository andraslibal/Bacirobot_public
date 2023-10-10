/*
 * Interaction.c
 *
 *  Created on: Feb 16, 2022
 *      Author: Arthur
 */

#include "main.h"

#include "stdint.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"

// for EEPROM operation
#include "I2CDevices.h"
#include "I2CEEPROMAddresses.h"

//#include "WS2812.h"
//#include "Wheel.h"

#include "Interaction.h"

#ifdef INTERACTION_SHOW_ON_WS2812
#include "WS2812.h"
#endif


// my variables
#include "Wheel.h"

uint8_t m_ucDebugBuffer[100];
uint8_t rotation_threshold = 100;
uint8_t wheel_speed = 15;


// STATIC (INTERN) prototypes
//==================================================================================================================== STATIC_START
// variables - external
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim2;			// started here, used as 10KHZ interrupt, also used in IrTsop.c (not the timer itself but the interrupt call according to stm32f4xx_it.c)
extern TIM_HandleTypeDef htim5;			// 1uS/Increment , started in IrTsop.c, !! used as general purpose timer. just to go around up to max and start over. without interrupt: Interaction.c, IrTsop.c,
extern uint8_t m_ucRobotAddress;		// holds the address configured with the on board switches. it is defined in the Address.c
//----------------------------------------------------------------------------------


// variables - internal
#ifdef INTERACTION_DEBUG_MODE
uint8_t m_ucDebugBuffer[400];
uint8_t m_ucDebugTemp = 0;
#endif
#define INTERACTION_SKIP_REAL_TIME		300
uint16_t m_uiInteractionSkipRealTime;	// used in the main loop to skip real time
//----------------------------------------------------------------------------------

volatile AcquisitionDataStructure			MeasurementData = {0};			/* data structure used during the data collection process (here is the data stored during the acquisition)*/
volatile AcquisitionResultDataStructure		MeasurementResult = {0};		/* data structure storing the raw-results from the MeasurementData structure after a new acquisition (here is the data stored after is acquired in the previous filed and the acquisition is done)*/
volatile SpikeFilterDataStructure			SpikeFilterData = {0};			/* used to filter out from the "MeasurementResult" data spikes which may show up due to short noises */
volatile MovingAverageFilterData			MovingAveragedData = {0};		/* used to filter out the raw data from the "MeasurementResult" after this was cleared of spikes, after averaging the result will be put back into the "MeasurementResult" structure */
// here perhaps a filtering stage would be a good idea ... some running average filter
volatile BaseLineCalibrationDataStructure	BaseLineData = {0};				/* from the raw measurements it determines the base line for each channel and for both self reflection and signal from other robots */
volatile SensitivityDataStructure			SensitivityData = {0};			/* contains the sensitivity factors for each channels for both self reflection and signal from another robot, stored and retrieved @ init from EEPROM */
volatile InteractionResultDataStructure		InteractionResultData = {0};	/* contains the final Interaction measurement data */


uint32_t m_ui32ADC[6];		// global variable used to store the ADC->DMA values during an ADC measurement


// ID pattern generation & detection
volatile IDPatternGenerationDataStructure	IDPatternGenerationData = {0};	/* used to generate the ID pattern over the side IR LEDs */
volatile IDPatternDetectionDataStructure	IDPatternDetectionData[6]= {0};	/* used to detect the IR pattern from nearby other robots */
volatile IDPatternResultDataStructure		IDPatternResultData;			/* holds the detected ID's of the robots on all the 6 channels. if the ID == 0 => there is no robot detected on that channel */


// EEPROM 24C512 related
volatile uint8_t							m_ucEEPROMNewBaseLineDataIsAvailable = 0;			// used to indicate to the EEPROM if a new base line was measured and it needs to be saved into the EEPROM
volatile uint8_t							m_ucEEPROMNewSensitivityAvailable[2][6] = {0};		// used to indicate to the EEPROM write routine that a new sensitivity setting was made and it needs to be stored into the EEPROM
// -- m_ucEEPROMNewSensitivityAvailable[x][y]	-- x={0,1} and y={0,1,2,3,4,5}
// ---- x == 0	=> Self Reflection sensitivity
// ---- x == 1	=> sensitivity from another robot
// ---- y == {0,1,2,3,4,5}	=> indicating the channel where the new sensitivity was changed
static volatile EEPROMDataTypeDef			EEpromData[32];		/* data structure to hold EEPROM address and data pairs */
static volatile EEPROMDataListTypeDef		EEpromDataList;		/* the data list that is used to operate the EEPROM in the main loop mode */
//----------------------------------------------------------------------------------



// prototypes
//----------------------------------------------------------------------------------
static void UsedInThe10KHZISR(void);			// called in the 10KHZ timer2 ISR
//----------------------------------------------------------------------------------
static void GenerateIRPattern(void);			// called in the 10KHZ timer2 ISR to generate new IR pattern
//----------------------------------------------------------------------------------
static void TriggerDataAcquisition(void);		// called in the 10HZ timer2 ISR to trigger another data acquisition
//----------------------------------------------------------------------------------
static void UsedInTheDMAISR(void);				// called from the ADC->DMA ISR to handle data collection after ADC is done
//----------------------------------------------------------------------------------
static void UsedInTheMainLoop(void);			// called from the main loop to handle the collected data
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
static void SpikeFilterInitialization(void);	// called to initialize the "SpikeFilterData" data structure.
//----------------------------------------------------------------------------------
#ifdef INTERACTION_USES_SPIKE_FILTER
static void SpikeFilterMain(void);				// called from the main loop to filter the spikes from the "MeasurementResult" structure.
//----------------------------------------------------------------------------------
static uint8_t SpikeSearchOnTheNewDataFromAnotherRobot(uint8_t ucChannel);		// compare a new data from another robot on channel i against the history buffer. return 0 if the data is no spike, returns 1if the data is found to be a spike
//----------------------------------------------------------------------------------
static void SpikeResetAFilterChannel(uint8_t ucChannel);						// resets a filter channel defined by ucChannel. => emptys the history buffer and the control variables
#endif
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
static void MovingAverageFilterInitialization(void);		// called in the initialization section of the code
//----------------------------------------------------------------------------------
#ifdef INTARACTION_USES_MOVING_AVERAGING_FILTER
static void MovingAverageFilterMain(void);					// called in the main loop section to filter out the raw data before base line calibration or further data processing
#endif
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
static void BaseLineInitialization(void);		// used to initialize the base line structure and calibration algorithm
//----------------------------------------------------------------------------------
static void BaselineCalibrationMain(void);		// called from the "UsedInTheMainLoop()" during baseline calibration
//----------------------------------------------------------------------------------
static void BaseLineTimer1KHZ(void);			// called from the 1KHZ timer to handle real time requirements in the base line calibration logic
//----------------------------------------------------------------------------------
static void BaseLineDataReset(void);			// called to reset the base line data structure "BaseLineData"
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
static void SensitivityInitialization(void);	// retrieve the sensitivity information from the EEPROM or from ROM
//----------------------------------------------------------------------------------
static void InteractionResultCalculationMain(void);		// used to calculate the final result of the interaction
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
// EEPROM
static void EEPROMInitialization(void);					// performs basic initialization settings with the EEPROM
//----------------------------------------------------------------------------------
static void EEPROMMainLoop(void);						// called from the main loop to handle EERPOM activities (mainly storing new-baseline values or new-sensitivity values)
//----------------------------------------------------------------------------------
static void EEPROMDoneCallback(EEPROMDataListTypeDef EEResultList);		// callback function to be called @ EEPORM operation completed
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
#ifdef INTERACTION_SHOW_ON_WS2812
uint8_t  m_ucWS2812UpdateSkipCounter;					// used to skip updates
#define INTERACTION_WS2812_UPDATE_SKIP_COUNTER_MAX		5		/* update every this time the WS2812 LEDs */
static void ShowInteractionOnWS2812(void);				// used to display the interaction result on the WS2812 LEDs
#endif
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
// ID Pattern
//----------------------------------------------------------------------------------
static void IDPatternInitialization(void);				// called in the initialization section
//----------------------------------------------------------------------------------
static void IDPatternMainLoop(void);					// called in the main loop
//----------------------------------------------------------------------------------
static void IDPatternAddressUpdate(void);				// called to update the Pattern Generation Buffer "ucPatternBuffer" whenever the Robot's address has changed
//----------------------------------------------------------------------------------
static void IDPatternTimer1KHZISR(void);				// called from the 1KHZ timer ISR
//----------------------------------------------------------------------------------
static void IDPatternChannelISR(uint8_t ucChannel);		// used to be called from the channel line change interrupt
//----------------------------------------------------------------------------------
static void IDPatternResetChannel(uint8_t ucChannel);			// clear the relevant parameters from a channel and prepare the channel for the next reception
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------

// debug
#ifdef INTERACTION_DEBUG_MODE
//----------------------------------------------------------------------------------
volatile uint8_t 	m_ucEnablePrintRaw 							= 0;	// if is set to 1 the raw values are printed, use serial commands to enable / disable raw printing 'r'=enable raw printing
static void DebugPrintRaw(void);			// used to print the raw values after acquisition
//----------------------------------------------------------------------------------
volatile uint8_t	m_ucPrintSpikeRemovedData					= 0;	// if is set to 1 the spike filtered data is printed over the serial debug line. to activate this press 't' key
#ifdef INTERACTION_USES_SPIKE_FILTER
static void DebugPrintSpikeFilteredData(void);	// used to print the spike filtered data
#endif
//----------------------------------------------------------------------------------
volatile uint8_t	m_ucPrintMovingAverageData					= 0;	// if this is set to 1 =>the moving averaged data is printed. to activate this print press the 'y' key
#ifdef INTARACTION_USES_MOVING_AVERAGING_FILTER
static void DebugMovingAveragData(void);		// used to print the moving average data
#endif
//----------------------------------------------------------------------------------
volatile uint8_t	m_ucPrintBaseLineCalibrationData			= 0;	// if set to 1 it will print the base line calibration once and it will be reseted to 0, use 'u' to print the base line calibration
static void DebugPrintBaseLine(void);		// used to print the base line calibration data
//----------------------------------------------------------------------------------
volatile uint8_t 	m_ucPrintCorrectedData						= 0;	// if set => prints continuously the corrected data, use 'i' to turn on this debug message
static void DebugPrintCorrectedData(void);		// used to print the corrected measurement data
//----------------------------------------------------------------------------------
volatile uint8_t	m_ucPrintFinalData							= 0;	// if set => prints continuously the uin32 final data. use 'o' to turn on this debug message
static void DebugPrintFinal32Data(void);		// used to print the final uint32 measurement result
//----------------------------------------------------------------------------------
volatile uint8_t	m_ucPrintFinalTruncatedData					= 0;	// if set => prints the truncated results. in range of 0 ... 255. use 'p' to turn on this message
static void DebugPrintFinalTruncatedData(void);	// used to print the final truncated data
//----------------------------------------------------------------------------------
volatile uint8_t	m_ucPrintSensitivityData					= 0;	// if set => print once the sensitivity data, use 'l' to print once all the data
volatile uint8_t	m_ucSensitivityDataChannel					= 0;	// indexes the Self or From Another Robot channel to be increased or decreased. use 'k' to change ={0,1,2,3,4,5}
volatile uint8_t	m_ucSensitivitySelectSelfOrAnotherRobot		= 0;	// selects the Sensitivity to be altered. ==0 => Self, ==1 => from another robot. use 'j' to change ={0,1}
// to increment or decrement the current value of the sensitivity use 'm' to increment or 'n' to decrement the current value
static void DebugPrintSensitivityData(void);			// used to print the sensitivity data
//----------------------------------------------------------------------------------
#ifdef false
static void ChangeSensitivityData(uint8_t ucUpDown);	// used to increment or to decrement the sensitivity data
#endif
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
static void StopAllDebugMessages(void);			// used to stop all the currently printing serial debug messages
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
#endif
//==================================================================================================================== STATIC_END


// API
//==================================================================================================================== API_START
// new result available callback
__attribute__ ((weak)) void InteractionNewResultAvailableCallback(InteractionResultDataStructure NewInteractionResult, IDPatternResultDataStructure NewIDResult)		// implement this function in the application section if the interaction information is needed
{	// this function is called every time a new interaction measurement is done an a new result is available
	// all the relevant data is available in the "NewInteractionResult" parameter

	// ----- most relevant data from the interaction ------
	// from auto-corelation
	//NewInteractionResult.ui32FinalDataFromSelfReflection[0 ... 5]		= the signal strength reflected from a foreign object on channel [0 ... 5]. range is from 0 ... max 4095
	//NewInteractionResult.ucFinalDataFromSelfReflection[0 ... 5]		= the same as above but the data is in the range of 0 ... 255 (truncated @ 255)
	// from another robot
	//NewInteractionResult.ui32FinalDataFromOtherRobot[0 ... 5]			= the signal received from another robot.
	//																			- if <5 	=> no other robot is detected
	//																			- if == 5 	=> a robot is detected but it is far away
	//																			- if > 5 	=> the signal is proportional with the closeness of the other robot. range is up to 4095
	//NewInteractionResult.ucFinalDataFromOtherRobot[0 ... 5]			= the same as the one from above but limited to 255 as maximum received closeness parameter.
	//NewInteractionResult.ui32AcquisitionDurationUS					= for statistics = the time it took to take 1 interaction measurement
	//NewInteractionResult.ui32AcquisitionRoundCounter					= the number of the measurement. after each new measurement this counter is incremented
	// ----------------------------------------------------

	// ----- most relevant data from the ID Detection -----
	//NewIDResult.ucDetectedRobotID[0 ... 5]		=> if == 0 => there is no robot detected on that channel. if != 0 => it represents the ID of the robot detected on that channel
	// ----------------------------------------------------

	// EX: interaction display
	//#define INTERACTION_SHOW_CALLBACK
	#if defined(INTERACTION_DEBUG_MODE) && defined(INTERACTION_SHOW_CALLBACK)
	sprintf((char * )m_ucDebugBuffer,"CLBK: S0=%lu\t S1=%lu\t S2=%lu\t S3=%lu\t S4=%lu\t S5=%lu\t\t "
													"O0=%lu\t O1=%lu\t O2=%lu\t O3=%lu\t O4=%lu\t O5=%lu\t TM=%lu\t RND=%lu\r\n",NewInteractionResult.ui32FinalDataFromSelfReflection[0],
																												NewInteractionResult.ui32FinalDataFromSelfReflection[1],
																												NewInteractionResult.ui32FinalDataFromSelfReflection[2],
																												NewInteractionResult.ui32FinalDataFromSelfReflection[3],
																												NewInteractionResult.ui32FinalDataFromSelfReflection[4],
																												NewInteractionResult.ui32FinalDataFromSelfReflection[5],
																													NewInteractionResult.ui32FinalDataFromOtherRobot[0],
																													NewInteractionResult.ui32FinalDataFromOtherRobot[1],
																													NewInteractionResult.ui32FinalDataFromOtherRobot[2],
																													NewInteractionResult.ui32FinalDataFromOtherRobot[3],
																													NewInteractionResult.ui32FinalDataFromOtherRobot[4],
																													NewInteractionResult.ui32FinalDataFromOtherRobot[5],
																													NewInteractionResult.ui32AcquisitionDurationUS,
																													NewInteractionResult.ui32AcquisitionRoundCounter);

	DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
	#endif


	// EX: interaction display
	//#define INTERACTION_SHOW_DETECTED_ID
	#if defined(INTERACTION_DEBUG_MODE) && defined(INTERACTION_SHOW_DETECTED_ID)
	sprintf((char * )m_ucDebugBuffer,"CLBK: CH0=%u\t CH1=%u\t CH2=%u\t CH3=%u\t CH4=%u\t CH5=%u\r\n", NewIDResult.ucDetectedRobotID[0],
																											NewIDResult.ucDetectedRobotID[1],
																											NewIDResult.ucDetectedRobotID[2],
																											NewIDResult.ucDetectedRobotID[3],
																											NewIDResult.ucDetectedRobotID[4],
																											NewIDResult.ucDetectedRobotID[5]);
	DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
	#endif

	return;
}
//----------------------------------------------------------------------------------
 void InteractionTurnIRPatternGenerationONorOFF(uint8_t ucIRPatternON)		//used to turn the IR pattern generation ON or OFF. if ucIRPatternON == 0 => the IR pattern generation is turned off and all the side IR LEDs are OFF
{
	if(ucIRPatternON)
	{	// turn ON
		HAL_GPIO_WritePin(IR_PWM_GPIO_Port, IR_PWM_Pin, GPIO_PIN_RESET);	// the IR driver  is ENABLED
	}
	else
	{	// turn OFF
		HAL_GPIO_WritePin(IR_PWM_GPIO_Port, IR_PWM_Pin, GPIO_PIN_SET);		// the IR driver  is DISABLED
	}

	return;
}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
 // Base line
 // before calling this function make sure that there are no other robots around and there are no objects around to get reflection from
void InteractionCalculateBaseline(void)			// calling this function starts the base line calibration.
{
	BaseLineDataReset();
	return;
}
//----------------------------------------------------------------------------------
BaseLineCalibrationDataStructure InteractionGetBaseLineData(void)		// calling this function will return the base line situation
{
	//BaseLineData.ucBaseLineCalibrationDone == 1 					=> the base line calibration is ok (data is available)
	//BaseLineData.ui32CalculatedSelfBaseLineValues[0 ... 5]		=> the calculated base line values for self reflection for each channels 0 ... 5
	//BaseLineData.ui32OtherRobotMaxNoise[0 ... 5]					=> the calculated base line values for signal from other robots, for each channel 0 ... 5
	return BaseLineData;	// the entire base line information
}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
// Sensitivity
uint8_t InteractionSetSensitivity(SENSITIVITY_TYPE ucSelfOrOther, uint8_t ucChannel, uint8_t ucValue)		// used to set the sensitivity of a channel from a given type (self or from another robot)
{
	// used to set the sensitivity for each channel and for both the self reflection and the signal from another robot
	// the input parameters are:
	// --- ucSelfOrOther	-> can be {SENSITIVITY_FROM_SELF_REFLECTION, SENSITIVITY_FROM_ANOTHER_ROBOT}
	// --- ucChannel		-> can be {0=CH0, 1, 2, 3, 4, 5=CH5}
	// --- ucValue			-> can be between 0 and 100
	// ----- 50				-> means the default sensitivity (GAIN=1)
	// ----- 1 ... 49		-> decreased sensitivity (GAIN<1)
	// ----- 0				-> OFF (GAIN=0)
	// ----- above 50		-> increased sensitivity (GAIN>1)
	// the function returns 0 if no error is detected or 0 if some error is found
	//---------------------------------------

	// -------- input parameters checking ----------
	// type of sensitivity
	if( (ucSelfOrOther != SENSITIVITY_FROM_SELF_REFLECTION) && (ucSelfOrOther != SENSITIVITY_FROM_ANOTHER_ROBOT) )	return 1;	// unknown sensitivity category error code
	// channel number range
	if( (ucChannel < 0) || (ucChannel > 5) )	return 2;	// wrong channel number
	// value to set
	if( (ucValue < 0) || (ucValue > 100) )		return 3;	// value is out of range
	// ---------------------------------------------


	// ------ set the sensitivity in the RAM -------
	if( ucSelfOrOther == SENSITIVITY_FROM_SELF_REFLECTION )
	{	// from auto-corelation
		if( (SensitivityData.ucSensitivityForSelfReflection[ucChannel] != ucValue) )
		{	// new value => set and store into the EEPROM
			SensitivityData.ucSensitivityForSelfReflection[ucChannel] 	= ucValue;
			// EEPROM
			m_ucEEPROMNewSensitivityAvailable[0][ucChannel]		= 1;	// set the appropriate flag for the EEPORM engine
		}
	}
	else
	{
		// from another robot
		if( (SensitivityData.ucSensitivityForOtherRobot[ucChannel] != ucValue) )
		{	// new value => set and store into the EEPROM
			SensitivityData.ucSensitivityForOtherRobot[ucChannel] 	= ucValue;
			// EEPROM
			m_ucEEPROMNewSensitivityAvailable[1][ucChannel]		= 1;	// set the appropriate flag for the EEPORM engine
		}
	}
	// ---------------------------------------------

	return 0;	// no error, all ok
}
//----------------------------------------------------------------------------------
uint8_t	InteractionIncDecSensitivity(SENSITIVITY_TYPE ucSelfOrOther, uint8_t ucChannel, INC_DEC_SENSITIVITY_TYPE ucIncDec)	// used to increment or decrement the sensitivity, with one unit, on a particular channel
{
	// used to set the sensitivity for each channel and for both the self reflection and the signal from another robot
	// the input parameters are:
	// --- ucSelfOrOther	-> can be {SENSITIVITY_FROM_SELF_REFLECTION, SENSITIVITY_FROM_ANOTHER_ROBOT}
	// --- ucChannel		-> can be {0=CH0, 1, 2, 3, 4, 5=CH5}
	// --- ucIncDec			-> can be {SENSITIVITY_INCREMENT, SENSITIVITY_DECREMENT}
	// returns an error code if some parameter is not proper or 0 if no error
	//---------------------------------------

	// -------- input parameters checking ----------
	// type of sensitivity
	if( (ucSelfOrOther != SENSITIVITY_FROM_SELF_REFLECTION) && (ucSelfOrOther != SENSITIVITY_FROM_ANOTHER_ROBOT) )	return 1;	// unknown sensitivity category error code
	// channel number range
	if( (ucChannel < 0) || (ucChannel > 5) )	return 2;	// wrong channel number
	// value to set
	if( (ucIncDec != SENSITIVITY_INCREMENT) && (ucIncDec != SENSITIVITY_DECREMENT) )		return 3;	// value is out of range
	// ---------------------------------------------

	// ------------ range checking -----------------
	if( (ucIncDec == SENSITIVITY_INCREMENT) )
	{	// increment if possible
		if( (ucSelfOrOther == SENSITIVITY_FROM_SELF_REFLECTION) )
		{	// auto-correlation
			if( (SensitivityData.ucSensitivityForSelfReflection[ucChannel] < 100) )
			{
				InteractionSetSensitivity(ucSelfOrOther,ucChannel, (SensitivityData.ucSensitivityForSelfReflection[ucChannel] + 1));
				return 0;	// no error
			}
			else
			{	// value is already at the edge => cannot be further increased
				return 4;	// the limit was already reached
			}
		}
		else
		{	// from another robot
			if( (SensitivityData.ucSensitivityForOtherRobot[ucChannel] < 100) )
			{
				InteractionSetSensitivity(ucSelfOrOther,ucChannel, (SensitivityData.ucSensitivityForOtherRobot[ucChannel] + 1));
				return 0;	// no error
			}
			else
			{	// value is already at the edge => cannot be further increased
				return 5;	// the limit was already reached
			}
		}

	}
	else
	{	// decrement if possible
		if( (ucSelfOrOther == SENSITIVITY_FROM_SELF_REFLECTION) )
		{	// auto-correlation
			if( (SensitivityData.ucSensitivityForSelfReflection[ucChannel] > 0) )
			{
				InteractionSetSensitivity(ucSelfOrOther,ucChannel, (SensitivityData.ucSensitivityForSelfReflection[ucChannel] - 1));
				return 0;	// no error
			}
			else
			{	// value is already at the edge => cannot be further increased
				return 6;	// the limit was already reached
			}
		}
		else
		{	// from another robot
			if( (SensitivityData.ucSensitivityForOtherRobot[ucChannel] > 0) )
			{
				InteractionSetSensitivity(ucSelfOrOther,ucChannel, (SensitivityData.ucSensitivityForOtherRobot[ucChannel] - 1));
				return 0;	// no error
			}
			else
			{	// value is already at the edge => cannot be further increased
				return 7;	// the limit was already reached
			}
		}
	}
	// ---------------------------------------------

	return 0;	// no error
}
//----------------------------------------------------------------------------------
SensitivityDataStructure InteractionGetSensitivityData(void)			// used to retrieve the current sensitivity setting
{
	// used to retrieve the sensitivity settings used to calculate the final interaction values. these values are stored in the EEPROM
	// most relevant parameters
	// each value must be in the range 0 ... 100.
	// ---- 50 			=>means no sensitivity adjustment (sensitivity=1)
	// ---- 1 ... 49 	=> lower sensitivity (below 1) (the value of 1 is the lowest, and the value of 0 is equivalent with OFF)
	// ---- above 50 	=> increased sensitivity (larger than 1)
	//SensitivityData.ucSensitivityForSelfReflection[0 ... 5]		=> represents the sensitivity for self reflected data, for all the 6 channels
	//SensitivityData.ucSensitivityForOtherRobot[0 ... 5]			=> represents the sensitivity for the signal received from another robot, for all the 6 channels
	//---------------------------------------

	return SensitivityData;
}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== API_END


// SYSTEM
//==================================================================================================================== SYSTEM_START
//----------------------------------------------------------------------------------
void InteractionInit(void)	// used to initialize the Interaction Module. Called from main.c file @ init section
{

	// EEPROM
	EEPROMInitialization();

	// ID Pattern
	IDPatternInitialization();

	// turn on the side IR LED drivers
	InteractionTurnIRPatternGenerationONorOFF(1);
	// set the number of measurements / acquisition
	MeasurementData.ucAcquisitionCounterMax = INTERACTION_NUMBER_OF_MEASUREMENTS_PER_ACQUISITIONS;

	// spike filter initialization
	SpikeFilterInitialization();			// not used for now

	// moving average filter
	MovingAverageFilterInitialization();	// not used for now

	// base line determination
	BaseLineInitialization();

	// sensitivity
	SensitivityInitialization();


	//HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim)
	if(HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
	{
		Error_Handler2(ERROR_CODE_INTERACTION_INIT_001);
	}





	return;
}
//----------------------------------------------------------------------------------
void InteractionMainLoop(void)	// loop function of the Interaction Module. Called from the main.c file @ main loop section
{
	// skip real time
	if(m_uiInteractionSkipRealTime)
	{
		m_uiInteractionSkipRealTime --;
		return;
	}
	m_uiInteractionSkipRealTime = INTERACTION_SKIP_REAL_TIME;
	//--------------------------------------------------------

	// --------------- check if acquisition is done --------------------------
	// this function checks if the acquisition is done in the "MeasurementData" variable
	// if the acquisition is done => will save the result in the "MeasurementResult"
	// and restarts another data acquisition
	// if the acquisition is done and new results are available the "MeasurementResult.ucNewDataAvailable" will be set
	UsedInTheMainLoop();	// save data if available and restart acquisition
	if( (MeasurementResult.ucNewDataAvailable) )	// this is a new RAW data without filtering of any kind. if filtering needs to be applied this here must change the new result availability after filtering and not after raw measurement
	{
		// -------------- raw printing ----------------
		#ifdef INTERACTION_DEBUG_MODE
		if( (m_ucEnablePrintRaw) )			// just press the 'r' key to activate
		{	// print the raw un-filtered data
			DebugPrintRaw();
		}
		#endif
		// --------------------------------------------
	}
	//------------------------------------------------------------------------

	#ifdef INTERACTION_USES_SPIKE_FILTER
	// ------------------- spikes remover filter -----------------------------
	// check if new results are available
	// by commenting out this filed the code below will remain the same but without the spike remove filter contribution
	if( (MeasurementResult.ucNewDataAvailable) )	// this is a new RAW data without filtering of any kind. if filtering needs to be applied this here must change the new result availability after filtering and not after raw measurement
	{
		SpikeFilterMain();	// if the new data is not a spike it will still be available in the ...
		// in case a spike is detected the "MeasurementResult.ucNewDataAvailable = 0;" will be set to 0 and it is like no new data is available
		#ifdef INTERACTION_DEBUG_MODE
		if( (MeasurementResult.ucNewDataAvailable) )
		{	// here the data is already spike free
			if( (m_ucPrintSpikeRemovedData) )	// just press the 't' key to activate
			{	// print the spike free data
				DebugPrintSpikeFilteredData();
			}
		}
		#endif
	}
	//------------------------------------------------------------------------
	#endif

	#ifdef INTARACTION_USES_MOVING_AVERAGING_FILTER
	// ------------------- moving average filter -----------------------------
	// it will average the last n measurements and the result is saved back into the same variables:
	// --- MeasurementResult.ui32SignalFromSelfReflection[]
	// --- MeasurementResult.ui32SignalFromAnotherRobot[]
	// by commenting out this section the code will still be the same but without averaging filter
	if( (MeasurementResult.ucNewDataAvailable) )
	{
		MovingAverageFilterMain();
		#ifdef INTERACTION_DEBUG_MODE
		if( (m_ucPrintMovingAverageData) )		// just press the 'y' key to activate
		{
			DebugMovingAveragData();
		}
		#endif
	}
	//------------------------------------------------------------------------
	#endif

	// ------------- do something with the measured data ---------------------
	// this can be implemented in another function
	// check if new results are available
	if( (MeasurementResult.ucNewDataAvailable) )	// here the data (if available) is already spike free
	{
		MeasurementResult.ucNewDataAvailable = 0;


		// - check if exist the base line calibration -
		// check if the base line is already measured or it needs to be measured
		// if needed measure the base line for both REFLECTION and FROM OTHER ROBOTS
		// uses the data structures:
		// - BaseLineData
		BaselineCalibrationMain();
		#ifdef INTERACTION_DEBUG_MODE
		if( (m_ucPrintBaseLineCalibrationData) )		// just press the 'u' key to print once
		{
			// only print once the base line calibration
			m_ucPrintBaseLineCalibrationData = 0;
			DebugPrintBaseLine();
		}
		#endif
		// --------------------------------------------


		// ------- calculate the final Result ---------
		// calculate final value with the subtracted base line
		// calculate the sensitivity coefficient for each channel and for both the SELF REFLECTION and the SIGNAL FROM OTHER ROBOT
		// uses the data structures:
		// - SensitivityData
		// - InteractionResultData
		if( (BaseLineData.ucBaseLineCalibrationDone) )
		{	// calculate new result only if there is a determined baseline
			InteractionResultCalculationMain();
			#ifdef INTERACTION_DEBUG_MODE
			if( (InteractionResultData.ucNewInteractionResultAvailable) )
			{
				// Corrected Data
				if( (m_ucPrintCorrectedData) )			DebugPrintCorrectedData();		// just press the 'i' key to activate

				// Final data uint32
				if( (m_ucPrintFinalData) )				DebugPrintFinal32Data();		// just press the 'o' key to activate

				// Final Truncated Data
				if( (m_ucPrintFinalTruncatedData) )		DebugPrintFinalTruncatedData();	// just press the 'p' key to activate
			}
			#endif

			// ------- display the result on WS2812 -------
			#ifdef INTERACTION_SHOW_ON_WS2812
			ShowInteractionOnWS2812();
			#endif
			// --------------------------------------------
		}
		// --------------------------------------------


		// ------ debug print sensitivity -------------
		#ifdef INTERACTION_DEBUG_MODE
		if( m_ucPrintSensitivityData )
		{
			DebugPrintSensitivityData();
			m_ucPrintSensitivityData = 0;	// only print once
		}
		#endif
		// --------------------------------------------

		// --------------------------------------------
		// --------------------------------------------


	}
	// -----------------------------------------------------------------------


	// ------------------ handle EEPROM activities ---------------------------
	// called from the main loop to handle EERPOM activities (mainly storing new-baseline values or new-sensitivity values)
	EEPROMMainLoop();
	// -----------------------------------------------------------------------


	// -------------------- handle the ID Pattern ----------------------------
	// updates the ID buffer if the Address of the robot has changed
	// handles all it is needed @ ID decoding
	IDPatternMainLoop();
	// -----------------------------------------------------------------------





	return;
}
//----------------------------------------------------------------------------------
void InteractionTimer1KHZISR(void)	// 1KHZ timer function called from System ISR from stm32f4xx_it.c file @ SysTick Function (!!! keep as short as possible !!!)
{
	// base line calibration timer
	BaseLineTimer1KHZ();

	// ID timeout timer
	IDPatternTimer1KHZISR();


	return;
}
//----------------------------------------------------------------------------------
void InteractionTimer10KHZISR(void)	// 10KHZ timer function called from Timer2 ISR from stm32f4xx_it.c file (!!! keep as short as possible !!!)
{

	UsedInThe10KHZISR();

	return;
}
//----------------------------------------------------------------------------------
void InteractionADC1DMACallback()	// used to be called from the stm32f4xx_it.c once the DMA1 reads all the ADC values related to the 6 photodetector sensors
{
	UsedInTheDMAISR();

	return;
}
//----------------------------------------------------------------------------------
void InteractionCH0Callback()	// used to be called from the stm32f4xx_it.c once the signal is strong enough to trigger an interrupt
{
	IDPatternChannelISR(0);
	return;
}
//----------------------------------------------------------------------------------
void InteractionCH1Callback()	// used to be called from the stm32f4xx_it.c once the signal is strong enough to trigger an interrupt
{
	IDPatternChannelISR(1);
	return;
}
//----------------------------------------------------------------------------------
void InteractionCH2Callback()	// used to be called from the stm32f4xx_it.c once the signal is strong enough to trigger an interrupt
{
	IDPatternChannelISR(2);
	return;
}
//----------------------------------------------------------------------------------
void InteractionCH3Callback()	// used to be called from the stm32f4xx_it.c once the signal is strong enough to trigger an interrupt
{
	IDPatternChannelISR(3);
	return;
}
//----------------------------------------------------------------------------------
void InteractionCH4Callback()	// used to be called from the stm32f4xx_it.c once the signal is strong enough to trigger an interrupt
{
	IDPatternChannelISR(4);
	return;
}
//----------------------------------------------------------------------------------
void InteractionCH5Callback()	// used to be called from the stm32f4xx_it.c once the signal is strong enough to trigger an interrupt
{
	IDPatternChannelISR(5);
	return;
}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
#ifdef INTERACTION_DEBUG_MODE
void InteractionDebugRX(uint8_t *ucRXBuffer, uint8_t ucRXBufferLength)	// used to receive RX data from the debug interface
{
	return;
}
void InteractionDebugRXChar(uint8_t ucRXChar)	// used to receive RX single char from the debug interface
{
	// ---------- trigger ADC->DMA sampling ----------
	if( ucRXChar == 'q' )
	{
		// HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef* hadc, uint32_t* pData, uint32_t Length)
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);	//  debug on PC13 HIGH
		//m_ucDMADone = 0;
		//if( HAL_ADC_Start_DMA(&hadc1,m_ui32ADC,6) != HAL_OK )
		//{
		//	Error_Handler2(ERROR_CODE_INTERACTION_DEBUG_001);
		//}
		return;
	}
	// -----------------------------------------------

	// -------------- toggle debug pin ---------------
	if( ucRXChar == 'a' )
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);	//  debug on PC13 HIGH
		return;
	}
	if( ucRXChar == 'z' )
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);	//  debug on PC13 LOW
		return;
	}
	// -----------------------------------------------

	// ----- channel setting for debug display -------
	if( ucRXChar == 'w' )
	{
		if(m_ucDebugTemp > 0)	m_ucDebugTemp --;
		return;
	}
	if( ucRXChar == 's' )
	{
		if(m_ucDebugTemp < 5)	m_ucDebugTemp ++;
		return;
	}
	// -----------------------------------------------

	// ------- IR pattern generation ON / OFF --------
	if( ucRXChar == 'e' )
	{
		InteractionTurnIRPatternGenerationONorOFF(1);	// enable (by default is enabled)
		return;

	}
	if( ucRXChar == 'd' )
	{
		InteractionTurnIRPatternGenerationONorOFF(0);	// disable
		return;
	}
	// -----------------------------------------------

	// debug messages section
	// ------------- raw printing --------------------
	if( ucRXChar == 'r' )
	{	// RAW measured data printing. select channel using 'w' and 's' characters
		// disable all the rest of the debug-printings first
		StopAllDebugMessages();
		m_ucEnablePrintRaw = 1;	// enabled
		return;
	}
	// -----------------------------------------------

	// ------- Spike filtered data printing ----------
	if( ucRXChar == 't' )
	{	// spike removed data printing
		// disable all the rest of the debug-printings first
		StopAllDebugMessages();
		m_ucPrintSpikeRemovedData = 1;	// enabled
		return;
	}
	// -----------------------------------------------

	// ------- Moving Average data printing ----------
	if( ucRXChar == 'y' )
	{	// spike removed data printing
		// disable all the rest of the debug-printings first
		StopAllDebugMessages();
		m_ucPrintMovingAverageData = 1;	// enabled
		return;
	}
	// -----------------------------------------------

	// --------- Base Line Calibration ---------------
	if( ucRXChar == 'u' )
	{	// display the base line calibration
		// disable all the rest of the debug-printings first
		StopAllDebugMessages();
		m_ucPrintBaseLineCalibrationData = 1;	// enabled
		return;
	}
	if( ucRXChar == 'U' )
	{	// trigger a base line calibration
		BaseLineDataReset();
		m_ucEnablePrintRaw = 0;	// disable
		m_ucPrintBaseLineCalibrationData = 1;	// enabled
		return;
	}
	// -----------------------------------------------

	// -------- Corrected Data printing --------------
	if( ucRXChar == 'i' )
	{	// display the base line corrected interaction result for all channels
		StopAllDebugMessages();
		m_ucPrintCorrectedData = 1;	// enabled
		return;
	}
	// -----------------------------------------------

	// ------ Final uint32 Values printing -----------
	if( ucRXChar == 'o' )
	{	// display the uint32 final interaction results
		StopAllDebugMessages();
		m_ucPrintFinalData = 1;	// enabled
		return;
	}
	// -----------------------------------------------

	// ----- Final truncated Values printing ---------
	if( ucRXChar == 'p' )
	{	// display the truncated final interaction results
		StopAllDebugMessages();
		m_ucPrintFinalTruncatedData = 1;	// enabled
		return;
	}
	// -----------------------------------------------


	// sensitivity settings
	// -------- Print Sensitivity Settings -----------
	if( ucRXChar == 'l')
	{	// print the current sensitivity settings without changing anything
		StopAllDebugMessages();
		m_ucPrintSensitivityData = 1;
		return;
	}
	// -----------------------------------------------

	// --------- change channel to be set ------------
	if( ucRXChar == 'k')
	{	// change channel index and print the current sensitivity settings
		m_ucSensitivityDataChannel ++;
		if( m_ucSensitivityDataChannel > 5 )	m_ucSensitivityDataChannel = 0;
		StopAllDebugMessages();
		m_ucPrintSensitivityData = 1;
		return;
	}
	// -----------------------------------------------

	// ------ change SELF or ANOTHER to be set -------
	if( ucRXChar == 'j')
	{	// change selected filed to be altered: SELF reflection or ANOTHER ROBOT and print the current sensitivity settings
		if( m_ucSensitivitySelectSelfOrAnotherRobot )	m_ucSensitivitySelectSelfOrAnotherRobot = 0;	// from self reflection
		else   											m_ucSensitivitySelectSelfOrAnotherRobot = 1;	// from another robot
		StopAllDebugMessages();
		m_ucPrintSensitivityData = 1;
		return;
	}
	// -----------------------------------------------

	// ------- change selected field UP/DOWN ---------
	if( ucRXChar == 'm')
	{	// increase the selected sensitivity field
		//ChangeSensitivityData(1);	// UP
		InteractionIncDecSensitivity(m_ucSensitivitySelectSelfOrAnotherRobot, m_ucSensitivityDataChannel, SENSITIVITY_INCREMENT);
		StopAllDebugMessages();
		m_ucPrintSensitivityData = 1;
		return;
	}
	if( ucRXChar == 'n')
	{	// decrease the selected sensitivity field
		//ChangeSensitivityData(0);	// DOWN
		InteractionIncDecSensitivity(m_ucSensitivitySelectSelfOrAnotherRobot, m_ucSensitivityDataChannel, SENSITIVITY_DECREMENT);
		StopAllDebugMessages();
		m_ucPrintSensitivityData = 1;
		return;
	}
	// -----------------------------------------------









	if( ucRXChar == '.' )
	{
		//BaseLineCalibrationShowOnWs2812(128);
		return;
	}
	if( ucRXChar == ',' )
	{
		//BaseLineCalibrationShowOnWs2812(0);
		return;
	}



	// -----------------------------------------------
	// -----------------------------------------------
	// -----------------------------------------------
	// -----------------------------------------------


	return;
}
#endif
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== SYSTEM_END


// STM32_HAL
//==================================================================================================================== STATIC_START
// defined in .c file
/*
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if(htim->Instance == TIM2)
	{	// 10KHZ interrupt


		return;
	}

	return;
}
*/
//----------------------------------------------------------------------------------
/*
 // implemented in main.c
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{	//UNUSED(hadc);

	if(hadc->Instance == ADC1)
	{
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);  //	debug on PC13 LOW

		m_ucDebugTemp = 1;	// conversion done
		return;
	}

	return;

}
*/
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== STATIC_END



// STATIC (INTERN) implementation
//==================================================================================================================== STATIC_START
//----------------------------------------------------------------------------------
static void UsedInThe10KHZISR(void)			// called in the 10KHZ timer2 ISR
{
	// ------- ID sending over the Side IR LEDs ---------
	if( (IDPatternGenerationData.ucCurrentPattern) )
	{
		IDPatternGenerationData.ucCurrentPattern --;
		return;
	}
	IDPatternGenerationData.ucCurrentPattern = IDPatternGenerationData.ucPatternBuffer[IDPatternGenerationData.ucPatternIndex];
	IDPatternGenerationData.ucPatternIndex ++;
	if( (IDPatternGenerationData.ucPatternIndex > INITIALIZATION_ID_PATTERN_MAX_INDEX) )	IDPatternGenerationData.ucPatternIndex = 0;
	// --------------------------------------------------


	// generate new IR pattern regardless if the acquisition is running or has finished
	GenerateIRPattern();


	// acquire another set of data if required
	TriggerDataAcquisition();

	return;
}
//----------------------------------------------------------------------------------
static void GenerateIRPattern(void)		// called in the 10KHZ timer2 ISR to generate new IR pattern
{	// generate new IR pattern on the side-IR LEDs

	// change the pattern identifier to be ready for the TriggerDataAcquisition() function
	if( (MeasurementData.ucIRPatternNumber) )	MeasurementData.ucIRPatternNumber = 0;
	else 										MeasurementData.ucIRPatternNumber = 1;

	// set the pattern according to the Pattern Number
	if( (MeasurementData.ucIRPatternNumber) )
	{	/* if(ucIRPatternNumber == 1)	=> the {1,3,5} IRChannles are ON and the {0,2,4} IRChannels are OFF */
		// set the LED=OFF  channels
		HAL_GPIO_WritePin(IR_EN_0_GPIO_Port, IR_EN_0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(IR_EN_2_GPIO_Port, IR_EN_2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(IR_EN_4_GPIO_Port, IR_EN_4_Pin, GPIO_PIN_RESET);
		// set the LED=ON channel
		HAL_GPIO_WritePin(IR_EN_1_GPIO_Port, IR_EN_1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(IR_EN_3_GPIO_Port, IR_EN_3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(IR_EN_5_GPIO_Port, IR_EN_5_Pin, GPIO_PIN_SET);
	}
	else
	{	/* if(ucIRPatternNumber == 0)	=> the {0,2,4} IRChannles are ON and the {1,3,5} IRChannels are OFF */
		// set the LED=OFF  channels
		HAL_GPIO_WritePin(IR_EN_1_GPIO_Port, IR_EN_1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(IR_EN_3_GPIO_Port, IR_EN_3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(IR_EN_5_GPIO_Port, IR_EN_5_Pin, GPIO_PIN_RESET);
		// set the LED=ON channel
		HAL_GPIO_WritePin(IR_EN_0_GPIO_Port, IR_EN_0_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(IR_EN_2_GPIO_Port, IR_EN_2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(IR_EN_4_GPIO_Port, IR_EN_4_Pin, GPIO_PIN_SET);
	}

	return;
}
//----------------------------------------------------------------------------------
static void TriggerDataAcquisition(void)	// called in the 10HZ timer2 ISR to trigger another data acquisition
{
	if( (MeasurementData.ucAcquisitionCounter == 0) )	return;		// acquisition round is over => no new measurement to be performed

	// perform another ADC->DMA measurement
	// store data into the m_ui32ADC[] buffer
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);	//  debug on PC13 HIGH
	if( HAL_ADC_Start_DMA(&hadc1,m_ui32ADC,6) != HAL_OK )
	{
		Error_Handler2(ERROR_CODE_INTERACTION_DEBUG_001);
	}
	return;
}
//----------------------------------------------------------------------------------
static void UsedInTheDMAISR(void)			// called from the ADC->DMA ISR to handle data collection after ADC is done
{	// ADC->DMA measurement is done => collect the data and decrement the acquisition counter

	uint8_t ucIndex;
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);	//  debug on PC13 HIGH

	if( (MeasurementData.ucAcquisitionCounter % 2) )
	{	// just the auto-double triggering DMA bulshit !, without this the channels 3,4 and 5 are misbehaving ?????

		if( (MeasurementData.ucIRPatternNumber) )
		{	/* if(ucIRPatternNumber == 1)	=> the {1,3,5} IRChannles are ON and the {0,2,4} IRChannels are OFF */

			// ---------- CH1 ------------- (ON)
			ucIndex = 1;
			MeasurementData.ucNumberOfLEDOnValues[ucIndex] ++;
			MeasurementData.ui32AccuLEDOn[ucIndex] += m_ui32ADC[ucIndex];
			if( (MeasurementData.ui32MaxLEDOn[ucIndex] < m_ui32ADC[ucIndex]) )		MeasurementData.ui32MaxLEDOn[ucIndex] = m_ui32ADC[ucIndex];
			if( (MeasurementData.ui32MinLEDOn[ucIndex] > m_ui32ADC[ucIndex]) )		MeasurementData.ui32MinLEDOn[ucIndex] = m_ui32ADC[ucIndex];
			// ----------------------------

			// ---------- CH3 ------------- (ON)
			ucIndex = 3;
			MeasurementData.ucNumberOfLEDOnValues[ucIndex] ++;
			MeasurementData.ui32AccuLEDOn[ucIndex] += m_ui32ADC[ucIndex];
			if( (MeasurementData.ui32MaxLEDOn[ucIndex] < m_ui32ADC[ucIndex]) )		MeasurementData.ui32MaxLEDOn[ucIndex] = m_ui32ADC[ucIndex];
			if( (MeasurementData.ui32MinLEDOn[ucIndex] > m_ui32ADC[ucIndex]) )		MeasurementData.ui32MinLEDOn[ucIndex] = m_ui32ADC[ucIndex];
			// ----------------------------

			// ---------- CH5 ------------- (ON)
			ucIndex = 5;
			MeasurementData.ucNumberOfLEDOnValues[ucIndex] ++;
			MeasurementData.ui32AccuLEDOn[ucIndex] += m_ui32ADC[ucIndex];
			if( (MeasurementData.ui32MaxLEDOn[ucIndex] < m_ui32ADC[ucIndex]) )		MeasurementData.ui32MaxLEDOn[ucIndex] = m_ui32ADC[ucIndex];
			if( (MeasurementData.ui32MinLEDOn[ucIndex] > m_ui32ADC[ucIndex]) )		MeasurementData.ui32MinLEDOn[ucIndex] = m_ui32ADC[ucIndex];
			// ----------------------------



			// ---------- CH0 ------------- (OFF)
			ucIndex = 0;
			MeasurementData.ucNumberOfLEDOffValues[ucIndex] ++;
			MeasurementData.ui32AccuLEDOff[ucIndex] += m_ui32ADC[ucIndex];
			if( (MeasurementData.ui32MaxLEDOff[ucIndex] < m_ui32ADC[ucIndex]) )		MeasurementData.ui32MaxLEDOff[ucIndex] = m_ui32ADC[ucIndex];
			if( (MeasurementData.ui32MinLEDOff[ucIndex] > m_ui32ADC[ucIndex]) )		MeasurementData.ui32MinLEDOff[ucIndex] = m_ui32ADC[ucIndex];
			// ----------------------------

			// ---------- CH2 ------------- (OFF)
			ucIndex = 2;
			MeasurementData.ucNumberOfLEDOffValues[ucIndex] ++;
			MeasurementData.ui32AccuLEDOff[ucIndex] += m_ui32ADC[ucIndex];
			if( (MeasurementData.ui32MaxLEDOff[ucIndex] < m_ui32ADC[ucIndex]) )		MeasurementData.ui32MaxLEDOff[ucIndex] = m_ui32ADC[ucIndex];
			if( (MeasurementData.ui32MinLEDOff[ucIndex] > m_ui32ADC[ucIndex]) )		MeasurementData.ui32MinLEDOff[ucIndex] = m_ui32ADC[ucIndex];
			// ----------------------------

			// ---------- CH4 ------------- (OFF)
			ucIndex = 4;
			MeasurementData.ucNumberOfLEDOffValues[ucIndex] ++;
			MeasurementData.ui32AccuLEDOff[ucIndex] += m_ui32ADC[ucIndex];
			if( (MeasurementData.ui32MaxLEDOff[ucIndex] < m_ui32ADC[ucIndex]) )		MeasurementData.ui32MaxLEDOff[ucIndex] = m_ui32ADC[ucIndex];
			if( (MeasurementData.ui32MinLEDOff[ucIndex] > m_ui32ADC[ucIndex]) )		MeasurementData.ui32MinLEDOff[ucIndex] = m_ui32ADC[ucIndex];
			// ----------------------------
		}
		else
		{	/* if(ucIRPatternNumber == 0)	=> the {0,2,4} IRChannles are ON and the {1,3,5} IRChannels are OFF */

			// ---------- CH0 ------------- (ON)
			ucIndex = 0;
			MeasurementData.ucNumberOfLEDOnValues[ucIndex] ++;
			MeasurementData.ui32AccuLEDOn[ucIndex] += m_ui32ADC[ucIndex];
			if( (MeasurementData.ui32MaxLEDOn[ucIndex] < m_ui32ADC[ucIndex]) )		MeasurementData.ui32MaxLEDOn[ucIndex] = m_ui32ADC[ucIndex];
			if( (MeasurementData.ui32MinLEDOn[ucIndex] > m_ui32ADC[ucIndex]) )		MeasurementData.ui32MinLEDOn[ucIndex] = m_ui32ADC[ucIndex];
			// ----------------------------

			// ---------- CH2 ------------- (ON)
			ucIndex = 2;
			MeasurementData.ucNumberOfLEDOnValues[ucIndex] ++;
			MeasurementData.ui32AccuLEDOn[ucIndex] += m_ui32ADC[ucIndex];
			if( (MeasurementData.ui32MaxLEDOn[ucIndex] < m_ui32ADC[ucIndex]) )		MeasurementData.ui32MaxLEDOn[ucIndex] = m_ui32ADC[ucIndex];
			if( (MeasurementData.ui32MinLEDOn[ucIndex] > m_ui32ADC[ucIndex]) )		MeasurementData.ui32MinLEDOn[ucIndex] = m_ui32ADC[ucIndex];
			// ----------------------------

			// ---------- CH4 ------------- (ON)
			ucIndex = 4;
			MeasurementData.ucNumberOfLEDOnValues[ucIndex] ++;
			MeasurementData.ui32AccuLEDOn[ucIndex] += m_ui32ADC[ucIndex];
			if( (MeasurementData.ui32MaxLEDOn[ucIndex] < m_ui32ADC[ucIndex]) )		MeasurementData.ui32MaxLEDOn[ucIndex] = m_ui32ADC[ucIndex];
			if( (MeasurementData.ui32MinLEDOn[ucIndex] > m_ui32ADC[ucIndex]) )		MeasurementData.ui32MinLEDOn[ucIndex] = m_ui32ADC[ucIndex];
			// ----------------------------



			// ---------- CH1 ------------- (OFF)
			ucIndex = 1;
			MeasurementData.ucNumberOfLEDOffValues[ucIndex] ++;
			MeasurementData.ui32AccuLEDOff[ucIndex] += m_ui32ADC[ucIndex];
			if( (MeasurementData.ui32MaxLEDOff[ucIndex] < m_ui32ADC[ucIndex]) )		MeasurementData.ui32MaxLEDOff[ucIndex] = m_ui32ADC[ucIndex];
			if( (MeasurementData.ui32MinLEDOff[ucIndex] > m_ui32ADC[ucIndex]) )		MeasurementData.ui32MinLEDOff[ucIndex] = m_ui32ADC[ucIndex];
			// ----------------------------

			// ---------- CH3 ------------- (OFF)
			ucIndex = 3;
			MeasurementData.ucNumberOfLEDOffValues[ucIndex] ++;
			MeasurementData.ui32AccuLEDOff[ucIndex] += m_ui32ADC[ucIndex];
			if( (MeasurementData.ui32MaxLEDOff[ucIndex] < m_ui32ADC[ucIndex]) )		MeasurementData.ui32MaxLEDOff[ucIndex] = m_ui32ADC[ucIndex];
			if( (MeasurementData.ui32MinLEDOff[ucIndex] > m_ui32ADC[ucIndex]) )		MeasurementData.ui32MinLEDOff[ucIndex] = m_ui32ADC[ucIndex];
			// ----------------------------

			// ---------- CH5 ------------- (OFF)
			ucIndex = 5;
			MeasurementData.ucNumberOfLEDOffValues[ucIndex] ++;
			MeasurementData.ui32AccuLEDOff[ucIndex] += m_ui32ADC[ucIndex];
			if( (MeasurementData.ui32MaxLEDOff[ucIndex] < m_ui32ADC[ucIndex]) )		MeasurementData.ui32MaxLEDOff[ucIndex] = m_ui32ADC[ucIndex];
			if( (MeasurementData.ui32MinLEDOff[ucIndex] > m_ui32ADC[ucIndex]) )		MeasurementData.ui32MinLEDOff[ucIndex] = m_ui32ADC[ucIndex];
			// ----------------------------
		}
	}

	// get the end moment
	if( (MeasurementData.ucAcquisitionCounter == 1) )
	{
		MeasurementData.ui32AcquisitionEndMomentUS = __HAL_TIM_GET_COUNTER(&htim5);
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);	//  debug on PC13 LOW
	}
	// decrement acquisition counter
	if( (MeasurementData.ucAcquisitionCounter > 0) )	MeasurementData.ucAcquisitionCounter --;
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);	//  debug on PC13 LOW

	return;
}
//----------------------------------------------------------------------------------
static void UsedInTheMainLoop(void)			// called from the main loop to handle the collected data
{
	// ------ Save the measured data and trigger another measurement ---------
	uint8_t		i;
	int32_t		i32Temp;
	double 		dTempON, dTempOFF;

	// check if the current measurement is done
	if( (MeasurementData.ucAcquisitionCounter == 0) )
	{	// last acquisition done

		// only update the Result if the old result was read out
		if( (MeasurementResult.ucNewDataAvailable == 0) )
		{
			// calculate and save the measurement duration result
			if( (MeasurementData.ui32AcquisitionStartMomentUS > MeasurementData.ui32AcquisitionEndMomentUS) )	MeasurementResult.ui32AcquisitionDurationUS = ( (0xFFFFFFFF - MeasurementData.ui32AcquisitionStartMomentUS) + MeasurementData.ui32AcquisitionEndMomentUS );
			else   																								MeasurementResult.ui32AcquisitionDurationUS = ( MeasurementData.ui32AcquisitionEndMomentUS - MeasurementData.ui32AcquisitionStartMomentUS );

			// store the acquisition number
			MeasurementResult.ui32AcquisitionRoundCounter = MeasurementData.ui32AcquisitionRoundCounter;

			// prepare the results
			for(i=0; i<6; i++)
			{
				// calculate signal from self reflection
				dTempON   = (double) MeasurementData.ui32AccuLEDOn[i];
				dTempON  /= (double) MeasurementData.ucNumberOfLEDOnValues[i];
				dTempOFF  = (double) MeasurementData.ui32AccuLEDOff[i];
				dTempOFF /= (double) MeasurementData.ucNumberOfLEDOffValues[i];
				i32Temp   = (int32_t)(dTempON - dTempOFF);
				if( (i32Temp > 0) )		MeasurementResult.ui32SignalFromSelfReflection[i] = (uint32_t)i32Temp;
				else 					MeasurementResult.ui32SignalFromSelfReflection[i] = 0;


				// calculate signal from another robot
				i32Temp = (int32_t) ( (MeasurementData.ui32MaxLEDOn[i] - MeasurementData.ui32MinLEDOn[i]) + (MeasurementData.ui32MaxLEDOff[i] - MeasurementData.ui32MinLEDOff[i]) );
				if( (i32Temp > 0) )		MeasurementResult.ui32SignalFromAnotherRobot[i] = (uint32_t)i32Temp;
				else  					MeasurementResult.ui32SignalFromAnotherRobot[i] = 0;

			}

			// indicate that new result is available. this field must be cleared to 0 in order to update this structure again after another acquisition round
			MeasurementResult.ucNewDataAvailable = 1;

		}


		// clear the Measurement data structure
		for(i=0; i<6; i++)
		{
			// ON values
			MeasurementData.ucNumberOfLEDOnValues[i] = 0;
			MeasurementData.ui32AccuLEDOn[i] = 0;
			MeasurementData.ui32MaxLEDOn[i] = 0;
			MeasurementData.ui32MinLEDOn[i] = 4095;

			// OFF values
			MeasurementData.ucNumberOfLEDOffValues[i] = 0;
			MeasurementData.ui32AccuLEDOff[i] = 0;
			MeasurementData.ui32MaxLEDOff[i] = 0;
			MeasurementData.ui32MinLEDOff[i] = 4095;
		}

		// restart another measurement sequence
		if( (MeasurementData.ui32AcquisitionRoundCounter < 0xFFFFFFFF) )	MeasurementData.ui32AcquisitionRoundCounter ++;
		MeasurementData.ui32AcquisitionStartMomentUS = __HAL_TIM_GET_COUNTER(&htim5);
		MeasurementData.ucAcquisitionCounter = MeasurementData.ucAcquisitionCounterMax;
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);	//  debug on PC13 HIGH
		return;
	}
	// -----------------------------------------------------------------------


	return;
}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
static void SpikeFilterInitialization(void)	// called to initialize the "SpikeFilterData" data structure.
{
	// nothing here for now
	return;
}
//----------------------------------------------------------------------------------
#ifdef INTERACTION_USES_SPIKE_FILTER
static void SpikeFilterMain(void)				// called from the main loop to filter the spikes from the "MeasurementResult" structure.
{	// MeasurementResult.ucNewDataAvailable = 0;

	uint8_t i;			// for indexing

	// signal from another robot
	for(i=0;i<6;i++)
	{	// i is used to cycle all the 6 channels

		// ------ check if the filter has the full history buffer loaded with data --------
		if( (SpikeFilterData.ucSamplesHistoryCurrentDepth[i] < INTERACTION_SPIKE_HISTORY_DEPTH) )
		{	// the buffer is not yet full => keep filling in the buffer

			// check if we are right after spike filter reset => there is no data in the history buffer => if this is the case insert the first element
			if( (SpikeFilterData.ucSamplesHistoryCurrentDepth[i] == 0) )			// the history buffer is completely empty for this channel
			{
				SpikeFilterData.ui32SamplesHistory[i][0] 			= MeasurementResult.ui32SignalFromAnotherRobot[i];
				SpikeFilterData.ucSamplesHistoryCurrentDepth[i] 	= 1;	// the first element is inserted into the history buffer
			}
			else if( !(SpikeSearchOnTheNewDataFromAnotherRobot(i)) )				// the history buffer contains some elements => add next element if not a spike
			{	// the new data is not a spike => add it to the history buffer
				SpikeFilterData.ucSamplesSuccessiveErrorCounter[i] 	= 0;	// reset the successive error counter
				SpikeFilterData.ui32SamplesHistory[i][SpikeFilterData.ucSamplesHistoryCurrentDepth[i]] 			= MeasurementResult.ui32SignalFromAnotherRobot[i];	// add this new element
				SpikeFilterData.ucSamplesHistoryCurrentDepth[i] ++;																									// increase the history buffer current depth
			}
			else
			{	// the current element is considered to be a spike => do not add to the history buffer => increment the consecutive error counter
				SpikeFilterData.ucSamplesSuccessiveErrorCounter[i] ++;
				if( (SpikeFilterData.ucSamplesSuccessiveErrorCounter[i] >= INTERACTION_SPIKE_MAXIMUM_CONSECUTIVE_ERRORS) )
				{	// too many consecutive errors => reset the spike buffer's current channel
					SpikeResetAFilterChannel(i);
				}
			}

			// this round no data is available because the filter dose not yet work
			MeasurementResult.ucNewDataAvailable = 0;
			continue;	// go to the next channel
		}
		else
		{	// the buffer is full => one can check the new data against the entire buffer
			if( !(SpikeSearchOnTheNewDataFromAnotherRobot(i)) )
			{	// no spike detected => the new data filed is not reset (MeasurementResult.ucNewDataAvailable ---> left to be 1)
				SpikeFilterData.ucSamplesSuccessiveErrorCounter[i] 	= 0;	// reset the successive error counter
				// add the new data into the history buffer
				SpikeFilterData.ui32SamplesHistory[i][SpikeFilterData.ucSamplesHistoryIndex[i]] 			= MeasurementResult.ui32SignalFromAnotherRobot[i];	// add this new element

				// increment the next index on the history buffer => to point tot he oldest data in the history buffer
				SpikeFilterData.ucSamplesHistoryIndex[i] ++;
				if( (SpikeFilterData.ucSamplesHistoryIndex[i] >= (INTERACTION_SPIKE_HISTORY_DEPTH)) )		SpikeFilterData.ucSamplesHistoryIndex[i] = 0;
			}
			else
			{	// spike detected
				MeasurementResult.ucNewDataAvailable = 0;	// the data is a spike
				SpikeFilterData.ucSamplesSuccessiveErrorCounter[i] ++;
				if( (SpikeFilterData.ucSamplesSuccessiveErrorCounter[i] >= INTERACTION_SPIKE_MAXIMUM_CONSECUTIVE_ERRORS) )
				{	// too many consecutive errors => reset the spike buffer's current channel
					SpikeResetAFilterChannel(i);
				}
			}
		}
		// --------------------------------------------------------------------------------

	}

	// signal from self reflection
	// not implemented

	return;
}

//----------------------------------------------------------------------------------
static uint8_t SpikeSearchOnTheNewDataFromAnotherRobot(uint8_t ucChannel)		// compare a new data from another robot on channel i against the history buffer. return 0 if the data is no spike, returns 1if the data is found to be a spike
{	// this function compares the new data available from 		"MeasurementResult.ui32SignalFromAnotherRobot[ucChannel]"
	// against all the elements stored in the history buffer 	"SpikeFilterData.ui32SamplesHistory[ucChannel][k]"
	// with k ranging from 0 to "SpikeFilterData.ucSamplesHistoryCurrentDepth[ucChannel]"
	// if at least one difference is detected to be larger than the "INTERACTION_SPIKE_MAXIMUM_SPIKE" value => the new sample is considered to be a spike and the return value of this function will be 1
	// if none of the history values do not differ from the new value that much than the new sample is considered to be ok => no spike ant this function return 0
	// -- ucChannel = {0,1,2,3,4,5}
	// -- returns 1 if the new data is considered to be a spike
	// -- returns 0 if the new data is considered not to be a spike
	//------------

	uint8_t k;	// for indexing

	// the history buffer is considered not to be completely empty => SpikeFilterData.ucSamplesHistoryCurrentDepth[i] > 0
	for(k=0; k<SpikeFilterData.ucSamplesHistoryCurrentDepth[ucChannel]; k++)
	{
		if( abs(MeasurementResult.ui32SignalFromAnotherRobot[ucChannel] - SpikeFilterData.ui32SamplesHistory[ucChannel][k]) > INTERACTION_SPIKE_MAXIMUM_SPIKE )		return 1;		// spike detected
	}


	return 0;	// the new data is not a spike
}
//----------------------------------------------------------------------------------
static void SpikeResetAFilterChannel(uint8_t ucChannel)						// resets a filter channel defined by ucChannel. => emptys the history buffer and the control variables
{	// -- ucChannel = {0,1,2,3,4,5}
	// -- if ucChannel > 5 => reset all the channels
	//---------------------------------
	uint8_t i;

	// reset all the channels
	if( (ucChannel > 5) )
	{	// reset all channels
		for(i=0; i<5; i++)
		{
			SpikeFilterData.ucSamplesHistoryCurrentDepth[i]		= 0;
			SpikeFilterData.ucSamplesHistoryIndex[i]			= 0;
			SpikeFilterData.ucSamplesSuccessiveErrorCounter[i]	= 0;
		}
		return;
	}

	// only reset a specific channel
	// empty the history depth
	SpikeFilterData.ucSamplesHistoryCurrentDepth[ucChannel]		= 0;

	// empty the next index field
	SpikeFilterData.ucSamplesHistoryIndex[ucChannel]			= 0;	// where to put the next valid data in the history buffer after the buffer is fully operational

	// empty the successive error counter
	SpikeFilterData.ucSamplesSuccessiveErrorCounter[ucChannel]	= 0;

	// the history buffer itself nose not need to be empty because it is always rewritten

	return;
}
#endif
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
static void MovingAverageFilterInitialization(void)		// called in the initialization section of the code
{
	// nothing here for now ...
	return;
}
//----------------------------------------------------------------------------------
#ifdef INTARACTION_USES_MOVING_AVERAGING_FILTER
static void MovingAverageFilterMain(void)					// called in the main loop section to filter out the raw data before base line calibration or further data processing
{	// moving average the raw data and store back the result into the same data structure

	uint8_t i,j;
	uint32_t	ui32SelfReflection[6], uiFromAnotherRobot[6];

	// adding the new data
	for(i=0; i<6; i++)
	{	// do the same for each channel

		// from self reflection
		MovingAveragedData.ui32SamplesFromSelfReflection[i][MovingAveragedData.ucSampleIndexer[i]] 		= MeasurementResult.ui32SignalFromSelfReflection[i];

		// from another robot
		MovingAveragedData.ui32SamplesFromAnotherRobot[i][MovingAveragedData.ucSampleIndexer[i]]		= MeasurementResult.ui32SignalFromAnotherRobot[i];

		// update the indexer
		MovingAveragedData.ucSampleIndexer[i] ++;
		if( (MovingAveragedData.ucSampleIndexer[i] >= INTERACTION_MOVING_AVERAGE_FILTER_MAX_NUMBER_OF_SAMPLES) )	MovingAveragedData.ucSampleIndexer[i] = 0;
	}

	// accumulating all the data
	for(i=0; i<6; i++)
	{	// for all the channels

		// empty the accumulation buffers
		ui32SelfReflection[i] = 0;
		uiFromAnotherRobot[i] = 0;

		for(j=0; j<INTERACTION_MOVING_AVERAGE_FILTER_MAX_NUMBER_OF_SAMPLES; j++)
		{	// for all the samples
			// from self reflection
			ui32SelfReflection[i]	+= MovingAveragedData.ui32SamplesFromSelfReflection[i][j];

			// from other robot
			uiFromAnotherRobot[i]	+= MovingAveragedData.ui32SamplesFromAnotherRobot[i][j];
		}

		// for self reflection
		MeasurementResult.ui32SignalFromSelfReflection[i] 	= (ui32SelfReflection[i] >> INTERACTION_MOVING_AVERAGE_FILTER_MAX_DEPTH_POWER);

		// from another robot
		MeasurementResult.ui32SignalFromAnotherRobot[i]		= (uiFromAnotherRobot[i] >> INTERACTION_MOVING_AVERAGE_FILTER_MAX_DEPTH_POWER);

	}

	return;
}
#endif
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
static void BaseLineInitialization(void)		// used to initialize the base line structure and calibration algorithm
{	// called from the init section

	// ROM settings
	BaseLineData.ucAccumulationDivisionPower			= INTERACTION_BASE_LINE_CALIBRATION_MAX_ACCUMULATION_POWER;	/* the division after adding => to get the average, the number of adding is power of 2 => the division is just a shift */
	BaseLineData.ucAccumulationCounterMax 				= INTERACTION_BASE_LINE_CALIBRATION_MAX_ACCUMULATIONS;		/* determines the number of consecutive measurements for the average base line calculation */
	BaseLineData.ui16TimeToCheckCalibrationMSMax		= INTERACTION_BASE_LINE_CALIBRATION_TRIGGER_TIME_MS;		/* determines after how many milliseconds from restart check if a base line is present and if not trigger a base-line calibration */
	BaseLineData.ui32SelfMaxNoise						= INTERACTION_MAX_NOISE_SELF_REFLECTION;					/* determines the maximum noise allowed during the base line calibration */
	BaseLineData.ui32OtherRobotMaxNoise					= INTERACTION_MAX_NOISE_OTHER_ROBOT;						/*  */

	// EEPROM settings
	// check if the data is available in the EEPROM
	EEpromData[0].ui16EEPROMAddress		= EEPROM_INTERACTION_BASE_LINE_CALIBRATION_DONE_ADDRESS;
	if( I2CEEPROMReadIS(&EEpromData[0]) == HAL_OK )
	{	// EEPROM reading is ok
		if( (EEpromData[0].ui32EEPROMData == EEPROM_INTERACTION_BASE_LINE_CALIBRATION_MARKER) )
		{	// eeprom is formatted

			// read the self reflection base line data
			// CH0
			EEpromData[0].ui16EEPROMAddress		= EEPROM_INTERACTION_BASE_LINE_SELF_REFLECTION_CH0_ADDRESS;
			I2CEEPROMReadIS(&EEpromData[0]);
			BaseLineData.ui32CalculatedSelfBaseLineValues[0]	= EEpromData[0].ui32EEPROMData;
			// CH1
			EEpromData[0].ui16EEPROMAddress		= EEPROM_INTERACTION_BASE_LINE_SELF_REFLECTION_CH1_ADDRESS;
			I2CEEPROMReadIS(&EEpromData[0]);
			BaseLineData.ui32CalculatedSelfBaseLineValues[1]	= EEpromData[0].ui32EEPROMData;
			// CH2
			EEpromData[0].ui16EEPROMAddress		= EEPROM_INTERACTION_BASE_LINE_SELF_REFLECTION_CH2_ADDRESS;
			I2CEEPROMReadIS(&EEpromData[0]);
			BaseLineData.ui32CalculatedSelfBaseLineValues[2]	= EEpromData[0].ui32EEPROMData;
			// CH3
			EEpromData[0].ui16EEPROMAddress		= EEPROM_INTERACTION_BASE_LINE_SELF_REFLECTION_CH3_ADDRESS;
			I2CEEPROMReadIS(&EEpromData[0]);
			BaseLineData.ui32CalculatedSelfBaseLineValues[3]	= EEpromData[0].ui32EEPROMData;
			// CH4
			EEpromData[0].ui16EEPROMAddress		= EEPROM_INTERACTION_BASE_LINE_SELF_REFLECTION_CH4_ADDRESS;
			I2CEEPROMReadIS(&EEpromData[0]);
			BaseLineData.ui32CalculatedSelfBaseLineValues[4]	= EEpromData[0].ui32EEPROMData;
			// CH5
			EEpromData[0].ui16EEPROMAddress		= EEPROM_INTERACTION_BASE_LINE_SELF_REFLECTION_CH5_ADDRESS;
			I2CEEPROMReadIS(&EEpromData[0]);
			BaseLineData.ui32CalculatedSelfBaseLineValues[5]	= EEpromData[0].ui32EEPROMData;

			// read the base line values for the signal from another robot
			// CH0
			EEpromData[0].ui16EEPROMAddress		= EEPROM_INTERACTION_BASE_LINE_ANOTHER_ROBOT_CH0_ADDRESS;
			I2CEEPROMReadIS(&EEpromData[0]);
			BaseLineData.ui32CalculatedBaseValuesFromOtherRobot[0]	= EEpromData[0].ui32EEPROMData;
			// CH1
			EEpromData[0].ui16EEPROMAddress		= EEPROM_INTERACTION_BASE_LINE_ANOTHER_ROBOT_CH1_ADDRESS;
			I2CEEPROMReadIS(&EEpromData[0]);
			BaseLineData.ui32CalculatedBaseValuesFromOtherRobot[1]	= EEpromData[0].ui32EEPROMData;
			// CH2
			EEpromData[0].ui16EEPROMAddress		= EEPROM_INTERACTION_BASE_LINE_ANOTHER_ROBOT_CH2_ADDRESS;
			I2CEEPROMReadIS(&EEpromData[0]);
			BaseLineData.ui32CalculatedBaseValuesFromOtherRobot[2]	= EEpromData[0].ui32EEPROMData;
			// CH3
			EEpromData[0].ui16EEPROMAddress		= EEPROM_INTERACTION_BASE_LINE_ANOTHER_ROBOT_CH3_ADDRESS;
			I2CEEPROMReadIS(&EEpromData[0]);
			BaseLineData.ui32CalculatedBaseValuesFromOtherRobot[3]	= EEpromData[0].ui32EEPROMData;
			// CH4
			EEpromData[0].ui16EEPROMAddress		= EEPROM_INTERACTION_BASE_LINE_ANOTHER_ROBOT_CH4_ADDRESS;
			I2CEEPROMReadIS(&EEpromData[0]);
			BaseLineData.ui32CalculatedBaseValuesFromOtherRobot[4]	= EEpromData[0].ui32EEPROMData;
			// CH5
			EEpromData[0].ui16EEPROMAddress		= EEPROM_INTERACTION_BASE_LINE_ANOTHER_ROBOT_CH5_ADDRESS;
			I2CEEPROMReadIS(&EEpromData[0]);
			BaseLineData.ui32CalculatedBaseValuesFromOtherRobot[5]	= EEpromData[0].ui32EEPROMData;

			BaseLineData.ucBaseLineCalibrationDone = 1;	// indicate that there is no need for base line calibration


			#if defined(INTERACTION_DEBUG_MODE) && defined(INTERACTION_EEPROM_DEBUG)
			sprintf((char * )m_ucDebugBuffer,"EE_BaseLineInitialization()__A\r\n");
			DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
			return;
			#endif
		}
		#if defined(INTERACTION_DEBUG_MODE) && defined(INTERACTION_EEPROM_DEBUG)
		sprintf((char * )m_ucDebugBuffer,"EE_BaseLineInitialization()__B_NotFormatted\r\n");
		DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
		return;
		#endif
	}
	#if defined(INTERACTION_DEBUG_MODE) && defined(INTERACTION_EEPROM_DEBUG)
	sprintf((char * )m_ucDebugBuffer,"EE_BaseLineInitialization()__C_ReadError\r\n");
	DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
	return;
	#endif



	// EEPROM formating
	// no data here for now, only later after the base line calibration is done

	return;
}
//----------------------------------------------------------------------------------
static void BaselineCalibrationMain(void)			// called from the "UsedInTheMainLoop()" during baseline calibration
{	// all data is stored in the: "BaseLineData" variable
	// full base line calibration without noise takes 8uS
	//-----------------------------------------------------
	uint8_t i;	// for indexing
	uint8_t ucNoiseS, ucNoiseO;	// noise condition from another robot and from self

	// check if the base line is already detected and nothing else is to be done
	if((BaseLineData.ucBaseLineCalibrationDone))	return;	// nothing else to do
	//--------------------------------------------

	// check if is the right time to do the calibration
	if( (BaseLineData.ui16TimeToCheckCalibrationMS < BaseLineData.ui16TimeToCheckCalibrationMSMax) )	return;	// is not yet the right time after reset
	//--------------------------------------------

	// debug
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);	//  debug on PC13 HIGH


	// perform the accumulation
	for(i=0; i<6; i++)
	{
		// signal from self reflection
		BaseLineData.ui32AccuSignalFromSelfReflection[i]	+= MeasurementResult.ui32SignalFromSelfReflection[i];		// add signal from each channel into accumulator
		if( (BaseLineData.ui32AccuSignalFromSelfReflectionMin[i] > MeasurementResult.ui32SignalFromSelfReflection[i]) )		BaseLineData.ui32AccuSignalFromSelfReflectionMin[i] = MeasurementResult.ui32SignalFromSelfReflection[i];	// minimum detection
		if( (BaseLineData.ui32AccuSignalFromSelfReflectionMax[i] < MeasurementResult.ui32SignalFromSelfReflection[i]) )		BaseLineData.ui32AccuSignalFromSelfReflectionMax[i] = MeasurementResult.ui32SignalFromSelfReflection[i];	// maximum detection

		// signal from another robot
		BaseLineData.ui32AccuSignalFromOtherRobot[i]		+= MeasurementResult.ui32SignalFromAnotherRobot[i];			// add the signal from another robot into accumulator
		if( (BaseLineData.ui32AccuSignalFromOtherRobotMin[i] > MeasurementResult.ui32SignalFromAnotherRobot[i]) )			BaseLineData.ui32AccuSignalFromOtherRobotMin[i] = MeasurementResult.ui32SignalFromAnotherRobot[i];		// minimum detection
		if( (BaseLineData.ui32AccuSignalFromOtherRobotMax[i] < MeasurementResult.ui32SignalFromAnotherRobot[i]) )			BaseLineData.ui32AccuSignalFromOtherRobotMax[i] = MeasurementResult.ui32SignalFromAnotherRobot[i];		// maximum detection
	}

	if( (BaseLineData.ucAccumulationCounter) )		BaseLineData.ucAccumulationCounter --;
	if( (BaseLineData.ucAccumulationCounter == 0) )
	{	// end of the accumulation

		// ---------- check the noise condition -------------
		ucNoiseS = 0;	// noise from self initialized as ok
		ucNoiseO = 0;	// noise from other robot initialized as ok
		for(i=0; i<6; i++)
		{
			// from self reflection
			if( ( (BaseLineData.ui32AccuSignalFromSelfReflectionMax[i] - BaseLineData.ui32AccuSignalFromSelfReflectionMin[i] ) > BaseLineData.ui32SelfMaxNoise ) )	ucNoiseS = 1;	// too large noise detected => no base line calibration this time

			// from another robot
			if( ( (BaseLineData.ui32AccuSignalFromOtherRobotMax[i] - BaseLineData.ui32AccuSignalFromOtherRobotMin[i] ) > BaseLineData.ui32OtherRobotMaxNoise ) )	ucNoiseO = 1;	// too large noise detected => no base line calibration this time
		}
		if( (ucNoiseS || ucNoiseO) )
		{	// too large noise
			// restart the base line calibration
			BaseLineDataReset();	// clear all the accumulators
			return;
		}
		// noise seems to be ok => in the range
		// --------------------------------------------------


		// -------- calculate the base line values ----------
		for(i=0; i<6; i++)
		{
			// from self reflection
			BaseLineData.ui32AccuSignalFromSelfReflection[i] 		>>= BaseLineData.ucAccumulationDivisionPower;						// divide with the number of accumulations (always power of 2)
			BaseLineData.ui32AccuSignalFromSelfReflection[i] 		+=  (BaseLineData.ui32AccuSignalFromSelfReflection[i] >> 1);		// add 50% to the base line
			BaseLineData.ui32CalculatedSelfBaseLineValues[i] 		= 	 BaseLineData.ui32AccuSignalFromSelfReflection[i];				// store the result into the dedicated field

			// from another robot
			BaseLineData.ui32AccuSignalFromOtherRobot[i]			>>= BaseLineData.ucAccumulationDivisionPower;						// divide with the number of accumulations (always power of 2)
			BaseLineData.ui32AccuSignalFromOtherRobot[i]			+=  (BaseLineData.ui32AccuSignalFromOtherRobot[i] >> 1);			// add 50% to the base line
			BaseLineData.ui32CalculatedBaseValuesFromOtherRobot[i]	=	BaseLineData.ui32AccuSignalFromOtherRobot[i];					// store the result into the dedicated field
		}
		// store the values into the  EEPROM
		m_ucEEPROMNewBaseLineDataIsAvailable = 1;
		BaseLineData.ucBaseLineCalibrationDone = 1;	// base line was detected
		// --------------------------------------------------

		// --------------------------------------------------
		// --------------------------------------------------
		// --------------------------------------------------
	}

	// debug
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);	//  debug on PC13 LOW

	return;
}
//----------------------------------------------------------------------------------
static void BaseLineTimer1KHZ(void)			// called from the 1KHZ timer to handle real time requirements in the base line calibration logic
{
	if( (BaseLineData.ui16TimeToCheckCalibrationMS < BaseLineData.ui16TimeToCheckCalibrationMSMax) )	BaseLineData.ui16TimeToCheckCalibrationMS ++;
	return;
}
//----------------------------------------------------------------------------------
static void BaseLineDataReset(void)			// called to reset the base line data structure "BaseLineData"
{	// clears all the accumulators and the counter variable
	// -----------------------------------------
	uint8_t i;

	for(i=0; i<6; i++)
	{
		// self reflections
		BaseLineData.ui32AccuSignalFromSelfReflection[i] 		= 0;
		BaseLineData.ui32AccuSignalFromSelfReflectionMin[i]		= 4095;
		BaseLineData.ui32AccuSignalFromSelfReflectionMax[i]		= 0;
		BaseLineData.ui32CalculatedSelfBaseLineValues[i]		= 0;

		// from other robots
		BaseLineData.ui32AccuSignalFromOtherRobot[i]			= 0;
		BaseLineData.ui32AccuSignalFromOtherRobotMin[i]			= 4095;
		BaseLineData.ui32AccuSignalFromOtherRobotMax[i]			= 0;
		BaseLineData.ui32CalculatedBaseValuesFromOtherRobot[i]	= 0;
	}

	BaseLineData.ucAccumulationCounter = BaseLineData.ucAccumulationCounterMax;		// from this value counts down. this is power of 2 def. by: "INTERACTION_BASE_LINE_CALIBRATION_MAX_ACCUMULATIONS"
	BaseLineData.ucBaseLineCalibrationDone = 0;		// not yet accumulated

	return;
}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
static void SensitivityInitialization(void)	// retrieve the sensitivity information from the EEPROM or from ROM
{
	// 0) set the ROM values (some default values)
	// self reflection
	SensitivityData.ucSensitivityForSelfReflection[0] = 50;
	SensitivityData.ucSensitivityForSelfReflection[1] = 50;
	SensitivityData.ucSensitivityForSelfReflection[2] = 50;
	SensitivityData.ucSensitivityForSelfReflection[3] = 50;
	SensitivityData.ucSensitivityForSelfReflection[4] = 50;
	SensitivityData.ucSensitivityForSelfReflection[5] = 50;
	// from other robot
	SensitivityData.ucSensitivityForOtherRobot[0] = 50;
	SensitivityData.ucSensitivityForOtherRobot[1] = 50;
	SensitivityData.ucSensitivityForOtherRobot[2] = 50;
	SensitivityData.ucSensitivityForOtherRobot[3] = 50;
	SensitivityData.ucSensitivityForOtherRobot[4] = 50;
	SensitivityData.ucSensitivityForOtherRobot[5] = 50;


	// 1) if present in EEPROM => retrieve from EEPROM and replace the earlier set ROM values
	// check if the data is available in the EEPROM
	EEpromData[0].ui16EEPROMAddress		= EEPROM_INTERACTION_SENSITIVITY_DATA_OK_ADDRESS;
	if( I2CEEPROMReadIS(&EEpromData[0]) == HAL_OK )
	{	// EEPROM reading is ok
		if( (EEpromData[0].ui32EEPROMData == EEPROM_INTERACTION_SENSITIVITY_DATA_OK_MARKER) )
		{	// eeprom is formatted

			// read the self reflection sensitivity values
			// CH0
			EEpromData[0].ui16EEPROMAddress		= EEPROM_INTERACTION_SENSITIVITY_SELF_REFLECTION_CH0_ADDRESS;
			I2CEEPROMReadIS(&EEpromData[0]);
			SensitivityData.ucSensitivityForSelfReflection[0]	= EEpromData[0].ui32EEPROMData;
			// CH1
			EEpromData[0].ui16EEPROMAddress		= EEPROM_INTERACTION_SENSITIVITY_SELF_REFLECTION_CH1_ADDRESS;
			I2CEEPROMReadIS(&EEpromData[0]);
			SensitivityData.ucSensitivityForSelfReflection[1]	= EEpromData[0].ui32EEPROMData;
			// CH2
			EEpromData[0].ui16EEPROMAddress		= EEPROM_INTERACTION_SENSITIVITY_SELF_REFLECTION_CH2_ADDRESS;
			I2CEEPROMReadIS(&EEpromData[0]);
			SensitivityData.ucSensitivityForSelfReflection[2]	= EEpromData[0].ui32EEPROMData;
			// CH3
			EEpromData[0].ui16EEPROMAddress		= EEPROM_INTERACTION_SENSITIVITY_SELF_REFLECTION_CH3_ADDRESS;
			I2CEEPROMReadIS(&EEpromData[0]);
			SensitivityData.ucSensitivityForSelfReflection[3]	= EEpromData[0].ui32EEPROMData;
			// CH4
			EEpromData[0].ui16EEPROMAddress		= EEPROM_INTERACTION_SENSITIVITY_SELF_REFLECTION_CH4_ADDRESS;
			I2CEEPROMReadIS(&EEpromData[0]);
			SensitivityData.ucSensitivityForSelfReflection[4]	= EEpromData[0].ui32EEPROMData;
			// CH5
			EEpromData[0].ui16EEPROMAddress		= EEPROM_INTERACTION_SENSITIVITY_SELF_REFLECTION_CH5_ADDRESS;
			I2CEEPROMReadIS(&EEpromData[0]);
			SensitivityData.ucSensitivityForSelfReflection[5]	= EEpromData[0].ui32EEPROMData;

			// read the sensitivity settings for the signal from another robot
			// CH0
			EEpromData[0].ui16EEPROMAddress		= EEPROM_INTERACTION_SENSITIVITY_ANOTHER_ROBOT_CH0_ADDRESS;
			I2CEEPROMReadIS(&EEpromData[0]);
			SensitivityData.ucSensitivityForOtherRobot[0]	= EEpromData[0].ui32EEPROMData;
			// CH1
			EEpromData[0].ui16EEPROMAddress		= EEPROM_INTERACTION_SENSITIVITY_ANOTHER_ROBOT_CH1_ADDRESS;
			I2CEEPROMReadIS(&EEpromData[0]);
			SensitivityData.ucSensitivityForOtherRobot[1]	= EEpromData[0].ui32EEPROMData;
			// CH2
			EEpromData[0].ui16EEPROMAddress		= EEPROM_INTERACTION_SENSITIVITY_ANOTHER_ROBOT_CH2_ADDRESS;
			I2CEEPROMReadIS(&EEpromData[0]);
			SensitivityData.ucSensitivityForOtherRobot[2]	= EEpromData[0].ui32EEPROMData;
			// CH3
			EEpromData[0].ui16EEPROMAddress		= EEPROM_INTERACTION_SENSITIVITY_ANOTHER_ROBOT_CH3_ADDRESS;
			I2CEEPROMReadIS(&EEpromData[0]);
			SensitivityData.ucSensitivityForOtherRobot[3]	= EEpromData[0].ui32EEPROMData;
			// CH4
			EEpromData[0].ui16EEPROMAddress		= EEPROM_INTERACTION_SENSITIVITY_ANOTHER_ROBOT_CH4_ADDRESS;
			I2CEEPROMReadIS(&EEpromData[0]);
			SensitivityData.ucSensitivityForOtherRobot[4]	= EEpromData[0].ui32EEPROMData;
			// CH5
			EEpromData[0].ui16EEPROMAddress		= EEPROM_INTERACTION_SENSITIVITY_ANOTHER_ROBOT_CH5_ADDRESS;
			I2CEEPROMReadIS(&EEpromData[0]);
			SensitivityData.ucSensitivityForOtherRobot[5]	= EEpromData[0].ui32EEPROMData;

			#if defined(INTERACTION_DEBUG_MODE) && defined(INTERACTION_EEPROM_DEBUG)
			//#ifdef INTERACTION_DEBUG_MODE
			sprintf((char * )m_ucDebugBuffer,"EE_SensitivityInitialization()_R_A_retrieved\r\n");
			DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
			//return;
			#endif
			return;	// all done EERPOM values were retrieved.
		}
		else
		{	// no marker found
			#if defined(INTERACTION_DEBUG_MODE) && defined(INTERACTION_EEPROM_DEBUG)
			//#ifdef INTERACTION_DEBUG_MODE
			sprintf((char * )m_ucDebugBuffer,"EE_SensitivityInitialization()_R_B_notretrieved\r\n");
			DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
			//return;
			#endif
		}

	}
	else
	{	//EEPROM read error
		#if defined(INTERACTION_DEBUG_MODE) && defined(INTERACTION_EEPROM_DEBUG)
		//#ifdef INTERACTION_DEBUG_MODE
		sprintf((char * )m_ucDebugBuffer,"EE_SensitivityInitialization()_R_C_ReadError\r\n");
		DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
		//return;
		#endif
	}


	// 2) if not present in EEPROM => store the ROM values in the EEPROM
	// 2.a) store the values in the EEPROM
	// write the self reflection sensitivity data
	// CH0
	EEpromData[0].ui16EEPROMAddress		= EEPROM_INTERACTION_SENSITIVITY_SELF_REFLECTION_CH0_ADDRESS;
	EEpromData[0].ui32EEPROMData		= SensitivityData.ucSensitivityForSelfReflection[0];
	I2CEEPROMWriteIS(&EEpromData[0]);
	// CH1
	EEpromData[0].ui16EEPROMAddress		= EEPROM_INTERACTION_SENSITIVITY_SELF_REFLECTION_CH1_ADDRESS;
	EEpromData[0].ui32EEPROMData		= SensitivityData.ucSensitivityForSelfReflection[1];
	I2CEEPROMWriteIS(&EEpromData[0]);
	// CH2
	EEpromData[0].ui16EEPROMAddress		= EEPROM_INTERACTION_SENSITIVITY_SELF_REFLECTION_CH2_ADDRESS;
	EEpromData[0].ui32EEPROMData		= SensitivityData.ucSensitivityForSelfReflection[2];
	I2CEEPROMWriteIS(&EEpromData[0]);
	// CH3
	EEpromData[0].ui16EEPROMAddress		= EEPROM_INTERACTION_SENSITIVITY_SELF_REFLECTION_CH3_ADDRESS;
	EEpromData[0].ui32EEPROMData		= SensitivityData.ucSensitivityForSelfReflection[3];
	I2CEEPROMWriteIS(&EEpromData[0]);
	// CH4
	EEpromData[0].ui16EEPROMAddress		= EEPROM_INTERACTION_SENSITIVITY_SELF_REFLECTION_CH4_ADDRESS;
	EEpromData[0].ui32EEPROMData		= SensitivityData.ucSensitivityForSelfReflection[4];
	I2CEEPROMWriteIS(&EEpromData[0]);
	// CH5
	EEpromData[0].ui16EEPROMAddress		= EEPROM_INTERACTION_SENSITIVITY_SELF_REFLECTION_CH5_ADDRESS;
	EEpromData[0].ui32EEPROMData		= SensitivityData.ucSensitivityForSelfReflection[5];
	I2CEEPROMWriteIS(&EEpromData[0]);

	// write the sensitivity data for the signal from another robot
	// CH0
	EEpromData[0].ui16EEPROMAddress		= EEPROM_INTERACTION_SENSITIVITY_ANOTHER_ROBOT_CH0_ADDRESS;
	EEpromData[0].ui32EEPROMData		= SensitivityData.ucSensitivityForOtherRobot[0];
	I2CEEPROMWriteIS(&EEpromData[0]);
	// CH1
	EEpromData[0].ui16EEPROMAddress		= EEPROM_INTERACTION_SENSITIVITY_ANOTHER_ROBOT_CH1_ADDRESS;
	EEpromData[0].ui32EEPROMData		= SensitivityData.ucSensitivityForOtherRobot[1];
	I2CEEPROMWriteIS(&EEpromData[0]);
	// CH2
	EEpromData[0].ui16EEPROMAddress		= EEPROM_INTERACTION_SENSITIVITY_ANOTHER_ROBOT_CH2_ADDRESS;
	EEpromData[0].ui32EEPROMData		= SensitivityData.ucSensitivityForOtherRobot[2];
	I2CEEPROMWriteIS(&EEpromData[0]);
	// CH3
	EEpromData[0].ui16EEPROMAddress		= EEPROM_INTERACTION_SENSITIVITY_ANOTHER_ROBOT_CH3_ADDRESS;
	EEpromData[0].ui32EEPROMData		= SensitivityData.ucSensitivityForOtherRobot[3];
	I2CEEPROMWriteIS(&EEpromData[0]);
	// CH4
	EEpromData[0].ui16EEPROMAddress		= EEPROM_INTERACTION_SENSITIVITY_ANOTHER_ROBOT_CH4_ADDRESS;
	EEpromData[0].ui32EEPROMData		= SensitivityData.ucSensitivityForOtherRobot[4];
	I2CEEPROMWriteIS(&EEpromData[0]);
	// CH5
	EEpromData[0].ui16EEPROMAddress		= EEPROM_INTERACTION_SENSITIVITY_ANOTHER_ROBOT_CH5_ADDRESS;
	EEpromData[0].ui32EEPROMData		= SensitivityData.ucSensitivityForOtherRobot[5];
	I2CEEPROMWriteIS(&EEpromData[0]);

	// write the marker
	EEpromData[0].ui16EEPROMAddress		= EEPROM_INTERACTION_SENSITIVITY_DATA_OK_ADDRESS;
	EEpromData[0].ui32EEPROMData		= EEPROM_INTERACTION_SENSITIVITY_DATA_OK_MARKER;
	I2CEEPROMWriteIS(&EEpromData[0]);


	return;
}
//----------------------------------------------------------------------------------
static void InteractionResultCalculationMain(void)		// used to calculate the final result of the interaction
{
	uint8_t i;
	double dTemp;

	// debug
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);	//  debug on PC13 HIGH

	// set the statistics from the raw measurements
	InteractionResultData.ui32AcquisitionDurationUS 		= MeasurementResult.ui32AcquisitionDurationUS;
	InteractionResultData.ui32AcquisitionRoundCounter		= MeasurementResult.ui32AcquisitionRoundCounter;

	// ----------- determine the corrected data ---------------
	for(i=0; i<6; i++)
	{
		// --------- for self reflection -----------
		if( (MeasurementResult.ui32SignalFromSelfReflection[i] > BaseLineData.ui32CalculatedSelfBaseLineValues[i]) )
		{	// signal is above the baseline => subtract the baseline
			InteractionResultData.ui32CorrectedDataFromSelfReflection[i] = (MeasurementResult.ui32SignalFromSelfReflection[i] - BaseLineData.ui32CalculatedSelfBaseLineValues[i]) ;
		}
		else
		{	// below the baseline => considered 0
			InteractionResultData.ui32CorrectedDataFromSelfReflection[i] = 0;
		}
		// -----------------------------------------

		// --------- from other robot --------------
		if( (MeasurementResult.ui32SignalFromAnotherRobot[i] > BaseLineData.ui32CalculatedBaseValuesFromOtherRobot[i]) )
		{	// above the baseline =>further correction
			if( (MeasurementResult.ui32SignalFromAnotherRobot[i] < (3*BaseLineData.ui32CalculatedBaseValuesFromOtherRobot[i])) )
			{	// another robot is not even detected => interaction considered to be 0
				InteractionResultData.ui32CorrectedDataFromOtherRobot[i] = 0;
			}
			else
			{
				if( (MeasurementResult.ui32SignalFromAnotherRobot[i] < INTERACTION_DETECTED_OTHER_ROBOT_THRESHOLD_VALUE) )
				{	// other robot is detected but it is far away => consider interaction to be 1
					InteractionResultData.ui32CorrectedDataFromOtherRobot[i] = 5;	// weak interaction
				}
				else
				{	// other robot is detected and the signal is quite powerful
					InteractionResultData.ui32CorrectedDataFromOtherRobot[i] = 5+((MeasurementResult.ui32SignalFromAnotherRobot[i] - INTERACTION_DETECTED_OTHER_ROBOT_THRESHOLD_VALUE)>>3);	// divided by 8
				}
			}
		}
		else
		{	// below baseline => consider 0
			InteractionResultData.ui32CorrectedDataFromOtherRobot[i] = 0;
		}
		// -----------------------------------------
	}
	// --------------------------------------------------------


	// ------------ add the sensitivity factor ----------------
	for(i=0; i<6; i++)
	{
		// --------- for self reflection -----------
		dTemp  = (double)InteractionResultData.ui32CorrectedDataFromSelfReflection[i];
		dTemp *= (double)SensitivityData.ucSensitivityForSelfReflection[i];
		dTemp /= 50;	// 50 is the default sensitivity considered as factor of 1
		InteractionResultData.ui32FinalDataFromSelfReflection[i] = (uint32_t)dTemp;
		if(dTemp > 255) dTemp = 255;
		InteractionResultData.ucFinalDataFromSelfReflection[i] = (uint8_t)dTemp;
		// -----------------------------------------


		// --------- from other robot --------------
		if( (InteractionResultData.ui32CorrectedDataFromOtherRobot[i] < 2) )
		{	// the special cases
			InteractionResultData.ui32FinalDataFromOtherRobot[i] = InteractionResultData.ui32CorrectedDataFromOtherRobot[i];
			InteractionResultData.ucFinalDataFromOtherRobot[i]   = (uint8_t)InteractionResultData.ui32CorrectedDataFromOtherRobot[i];
		}
		else
		{	// strong signal situation
			dTemp  = (double)InteractionResultData.ui32CorrectedDataFromOtherRobot[i];
			dTemp *= (double)SensitivityData.ucSensitivityForOtherRobot[i];
			dTemp /= 50;	// default division
			InteractionResultData.ui32FinalDataFromOtherRobot[i] = (uint32_t)dTemp;
			if(dTemp > 255) dTemp = 255;
			InteractionResultData.ucFinalDataFromOtherRobot[i] = (uint8_t)dTemp;
		}
		// -----------------------------------------
	}
	// --------------------------------------------------------


	// indicate that new result is available
	InteractionResultData.ucNewInteractionResultAvailable 	= 1;
	// call the callback function
	InteractionNewResultAvailableCallback(InteractionResultData, IDPatternResultData);

	// debug
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);	//  debug on PC13 LOW

	return;
}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
// EEPROM
//static volatile EEPROMDataTypeDef			EEpromData[32];		/* data structure to hold EEPROM address and data pairs */
//static volatile EEPROMDataListTypeDef		EEpromDataList;		/* the data list that is used to operate the EEPROM in the main loop mode */
static void EEPROMInitialization(void)					// performs basic initialization settings with the EEPROM
{
	EEpromDataList.pEEDTList = EEpromData;	// link the data filed to the list structure for main loop EEPROM activities.
					// for the initialization section only the EEpromData[] buffer is used
	return;
}
//----------------------------------------------------------------------------------
static void EEPROMMainLoop(void)						// called from the main loop to handle EERPOM activities (mainly storing new-baseline values or new-sensitivity values)
{
	uint8_t ucLength, ucChannel;
	//-----------------------------

	if( (I2CEEPROMChekListStatusML() != LIST_IS_EMPTY) )	return;		// the list is not empty, must wait to empty up before launching a new read or write operation
	// list is empty => can launch another operation

	// --------- check if a new baseline calibration is available -------------
	if(m_ucEEPROMNewBaseLineDataIsAvailable)
	{
		m_ucEEPROMNewBaseLineDataIsAvailable = 0;	// data stored into the new list and prepared to be written into the EEPROM
		// ------ build up the list to be sent to the EERPOM --------
		ucLength = 0;
		// self reflection sensitivity
		// CH0
		ucChannel = 0;
		EEpromDataList.pEEDTList[ucLength].ui16EEPROMAddress		= EEPROM_INTERACTION_BASE_LINE_SELF_REFLECTION_CH0_ADDRESS;
		EEpromDataList.pEEDTList[ucLength].ui32EEPROMData			= BaseLineData.ui32CalculatedSelfBaseLineValues[ucChannel];
		ucLength ++;
		// CH1
		ucChannel = 1;
		EEpromDataList.pEEDTList[ucLength].ui16EEPROMAddress		= EEPROM_INTERACTION_BASE_LINE_SELF_REFLECTION_CH1_ADDRESS;
		EEpromDataList.pEEDTList[ucLength].ui32EEPROMData			= BaseLineData.ui32CalculatedSelfBaseLineValues[ucChannel];
		ucLength ++;
		// CH2
		ucChannel = 2;
		EEpromDataList.pEEDTList[ucLength].ui16EEPROMAddress		= EEPROM_INTERACTION_BASE_LINE_SELF_REFLECTION_CH2_ADDRESS;
		EEpromDataList.pEEDTList[ucLength].ui32EEPROMData			= BaseLineData.ui32CalculatedSelfBaseLineValues[ucChannel];
		ucLength ++;
		// CH3
		ucChannel = 3;
		EEpromDataList.pEEDTList[ucLength].ui16EEPROMAddress		= EEPROM_INTERACTION_BASE_LINE_SELF_REFLECTION_CH3_ADDRESS;
		EEpromDataList.pEEDTList[ucLength].ui32EEPROMData			= BaseLineData.ui32CalculatedSelfBaseLineValues[ucChannel];
		ucLength ++;
		// CH4
		ucChannel = 4;
		EEpromDataList.pEEDTList[ucLength].ui16EEPROMAddress		= EEPROM_INTERACTION_BASE_LINE_SELF_REFLECTION_CH4_ADDRESS;
		EEpromDataList.pEEDTList[ucLength].ui32EEPROMData			= BaseLineData.ui32CalculatedSelfBaseLineValues[ucChannel];
		ucLength ++;
		// CH5
		ucChannel = 5;
		EEpromDataList.pEEDTList[ucLength].ui16EEPROMAddress		= EEPROM_INTERACTION_BASE_LINE_SELF_REFLECTION_CH5_ADDRESS;
		EEpromDataList.pEEDTList[ucLength].ui32EEPROMData			= BaseLineData.ui32CalculatedSelfBaseLineValues[ucChannel];
		ucLength ++;

		// signal from another robot
		// CH0
		ucChannel = 0;
		EEpromDataList.pEEDTList[ucLength].ui16EEPROMAddress		= EEPROM_INTERACTION_BASE_LINE_ANOTHER_ROBOT_CH0_ADDRESS;
		EEpromDataList.pEEDTList[ucLength].ui32EEPROMData			= BaseLineData.ui32CalculatedBaseValuesFromOtherRobot[ucChannel];
		ucLength ++;
		// CH1
		ucChannel = 1;
		EEpromDataList.pEEDTList[ucLength].ui16EEPROMAddress		= EEPROM_INTERACTION_BASE_LINE_ANOTHER_ROBOT_CH1_ADDRESS;
		EEpromDataList.pEEDTList[ucLength].ui32EEPROMData			= BaseLineData.ui32CalculatedBaseValuesFromOtherRobot[ucChannel];
		ucLength ++;
		// CH2
		ucChannel = 2;
		EEpromDataList.pEEDTList[ucLength].ui16EEPROMAddress		= EEPROM_INTERACTION_BASE_LINE_ANOTHER_ROBOT_CH2_ADDRESS;
		EEpromDataList.pEEDTList[ucLength].ui32EEPROMData			= BaseLineData.ui32CalculatedBaseValuesFromOtherRobot[ucChannel];
		ucLength ++;
		// CH3
		ucChannel = 3;
		EEpromDataList.pEEDTList[ucLength].ui16EEPROMAddress		= EEPROM_INTERACTION_BASE_LINE_ANOTHER_ROBOT_CH3_ADDRESS;
		EEpromDataList.pEEDTList[ucLength].ui32EEPROMData			= BaseLineData.ui32CalculatedBaseValuesFromOtherRobot[ucChannel];
		ucLength ++;
		// CH4
		ucChannel = 4;
		EEpromDataList.pEEDTList[ucLength].ui16EEPROMAddress		= EEPROM_INTERACTION_BASE_LINE_ANOTHER_ROBOT_CH4_ADDRESS;
		EEpromDataList.pEEDTList[ucLength].ui32EEPROMData			= BaseLineData.ui32CalculatedBaseValuesFromOtherRobot[ucChannel];
		ucLength ++;
		// CH5
		ucChannel = 5;
		EEpromDataList.pEEDTList[ucLength].ui16EEPROMAddress		= EEPROM_INTERACTION_BASE_LINE_ANOTHER_ROBOT_CH5_ADDRESS;
		EEpromDataList.pEEDTList[ucLength].ui32EEPROMData			= BaseLineData.ui32CalculatedBaseValuesFromOtherRobot[ucChannel];
		ucLength ++;

		// add also the base line marker
		EEpromDataList.pEEDTList[ucLength].ui16EEPROMAddress		= EEPROM_INTERACTION_BASE_LINE_CALIBRATION_DONE_ADDRESS;
		EEpromDataList.pEEDTList[ucLength].ui32EEPROMData			= EEPROM_INTERACTION_BASE_LINE_CALIBRATION_MARKER;
		ucLength ++;

		// prepare the list control parameters and call the API for EEPROM write
		//EEpromDataList.ucCallbackToCall 	= INTERACTION_SECTION;
		EEpromDataList.ucReadWrite			= EEPROM_WRITE;
		EEpromDataList.ucSizeOfTheList		= ucLength;
		if(I2CEEPROMReadWriteListML(EEpromDataList, &EEPROMDoneCallback) != HAL_OK)
		{	// some error occurred
			#if defined(INTERACTION_DEBUG_MODE) && defined(INTERACTION_EEPROM_DEBUG)
			//#ifdef INTERACTION_DEBUG_MODE
			sprintf((char * )m_ucDebugBuffer,"EE_EEPROMMainLoop()__1\r\n");
			DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
			#endif
		}
		// ----------------------------------------------------------
		return;
	}
	// ------------------------------------------------------------------------

	// --------------- check for new sensitivity data -------------------------
	// check if new sensitivity setting are available and waiting to be written into the EEPROM
	ucLength=0;
	// self reflection sensitivity
	// CH0
	ucChannel = 0;
	if( (m_ucEEPROMNewSensitivityAvailable[0][ucChannel]) )
	{
		m_ucEEPROMNewSensitivityAvailable[0][ucChannel]				= 0;	// clear this flag
		EEpromDataList.pEEDTList[ucLength].ui16EEPROMAddress		= EEPROM_INTERACTION_SENSITIVITY_SELF_REFLECTION_CH0_ADDRESS;
		EEpromDataList.pEEDTList[ucLength].ui32EEPROMData			= SensitivityData.ucSensitivityForSelfReflection[ucChannel];
		ucLength ++;
	}
	// CH1
	ucChannel = 1;
	if( (m_ucEEPROMNewSensitivityAvailable[0][ucChannel]) )
	{
		m_ucEEPROMNewSensitivityAvailable[0][ucChannel]				= 0;	// clear this flag
		EEpromDataList.pEEDTList[ucLength].ui16EEPROMAddress		= EEPROM_INTERACTION_SENSITIVITY_SELF_REFLECTION_CH1_ADDRESS;
		EEpromDataList.pEEDTList[ucLength].ui32EEPROMData			= SensitivityData.ucSensitivityForSelfReflection[ucChannel];
		ucLength ++;
	}
	// CH2
	ucChannel = 2;
	if( (m_ucEEPROMNewSensitivityAvailable[0][ucChannel]) )
	{
		m_ucEEPROMNewSensitivityAvailable[0][ucChannel]				= 0;	// clear this flag
		EEpromDataList.pEEDTList[ucLength].ui16EEPROMAddress		= EEPROM_INTERACTION_SENSITIVITY_SELF_REFLECTION_CH2_ADDRESS;
		EEpromDataList.pEEDTList[ucLength].ui32EEPROMData			= SensitivityData.ucSensitivityForSelfReflection[ucChannel];
		ucLength ++;
	}
	// CH3
	ucChannel = 3;
	if( (m_ucEEPROMNewSensitivityAvailable[0][ucChannel]) )
	{
		m_ucEEPROMNewSensitivityAvailable[0][ucChannel]				= 0;	// clear this flag
		EEpromDataList.pEEDTList[ucLength].ui16EEPROMAddress		= EEPROM_INTERACTION_SENSITIVITY_SELF_REFLECTION_CH3_ADDRESS;
		EEpromDataList.pEEDTList[ucLength].ui32EEPROMData			= SensitivityData.ucSensitivityForSelfReflection[ucChannel];
		ucLength ++;
	}
	// CH4
	ucChannel = 4;
	if( (m_ucEEPROMNewSensitivityAvailable[0][ucChannel]) )
	{
		m_ucEEPROMNewSensitivityAvailable[0][ucChannel]				= 0;	// clear this flag
		EEpromDataList.pEEDTList[ucLength].ui16EEPROMAddress		= EEPROM_INTERACTION_SENSITIVITY_SELF_REFLECTION_CH4_ADDRESS;
		EEpromDataList.pEEDTList[ucLength].ui32EEPROMData			= SensitivityData.ucSensitivityForSelfReflection[ucChannel];
		ucLength ++;
	}
	// CH5
	ucChannel = 5;
	if( (m_ucEEPROMNewSensitivityAvailable[0][ucChannel]) )
	{
		m_ucEEPROMNewSensitivityAvailable[0][ucChannel]				= 0;	// clear this flag
		EEpromDataList.pEEDTList[ucLength].ui16EEPROMAddress		= EEPROM_INTERACTION_SENSITIVITY_SELF_REFLECTION_CH5_ADDRESS;
		EEpromDataList.pEEDTList[ucLength].ui32EEPROMData			= SensitivityData.ucSensitivityForSelfReflection[ucChannel];
		ucLength ++;
	}

	// signal from another robot
	// CH0
	ucChannel = 0;
	if( (m_ucEEPROMNewSensitivityAvailable[1][ucChannel]) )
	{
		m_ucEEPROMNewSensitivityAvailable[1][ucChannel]				= 0;	// clear this flag
		EEpromDataList.pEEDTList[ucLength].ui16EEPROMAddress		= EEPROM_INTERACTION_SENSITIVITY_ANOTHER_ROBOT_CH0_ADDRESS;
		EEpromDataList.pEEDTList[ucLength].ui32EEPROMData			= SensitivityData.ucSensitivityForOtherRobot[ucChannel];
		ucLength ++;
	}
	// CH1
	ucChannel = 1;
	if( (m_ucEEPROMNewSensitivityAvailable[1][ucChannel]) )
	{
		m_ucEEPROMNewSensitivityAvailable[1][ucChannel]				= 0;	// clear this flag
		EEpromDataList.pEEDTList[ucLength].ui16EEPROMAddress		= EEPROM_INTERACTION_SENSITIVITY_ANOTHER_ROBOT_CH1_ADDRESS;
		EEpromDataList.pEEDTList[ucLength].ui32EEPROMData			= SensitivityData.ucSensitivityForOtherRobot[ucChannel];
		ucLength ++;
	}
	// CH2
	ucChannel = 2;
	if( (m_ucEEPROMNewSensitivityAvailable[1][ucChannel]) )
	{
		m_ucEEPROMNewSensitivityAvailable[1][ucChannel]				= 0;	// clear this flag
		EEpromDataList.pEEDTList[ucLength].ui16EEPROMAddress		= EEPROM_INTERACTION_SENSITIVITY_ANOTHER_ROBOT_CH2_ADDRESS;
		EEpromDataList.pEEDTList[ucLength].ui32EEPROMData			= SensitivityData.ucSensitivityForOtherRobot[ucChannel];
		ucLength ++;
	}
	// CH3
	ucChannel = 3;
	if( (m_ucEEPROMNewSensitivityAvailable[1][ucChannel]) )
	{
		m_ucEEPROMNewSensitivityAvailable[1][ucChannel]				= 0;	// clear this flag
		EEpromDataList.pEEDTList[ucLength].ui16EEPROMAddress		= EEPROM_INTERACTION_SENSITIVITY_ANOTHER_ROBOT_CH3_ADDRESS;
		EEpromDataList.pEEDTList[ucLength].ui32EEPROMData			= SensitivityData.ucSensitivityForOtherRobot[ucChannel];
		ucLength ++;
	}
	// CH4
	ucChannel = 4;
	if( (m_ucEEPROMNewSensitivityAvailable[1][ucChannel]) )
	{
		m_ucEEPROMNewSensitivityAvailable[1][ucChannel]				= 0;	// clear this flag
		EEpromDataList.pEEDTList[ucLength].ui16EEPROMAddress		= EEPROM_INTERACTION_SENSITIVITY_ANOTHER_ROBOT_CH4_ADDRESS;
		EEpromDataList.pEEDTList[ucLength].ui32EEPROMData			= SensitivityData.ucSensitivityForOtherRobot[ucChannel];
		ucLength ++;
	}
	// CH5
	ucChannel = 5;
	if( (m_ucEEPROMNewSensitivityAvailable[1][ucChannel]) )
	{
		m_ucEEPROMNewSensitivityAvailable[1][ucChannel]				= 0;	// clear this flag
		EEpromDataList.pEEDTList[ucLength].ui16EEPROMAddress		= EEPROM_INTERACTION_SENSITIVITY_ANOTHER_ROBOT_CH5_ADDRESS;
		EEpromDataList.pEEDTList[ucLength].ui32EEPROMData			= SensitivityData.ucSensitivityForOtherRobot[ucChannel];
		ucLength ++;
	}

	// sensitivity marker write

	if(ucLength)
	{	// some change was detected => the list is not empty
		//EEpromDataList.ucCallbackToCall 	= INTERACTION_SECTION;
		EEpromDataList.ucReadWrite			= EEPROM_WRITE;
		EEpromDataList.ucSizeOfTheList		= ucLength;
		if(I2CEEPROMReadWriteListML(EEpromDataList, &EEPROMDoneCallback) != HAL_OK)
		{	// some error occurred
			#if defined(INTERACTION_DEBUG_MODE) && defined(INTERACTION_EEPROM_DEBUG)
			//#ifdef INTERACTION_DEBUG_MODE
			sprintf((char * )m_ucDebugBuffer,"EE_EEPROMMainLoop()__2\r\n");
			DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
			#endif
		}
	}
	// ------------------------------------------------------------------------

	return;
}
//----------------------------------------------------------------------------------
static void EEPROMDoneCallback(EEPROMDataListTypeDef EEResultList)		// callback function to be called @ EEPORM operation completed
{
	if( (EEResultList.ucReadWrite == EEPROM_READ) )
	{	// it was a read operation => data is available in the EEResultList
		//EEResultList.ucSizeOfTheList = nr of element read from the EEPROM
		//EEResultList.pEEDTList[0].ui32EEPROMData = the first data
		#if defined(INTERACTION_DEBUG_MODE) && defined(INTERACTION_EEPROM_DEBUG)
		//#ifdef INTERACTION_DEBUG_MODE
		sprintf((char * )m_ucDebugBuffer,"EE_EEPROMDoneCallback()__READ_: Len=%u, list=%u, Err=%u\r\n",EEResultList.ucSizeOfTheList,EEResultList.ucListIsPopulated,EEResultList.ucErrorCounter);
		DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
		return;
		#endif

	}
	else
	{	// it was a write
		#if defined(INTERACTION_DEBUG_MODE) && defined(INTERACTION_EEPROM_DEBUG)
		//#ifdef INTERACTION_DEBUG_MODE
		sprintf((char * )m_ucDebugBuffer,"EE_EEPROMDoneCallback()__WRITE_: Len=%u, list=%u, Err=%u\r\n",EEResultList.ucSizeOfTheList,EEResultList.ucListIsPopulated,EEResultList.ucErrorCounter);
		DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
		return;
		#endif
	}

	return;
}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
#ifdef INTERACTION_SHOW_ON_WS2812
//uint8_t  m_ucWS2812UpdateSkipCounter;					// used to skip updates
//#define INTERACTION_WS2812_UPDATE_SKIP_COUNTER_MAX		32		/* update every this time the WS2812 LEDs */
static void ShowInteractionOnWS2812(void)				// used to display the interaction result on the WS2812 LEDs
{
	// ----------- skip WS 2812 updates ----------------
	if( (m_ucWS2812UpdateSkipCounter) )
	{
		m_ucWS2812UpdateSkipCounter --;
		return;
	}
	m_ucWS2812UpdateSkipCounter = INTERACTION_WS2812_UPDATE_SKIP_COUNTER_MAX;
	//--------------------------------------------------

	// displays the self reflection with RED
	// displays the signal from other robots with GREEN
	// ------------- show on the WS2812 ----------------
	uint8_t uiDisplayChannel;
	uint8_t ucWS2812Intensity = 255;
	// channel 0
	uiDisplayChannel = 0;
	if( (InteractionResultData.ucFinalDataFromOtherRobot[uiDisplayChannel] == 0) )
	{	// if there is no signal detected from another robot => use the self reflected information => display RED, GREEN=0
		WS2812SetDisplay(DISPLY_SYSTEM_1, 1  , ucWS2812Intensity, InteractionResultData.ucFinalDataFromSelfReflection[uiDisplayChannel], 0, 0);
		WS2812SetDisplay(DISPLY_SYSTEM_1, 2  , ucWS2812Intensity, InteractionResultData.ucFinalDataFromSelfReflection[uiDisplayChannel], 0, 0);
		if (InteractionResultData.ucFinalDataFromSelfReflection[uiDisplayChannel] > rotation_threshold) {
			WheelRobotStop();
			WheelRobotGoPWMTmo(GOING_FORWARD, wheel_speed, 1);
		}
	}
	else
	{	// there is signal detected from another robot => RED=0 and display GREEN / BLUE
		if (InteractionResultData.ucFinalDataFromOtherRobot[uiDisplayChannel] > rotation_threshold) {
			WheelRobotStop();
			WheelRobotGoPWMTmo(GOING_BACKWARD, wheel_speed, 1);
		}
		if( (IDPatternResultData.ucDetectedRobotID[uiDisplayChannel] == 0) )
		{	// robot is too far away => GREEN
			WS2812SetDisplay(DISPLY_SYSTEM_1, 1  , ucWS2812Intensity, 0, InteractionResultData.ucFinalDataFromOtherRobot[uiDisplayChannel], 0);
			WS2812SetDisplay(DISPLY_SYSTEM_1, 2  , ucWS2812Intensity, 0, InteractionResultData.ucFinalDataFromOtherRobot[uiDisplayChannel], 0);
		}
		else
		{	// robot is close => can detect the ID also => BLUE
			WS2812SetDisplay(DISPLY_SYSTEM_1, 1  , ucWS2812Intensity, 0, 0, InteractionResultData.ucFinalDataFromOtherRobot[uiDisplayChannel]);
			WS2812SetDisplay(DISPLY_SYSTEM_1, 2  , ucWS2812Intensity, 0, 0, InteractionResultData.ucFinalDataFromOtherRobot[uiDisplayChannel]);
		}

	}

	// channel 1
	uiDisplayChannel = 1;
	if( (InteractionResultData.ucFinalDataFromOtherRobot[uiDisplayChannel] == 0) )
	{	// if there is no signal detected from another robot => use the self reflected information => display RED, GREEN=0
		WS2812SetDisplay(DISPLY_SYSTEM_1, 3  , ucWS2812Intensity, InteractionResultData.ucFinalDataFromSelfReflection[uiDisplayChannel], 0, 0);
		WS2812SetDisplay(DISPLY_SYSTEM_1, 4  , ucWS2812Intensity, InteractionResultData.ucFinalDataFromSelfReflection[uiDisplayChannel], 0, 0);
		if (InteractionResultData.ucFinalDataFromSelfReflection[uiDisplayChannel] > rotation_threshold) {
			WheelRobotStop();
			WheelRobotRotate(1);
		}
	}
	else
	{	// there is signal detected from another robot => RED=0 and display GREEN
		if (InteractionResultData.ucFinalDataFromOtherRobot[uiDisplayChannel] > rotation_threshold) {
			WheelRobotStop();
			WheelRobotRotate(1);
		}
		if( (IDPatternResultData.ucDetectedRobotID[uiDisplayChannel] == 0) )
		{	// robot is too far away => GREEN
			WS2812SetDisplay(DISPLY_SYSTEM_1, 3  , ucWS2812Intensity, 0, InteractionResultData.ucFinalDataFromOtherRobot[uiDisplayChannel], 0);
			WS2812SetDisplay(DISPLY_SYSTEM_1, 4  , ucWS2812Intensity, 0, InteractionResultData.ucFinalDataFromOtherRobot[uiDisplayChannel], 0);
		}
		else
		{	// robot is close => can detect the ID also => BLUE
			WS2812SetDisplay(DISPLY_SYSTEM_1, 3  , ucWS2812Intensity, 0, 0, InteractionResultData.ucFinalDataFromOtherRobot[uiDisplayChannel]);
			WS2812SetDisplay(DISPLY_SYSTEM_1, 4  , ucWS2812Intensity, 0, 0, InteractionResultData.ucFinalDataFromOtherRobot[uiDisplayChannel]);
		}
	}

	// channel 2
	uiDisplayChannel = 2;
	if( (InteractionResultData.ucFinalDataFromOtherRobot[uiDisplayChannel] == 0) )
	{	// if there is no signal detected from another robot => use the self reflected information => display RED, GREEN=0
		WS2812SetDisplay(DISPLY_SYSTEM_1, 5  , ucWS2812Intensity, InteractionResultData.ucFinalDataFromSelfReflection[uiDisplayChannel], 0, 0);
		WS2812SetDisplay(DISPLY_SYSTEM_1, 6  , ucWS2812Intensity, InteractionResultData.ucFinalDataFromSelfReflection[uiDisplayChannel], 0, 0);
		if (InteractionResultData.ucFinalDataFromSelfReflection[uiDisplayChannel] > rotation_threshold) {
			WheelRobotStop();
			WheelRobotRotate(-1);
		}
	}
	else
	{	// there is signal detected from another robot => RED=0 and display GREEN
		if (InteractionResultData.ucFinalDataFromOtherRobot[uiDisplayChannel] > rotation_threshold) {
			WheelRobotStop();
			WheelRobotRotate(-1);
		}
		if( (IDPatternResultData.ucDetectedRobotID[uiDisplayChannel] == 0) )
		{	// robot is too far away => GREEN
			WS2812SetDisplay(DISPLY_SYSTEM_1, 5  , ucWS2812Intensity, 0, InteractionResultData.ucFinalDataFromOtherRobot[uiDisplayChannel], 0);
			WS2812SetDisplay(DISPLY_SYSTEM_1, 6  , ucWS2812Intensity, 0, InteractionResultData.ucFinalDataFromOtherRobot[uiDisplayChannel], 0);
		}
		else
		{	// robot is close => can detect the ID also => BLUE
			WS2812SetDisplay(DISPLY_SYSTEM_1, 5  , ucWS2812Intensity, 0, 0, InteractionResultData.ucFinalDataFromOtherRobot[uiDisplayChannel]);
			WS2812SetDisplay(DISPLY_SYSTEM_1, 6  , ucWS2812Intensity, 0, 0, InteractionResultData.ucFinalDataFromOtherRobot[uiDisplayChannel]);
		}
	}

	// channel 3
	uiDisplayChannel = 3;
	if( (InteractionResultData.ucFinalDataFromOtherRobot[uiDisplayChannel] == 0) )
	{	// if there is no signal detected from another robot => use the self reflected information => display RED, GREEN=0
		WS2812SetDisplay(DISPLY_SYSTEM_1, 7  , ucWS2812Intensity, InteractionResultData.ucFinalDataFromSelfReflection[uiDisplayChannel], 0, 0);
		WS2812SetDisplay(DISPLY_SYSTEM_1, 8  , ucWS2812Intensity, InteractionResultData.ucFinalDataFromSelfReflection[uiDisplayChannel], 0, 0);
		if (InteractionResultData.ucFinalDataFromSelfReflection[uiDisplayChannel] > rotation_threshold) {
			WheelRobotStop();
			WheelRobotGoPWMTmo(GOING_BACKWARD, wheel_speed, 1);
		}
	}
	else
	{	// there is signal detected from another robot => RED=0 and display GREEN
		if (InteractionResultData.ucFinalDataFromOtherRobot[uiDisplayChannel] > rotation_threshold) {
			WheelRobotStop();
			WheelRobotGoPWMTmo(GOING_FORWARD, wheel_speed, 1);
		}
		if( (IDPatternResultData.ucDetectedRobotID[uiDisplayChannel] == 0) )
		{	// robot is too far away => GREEN
			WS2812SetDisplay(DISPLY_SYSTEM_1, 7  , ucWS2812Intensity, 0, InteractionResultData.ucFinalDataFromOtherRobot[uiDisplayChannel], 0);
			WS2812SetDisplay(DISPLY_SYSTEM_1, 8  , ucWS2812Intensity, 0, InteractionResultData.ucFinalDataFromOtherRobot[uiDisplayChannel], 0);
		}
		else
		{	// robot is close => can detect the ID also => BLUE
			WS2812SetDisplay(DISPLY_SYSTEM_1, 7  , ucWS2812Intensity, 0, 0, InteractionResultData.ucFinalDataFromOtherRobot[uiDisplayChannel]);
			WS2812SetDisplay(DISPLY_SYSTEM_1, 8  , ucWS2812Intensity, 0, 0, InteractionResultData.ucFinalDataFromOtherRobot[uiDisplayChannel]);
		}
	}

	// channel 4
	uiDisplayChannel = 4;
	if( (InteractionResultData.ucFinalDataFromOtherRobot[uiDisplayChannel] == 0) )
	{	// if there is no signal detected from another robot => use the self reflected information => display RED, GREEN=0
		WS2812SetDisplay(DISPLY_SYSTEM_1, 9  , ucWS2812Intensity, InteractionResultData.ucFinalDataFromSelfReflection[uiDisplayChannel], 0, 0);
		WS2812SetDisplay(DISPLY_SYSTEM_1, 10 , ucWS2812Intensity, InteractionResultData.ucFinalDataFromSelfReflection[uiDisplayChannel], 0, 0);
		if (InteractionResultData.ucFinalDataFromSelfReflection[uiDisplayChannel] > rotation_threshold) {
			WheelRobotStop();
			WheelRobotRotate(1);
		}
	}
	else
	{	// there is signal detected from another robot => RED=0 and display GREEN
		if (InteractionResultData.ucFinalDataFromOtherRobot[uiDisplayChannel] > rotation_threshold) {
			WheelRobotStop();
			WheelRobotRotate(1);
		}
		if( (IDPatternResultData.ucDetectedRobotID[uiDisplayChannel] == 0) )
		{	// robot is too far away => GREEN
			WS2812SetDisplay(DISPLY_SYSTEM_1, 9  , ucWS2812Intensity, 0, InteractionResultData.ucFinalDataFromOtherRobot[uiDisplayChannel], 0);
			WS2812SetDisplay(DISPLY_SYSTEM_1, 10 , ucWS2812Intensity, 0, InteractionResultData.ucFinalDataFromOtherRobot[uiDisplayChannel], 0);
		}
		else
		{	// robot is close => can detect the ID also => BLUE
			WS2812SetDisplay(DISPLY_SYSTEM_1, 9  , ucWS2812Intensity, 0, 0, InteractionResultData.ucFinalDataFromOtherRobot[uiDisplayChannel]);
			WS2812SetDisplay(DISPLY_SYSTEM_1, 10 , ucWS2812Intensity, 0, 0, InteractionResultData.ucFinalDataFromOtherRobot[uiDisplayChannel]);
		}
	}

	// channel 5
	uiDisplayChannel = 5;
	if( (InteractionResultData.ucFinalDataFromOtherRobot[uiDisplayChannel] == 0) )
	{	// if there is no signal detected from another robot => use the self reflected information => display RED, GREEN=0
		WS2812SetDisplay(DISPLY_SYSTEM_1, 11 , ucWS2812Intensity, InteractionResultData.ucFinalDataFromSelfReflection[uiDisplayChannel], 0, 0);
		WS2812SetDisplay(DISPLY_SYSTEM_1, 12 , ucWS2812Intensity, InteractionResultData.ucFinalDataFromSelfReflection[uiDisplayChannel], 0, 0);
		if (InteractionResultData.ucFinalDataFromSelfReflection[uiDisplayChannel] > rotation_threshold) {
			WheelRobotStop();
			WheelRobotRotate(-1);
		}
	}
	else
	{	// there is signal detected from another robot => RED=0 and display GREEN
		if (InteractionResultData.ucFinalDataFromOtherRobot[uiDisplayChannel] > rotation_threshold) {
			WheelRobotStop();
			WheelRobotRotate(-1);
		}
		if( (IDPatternResultData.ucDetectedRobotID[uiDisplayChannel] == 0) )
		{	// robot is too far away => GREEN
			WS2812SetDisplay(DISPLY_SYSTEM_1, 11 , ucWS2812Intensity, 0, InteractionResultData.ucFinalDataFromOtherRobot[uiDisplayChannel], 0);
			WS2812SetDisplay(DISPLY_SYSTEM_1, 12 , ucWS2812Intensity, 0, InteractionResultData.ucFinalDataFromOtherRobot[uiDisplayChannel], 0);
		}
		else
		{	// robot is close => can detect the ID also => BLUE
			WS2812SetDisplay(DISPLY_SYSTEM_1, 11 , ucWS2812Intensity, 0, 0, InteractionResultData.ucFinalDataFromOtherRobot[uiDisplayChannel]);
			WS2812SetDisplay(DISPLY_SYSTEM_1, 12 , ucWS2812Intensity, 0, 0, InteractionResultData.ucFinalDataFromOtherRobot[uiDisplayChannel]);
		}
	}


	WS2812ShowDisplay(DISPLY_SYSTEM_1,50);		// show DISPLY_SYSTEM_1 and do not refresh for 50mS
	// -------------------------------------------------

	return;
}
#endif
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
// ID Pattern
//----------------------------------------------------------------------------------
static void IDPatternInitialization(void)				// called in the initialization section
{
	IDPatternGenerationData.ucRobotAddress = 0;
	return;
}
//----------------------------------------------------------------------------------
static void IDPatternMainLoop(void)						// called in the main loop
{
	uint8_t i;

	// -------check if the address of the robot has changed -------
	if( (IDPatternGenerationData.ucRobotAddress != m_ucRobotAddress) )
	{
		IDPatternGenerationData.ucRobotAddress = m_ucRobotAddress;
		IDPatternAddressUpdate();
	}
	// ------------------------------------------------------------

	// ---------------- check channels status ---------------------
	for(i=0;i<6;i++)
	{
		if( (IDPatternDetectionData[i].ucDecodingState == INTERACTION_ID_RX_FINALE_STAGE_VALUE) )
		{	// some data was received in the i channel
			IDPatternResultData.ucDetectedRobotID[i] 	= IDPatternDetectionData[i].ucReceivedData;
			IDPatternResultData.ucNewResultAvailable[i]	= 1;	// new result is available (only useful if implement som callback function for a channel state)
			// reset channel
			IDPatternResetChannel(i);
		}
		if( (IDPatternDetectionData[i].ucDecodingState == INTARACTION_ID_RX_STAGE_ERROR) )
		{	// some error detected on the channel
			// reset channel
			IDPatternResetChannel(i);
		}
	}
	// ------------------------------------------------------------


	// ------------------------------------------------------------
	// ------------------------------------------------------------
	// ------------------------------------------------------------

	return;
}
//----------------------------------------------------------------------------------
static void IDPatternAddressUpdate(void)				// called to update the Pattern Generation Buffer "ucPatternBuffer" whenever the Robot's address has changed
{	// this function updates the IR Pattern Buffer "ucPatternBuffer" based on the address of the robot
	// this function assumes that the current address of the robot is already updated into the "IDPatternGenerationData.ucRobotAddress"
	//---------------------------------------
	// the structure of the IR Pattern buffer "ucPatternBuffer" is the following:
	// - value of 0		=> 100ms
	// - value of 1		=> 200ms
	// - value of 2		=> 300ms
	// ---- header of the buffer
	// -- ucPatternBuffer[0]						= 0		// always => 100ms pulse
	// -- ucPatternBuffer[1]						= 0		// always => 100ms pulse
	// -- ucPatternBuffer[2]						= 0		// always => 100ms pulse
	// -- ucPatternBuffer[3]						= 0		// always => 100ms pulse
	// ----- data in the buffer
	// --- byte 0 = ucRobotAddress & (1<<5) (MSB)
	// -- ucRobotAddress[04], ucRobotAddress[05]	= 		// ==1 if the (ucRobotAddress & (1<<5) == LOW), ==2 if the (ucRobotAddress & (1<<5) == HIGH)
	// --- byte 1 = ucRobotAddress & (1<<4)
	// -- ucRobotAddress[06], ucRobotAddress[07]	= 		// ==1 if the (ucRobotAddress & (1<<4) == LOW), ==2 if the (ucRobotAddress & (1<<4) == HIGH)
	// --- byte 2 = ucRobotAddress & (1<<3)
	// -- ucRobotAddress[08], ucRobotAddress[09]	= 		// ==1 if the (ucRobotAddress & (1<<3) == LOW), ==2 if the (ucRobotAddress & (1<<3) == HIGH)
	// --- byte 3 = ucRobotAddress & (1<<2)
	// -- ucRobotAddress[10], ucRobotAddress[11]	= 		// ==1 if the (ucRobotAddress & (1<<2) == LOW), ==2 if the (ucRobotAddress & (1<<2) == HIGH)
	// --- byte 4 = ucRobotAddress & (1<<1)
	// -- ucRobotAddress[12], ucRobotAddress[13]	= 		// ==1 if the (ucRobotAddress & (1<<1) == LOW), ==2 if the (ucRobotAddress & (1<<1) == HIGH)
	// --- byte 5 = ucRobotAddress & (1<<0) (LSB)
	// -- ucRobotAddress[14], ucRobotAddress[15]	= 		// ==1 if the (ucRobotAddress & (1<<0) == LOW), ==2 if the (ucRobotAddress & (1<<0) == HIGH)
	// -----
	// --- byte 0 = ucRobotAddress & (1<<5) (MSB)
	// -- ucRobotAddress[16], ucRobotAddress[17]	= 		// ==2 if the (ucRobotAddress & (1<<5) == LOW), ==1 if the (ucRobotAddress & (1<<5) == HIGH)
	// --- byte 1 = ucRobotAddress & (1<<4)
	// -- ucRobotAddress[18], ucRobotAddress[19]	= 		// ==2 if the (ucRobotAddress & (1<<4) == LOW), ==1 if the (ucRobotAddress & (1<<4) == HIGH)
	// --- byte 2 = ucRobotAddress & (1<<3)
	// -- ucRobotAddress[20], ucRobotAddress[21]	= 		// ==2 if the (ucRobotAddress & (1<<3) == LOW), ==1 if the (ucRobotAddress & (1<<3) == HIGH)
	// --- byte 3 = ucRobotAddress & (1<<2)
	// -- ucRobotAddress[22], ucRobotAddress[23]	= 		// ==2 if the (ucRobotAddress & (1<<2) == LOW), ==1 if the (ucRobotAddress & (1<<2) == HIGH)
	// --- byte 4 = ucRobotAddress & (1<<1)
	// -- ucRobotAddress[24], ucRobotAddress[25]	= 		// ==2 if the (ucRobotAddress & (1<<1) == LOW), ==1 if the (ucRobotAddress & (1<<1) == HIGH)
	// --- byte 5 = ucRobotAddress & (1<<0) (LSB)
	// -- ucRobotAddress[26], ucRobotAddress[27]	= 		// ==2 if the (ucRobotAddress & (1<<0) == LOW), ==1 if the (ucRobotAddress & (1<<0) == HIGH)
	//---------------------------------------

	uint8_t i,j;

	for(i=4,j=5; i<15; i+=2,j--)
	{	// 4,6,8,10,12,14 -> +12 -> 16,18,20,22,24,26

		if( (IDPatternGenerationData.ucRobotAddress & (0x01U<<j)) )
		{
			IDPatternGenerationData.ucPatternBuffer[i+0] 	= 2;
			IDPatternGenerationData.ucPatternBuffer[i+1] 	= 2;
			// +12
			IDPatternGenerationData.ucPatternBuffer[i+12] 	= 1;
			IDPatternGenerationData.ucPatternBuffer[i+12+1] = 1;
		}
		else
		{
			IDPatternGenerationData.ucPatternBuffer[i+0] 	= 1;
			IDPatternGenerationData.ucPatternBuffer[i+1] 	= 1;
			// +12
			IDPatternGenerationData.ucPatternBuffer[i+12] 	= 2;
			IDPatternGenerationData.ucPatternBuffer[i+12+1] = 2;
		}
	}

	#define INTERACTION_PRINT_IDBUFFER
	#if defined(INTERACTION_PRINT_IDBUFFER) && defined(INTERACTION_DEBUG_MODE)
	if( (IDPatternGenerationData.ucRobotAddress) )	// only print if not ZERO
	{
		sprintf((char * )m_ucDebugBuffer,"ID: ADD=%u\t i4=%u\t i6=%u\t i8=%u\t i10=%u\t i12=%u\t i14=%u\t\t "
														"i16=%u\t i18=%u\t i20=%u\t i22=%u\t i24=%u\t i26=%u\t\r\n",IDPatternGenerationData.ucRobotAddress,
																														IDPatternGenerationData.ucPatternBuffer[4],
																														IDPatternGenerationData.ucPatternBuffer[6],
																														IDPatternGenerationData.ucPatternBuffer[8],
																														IDPatternGenerationData.ucPatternBuffer[10],
																														IDPatternGenerationData.ucPatternBuffer[12],
																														IDPatternGenerationData.ucPatternBuffer[14],
																															IDPatternGenerationData.ucPatternBuffer[4+12],
																															IDPatternGenerationData.ucPatternBuffer[6+12],
																															IDPatternGenerationData.ucPatternBuffer[8+12],
																															IDPatternGenerationData.ucPatternBuffer[10+12],
																															IDPatternGenerationData.ucPatternBuffer[12+12],
																															IDPatternGenerationData.ucPatternBuffer[14+12]);

		while( DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer)) != DEBUG_TX_OK);
	}
	#endif


	return;
}
//----------------------------------------------------------------------------------
static void IDPatternTimer1KHZISR(void)					// called from the 1KHZ timer ISR
{
	uint8_t ucChannel;


	// ----------------- check all channels -----------------
	for(ucChannel=0; ucChannel<6; ucChannel++)
	{
		if( (IDPatternDetectionData[ucChannel].ucTimeoutCounterMS) )
		{
			IDPatternDetectionData[ucChannel].ucTimeoutCounterMS --;
			if( (IDPatternDetectionData[ucChannel].ucTimeoutCounterMS == 0) )
			{	// timeout
				IDPatternDetectionData[ucChannel].ucDecodingState = INTARACTION_ID_RX_STAGE_ERROR;
			}
		}
		if( (IDPatternDetectionData[ucChannel].ui16ExpirationCounterMS) )
		{
			IDPatternDetectionData[ucChannel].ui16ExpirationCounterMS --;
			if( (IDPatternDetectionData[ucChannel].ui16ExpirationCounterMS == 0) )
			{	// expired
				IDPatternDetectionData[ucChannel].ucReceivedData = 0;			// robot with ID0 dose not exist
				IDPatternDetectionData[ucChannel].ucReceivedInvertedData = 0;
				IDPatternDetectionData[ucChannel].ucDecodingState = INTERACTION_ID_RX_FINALE_STAGE_VALUE;	// is the same as a new result is representing that the robot is no longer present
			}
		}
	}
	// ----------------------------------------------------



	return;
}
//----------------------------------------------------------------------------------
static void IDPatternChannelISR(uint8_t ucChannel)		// used to be called from the channel line change interrupt
{
	uint32_t	ui32TimeNow = __HAL_TIM_GET_COUNTER(&htim5);
	uint32_t	ui32Temp;
	//uint8_t		ucChannel	= 0;	// for this ISR
	uint8_t		ucRxSymbol;

	// ------ check if reception must proceed or it is in a final state -------
	if( (IDPatternDetectionData[ucChannel].ucDecodingState  == INTERACTION_ID_RX_FINALE_STAGE_VALUE) )
	{
		IDPatternDetectionData[ucChannel].ui32LastInterruptTimeUS = ui32TimeNow;
		return;		// data was received but needs to be processed in the main loop
	}
	if( (IDPatternDetectionData[ucChannel].ucDecodingState  == INTARACTION_ID_RX_STAGE_ERROR) )
	{
		IDPatternDetectionData[ucChannel].ui32LastInterruptTimeUS = ui32TimeNow;
		return;		// some error occurred. the channel variable structure must be cleared before another reception can begin
	}
	// ------------------------------------------------------------------------

	// --------------------- detect the symbol type ---------------------------
	if( (ui32TimeNow > IDPatternDetectionData[ucChannel].ui32LastInterruptTimeUS) )		ui32Temp = (ui32TimeNow - IDPatternDetectionData[ucChannel].ui32LastInterruptTimeUS);
	else 																				ui32Temp = ( (0xFFFFFFFF - IDPatternDetectionData[ucChannel].ui32LastInterruptTimeUS) + ui32TimeNow );
	IDPatternDetectionData[ucChannel].ui32LastInterruptTimeUS = ui32TimeNow;
	if( (ui32Temp > (100 - INTERACTION_ID_RX_TIME_TOLERANCE_US)) && (ui32Temp < (100 + INTERACTION_ID_RX_TIME_TOLERANCE_US)) )
	{	// 0 type (header) = 100uS
		ucRxSymbol = 0;
	}
	else if( (ui32Temp > (200 - INTERACTION_ID_RX_TIME_TOLERANCE_US)) && (ui32Temp < (200 + INTERACTION_ID_RX_TIME_TOLERANCE_US)) )
	{	// 1 type = logical LOW = 200uS
		ucRxSymbol = 1;
	}
	else if( (ui32Temp > (300 - INTERACTION_ID_RX_TIME_TOLERANCE_US)) && (ui32Temp < (300 + INTERACTION_ID_RX_TIME_TOLERANCE_US)) )
	{	// 2 type = logical HIGH = 300uS
		ucRxSymbol = 2;
	}
	else ucRxSymbol = 3;		// error
	// ------------------------------------------------------------------------

	// ------------------ set the timeout watch-dog ---------------------------
	if( (IDPatternDetectionData[ucChannel].ucDecodingState > 0) )
	{	// some reception is in progress
		IDPatternDetectionData[ucChannel].ucTimeoutCounterMS = INTERACTION_ID_RX_TIMEOUT_MS;	// reset the timeout
	}
	// ------------------------------------------------------------------------

	// -------------- check for symbol error relevance ------------------------
	if( (ucRxSymbol > 2) )
	{	// symbol error occurred
		if( (IDPatternDetectionData[ucChannel].ucDecodingState > 0) )
		{	// the symbol error occurred while the communication was in progress => communication error
			IDPatternDetectionData[ucChannel].ucDecodingState = INTARACTION_ID_RX_STAGE_ERROR;
		}
		return;
	}
	// ------------------------------------------------------------------------

	// ----------------- check for the header section -------------------------
	if( (IDPatternDetectionData[ucChannel].ucDecodingState < 4) )
	{	// 0->1, 1->2, 2->3, 3->4 are the header section representing 4 consecutive 100uS symbols
		if( (ucRxSymbol == 0 ) )
		{	// correct symbol was received
			IDPatternDetectionData[ucChannel].ucDecodingState ++;
			IDPatternDetectionData[ucChannel].ucReferenceSymbolExist = 0;	// just make sure that once the data reception starts it will start with the start symbol
			IDPatternDetectionData[ucChannel].ucReceptionBufferIndex = 0;	// just make sure that the buffer will be populated starting from index 0
		}
		else
		{	// incorrect symbol was received
			IDPatternDetectionData[ucChannel].ucDecodingState = 0;		// do not consider an error just searching for the start of the frame
		}
		return;
	}
	// ------------------------------------------------------------------------

	// ------------------- detect symbol type error ---------------------------
	// at this point only ucRxSymbol=1 and ucRxSymbol=2 are allowed
	if( (ucRxSymbol != 1) && (ucRxSymbol != 2) )
	{	// symbol error
		IDPatternDetectionData[ucChannel].ucDecodingState = INTARACTION_ID_RX_STAGE_ERROR;
		return;
	}
	// ------------------------------------------------------------------------

	// ------------------- this is the data section ---------------------------
	if( (IDPatternDetectionData[ucChannel].ucReferenceSymbolExist == 0) )
	{	// this is the first from the pair. so just store it
		IDPatternDetectionData[ucChannel].ucReferenceSymbol = ucRxSymbol;
		IDPatternDetectionData[ucChannel].ucReferenceSymbolExist = 1;
	}
	else
	{	// this is data => collect the data if it is right
		IDPatternDetectionData[ucChannel].ucReferenceSymbolExist = 0;		// make sure that the next symbol is again the reference symbol
		if( (ucRxSymbol == IDPatternDetectionData[ucChannel].ucReferenceSymbol) )
		{	// the received symbol matches the reference symbol => all seems to be ok.
			if( (ucRxSymbol == 2) )
			{	// logical HIGH
				if( (IDPatternDetectionData[ucChannel].ucReceptionBufferIndex < 6) )
				{	// receiving the normal data (not the inverted data): 0,1,2,3,4,5
					IDPatternDetectionData[ucChannel].ucReceivedData 			|= (1 << (5-IDPatternDetectionData[ucChannel].ucReceptionBufferIndex) );
				}
				else
				{	// receiving the inverted data: 6,7,8,9,10,11
					IDPatternDetectionData[ucChannel].ucReceivedInvertedData 	|= (1 << (11-IDPatternDetectionData[ucChannel].ucReceptionBufferIndex) );
				}
			}
			IDPatternDetectionData[ucChannel].ucReceptionBufferIndex ++;
			if( (IDPatternDetectionData[ucChannel].ucReceptionBufferIndex  >= 12) )
			{	// end of the reception
				if( ((IDPatternDetectionData[ucChannel].ucReceivedData + IDPatternDetectionData[ucChannel].ucReceivedInvertedData) == 63) )
				{	// the two data complement each other => the sum is 0x0011 1111 as expected
					IDPatternDetectionData[ucChannel].ucDecodingState = INTERACTION_ID_RX_FINALE_STAGE_VALUE;	// just process the data from the buffer in the main
					IDPatternDetectionData[ucChannel].ui16ExpirationCounterMS = INTERACTION_ID_RX_EXPIRATION_TIMEOUT_MS;
				}
				else
				{	// some data reception error occurred
					IDPatternDetectionData[ucChannel].ucDecodingState = INTARACTION_ID_RX_STAGE_ERROR;
				}
				return;
			}
		}
		else
		{	// the next symbol dose not matches the reference symbol => error
			IDPatternDetectionData[ucChannel].ucDecodingState = INTARACTION_ID_RX_STAGE_ERROR;
		}
	}
	return;
	// ------------------------------------------------------------------------
	return;
}
//----------------------------------------------------------------------------------
static void IDPatternResetChannel(uint8_t ucChannel)			// clear the relevant parameters from a channel and prepare the channel for the next reception
{
	IDPatternDetectionData[ucChannel].ucDecodingState = 0;
	IDPatternDetectionData[ucChannel].ucReferenceSymbolExist = 0;
	IDPatternDetectionData[ucChannel].ucReceptionBufferIndex = 0;
	IDPatternDetectionData[ucChannel].ucTimeoutCounterMS = 0;
	IDPatternDetectionData[ucChannel].ucReceivedData = 0;
	IDPatternDetectionData[ucChannel].ucReceivedInvertedData = 0;

	return;
}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------

// debug
#ifdef INTERACTION_DEBUG_MODE
//----------------------------------------------------------------------------------
//volatile uint8_t m_ucEnablePrintRaw = 0;	// if is set to 1 the raw values are printed, use serial commands to enable / disable raw printing 'r'=enable raw printing
static void DebugPrintRaw(void)			// used to print the raw values after acquisition
{
	sprintf((char * )m_ucDebugBuffer,"RAW: CH=%u\t Robot=%lu\t  Self=%lu\t  Time=%lu\t  Rnd=%lu\r\n",m_ucDebugTemp,
																					MeasurementResult.ui32SignalFromAnotherRobot[m_ucDebugTemp],
																					MeasurementResult.ui32SignalFromSelfReflection[m_ucDebugTemp],
																					MeasurementResult.ui32AcquisitionDurationUS,
																					MeasurementResult.ui32AcquisitionRoundCounter);

	DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
	return;
}
//----------------------------------------------------------------------------------
//volatile uint8_t	m_ucPrintSpikeRemovedData					= 0;	// if is set to 1 the spike filtered data is printed over the serial debug line. to activate this press 't' key
#ifdef INTERACTION_USES_SPIKE_FILTER
static void DebugPrintSpikeFilteredData(void)	// used to print the spike filtered data
{
	// spike removed data
	sprintf((char * )m_ucDebugBuffer,"SPK: S0=%lu\t S1=%lu\t S2=%lu\t S3=%lu\t S4=%lu\t S5=%lu\t\t "
												"O0=%lu\t O1=%lu\t O2=%lu\t O3=%lu\t O4=%lu\t O5=%lu\t\r\n",MeasurementResult.ui32SignalFromSelfReflection[0],
																											MeasurementResult.ui32SignalFromSelfReflection[1],
																											MeasurementResult.ui32SignalFromSelfReflection[2],
																											MeasurementResult.ui32SignalFromSelfReflection[3],
																											MeasurementResult.ui32SignalFromSelfReflection[4],
																											MeasurementResult.ui32SignalFromSelfReflection[5],
																												MeasurementResult.ui32SignalFromAnotherRobot[0],
																												MeasurementResult.ui32SignalFromAnotherRobot[1],
																												MeasurementResult.ui32SignalFromAnotherRobot[2],
																												MeasurementResult.ui32SignalFromAnotherRobot[3],
																												MeasurementResult.ui32SignalFromAnotherRobot[4],
																												MeasurementResult.ui32SignalFromAnotherRobot[5]);

	DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
	return;
}
#endif
//----------------------------------------------------------------------------------
//volatile uint8_t	m_ucPrintMovingAverageData					= 0;	// if this is set to 1 =>the moving averaged data is printed. to activate this print press the 'y' key
#ifdef INTARACTION_USES_MOVING_AVERAGING_FILTER
static void DebugMovingAveragData(void)		// used to print the moving average data
{
	// spike removed data
	sprintf((char * )m_ucDebugBuffer,"MAF: S0=%lu\t S1=%lu\t S2=%lu\t S3=%lu\t S4=%lu\t S5=%lu\t\t "
													"O0=%lu\t O1=%lu\t O2=%lu\t O3=%lu\t O4=%lu\t O5=%lu\t\r\n",MeasurementResult.ui32SignalFromSelfReflection[0],
																												MeasurementResult.ui32SignalFromSelfReflection[1],
																												MeasurementResult.ui32SignalFromSelfReflection[2],
																												MeasurementResult.ui32SignalFromSelfReflection[3],
																												MeasurementResult.ui32SignalFromSelfReflection[4],
																												MeasurementResult.ui32SignalFromSelfReflection[5],
																													MeasurementResult.ui32SignalFromAnotherRobot[0],
																													MeasurementResult.ui32SignalFromAnotherRobot[1],
																													MeasurementResult.ui32SignalFromAnotherRobot[2],
																													MeasurementResult.ui32SignalFromAnotherRobot[3],
																													MeasurementResult.ui32SignalFromAnotherRobot[4],
																													MeasurementResult.ui32SignalFromAnotherRobot[5]);

	DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
	return;
}
#endif
//----------------------------------------------------------------------------------
//volatile uint8_t	m_ucPrintBaseLineCalibrationData			= 0;	// if set to 1 it will print the base line calibration once and it will be reseted to 0.use 'u' key to print once the base line value
static void DebugPrintBaseLine(void)		// used to print the base line calibration data
{

	sprintf((char * )m_ucDebugBuffer,"BLC: BLD=%u\r\n"
			"S0=%lu\t S1=%lu\t S2=%lu\t S3=%lu\t S4=%lu\t S5=%lu\r\n"
			"O0=%lu\t O1=%lu\t O2=%lu\t O3=%lu\t O4=%lu\t O5=%lu\r\n ",BaseLineData.ucBaseLineCalibrationDone,
																			BaseLineData.ui32CalculatedSelfBaseLineValues[0],
																			BaseLineData.ui32CalculatedSelfBaseLineValues[1],
																			BaseLineData.ui32CalculatedSelfBaseLineValues[2],
																			BaseLineData.ui32CalculatedSelfBaseLineValues[3],
																			BaseLineData.ui32CalculatedSelfBaseLineValues[4],
																			BaseLineData.ui32CalculatedSelfBaseLineValues[5],
																				BaseLineData.ui32CalculatedBaseValuesFromOtherRobot[0],
																				BaseLineData.ui32CalculatedBaseValuesFromOtherRobot[1],
																				BaseLineData.ui32CalculatedBaseValuesFromOtherRobot[2],
																				BaseLineData.ui32CalculatedBaseValuesFromOtherRobot[3],
																				BaseLineData.ui32CalculatedBaseValuesFromOtherRobot[4],
																				BaseLineData.ui32CalculatedBaseValuesFromOtherRobot[5]);

	DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
	return;
}
//----------------------------------------------------------------------------------
//volatile uint8_t 	m_ucPrintCorrectedData						= 0;	// if set => prints continuously the corrected data, use 'i' to turn on.
static void DebugPrintCorrectedData(void)		// used to print the corrected measurement data
{
	// Corrected Data
	sprintf((char * )m_ucDebugBuffer,"CD: S0=%lu\t S1=%lu\t S2=%lu\t S3=%lu\t S4=%lu\t S5=%lu\t\t "
											"O0=%lu\t O1=%lu\t O2=%lu\t O3=%lu\t O4=%lu\t O5=%lu\t\r\n",InteractionResultData.ui32CorrectedDataFromSelfReflection[0],
																										InteractionResultData.ui32CorrectedDataFromSelfReflection[1],
																										InteractionResultData.ui32CorrectedDataFromSelfReflection[2],
																										InteractionResultData.ui32CorrectedDataFromSelfReflection[3],
																										InteractionResultData.ui32CorrectedDataFromSelfReflection[4],
																										InteractionResultData.ui32CorrectedDataFromSelfReflection[5],
																											InteractionResultData.ui32CorrectedDataFromOtherRobot[0],
																											InteractionResultData.ui32CorrectedDataFromOtherRobot[1],
																											InteractionResultData.ui32CorrectedDataFromOtherRobot[2],
																											InteractionResultData.ui32CorrectedDataFromOtherRobot[3],
																											InteractionResultData.ui32CorrectedDataFromOtherRobot[4],
																											InteractionResultData.ui32CorrectedDataFromOtherRobot[5]);

	DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
	return;
}
//----------------------------------------------------------------------------------
//volatile uint8_t	m_ucPrintFinalData							= 0;	// if set => prints continuously the uin32 final data. use 'o' to turn on this printing
static void DebugPrintFinal32Data(void)		// used to print the final uint32 measurement result
{
	// Corrected Data
	sprintf((char * )m_ucDebugBuffer,"FD: S0=%lu\t S1=%lu\t S2=%lu\t S3=%lu\t S4=%lu\t S5=%lu\t\t "
												"O0=%lu\t O1=%lu\t O2=%lu\t O3=%lu\t O4=%lu\t O5=%lu\t\r\n",InteractionResultData.ui32FinalDataFromSelfReflection[0],
																											InteractionResultData.ui32FinalDataFromSelfReflection[1],
																											InteractionResultData.ui32FinalDataFromSelfReflection[2],
																											InteractionResultData.ui32FinalDataFromSelfReflection[3],
																											InteractionResultData.ui32FinalDataFromSelfReflection[4],
																											InteractionResultData.ui32FinalDataFromSelfReflection[5],
																												InteractionResultData.ui32FinalDataFromOtherRobot[0],
																												InteractionResultData.ui32FinalDataFromOtherRobot[1],
																												InteractionResultData.ui32FinalDataFromOtherRobot[2],
																												InteractionResultData.ui32FinalDataFromOtherRobot[3],
																												InteractionResultData.ui32FinalDataFromOtherRobot[4],
																												InteractionResultData.ui32FinalDataFromOtherRobot[5]);

	DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));

	return;
}
//----------------------------------------------------------------------------------
//volatile uint8_t	m_ucPrintFinalTruncatedData					= 0;	// if set => prints the truncated results. in range of 0 ... 255. use 'p' to turn on this print message
static void DebugPrintFinalTruncatedData(void)	// used to print the final truncated data
{
	// Corrected and Truncated Data
	sprintf((char * )m_ucDebugBuffer,"FTD: S0=%u\t S1=%u\t S2=%u\t S3=%u\t S4=%u\t S5=%u\t\t "
													"O0=%u\t O1=%u\t O2=%u\t O3=%u\t O4=%u\t O5=%u\t\r\n",InteractionResultData.ucFinalDataFromSelfReflection[0],
																												InteractionResultData.ucFinalDataFromSelfReflection[1],
																												InteractionResultData.ucFinalDataFromSelfReflection[2],
																												InteractionResultData.ucFinalDataFromSelfReflection[3],
																												InteractionResultData.ucFinalDataFromSelfReflection[4],
																												InteractionResultData.ucFinalDataFromSelfReflection[5],
																													InteractionResultData.ucFinalDataFromOtherRobot[0],
																													InteractionResultData.ucFinalDataFromOtherRobot[1],
																													InteractionResultData.ucFinalDataFromOtherRobot[2],
																													InteractionResultData.ucFinalDataFromOtherRobot[3],
																													InteractionResultData.ucFinalDataFromOtherRobot[4],
																													InteractionResultData.ucFinalDataFromOtherRobot[5]);

	DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
	return;
}
//----------------------------------------------------------------------------------
//volatile uint8_t	m_ucPrintSensitivityData					= 0;	// if set => print once the sensitivity data, use 'l' to print once all the data
//volatile uint8_t	m_ucSensitivityDataChannel					= 0;	// indexes the Self or From Another Robot channel to be increased or decreased. use 'k' to change ={0,1,2,3,4,5}
//volatile uint8_t	m_ucSensitivitySelectSelfOrAnotherRobot		= 0;	// selects the Sensitivity to be altered. ==0 => Self, ==1 => from another robot. use 'j' to change ={0,1}
// to increment or decrement the current value of the sensitivity use 'm' to increment or 'n' to decrement the current value
static void DebugPrintSensitivityData(void)			// used to print the sensitivity data
{
	if( (m_ucSensitivitySelectSelfOrAnotherRobot) )
	{	// alter the sensitivity from another robot
		sprintf((char * )m_ucDebugBuffer,"SENS[Another]: CH=%u\t\t"
				"S0=%u\t S1=%u\t S2=%u\t S3=%u\t S4=%u\t S5=%u\t\t "
					"O0=%u\t O1=%u\t O2=%u\t O3=%u\t O4=%u\t O5=%u\t\r\n",m_ucSensitivityDataChannel,
																				SensitivityData.ucSensitivityForSelfReflection[0],
																				SensitivityData.ucSensitivityForSelfReflection[1],
																				SensitivityData.ucSensitivityForSelfReflection[2],
																				SensitivityData.ucSensitivityForSelfReflection[3],
																				SensitivityData.ucSensitivityForSelfReflection[4],
																				SensitivityData.ucSensitivityForSelfReflection[5],
																					SensitivityData.ucSensitivityForOtherRobot[0],
																					SensitivityData.ucSensitivityForOtherRobot[1],
																					SensitivityData.ucSensitivityForOtherRobot[2],
																					SensitivityData.ucSensitivityForOtherRobot[3],
																					SensitivityData.ucSensitivityForOtherRobot[4],
																					SensitivityData.ucSensitivityForOtherRobot[5]);
	}
	else
	{	// alter the sensitivity from self reflection
		sprintf((char * )m_ucDebugBuffer,"SENS[Self]: CH=%u\t\t"
					"S0=%u\t S1=%u\t S2=%u\t S3=%u\t S4=%u\t S5=%u\t\t "
						"O0=%u\t O1=%u\t O2=%u\t O3=%u\t O4=%u\t O5=%u\t\r\n",m_ucSensitivityDataChannel,
																					SensitivityData.ucSensitivityForSelfReflection[0],
																					SensitivityData.ucSensitivityForSelfReflection[1],
																					SensitivityData.ucSensitivityForSelfReflection[2],
																					SensitivityData.ucSensitivityForSelfReflection[3],
																					SensitivityData.ucSensitivityForSelfReflection[4],
																					SensitivityData.ucSensitivityForSelfReflection[5],
																						SensitivityData.ucSensitivityForOtherRobot[0],
																						SensitivityData.ucSensitivityForOtherRobot[1],
																						SensitivityData.ucSensitivityForOtherRobot[2],
																						SensitivityData.ucSensitivityForOtherRobot[3],
																						SensitivityData.ucSensitivityForOtherRobot[4],
																						SensitivityData.ucSensitivityForOtherRobot[5]);
	}
	//sprintf((char * )m_ucDebugBuffer,"something shorter\r\n");
	DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));

	m_ucPrintSensitivityData = 0;	// only print once (dummy, it is also cleaned in the main loop)
	return;
}
//----------------------------------------------------------------------------------
#ifdef false
static void ChangeSensitivityData(uint8_t ucUpDown)		// used to increment or to decrement the sensitivity data
{	// -- ucUpDown == 0 => down
	// -- ucUpDown == 1 => up
	//------------------------------

	if( (m_ucSensitivitySelectSelfOrAnotherRobot) )
	{	// from another robot
		if( (ucUpDown) )
		{	// up if possible
			if( (SensitivityData.ucSensitivityForOtherRobot[m_ucSensitivityDataChannel] < 100) )		SensitivityData.ucSensitivityForOtherRobot[m_ucSensitivityDataChannel] ++;
		}
		else
		{	// down if possible
			if( (SensitivityData.ucSensitivityForOtherRobot[m_ucSensitivityDataChannel] > 0) )			SensitivityData.ucSensitivityForOtherRobot[m_ucSensitivityDataChannel] --;
		}
	}
	else
	{	// from self reflection
		if( (ucUpDown) )
		{	// up if possible
			if( (SensitivityData.ucSensitivityForSelfReflection[m_ucSensitivityDataChannel] < 100) )	SensitivityData.ucSensitivityForSelfReflection[m_ucSensitivityDataChannel] ++;
		}
		else
		{	// down if possible
			if( (SensitivityData.ucSensitivityForSelfReflection[m_ucSensitivityDataChannel] > 0) )		SensitivityData.ucSensitivityForSelfReflection[m_ucSensitivityDataChannel] --;
		}
	}

	m_ucPrintSensitivityData = 1;	// print once

	return;
}
#endif
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
static void StopAllDebugMessages(void)			// used to stop all the currently printing serial debug messages
{
	// stop the Raw Data display messages
	m_ucEnablePrintRaw = 0;

	// spike removal filter
	m_ucPrintSpikeRemovedData = 0;

	// moving average filter output
	m_ucPrintMovingAverageData = 0;

	// stop the base line calibration message (this is dummy because this message only prints 1 times)
	m_ucPrintBaseLineCalibrationData = 0;

	// stop Corrected Data messages
	m_ucPrintCorrectedData = 0;

	// stop Final uint32 data messages
	m_ucPrintFinalData = 0;

	// stop final truncated messages
	m_ucPrintFinalTruncatedData = 0;



	return;
}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
#endif
//==================================================================================================================== STATIC_END





