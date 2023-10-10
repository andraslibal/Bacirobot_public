/*
 * QRE.c
 *
 *  Created on: Feb 19, 2022
 *      Author: Arthur
 */


#include "main.h"

#include "stdint.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"

//#include "WS2812.h"

#include "QRE.h"


// STATIC (INTERN) prototypes
//==================================================================================================================== STATIC_START
// variables - external
extern ADC_HandleTypeDef hadc2;
extern DMA_HandleTypeDef hdma_adc2;
//----------------------------------------------------------------------------------


// variables - internal
#ifdef QRE_DEBUG_MODE
static uint8_t 	m_ucDebugBuffer[250];
//static uint8_t 	m_ucDebugTemp;
static uint8_t	m_ucDebugMessageID;
#endif
#define QRE_SKIP_REAL_TIME		2000
uint16_t m_uiQRESkipRealTime;	// used in the main loop to skip real time
//----------------------------------------------------------------------------------
volatile QREMeasurementDataType			QREMeasurementData = {0};
volatile QREResultDataType				QREResult = {0};
volatile QREOffsetDetectionDataType		QREOffsetDetection = {0};

uint32_t	m_ui32ADCData[8];		/* used to store the ADC conversion result */

//----------------------------------------------------------------------------------


// prototypes
//==================================================================================================================== STATIC_START

static void CalledFromTimer1KHZISR(void);			// for LED ON/OFF switching and	ADC measurement triggering
//----------------------------------------------------------------------------------
static void CalledFromADCDMAISR(void);				// handles data storage once the ADC conversion has finished
//----------------------------------------------------------------------------------
static void CalledFromMainLoop(void);				// handles main loop activities related to the QRE operation
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------

// bar detection
static void OffsetCalibrationMainLoop(void);		// used during the offset calibration
//----------------------------------------------------------------------------------
static void OffsetCalibrationTimer1KHZISR(void);	// used for the calibration timeout
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------

// debug for QRE
#ifdef QRE_DEBUG_MODE
//----------------------------------------------------------------------------------
static void DebugShowMainLoop(void);			// used in the main loop for debug messages
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
#endif
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== STATIC_END


// API
//==================================================================================================================== API_START

// callback API
//----------------------------------------------------------------------------------
__attribute__ ((weak))	void QREResultCallback(QREResultDataType QREData)
{
	// index [0]   = FRONT_CENTRAL
	// index [1]   = FRONT_LEFT_MIDDLE
	// index [2]   = FRONT_RIGHT_MIDDLE
	// index [3]   = FRONT_LEFT_SIDE
	// index [4]   = FRONT_RIGHT_SIDE
	// index [5]   = BELOW_FRONT
	// index [6]   = BELOW_CENTRAL
	// index [7]   = BELOW_BACK
	//----------------------------
	// QREData.ui32CorrelationMagnitude[]	=  the magnitude of the correlation between the signal measured on the QRE and the emitted light by the QRE
	// QREData.ui32SelfIntensity[]  		=  the intensity measured by the QRE correlated with the self LED. 0xFFFFFFFF represents that there is no correlation present
	// QREData.ui32AmbientSignal[]  		=  proportional with the light measured from the environment, without the contribution of the emitted light by the QRE's LED
	// QREData.ui32BarDetection[]			=  the value is proportional with the darkness detected by the QRE considering the self reflected light (mainly) -- this works only after offset calibration --
	// QREData.ui32BarDetectionOffset[]		=  the intensity values detected during offset calibration, these values + 12.5% are used later for BarDetection
	//-----------------------------------------------------------------

	//#define QRE_CALLBACK_SHOW
	#if defined(QRE_DEBUG_MODE) && defined(QRE_CALLBACK_SHOW)
	sprintf((char * )m_ucDebugBuffer,"QRE: %lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\t\t %lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\r\n",QREData.ui32CorrelationMagnitude[0],
																													QREData.ui32CorrelationMagnitude[1],
																													QREData.ui32CorrelationMagnitude[2],
																													QREData.ui32CorrelationMagnitude[3],
																													QREData.ui32CorrelationMagnitude[4],
																													QREData.ui32CorrelationMagnitude[5],
																													QREData.ui32CorrelationMagnitude[6],
																													QREData.ui32CorrelationMagnitude[7],
																														QREData.ui32SelfIntensity[0],
																														QREData.ui32SelfIntensity[1],
																														QREData.ui32SelfIntensity[2],
																														QREData.ui32SelfIntensity[3],
																														QREData.ui32SelfIntensity[4],
																														QREData.ui32SelfIntensity[5],
																														QREData.ui32SelfIntensity[6],
																														QREData.ui32SelfIntensity[7]);
	DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
	#endif
}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------

// regular API
//----------------------------------------------------------------------------------
void QREStartOffsetCalibration(uint16_t ui16TimeoutMS)		// used to start the offset calibration, also the timeout to self finish the calibration must be specified in mS. if timeout is set to 0 => no timeout will be used.
{	// this function starts an offset calibration process
	// the ui16TimeoutMS is used to specify a timeout such that after the timeout passes the calibration is considered done
	// a calibration can be ended at any time using the "QREEndOffsetCalibration" function
	// -- ui16TimeoutMS = timeout in milliseconds, range: [0 ... 65535]
	// -- if(ui16TimeoutMS == 0) => no timeout is used
	//---------------------------------

	// clear the timeout buffer
	QREOffsetDetection.ucMaximumDetectedIntensity[0] = 0;	// first channel
	QREOffsetDetection.ucMaximumDetectedIntensity[1] = 0;	// intermediate channel
	QREOffsetDetection.ucMaximumDetectedIntensity[2] = 0;	// intermediate channel
	QREOffsetDetection.ucMaximumDetectedIntensity[3] = 0;	// intermediate channel
	QREOffsetDetection.ucMaximumDetectedIntensity[4] = 0;	// intermediate channel
	QREOffsetDetection.ucMaximumDetectedIntensity[5] = 0;	// intermediate channel
	QREOffsetDetection.ucMaximumDetectedIntensity[6] = 0;	// intermediate channel
	QREOffsetDetection.ucMaximumDetectedIntensity[7] = 0;	// last channel

	// Calibration is not done yet
	QREOffsetDetection.ucOffsetDetectionDone = 0;

	// calibration in progress
	QREOffsetDetection.ucOffsetDetectionInProgress = 1;

	// timeout if it is the case
	if(ui16TimeoutMS)	QREOffsetDetection.ucOfsetDetectionTimeoutMS = ui16TimeoutMS;

	return;
}
//----------------------------------------------------------------------------------
void QREEndOffsetCalibration(void)				// used to terminate the offset calibration (this function will automatically update the "ui32BarDetectionOffset[]" field in the "QREResultDataType" structure;
{	// this terminates the offset calibration and populate the "ui32BarDetectionOffset" filed in the global "QREResultDataType" type variable
	//------------------------------

	uint8_t i;

	// set the "ui32BarDetectionOffset" filed
	for(i=0; i<8; i++)
	{
		if( (QREOffsetDetection.ucMaximumDetectedIntensity[i] > (QREOffsetDetection.ucMaximumDetectedIntensity[i] >> 3) ) ) QREResult.ui32BarDetectionOffset[i] = QREOffsetDetection.ucMaximumDetectedIntensity[i] - (QREOffsetDetection.ucMaximumDetectedIntensity[i] >> 3);		// the value + 12.5%
		else 																												QREResult.ui32BarDetectionOffset[i] = QREOffsetDetection.ucMaximumDetectedIntensity[i];
	}

	// Calibration is not done yet
	QREOffsetDetection.ucOffsetDetectionDone = 1;

	// calibration in progress
	QREOffsetDetection.ucOffsetDetectionInProgress = 0;

	// timeout
	QREOffsetDetection.ucOfsetDetectionTimeoutMS = 0;

	return;
}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== API_END


// SYSTEM
//==================================================================================================================== SYSTEM_START
void QREInit(void)	// used to initialize the QRE Module. Called from main.c file @ init section
{

	QREMeasurementData.ucSkipLEDSwitchingCounterLimit 	= (uint8_t) QRE_SKIP_COUNTER_DEFAULT;
	QREMeasurementData.ucADCMeasurementCounterLimit 	= QRE_MEASUREMENTS_LIMIT;

	return;
}
//----------------------------------------------------------------------------------
void QREMainLoop(void)	// loop function of the QRE Module. Called from the main.c file @ main loop section
{
	//uint16_t uiTemp[8];

	// skip real time
	if(m_uiQRESkipRealTime)
	{
		m_uiQRESkipRealTime --;
		return;
	}
	m_uiQRESkipRealTime = QRE_SKIP_REAL_TIME;
	//--------------------------------------------------------

	// ----------- processes the received data ------------------
	CalledFromMainLoop();		// gets the measured data
	// ----------------------------------------------------------

	// --------------- handles stripe detection -----------------
	OffsetCalibrationMainLoop();	// adds the offset component and the dark stripe detection
	// ----------------------------------------------------------

	// ----------------- debug function -------------------------
	#ifdef QRE_DEBUG_MODE
	DebugShowMainLoop();
	#endif
	// ----------------------------------------------------------

	// -------- call the callback if data is available ----------
	if( (QREResult.ucNewResultIsAvailable  == 1) )
	{
		QREResultCallback(QREResult);
		QREResult.ucNewResultIsAvailable  = 0;
	}
	// ----------------------------------------------------------

	return;
}
//----------------------------------------------------------------------------------
void QRETimer1KHZISR(void)	// 1KHZ timer function called from System ISR from stm32f4xx_it.c file @ SysTick Function (!!! keep as short as possible !!!)
{

	CalledFromTimer1KHZISR();

	OffsetCalibrationTimer1KHZISR();

	return;
}
//----------------------------------------------------------------------------------
void QREADC2DMACallback(void)	// used to be called from the stm32f4xx_it.c once the DMA reads all the ADC values related to the 8 QRE sensors
{

	CalledFromADCDMAISR();

	return;
}
//----------------------------------------------------------------------------------
#ifdef QRE_DEBUG_MODE
void QREDebugRX(uint8_t *ucRXBuffer, uint8_t ucRXBufferLength)	// used to receive RX data from the debug interface
{
	return;
}
void QREDebugRXChar(uint8_t ucRXChar)	// used to receive RX single char from the debug interface
{

	if( ucRXChar == 'q' )
	{
		// HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef* hadc, uint32_t* pData, uint32_t Length)
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);	//  debug on PC13 HIGH
		//if( HAL_ADC_Start_DMA(&hadc2,m_ui32SamplingADC,8) != HAL_OK )
		if( HAL_ADC_Start_DMA(&hadc2,m_ui32ADCData,8) != HAL_OK )
		{
			Error_Handler2(ERROR_CODE_QRE_DEBUG_001);
		}
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);	//  debug on PC13 HIGH
		return;
	}

	if( ucRXChar == 'w' )
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);	//  debug on PC13 HIGH
		return;
	}
	if( ucRXChar == 'e' )
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);  //	debug on PC13 LOW
		return;
	}

	if( ucRXChar == 'a' )
	{
		HAL_GPIO_WritePin(Q0_GPIO_Port, Q0_Pin, GPIO_PIN_SET);	//  IR LED is turned ON
		return;
	}
	if( ucRXChar == 's' )
	{
		HAL_GPIO_WritePin(Q0_GPIO_Port, Q0_Pin, GPIO_PIN_RESET);  //	IR LED is turned OFF
		return;
	}



	if( ucRXChar == 'R' )
	{
		//if(QRESkipMax < 255)	QRESkipMax ++;
		return;
	}
	if( ucRXChar == 'r' )
	{
		//if(QRESkipMax)	QRESkipMax --;
		return;
	}

	// -------- debug messages for QRE -----------
	if( ucRXChar == 't' )
	{	// display the correlation data and the self intensity
		m_ucDebugMessageID = 1;
		return;
	}
	if( ucRXChar == 'y' )
	{	// display the correlation data and the Ambient Intensity
		m_ucDebugMessageID = 2;
		return;
	}
	if( ucRXChar == 'u' )
	{	// display once the Offset Calibration data
		m_ucDebugMessageID = 3;
		return;
	}
	if( ucRXChar == 'i' )
	{	// display the correlation data and the bar detection value
		m_ucDebugMessageID = 4;
		return;
	}
	if( ucRXChar == 'o' )
	{	// Start offset calibration
		QREStartOffsetCalibration(0);	// infinitely
		// display once that info message
		m_ucDebugMessageID = 5;
		return;
	}
	if( ucRXChar == 'O' )
	{	// Start offset calibration
		QREStartOffsetCalibration(3000);	// for 3 seconds
		// display once that info message
		m_ucDebugMessageID = 5;
		return;
	}
	if( ucRXChar == 'p' )
	{	// End the offset calibration
		QREEndOffsetCalibration();
		// display once that info message
		m_ucDebugMessageID = 6;
		return;
	}
	// -------------------------------------------




	/*
	// callback testing
	if( ucRXChar == 'z' )
	{
		m_ucDebugTemp = 1;	// show contrast
		return;
	}
	if( ucRXChar == 'x' )
	{
		m_ucDebugTemp = 0;	// show dark
		return;
	}
	*/


	return;
}
#endif
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== SYSTEM_END




// STATIC (INTERN) implementation
//==================================================================================================================== STATIC_START

static void CalledFromTimer1KHZISR(void)			// for LED ON/OFF switching and	ADC measurement triggering
{
	// -------------- the measurement is done -----------------
	if( (QREMeasurementData.ucMeasurementDone == 1) )
	{	// measurement is done and is waiting to be processed in the main loop
		return;
	}
	// --------------------------------------------------------

	// -------------- the measurement is done -----------------
	if( (QREMeasurementData.ucMeasurementDone == 2) )
	{	// now we start another measurement sequence

		// turn LED OFF and take another measurement
		QREMeasurementData.ucIRLEDOnOffStatus = IR_LED_ON;
		HAL_GPIO_WritePin(Q0_GPIO_Port, Q0_Pin, GPIO_PIN_SET);	//  IR LED is turned ON

		// re-arm the counters
		QREMeasurementData.ucSkipLEDSwitchingCounter 	= QREMeasurementData.ucSkipLEDSwitchingCounterLimit;
		QREMeasurementData.ucADCMeasurementCounter		= QREMeasurementData.ucADCMeasurementCounterLimit;

		// wait for the end of the measurement
		QREMeasurementData.ucMeasurementDone = 0;
		return;
	}
	// --------------------------------------------------------

	// --------- skip the time between IR changes -------------
	if( (QREMeasurementData.ucSkipLEDSwitchingCounter) )
	{	// skip time
		QREMeasurementData.ucSkipLEDSwitchingCounter --;
		return;
	}
	// --------------------------------------------------------

	// --------- perform the measurement series ---------------
	if( (QREMeasurementData.ucADCMeasurementCounter) )
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);	//  debug on PC13 HIGH

		// ADC trigger
		if( HAL_ADC_Start_DMA(&hadc2,m_ui32ADCData,8) != HAL_OK )
		{
			#ifdef QRE_DEBUG_MODE
			sprintf((char * )m_ucDebugBuffer,"QRE Error 01\r\n");
			DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
			#endif
			Error_Handler2(ERROR_CODE_QRE_DEBUG_001);
		}

		QREMeasurementData.ucADCMeasurementCounter --;

		return;
	}
	// --------------------------------------------------------

	// ------- switch the IR pattern and check for the end of the measurements ---------
	if( (QREMeasurementData.ucIRLEDOnOffStatus == IR_LED_ON) )
	{	// change the IR pattern
		// turn LED OFF and take another measurement
		QREMeasurementData.ucIRLEDOnOffStatus = IR_LED_OFF;
		HAL_GPIO_WritePin(Q0_GPIO_Port, Q0_Pin, GPIO_PIN_RESET);  //	IR LED is turned OFF

		// re-arm the counters
		QREMeasurementData.ucSkipLEDSwitchingCounter 	= QREMeasurementData.ucSkipLEDSwitchingCounterLimit;
		QREMeasurementData.ucADCMeasurementCounter		= QREMeasurementData.ucADCMeasurementCounterLimit;

		return;
	}
	// set the end of the measurement
	QREMeasurementData.ucMeasurementDone = 1;
	// ---------------------------------------------------------------------------------

	return;
}
//----------------------------------------------------------------------------------
static void CalledFromADCDMAISR(void)				// handles data storage once the ADC conversion has finished
{	// ADC done => the result is in the: m_ui32ADCData variable
	// the data will be saved into the: QREAcquisitionData structure
	//------------------------------------------

	uint8_t i;

	for(i=0; i<8; i++)
	{
		// collect the measured data for all the channels
		if( (QREMeasurementData.ucIRLEDOnOffStatus == IR_LED_ON) )
		{	// the IR  LED was ON
			QREMeasurementData.ui32AccumulatedLEDONValues[i]	+= (4095 - m_ui32ADCData[i]);
			QREMeasurementData.ucAccumulationCounterForON ++;
		}
		else
		{	// the IR  LED was OFF
			QREMeasurementData.ui32AccumulatedLEDOFFValues[i]	+= (4095 - m_ui32ADCData[i]);
			QREMeasurementData.ucAccumulationCounterForOFF ++;
		}
	}

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);	//  debug on PC13 LOW

	return;
}
//----------------------------------------------------------------------------------
static void CalledFromMainLoop(void)				// handles main loop activities related to the QRE operation
{	// QREResult
	// QREMeasurementData
	//---------------------------------

	uint8_t 	i;

	if( (QREMeasurementData.ucMeasurementDone == 1) )
	{	// measurement is done => process the data and reset the "QREMeasurementData" structure

		// ----------- store the result in the result data structure --------------
		if( (QREResult.ucNewResultIsAvailable == 0) )
		{
			QREResult.ucNewResultIsAvailable = 1;

			for(i=0; i<8; i++)
			{
				// calculate the correlation
				if( (QREMeasurementData.ui32AccumulatedLEDONValues[i] > QREMeasurementData.ui32AccumulatedLEDOFFValues[i]) )	QREResult.ui32CorrelationMagnitude[i] = (QREMeasurementData.ui32AccumulatedLEDONValues[i] - QREMeasurementData.ui32AccumulatedLEDOFFValues[i]);
				else  																											QREResult.ui32CorrelationMagnitude[i] = 0;

				// self reflection
				if( (QREResult.ui32CorrelationMagnitude[i] > QRE_SELF_REFLECTION_THRESHOLD) )		QREResult.ui32SelfIntensity[i] = QREMeasurementData.ui32AccumulatedLEDONValues[i];
				else 																				QREResult.ui32SelfIntensity[i] = 0xFFFFFFFF;

				// ambient light
				QREResult.ui32AmbientSignal[i] = QREMeasurementData.ui32AccumulatedLEDOFFValues[i];	// consider only the signal from the external sources
			}
		}
		// ------------------------------------------------------------------------

		// ------------ Clear the Measurement data structure ----------------------
		QREMeasurementData.ucAccumulationCounterForON = 0;		// not used for now
		QREMeasurementData.ucAccumulationCounterForOFF = 0;		// not used for now
		for(i=0; i<8; i++)
		{
			QREMeasurementData.ui32AccumulatedLEDOFFValues[i] 	= 0;
			QREMeasurementData.ui32AccumulatedLEDONValues[i]	= 0;
		}
		QREMeasurementData.ucMeasurementDone = 2;	// start another measurement
		// ------------------------------------------------------------------------

	}
	// ----------------------------------------------------------



	// ----------------------------------------------------------

	return;
}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------

// bar detection
static void OffsetCalibrationMainLoop(void)		// used during the offset calibration
{
	uint8_t i;

	// ------------- check if there is  new data available ------------------
	if( (QREResult.ucNewResultIsAvailable == 0) )	return;	// no new data
	// ----------------------------------------------------------------------

	// ------------------ used during offset calibration --------------------
	if( (QREOffsetDetection.ucOffsetDetectionInProgress == 1) )
	{	// get the maximum
		for(i=0; i<8; i++)
		{
			if(QREResult.ui32SelfIntensity[i] < 0xFFFFFFFF)
			{
				if( (QREOffsetDetection.ucMaximumDetectedIntensity[i] < QREResult.ui32SelfIntensity[i]) ) 	QREOffsetDetection.ucMaximumDetectedIntensity[i] = QREResult.ui32SelfIntensity[i];
			}

		}
		return;
	}
	// ----------------------------------------------------------------------

	// ----------------- used after offset calibration ----------------------
	if( (QREOffsetDetection.ucOffsetDetectionDone == 1) )
	{// offset calibration already is done and the offset thresholds are stored in the "QREResult.ui32BarDetectionOffset[]" field
		for(i=0; i<8; i++)
		{
			if( (QREResult.ui32SelfIntensity[i] < QREResult.ui32BarDetectionOffset[i]) )	QREResult.ui32BarDetection[i] = (QREResult.ui32BarDetectionOffset[i] - QREResult.ui32SelfIntensity[i]);
			else 																			QREResult.ui32BarDetection[i] = 0;
		}
		return;
	}
	// ----------------------------------------------------------------------

	return;
}
//----------------------------------------------------------------------------------
static void OffsetCalibrationTimer1KHZISR(void)	// used for the calibration timeout
{
	if( (QREOffsetDetection.ucOfsetDetectionTimeoutMS) )
	{
		QREOffsetDetection.ucOfsetDetectionTimeoutMS --;
		if( (QREOffsetDetection.ucOfsetDetectionTimeoutMS == 0) )	QREEndOffsetCalibration();
	}

	return;
}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------

// debug for QRE
#ifdef QRE_DEBUG_MODE
//----------------------------------------------------------------------------------
static void DebugShowMainLoop(void)			// used in the main loop for debug messages
{

	if( (m_ucDebugMessageID == 1) )
	{	// display the correlation data and the self intensity
		sprintf((char * )m_ucDebugBuffer,"QRE_I: %lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\t\t %lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\r\n",QREResult.ui32CorrelationMagnitude[0],
																															QREResult.ui32CorrelationMagnitude[1],
																															QREResult.ui32CorrelationMagnitude[2],
																															QREResult.ui32CorrelationMagnitude[3],
																															QREResult.ui32CorrelationMagnitude[4],
																															QREResult.ui32CorrelationMagnitude[5],
																															QREResult.ui32CorrelationMagnitude[6],
																															QREResult.ui32CorrelationMagnitude[7],
																																QREResult.ui32SelfIntensity[0],
																																QREResult.ui32SelfIntensity[1],
																																QREResult.ui32SelfIntensity[2],
																																QREResult.ui32SelfIntensity[3],
																																QREResult.ui32SelfIntensity[4],
																																QREResult.ui32SelfIntensity[5],
																																QREResult.ui32SelfIntensity[6],
																																QREResult.ui32SelfIntensity[7]);
		DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
		return;
	}
	else if( (m_ucDebugMessageID == 2) )
	{	// display the correlation data and the Ambient Intensity
		sprintf((char * )m_ucDebugBuffer,"QRE_A: %lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\t\t %lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\r\n",QREResult.ui32CorrelationMagnitude[0],
																															QREResult.ui32CorrelationMagnitude[1],
																															QREResult.ui32CorrelationMagnitude[2],
																															QREResult.ui32CorrelationMagnitude[3],
																															QREResult.ui32CorrelationMagnitude[4],
																															QREResult.ui32CorrelationMagnitude[5],
																															QREResult.ui32CorrelationMagnitude[6],
																															QREResult.ui32CorrelationMagnitude[7],
																																QREResult.ui32AmbientSignal[0],
																																QREResult.ui32AmbientSignal[1],
																																QREResult.ui32AmbientSignal[2],
																																QREResult.ui32AmbientSignal[3],
																																QREResult.ui32AmbientSignal[4],
																																QREResult.ui32AmbientSignal[5],
																																QREResult.ui32AmbientSignal[6],
																																QREResult.ui32AmbientSignal[7]);
		DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
		return;
	}
	else if( (m_ucDebugMessageID == 3) )
	{	// display once the Offset Calibration data
		sprintf((char * )m_ucDebugBuffer,"QRE_OC: %lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\t\t %lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\t\tD=%u\tTm=%u\r\n",QREOffsetDetection.ucMaximumDetectedIntensity[0],
																															QREOffsetDetection.ucMaximumDetectedIntensity[1],
																															QREOffsetDetection.ucMaximumDetectedIntensity[2],
																															QREOffsetDetection.ucMaximumDetectedIntensity[3],
																															QREOffsetDetection.ucMaximumDetectedIntensity[4],
																															QREOffsetDetection.ucMaximumDetectedIntensity[5],
																															QREOffsetDetection.ucMaximumDetectedIntensity[6],
																															QREOffsetDetection.ucMaximumDetectedIntensity[7],
																																QREResult.ui32BarDetectionOffset[0],
																																QREResult.ui32BarDetectionOffset[1],
																																QREResult.ui32BarDetectionOffset[2],
																																QREResult.ui32BarDetectionOffset[3],
																																QREResult.ui32BarDetectionOffset[4],
																																QREResult.ui32BarDetectionOffset[5],
																																QREResult.ui32BarDetectionOffset[6],
																																QREResult.ui32BarDetectionOffset[7],
																																  QREOffsetDetection.ucOffsetDetectionDone,
																																  QREOffsetDetection.ucOfsetDetectionTimeoutMS);
		DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
		m_ucDebugMessageID = 0;	// only display once
		return;
	}
	else if( (m_ucDebugMessageID == 4) )
	{	// display the correlation data and the bar detection value
		sprintf((char * )m_ucDebugBuffer,"QRE_BAR: %lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\t\t %lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\r\n",QREResult.ui32CorrelationMagnitude[0],
																															QREResult.ui32CorrelationMagnitude[1],
																															QREResult.ui32CorrelationMagnitude[2],
																															QREResult.ui32CorrelationMagnitude[3],
																															QREResult.ui32CorrelationMagnitude[4],
																															QREResult.ui32CorrelationMagnitude[5],
																															QREResult.ui32CorrelationMagnitude[6],
																															QREResult.ui32CorrelationMagnitude[7],
																																QREResult.ui32BarDetection[0],
																																QREResult.ui32BarDetection[1],
																																QREResult.ui32BarDetection[2],
																																QREResult.ui32BarDetection[3],
																																QREResult.ui32BarDetection[4],
																																QREResult.ui32BarDetection[5],
																																QREResult.ui32BarDetection[6],
																																QREResult.ui32BarDetection[7]);
		DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
		return;
	}
	else if( (m_ucDebugMessageID == 5) )
	{	// Start offset calibration
		sprintf((char * )m_ucDebugBuffer,"QRE_START_OFFSET: %u\r\n",QREOffsetDetection.ucOfsetDetectionTimeoutMS);
		DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
		m_ucDebugMessageID = 0;
		return;
	}
	else if( (m_ucDebugMessageID == 6) )
	{	// End offset calibration
		sprintf((char * )m_ucDebugBuffer,"QRE_END_OFFSET:\r\n");
		DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
		m_ucDebugMessageID = 0;
		return;
	}



	return;
}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
#endif
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== STATIC_END
