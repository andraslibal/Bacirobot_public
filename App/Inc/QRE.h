/*
 * QRE.h
 *
 *  Created on: Feb 19, 2022
 *      Author: Arthur
 */

#ifndef INC_QRE_H_
#define INC_QRE_H_



// Settings
/*
 *
 *
 *
 */


//==================================================================================================================== USECASE START
/*
 *1) first make the offset calibration
 * -- place the robot on the surface it is supposed to run
 * -- start the offset calibration using the API function: QREStartOffsetCalibration(0);
 * -- move the robot around on the surface to make sure that every sensor detects the lowest and the highest reflectivity of the surface
 * -- end the surface calibration using: QREEndOffsetCalibration()
 *
 * 2) implement the callback function: QREResultCallback(QREResultDataType QREData)
 * -- use the "ui32BarDetection" field of the QREData in order to detect the dark stripe below the robot
 * -- for the sensor index have a look at the implementation of the: QREResultCallback() function
 *
 */
//==================================================================================================================== USECASE END


//#define QRE_DEBUG_MODE


// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);	//  debug on PC13 HIGH
// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);  //	debug on PC13 LOW
//====================================================================================================================

// custom data structures
//==================================================================================================================== CUSTOM_DATA_STRUCTS_START

typedef enum
{
	IR_LED_OFF			= 0U,		/* the IR LED on the QRE sensors is OFF */
	IR_LED_ON			= 1U		/* the IR LED on the QRE sensor is ON */
}QRE_IR_LED_STATUS;
//----------------------------------------------------------------------------------

#define QRE_SKIP_COUNTER_DEFAULT				4				/* initial value for the "ucSkipLEDSwitchingCounterLimit" parameter */
#define QRE_MEASUREMENTS_LIMIT					8				/* the default value of the "ucADCMeasurementCounterLimit" variable */
#define QRE_SELF_REFLECTION_THRESHOLD			50				/* the smallest difference between LED_ON - LED_OFF value in order to consider the signal to be valid (self reflected) */

typedef struct
{
	uint8_t					ucSkipLEDSwitchingCounter;			/* used to skip several 1ms time slots in order to increase the sensitivity of the QRE1113 with the 470K resistor */
	uint8_t					ucSkipLEDSwitchingCounterLimit;		/* the limit of the value from above, set according to the  "QRE_SKIP_COUNTER_DEFAULT" value */
	uint8_t					ucADCMeasurementCounter;			/* determines how many measurements to take @ 1 acquisition using the ADC->DMA engine */
	uint8_t					ucADCMeasurementCounterLimit;		/* set according to the "QRE_MEASUREMENTS_LIMIT" value */
	uint8_t					ucMeasurementDone;					/* it is set to 1 if the current measurement has completed */
	QRE_IR_LED_STATUS		ucIRLEDOnOffStatus;					/* if the IR LEDs from the QRE sensors is ON this value is set to 1, otherwise it is set to 0 */


	uint32_t				ui32AccumulatedLEDONValues[8];		/* it is the sum of all values measured while the IR LED was set to OFF. the individual values are  ui32AccumulatedLEDONValues = (4095 - ADC) */
	uint8_t					ucAccumulationCounterForON;			/* the number of ADC measurements done while the IR LED on the QRE sensor was ON -> the measurement is not normalized => this parameter is not used at the moment  */
	uint32_t				ui32AccumulatedLEDOFFValues[8];		/* it is the sum of all values measured while the IR LED was set to OFF. the individual values are  ui32AccumulatedLEDONValues = (4095 - ADC) */
	uint8_t					ucAccumulationCounterForOFF;		/* the number of ADC measurements done while the IR LED on the QRE sensor was OFF -> the measurement is not normalized => this parameter is not used at the moment */
}QREMeasurementDataType;

typedef struct
{
	uint8_t					ucNewResultIsAvailable;				/* set to 1 each time a new measurement was finished */
	uint32_t				ui32SelfIntensity[8];				/* the measured intensity if the signal is self reflected */
	uint32_t				ui32CorrelationMagnitude[8];		/* the correlation the difference between the signal with the LED ON and the LED OFF */
	uint32_t				ui32AmbientSignal[8];				/* represents the signal collected while the LED was turned OFF */
	uint32_t				ui32BarDetectionOffset[8];			/* the offset used for dark strip detection = maximum of ui32CorrelationMagnitude[i] + 25% */
	uint32_t				ui32BarDetection[8];				/* the value of the detected bar (dark region) = ui32BarDetectionOffset[i] - ui32CorrelationMagnitude[i] */
}QREResultDataType;

typedef struct
{
	uint8_t					ucOffsetDetectionInProgress;		/* set to 1 if t the offset detection is in progress, otherwise is set to 0 */
	uint8_t					ucOffsetDetectionDone;				/* set to 1 after the offset calibration has finished and cleared to 0 at offset calibration start */
	uint16_t				ucOfsetDetectionTimeoutMS;			/* the offset detection can be triggered to automatically end in a timeout set to this counter @ the beginning of the detection process. each unit = 1mS */
	uint32_t				ucMaximumDetectedIntensity[8];		/* it will store the maximum detected intensity during the offset detection */
}QREOffsetDetectionDataType;
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== CUSTOM_DATA_STRUCTS_END




// API
//==================================================================================================================== API_START

// callback API
void QREResultCallback(QREResultDataType QREData);		// called @ measurement done
//----------------------------------------------------------------------------------

// regular API
//----------------------------------------------------------------------------------
void QREStartOffsetCalibration(uint16_t ui16TimeoutMS);		// used to start the offset calibration, also the timeout to self finish the calibration must be specified in mS. if timeout is set to 0 => no timeout will be used.
//----------------------------------------------------------------------------------
void QREEndOffsetCalibration(void);				// used to terminate the offset calibration (this function will automatically update the "ui32BarDetectionOffset[]" field in the "QREResultDataType" structure;
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== API_END



// SYSTEM
//==================================================================================================================== SYSTEM_START
void QREInit(void);				// used to initialize the QRE Module. Called from main.c file @ init section
//----------------------------------------------------------------------------------
void QREMainLoop(void);			// loop function of the QRE Module. Called from the main.c file @ main loop section
//----------------------------------------------------------------------------------
void QRETimer1KHZISR(void);		// 1KHZ timer function called from System ISR from stm32f4xx_it.c file @ SysTick Function (!!! keep as short as possible !!!)
//----------------------------------------------------------------------------------
void QREADC2DMACallback(void);	// used to be called from the stm32f4xx_it.c once the DMA reads all the ADC values related to the 8 QRE sensors
//----------------------------------------------------------------------------------
#ifdef QRE_DEBUG_MODE
void QREDebugRX(uint8_t *ucRXBuffer, uint8_t ucRXBufferLength);		// used to receive RX data from the debug interface
void QREDebugRXChar(uint8_t ucRXChar);		// used to receive RX single char from the debug interface
#endif
//----------------------------------------------------------------------------------
//==================================================================================================================== SYSTEM_END




// STATIC (INTERN)
//==================================================================================================================== STATIC_START
// defined in .c file
//==================================================================================================================== STATIC_END




// Resources
/*
#define Q0_Pin GPIO_PIN_12
#define Q0_GPIO_Port GPIOC

#define ADC2_QA1_Pin GPIO_PIN_5
#define ADC2_QA1_GPIO_Port GPIOC
#define ADC2_QA2_Pin GPIO_PIN_4
#define ADC2_QA2_GPIO_Port GPIOC
#define ADC2_QA3_Pin GPIO_PIN_0
#define ADC2_QA3_GPIO_Port GPIOB
#define ADC2_QA4_Pin GPIO_PIN_7
#define ADC2_QA4_GPIO_Port GPIOA
#define ADC2_QA5_Pin GPIO_PIN_0
#define ADC2_QA5_GPIO_Port GPIOC
#define ADC2_QA6_Pin GPIO_PIN_0
#define ADC2_QA6_GPIO_Port GPIOA
#define ADC2_QA7_Pin GPIO_PIN_1
#define ADC2_QA7_GPIO_Port GPIOA
#define ADC2_QA8_Pin GPIO_PIN_2
#define ADC2_QA8_GPIO_Port GPIOA
 *
 *
 *
 *
 */


#endif /* INC_QRE_H_ */
