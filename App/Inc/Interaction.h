/*
 * Interaction.h
 *
 *  Created on: Feb 16, 2022
 *      Author: Arthur
 */

#ifndef INC_INTERACTION_H_
#define INC_INTERACTION_H_




// Settings
// the Interaction section can be parameterized:
// 1) the spike filter:
//#define INTERACTION_USES_SPIKE_FILTER
// other parameters can be changed down below on this header file:
// -- #define INTERACTION_SPIKE_HISTORY_DEPTH								5		/* details can be found down @ the filter section */
// -- #define INTERACTION_SPIKE_MAXIMUM_SPIKE								200		/* details can be found down @ the filter section */
// -- #define INTERACTION_SPIKE_MAXIMUM_CONSECUTIVE_ERRORS					5		/* details can be found down @ the filter section */


// 2) the Moving Averaging Filter:
#define INTARACTION_USES_MOVING_AVERAGING_FILTER
// -- #define INTERACTION_MOVING_AVERAGE_FILTER_MAX_DEPTH_POWER				4		/* details can be found down @ the filter section */


//==================================================================================================================== USECASE START
/*
 * Interaction is measured in 2 ways:
 * --- as reflection from the signal emitted of the own side LED's and after correlation calculation this is referred to as SELF REFLECTION or CORRELATION signal
 * --- as the signal received from another robot
 * on each type of interaction is applied a gain factor called sensitivity parameter. for each type of interaction measurement and each type of sensitivity can be adjusted in the range of 0 to 100
 * -- sensitivity of 50 means 		GAIN=1
 * -- sensitivity below 50 means 	GAIN<1
 * -- sensitivity above 50 means	GAIN>1
 *
 *-- in order to use the measured data just implement the callback function in your code: "InteractionNewResultAvailableCallback()"
 *-- read out the interaction result from the input parameter of the callback
 *-- in case you need to re-trigger the base line calibration => use the API function: "InteractionCalculateBaseline()"
 *-- if you need to check the current base line values use the function: "InteractionGetBaseLineData()"
 *-- in order to change the sensitivity on a channel use one of the functions:
 *---- InteractionSetSensitivity();
 *---- InteractionIncDecSensitivity()
 *-- in order to read the current sensitivity settings use the function: "InteractionGetSensitivityData()"
 *
 */
//==================================================================================================================== USECASE END


//#define INTERACTION_DEBUG_MODE


//#define INTERACTION_SHOW_ON_WS2812		/* if this define is not commented the interaction data is shown on the WS2812 LED's */
//#define INTERACTION_EEPROM_DEBUG		/* only usable if the "INTERACTION_DEBUG_MODE" is defined as well */

// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);	//  debug on PC13 HIGH
// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);  //	debug on PC13 LOW
//====================================================================================================================

// custom data structures
//==================================================================================================================== CUSTOM_DATA_STRUCTS_START
// for all [6] element buffers:
// ---- BufferName[0]	=> refers to channel 0	(front)
// ---- BufferName[1]	=> refers to channel 1	(front-left)
// ---- BufferName[2]	=> refers to channel 2	(rear-left)
// ---- BufferName[3]	=> refers to channel 3	(rear)
// ---- BufferName[4]	=> refers to channel 4	(rear-right)
// ---- BufferName[5]	=> refers to channel 4	(front-right)

#define INTERACTION_NUMBER_OF_MEASUREMENTS_PER_ACQUISITIONS		60			/* defines the number of measurement-rounds per acquisition. how many measurement rounds are accumulated into one acquisition result */
typedef struct
{
	// variables holding the acquisition control
	uint8_t				ucIRPatternNumber;							/* it determines the type of IR pattern currently displayed on the side IR LEDs */
																	/* if(ucIRPatternNumber == 0)	=> the {0,2,4} IRChannles are ON and the {1,3,5} IRChannels are OFF */
																	/* if(ucIRPatternNumber == 1)	=> the {1,3,5} IRChannles are ON and the {0,2,4} IRChannels are OFF */
	uint32_t			ui32AcquisitionStartMomentUS;				/* it holds the moment in time when the Acquisition has started (the time is obtained using the TIMER5 => __HAL_TIM_GET_COUNTER(&htim5). 1us/step) */
	uint32_t			ui32AcquisitionEndMomentUS;					/* it holds the moment in time when the Acquisition has ended   (the time is obtained using the TIMER5 => __HAL_TIM_GET_COUNTER(&htim5). 1us/step) */
	uint8_t				ucAcquisitionCounter;						/* counts the acquisition round, after each IR pattern change this counter is decremented. once zero is reached it is considered that the acquisition has ended and the data is ready to be operated using the normal main loop task */
	uint8_t				ucAcquisitionCounterMax;					/* the number of IR patterns and measurements performed during an acquisition process. to have the same number of ON and OFF values this number should be even */
	uint32_t			ui32AcquisitionRoundCounter;				/* this variable is incremented after each acquisition process */

	// variables used for Data collection during an acquisition sequence
	// ON values
	uint32_t			ui32MaxLEDOn[6];							/* holds the maximum value detected on every channel as long as the corresponding side-IR LED is 	turned ON  */
	uint32_t			ui32MinLEDOn[6];							/* holds the minimum value detected on every channel while the corresponding side-IR LED is 		turned ON  */
	uint32_t			ui32AccuLEDOn[6];							/* the accumulated value of each measurement during the acquisition while the side LED was 			turned ON  */
	uint8_t				ucNumberOfLEDOnValues[6];					/* the number of measurements done with the side-IR LED 											turned ON  */
	// OFF values
	uint32_t			ui32MaxLEDOff[6];							/* holds the maximum value detected on every channel as long as the corresponding side-IR LED is 	turned OFF */
	uint32_t			ui32MinLEDOff[6];							/* holds the minimum value detected on every channel while the corresponding side-IR LED is 		turned OFF */
	uint32_t			ui32AccuLEDOff[6];							/* the accumulated value of each measurement during the acquisition while the side LED was 			turned OFF */
	uint8_t				ucNumberOfLEDOffValues[6];					/* the number of measurements done with the side-IR LED 											turned OFF */
}AcquisitionDataStructure;		// is the main data structure controlling the acquisition process, the IR Pattern generation and the data collection
//----------------------------------------------------------------------------------

typedef struct
{
	// acquisition process related data fields
	uint8_t				ucNewDataAvailable;							/* set to 1 if a new set of data is available. until this filed is not cleared to 0 no new data update will be made into this structure */
	uint32_t			ui32AcquisitionRoundCounter;				/* identifies the Acquisition this data is originating from */
	uint32_t			ui32AcquisitionDurationUS;					/* the duration of the Acquisition the data data is coming from in microseconds */

	// measurement related data fields
	uint32_t			ui32SignalFromSelfReflection[6];			/* it represents the correlation result from the self IR pattern */
	uint32_t			ui32SignalFromAnotherRobot[6];				/* it represents the signal gathered from an external IR source, most likely from another robot */
}AcquisitionResultDataStructure;
//----------------------------------------------------------------------------------

#define INTERACTION_SPIKE_HISTORY_DEPTH								5		/* determines how many older samples will be kept and the new data will be compared against */
#define INTERACTION_SPIKE_MAXIMUM_SPIKE								200		/* if the step between the current sample and at least one of the history samples is larger than this step => the new sample will be thrown away (disregarded) */
																				/* if the new sample do not differ this much => it will replace the oldest history sample and it will be available at the exit of the filter */
#define INTERACTION_SPIKE_MAXIMUM_CONSECUTIVE_ERRORS				5		/* if there are more than this many consecutive disregarded samples the filter will be restarted and all the history samples reconstructed */
typedef struct
{
	// used to remove data from the signal received from another robot => that is the most sensitive, for self reflection the correlation will get rid of the noise
	// input data related variables
	uint32_t			ui32SamplesHistory[6][20];					/* this is the buffer holding the history samples for all the 6 channels and with a maximum depth of 20 elements => "INTERACTION_SPIKE_HISTORY_DEPTH" can not exceed this limit */
	uint8_t				ucSamplesHistoryCurrentDepth[6];			/* used to re-populate the history buffer after successive unsuitable new data, the number of elements in the history buffer */
	uint8_t				ucSamplesHistoryIndex[6];					/* holds the index of the oldest sample in the "ui32SamplesHistory[][]" buffer => this will be replaced with the new valid measurement data */
	uint8_t				ucSamplesSuccessiveErrorCounter[6];			/* counts the successive errors on each channel */

	// output data related variables
	//uint32_t			ui32OutputValues[6];						/* the output of the filter, a channel is updated if the input data is fitting in the filter condition, if not the old value will be kept in the filter*/
	//uint8_t			ucNewOutputDataIsAvailable;					/* if at least one channel is with output this value will be set to 1 */


}SpikeFilterDataStructure;	// this filter is applied to the result data from the "AcquisitionResultDataStructure" and it removes spikes due to parasitic coupling
//----------------------------------------------------------------------------------
#define INTERACTION_MOVING_AVERAGE_FILTER_MAX_DEPTH_POWER			4	/* the depth of the filter as power of 2 */
/* 1, 2, 3, 4, 5, 6 */
#define INTERACTION_MOVING_AVERAGE_FILTER_MAX_NUMBER_OF_SAMPLES		(0x01U << INTERACTION_MOVING_AVERAGE_FILTER_MAX_DEPTH_POWER)	/* the number of samples for each channel to accumulate and than averaged, it is power of 2 */
/* 1->2, 2->4, 3->8, 4-> 16, 5->32, 6->64 => the number of samples as function of the power of the depth defined by "INTERACTION_MOVING_AVERAGE_FILTER_MAX_DEPTH_POWER" */
typedef struct
{
	// variables used for self reflection
	uint32_t			ui32SamplesFromSelfReflection[6][64];		/* the accumulator buffer for the self reflection signal, for all the 6 channels */

	// variables used for signal from another robot
	uint32_t			ui32SamplesFromAnotherRobot[6][64];			/* the accumulator buffer for the signal received from another robot, for all the 6 channels */

	// indexer
	uint8_t				ucSampleIndexer[6];							/* used to index where to replace the old buffer data with the newly measured data, it runs up to "INTERACTION_MOVING_AVERAGE_FILTER_MAX_NUMBER_OF_SAMPLES" */
}MovingAverageFilterData;	// this filter is applied after the spike remover filter and before the base line correction
//----------------------------------------------------------------------------------

#define INTERACTION_BASE_LINE_CALIBRATION_MAX_ACCUMULATION_POWER	6	/* determine the 2^power expression of the total number of accumulations done during a base line calibration => see the define below -> "ucAccumulationDivisionPower" */
/* 1, 2, 3, 4, 5, 6, 7 */
#define INTERACTION_BASE_LINE_CALIBRATION_MAX_ACCUMULATIONS			(0x01U << INTERACTION_BASE_LINE_CALIBRATION_MAX_ACCUMULATION_POWER);		/*  will be used to initialize the "ucAccumulationCounterMax" field */
/* 1->2, 2->4, 3->8, 4->16, 5->32, 6->64, 7->128 */
#define INTERACTION_BASE_LINE_CALIBRATION_TRIGGER_TIME_MS			3000	/* the time in ms that will trigger a base line calibration if none is set before or restored from the EEPROM. used in "ui16TimeToCheckCalibrationMSMax" */
#define INTERACTION_MAX_NOISE_SELF_REFLECTION						40	/* is the maximum noise what is consider to be OK between the maximum and the minimum values recorded during base line calibration -> "ui32SelfMaxNoise" */
#define INTERACTION_MAX_NOISE_OTHER_ROBOT							80	/* is the maximum noise what can be considered to be OK during other robot max and min value while trying to establish the base line -> "ui32OtherRobotMaxNoise" */
typedef struct
{
	// administrative variables used during the base line calibration
	uint8_t				ucAccumulationDivisionPower;				/* accumulation counter is a power of 2 => the division to determine the average is just a shift given by this value set from "INTERACTION_BASE_LINE_CALIBRATION_MAX_ACCUMULATION_POWER" */
	uint8_t				ucAccumulationCounter;						/* during the base line calibration several consecutive samples are accumulated to determine the average threshold for each channel, this is the counting of the successive samples */
	uint8_t				ucAccumulationCounterMax;					/* the number of successive accumulations needed to be done during base line calibration */
	uint8_t				ucBaseLineCalibrationDone;					/* if set to 1 => the base line calibration is done and the "ui32CalculatedSelfBaseLineValues[]" and "ui32CalculatedBaseValuesFromOtherRobot[]" contains valid data */
	uint16_t			ui16TimeToCheckCalibrationMS;				/* this timer is incremented in the 1KHZ ISR from 0 after start until it reaches the "ui16TimeToCheckCalibrationMSMax" value and at this time if there is no base line => a base line calibration will be triggered */
	uint16_t			ui16TimeToCheckCalibrationMSMax;			/* time in ms that determines after how many milliseconds from reboot the base line calibration is checked */

	// from self reflection
	uint32_t			ui32AccuSignalFromSelfReflection[6];		/* accumulates the self reflection signal for all the samples during the base line calibration  */
	uint32_t			ui32AccuSignalFromSelfReflectionMin[6];		/* holds the minimum value for detected during the accumulation, individually for each channel (min and max values are used to estimate the maximum noise during the base line calibration) */
	uint32_t			ui32AccuSignalFromSelfReflectionMax[6];		/* holds the maximum value for detected during the accumulation, individually for each channel (min and max values are used to estimate the maximum noise during the base line calibration) */
	uint32_t			ui32SelfMaxNoise;							/* the maximum noise allowed in order to be considered the base line calibration valid =>  (ui32AccuSignalFromSelfReflectionMax[i] - ui32AccuSignalFromSelfReflectionMin[i]) < ui32SelfMaxNoise, for all 'i' values */
	uint32_t			ui32CalculatedSelfBaseLineValues[6];		/* represents the values that are considered the base line values for all 6 channels => the real value is the raw values - this value if the raw value is larger otherwise it is 0 */
																		/* this value is the base line measured + 50% these values must be stored in EEPROM */

	// from other robot
	uint32_t			ui32AccuSignalFromOtherRobot[6];			/* summed value collecting all the samples during the "ucAccumulationCounterMax" counts */
	uint32_t			ui32AccuSignalFromOtherRobotMin[6];			/* the lowest value detected during the accumulation */
	uint32_t			ui32AccuSignalFromOtherRobotMax[6];			/* the highest value detected during the accumulation */
	uint32_t			ui32OtherRobotMaxNoise;						/* the maximum noise allowed during the base line calibration on all channels */
	uint32_t			ui32CalculatedBaseValuesFromOtherRobot[6];	/* represents the determined base values such that the real value is the raw value - this value if the raw value is larger otherwise it is 0 */
																		/* this value is the base line measured + 50% these values must be stored in EEPROM */
}BaseLineCalibrationDataStructure;
//----------------------------------------------------------------------------------

typedef enum
{
	SENSITIVITY_FROM_SELF_REFLECTION			= 0U,				/* identifies the sensitivity settings from autocorrelation */
	SENSITIVITY_FROM_ANOTHER_ROBOT				= 1U				/* sensitivity for the signal received from another robot */
}SENSITIVITY_TYPE;

typedef enum
{
	SENSITIVITY_INCREMENT						= 0U,				/*  */
	SENSITIVITY_DECREMENT						= 1U				/*  */
}INC_DEC_SENSITIVITY_TYPE;

typedef struct
{
	uint8_t				ucSensitivityForSelfReflection[6];			/* value between 0 and 100. 0 means no sensitivity at all, 50 means sensitivity of 1 (no change from the measured data, 100 means twice the measured data */
	uint8_t				ucSensitivityForOtherRobot[6];				/* value between 0 and 100. 0 means no sensitivity at all, 50 means sensitivity of 1 (no change from the measured data, 100 means twice the measured data */
}SensitivityDataStructure;			/* this data structure holds the sensitivity parameters for each channel for both SELF reflection and for OTHER ROBOT, these values are also stored in the EEPROM */
//----------------------------------------------------------------------------------

#define INTERACTION_DETECTED_OTHER_ROBOT_THRESHOLD_VALUE			400
typedef struct
{
	// Measurement related parameters
	uint32_t			ui32AcquisitionDurationUS;					/* holds the time in microseconds of how much took a single ADC->DMA measurement for all the 6 channels */
	uint32_t			ui32AcquisitionRoundCounter;				/* holds the number of full acquisitions since the last reset */
	uint8_t				ucNewInteractionResultAvailable;			/* is set to 1 each time a new "InteractionResultDataStructure" data is available */

	// real values after subtracting the base line from the raw measurement
	uint32_t			ui32CorrectedDataFromSelfReflection[6];		/* data that is obtained from the raw value after subtraction of the corresponding base line value */
	uint32_t			ui32CorrectedDataFromOtherRobot[6];			/* data that is obtained from the raw value after subtraction of the corresponding base line value */
																		/* here a second correction is applied. if the value is less than "INTERACTION_DETECTED_OTHER_ROBOT_THRESHOLD_VALUE" but larger than 3 times the baseline this value will be 1 */
																		/* above this value the real difference is considered between the RAW and the (INTERACTION_DETECTED_OTHER_ROBOT_TRESHOLD_VALUE) */
																		/* below 3 times the base line this value is 0 */

	// final results including also the sensitivity parameter from "SensitivityDataStructure"
	uint32_t			ui32FinalDataFromSelfReflection[6];			/* data obtained from the "ui32CorrectedDataFromSelfReflection" and applied the sensitivity correction also */
	uint32_t			ui32FinalDataFromOtherRobot[6];				/* data obtained from the "ui32CorrectedDataFromOtherRobot" and applied the sensitivity correction also, sensitivity is applied only if the "ui32CorrectedDataFromOtherRobot" is larger than 1 */

	// final result truncated into byte value 0...255 for each data
	uint8_t				ucFinalDataFromSelfReflection[6];			/* same as "ui32FinalDataFromSelfReflection[]" but in a truncated range */
	uint8_t				ucFinalDataFromOtherRobot[6];				/* same as "ui32FinalDataFromOtherRobot[]" but in a truncated range */
}InteractionResultDataStructure;	/* this si the result value of the interaction measurement */
//----------------------------------------------------------------------------------

// ID Pattern section
#define INITIALIZATION_ID_PATTERN_MAX_BUFFER_SIZE					60		/* it determine the maximum size of the pattern, the depth of the "ucPatternBuffer" buffer */
#define INITIALIZATION_ID_PATTERN_MAX_INDEX							27		/* @ this version of the ID the maximum index is 27. [0 ... 3]=the header, [4,5 ... 14,15]=the data, [16,17 ... 26,27]=the inverted data  */
typedef struct
{
	// pattern generation related variables
	uint8_t				ucPatternBuffer[INITIALIZATION_ID_PATTERN_MAX_BUFFER_SIZE];		/* the buffer contains the 100ms-skip IR-Pattern-Change data:
																		 	 	 	 	 	 -- 0=>no-skip(=> 100ms+0ms)(used for marker to detect the start of the frame) => 5KHZ
																		 	 	 	 	 	 -- 1=>1 skip (=> 100ms+100ms) (used as LOGICAL LOW) => 2.5 KHZ
																		 	 	 	 	 	 -- 2=>2 skips (=> 100ms+200ms) (used as logical HIGH) => 1.666 KHZ */
	uint8_t				ucCurrentPattern;							/* used to be decremented during the pattern playing. it is initialized with one value of the "ucPatternBuffer" */
	uint8_t				ucPatternIndex;								/* used to index the "ucPatternBuffer" buffer during the pattern playing */

	// pattern construction related variable
	uint8_t				ucRobotAddress;								/* used to detect if during run time the robot address was change => also update the IDPattern ("ucPatternBuffer") */
}IDPatternGenerationDataStructure;

#define INTERACTION_ID_RX_TIME_TOLERANCE_US				30			/* represents the time tolerance for reception (value of 30 is the default) => 100uS=>[100-30 ... 100+30], 200uS=>[200-30 ... 200+30], 300uS=>[300-30 ... 300+30] */
#define INTERACTION_ID_RX_FINALE_STAGE_VALUE			100			/* after receiving a complete package of data this will be the value of the "ucDecodingState" =>and in the main loop it will be processed */
#define INTARACTION_ID_RX_STAGE_ERROR					200			/* if an error will be detected => the "ucDecodingState" variable will be set to this value. in the main loop all the variables will be cleared and the "ucDecodingState" set back to 0 */

#define INTERACTION_ID_RX_TIMEOUT_MS					10			/* the timeout value in ms used during the reception of the ID */
#define INTERACTION_ID_RX_EXPIRATION_TIMEOUT_MS			200		/* used to initialize the expiration counter "ui16ExpirationCounterMS" */
typedef struct
{
	// status control section
	uint8_t				ucDecodingState;							/* used to handle the state machine of the communication
	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 == 0 			=> no decoding is in progress
	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 == 1 ... 3		=> receiving the header
	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 == 4			=> receiving the data */
	uint32_t			ui32LastInterruptTimeUS;					/* the time in microseconds when the previous interrupt has occurred (from timer5) */

	// reception section
	uint8_t				ucReferenceSymbol;							/* the reference symbol is used to confirm the second symbol in the pair = {0,1,2} */
	uint8_t				ucReferenceSymbolExist;						/* each bit consists of 2 symbols, the first is the reference symbol and if the next is identical with the reference the bit is considered to be ok */
	uint8_t				ucReceivedData;
	uint8_t				ucReceivedInvertedData;
	uint8_t				ucReceptionBufferIndex;						/* used to index the "ucReceptionBuffer" reception buffer during the reception */
	//uint8_t				ucNewResultIsAvailable;						/* is set to 1 each time is a change in the detection (new robot ID was detected or the last ID expired due to to "ui16ExpirationCounterMS" time elapse) */

	// timeout section
	uint8_t				ucTimeoutCounterMS;							/* timeout counter during the reception process */
	uint16_t			ui16ExpirationCounterMS;					/* after an address is detected this counter is started, the same address needs to be re-detected within that timeout in order to still confirm the presence of the robot */

}IDPatternDetectionDataStructure;

typedef struct
{
	uint8_t				ucNewResultAvailable[6];					/* set to 1 each time a new result is available */  /* is there implemented but not used currently, only if callback will be implemented on every channel's status */
	uint8_t				ucDetectedRobotID[6];						/* is is set to 1 */
}IDPatternResultDataStructure;
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== CUSTOM_DATA_STRUCTS_END




// API
//==================================================================================================================== API_START
// new result available callback
void InteractionNewResultAvailableCallback(InteractionResultDataStructure NewInteractionResult, IDPatternResultDataStructure NewIDResult);		// implement this function in the application section if the interaction information is needed
//----------------------------------------------------------------------------------
// IR side LED's
void InteractionTurnIRPatternGenerationONorOFF(uint8_t ucIRPatternON);		//used to turn the IR pattern generation ON or OFF. if ucIRPatternON == 0 => the IR pattern generation is turned off and all the side IR LEDs are OFF
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
// Base line
// before calling this function make sure that there are no other robots around and there are no objects around to get reflection from
void InteractionCalculateBaseline(void);			// calling this function determines the baseline for each channels for both the self reflection and the signal from other robots.
//----------------------------------------------------------------------------------
BaseLineCalibrationDataStructure InteractionGetBaseLineData(void);		// calling this function will return the base line situation
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
// Sensitivity
uint8_t InteractionSetSensitivity(SENSITIVITY_TYPE ucSelfOrOther, uint8_t ucChannel, uint8_t ucValue);		// used to set the sensitivity of a channel from a given type (self or from another robot)
//----------------------------------------------------------------------------------
uint8_t	InteractionIncDecSensitivity(SENSITIVITY_TYPE ucSelfOrOther, uint8_t ucChannel, INC_DEC_SENSITIVITY_TYPE ucIncDec);	// used to increment or decrement the sensitivity, with one unit, on a particular channel
//----------------------------------------------------------------------------------
SensitivityDataStructure InteractionGetSensitivityData(void);			// used to retrieve the current sensitivity setting
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
// other
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------

// Callbacks
//----------------------------------------------------------------------------------
//void InteractionMeasurementAvailableCallback(InteractionResultData NewData);		// called from the main loop once a new set of interaction data is available (if the acquisition is set to 60 rounds => this function is called at approximately 12ms interval)
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== API_END



// SYSTEM
//==================================================================================================================== SYSTEM_START
void InteractionInit(void);				// used to initialize the Interaction Module. Called from main.c file @ init section
//----------------------------------------------------------------------------------
void InteractionMainLoop(void);			// loop function of the Interaction Module. Called from the main.c file @ main loop section
//----------------------------------------------------------------------------------
void InteractionTimer1KHZISR(void);		// 1KHZ timer function called from System ISR from stm32f4xx_it.c file @ SysTick Function (!!! keep as short as possible !!!)
//----------------------------------------------------------------------------------
void InteractionTimer10KHZISR(void);	// 10KHZ timer function called from Timer2 ISR from stm32f4xx_it.c file (!!! keep as short as possible !!!)
//----------------------------------------------------------------------------------
void InteractionADC1DMACallback();	// used to be called from the stm32f4xx_it.c once the DMA1 reads all the ADC values related to the 6 photodetector sensors
//----------------------------------------------------------------------------------
void InteractionCH0Callback();		// used to be called from the stm32f4xx_it.c once the signal is strong enough to trigger an interrupt
//----------------------------------------------------------------------------------
void InteractionCH1Callback();		// used to be called from the stm32f4xx_it.c once the signal is strong enough to trigger an interrupt
//----------------------------------------------------------------------------------
void InteractionCH2Callback();		// used to be called from the stm32f4xx_it.c once the signal is strong enough to trigger an interrupt
//----------------------------------------------------------------------------------
void InteractionCH3Callback();		// used to be called from the stm32f4xx_it.c once the signal is strong enough to trigger an interrupt
//----------------------------------------------------------------------------------
void InteractionCH4Callback();		// used to be called from the stm32f4xx_it.c once the signal is strong enough to trigger an interrupt
//----------------------------------------------------------------------------------
void InteractionCH5Callback();		// used to be called from the stm32f4xx_it.c once the signal is strong enough to trigger an interrupt
//----------------------------------------------------------------------------------
#ifdef INTERACTION_DEBUG_MODE
void InteractionDebugRX(uint8_t *ucRXBuffer, uint8_t ucRXBufferLength);		// used to receive RX data from the debug interface
void InteractionDebugRXChar(uint8_t ucRXChar);		// used to receive RX single char from the debug interface
#endif
//----------------------------------------------------------------------------------
//==================================================================================================================== SYSTEM_END



// STATIC (INTERN)
//==================================================================================================================== STATIC_START
// defined in .c file
//==================================================================================================================== STATIC_END




// Resources
/*
#define ADC_CH_0_Pin GPIO_PIN_1
#define ADC_CH_0_GPIO_Port GPIOB
#define ADC_CH_1_Pin GPIO_PIN_4
#define ADC_CH_1_GPIO_Port GPIOA
#define ADC_CH_2_Pin GPIO_PIN_5
#define ADC_CH_2_GPIO_Port GPIOA
#define ADC_CH_3_Pin GPIO_PIN_3
#define ADC_CH_3_GPIO_Port GPIOC
#define ADC_CH_4_Pin GPIO_PIN_2
#define ADC_CH_4_GPIO_Port GPIOC
#define ADC_CH_5_Pin GPIO_PIN_1
#define ADC_CH_5_GPIO_Port GPIOC

#define IR_PWM_Pin GPIO_PIN_15
#define IR_PWM_GPIO_Port GPIOA

#define IR_EN_0_Pin GPIO_PIN_12
#define IR_EN_0_GPIO_Port GPIOA
#define IR_EN_1_Pin GPIO_PIN_10
#define IR_EN_1_GPIO_Port GPIOC
#define IR_EN_2_Pin GPIO_PIN_11
#define IR_EN_2_GPIO_Port GPIOC
#define IR_EN_3_Pin GPIO_PIN_15
#define IR_EN_3_GPIO_Port GPIOC
#define IR_EN_4_Pin GPIO_PIN_9
#define IR_EN_4_GPIO_Port GPIOB
#define IR_EN_5_Pin GPIO_PIN_8
#define IR_EN_5_GPIO_Port GPIOB


#define INT_CH_0_Pin GPIO_PIN_9
#define INT_CH_0_GPIO_Port GPIOA
#define INT_CH_0_EXTI_IRQn EXTI9_5_IRQn
#define INT_CH_1_Pin GPIO_PIN_8
#define INT_CH_1_GPIO_Port GPIOD
#define INT_CH_1_EXTI_IRQn EXTI9_5_IRQn
#define INT_CH_2_Pin GPIO_PIN_2
#define INT_CH_2_GPIO_Port GPIOB
#define INT_CH_2_EXTI_IRQn EXTI2_IRQn
#define INT_CH_3_Pin GPIO_PIN_1
#define INT_CH_3_GPIO_Port GPIOE
#define INT_CH_3_EXTI_IRQn EXTI1_IRQn
#define INT_CH_4_Pin GPIO_PIN_0
#define INT_CH_4_GPIO_Port GPIOE
#define INT_CH_4_EXTI_IRQn EXTI0_IRQn
#define INT_CH_5_Pin GPIO_PIN_10
#define INT_CH_5_GPIO_Port GPIOA
#define INT_CH_5_EXTI_IRQn EXTI15_10_IRQn
 */


// resources
/* TIM2:
 * it is configured to fire UpdateEvent Interrupt @ 10KHZ rate
 * - input frequency is 90MHZ, Prescaler=0=>1, ARR=8999=>9000
 * the IR LED commands are:
 * - IR_PWM   =  PA15
 * - IR_EN_0  =  PA12
 * - IR_EN_1  =  PC10
 * - IR_EN_2  =  PC11
 * - IR_EN_3  =  PC15
 * - IR_EN_4  =  PB9
 * - IR_EN_5  =  PB8
 *
 *
 * TIM5:
 * it is let to run without interrupt @ 90MHZ. is can be used as general counting unit by
 * taking the CNT register's value whenever it is needed
 * this timer is mainly intended for pulse measurements during signal decoding
 * Pulses can be decoded with EXTI interrupts:
 * - INT_CH_0  =  EXTI_9  =  PA9 (EXTI_9__5)
 * - INT_CH_1  =  EXTI_8  =  PD8 (EXTI_9__5)
 * - INT_CH_2  =  EXTI_2  =  PB2
 * - INT_CH_3  =  EXTI_1  =  PE1
 * - INT_CH_4  =  EXTI_0  =  PE0
 * - INT_CH_5  =  EXTI_10 =  PA10 (EXTI_15__10)
 *
 *
 * ADC1:
 * it is used to measure the signal from the 6 horizontal zones in order to detect obstacles or interactions with other robots
 * it is used in scanning mode with 15 cycles sampling and 12bit resolution => time per sample = 30cycles @ 22.5MHZ => 1.33uS/ measurement
 * => for the 6 channels to be sampled => 6*1.33uS = 8uS... in reality takes 19uS
 * DMA2 Stream0 is used to store the 6 results during one scan
 * ADC1 channels are:
 * - ADC_CH_0  =  ADC1_IN9  =  PB1
 * - ADC_CH_1  =  ADC1_IN4  =  PA4
 * - ADC_CH_2  =  ADC1_IN5  =  PA5
 * - ADC_CH_3  =  ADC1_IN13 =  PC3
 * - ADC_CH_4  =  ADC1_IN12 =  PC2
 * - ADC_CH_5  =  ADC1_IN11 =  PC1
 *
 *
 */

#endif /* INC_INTERACTION_H_ */
