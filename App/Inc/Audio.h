/*
 * Audio.h
 *
 *  Created on: Feb 19, 2022
 *      Author: Arthur
 */

#ifndef INC_AUDIO_H_
#define INC_AUDIO_H_

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


//#define AUDIO_DEBUG_MODE
// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);		//  debug on PC13 HIGH
// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);  	//	debug on PC13 LOW
//====================================================================================================================

// custom data structures
//==================================================================================================================== CUSTOM_DATA_STRUCTS_START
// https://www.engineeringtoolbox.com/note-frequencies-d_520.html
// use the EXCEL: AudioSTM.xlsx
#define AUDIO_MUTE		0

#define AUDIO_ARR_C4	17202
#define AUDIO_ARR_D4	15322
#define AUDIO_ARR_E4	13653
#define AUDIO_ARR_F4	12887
#define AUDIO_ARR_G4	11480
#define AUDIO_ARR_A4	10227
#define AUDIO_ARR_B4	9111

#define AUDIO_ARR_C5	8599
#define AUDIO_ARR_D5	7662
#define AUDIO_ARR_E5	6825
#define AUDIO_ARR_F5	6442
#define AUDIO_ARR_G5	5740
#define AUDIO_ARR_A5	5114
#define AUDIO_ARR_B5	4556
//----------------------------------------------------------------------------------

#define AUDIO_MAXIMUM_PARTITURE_SIZE				30	/* the maximum number of notes in a partiture. increase this value if more notes are needed / partiture */

typedef struct
{
	uint16_t		ui16PartitureNote[AUDIO_MAXIMUM_PARTITURE_SIZE];		/* holds the ARR value for every index. according to deffines from the beginning of the file. Ex: AUDIO_ARR_C4, AUDIO_ARR_F5, ... */
	uint16_t		ui16PartitureTime[AUDIO_MAXIMUM_PARTITURE_SIZE];		/* holds the time in mS for the corresponding note. the playing of the note will always take 1 additional mS */
	uint8_t			ucPartiturePlayIndex;									/* holds the current index of the note is being played */
	uint8_t			ucPartitureEndIndex;									/* holds the end index of the last note. it must be below "AUDIO_MAXIMUM_PARTITURE_SIZE" */
	uint8_t			ucPartitureID;											/* an ID value representing the identifier of the partiture */
	uint8_t			ucPartitureIsPlaying;									/* as long as the partiture is being played this value is set to 1, else is 0 */
}AudioPartitureDataType;
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== CUSTOM_DATA_STRUCTS_END




// API
//==================================================================================================================== API_START

uint8_t AudioPlayPartiture(uint8_t ucPartitureIndex);			// used to play a partiturte with a specific index
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== API_END



// SYSTEM
//==================================================================================================================== SYSTEM_START
void AudioInit(void);				// used to initialize the Application Module. Called from main.c file @ init section
//----------------------------------------------------------------------------------
void AudioMainLoop(void);			// loop function of the Application Module. Called from the main.c file @ main loop section
//----------------------------------------------------------------------------------
void AudioTimer1KHZISR(void);		// 1KHZ timer function called from System ISR from stm32f4xx_it.c file @ SysTick Function (!!! keep as short as possible !!!)
//----------------------------------------------------------------------------------
#ifdef AUDIO_DEBUG_MODE
void AudioDebugRX(uint8_t *ucRXBuffer, uint8_t ucRXBufferLength);		// used to receive RX data from the debug interface
void AudioDebugRXChar(uint8_t ucRXChar);		// used to receive RX single char from the debug interface
#endif
//----------------------------------------------------------------------------------
//==================================================================================================================== SYSTEM_END




// STATIC (INTERN)
//==================================================================================================================== STATIC_START
// defined in .c file
//==================================================================================================================== STATIC_END




// Resources
/*
#define BUZZER_Pin GPIO_PIN_14
#define BUZZER_GPIO_Port GPIOB

TIM_HandleTypeDef htim12;		// used for Audio frequency generation: (TIM12_CH1 -> PB14 = BUZZER). used in Audio.c

 *
 * Timer12 is clocked from APB1 @ 90MHZ => with the prescaler PSC=9 => the increment clock is 90/(9+1) = 9MHZ = 111.111ns incremental frequency period:
 * ARR = [0 ... 65535] => AudioPeriod = 2*(111.111*(ARR+1))ns => Audio Frequency = 1000000000/(222.222*(ARR+1)) HZ = [4.5MHZ ... 68.66HZ]
 * ARR[HZ] = (1000000000/(222.222 * FAudio[HZ])) - 1
 *
 */

#endif /* INC_AUDIO_H_ */
