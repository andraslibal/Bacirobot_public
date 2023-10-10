/*
 * Audio.c
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

#include "Audio.h"


// STATIC (INTERN) prototypes
//==================================================================================================================== STATIC_START
// variables - external
extern TIM_HandleTypeDef htim12;
//----------------------------------------------------------------------------------


// variables - internal
#ifdef AUDIO_DEBUG_MODE
static uint8_t m_ucDebugBuffer[100];
static uint8_t m_ucDebugTemp;
#endif
#define AUDIO_SKIP_REAL_TIME		2000
uint16_t m_uiAudioSkipRealTime;	// used in the main loop to skip real time
//----------------------------------------------------------------------------------
#define AUDIO_MAXIMUM_NUMBER_OF_PARTITURES		10		/* increase this value if more partitures are needed */
static volatile AudioPartitureDataType			PartitureBuffer[AUDIO_MAXIMUM_NUMBER_OF_PARTITURES] = {0};	// set of 10 partitures
static volatile AudioPartitureDataType			PlayingPartiture = {0};		// represents the partiture that is currently being played
static volatile uint8_t							m_ucStartPlayDone = 0;
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------


// prototypes

//----------------------------------------------------------------------------------
static void AudioOff(void);	// used torn off the buzzer
//----------------------------------------------------------------------------------
static void AudioOn(void);	// used to connect the timer to the Buzzer
//----------------------------------------------------------------------------------
#ifdef AUDIO_DEBUG_MODE
static void PartitureInit(void);						// it is used to intialize the partitures
#endif
//----------------------------------------------------------------------------------
static void LoadPartiture(uint8_t  ucPartitureIndex);			// it is used to load a partiture from the partiture buffer "PartitureBuffer[]" into the "PlayingPartiture" variable and start playing that partiture
//----------------------------------------------------------------------------------
static void PlayPartitureTimer1KHZISR(void);					// called from the timer to play the active partiture
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
#ifdef AUDIO_DEBUG_MODE
static void PrintInfoMainLoop(void);		// used to print the timer12 setting
#endif
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== STATIC_END


// API
//==================================================================================================================== API_START

uint8_t AudioPlayPartiture(uint8_t ucPartitureIndex)			// used to play a partiturte with a specific index
{
	if( (ucPartitureIndex >= AUDIO_MAXIMUM_NUMBER_OF_PARTITURES) )		return 1;	// out of range partiture index

	if( (PlayingPartiture.ucPartitureIsPlaying == 1) )
	{
		if( (ucPartitureIndex == PlayingPartiture.ucPartitureID) )		return 2;	// already playing
		else
		{
			PlayingPartiture.ucPartitureIsPlaying = 0;
			LoadPartiture(ucPartitureIndex);
			return 0;	// stop current partiture and play another
		}
	}
	else
	{	// nothing is playing => start a partiture
		LoadPartiture(ucPartitureIndex);
		return 0;
	}
}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== API_END


// SYSTEM
//==================================================================================================================== SYSTEM_START
void AudioInit(void)	// used to initialize the Audio Module. Called from main.c file @ init section
{
	// turn off the buzzer
	AudioOff();	// disable the buzzer

	// HAL_StatusTypeDef HAL_TIM_OC_Start(TIM_HandleTypeDef *htim, uint32_t Channel)
	if(HAL_TIM_OC_Start(&htim12,TIM_CHANNEL_1) != HAL_OK)	// start the audio timer (TIM12_CH1)
	{
		Error_Handler2(ERROR_CODE_AUDIO_INIT_001);
		return;
	}

	#ifdef AUDIO_DEBUG_MODE
	PartitureInit();
	#endif


	return;
}
//----------------------------------------------------------------------------------
void AudioMainLoop(void)	// loop function of the Audio Module. Called from the main.c file @ main loop section
{

	// skip real time
	if(m_uiAudioSkipRealTime)
	{
		m_uiAudioSkipRealTime --;
		return;
	}
	m_uiAudioSkipRealTime = AUDIO_SKIP_REAL_TIME;
	//--------------------------------------------------------

	#ifdef AUDIO_DEBUG_MODE
	PrintInfoMainLoop();
	#endif

	if( m_ucStartPlayDone == 0 )
	{
		AudioPlayPartiture(1);
		m_ucStartPlayDone = 1;
	}







	return;
}
//----------------------------------------------------------------------------------
void AudioTimer1KHZISR(void)	// 1KHZ timer function called from System ISR from stm32f4xx_it.c file @ SysTick Function (!!! keep as short as possible !!!)
{
	PlayPartitureTimer1KHZISR();

	return;
}
//----------------------------------------------------------------------------------
#ifdef AUDIO_DEBUG_MODE
void AudioDebugRX(uint8_t *ucRXBuffer, uint8_t ucRXBufferLength)	// used to receive RX data from the debug interface
{
	return;
}
void AudioDebugRXChar(uint8_t ucRXChar)	// used to receive RX single char from the debug interface
{
	if( ucRXChar == 'q' )
	{
		// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);			//  debug on PC13 HIGH
		// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);  		//	debug on PC13 LOW
		return;
	}

	if( ucRXChar == 'e' )
	{
		// enable audio
		AudioOn();
		return;
	}
	if( ucRXChar == 'd' )
	{
		// disable audio
		AudioOff();
		return;
	}
	if( ucRXChar == 'i' )
	{
		// print info
		m_ucDebugTemp = 1;
		return;
	}

	if( ucRXChar == 'a' )
	{
		// set arr
		(&htim12)->Instance->ARR = AUDIO_ARR_C5;
		return;
	}
	if( ucRXChar == 'A' )
	{
		// set arr
		__HAL_TIM_SET_AUTORELOAD(&htim12, AUDIO_ARR_C4);
		return;
	}

	if( ucRXChar == 'z' )
	{
		// play partiture 0
		AudioPlayPartiture(0);
		return;
	}

	if( ucRXChar == 'Z' )
	{
		// play partiture 1
		AudioPlayPartiture(1);
		return;
	}



	return;
}
#endif
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== SYSTEM_END




// STATIC (INTERN) implementation
//==================================================================================================================== STATIC_START

//----------------------------------------------------------------------------------
// #define BUZZER_Pin GPIO_PIN_14
// #define BUZZER_GPIO_Port GPIOB
static void AudioOff(void)	// used torn off the buzzer
{	// BUZZER is PB14
	/*
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = BUZZER_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_TIM12;
    HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);
	*/

	// PB14 = input => MODER[29][28] = [0,0] => cleared
	BUZZER_GPIO_Port->MODER &= ~(0x03 << 28U) ;

	return;
}
//----------------------------------------------------------------------------------
static void AudioOn(void)	// used to connect the timer to the Buzzer
{
	/*
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = BUZZER_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_TIM12;
    HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);
	*/

	// PB14 = alternative function mode => MODE[29][28] = [1,0]
	BUZZER_GPIO_Port->MODER |= (0x02 << 28U);

	return;
}
//----------------------------------------------------------------------------------
#ifdef AUDIO_DEBUG_MODE
static void PartitureInit(void)						// it is used to intialize the partitures
{
	uint8_t ucPartitureIndex;

	// ------------------ Partiture 00 ------------------------
	ucPartitureIndex = 0;

	//---------
	PartitureBuffer[ucPartitureIndex].ucPartitureID = ucPartitureIndex;

	PartitureBuffer[ucPartitureIndex].ui16PartitureTime[0] 		= 50;				// time in mS
	PartitureBuffer[ucPartitureIndex].ui16PartitureNote[0]		= AUDIO_ARR_C5;		// note from Audio.h

	PartitureBuffer[ucPartitureIndex].ui16PartitureTime[1] 		= 50;				// time in mS
	PartitureBuffer[ucPartitureIndex].ui16PartitureNote[1]		= AUDIO_ARR_D5;		// note from Audio.h

	PartitureBuffer[ucPartitureIndex].ui16PartitureTime[2] 		= 50;				// time in mS
	PartitureBuffer[ucPartitureIndex].ui16PartitureNote[2]		= AUDIO_ARR_C5;		// note from Audio.h

	PartitureBuffer[ucPartitureIndex].ui16PartitureTime[3] 		= 50;				// time in mS
	PartitureBuffer[ucPartitureIndex].ui16PartitureNote[3]		= AUDIO_ARR_D5;		// note from Audio.h

	PartitureBuffer[ucPartitureIndex].ui16PartitureTime[4] 		= 50;				// time in mS
	PartitureBuffer[ucPartitureIndex].ui16PartitureNote[4]		= AUDIO_ARR_C5;		// note from Audio.h

	PartitureBuffer[ucPartitureIndex].ui16PartitureTime[5] 		= 50;				// time in mS
	PartitureBuffer[ucPartitureIndex].ui16PartitureNote[5]		= AUDIO_ARR_D5;		// note from Audio.h

	PartitureBuffer[ucPartitureIndex].ui16PartitureTime[6] 		= 100;				// time in mS
	PartitureBuffer[ucPartitureIndex].ui16PartitureNote[6]		= AUDIO_MUTE;		// note from Audio.h

	PartitureBuffer[ucPartitureIndex].ucPartitureEndIndex = 6;		// the index of the last note
	// -----------
	// --------------------------------------------------------

	// ------------------ Partiture 01 ------------------------
	ucPartitureIndex = 1;

	//---------
	PartitureBuffer[ucPartitureIndex].ucPartitureID = ucPartitureIndex;

	PartitureBuffer[ucPartitureIndex].ui16PartitureTime[0] 		= 50;				// time in mS
	PartitureBuffer[ucPartitureIndex].ui16PartitureNote[0]		= AUDIO_ARR_C5;		// note from Audio.h

	PartitureBuffer[ucPartitureIndex].ui16PartitureTime[1] 		= 50;				// time in mS
	PartitureBuffer[ucPartitureIndex].ui16PartitureNote[1]		= AUDIO_ARR_E5;		// note from Audio.h

	PartitureBuffer[ucPartitureIndex].ui16PartitureTime[2] 		= 50;				// time in mS
	PartitureBuffer[ucPartitureIndex].ui16PartitureNote[2]		= AUDIO_ARR_D5;		// note from Audio.h

	PartitureBuffer[ucPartitureIndex].ui16PartitureTime[3] 		= 50;				// time in mS
	PartitureBuffer[ucPartitureIndex].ui16PartitureNote[3]		= AUDIO_ARR_G5;		// note from Audio.h

	PartitureBuffer[ucPartitureIndex].ui16PartitureTime[4] 		= 50;				// time in mS
	PartitureBuffer[ucPartitureIndex].ui16PartitureNote[4]		= AUDIO_ARR_F5;		// note from Audio.h

	PartitureBuffer[ucPartitureIndex].ui16PartitureTime[5] 		= 50;				// time in mS
	PartitureBuffer[ucPartitureIndex].ui16PartitureNote[5]		= AUDIO_ARR_B5;		// note from Audio.h

	PartitureBuffer[ucPartitureIndex].ui16PartitureTime[6] 		= 50;				// time in mS
	PartitureBuffer[ucPartitureIndex].ui16PartitureNote[6]		= AUDIO_ARR_A5;		// note from Audio.h

	PartitureBuffer[ucPartitureIndex].ucPartitureEndIndex = 6;		// the index of the last note
	// -----------
	// --------------------------------------------------------

	// ------------------ Partiture 02 ------------------------
	ucPartitureIndex = 2;
	// --------------------------------------------------------

	// ------------------ Partiture 03 ------------------------
	ucPartitureIndex = 3;
	// --------------------------------------------------------

	// ------------------ Partiture 04 ------------------------
	ucPartitureIndex = 4;
	// --------------------------------------------------------

	// ------------------ Partiture 05 ------------------------
	// --------------------------------------------------------
	// --------------------------------------------------------
	// --------------------------------------------------------
	// --------------------------------------------------------

	return;
}
#endif
//----------------------------------------------------------------------------------
static void LoadPartiture(uint8_t  ucPartitureIndex)			// it is used to load a partiture from the partiture buffer "PartitureBuffer[]" into the "PlayingPartiture" variable and start playing that partiture
{
	uint8_t i;

	if( (ucPartitureIndex >= AUDIO_MAXIMUM_NUMBER_OF_PARTITURES) )		return;		//index out of range

	if( (PartitureBuffer[ucPartitureIndex].ui16PartitureNote[0] == 0) )	return;		// this partiture is empty

	for(i=0;i<AUDIO_MAXIMUM_PARTITURE_SIZE;i++)
	{
		PlayingPartiture.ui16PartitureNote[i]	= PartitureBuffer[ucPartitureIndex].ui16PartitureNote[i];
		PlayingPartiture.ui16PartitureTime[i]	= PartitureBuffer[ucPartitureIndex].ui16PartitureTime[i];
	}
	PlayingPartiture.ucPartitureID = ucPartitureIndex;

	PlayingPartiture.ucPartitureEndIndex = PartitureBuffer[ucPartitureIndex].ucPartitureEndIndex;

	PlayingPartiture.ucPartiturePlayIndex = 255;	// indicating to start playing

	PlayingPartiture.ucPartitureIsPlaying = 0;	// not playing yet but it will start from "PlayPartitureTimer1KHZISR()" function

	return;
}
//----------------------------------------------------------------------------------
static void PlayPartitureTimer1KHZISR(void)					// called from the timer to play the active partiture
{
	// check if a partiture is about to start
	if( (PlayingPartiture.ucPartiturePlayIndex == 255) )
	{
		PlayingPartiture.ucPartiturePlayIndex = 0;
		PlayingPartiture.ucPartitureIsPlaying = 1;

		if( (PlayingPartiture.ui16PartitureNote[PlayingPartiture.ucPartiturePlayIndex] != AUDIO_MUTE) )
		{	// play the note if it is not mute time
			__HAL_TIM_SET_AUTORELOAD(&htim12, PlayingPartiture.ui16PartitureNote[PlayingPartiture.ucPartiturePlayIndex]);
			AudioOn();
		}
		return;
	}

	// check if a partiture is being played
	if( (PlayingPartiture.ucPartitureIsPlaying == 1) )
	{
		if( (PlayingPartiture.ui16PartitureTime[PlayingPartiture.ucPartiturePlayIndex] > 0) )
		{	// still playing
			PlayingPartiture.ui16PartitureTime[PlayingPartiture.ucPartiturePlayIndex] --;
		}
		else
		{	// end of the current note
			// check if the current note was not the last note from the partiture
			if( (PlayingPartiture.ucPartiturePlayIndex < PlayingPartiture.ucPartitureEndIndex) )
			{	// still more notes to be played => take the next one

				PlayingPartiture.ucPartiturePlayIndex ++;
				if( (PlayingPartiture.ui16PartitureNote[PlayingPartiture.ucPartiturePlayIndex] == AUDIO_MUTE) )
				{	// mute time
					AudioOff();
				}
				else
				{	// play the note
					__HAL_TIM_SET_AUTORELOAD(&htim12, PlayingPartiture.ui16PartitureNote[PlayingPartiture.ucPartiturePlayIndex]);
					AudioOn();
				}

			}
			else
			{	// the end of the partiture
				PlayingPartiture.ucPartitureIsPlaying = 0;
				AudioOff();
			}
		}
	}

	return;
}
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
#ifdef AUDIO_DEBUG_MODE
static void PrintInfoMainLoop(void)		// used to print the timer12 setting
{
	uint16_t uiA, uiB, uiC, uia, uic;

	// ------------------------------------------
	if(m_ucDebugTemp == 1)
	{	// print info
		m_ucDebugTemp = 0;

		uiA = __HAL_TIM_GET_AUTORELOAD(&htim12);
		uia = (&htim12)->Instance->ARR;
		//uiB = __HAL_TIM_GET_CLOCKDIVISION(&htim12);
		uiB = (&htim12)->Instance->PSC;
		uiC = __HAL_TIM_GET_COMPARE(&htim12,TIM_CHANNEL_1);
		uic = (&htim12)->Instance->CCR1;

		sprintf((char * )m_ucDebugBuffer,"Audio_i: ARR=%u,\t PRESC=%u,\t TIM12_CCR1=%u\t %u\t %u\r\n",uiA, uiB, uiC, uia, uic);
		DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));

		return;
	}
	// ------------------------------------------


	// ------------------------------------------
	// ------------------------------------------
	// ------------------------------------------
	// ------------------------------------------


	return;
}
#endif
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== STATIC_END
