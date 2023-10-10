/*
 * VL53l0x.c
 *
 *  Created on: Aug 15, 2022
 *      Author: Arthur
 */


#include "main.h"

#include "stdint.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"

//#include "WS2812.h"
//#include "Wheel.h"

#include "VL53l0x.h"


// STATIC (INTERN) prototypes
//==================================================================================================================== STATIC_START
// variables - external
extern UART_HandleTypeDef huart3;		// for communication with the VL53L0x
//----------------------------------------------------------------------------------


// variables - internal
#ifdef VL53L0X_DEBUG_MODE
static uint8_t m_ucDebugBuffer[200];
#endif
#define VL53L0X_SKIP_REAL_TIME		2000
uint16_t m_uiVL53l0xSkipRealTime;	// used in the main loop to skip real time

// Data reception variables
#define VL53L0X_RX_COMPLETED_INDEX			100				/* after the RX has completed => the "m_ucRXBufferIndex" variable will get this value */
static volatile uint8_t		m_ucRXChar;						// used to receive the character form the VL53L0X sensor
static volatile uint8_t		m_ucRXDataBuffer[50];			// buffer to concatenate the RX data
static volatile uint8_t		m_ucRXBufferIndex;				// used to handle the reception and to index and fill the RX buffer during the reception
static volatile uint8_t		m_ucRXTimeoutMS;				// used to detect timeout situation during an on-going reception

// timer variable
static volatile uint16_t	m_ui16TimerDivider1KHZ;			// used to divide the 1KHz timer to receive the 10 HZ interrogation interval

// measurement result
static volatile uint16_t 	m_ui16VL53L0XResult;			// contains the measurement result;
//----------------------------------------------------------------------------------


// prototypes
static void RXConcatenate(void);			// received data is available in the global variable "m_ucRXChar"
//----------------------------------------------------------------------------------
static void RXDecodeData(void);				// decodes the received data from the reception buffer
//----------------------------------------------------------------------------------
static void RXReset(void);					// clears the RX parameters in order to start another reception
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== STATIC_END


// API
//==================================================================================================================== API_START
// callback
__attribute__ ((weak)) void VL53l0xReslutCallback(uint16_t ui16BL53l0xResultMM)			// if implemented in the client *.c file it will be called every time a new result is available
{	// called every time a new measurement result is available
	// -- ui16BL53l0xResultMM	== 0	=> measurement error, no data is available
	// -- ui16BL53l0xResultMM	!= 0	=> the distance in [mm] to the detected object
	//-------------------------------


	#ifdef VL53L0X_DEBUG_MODE
	if(ui16BL53l0xResultMM)
	{	// measurement available
		sprintf((char * )m_ucDebugBuffer,"VL53: Dist=%u [mm]\r\n",ui16BL53l0xResultMM);
	}
	else
	{	// error, no data available
		sprintf((char * )m_ucDebugBuffer,"VL53: Error, No Data\r\n");
	}
	DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
	#endif

	return;
}
//----------------------------------------------------------------------------------
// normal
uint16_t VL53l0xGetResult(void)			// returns the measured result in [mm] units. if 0=> no measurement is available
{
	return m_ui16VL53L0XResult;
}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== API_END


// SYSTEM
//==================================================================================================================== SYSTEM_START
void VL53l0xInit(void)	// used to initialize the VL53l0x Module. Called from main.c file @ init section
{
	// trigger the first reception
	HAL_UART_Receive_IT(&huart3, (uint8_t*)&m_ucRXChar, 1);

	return;
}
//----------------------------------------------------------------------------------
void VL53l0xMainLoop(void)	// loop function of the VL53l0x Module. Called from the main.c file @ main loop section
{
	uint8_t ucTXData;

	// skip real time
	if(m_uiVL53l0xSkipRealTime)
	{
		m_uiVL53l0xSkipRealTime --;
		return;
	}
	m_uiVL53l0xSkipRealTime = VL53L0X_SKIP_REAL_TIME;
	//--------------------------------------------------------

	// ------------ TX section ----------------
	if( m_ui16TimerDivider1KHZ == 0 )
	{	// trigger another interrogation
		ucTXData = 'i';
		// HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size, uint32_t Timeout)
		// HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size)
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);	//  debug on PC13 HIGH
		//HAL_UART_Transmit(&huart3,&ucTXData,1,HAL_MAX_DELAY);
		HAL_UART_Transmit_IT(&huart3,&ucTXData,1);
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);  //	debug on PC13 LOW
		m_ui16TimerDivider1KHZ = 999;
	}
	// ----------------------------------------


	// ------------- RX Section ---------------
	// check RX timeout
	if( (m_ucRXTimeoutMS == 0) && (m_ucRXBufferIndex != 0) )
	{	// timeout occurred during reception
		RXReset();
	}
	// decode received data
	if( (m_ucRXBufferIndex == VL53L0X_RX_COMPLETED_INDEX) )
	{	// reception has completed
		RXDecodeData();
		RXReset();
		m_ui16TimerDivider1KHZ = 99;
	}
	// ----------------------------------------


	// ----------------------------------------
	// ----------------------------------------


	return;
}
//----------------------------------------------------------------------------------
void VL53l0xTimer1KHZISR(void)	// 1KHZ timer function called from System ISR from stm32f4xx_it.c file @ SysTick Function (!!! keep as short as possible !!!)
{
	if( m_ui16TimerDivider1KHZ )	m_ui16TimerDivider1KHZ --;		// for TX re-transmission
	if( m_ucRXTimeoutMS )			m_ucRXTimeoutMS --;				// for RX timeout
	return;
}
//----------------------------------------------------------------------------------
void VL53l0xRXComplete(void)		// will be called from the RX complete callback from main.c
{
	// 1 byte was received
	m_ucRXTimeoutMS = 10;		// set another 10ms timeout
	RXConcatenate();			// add m_ucRXChar to the RX buffer

	// trigger another reception
	HAL_UART_Receive_IT(&huart3, (uint8_t*)&m_ucRXChar, 1);
	return;
}
//----------------------------------------------------------------------------------
#ifdef VL53L0X_DEBUG_MODE
void VL53l0xDebugRXString(uint8_t *ucRXBuffer, uint8_t ucRXBufferLength)	// used to receive RX data from the debug interface
{

	return;
}
void VL53l0xDebugRXChar(uint8_t ucRXBuffer)	// used to receive RX char from the debug interface
{

	return;
}
#endif
//----------------------------------------------------------------------------------
//==================================================================================================================== SYSTEM_END


// STATIC (INTERN) implementation
//==================================================================================================================== STATIC_START
static void RXConcatenate(void)			// received data is available in the global variable "m_ucRXChar"
{
	if( (m_ucRXBufferIndex == VL53L0X_RX_COMPLETED_INDEX) )	return;	// some data is already available tin the RXBuffer just needs to be decoded

	if( (m_ucRXChar == '\n') || (m_ucRXChar == '\r') )
	{	// some end of row char was received
		if( m_ucRXBufferIndex == 0 )
		{	// just skip, is the start of the buffer
			return;
		}
		else
		{	// is the end of the buffer
			m_ucRXDataBuffer[m_ucRXBufferIndex] = 0;	// end of the string
			m_ucRXBufferIndex = VL53L0X_RX_COMPLETED_INDEX;
			//in main call the RXDecodeData();
		}
	}
	else
	{	// some RX data from the communication
		m_ucRXDataBuffer[m_ucRXBufferIndex] = m_ucRXChar;
		m_ucRXBufferIndex ++;
		if( (m_ucRXBufferIndex > 49) )
		{	// too many data => reception error
			RXReset();
		}
	}
	return;
}
//----------------------------------------------------------------------------------
static void RXDecodeData(void)				// decodes the received data from the reception buffer
{
	uint16_t ui16Temp;

	if( (m_ucRXDataBuffer[0] == 'M') && (m_ucRXDataBuffer[1] == ':') )
	{	// measurement result
		ui16Temp = (uint16_t)atoi((char *)&m_ucRXDataBuffer[2]);
		m_ui16VL53L0XResult = ui16Temp;
		#ifdef VL53L0X_DEBUG_MODE
		//sprintf((char * )m_ucDebugBuffer,"VL53: Measurement: %u\r\n",ui16Temp);
		#endif
	}
	else if( (m_ucRXDataBuffer[0] == 'E') && (m_ucRXDataBuffer[1] == ':') )
	{	// error
		m_ui16VL53L0XResult = 0;
		#ifdef VL53L0X_DEBUG_MODE
		//sprintf((char * )m_ucDebugBuffer,"VL53: Error\r\n");
		#endif
	}
	else
	{	// unknown string
		m_ui16VL53L0XResult = 0;
		#ifdef VL53L0X_DEBUG_MODE
		//sprintf((char * )m_ucDebugBuffer,"VL53: Unknown\r\n");
		#endif
	}

	VL53l0xReslutCallback(m_ui16VL53L0XResult);

	#ifdef VL53L0X_DEBUG_MODE
	//sprintf((char * )m_ucDebugBuffer,"VL53: >%s<\r\n",m_ucRXDataBuffer);
	//DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
	#endif

	return;
}
//----------------------------------------------------------------------------------
static void RXReset(void)					// clears the RX parameters in order to start another reception
{
	uint8_t i;

	for(i=0; i<50; i++)		m_ucRXDataBuffer[i] = 0;
	m_ucRXBufferIndex = 0;
	return;
}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== STATIC_END
