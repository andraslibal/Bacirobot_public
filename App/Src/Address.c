#include "main.h"

#include "stdint.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"

//#include "WS2812.h"
//#include "Wheel.h"

#include "Address.h"


// STATIC (INTERN) prototypes
//==================================================================================================================== STATIC_START
// variables - external
//----------------------------------------------------------------------------------


// variables - internal
#ifdef ADDRESS_DEBUG_MODE
uint8_t m_ucDebugBuffer[100];
#endif
#define ADDRESS_SKIP_REAL_TIME		2000
uint16_t m_uiAddressSkipRealTime;	// used in the main loop to skip real time
//----------------------------------------------------------------------------------

// address related (will be exported)
uint8_t m_ucRobotAddress;			// holds the address configured with the on board switches. it is externed to be available in other *.c files
//----------------------------------------------------------------------------------


// prototypes

//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== STATIC_END


// API
//==================================================================================================================== API_START
void AddressPowerOffRobot(void)			// used to power off the device
{	// pin PE4 can turn off the power of the unit. by setting the PE4 as OUTPUT_PP and setting a logic level of LOW the device will turn off
	GPIO_InitTypeDef MyGPIOSetting = {0};

	MyGPIOSetting.Pin 	= GPIO_PIN_4;
	MyGPIOSetting.Mode	= GPIO_MODE_OUTPUT_PP;
	MyGPIOSetting.Speed	= GPIO_SPEED_LOW;
	MyGPIOSetting.Pull	= GPIO_NOPULL;

	HAL_GPIO_Init(GPIOE, &MyGPIOSetting);

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);

	return;
}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== API_END


// SYSTEM
//==================================================================================================================== SYSTEM_START
void AddressInit(void)	// used to initialize the Address Module. Called from main.c file @ init section
{
	m_ucRobotAddress = 0;

	return;
}
//----------------------------------------------------------------------------------
void AddressMainLoop(void)	// loop function of the Address Module. Called from the main.c file @ main loop section
{
	// skip real time
	if(m_uiAddressSkipRealTime)
	{
		m_uiAddressSkipRealTime --;
		return;
	}
	m_uiAddressSkipRealTime = ADDRESS_SKIP_REAL_TIME;
	//--------------------------------------------------------


	// get the address
	uint8_t ucTempAddress = 0;

	if(HAL_GPIO_ReadPin(ADD0_GPIO_Port, ADD0_Pin) == GPIO_PIN_RESET)	ucTempAddress |= (1<<0);
	if(HAL_GPIO_ReadPin(ADD1_GPIO_Port, ADD1_Pin) == GPIO_PIN_RESET)	ucTempAddress |= (1<<1);
	if(HAL_GPIO_ReadPin(ADD2_GPIO_Port, ADD2_Pin) == GPIO_PIN_RESET)	ucTempAddress |= (1<<2);
	if(HAL_GPIO_ReadPin(ADD3_GPIO_Port, ADD3_Pin) == GPIO_PIN_RESET)	ucTempAddress |= (1<<3);
	if(HAL_GPIO_ReadPin(ADD4_GPIO_Port, ADD4_Pin) == GPIO_PIN_RESET)	ucTempAddress |= (1<<4);
	if(HAL_GPIO_ReadPin(ADD5_GPIO_Port, ADD5_Pin) == GPIO_PIN_RESET)	ucTempAddress |= (1<<5);
	if(HAL_GPIO_ReadPin(ADD6_GPIO_Port, ADD6_Pin) == GPIO_PIN_RESET)	ucTempAddress |= (1<<6);
	if(HAL_GPIO_ReadPin(ADD7_GPIO_Port, ADD7_Pin) == GPIO_PIN_RESET)	ucTempAddress |= (1<<7);

	if(ucTempAddress != m_ucRobotAddress)
	{
		m_ucRobotAddress = ucTempAddress;

		#ifdef ADDRESS_DEBUG_MODE
			sprintf((char *)m_ucDebugBuffer,"Robot Address = %d\r\n",m_ucRobotAddress);
			DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
		#endif

	}



	return;
}
//----------------------------------------------------------------------------------
void AddressTimer1KHZISR(void)	// 1KHZ timer function called from System ISR from stm32f4xx_it.c file @ SysTick Function (!!! keep as short as possible !!!)
{


	return;
}
//----------------------------------------------------------------------------------
#ifdef ADDRESS_DEBUG_MODE
void AddressDebugRXString(uint8_t *ucRXBuffer, uint8_t ucRXBufferLength)	// used to receive RX data from the debug interface
{

	return;
}
void AddressDebugRXChar(uint8_t ucRXBuffer)	// used to receive RX char from the debug interface
{
	if(ucRXBuffer == 'i')
	{
		sprintf((char *)m_ucDebugBuffer,"Robot Address = %d\r\n",m_ucRobotAddress);
		DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
	}
	if(ucRXBuffer == 'f')
	{	// turn robot OFF
		AddressPowerOffRobot();
	}
	return;
}
#endif
//----------------------------------------------------------------------------------
//==================================================================================================================== SYSTEM_END


// STATIC (INTERN) implementation
//==================================================================================================================== STATIC_START

//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== STATIC_END
