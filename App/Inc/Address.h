/*
 * Address.h
 *
 *  Created on: Apr 12, 2022
 *      Author: Arthur
 */

#ifndef INC_ADDRESS_H_
#define INC_ADDRESS_H_


/* hardware
#define ADD0_Pin GPIO_PIN_0
#define ADD0_GPIO_Port GPIOD

#define ADD1_Pin GPIO_PIN_1
#define ADD1_GPIO_Port GPIOD

#define ADD2_Pin GPIO_PIN_2
#define ADD2_GPIO_Port GPIOD

#define ADD3_Pin GPIO_PIN_3
#define ADD3_GPIO_Port GPIOD

#define ADD4_Pin GPIO_PIN_4
#define ADD4_GPIO_Port GPIOD

#define ADD5_Pin GPIO_PIN_7
#define ADD5_GPIO_Port GPIOD

#define ADD6_Pin GPIO_PIN_4
#define ADD6_GPIO_Port GPIOB

#define ADD7_Pin GPIO_PIN_5
#define ADD7_GPIO_Port GPIOB
 */


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


//#define ADDRESS_DEBUG_MODE
//====================================================================================================================

// custom data structures
//==================================================================================================================== CUSTOM_DATA_STRUCTS_START

//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== CUSTOM_DATA_STRUCTS_END




// API
//==================================================================================================================== API_START
void AddressPowerOffRobot(void);			// used to power off the device
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== API_END



// SYSTEM
//==================================================================================================================== SYSTEM_START
void AddressInit(void);				// used to initialize the Address Module. Called from main.c file @ init section
//----------------------------------------------------------------------------------
void AddressMainLoop(void);			// loop function of the Address Module. Called from the main.c file @ main loop section
//----------------------------------------------------------------------------------
void AddressTimer1KHZISR(void);		// 1KHZ timer function called from System ISR from stm32f4xx_it.c file @ SysTick Function (!!! keep as short as possible !!!)
//----------------------------------------------------------------------------------
#ifdef ADDRESS_DEBUG_MODE
void AddressDebugRXString(uint8_t *ucRXBuffer, uint8_t ucRXBufferLength);		// used to receive RX data from the debug interface
void AddressDebugRXChar(uint8_t ucRXBuffer);		// used to receive RX char from the debug interface
#endif
//----------------------------------------------------------------------------------
//==================================================================================================================== SYSTEM_END



// STATIC (INTERN)
//==================================================================================================================== STATIC_START
// defined in .c file
//==================================================================================================================== STATIC_END

#endif /* INC_ADDRESS_H_ */
