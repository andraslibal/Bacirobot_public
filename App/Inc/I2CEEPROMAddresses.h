/*
 * I2CDevices.h
 *
 *  Created on: Aug 08, 2022
 *      Author: Arthur
 */

/*		!!!!! IMPORTANT !!!!!
 *
 *   !!!!! HAL-API can ONLY be used @ initialization section to read and to write from and into the EEPROM !!!!!!
 *   : -- for READ 		=> HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
 *   : -- for WRITE		=> HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
 *   -----------------------------------------------
 *
 *   for the main loop EEPROM access the specific EEPROM API must be used to access the 24C512 EEPROM
 *   : -- for READ		=>
 *   : -- for WRITE		=>
 *   -----------------------------------------------
 *
 */


// EEPROM access explanation
//====================================================================================================================================================== EEPROM Access EX: - START
/*
 * 1) In order to access the EEPROM need to include the:
 * -- I2CDevice.h
 * -- I2CEEPROMAddresses.h
 *
 * 2) need to define 2 types of variables:
 * -- static volatile EEPROMDataTypeDef			EEpromData[32];		// data structure to hold EEPROM address and data pairs (if the EEPROM is accessed only @ initialization the variable from below is not required, and this one dose not need to be a vector but just a simple variable)
 * -- static volatile EEPROMDataListTypeDef		EEpromDataList;		// the data list that is used to operate the EEPROM in the main loop mode (only needed if also wan to access the EEPROM during the program execution not only @ the initialization section)
 *
 * 3) to access the EEPROM only from the initialization  section => use the 2 API functions:
 * -- I2CEEPROMReadIS() 	- to Read from the EEPROM
 * -- I2CEEPROMWriteIS()	- to Write into the EEPROM - both by prepare priorly the "EEpromData" variable
 *
 * 4) to access the EEPROM from inside the main loop
 * -- link the data bugger "EEpromData" to the pointer from the list variable: "EEpromDataList.pEEDTList"  => "EEpromDataList.pEEDTList = EEpromData"
 * -- define a callback function of the type: "void EEPROMDoneCallback(EEPROMDataListTypeDef EEResultList)"
 * -- build up the list according to your need Read/Write, the number of items to be read or written ..., everything inside the "EEpromDataList" variable
 * -- call the API function: "I2CEEPROMReadWriteListML()" providing the prepared list and the pointer of the callback function
 * -- after the EEPORM operation is finished the callback function will be called with the result => check the errors or if read operation was performed read out the received data from the list inside the callback
 */
//====================================================================================================================================================== EEPROM Access EX: - END


#ifndef INC_I2CEEPROMADDRESSES_H_
#define INC_I2CEEPROMADDRESSES_H_

// 24C512 EEPROM I2C Infrastructure:
// ---- hi2c1
// ---- I2C Address = 0xA0
// ---- READ:
/*
 *  HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
 *  if(HAL_I2C_Mem_Read(&hi2c1, 0xA0, 10, 2,m_ucI2CRXDataBuffer,20, HAL_MAX_DELAY) != HAL_OK)
 *	{
 *		//Error_Handler2(ERROR_CODE_I2CDEVICES_DEBUG_001);
 *	}
 */
// ---- WRITE:
/*
 *  HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
 *  if(HAL_I2C_Mem_Write(&hi2c1, 0xA0, 10, 2,m_ucI2CTXDataBuffer,6, HAL_MAX_DELAY) != HAL_OK)
 *	{
 *		//Error_Handler2(ERROR_CODE_I2CDEVICES_DEBUG_001);
 *	}
 */
//---------------------------------------------
// 24C512 address map
// all the data fields are stored as uint32 variables using exactly 4 bytes
// https://www.tme.eu/Document/8e61899813b938a80a30a994d394ca3e/m24512-w.pdf
// –- Byte write within 5 ms
// -– Page write within 5 ms
// –- 512 Kbit (64 Kbyte) of EEPROM
// -– Page size: 128 byte
//---------------------------------------------
// ===> PAGE00  -> START_ADDRESS = 0000			-			END_ADDRESS = 0127
// ===> PAGE01  -> START_ADDRESS = 0128			-			END_ADDRESS = 0255
// ===> PAGE02  -> START_ADDRESS = 0256			-			END_ADDRESS = 0383
// ===> PAGE03  -> START_ADDRESS = 0384			-			END_ADDRESS = 0511
// ===> PAGE04  -> START_ADDRESS = 0512			-			END_ADDRESS = 0639
// ===> PAGEn	-> START_ADDRESS = n*128		-			END_ADDRESS = (n*128)+127
// ===> n between 0 and 511 => 512 pages / each of 128bytes => 32 values to be stored @ uint32+t format
//--------------------------------------------

#define EEPROM_I2C_ADDRESS_ON_I2C1			0xA0					/* the I2C address of the EEPROM on the I2C1 bus */

//==================================================================================================================== INTERACTION START
// all addresses used in the Interaction.c file
//----------------------------------------------------------------------------------
// The Interaction Base Address. All addresses from interaction relates to this address
// use PAGE01
// => START_ADDRESS = 1*128 		= 128
// => END_ADDRESS	= (1*128)+127	= 255
#define EEPROM_INTERACTION_BASE_ADDRESS			128
//----------------------------------------------------------------------------------
// EEPROM Formatted address and marker
#define EEPROM_INTARACTION_FORMATTED_ADDRESS								(EEPROM_INTERACTION_BASE_ADDRESS + 4)		/* 128 + 4 = 132 */
//#define EEPROM_INTERACTION_FORMATTED_MARKER									1234		/* the value it is supposed to be found @ the EEPROM "EEPROM_INTARACTION_FORMATTED_ADDRESS" address if the EEPROM is already formatted with the interaction fields */
//----------------------------------------------------------------------------------
#define EEPROM_INTERACTION_BASE_LINE_CALIBRATION_DONE_ADDRESS				(EEPROM_INTERACTION_BASE_ADDRESS + 8)		/* 128 + 8  = 136 */
#define EEPROM_INTERACTION_BASE_LINE_CALIBRATION_MARKER						0x55AA
//----------------------------------------------------------------------------------
#define EEPROM_INTERACTION_BASE_LINE_SELF_REFLECTION_CH0_ADDRESS			(EEPROM_INTERACTION_BASE_ADDRESS + 12)		/* 128 + 12 = 140 */
#define EEPROM_INTERACTION_BASE_LINE_SELF_REFLECTION_CH1_ADDRESS			(EEPROM_INTERACTION_BASE_ADDRESS + 16)		/* 128 + 16 = 144 */
#define EEPROM_INTERACTION_BASE_LINE_SELF_REFLECTION_CH2_ADDRESS			(EEPROM_INTERACTION_BASE_ADDRESS + 20)		/* 128 + 20 = 148 */
#define EEPROM_INTERACTION_BASE_LINE_SELF_REFLECTION_CH3_ADDRESS			(EEPROM_INTERACTION_BASE_ADDRESS + 24)		/* 128 + 24 = 152 */
#define EEPROM_INTERACTION_BASE_LINE_SELF_REFLECTION_CH4_ADDRESS			(EEPROM_INTERACTION_BASE_ADDRESS + 28)		/* 128 + 28 = 156 */
#define EEPROM_INTERACTION_BASE_LINE_SELF_REFLECTION_CH5_ADDRESS			(EEPROM_INTERACTION_BASE_ADDRESS + 32)		/* 128 + 32 = 160 */
//----------------------------------------------------------------------------------
#define EEPROM_INTERACTION_BASE_LINE_ANOTHER_ROBOT_CH0_ADDRESS				(EEPROM_INTERACTION_BASE_ADDRESS + 36)		/* 128 + 36 = 164 */
#define EEPROM_INTERACTION_BASE_LINE_ANOTHER_ROBOT_CH1_ADDRESS				(EEPROM_INTERACTION_BASE_ADDRESS + 40)		/* 128 + 40 = 168 */
#define EEPROM_INTERACTION_BASE_LINE_ANOTHER_ROBOT_CH2_ADDRESS				(EEPROM_INTERACTION_BASE_ADDRESS + 44)		/* 128 + 44 = 172 */
#define EEPROM_INTERACTION_BASE_LINE_ANOTHER_ROBOT_CH3_ADDRESS				(EEPROM_INTERACTION_BASE_ADDRESS + 48)		/* 128 + 48 = 176 */
#define EEPROM_INTERACTION_BASE_LINE_ANOTHER_ROBOT_CH4_ADDRESS				(EEPROM_INTERACTION_BASE_ADDRESS + 52)		/* 128 + 52 = 180 */
#define EEPROM_INTERACTION_BASE_LINE_ANOTHER_ROBOT_CH5_ADDRESS				(EEPROM_INTERACTION_BASE_ADDRESS + 56)		/* 128 + 56 = 184 */
//----------------------------------------------------------------------------------
#define EEPROM_INTERACTION_SENSITIVITY_DATA_OK_ADDRESS						(EEPROM_INTERACTION_BASE_ADDRESS + 76)		/* 128 + 76 = 204 */
#define EEPROM_INTERACTION_SENSITIVITY_DATA_OK_MARKER						0xAA
//----------------------------------------------------------------------------------
#define EEPROM_INTERACTION_SENSITIVITY_SELF_REFLECTION_CH0_ADDRESS			(EEPROM_INTERACTION_BASE_ADDRESS + 80)		/* 128 + 80 = 208 */
#define EEPROM_INTERACTION_SENSITIVITY_SELF_REFLECTION_CH1_ADDRESS			(EEPROM_INTERACTION_BASE_ADDRESS + 84)		/* 128 + 84 = 212 */
#define EEPROM_INTERACTION_SENSITIVITY_SELF_REFLECTION_CH2_ADDRESS			(EEPROM_INTERACTION_BASE_ADDRESS + 88)		/* 128 + 88 = 216 */
#define EEPROM_INTERACTION_SENSITIVITY_SELF_REFLECTION_CH3_ADDRESS			(EEPROM_INTERACTION_BASE_ADDRESS + 92)		/* 128 + 92 = 220 */
#define EEPROM_INTERACTION_SENSITIVITY_SELF_REFLECTION_CH4_ADDRESS			(EEPROM_INTERACTION_BASE_ADDRESS + 96)		/* 128 + 96 = 224 */
#define EEPROM_INTERACTION_SENSITIVITY_SELF_REFLECTION_CH5_ADDRESS			(EEPROM_INTERACTION_BASE_ADDRESS + 100)	/* 128 + 100 = 228 */
//----------------------------------------------------------------------------------
#define EEPROM_INTERACTION_SENSITIVITY_ANOTHER_ROBOT_CH0_ADDRESS			(EEPROM_INTERACTION_BASE_ADDRESS + 104)	/* 128 + 104 = 232 */
#define EEPROM_INTERACTION_SENSITIVITY_ANOTHER_ROBOT_CH1_ADDRESS			(EEPROM_INTERACTION_BASE_ADDRESS + 108)	/* 128 + 108 = 236 */
#define EEPROM_INTERACTION_SENSITIVITY_ANOTHER_ROBOT_CH2_ADDRESS			(EEPROM_INTERACTION_BASE_ADDRESS + 112)	/* 128 + 112 = 240 */
#define EEPROM_INTERACTION_SENSITIVITY_ANOTHER_ROBOT_CH3_ADDRESS			(EEPROM_INTERACTION_BASE_ADDRESS + 116)	/* 128 + 116 = 244 */
#define EEPROM_INTERACTION_SENSITIVITY_ANOTHER_ROBOT_CH4_ADDRESS			(EEPROM_INTERACTION_BASE_ADDRESS + 120)	/* 128 + 120 = 248 */
#define EEPROM_INTERACTION_SENSITIVITY_ANOTHER_ROBOT_CH5_ADDRESS			(EEPROM_INTERACTION_BASE_ADDRESS + 124)	/* 128 + 124 = 252 */
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== INTERACTION END





#endif
