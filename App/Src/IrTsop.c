/*
 * IrTsop.c
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

#include "IrTsop.h"


// STATIC (INTERN) prototypes
//==================================================================================================================== STATIC_START
// variables - external
extern TIM_HandleTypeDef htim8;		// used in IrTsop to generate the 56KHZ IR modulation (TIM8_CH3 -> PC8) (initialized here)
extern TIM_HandleTypeDef htim5;		// 1uS/Increment, started in IrTsop.c, !! used as general purpose timer. just to go around up to max and start over. without interrupt: Interaction.c, IrTsop.c,
// uses also the htim2 -> as the 10KHZ ISR() function. the Timer2 is initialized in Interaction.c
extern uint8_t m_ucRobotAddress;	// defined in Address.c and it contains the Robot Address
//----------------------------------------------------------------------------------


// variables - internal
#ifdef IRTSOP_DEBUG_MODE
uint8_t m_ucDebugBuffer[100];
uint8_t m_ucDebugTemp;
#endif
#define IRTSOP_SKIP_REAL_TIME		2000
uint16_t m_uiIrTsopSkipRealTime;	// used in the main loop to skip real time
//----------------------------------------------------------------------------------
// RX Section
IRRXPulseData 		IRRxPulse 		= {0};		// keeps track of the RX pulses
IRRXMessageData   	IRRxMessage		= {0};		// collects IR pulses during IR message reception
// TX Section
IRTXMessageData		IRTXMessage		= {0};		// structure to handle the transmission
//----------------------------------------------------------------------------------


// prototypes
// IR RX section
static void IrRxInit(void);						// called @ initialization
//----------------------------------------------------------------------------------
static void IrRxClearReceivedMessage(void);		// used to clear the content of the last received and decoded message from the IRRxMessage structure
//----------------------------------------------------------------------------------
static void IrRxResetRxStructure(void);			// used to reset the IRRxMessage structure
//----------------------------------------------------------------------------------
static void IrRxCollectPulses(void);			// used to collect IR pulses into the IRRxMessage structure, called from the TSOP pin change ISR
//----------------------------------------------------------------------------------
static void IrRxMainLoop(void);					// called from the main loop to handle task related activities
//----------------------------------------------------------------------------------
static void IrRxDecodeReceivedMessage(void);	// called after an IR message was received
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------

// IR TX section
//----------------------------------------------------------------------------------
static void IrLEDOff(void);		// used to turn the LED OFF
//----------------------------------------------------------------------------------
static void IrLEDOn(void);		// used to emit the 56KHZ signal on the LED
//----------------------------------------------------------------------------------
static void IRTxResetTransmission(void);	// used to clear the transmission related structure (IRTXMessage)
//----------------------------------------------------------------------------------
static void IrTxGetNextSymbor(void);		// used during transmission and called from the 10KHZ ISR to prepare the ucIRON10KHZCounter and ucIROFF10KHZCounter counters for the next IR symbo to be transmitted
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== STATIC_END


// API
//==================================================================================================================== API_START
__attribute__ ((weak)) void IrTsopMessageReceivedCallback(void)			// callback type function to be implemented in the Application section
{	// called if a new message was received over the IT TSOP channel
	// in order to auto-trigger the next reception message the current message's ucNewDecodedMessageAvailable properties must be cleared before returning for this function
	// must include "IrTsop.h" header
	//-----------------------------------




	//// ------------------ implementation example --------------------------
	//RXDecodedMessageData *MyIRMessage = IrTsopGetLastReceivedMessage();	// define the pointer to hold the decoded message
	//
	//sprintf((char *)m_ucDebugBuffer,"CLBK:Broadcast=%d, Length=%d, >%s<\r\n", 	MyIRMessage->ucDecodedMessageBroadcast, 		/* if the message is broadcast type this value is set to 1, if the message is only for this robot this value is 0 */
	//																			MyIRMessage->ucDecodedMessageLength, 			/* the number of characters in the NULL terminated ucDecodedMessage[] buffer from below */
	//																			(char*)MyIRMessage->ucDecodedMessage);			/* the content of the received message with NULL termination (string format) */
	//DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
	//
	// if this flag is cleared 		=> the next received IR message will trigger another callback
	// if this flag is not cleared 	=> the next received IR message will not trigger callback but it still can be red-out using the "IrTsopGetLastReceivedMessage()" function
	//MyIRMessage->ucNewDecodedMessageAvailable = 0;		// clear the new message flag. if the new message flag is not cleared the next message will still arrive into the message buffer but no callback triggering will happen
	// --------------------------------------------------------------------




	return;
}
//----------------------------------------------------------------------------------
RXDecodedMessageData* IrTsopGetLastReceivedMessage(void)		// can be called from Application section to get the last received IR message
{	// ---
	return &IRRxMessage.DecodedMessage;
}
//----------------------------------------------------------------------------------
IrTxErrorStatus IrTsopSendASCIIMessage2PC(char *chMessage)		// function to send the IR message to PC
{
	if( (chMessage[0] == 0) )		return IrTxMessageTooShortError;	// at lest 1 data-byte must be present
	if( (strlen(chMessage) > 32) )	return IrTxMessageTooLongError;		// max 32bytes / message
	//--------------------------------



	uint8_t i,j,ucCRC;
	ucCRC = 0;
	j = (uint8_t)strlen(chMessage);
	// prepare the address field
	IRTXMessage.ucTXBuffer[0] = m_ucRobotAddress;
	ucCRC += IRTXMessage.ucTXBuffer[0];

	// prepare CNT field
	IRTXMessage.ucTXBuffer[1] = (j+1);
	ucCRC += IRTXMessage.ucTXBuffer[1];

	// prepare data to be sent and add CRC
	for(i=0;i<j;i++)
	{
		IRTXMessage.ucTXBuffer[i+2] = (uint8_t)chMessage[i];
		ucCRC += IRTXMessage.ucTXBuffer[i+2];
	}
	IRTXMessage.ucTXBuffer[i+2] = ucCRC;

	// prepare indexes
	IRTXMessage.ucByteIndex = 0;
	IRTXMessage.ucBitIndex  = 0;
	IRTXMessage.uiSymbolsLetfToSend = (uint16_t)((j+3)*8);

	#ifdef IRTSOP_DEBUG_MODE
		sprintf((char *)m_ucDebugBuffer,"IRTsopTX: >%s< by=%d, bi=%d, ind=%d\r\n", chMessage, IRTXMessage.ucByteIndex, IRTXMessage.ucBitIndex, IRTXMessage.uiSymbolsLetfToSend);
		DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
	#endif

	// start the IR transmission
	IRTXMessage.ucIRON10KHZCounter  = 70;		// 7000us
	IRTXMessage.ucIROFF10KHZCounter = 5;		// 500us
	IRTXMessage.ucTransmissionInProgress = 1;	// start TX sending @ 10KHZ ISR

	return IrTxNoError;
}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== API_END


// SYSTEM
//==================================================================================================================== SYSTEM_START
void IrTsopInit(void)	// used to initialize the IrTsop Module. Called from main.c file @ init section
{
	// turn off the IR LED
	IrLEDOff();	// disable the IR LED

	// start the 56KHZ timer
	// HAL_StatusTypeDef HAL_TIM_OC_Start(TIM_HandleTypeDef *htim, uint32_t Channel)
	if(HAL_TIM_OC_Start(&htim8,TIM_CHANNEL_3) != HAL_OK)	// start the 56KHZ signal on PC8 (TIM8_CH3)
	{
		Error_Handler2(ERROR_CODE_IRTSOP_INIT_001);
		return;
	}
	// start the general purpose timer = timer5.  prescaler=(89+1), 32 bit timer. Timer5 is on APB1 => Input Clock = 45X2 = 90MHZ, => increment speed = 1MHZ = 1uS/step
	if(HAL_TIM_Base_Start(&htim5) != HAL_OK)
	{
		Error_Handler2(ERROR_CODE_IRTSOP_INIT_002);
		return;
	}


	// ----------- IR RX section -------------
	IrRxInit();
	// ---------------------------------------

	// ----------- IR TX section -------------
	IRTxResetTransmission();	// dummy, just kept for the logical perspective
	// ---------------------------------------



	return;
}
//----------------------------------------------------------------------------------
void IrTsopMainLoop(void)	// loop function of the IrTsop Module. Called from the main.c file @ main loop section
{

	// skip real time
	if(m_uiIrTsopSkipRealTime)
	{
		m_uiIrTsopSkipRealTime --;
		return;
	}
	m_uiIrTsopSkipRealTime = IRTSOP_SKIP_REAL_TIME;
	//--------------------------------------------------------



	// ----------- IR RX section -------------
	IrRxMainLoop();
	// ---------------------------------------

	// ----------- IR TX section -------------
	// nothing here
	#ifdef IRTSOP_DEBUG_MODE
		if(m_ucDebugTemp)
		{
			if( (m_ucDebugTemp == 'i') )
			{
				#ifdef IRTSOP_DEBUG_MODE
					sprintf((char *)m_ucDebugBuffer,"IRTsopTX: by=%d, bi=%d, ind=%d\r\n", IRTXMessage.ucByteIndex, IRTXMessage.ucBitIndex, IRTXMessage.uiSymbolsLetfToSend);
					DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
				#endif
			}
			if( (m_ucDebugTemp == 'q') )	IrTsopSendASCIIMessage2PC("My First Message");
			if( (m_ucDebugTemp == 'a') )	IrTsopSendASCIIMessage2PC("Something New!");
			if( (m_ucDebugTemp == 'z') )	IrTsopSendASCIIMessage2PC("Short");
			m_ucDebugTemp = 0;
		}
	#endif
	// ---------------------------------------



	/*
	#ifdef IRTSOP_DEBUG_MODE
		if(IRRxPulse.RXNewPulseAvailable)
		{
			IRRxPulse.RXNewPulseAvailable = 0;
			sprintf((char *)m_ucDebugBuffer,"IR=%lu, Type=%d\r\n", IRRxPulse.ui32RXPulseDuration,IRRxPulse.RXPulseType);
			DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
		}
	#endif
	 */

	/*
	#ifdef IRTSOP_DEBUG_IRRX
		if(IRRxMessage.DecodedMessage.ucNewDecodedMessageAvailable)
		{
			IRRxMessage.DecodedMessage.ucNewDecodedMessageAvailable = 0;
			sprintf((char *)m_ucDebugBuffer,"Broadcast=%d, Length=%d, >%s<\r\n", IRRxMessage.DecodedMessage.ucDecodedMessageBroadcast,
					IRRxMessage.DecodedMessage.ucDecodedMessageLength,
					(char*)IRRxMessage.DecodedMessage.ucDecodedMessage);
			DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
		}
	#endif
	*/





	return;
}
//----------------------------------------------------------------------------------
void IrTsopTimer1KHZISR(void)	// 1KHZ (1mS) timer function called from System ISR from stm32f4xx_it.c file @ SysTick Function (!!! keep as short as possible !!!)
{


	return;
}
//----------------------------------------------------------------------------------
void IrTsopTimer10KHZISR(void)	// 10KHZ (100uS) timer function called from Tim2 ISR (!!! keep as short as possible !!!)
{
	// check if an IR transmission is in progress or not
	if( (!IRTXMessage.ucTransmissionInProgress) )  return;		// no transmission is in progress

	if( (IRTXMessage.ucIRON10KHZCounter) )
	{	// 56KHZ IR is ON
		IrLEDOn();
		IRTXMessage.ucIRON10KHZCounter --;
		return;
	}

	if( (IRTXMessage.ucIROFF10KHZCounter) )
	{	// IR is OFF
		IrLEDOff();
		IRTXMessage.ucIROFF10KHZCounter --;
		return;
	}

	// last symbol was transmitted => get the next symbol
	IrTxGetNextSymbor();

	return;
}
//----------------------------------------------------------------------------------
void IrTsopCallback()	// used to be called from the stm32f4xx_it.c @ every rising or falling edge of the TSOP4856
{	// __HAL_TIM_GET_COUNTER(htim5);		// general purpose timer without interrupt and with 1uS/Increment


	// -------------- Measure the length of the IR pulse  ---------------
	if(HAL_GPIO_ReadPin(TSOP_GPIO_Port, TSOP_Pin) == GPIO_PIN_RESET)
	{	// TSOP4856 falling edge was detected => is the START of the IR Pulse
		IRRxPulse.ui32RXPulseStart = __HAL_TIM_GET_COUNTER(&htim5);	// content of the Timer5
		IRRxPulse.ucRXPulseStatus = 1;		// pulse start detected
		return;
	}
	else
	{	// TSOP4856 rising edge was detected => is the END of the IR Pulse
		if(IRRxPulse.ucRXPulseStatus != 1)	return;		// no valid pulse measurement was in progress

		// pulse measurement was in progress
		IRRxPulse.ucRXPulseStatus = 0;	// no pulse measurement is in progress from now on. Pulse start must be detected
		IRRxPulse.ui32RXPulseEnd = __HAL_TIM_GET_COUNTER(&htim5);	// content of the Timer5
		if(IRRxPulse.ui32RXPulseEnd > IRRxPulse.ui32RXPulseStart)	IRRxPulse.ui32RXPulseDuration = (IRRxPulse.ui32RXPulseEnd - IRRxPulse.ui32RXPulseStart);					// no overflow
		else														IRRxPulse.ui32RXPulseDuration = (IRRxPulse.ui32RXPulseEnd + (0xFFFFFFFF - IRRxPulse.ui32RXPulseStart));		// with overflow
		IRRxPulse.RXNewPulseAvailable = 1;		// indicate the new pulse was detected
	}
	// ------------------------------------------------------------------


	// ------------------- Check the pulse type  ------------------------
	IRRxPulse.RXPulseType = RXPulseError;	// by default
	// detect the 300uS pulse => [150uS .... 500uS]
	if( (IRRxPulse.ui32RXPulseDuration > 150) && (IRRxPulse.ui32RXPulseDuration < 500) )					IRRxPulse.RXPulseType = RXPulse300uS;		// valid logical 0 was detected
	else
	{	// try for 700uS pulse => [550uS ... 900uS]
		if( (IRRxPulse.ui32RXPulseDuration > 550) && (IRRxPulse.ui32RXPulseDuration < 900) )				IRRxPulse.RXPulseType = RXPulse700uS;		// valid logical 1 was detected
		else
		{	// try for 3mS pulse => [2800uS .... 3500uS]
			if( (IRRxPulse.ui32RXPulseDuration > 2800) && (IRRxPulse.ui32RXPulseDuration < 3500) )			IRRxPulse.RXPulseType = RXPulse3mS;			// valid PC->Robot IRstart frame was detected
			else
			{	// try for 7ms pulse => [6500uS ... 7500uS]
				if( (IRRxPulse.ui32RXPulseDuration > 6500) && (IRRxPulse.ui32RXPulseDuration < 7500) )		IRRxPulse.RXPulseType = RXPulse7mS;			// valid Robot->PC IRstart frame was detected
				else
				{	// not a valid pulse
					IRRxPulse.RXNewPulseAvailable = 0;		// indicate that NO valid new pulse was detected
					if( (IRRxMessage.uiRXStatus != 0) )
					{	// call the reset of the pulse collector structure
						IrRxResetRxStructure();
						return;
					}
				}
			}
		}
	}
	// ------------------------------------------------------------------


	// ---------- call the pulse concatenating function -----------------
	IrRxCollectPulses();
	// ------------------------------------------------------------------


	// ------------------------------------------------------------------
	// ------------------------------------------------------------------

	return;
}
//----------------------------------------------------------------------------------
#ifdef IRTSOP_DEBUG_MODE
void IrTsopDebugRX(uint8_t *ucRXBuffer, uint8_t ucRXBufferLength)	// used to receive RX data from the debug interface
{
	return;
}
void IrTsopDebugRXChar(uint8_t ucRXChar)	// used to receive RX single char from the debug interface
{
	m_ucDebugTemp = ucRXChar;

	if( ucRXChar == 'q' )
	{
		// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);			//  debug on PC13 HIGH
		// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);  		//	debug on PC13 LOW
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
// IR RX section
static void IrRxInit(void)						// called @ initialization
{	// uses:
	// -- IRRxPulse 	// keeps track of the RX pulses
	// --IRRxMessage	// collects IR pulses during IR message reception
	// --------------------------

	IrRxResetRxStructure();
	IrRxClearReceivedMessage();

	return;
}
//----------------------------------------------------------------------------------
static void IrRxClearReceivedMessage(void)		// used to clear the content of the last received and decoded message from the IRRxMessage structure
{	// uses:
	// -- IRRxPulse 	// keeps track of the RX pulses
	// --IRRxMessage	// collects IR pulses during IR message reception
	// --------------------------
	uint8_t i;

	for(i=0;i<40;i++) 	IRRxMessage.DecodedMessage.ucDecodedMessage[i] = 0;
	IRRxMessage.DecodedMessage.ucMessageSourceType				= MessageTypeUnknown;
	IRRxMessage.DecodedMessage.ucDecodedMessageBroadcast 		= 0;
	IRRxMessage.DecodedMessage.ucNewDecodedMessageAvailable 	= 0;
	IRRxMessage.DecodedMessage.ucDecodedMessageLength			= 0;

	return;
}
//----------------------------------------------------------------------------------
static void IrRxResetRxStructure(void)			// used to reset the IRRxMessage structure
{	// uses:
	// -- IRRxPulse 	// keeps track of the RX pulses
	// --IRRxMessage	// collects IR pulses during IR message reception
	// --------------------------
	uint8_t i;

	#ifdef IRTSOP_DEBUG_IRRX
	if( (IRRxMessage.uiRXStatus != 500) )
	{
		sprintf((char *)m_ucDebugBuffer,"IrRxDecode: uiRXStatus Error: Val.=%d\r\n", IRRxMessage.uiRXStatus);
		DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
	}
	#endif

	for(i=0;i<45;i++) IRRxMessage.ucRXBuffer[i] 	= 0;

	IRRxMessage.ucRXByteIndex 	= 0;
	IRRxMessage.ucRXBitIndex	= 0;
	IRRxMessage.uiRXStatus 		= 0;


	return;
}
//----------------------------------------------------------------------------------
static void IrRxCollectPulses(void)			// used to collect IR pulses into the IRRxMessage structure, called from the TSOP pin change ISR
{	// uses:
	// -- IRRxPulse 	// keeps track of the RX pulses
	// --IRRxMessage	// collects IR pulses during IR message reception
	// --------------------------
	// called each time a new IR pulse was received
	// called from the TSOP ISR => KEEP AS SHORT AS POSSIBLE
	//---------------------------

	// ---- check if there is no message already in the buffer ------
	if( (IRRxMessage.uiRXStatus == 500) )
	{	// is a message already received and must be decoded
		IRRxPulse.RXNewPulseAvailable = 0;	// dummy
		return;
	}
	// --------------------------------------------------------------

	// ---------- Waiting for the start of the reception ------------
	if( (IRRxMessage.uiRXStatus == 0) )
	{	// no Reception is in progress => waiting for the start field PC->ROBOT
		if( (IRRxPulse.RXPulseType == RXPulse3mS) )
		{	// a PC->ROBOT type of message was received => start of the reception
			IRRxMessage.uiRXStatus = 1;
			// MessagePC2ROBOT  - type of message
			IRRxMessage.ui32LastIRPulseTime = __HAL_TIM_GET_COUNTER(&htim5);
		}

		//// not used in the current version
		//if( (IRRxPulse.RXPulseType == RXPulse7mS) )
		//{	// a ROBOT->PC type of message was received => start of the reception
		//	IRRxMessage.uiRXStatus = 1;
		//	IRRxMessage.ui32LastIRPulseTime = __HAL_TIM_GET_COUNTER(&htim5);
		//}

		IRRxPulse.RXNewPulseAvailable = 0;	// dummy
		return;
	}
	// --------------------------------------------------------------

	IRRxPulse.RXNewPulseAvailable = 0;	// dummy

	// ------ Collecting the IR pulses as data bits of info. --------
	// check for the expected type of IR pulses
	if( (IRRxPulse.RXPulseType != RXPulse300uS) && (IRRxPulse.RXPulseType != RXPulse700uS) )
	{	// some wrong type of pulse
		IrRxResetRxStructure();
		return;
	}

	// get the time of the current pulse
	IRRxMessage.ui32LastIRPulseTime = __HAL_TIM_GET_COUNTER(&htim5);

	// populate the buffer if the received data is logical HIGH
	if( (IRRxPulse.RXPulseType == RXPulse700uS) )	IRRxMessage.ucRXBuffer[IRRxMessage.ucRXByteIndex] |= (1<<(IRRxMessage.ucRXBitIndex));

	// calculate the next indexes for the reception buffer
	IRRxMessage.uiRXStatus ++;
	IRRxMessage.ucRXBitIndex ++;
	if( (IRRxMessage.ucRXBitIndex > 7) )
	{
		IRRxMessage.ucRXBitIndex = 0;
		IRRxMessage.ucRXByteIndex ++;
		if( (IRRxMessage.ucRXByteIndex > 34) )
		{	// too long message
			IrRxResetRxStructure();
			return;
		}
	}
	// --------------------------------------------------------------


	return;
}
//----------------------------------------------------------------------------------
static void IrRxMainLoop(void)				// called from the main loop to handle task related activities
{	// uses:
	// -- IRRxPulse 	// keeps track of the RX pulses
	// --IRRxMessage	// collects IR pulses during IR message reception
	// --------------------------
	uint32_t uiElapsedTime;

	// ----------- check for timeout -----------
	if( (IRRxMessage.uiRXStatus != 0) )
	{	// a reception is in progress
		IRRxMessage.ui32TimeTemp = __HAL_TIM_GET_COUNTER(&htim5);	// time in us

		if( (IRRxMessage.ui32TimeTemp > IRRxMessage.ui32LastIRPulseTime) )		uiElapsedTime = (IRRxMessage.ui32TimeTemp - IRRxMessage.ui32LastIRPulseTime);
		else 																	uiElapsedTime = ((0xFFFFFFFF - IRRxMessage.ui32LastIRPulseTime) + IRRxMessage.ui32TimeTemp);	// if timer overflow occurred

		if( (uiElapsedTime > 20000) )
		{	// more than 20ms between 2 consecutive IR pulses => timeout and time to decode the received buffer
			// set the Status to 500 indicating that no further data can be put inside the reception buffer
			IRRxMessage.uiRXStatus = 500;
			IrRxDecodeReceivedMessage();
		}
	}
	// -----------------------------------------

	return;
}
//----------------------------------------------------------------------------------
static void IrRxDecodeReceivedMessage(void)	// called after an IR message was received
{	// uses:
	// -- IRRxPulse 	// keeps track of the RX pulses
	// --IRRxMessage	// collects IR pulses during IR message reception
	// --------------------------
	uint8_t i;	// temp var for indexing

	// ---- check for the minimum valid length of the  received buffer ----
	// if 1 Data byte => CNT = 2
	// => ucRXByteIndex is minimum 33
	if( (IRRxMessage.uiRXStatus < 33) )
	{	// buffer is too short !
		#ifdef IRTSOP_DEBUG_IRRX
			sprintf((char *)m_ucDebugBuffer,"IrRxDecode: buffer is too short: Length=%d\r\n", IRRxMessage.uiRXStatus);
			DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
		#endif
		IrRxResetRxStructure();
		return;
	}
	// --------------------------------------------------------------------

	// --------- check for the expected value of the Bit Index ------------
	if( (IRRxMessage.ucRXBitIndex != 0) )
	{	// wrong bit index => the message dose not contain complete no. of bytes
		#ifdef IRTSOP_DEBUG_IRRX
			sprintf((char *)m_ucDebugBuffer,"IrRxDecode: Wrong value of the Bit Index: Value=%d\r\n", IRRxMessage.ucRXBitIndex);
			DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
		#endif
		IrRxResetRxStructure();
		return;
	}
	// --------------------------------------------------------------------

	// --------------- retrieve the Address field -------------------------
	// check for matching address
	if( (IRRxMessage.ucRXBuffer[0] != 255) && (IRRxMessage.ucRXBuffer[0] != m_ucRobotAddress) )
	{	// the message is not for this robot
		#ifdef IRTSOP_DEBUG_IRRX
			sprintf((char *)m_ucDebugBuffer,"IrRxDecode: Wrong address: Add1=%d, Add2=%d\r\n", m_ucRobotAddress,IRRxMessage.ucRXBuffer[0]);
			DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
		#endif
		IrRxResetRxStructure();
		return;
	}
	// --------------------------------------------------------------------

	// ------------ retrieve and check the CNT field ----------------------
	// for a message with 1 data field => CNT=2 and ByteIndex=4
	// ==> CNT==(ByteIndex-2)
	uint8_t ucCnt;
	ucCnt = IRRxMessage.ucRXBuffer[1];
	if( (ucCnt != (IRRxMessage.ucRXByteIndex - 2)) )
	{	// CNT error
		#ifdef IRTSOP_DEBUG_IRRX
			sprintf((char *)m_ucDebugBuffer,"IrRxDecode: CNT field Error: CNT1=%d, CNT2=%d\r\n", ucCnt, (IRRxMessage.ucRXByteIndex - 2));
			DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
		#endif
		IrRxResetRxStructure();
		return;
	}
	// --------------------------------------------------------------------

	// --------------- check the CRC of the message -----------------------
	uint8_t ucCRC = 0;
	for(i=0;i<(IRRxMessage.ucRXByteIndex - 1);i++)	ucCRC += IRRxMessage.ucRXBuffer[i];
	if( (ucCRC != IRRxMessage.ucRXBuffer[(IRRxMessage.ucRXByteIndex - 1)]) )
	{	// CRC dose not match
		#ifdef IRTSOP_DEBUG_IRRX
			sprintf((char *)m_ucDebugBuffer,"IrRxDecode: Wrong CRC value: CRC1=%d, CRC2=%d\r\n", IRRxMessage.ucRXBuffer[(IRRxMessage.ucRXByteIndex - 1)], ucCRC);
			DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
		#endif
		IrRxResetRxStructure();
		return;
	}
	// --------------------------------------------------------------------


	// --------- prepare the "RXDecodedMessageData" structure -------------
	// copy the received message
	for(i=0;i<(ucCnt-1);i++)	IRRxMessage.DecodedMessage.ucDecodedMessage[i] = IRRxMessage.ucRXBuffer[(i + 2)];
	IRRxMessage.DecodedMessage.ucDecodedMessage[i] = 0;	// string terminator mode
	// set the number of received bytes inside the buffer
	IRRxMessage.DecodedMessage.ucDecodedMessageLength = (ucCnt - 1);
	// type of the message
	IRRxMessage.DecodedMessage.ucMessageSourceType = MessagePC2ROBOT;	// the only type currently supported
	// check for broadcast
	if( (IRRxMessage.ucRXBuffer[0] == 255) )	IRRxMessage.DecodedMessage.ucDecodedMessageBroadcast = 1;	// was a broadcast message
	// new message is available
	if( (IRRxMessage.DecodedMessage.ucNewDecodedMessageAvailable == 0) )
	{	// must set the new message flag and also trigger the callback function
		IRRxMessage.DecodedMessage.ucNewDecodedMessageAvailable = 1;	// new message was received
		IrTsopMessageReceivedCallback();
	}
	// reset the IR structure to enable new incoming message
	IrRxResetRxStructure();
	// --------------------------------------------------------------------
	// --------------------------------------------------------------------


	return;
}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------


// IR TX Section
//----------------------------------------------------------------------------------
// #define IR_TSOP_Pin GPIO_PIN_8
// #define IR_TSOP_GPIO_Port GPIOC
static void IrLEDOff(void)	// used to turn the LED OFF
{	// LED is on PC8
	/*
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = IR_TSOP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
	HAL_GPIO_Init(IR_TSOP_GPIO_Port, &GPIO_InitStruct);
	*/

	// PC8 set to input
	IR_TSOP_GPIO_Port->MODER &= ~(0x03 << 16);	// MODER [17][16] = [0,0] -> cleared

	return;
}
//----------------------------------------------------------------------------------
static void IrLEDOn(void)	// used to emit the 56KHZ signal on the LED
{
	/*
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = IR_TSOP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
	HAL_GPIO_Init(IR_TSOP_GPIO_Port, &GPIO_InitStruct);
	*/

	// PC8 set to alternative function mode
	IR_TSOP_GPIO_Port->MODER |= (0x02 << 16);	// MODER [17][16] = [1,0]

	return;
}
//----------------------------------------------------------------------------------
static void IRTxResetTransmission(void)	// used to clear the transmission related structure (IRTXMessage)
{
	uint8_t i;

	IRTXMessage.ucTransmissionInProgress = 0;
	for(i=0; i<40; i++)	IRTXMessage.ucTXBuffer[i] = 0;
	IRTXMessage.ucBitIndex = 0;
	IRTXMessage.ucByteIndex = 0;
	IRTXMessage.uiSymbolsLetfToSend = 0;
	IRTXMessage.ucIROFF10KHZCounter = 0;
	IRTXMessage.ucIRON10KHZCounter  = 0;
	return;
}
//----------------------------------------------------------------------------------
static void IrTxGetNextSymbor(void)		// used during transmission and called from the 10KHZ ISR to prepare the ucIRON10KHZCounter and ucIROFF10KHZCounter counters for the next IR symbol to be transmitted
{	// called from the 10KHZ ISR => keep as short as possible
	//-------------------------------

	// check for the end of transmission
	if(IRTXMessage.uiSymbolsLetfToSend == 0)
	{
		IRTXMessage.ucTransmissionInProgress = 0;	// no transmission is in progress
		IrLEDOff();	// dummy, but just to be on the safe side ...
		return;
	}

	// transmission has not yet ended => prepare the next symbol
	//IrLEDOn();
	IRTXMessage.uiSymbolsLetfToSend --;

	// check for logical 1 (high)
	if( (IRTXMessage.ucTXBuffer[IRTXMessage.ucByteIndex] & (1<<IRTXMessage.ucBitIndex)) )
	{	// send HIGH
		IRTXMessage.ucIRON10KHZCounter 	= 7;
		IRTXMessage.ucIROFF10KHZCounter = 3;
	}
	else
	{	// send LOW
		IRTXMessage.ucIRON10KHZCounter  = 3;
		IRTXMessage.ucIROFF10KHZCounter = 7;
	}

	IRTXMessage.ucBitIndex ++;
	if( (IRTXMessage.ucBitIndex > 7) )
	{
		IRTXMessage.ucBitIndex = 0;
		IRTXMessage.ucByteIndex ++;		// must be careful not to exceed the value of 39 (ucTXBuffer[40]), normally must not get over 34
	}

	return;
}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== STATIC_END
