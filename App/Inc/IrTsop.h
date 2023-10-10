/*
 * IrTsop.h
 *
 *  Created on: Feb 19, 2022
 *      Author: Arthur
 */

#ifndef INC_IRTSOP_H_
#define INC_IRTSOP_H_

// Settings
/*
 *
 *
 *
 */


//==================================================================================================================== USECASE START
/*
 *-- RECEPTION
 * - for asynchronous mode use the: IrTsopMessageReceivedCallback()
 * in order to keep the interaction asynchronous during each use of the IrTsopMessageReceivedCallback() function the:
 * --- RXDecodedMessageData.ucNewDecodedMessageAvailable must be cleared to zero
 * - for synchronous just call the: IrTsopGetLastReceivedMessage()
 * in case of synchronous use newer received message may overwrite older message in case the IrTsopGetLastReceivedMessage() is not called between receptions
 *
 *
 *-- TRANSMISSION
 *-- IrTsopSendASCIIMessage2PC()
 * just call the API function: IrTsopSendASCIIMessage2PC() with the message you need to transmit to the PC
 * the message must be at least 1 byte long and not longer than 32 bytes. The message must be ZERO ended as a standard text type
 * The function returns the error code
 *
 */
//==================================================================================================================== USECASE END

/*
 * - the format of the message from the PC is:
 * --------------------
 * | PC->ROBOT | ADDRESS | CNT | DATA0 ... DATA31 | CRC |
 * --- PC->ROBOT			= the 3000uS IR pulse + 500uS OFF
 * --- ADDRESS				= one byte, is the address of the destination robot, 255 represents broadcast
 * --- CNT					= one byte, represents the number of bytes until the end of the message. For 1 data byte => CNT=2, for 32 data bytes => CNT=33
 * --- DATA0 ... DATA31		= one to 32 bytes, representing the content of the message
 * --- CRC					= one byte, representing the sum modulo 256 of all the bytes from before ==> (ADDRESS + CNT + DATA0 + ... + CRC)modulo256
 * --------------------
 *
 * the format of the message from the ROBOT is:
 * --------------------
 * | ROBOT->PC | ADDRESS | CNT | DATA0 ... DATA31 | CRC |
 * --- ROBOT->PC			= the 7000uS IR pulse + 500uS OFF
 * --- ADDRESS				= one byte, is the address of the address of the current robot (the one sending the message)
 * --- CNT					= one byte, represents the number of bytes until the end of the message. For 1 data byte => CNT=2, for 32 data bytes => CNT=33
 * --- DATA0 ... DATA31		= one to 32 bytes, representing the content of the message
 * --- CRC					= one byte, representing the sum modulo 256 of all the bytes from before ==> (ADDRESS + CNT + DATA0 + ... + CRC)modulo256
 * --------------------
 *
 * all bytes are sent and received LSB first => 0x53 => 11001010 => 1100=0x03 and 1010=0x50
 */

//#define IRTSOP_DEBUG_MODE		/* main debug flag */

//#define IRTSOP_DEBUG_IRRX		/* secondary debug flag, related only to the IR reception section */
// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);		//  debug on PC13 HIGH
// HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);  	//	debug on PC13 LOW
#ifndef IRTSOP_DEBUG_MODE
	#ifdef IRTSOP_DEBUG_IRRX
		#undef IRTSOP_DEBUG_IRRX
	#endif
#endif
//====================================================================================================================

// custom data structures
//==================================================================================================================== CUSTOM_DATA_STRUCTS_START
// RX Section:
typedef enum
{
	RXPulseError		=0U,		/* no valid RX pulse was detected */
	RXPulse300uS		=1U,		/* 300uS pulse was detected == logical 0 */
	RXPulse700uS		=2U,		/* 700uS pulse was detected == logical 1 */
	RXPulse3mS			=3U,		/* 3mS PC->Robot IR start frame was detected */
	RXPulse7mS			=4U			/* 7mS Robot->PC IR start frame was detected */

}RXPulseType;
//----------------------------------------------------------------------------------
typedef struct
{
	uint8_t			ucRXPulseStatus;			/* if == 0 => no pulse has started (waiting Start of the IR Pulse, if == 1 => IR pulse already started waiting for the End of the IR Pulse) */
	uint32_t		ui32RXPulseStart;			/* receives the timer5 content at falling edge of the TSOP4856 => Start of the IR Pulse, (Timer5 count represents 1uS) */
	uint32_t		ui32RXPulseEnd;				/* receives the timer5 content at rising  edge of the TSOP4856 => End   of the IR Pulse */
	uint32_t		ui32RXPulseDuration;		/* holds the IR Pulse duration */
	RXPulseType		RXPulseType;				/* keeps the pulse type */
	uint8_t			RXNewPulseAvailable;		/* always set to 1 if a new RX pulse was detected */
}IRRXPulseData;
//----------------------------------------------------------------------------------
typedef enum
{
	MessageTypeUnknown	=0U,					/* if it is not known the sender, only used for unformatted message */
	MessagePC2ROBOT		=1U,					/* the sender of the message was the PC 		=> PC->ROBOT header was received */
	MessageROBOT2PC		=2U 					/* the sender of the message was another ROBOT 	=> ROBOT->PC header was received */
}RXMessageType;
//----------------------------------------------------------------------------------
typedef struct
{
	uint8_t		ucNewDecodedMessageAvailable;	/* is set to 1 if a new valid message was decoded and stored in the ucDecodedMessage[] buffer */
	uint8_t		ucMessageSourceType;			/* is of type "RXMessageType", identifying the type of the sender */
	uint8_t		ucDecodedMessageBroadcast;		/* set to 1 in case the received message was a broadcast type of message	*/
	uint8_t		ucDecodedMessageLength;			/* it holds the last number data bytes in the ucDecodedMessage[] buffer		*/
	uint8_t 	ucDecodedMessage[40];			/* it holds the last valid received message (the data fields), the ucDecodedMessage[ucDecodedMessageLength] is set to 0 => can be used as TEXT	*/
}RXDecodedMessageData;
//----------------------------------------------------------------------------------
typedef struct
{
	// Reception realted
	uint8_t			ucRXBuffer[45];			// IR-RX buffer
	uint16_t		uiRXStatus;						/* it is used to keep track of the RX progress */
	// - uiRXStatus == 000 				=> no RX is in progress and waiting for the 3mS PC->Robot IR start frame
	// - uiRXStatus == 000  -> 001		=> the 3mS PC->Robot IR start frame  was detected and now waiting for the data
	// - uiRXStatus == 001  -> 002 		=> first    bit from the first    byte was detected => (byte=00,  bit=0, LSB)  (the ADDRESS   	Byte)
	// - uiRXStatus == 002  -> 003 		=> second   bit from the first    byte was detected => (byte=00,  bit=1,    )  (the ADDRESS   	Byte)
	// - uiRXStatus == 008  -> 009 		=> last     bit from the first    byte was detected => (byte=00,  bit=7, MSB)  (the ADDRESS   	Byte)
	// - uiRXStatus == 009  -> 010 		=> first    bit from the second   byte was detected => (byte=01,  bit=0, LSB)  (the CNT       	Byte)
	// - uiRXStatus == 016  -> 017 		=> last     bit from the second   byte was detected => (byte=01,  bit=7, MSB)  (the CNT       	Byte)
	// - uiRXStatus == 017  -> 018 		=> first    bit from the third    byte was detected => (byte=02,  bit=0, LSB)  (the DATA0     	Byte)
	// - uiRXStatus == 024  -> 025 		=> last     bit from the third    byte was detected => (byte=02,  bit=7, MSB)  (the DATA0     	Byte)
	// - uiRXStatus == 025  -> 026 		=> first    bit from the third    byte was detected => (byte=03,  bit=0, LSB)  (the DATA1/CRC   Byte)
	// - uiRXStatus == 032  -> 033 		=> last     bit from the third    byte was detected => (byte=03,  bit=7, MSB)  (the DATA1/CRC   Byte)

	uint8_t		ucRXByteIndex;				// together with ucRXBitIndex  represents the byte and bit currently being used to store the RXPulseType value
	uint8_t		ucRXBitIndex;				// together with ucRXByteIndex represents the byte and bit currently being used to store the RXPulseType value
	// ucRXByteIndex == 00 => ucRXBitIndex = [0 ... 7],  uiRXStatus = [002 ... 009],  ADDRESS
	// ucRXByteIndex == 01 => ucRXBitIndex = [0 ... 7],  uiRXStatus = [010 ... 017],  CNT
	// ucRXByteIndex == 02 => ucRXBitIndex = [0 ... 7],  uiRXStatus = [018 ... 025],  DATA00
	// ucRXByteIndex == 03 => ucRXBitIndex = [0 ... 7],  uiRXStatus = [026 ... 033],  DATA01/CRC
	// ucRXByteIndex == 04 => ucRXBitIndex = [0 ... 7],  uiRXStatus = [034 ... 041],  DATA02/CRC
	// ...
	// ucRXByteIndex == 33 => ucRXBitIndex = [0 ... 7],  uiRXStatus = [266 ... 273],  DATA31/CRC
	// ucRXByteIndex == 34 => ucRXBitIndex = [0 ... 7],  uiRXStatus = [274 ... 281],  CRC
	// ucRXByteIndex == n  => uiRXStatus = [((n*8)+2) ... (((n*8)+2)+7)]

	// Reception Timeout related
	uint32_t	ui32LastIRPulseTime;		// represents the moment in ms when the last IR pulse was received. it is based on "__HAL_TIM_GET_COUNTER(&htim5);	// content of the Timer5"
	uint32_t	ui32TimeTemp;				// used to calculate the time since the last IR pulse
	// -- ui32TimeTemp = __HAL_TIM_GET_COUNTER(&htim5);  -> if ui32TimeTemp < ui32LastIRPulseTime => time difference is: ((0xFFFFFFFF - ui32LastIRPulseTime) + ui32TimeTemp)

	// Decoded Message
	RXDecodedMessageData	DecodedMessage;		// holds the decoded message data


}IRRXMessageData;
//----------------------------------------------------------------------------------
// TX section
typedef enum
{
	IrTxNoError						= 0U,		/* All was ok and the IR message was sent */
	IrTxMessageTooShortError		= 1U,		/* The message to be sent was bye long => nothing to be sent */
	IrTxMessageTooLongError			= 2U		/* The message to be sent is longer than 32 bytes */

}IrTxErrorStatus;
//----------------------------------------------------------------------------------
typedef struct
{
	// each counter is decremented @ 10KHZ => 100uS:
	// for ROBOT->PC:
	//	--- IR_ON		=> 7000us time 	=> 	70 counts of ON		(ucIRON10KHZCounter)
	//  --- IR_OFF		=>  500us time	=> 	5  counts of OFF	(ucIROFF10KHZCounter)
	// for LOGIC_HIGH:
	//  --- IR_ON		=>  700us time	=>  7  counts of ON
	//  --- IR_OFF		=>  300us time	=>  3  counts of OFF
	// for LOGIC_LOW:
	//  --- IR_ON		=>  300us time	=>  3  counts of ON
	//  --- IR_OFF		=>  700us time	=>  7  counts of OFF
	uint8_t		ucIRON10KHZCounter;				/* is decremented from the 10KHZ timer ISR, as long as this value is not ZERO => the 56KHZ IR signal is emitted */
	uint8_t		ucIROFF10KHZCounter;			/* is decremented from the 10KHZ timer ISR, as long as this value is not ZERO => the 56KHZ IR is NOT emitted and the next IR symbol is not loaded */
	uint16_t	uiSymbolsLetfToSend;			/* the number of symbols left to send a symbol is a pair of IR_ON+IR_OFF values => the number of bits */
	uint8_t		ucByteIndex;					/* the index of the current byte being transmitted */
	uint8_t		ucBitIndex;						/* the index of the current bit being transmitted */
	uint8_t		ucTXBuffer[40];					/* holds the data being transmitted */
	uint8_t		ucTransmissionInProgress;		/* set to 1 if an IR transmission is in progress */
	// -- ucTXBuffer[00]	=> ADDRESS		=> the address of the ROBOT -> to be transmitted to the PC
	// -- ucTXBuffer[01]	=> CNT			=> the number of bytes to be transmitted including also the CRC
	// -- ucTXBuffer[02]	=> DATA00		=> the first data byte
	// -- ucTXBuffer[03]	=> DATA01/CRC	=> the second byte or the CRC
	// ....
	// -- ucTXBuffer[33]	=> DATA31/CRC	=> the last data or the CRC
	// -- ucTXBuffer[34]	=> CRC			=> the CRC in case of a 32 bytes data package
}IRTXMessageData;
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== CUSTOM_DATA_STRUCTS_END




// API
//==================================================================================================================== API_START
// RX
void IrTsopMessageReceivedCallback(void);			// callback type function to be implemented in the Application section
//----------------------------------------------------------------------------------
//void IrTsopGetLastReceivedMessage(RXDecodedMessageData *pNewIRMessage);		// can be called from Application section to get the last received IR message
RXDecodedMessageData* IrTsopGetLastReceivedMessage(void);
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
// TX
//----------------------------------------------------------------------------------
IrTxErrorStatus IrTsopSendASCIIMessage2PC(char *chMessage);		// function to send the IR message to PC
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== API_END



// SYSTEM
//==================================================================================================================== SYSTEM_START
void IrTsopInit(void);				// used to initialize the IrTsop Module. Called from main.c file @ init section
//----------------------------------------------------------------------------------
void IrTsopMainLoop(void);			// loop function of the IrTsop Module. Called from the main.c file @ main loop section
//----------------------------------------------------------------------------------
void IrTsopTimer1KHZISR(void);		// 1KHZ timer function called from System ISR from stm32f4xx_it.c file @ SysTick Function (!!! keep as short as possible !!!)
//----------------------------------------------------------------------------------
void IrTsopTimer10KHZISR(void);		// 10KHZ timer function called from Tim2 ISR (!!! keep as short as possible !!!)
//----------------------------------------------------------------------------------
void IrTsopCallback();				// used to be called from the stm32f4xx_it.c @ every rising or falling edge of the TSOP4856
//----------------------------------------------------------------------------------
#ifdef IRTSOP_DEBUG_MODE
void IrTsopDebugRX(uint8_t *ucRXBuffer, uint8_t ucRXBufferLength);		// used to receive RX data from the debug interface
void IrTsopDebugRXChar(uint8_t ucRXChar);		// used to receive RX single char from the debug interface
#endif
//----------------------------------------------------------------------------------
//==================================================================================================================== SYSTEM_END




// STATIC (INTERN)
//==================================================================================================================== STATIC_START
// defined in .c file
//==================================================================================================================== STATIC_END




// Resources
/*
#define TSOP_Pin GPIO_PIN_3
#define TSOP_GPIO_Port GPIOE
#define TSOP_EXTI_IRQn EXTI3_IRQn

#define IR_TSOP_Pin GPIO_PIN_8
#define IR_TSOP_GPIO_Port GPIOC

it uses TIMER8 => // used in IrTsop to generate the 56KHZ IR modulation (TIM8_CH3 -> PC8)
it also uses the free running TIM5 timer to get a real time measurement
it also uses the 10KHZ Timer2 ISR for the IR Sending
 *
 *
 *
 *
 */

#endif /* INC_IRTSOP_H_ */
