#include "main.h"

#include "stdint.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"

#include "WS2812.h"
#include "Wheel.h"
#include "Interaction.h"

#include "Application.h"
#include "I2CDevices.h"
#include "VL53l0x.h"

#include <math.h>

// my variables
uint8_t m_ucDebugBuffer_app[100];

uint16_t run_timer;
uint8_t run_or_tumble;

#define RUN_TIMER_DEFAULT 10

#define WHEEL_RUN_SPEED 50 // [50..500] !!!!!

#define RUN 1
#define TUMBLE 0

#define BLACK_WHITE_THRESHOLD 100 // [0..65535]

uint8_t uiDisplayChannel;
uint8_t ucWS2812Intensity = 255;

InteractionResultDataStructure m_NewInteractionResult;
IDPatternResultDataStructure m_NewIDResult;
uint8_t m_ucInteractionOtherRobotsAddressOld[6];		// it is used to detect change in the neighborhood of the robot and reset the auto power off counter if activity is detected around this robot

COMMAND_CALLBACK_REASON m_NewCallbackReason;

double new_x;
double new_y;

int16_t calculated_angle;

#define IR_THRESHOLD 20

uint8_t IR0, IR1, IR2, IR3, IR4, IR5;

uint8_t myR, myG, myB;

uint8_t recR[50], recG[50], recB[50];

// STATIC (INTERN) prototypes
//==================================================================================================================== STATIC_START
// variables - external
//----------------------------------------------------------------------------------


// variables - internal
#ifdef APPLICATION_DEBUG_MODE
//uint8_t m_ucDebugBuffer[100];
#endif
#define APPLICATION_SKIP_REAL_TIME		200
uint16_t m_uiApplicationSkipRealTime;	// used in the main loop to skip real time
//----------------------------------------------------------------------------------


// prototypes

//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== STATIC_END


// API
//==================================================================================================================== API_START

//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== API_END


void InteractionNewResultAvailableCallback(InteractionResultDataStructure NewInteractionResult, IDPatternResultDataStructure NewIDResult) {
	m_NewInteractionResult = NewInteractionResult;
	m_NewIDResult = NewIDResult;

	for(uint8_t i=0;i<6;i++)
	{
		if( (m_ucInteractionOtherRobotsAddressOld[i] != m_NewIDResult.ucDetectedRobotID[i] ) )
		{
			m_ucInteractionOtherRobotsAddressOld[i] = m_NewIDResult.ucDetectedRobotID[i];
		}
	}
}

void WheelRobotCommandEndCallback(COMMAND_CALLBACK_REASON CallbackReason) {
	m_NewCallbackReason = CallbackReason;
}

// SYSTEM
//==================================================================================================================== SYSTEM_START
void ApplicationInit(void)	// used to initialize the Application Module. Called from main.c file @ init section
{
	srand(HAL_GetTick());

	m_NewCallbackReason = 1;

	run_timer = RUN_TIMER_DEFAULT;
	run_or_tumble = RUN;

	myR = 255;
	myG = 0;
	myB = 255;

	WS2812SetDisplay(DISPLY_SYSTEM_1, 1, 50, myR, myG, myB);
	WS2812SetDisplay(DISPLY_SYSTEM_1, 2, 50, myR, myG, myB);
	WS2812SetDisplay(DISPLY_SYSTEM_1, 7, 50, myR, myG, myB);
	WS2812SetDisplay(DISPLY_SYSTEM_1, 8, 50, myR, myG, myB);
	WS2812ShowDisplay(DISPLY_SYSTEM_1, 100);

	return;
}
//----------------------------------------------------------------------------------
void ApplicationMainLoop(void)	// loop function of the Application Module. Called from the main.c file @ main loop section
{
	// skip real time
	if(m_uiApplicationSkipRealTime)
	{
		m_uiApplicationSkipRealTime --;
		return;
	}
	m_uiApplicationSkipRealTime = APPLICATION_SKIP_REAL_TIME;
	//--------------------------------------------------------

	if (run_timer == 0) {
		run_timer = RUN_TIMER_DEFAULT;

		if (m_NewCallbackReason != 0) {
			m_NewCallbackReason = 0;
			if (run_or_tumble == RUN) {
				run_or_tumble = TUMBLE;

				WheelRobotGoSpeedTmo(GOING_FORWARD, WHEEL_RUN_SPEED, 7);
			}
			else {
				// both on white, so, inside the play area

				WS2812EmptyDisplay(DISPLY_SYSTEM_1);

				WS2812SetDisplay(DISPLY_SYSTEM_1, 1, 10, 255, 255, 255);
				WS2812SetDisplay(DISPLY_SYSTEM_1, 2, 10, 255, 255, 255);
				WS2812SetDisplay(DISPLY_SYSTEM_1, 7, 10, 255, 255, 255);
				WS2812SetDisplay(DISPLY_SYSTEM_1, 8, 10, 255, 255, 255);

				IR0 = (m_NewInteractionResult.ucFinalDataFromOtherRobot[0] < IR_THRESHOLD) ? 0 : m_NewInteractionResult.ucFinalDataFromOtherRobot[0];
				IR1 = (m_NewInteractionResult.ucFinalDataFromOtherRobot[1] < IR_THRESHOLD) ? 0 : m_NewInteractionResult.ucFinalDataFromOtherRobot[1];
				IR2 = (m_NewInteractionResult.ucFinalDataFromOtherRobot[2] < IR_THRESHOLD) ? 0 : m_NewInteractionResult.ucFinalDataFromOtherRobot[2];
				IR3 = (m_NewInteractionResult.ucFinalDataFromOtherRobot[3] < IR_THRESHOLD) ? 0 : m_NewInteractionResult.ucFinalDataFromOtherRobot[3];
				IR4 = (m_NewInteractionResult.ucFinalDataFromOtherRobot[4] < IR_THRESHOLD) ? 0 : m_NewInteractionResult.ucFinalDataFromOtherRobot[4];
				IR5 = (m_NewInteractionResult.ucFinalDataFromOtherRobot[5] < IR_THRESHOLD) ? 0 : m_NewInteractionResult.ucFinalDataFromOtherRobot[5];

				new_x = - IR0
					- (0.5*IR1)
					+ (0.5*IR2)
					+ IR3
					+ (0.5*IR4)
					- (0.5*IR5);

				new_y = - (0.866*IR1)
					- (0.866*IR2)
					+ (0.866*IR4)
					+ (0.866*IR5);

				calculated_angle = atan2(new_y, new_x) * 180 / 3.14159;

				uiDisplayChannel = 0;
				if( (m_NewInteractionResult.ucFinalDataFromOtherRobot[uiDisplayChannel] != 0) ) { // there is signal detected from another robot => RED=0 and display GREEN / BLUE
					if( (m_NewIDResult.ucDetectedRobotID[uiDisplayChannel] == 0) ) { // robot is too far away => GREEN
						WS2812SetDisplay(DISPLY_SYSTEM_1, 1, ucWS2812Intensity, 0, m_NewInteractionResult.ucFinalDataFromOtherRobot[uiDisplayChannel], 0);
						WS2812SetDisplay(DISPLY_SYSTEM_1, 2, ucWS2812Intensity, 0, m_NewInteractionResult.ucFinalDataFromOtherRobot[uiDisplayChannel], 0);
					}
					else { // robot is close => can detect the ID also => BLUE
						WS2812SetDisplay(DISPLY_SYSTEM_1, 1, ucWS2812Intensity, 0, 0, m_NewInteractionResult.ucFinalDataFromOtherRobot[uiDisplayChannel]);
						WS2812SetDisplay(DISPLY_SYSTEM_1, 2, ucWS2812Intensity, 0, 0, m_NewInteractionResult.ucFinalDataFromOtherRobot[uiDisplayChannel]);
					}
				}

				uiDisplayChannel = 1;
				if( (m_NewInteractionResult.ucFinalDataFromOtherRobot[uiDisplayChannel] != 0) ) { // there is signal detected from another robot => RED=0 and display GREEN / BLUE
					if( (m_NewIDResult.ucDetectedRobotID[uiDisplayChannel] == 0) ) { // robot is too far away => GREEN
						WS2812SetDisplay(DISPLY_SYSTEM_1, 3, ucWS2812Intensity, 0, m_NewInteractionResult.ucFinalDataFromOtherRobot[uiDisplayChannel], 0);
						WS2812SetDisplay(DISPLY_SYSTEM_1, 4, ucWS2812Intensity, 0, m_NewInteractionResult.ucFinalDataFromOtherRobot[uiDisplayChannel], 0);
					}
					else { // robot is close => can detect the ID also => BLUE
						WS2812SetDisplay(DISPLY_SYSTEM_1, 3, ucWS2812Intensity, 0, 0, m_NewInteractionResult.ucFinalDataFromOtherRobot[uiDisplayChannel]);
						WS2812SetDisplay(DISPLY_SYSTEM_1, 4, ucWS2812Intensity, 0, 0, m_NewInteractionResult.ucFinalDataFromOtherRobot[uiDisplayChannel]);
					}
				}

				uiDisplayChannel = 2;
				if( (m_NewInteractionResult.ucFinalDataFromOtherRobot[uiDisplayChannel] != 0) ) { // there is signal detected from another robot => RED=0 and display GREEN / BLUE
					if( (m_NewIDResult.ucDetectedRobotID[uiDisplayChannel] == 0) ) { // robot is too far away => GREEN
						WS2812SetDisplay(DISPLY_SYSTEM_1, 5, ucWS2812Intensity, 0, m_NewInteractionResult.ucFinalDataFromOtherRobot[uiDisplayChannel], 0);
						WS2812SetDisplay(DISPLY_SYSTEM_1, 6, ucWS2812Intensity, 0, m_NewInteractionResult.ucFinalDataFromOtherRobot[uiDisplayChannel], 0);
					}
					else { // robot is close => can detect the ID also => BLUE
						WS2812SetDisplay(DISPLY_SYSTEM_1, 5, ucWS2812Intensity, 0, 0, m_NewInteractionResult.ucFinalDataFromOtherRobot[uiDisplayChannel]);
						WS2812SetDisplay(DISPLY_SYSTEM_1, 6, ucWS2812Intensity, 0, 0, m_NewInteractionResult.ucFinalDataFromOtherRobot[uiDisplayChannel]);
					}
				}

				uiDisplayChannel = 3;
				if( (m_NewInteractionResult.ucFinalDataFromOtherRobot[uiDisplayChannel] != 0) ) { // there is signal detected from another robot => RED=0 and display GREEN / BLUE
					if( (m_NewIDResult.ucDetectedRobotID[uiDisplayChannel] == 0) ) { // robot is too far away => GREEN
						WS2812SetDisplay(DISPLY_SYSTEM_1, 7, ucWS2812Intensity, 0, m_NewInteractionResult.ucFinalDataFromOtherRobot[uiDisplayChannel], 0);
						WS2812SetDisplay(DISPLY_SYSTEM_1, 8, ucWS2812Intensity, 0, m_NewInteractionResult.ucFinalDataFromOtherRobot[uiDisplayChannel], 0);
					}
					else { // robot is close => can detect the ID also => BLUE
						WS2812SetDisplay(DISPLY_SYSTEM_1, 7, ucWS2812Intensity, 0, 0, m_NewInteractionResult.ucFinalDataFromOtherRobot[uiDisplayChannel]);
						WS2812SetDisplay(DISPLY_SYSTEM_1, 8, ucWS2812Intensity, 0, 0, m_NewInteractionResult.ucFinalDataFromOtherRobot[uiDisplayChannel]);
					}
				}

				uiDisplayChannel = 4;
				if( (m_NewInteractionResult.ucFinalDataFromOtherRobot[uiDisplayChannel] != 0) ) { // there is signal detected from another robot => RED=0 and display GREEN / BLUE
					if( (m_NewIDResult.ucDetectedRobotID[uiDisplayChannel] == 0) ) { // robot is too far away => GREEN
						WS2812SetDisplay(DISPLY_SYSTEM_1, 9, ucWS2812Intensity, 0, m_NewInteractionResult.ucFinalDataFromOtherRobot[uiDisplayChannel], 0);
						WS2812SetDisplay(DISPLY_SYSTEM_1, 10, ucWS2812Intensity, 0, m_NewInteractionResult.ucFinalDataFromOtherRobot[uiDisplayChannel], 0);
					}
					else { // robot is close => can detect the ID also => BLUE
						WS2812SetDisplay(DISPLY_SYSTEM_1, 9, ucWS2812Intensity, 0, 0, m_NewInteractionResult.ucFinalDataFromOtherRobot[uiDisplayChannel]);
						WS2812SetDisplay(DISPLY_SYSTEM_1, 10, ucWS2812Intensity, 0, 0, m_NewInteractionResult.ucFinalDataFromOtherRobot[uiDisplayChannel]);
					}
				}

				uiDisplayChannel = 5;
				if( (m_NewInteractionResult.ucFinalDataFromOtherRobot[uiDisplayChannel] != 0) ) { // there is signal detected from another robot => RED=0 and display GREEN / BLUE
					if( (m_NewIDResult.ucDetectedRobotID[uiDisplayChannel] == 0) ) { // robot is too far away => GREEN
						WS2812SetDisplay(DISPLY_SYSTEM_1, 11, ucWS2812Intensity, 0, m_NewInteractionResult.ucFinalDataFromOtherRobot[uiDisplayChannel], 0);
						WS2812SetDisplay(DISPLY_SYSTEM_1, 12, ucWS2812Intensity, 0, m_NewInteractionResult.ucFinalDataFromOtherRobot[uiDisplayChannel], 0);
					}
					else { // robot is close => can detect the ID also => BLUE
						WS2812SetDisplay(DISPLY_SYSTEM_1, 11, ucWS2812Intensity, 0, 0, m_NewInteractionResult.ucFinalDataFromOtherRobot[uiDisplayChannel]);
						WS2812SetDisplay(DISPLY_SYSTEM_1, 12, ucWS2812Intensity, 0, 0, m_NewInteractionResult.ucFinalDataFromOtherRobot[uiDisplayChannel]);
					}
				}

				if ( (calculated_angle >-9) && (calculated_angle < 9) ) { // minimum angle is -9 or 9
					WheelRobotGoSpeedTmo(GOING_FORWARD, WHEEL_RUN_SPEED, 1);
				}
				else {
					WheelRobotRotateTmo(calculated_angle, 15);
				}

				WS2812ShowDisplay(DISPLY_SYSTEM_1, 0);
			}
		}
	}

	return;
}
//----------------------------------------------------------------------------------
void ApplicationTimer1KHZISR(void)	// 1KHZ timer function called from System ISR from stm32f4xx_it.c file @ SysTick Function (!!! keep as short as possible !!!)
{
	run_timer--;

	return;
}
//----------------------------------------------------------------------------------
#ifdef APPLICATION_DEBUG_MODE
void ApplicationDebugRXString(uint8_t *ucRXBuffer, uint8_t ucRXBufferLength)	// used to receive RX data from the debug interface
{

	return;
}
void ApplicationDebugRXChar(uint8_t ucRXBuffer)	// used to receive RX char from the debug interface
{

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
