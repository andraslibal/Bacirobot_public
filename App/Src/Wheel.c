#include "main.h"

#include "stdint.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"

#include "Wheel.h"


// STATIC (INTERN) prototypes
//==================================================================================================================== STATIC_START
// variables - external
extern TIM_HandleTypeDef htim3;		// RIGHT
extern TIM_HandleTypeDef htim4;		// LEFT
extern TIM_HandleTypeDef htim9;		// PWM counter running @ 20KHZ. to change PWM use

//----------------------------------------------------------------------------------


// internal defines
// distance between the wheels in mm (the diameter from the wheels perspective (2X43.5 mm)
#define WHEEL_BASE_MM								87
#define ROBOT_TURNING_CIRCLE_LENGTH_MM				3.1415 * WHEEL_BASE_MM
// Wheel properties
#define WHEEL_DIAMETER_MM							40
#define WHEEL_LENGTH_MM						3.1415 * WHEEL_DIAMETER_MM
// Right Encoder
#define ENCODER_PULSES_PER_ROTATION_RIGHT_WHEEL		1431 //* 3 // * 3 because of motor 150:1 !!!!!
//#define ENCODER_PULSES_PER_MM_RIGHT_WHEEL			ENCODER_PULSES_PER_ROTATION_RIGHT_WHEEL / WHEEL_LENGTH_MM
// Left Encoder
#define ENCODER_PULSES_PER_ROTATION_LEFT_WHEEL		1431 //* 3 // * 3 because of motor 150:1 !!!!!
//#define ENCODER_PULSES_PER_MM_LEFT_WHEEL			ENCODER_PULSES_PER_ROTATION_LEFT_WHEEL / WHEEL_LENGTH_MM
#define CORRECTION_VALUE_BECAUSE_OF_MOTOR			11
//----------------------------------------------------------------------------------


// variables - internal
#ifdef WHEEL_DEBUG_MODE
uint8_t m_ucDebugBuffer[100];
#endif
#define WHEEL_SKIP_REAL_TIME		20000
uint16_t m_uiWheelSkipRealTime;	// used in the main loop to skip real time
// for 1KHZ timer
#define TIMER_DIVIDER_MAX		100
volatile uint16_t m_ui16TimerDivider;			// used to take speed encoder measurements every n ms (ex: 100ms)
//----------------------------------------------------------------------------------
// encoders related variables
WheelEncoderTypeDef m_EncoderRight = {0};
WheelEncoderTypeDef m_EncoderLeft  = {0};

// motor related variables
#define STALL_MAX_COUNT_VALUE		5
// - STALL_MAX_COUNT_VALUE represents the number of consecutive stall conditions before setting the stall flag on the motor (zero speed while PWM is non-zero).
//       each stall detection is taken in TIMER_DIVIDER_MAX milliseconds
WheelMotorTypeDef m_MotorRight = {0};
WheelMotorTypeDef m_MotorLeft  = {0};
//----------------------------------------------------------------------------------

// prototypes
//----------------------------------------------------------------------------------
static void EncoderTimer1KHZ(void);		// used to handle the encoder related real time
//----------------------------------------------------------------------------------
static void MotorTimer1KHZ(void);		// used to handle the motors related real time
//----------------------------------------------------------------------------------
static void MotorPWMSpeedTracking(WheelMotorTypeDef * MotorData);		// used inside the MotorTimer1KHZ function, during the PWM Speed Tracking process in order to estimate the new PWM setting while
																	// getting closer to the PWM Speed Target
//----------------------------------------------------------------------------------
static uint8_t GetPWMSpeedStep(uint8_t ucActualSpeedDifference);		// used inside the MotorPWMSpeedTracking() to get the next setting for the PWMRampCounter while the PWM speed tracking
//----------------------------------------------------------------------------------
static void MotorMainLoop(void);		// used in the system main loop to handle thread related activities with the motors
//----------------------------------------------------------------------------------
static uint8_t MMSSpeedToPWM(uint16_t uiMMSSpeed);			// used to estimate the PWMPercentage as function of the MMSSpeed
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== STATIC_END


// API
//==================================================================================================================== API_START
void WheelRobotStop(void)		// stops both motors
{
	WheelRightStop();
	WheelLeftStop();
	return;
}
//----------------------------------------------------------------------------------
ROBOT_COMMAND_STATUS WheelRobotGoPWM(ROBOT_DIRECTION Direction, uint8_t ucPWMPercentage)	// used to command both wheels for a certain direction. PWM can be between 15% and 100%
{
	uint8_t ucCommandStatusR, ucCommandStatusL;

	if( (Direction != GOING_FORWARD ) && (Direction != GOING_BACKWARD) )					return STATE_PARAMETER_ERROR;
	if( (ucPWMPercentage < 15) || (ucPWMPercentage > 100) )									return STATE_PARAMETER_ERROR;

	// empty the eventual timeout setting before starting to run the engines
	//m_MotorRight.ucTimeToRunHMS = 0;
	//m_MotorLeft.ucTimeToRunHMS = 0;

	// empty the distance to go in case it contain some data
	//m_MotorRight.ui32DistanceToGoPulses = 0;
	//m_MotorLeft.ui32DistanceToGoPulses = 0;

	// Right Wheel
	ucCommandStatusR = WheelRightGo(Direction, ucPWMPercentage);
	// Left Wheel
	ucCommandStatusL = WheelLeftGo(Direction, ucPWMPercentage);

	if(ucCommandStatusR != STATE_OK)	return ucCommandStatusR;
	if(ucCommandStatusL != STATE_OK)	return ucCommandStatusL;

	return STATE_OK;
}
//----------------------------------------------------------------------------------
ROBOT_COMMAND_STATUS WheelRobotGoPWMTmo(ROBOT_DIRECTION Direction, uint8_t ucPWMPercentage, uint8_t ucTimeToRunHMS) // used to command both wheels for a certain direction and a specified time in Hundreds of MilliSeconds.
// - PWM can be between 15% and 100%
// - Time is in range of[1 ... 250] representing max 25.0 Seconds
{
	if( (Direction != GOING_FORWARD ) && (Direction != GOING_BACKWARD) )					return STATE_PARAMETER_ERROR;
	if( (ucPWMPercentage < 15) || (ucPWMPercentage > 100) )									return STATE_PARAMETER_ERROR;
	if( (ucTimeToRunHMS < 1)   || (ucTimeToRunHMS > 250) )									return STATE_PARAMETER_ERROR;

	// set the timeout => after the time is elapsed the "WheelRobotCommandEndCallback()" callback function will be called
	m_MotorRight.ucTimeToRunHMS = ucTimeToRunHMS;
	m_MotorLeft.ucTimeToRunHMS = ucTimeToRunHMS;

	// empty the distance to go in case it contain some data
	//m_MotorRight.ui32DistanceToGoPulses = 0;
	//m_MotorLeft.ui32DistanceToGoPulses = 0;

	// start the robot
	return WheelRobotGoPWM(Direction,ucPWMPercentage);

	//return STATE_OK;
}
//----------------------------------------------------------------------------------
ROBOT_COMMAND_STATUS WheelRobotGoPWMTmoDist(ROBOT_DIRECTION Direction, uint8_t ucPWMPercentage, uint8_t ucTimeToRunHMS, uint16_t uiDistanceToGoMM) // used to command both wheels for a certain direction and a specified time in Hundreds of MilliSeconds or a specific distance given by uiDistanceToGoMM in [mm]
// - PWM can be between 15% and 100%
// - Time is in range of[1 ... 250] representing max 25.0 Seconds
// - Distance is in range of [1 ... 10000] representing max 10 m
{
	if( (Direction != GOING_FORWARD ) && (Direction != GOING_BACKWARD) )					return STATE_PARAMETER_ERROR;
	if( (ucPWMPercentage < 15) || (ucPWMPercentage > 100) )									return STATE_PARAMETER_ERROR;
	if( (ucTimeToRunHMS < 1)   || (ucTimeToRunHMS > 250) )									return STATE_PARAMETER_ERROR;
	if( (uiDistanceToGoMM < 1) || (uiDistanceToGoMM > 10000) )								return STATE_PARAMETER_ERROR;

	// convert mm distance into pulses distance
	// WHEEL_LENGTH_MM corresponds to ENCODER_PULSES_PER_ROTATION amount of pulses
	// => for uiDistanceToGoMM distance the number of pulses is: No = ((uiDistanceToGoMM * ENCODER_PULSES_PER_ROTATION) / WHEEL_LENGTH_MM)
	double dRightWheelPulses, dLeftWheelPulses;
	uint32_t ui32RightPulses, ui32LeftPulses;

	dRightWheelPulses  = (double)uiDistanceToGoMM;
	dRightWheelPulses *= ENCODER_PULSES_PER_ROTATION_RIGHT_WHEEL;
	dRightWheelPulses /= WHEEL_LENGTH_MM;
	ui32RightPulses = (uint32_t)dRightWheelPulses;

	dLeftWheelPulses  = (double)uiDistanceToGoMM;
	dLeftWheelPulses *= ENCODER_PULSES_PER_ROTATION_LEFT_WHEEL;
	dLeftWheelPulses /= WHEEL_LENGTH_MM;
	ui32LeftPulses = (uint32_t)dLeftWheelPulses;

	if( (Direction == GOING_FORWARD) )
	{
		m_MotorRight.ui32DistanceToGoPulses = (m_EncoderRight.ui32TotalForwardPulses + ui32RightPulses);	// target pulses to go
		m_MotorLeft.ui32DistanceToGoPulses = (m_EncoderLeft.ui32TotalForwardPulses + ui32LeftPulses);	// target pulses to go
	}
	else
	{
		m_MotorRight.ui32DistanceToGoPulses = (m_EncoderRight.ui32TotalBackwardPulses + ui32RightPulses);	// target pulses to go
		m_MotorLeft.ui32DistanceToGoPulses = (m_EncoderLeft.ui32TotalBackwardPulses + ui32LeftPulses);	// target pulses to go
	}


	return WheelRobotGoPWMTmo(Direction,ucPWMPercentage,ucTimeToRunHMS);

	//return STATE_OK;
}
//----------------------------------------------------------------------------------
ROBOT_COMMAND_STATUS WheelRobotGoSpeed(ROBOT_DIRECTION Direction, uint16_t uiSpeedMMS)		//  used to command both wheels for a certain direction. the speed is expressed in mm/sec. range [50 .... 500]
{
	if( (Direction != GOING_FORWARD ) && (Direction != GOING_BACKWARD) )					return STATE_PARAMETER_ERROR;
	if( (uiSpeedMMS < 50) || (uiSpeedMMS > 500) )											return STATE_PARAMETER_ERROR;

	uint8_t ucEstimatedPWMPercentage;

	ucEstimatedPWMPercentage = MMSSpeedToPWM(uiSpeedMMS);

	m_MotorRight.uiRequestedSpeedMMS = uiSpeedMMS;
	m_MotorLeft.uiRequestedSpeedMMS = uiSpeedMMS;

	return WheelRobotGoPWM(Direction,ucEstimatedPWMPercentage);

	//return STATE_OK;
}
//----------------------------------------------------------------------------------
ROBOT_COMMAND_STATUS WheelRobotGoSpeedTmo(ROBOT_DIRECTION Direction, uint16_t uiSpeedMMS, uint8_t ucTimeToRunHMS) // used to command both wheels for a certain direction and a specified time in Hundreds of MilliSeconds using a specified speed.
// - Speed (uiSpeedMMS) can be between in range [50 .... 500] expressed in mm/s units
// - Time is in range of[1 ... 250] representing max 25.0 Seconds
{
	if( (Direction != GOING_FORWARD ) && (Direction != GOING_BACKWARD) )					return STATE_PARAMETER_ERROR;
	if( (uiSpeedMMS < 50) || (uiSpeedMMS > 500) )											return STATE_PARAMETER_ERROR;
	if( (ucTimeToRunHMS < 1) || (ucTimeToRunHMS > 250) )									return STATE_PARAMETER_ERROR;

	uint8_t ucEstimatedPWMPercentage;

	ucEstimatedPWMPercentage = MMSSpeedToPWM(uiSpeedMMS);

	m_MotorRight.uiRequestedSpeedMMS = uiSpeedMMS;
	m_MotorLeft.uiRequestedSpeedMMS = uiSpeedMMS;

	return WheelRobotGoPWMTmo(Direction, ucEstimatedPWMPercentage, ucTimeToRunHMS);

	//return STATE_OK;
}
//----------------------------------------------------------------------------------
ROBOT_COMMAND_STATUS WheelRobotGoSpeedTmoDist(ROBOT_DIRECTION Direction, uint16_t uiSpeedMMS, uint8_t ucTimeToRunHMS, uint16_t uiDistanceToGoMM) // used to command both wheels for a certain direction and a specified time in Hundreds of MilliSeconds or a specific distance given by uiDistanceToGoMM in [mm] using a specified speed.
// - Speed (uiSpeedMMS) can be between in range [50 .... 500] expressed in mm/s units
// - Time is in range of[1 ... 250] representing max 25.0 Seconds
// - Distance is in range of [1 ... 10000] representing max 10 m
{
	if( (Direction != GOING_FORWARD ) && (Direction != GOING_BACKWARD) )					return STATE_PARAMETER_ERROR;
	if( (uiSpeedMMS < 50) || (uiSpeedMMS > 500) )											return STATE_PARAMETER_ERROR;
	if( (ucTimeToRunHMS < 1) || (ucTimeToRunHMS > 250) )									return STATE_PARAMETER_ERROR;
	if( (uiDistanceToGoMM < 1) || (uiDistanceToGoMM > 10000) )								return STATE_PARAMETER_ERROR;

	uint8_t ucEstimatedPWMPercentage;

	ucEstimatedPWMPercentage = MMSSpeedToPWM(uiSpeedMMS);

	m_MotorRight.uiRequestedSpeedMMS = uiSpeedMMS;
	m_MotorLeft.uiRequestedSpeedMMS = uiSpeedMMS;


	return WheelRobotGoPWMTmoDist(Direction, ucEstimatedPWMPercentage, ucTimeToRunHMS, uiDistanceToGoMM);

	//return STATE_OK;
}
//----------------------------------------------------------------------------------
ROBOT_COMMAND_STATUS WheelRobotRotate(int16_t iRotationAngle)		// used to turn the robot with a certain angle in the range of[-180 ... 180] degrees. trigonometric direction is positive
{
	if( (iRotationAngle < -180 ) && (iRotationAngle > 180) )					return STATE_PARAMETER_ERROR;
	if( (iRotationAngle == 0) )													return STATE_OK;


	// the length of the circle on which the wheels are turning is pi*Distance between the wheels and is stored in the ROBOT_TURNING_CIRCLE_LENGTH_MM define
	// in case of 180 degree turn, each wheel must travel in opposite directions the half the circle
	// the distance to travel for a wheel is then given by: DistMM = (ROBOT_TURNING_CIRCLE_LENGTH_MM * iRotationAngle) / (2 * 180)
	double dTemp,dRightWheelPulses, dLeftWheelPulses;
	uint32_t ui32RightPulses, ui32LeftPulses;

	dTemp  = (double)abs(iRotationAngle);
	dTemp *= (double)ROBOT_TURNING_CIRCLE_LENGTH_MM;
	dTemp /= (double)360;	// the distance to go in mm for a certain input angle


	// Right Motor
	// set the timeout
	m_MotorRight.ucPWMPercentageTarget = 100;
	// convert distance to go mm/s into distance to go in pulses
	dRightWheelPulses  = dTemp;
	dRightWheelPulses *= ENCODER_PULSES_PER_ROTATION_RIGHT_WHEEL;
	dRightWheelPulses /= WHEEL_LENGTH_MM;
	//ui32RightPulses = (uint32_t)dRightWheelPulses;

	// Left Motor
	// set the timeout
	m_MotorLeft.ucPWMPercentageTarget = 100;
	// convert distance to go mm/s into distance to go in pulses
	dLeftWheelPulses  = dTemp;
	dLeftWheelPulses *= ENCODER_PULSES_PER_ROTATION_LEFT_WHEEL;
	dLeftWheelPulses /= WHEEL_LENGTH_MM;
	//ui32LeftPulses = (uint32_t)dLeftWheelPulses;


	if( (iRotationAngle > 0) )
	{	// turn to LEFT is positive (trigonometric direction)
		dRightWheelPulses -= ((dRightWheelPulses * 10) / 100);	// correction factor
		if(dRightWheelPulses < 0) dRightWheelPulses = 0;
		ui32RightPulses = (uint32_t)dRightWheelPulses;
		dLeftWheelPulses -= ((dLeftWheelPulses * 10) / 100);	// correction factor
		if(dLeftWheelPulses < 0) dLeftWheelPulses = 0;
		ui32LeftPulses = (uint32_t)dLeftWheelPulses;
		m_MotorRight.ui32DistanceToGoPulses = (m_EncoderRight.ui32TotalForwardPulses + ui32RightPulses);	// target pulses to go
		m_MotorLeft.ui32DistanceToGoPulses  = (m_EncoderLeft.ui32TotalBackwardPulses + ui32LeftPulses);		// target pulses to go
		WheelRightGo(GOING_FORWARD,15);		// PWM speed is 15%
		WheelLeftGo(GOING_BACKWARD,15);		// PWM speed is 15%
	}
	else
	{	// turn to RIGHT is negative
		dRightWheelPulses -= ((dRightWheelPulses * 10) / 100);	// correction factor
		if(dRightWheelPulses < 0) dRightWheelPulses = 0;
		ui32RightPulses = (uint32_t)dRightWheelPulses;
		dLeftWheelPulses -= ((dLeftWheelPulses * 10) / 100);	// correction factor
		if(dLeftWheelPulses < 0) dLeftWheelPulses = 0;
		ui32LeftPulses = (uint32_t)dLeftWheelPulses;
		m_MotorRight.ui32DistanceToGoPulses = (m_EncoderRight.ui32TotalBackwardPulses + ui32RightPulses);	// target pulses to go
		m_MotorLeft.ui32DistanceToGoPulses  = (m_EncoderLeft.ui32TotalForwardPulses + ui32LeftPulses);		// target pulses to go
		WheelRightGo(GOING_BACKWARD,15);	// PWM speed is 15%
		WheelLeftGo(GOING_FORWARD,15);		// PWM speed is 15%
	}


	return STATE_OK;
}
//----------------------------------------------------------------------------------
ROBOT_COMMAND_STATUS WheelRobotRotateTmo(int16_t iRotationAngle, uint8_t ucPWMPercentage)		// used to turn the robot with a certain angle in the range of[-180 ... 180] degrees. trigonometric direction is positive using timeout
{
	if( (iRotationAngle < -180 ) && (iRotationAngle > 180) )					return STATE_PARAMETER_ERROR;
	if( (iRotationAngle > -9  ) && (iRotationAngle < 9) )						return STATE_PARAMETER_ERROR;
	if( (iRotationAngle == 0) )													return STATE_OK;

	double dTemp;
	dTemp  = (double)abs(iRotationAngle);
	dTemp *= CORRECTION_VALUE_BECAUSE_OF_MOTOR;
	dTemp /= 90;

	m_MotorRight.ucTimeToRunHMS = dTemp;
	m_MotorLeft.ucTimeToRunHMS = dTemp;

	if( (iRotationAngle > 0) )
	{	// turn to LEFT is positive (trigonometric direction)
		WheelRightGo(GOING_FORWARD,ucPWMPercentage);
		WheelLeftGo(GOING_BACKWARD,ucPWMPercentage);
	}
	else
	{	// turn to RIGHT is negative
		WheelRightGo(GOING_BACKWARD,ucPWMPercentage);
		WheelLeftGo(GOING_FORWARD,ucPWMPercentage);
	}

	return STATE_OK;
}
//----------------------------------------------------------------------------------
ROBOT_COMMAND_STATUS WheelRightGo(ROBOT_DIRECTION Direction, uint8_t ucPWMPercentage)	// used to command the Right motor. PWM is in the range of 15% ... 100%
{
	if( (Direction != GOING_FORWARD ) && (Direction != GOING_BACKWARD) )					return STATE_PARAMETER_ERROR;
	if( (ucPWMPercentage < 15) || (ucPWMPercentage > 100) )									return STATE_PARAMETER_ERROR;
	if(m_MotorRight.DirectionChangeWithoutStop == ROTATION_CHANGE_DETECTED)					return STATE_BUSY;		// too many commands and the last one (involving a rotation direction change is not yet done

	// check if the wheel is not presently turning in the opposite direction. in case it dose return busy state and send stop command to the wheel
	if ( (Direction == GOING_FORWARD) && (m_MotorRight.TurningDirection == WHEEL_TURNING_BACKWARD) )
	{
		m_MotorRight.DirectionChangeWithoutStop = ROTATION_CHANGE_DETECTED;	// most turn in opposite direction
		m_MotorRight.ucPWMPercentageTarget2 = ucPWMPercentage;
		// stop the motor first
		m_MotorRight.ucPWMPercentageTarget = 0;
		m_MotorRight.PWMSpeedStatus = PWM_SPEED_TRACKING;
		//WheelRightStop();
		return STATE_OK;
	}
	if ( (Direction == GOING_BACKWARD) && (m_MotorRight.TurningDirection == WHEEL_TURNING_FORWARD) )
	{
		m_MotorRight.DirectionChangeWithoutStop = ROTATION_CHANGE_DETECTED;	// most turn in opposite direction
		m_MotorRight.ucPWMPercentageTarget2 = ucPWMPercentage;
		// stop the motor first
		m_MotorRight.ucPWMPercentageTarget = 0;
		m_MotorRight.PWMSpeedStatus = PWM_SPEED_TRACKING;
		//WheelRightStop();
		return STATE_OK;
	}

	// there is no rotation change while the wheel was previously spinning in the opposite direction
	m_MotorRight.DirectionChangeWithoutStop = NO_ROTATION_CHANGE_DETECTED;
	m_MotorRight.ucPWMPercentageTarget2 = 0;

	m_MotorRight.ucPWMPercentageTarget = ucPWMPercentage;
	m_MotorRight.PWMSpeedStatus = PWM_SPEED_TRACKING;
	//__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,(uint16_t)m_MotorRight.ucPWMPercentageTarget);  //(commented here and ramped inside the MotorTimer1KHZ)

	if( Direction == GOING_FORWARD )
	{	// forward
		m_MotorRight.TurningDirection = WHEEL_TURNING_FORWARD;
		HAL_GPIO_WritePin(RIN1_GPIO_Port, RIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RIN1_GPIO_Port, RIN2_Pin, GPIO_PIN_SET);
	}
	else
	{	// backward
		m_MotorRight.TurningDirection = WHEEL_TURNING_BACKWARD;
		HAL_GPIO_WritePin(RIN1_GPIO_Port, RIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RIN1_GPIO_Port, RIN2_Pin, GPIO_PIN_RESET);
	}

	return STATE_OK;
}
//----------------------------------------------------------------------------------
void WheelRightStop(void)		// used to stop the right motor
{
	m_MotorRight.ucPWMPercentageTarget = 0;
	m_MotorRight.DirectionChangeWithoutStop = NO_ROTATION_CHANGE_DETECTED;
	m_MotorRight.ucPWMPercentageTarget2 = 0;
	m_MotorRight.ucTimeToRunHMS = 0;		// timeout
	m_MotorRight.ui32DistanceToGoPulses = 0;	// no pulse target
	m_MotorRight.uiRequestedSpeedMMS  = 0;		// no speed control
	//m_MotorRight.TurningDirection = WHEEL_NOT_TURNING;
	m_MotorRight.PWMSpeedStatus = PWM_SPEED_TRACKING;	// start the stopping process
	//__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,(uint16_t)m_MotorRight.ucPWMPercentageTarget);		// set PWM CH1@Tim9 to ZERO  //(commented here and ramped inside the MotorTimer1KHZ)
	//HAL_GPIO_WritePin(RIN1_GPIO_Port, RIN1_Pin, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(RIN1_GPIO_Port, RIN2_Pin, GPIO_PIN_RESET);

	return;
}
//----------------------------------------------------------------------------------
ROBOT_COMMAND_STATUS WheelLeftGo(ROBOT_DIRECTION Direction, uint8_t ucPWMPercentage)	// used to command the Right motor. PWM is in the range of 15% ... 100%
{
	if( (Direction != GOING_FORWARD ) && (Direction != GOING_BACKWARD) )					return STATE_PARAMETER_ERROR;
	if( (ucPWMPercentage < 15) || (ucPWMPercentage > 100) )									return STATE_PARAMETER_ERROR;
	if(m_MotorLeft.DirectionChangeWithoutStop == ROTATION_CHANGE_DETECTED)					return STATE_BUSY;		// too many commands and the last one (involving a rotation direction change is not yet done

	// check if the wheel is not presently turning in the opposite direction. in case it dose return busy state and send stop command to the wheel
	if ( (Direction == GOING_FORWARD) && (m_MotorLeft.TurningDirection == WHEEL_TURNING_BACKWARD) )
	{
		m_MotorLeft.DirectionChangeWithoutStop = ROTATION_CHANGE_DETECTED;	// most turn in opposite direction
		m_MotorLeft.ucPWMPercentageTarget2 = ucPWMPercentage;
		// stop the motor first
		m_MotorLeft.ucPWMPercentageTarget = 0;
		m_MotorLeft.PWMSpeedStatus = PWM_SPEED_TRACKING;
		//WheelLeftStop();
		return STATE_OK;
	}
	if ( (Direction == GOING_BACKWARD) && (m_MotorLeft.TurningDirection == WHEEL_TURNING_FORWARD) )
	{
		m_MotorLeft.DirectionChangeWithoutStop = ROTATION_CHANGE_DETECTED;	// most turn in opposite direction
		m_MotorLeft.ucPWMPercentageTarget2 = ucPWMPercentage;
		// stop the motor first
		m_MotorLeft.ucPWMPercentageTarget = 0;
		m_MotorLeft.PWMSpeedStatus = PWM_SPEED_TRACKING;
		//WheelLeftStop();
		return STATE_OK;
	}

	// there is no rotation change while the wheel was previously spinning in the opposite direction
	m_MotorLeft.DirectionChangeWithoutStop = NO_ROTATION_CHANGE_DETECTED;
	m_MotorLeft.ucPWMPercentageTarget2 = 0;

	m_MotorLeft.ucPWMPercentageTarget = ucPWMPercentage;
	m_MotorLeft.PWMSpeedStatus = PWM_SPEED_TRACKING;

	//__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2,(uint16_t)m_MotorLeft.ucPWMPercentageTarget); //(commented here and ramped inside the MotorTimer1KHZ)
	if( Direction == GOING_FORWARD )
	{	// forward
		m_MotorLeft.TurningDirection = WHEEL_TURNING_FORWARD;
		HAL_GPIO_WritePin(LIN1_GPIO_Port, LIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LIN1_GPIO_Port, LIN2_Pin, GPIO_PIN_RESET);
	}
	else
	{	// backward
		m_MotorLeft.TurningDirection = WHEEL_TURNING_BACKWARD;
		HAL_GPIO_WritePin(LIN1_GPIO_Port, LIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LIN1_GPIO_Port, LIN2_Pin, GPIO_PIN_SET);
	}

	return STATE_OK;
}
//----------------------------------------------------------------------------------
void WheelLeftStop(void)		// used to stop the left motor
{
	m_MotorLeft.ucPWMPercentageTarget = 0;
	m_MotorLeft.DirectionChangeWithoutStop = NO_ROTATION_CHANGE_DETECTED;
	m_MotorLeft.ucPWMPercentageTarget2 = 0;
	m_MotorLeft.ui32DistanceToGoPulses = 0;	// avoid getting false callback
	m_MotorLeft.ucTimeToRunHMS = 0;			// avoid getting false callback
	m_MotorLeft.uiRequestedSpeedMMS  = 0;	// no speed control
	//m_MotorLeft.TurningDirection = WHEEL_NOT_TURNING;
	m_MotorLeft.PWMSpeedStatus = PWM_SPEED_TRACKING;	// start the stopping process
	//__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2,(uint16_t)m_MotorLeft.ucPWMPercentageTarget);		// set PWM CH2@Tim9 to ZERO  //(commented here and ramped inside the MotorTimer1KHZ)
	//HAL_GPIO_WritePin(RIN1_GPIO_Port, LIN1_Pin, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(RIN1_GPIO_Port, LIN2_Pin, GPIO_PIN_RESET);
	return;
}
//----------------------------------------------------------------------------------
ROBOT_COMMAND_STATUS WheelLefRighttGo(ROBOT_DIRECTION Direction, uint8_t ucPWMPercentage, uint8_t ucLeftRightBalance)	// used to command the both motors. (mainly useful for line following)
// - PWM is in the range of 15% ... 100%
// - ucLeftRightBalance is in the range of [1 ... 200]. neutral is 100 with both engines @ the same speed.
// --- 1 	=> Left motor @ maximum speed and Right motor slowest speed
// --- 200	=> Left engine has the slowest speed and the Right engine is @ maximum speed
{
	if( (Direction != GOING_FORWARD ) && (Direction != GOING_BACKWARD) )					return STATE_PARAMETER_ERROR;
	if( (ucPWMPercentage < 15) || (ucPWMPercentage > 100) )									return STATE_PARAMETER_ERROR;
	if( (ucLeftRightBalance < 1) || (ucLeftRightBalance > 200) )							return STATE_PARAMETER_ERROR;

	int16_t iTempL, iTempR, iBalance;

	iTempL = (int16_t)ucPWMPercentage;
	iTempR = (int16_t)ucPWMPercentage;
	iBalance = (int16_t)ucLeftRightBalance;

	iTempL += (100 - iBalance);
	if(iTempL < 15) 	iTempL = 15;
	if(iTempL > 100)	iTempL = 100;

	iTempR += (iBalance - 100);
	if(iTempR < 15) 	iTempR = 15;
	if(iTempR > 100)	iTempR = 100;

//#ifdef WHEEL_DEBUG_MODE
//	sprintf((char *)m_ucDebugBuffer,"Dir=%d, PWML=%d, PWMR=%d, In=%d:%d\n\r",Direction,iTempL,iTempR,ucPWMPercentage,ucLeftRightBalance);
//	DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
//#endif

	WheelLeftGo(Direction, (uint8_t)iTempL);
	WheelRightGo(Direction, (uint8_t)iTempR);

	return STATE_OK;
}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
__attribute__((weak)) void WheelRobotCommandEndCallback(COMMAND_CALLBACK_REASON CallbackReason)		// called after each command finalization [__attribute__((weak)) ]
{
	//UNUSED(CallbackReason);		// to avoid warning

	// debug
#ifdef WHEEL_DEBUG_MODE
	sprintf((char *)m_ucDebugBuffer,"CommandEndCallback = %d\r\n",CallbackReason);
	DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
#endif
	return;
}
//----------------------------------------------------------------------------------
__attribute__((weak)) void WheelStallRightCallback(void)		// called if the will stalled (driving signal is not zero but the turning speed is zero) for the right motor [__attribute__((weak)) ]
{
	// debug
#ifdef WHEEL_DEBUG_MODE
	sprintf((char *)m_ucDebugBuffer,"RIGHT Wheel Stalled !!!\r\n");
	DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
#endif
	return;
}
//----------------------------------------------------------------------------------
__attribute__((weak)) void WheelStallLeftCallback(void)			// called if the will stalled (driving signal is not zero but the turning speed is zero) for the left motor [__attribute__((weak)) ]
{
	// debug
#ifdef WHEEL_DEBUG_MODE
	sprintf((char *)m_ucDebugBuffer,"LEFT Wheel Stalled !!!\r\n");
	DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
#endif
	return;
}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
uint16_t GetWheelSpeed(WHEEL_SIDE Side)		// returns the speed of the selected wheel in mm/s units
{
	if( (Side != SIDE_RIGHT) && (Side != SIDE_LEFT) )		return 0xFFFF;	// return some error code

	if(Side == SIDE_RIGHT)	return m_EncoderRight.uiSpeedMMS;
	else 					return m_EncoderLeft.uiSpeedMMS;
}
//----------------------------------------------------------------------------------
WHEEL_STALL_STATUS GetWheelStallCondition(WHEEL_SIDE Side)	// returns if the wheel is stalled or not
{
	if( (Side != SIDE_RIGHT) && (Side != SIDE_LEFT) )		return	0xFF;	// return some error code

	if(Side == SIDE_RIGHT)	return m_MotorRight.StallCondition;
	else  					return m_MotorLeft.StallCondition;
}
//----------------------------------------------------------------------------------
WHEEL_TURNING_DIRECTION GetWheelTurningDirection(WHEEL_SIDE Side)	// returns the momentary turning direction of the wheel
{
	if( (Side != SIDE_RIGHT) && (Side != SIDE_LEFT) ) 		return	0xFF;

	if(Side == SIDE_RIGHT)	return m_EncoderRight.TurningDirection;
	else  					return m_EncoderLeft.TurningDirection;
}
//----------------------------------------------------------------------------------
uint32_t GetWheelForwardPulses(WHEEL_SIDE Side)		// returns the total forward pulses of that wheel as counted by the wheel encoder since the last reset
{
	if( (Side != SIDE_RIGHT) && (Side != SIDE_LEFT) )		return 0xFFFFFFFF;	// return some error code

	if(Side == SIDE_RIGHT)	return m_EncoderRight.ui32TotalForwardPulses;
	else 					return m_EncoderLeft.ui32TotalForwardPulses;
}
//----------------------------------------------------------------------------------
uint32_t GetWheelBackwardPulses(WHEEL_SIDE Side)	// returns the total backward pulses of that wheel as counted by the wheel encoder since the last reset
{
	if( (Side != SIDE_RIGHT) && (Side != SIDE_LEFT) )		return 0xFFFFFFFF;	// return some error code

	if(Side == SIDE_RIGHT)	return m_EncoderRight.ui32TotalBackwardPulses;
	else 					return m_EncoderLeft.ui32TotalBackwardPulses;
}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== API_END


// SYSTEM
//==================================================================================================================== SYSTEM_START
void WheelInit(void)	// used to initialize the Wheel Module. Called from main.c file @ init section
{
	//Start the Encoder timers
	if(HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL) != HAL_OK)	// Right
    {
	    Error_Handler2(ERROR_CODE_WHEEL_INIT_001);
		//Error_Handler();
    }
	if(HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL) != HAL_OK) // Left
	{
		Error_Handler2(ERROR_CODE_WHEEL_INIT_002);
		//Error_Handler();
	}
	if(HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler2(ERROR_CODE_WHEEL_INIT_003);
		//Error_Handler();
	}
	if(HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler2(ERROR_CODE_WHEEL_INIT_004);
		//Error_Handler();
	}

	// timer 1KHZ
	m_ui16TimerDivider = 0;

	// encoder setup
	m_EncoderRight.EncoderSide = SIDE_RIGHT;
	m_EncoderLeft.EncoderSide = SIDE_LEFT;

	// motors
	m_MotorRight.MotorSide = SIDE_RIGHT;
	m_MotorLeft.MotorSide = SIDE_LEFT;
	m_MotorRight.ucStallMaxCountValue = STALL_MAX_COUNT_VALUE;
	m_MotorLeft.ucStallMaxCountValue = STALL_MAX_COUNT_VALUE;





	return;
}
//----------------------------------------------------------------------------------
void WheelMainLoop(void)	// loop function of the Wheel Module. Called from the main.c file @ main loop section
{
	// skip real time
	if(m_uiWheelSkipRealTime)
	{
		m_uiWheelSkipRealTime --;
		return;
	}
	m_uiWheelSkipRealTime = WHEEL_SKIP_REAL_TIME;
	//--------------------------------------------------------


	// -------------------------------------------------- ENCODER START

	// -------------------------------------------------- ENCODER END


	// -------------------------------------------------- MOTOR START
	MotorMainLoop();		// motor thread activities
	// -------------------------------------------------- MOTOR END





	// debug
#ifdef WHEEL_DEBUG_MODE

	//--------
	//sprintf((char *)m_ucDebugBuffer,"RMMS=%d,PWM=%d, LMMS=%dPWM=%d\r\n",m_EncoderRight.uiSpeedMMS,m_MotorRight.ucPWMPercentageTarget,
	//																	m_EncoderLeft.uiSpeedMMS,m_MotorLeft.ucPWMPercentageTarget);
	//DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
	// --------


	/*
	// RIGHT Encoder
	if(m_EncoderRight.UpdateStatus == NEW_ENCODER_VALUE)
	{
		m_EncoderRight.UpdateStatus = NO_ENCODER_UPDATE;
		sprintf((char *)m_ucDebugBuffer,"RIGHT = %u, %d, F=%ld, B=%ld\r\n",m_EncoderRight.uiSpeedMMS,(int)m_EncoderRight.TurningDirection,
																			m_EncoderRight.ui32TotalForwardPulses, m_EncoderRight.ui32TotalBackwardPulses);
		DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
	}
	*/


	/*
	// LEFT Encoder
	if(m_EncoderLeft.UpdateStatus == NEW_ENCODER_VALUE)
	{
		m_EncoderLeft.UpdateStatus = NO_ENCODER_UPDATE;
		sprintf((char *)m_ucDebugBuffer,"LEFT = %u, %d, F=%ld, B=%ld\r\n",m_EncoderLeft.uiSpeedMMS,(int)m_EncoderLeft.TurningDirection,
																		m_EncoderLeft.ui32TotalForwardPulses, m_EncoderLeft.ui32TotalBackwardPulses);
		DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
	}
	*/



	/*
	// Both
	if( (m_EncoderLeft.UpdateStatus == NEW_ENCODER_VALUE) && (m_EncoderRight.UpdateStatus == NEW_ENCODER_VALUE))
	{
		m_EncoderRight.UpdateStatus = NO_ENCODER_UPDATE;
		m_EncoderLeft.UpdateStatus  = NO_ENCODER_UPDATE;
		sprintf((char *)m_ucDebugBuffer,"L=%u, %d  R=%u, %d\r\n",m_EncoderLeft.uiSpeedMMS,(int)m_EncoderLeft.TurningDirection,
																	m_EncoderRight.uiSpeedMMS,(int)m_EncoderRight.TurningDirection);
		DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
	}
	*/
#endif



	return;
}
//----------------------------------------------------------------------------------
void WheelTimer1KHZISR(void)	// 1KHZ timer function called from System ISR from stm32f4xx_it.c file @ SysTick Function (!!! keep as short as possible !!!)
{	// max 2.2uS
	if(m_ui16TimerDivider >= TIMER_DIVIDER_MAX) 	m_ui16TimerDivider = 0;
	m_ui16TimerDivider ++;

	// -------------------------------------------------- ENCODER START
	// the speed of the encoders is measured every 100 ms => @ a speed of 1m/s 	= 100cm/s 	= 3.6km/h 	=> 1144.8 encoder pulses (the 100ms is set by TIMER_DIVIDER_MAX)
	// the speed of the encoders is measured every 100 ms => @ a speed of 0.1m/s = 10cm/s 	= 0.36km/h 	=> 114.48 encoder pulses
	// values for m_ui16TimerEncoderDivider = [1 ... 100]
	// interrupt times =>
	//		@ 10 => 800ns				(Read Right Encoder and establish the pulse difference between 2 readings and also determine the wheel direction) (uiPulseDiference, TurningDirection)
	// 		@ 11 => 800ns				(Read Left  Encoder and establish the pulse difference between 2 readings and also determine the wheel direction) (uiPulseDiference, TurningDirection)
	//		@ 20 => 400ns ... 650ns		(Calculate the Right Speed in mm/sec and mark the update status variable) (uiSpeedMMS, UpdateStatus)
	//		@ 30 => 400ns ... 650ns		(Calculate the Left  Speed in mm/sec and mark the update status variable) (uiSpeedMMS, UpdateStatus)
	//-----------------------------
#ifdef WHEEL_DEBUG_MODE
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);	// debug on PC13 HIGH
#endif
	EncoderTimer1KHZ();		// encoder timer
#ifdef WHEEL_DEBUG_MODE
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);	// debug on PC13 LOW
#endif
	// -------------------------------------------------- ENCODER END

	// -------------------------------------------------- MOTOR START
	// PWM frequency is 20KHZ => Period is 50uS => one update of the PWM Setting after every 50uS is applied to the motors
	// System timer is 1mS => 20 motor updates between 2 consecutive system timer interrupts
	// there are the following index entries:
	//	@ 15  =>  Ramp the PWM for the RIGHT Motor	// max 2.2uS, typically 400nS
	//  @ 16  =>  Ramp the PWM for the LEFT  Motor	// max 2.2uS, typically 400nS
	//  @ 25  =>  Test the RIGHT Motor Stall condition (it must always follow the Timer Encoder Calculate Right Speed (in this case is 20)) // max 700nS, typically 500nS
	//  @ 35  =>  Test the LEFT  Motor Stall condition (it must always follow the Timer Encoder Calculate Left  Speed (in this case is 30)) // max 700nS, typically 500nS
	//  @ 37  =>  RIGHT and LEFT Wheel Timeout check and trigger Callback function at timeout // max 600nS, typically 450nS
	//  @ 39  =>  RIGHT and LEFT Wheel Distance to Go // max 1.8uS, typically 430nS
	//  @ 41  =>  MMsSpeed to PWM Feedback loop	//max 1.3uS, typically 500nS
	MotorTimer1KHZ();	// motor loop control timer
	// -------------------------------------------------- MOTOR END


	return;
}
//----------------------------------------------------------------------------------
#ifdef WHEEL_DEBUG_MODE
uint16_t m_ui16DebugTemp, m_ui16DebugTemp2;
static uint8_t m_ucTemp, m_ucTemp2;
uint8_t m_ucReturnedValue;
void WheelDebugRXString(uint8_t *ucRXBuffer, uint8_t ucRXBufferLength)	// used to receive RX data from the debug interface
{
	return;
}
void WheelDebugRXChar(uint8_t ucRXChar)	// used to receive RX single char from the debug interface
{
	ROBOT_COMMAND_STATUS ReturnedStatus;
	// Motor Driver Debug Commands
	// Right Wheel
	if(ucRXChar == 'q')
	{
		//ReturnedStatus = WheelRightGo(GOING_FORWARD,15);
		ReturnedStatus = WheelRightGo(GOING_BACKWARD,15);
		sprintf((char *)m_ucDebugBuffer,"Status=%d\n\r",ReturnedStatus);
		DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
	}
	if(ucRXChar == 'w')
	{
		//ReturnedStatus = WheelRightGo(GOING_FORWARD,100);
		ReturnedStatus = WheelRightGo(GOING_BACKWARD,100);
		sprintf((char *)m_ucDebugBuffer,"Status=%d\n\r",ReturnedStatus);
		DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
	}
	if(ucRXChar == 'e')
	{
		ReturnedStatus = WheelRightGo(GOING_FORWARD,40);
		sprintf((char *)m_ucDebugBuffer,"Status=%d\n\r",ReturnedStatus);
		DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
	}
	if(ucRXChar == 'r')
	{
		WheelRightStop();
		sprintf((char *)m_ucDebugBuffer,"Right STOP\n\r");
		DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
	}
	// Right Wheel
	if(ucRXChar == 'a')
	{
		//ReturnedStatus = WheelLeftGo(GOING_FORWARD,15);
		ReturnedStatus = WheelLeftGo(GOING_BACKWARD,15);
		sprintf((char *)m_ucDebugBuffer,"Status=%d\n\r",ReturnedStatus);
		DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
	}
	if(ucRXChar == 's')
	{
		//ReturnedStatus = WheelLeftGo(GOING_FORWARD,100);
		ReturnedStatus = WheelLeftGo(GOING_BACKWARD,100);
		sprintf((char *)m_ucDebugBuffer,"Status=%d\n\r",ReturnedStatus);
		DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
	}
	if(ucRXChar == 'd')
	{
		ReturnedStatus = WheelLeftGo(GOING_FORWARD,30);
		sprintf((char *)m_ucDebugBuffer,"Status=%d\n\r",ReturnedStatus);
		DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
	}
	if(ucRXChar == 'f')
	{
		WheelLeftStop();
		sprintf((char *)m_ucDebugBuffer,"Left STOP\n\r");
		DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
	}
	// m_ucTemp 	=> PWMSpeed
	// m_ucTemp2 	=> Balance
	if(ucRXChar == 'z')
	{
		if( (m_ucTemp > 15) )	m_ucTemp --;
		WheelLefRighttGo(GOING_FORWARD,m_ucTemp,m_ucTemp2);
	}
	if(ucRXChar == 'x')
	{
		if( (m_ucTemp < 100) )	m_ucTemp ++;
		WheelLefRighttGo(GOING_FORWARD,m_ucTemp,m_ucTemp2);
	}
	if(ucRXChar == 'c')
	{
		if( (m_ucTemp2 > 1) )	m_ucTemp2 --;
		WheelLefRighttGo(GOING_FORWARD,m_ucTemp,m_ucTemp2);
	}
	if(ucRXChar == 'v')
	{
		if( (m_ucTemp2 < 200) )	m_ucTemp2 ++;
		WheelLefRighttGo(GOING_FORWARD,m_ucTemp,m_ucTemp2);
	}
	//WheelRobotStop
	if(ucRXChar == 'b')
	{
		WheelRobotStop();
	}
	// timeout test
	if(ucRXChar == 'n')	 WheelRobotGoPWMTmo(GOING_FORWARD,40,5);	// 0.5 seconds with PWM@40%
	if(ucRXChar == 'm')	 WheelRobotGoPWMTmo(GOING_FORWARD,80,100);	// 10 seconds with PWM@40%


	// ROBOT_COMMAND_STATUS WheelRobotGoPWMTmoDist(ROBOT_DIRECTION Direction, uint8_t ucPWMPercentage, uint8_t ucTimeToRunHMS, uint16_t uiDistanceToGoMM)
	if(ucRXChar == 'k')
	{
		m_ucReturnedValue = WheelRobotGoPWMTmoDist(GOING_FORWARD,50,100,300);
		sprintf((char *)m_ucDebugBuffer,"Value = %d\n\r",m_ucReturnedValue);
		DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
	}

	if(ucRXChar == 'l')
	{
		m_ucReturnedValue = WheelRobotGoPWMTmoDist(GOING_BACKWARD,30,100,200);
		sprintf((char *)m_ucDebugBuffer,"Value = %d\n\r",m_ucReturnedValue);
		DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
	}

	if(ucRXChar == 'p')
	{
		m_ucReturnedValue = WheelRobotGoPWMTmoDist(GOING_FORWARD,100,100,4000);
		sprintf((char *)m_ucDebugBuffer,"Value = %d\n\r",m_ucReturnedValue);
		DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
	}

	if(ucRXChar == ';')
	{
		m_ucReturnedValue = WheelRobotGoPWMTmoDist(GOING_FORWARD,100,10,4000);
		sprintf((char *)m_ucDebugBuffer,"Value = %d\n\r",m_ucReturnedValue);
		DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
	}

	//ROBOT_COMMAND_STATUS WheelRobotGoSpeed(ROBOT_DIRECTION Direction, uint16_t uiSpeedMMS)
	if(ucRXChar == '[')
	{
		m_ucReturnedValue = WheelRobotGoSpeed(GOING_FORWARD,250);
		sprintf((char *)m_ucDebugBuffer,"Value = %d\n\r",m_ucReturnedValue);
		DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
	}
	if(ucRXChar == ']')
	{
		m_ucReturnedValue = WheelRobotGoSpeed(GOING_FORWARD,400);
		sprintf((char *)m_ucDebugBuffer,"Value = %d\n\r",m_ucReturnedValue);
		DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
	}

	// WheelRobotGoSpeedTmoDist(ROBOT_DIRECTION Direction, uint16_t uiSpeedMMS, uint8_t ucTimeToRunHMS, uint16_t uiDistanceToGoMM)
	if(ucRXChar == '>')
	{
		m_ucReturnedValue = WheelRobotGoSpeedTmo(GOING_FORWARD,500,100);
		sprintf((char *)m_ucDebugBuffer,"Value = %d\n\r",m_ucReturnedValue);
		DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
	}
	if(ucRXChar == '<')
	{
		m_ucReturnedValue = WheelRobotGoSpeedTmo(GOING_BACKWARD,500,125);
		sprintf((char *)m_ucDebugBuffer,"Value = %d\n\r",m_ucReturnedValue);
		DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
	}

	// WheelRobotGoSpeedTmoDist(ROBOT_DIRECTION Direction, uint16_t uiSpeedMMS, uint8_t ucTimeToRunHMS, uint16_t uiDistanceToGoMM)
	if(ucRXChar == '{')
	{
		m_ucReturnedValue = WheelRobotGoSpeedTmoDist(GOING_FORWARD,300,125,500);
		sprintf((char *)m_ucDebugBuffer,"Value = %d\n\r",m_ucReturnedValue);
		DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
	}

	if(ucRXChar == '}')
	{
		m_ucReturnedValue = WheelRobotGoSpeedTmoDist(GOING_BACKWARD,100,125,500);
		sprintf((char *)m_ucDebugBuffer,"Value = %d\n\r",m_ucReturnedValue);
		DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
	}
	// WheelRobotRotate
	if(ucRXChar == '1')
	{
		m_ucReturnedValue = WheelRobotRotate(90);
	}
	if(ucRXChar == '2')
	{
		m_ucReturnedValue = WheelRobotRotate(-90);
	}



	/*
	if(ucRXChar == 'q')
	{	// read the 2 encoders CNT values
		sprintf((char *)m_ucDebugBuffer,"Left=%ld Right=%ld\n\r",__HAL_TIM_GET_COUNTER(&htim4),__HAL_TIM_GET_COUNTER(&htim3));
		DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
		return;
	}

	if(ucRXChar == 'a')
	{	// set Tim9 PWM duty (++) CHANNEL1
		if(m_ui16DebugTemp < 99)
		{
			m_ui16DebugTemp ++;
			__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,m_ui16DebugTemp);
			sprintf((char *)m_ucDebugBuffer,"Left=%ld Right=%ld, PWMR=%d\n\r",__HAL_TIM_GET_COUNTER(&htim4),__HAL_TIM_GET_COUNTER(&htim3),m_ui16DebugTemp);
			DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
		}
		return;
	}

	if(ucRXChar == 'z')
	{	// set Tim9 PWM duty (--) CHANNEL1
		if(m_ui16DebugTemp > 0)
		{
			m_ui16DebugTemp --;
			__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,m_ui16DebugTemp);
			sprintf((char *)m_ucDebugBuffer,"Left=%ld Right=%ld, PWMR=%d\n\r",__HAL_TIM_GET_COUNTER(&htim4),__HAL_TIM_GET_COUNTER(&htim3),m_ui16DebugTemp);
			DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
		}
		return;
	}

	if(ucRXChar == 's')
	{
		HAL_GPIO_WritePin(RIN1_GPIO_Port, RIN1_Pin, GPIO_PIN_SET);
		return;
	}
	if(ucRXChar == 'x')
	{
		HAL_GPIO_WritePin(RIN1_GPIO_Port, RIN1_Pin, GPIO_PIN_RESET);
		return;
	}
	if(ucRXChar == 'd')
	{
		HAL_GPIO_WritePin(RIN2_GPIO_Port, RIN2_Pin, GPIO_PIN_SET);
		return;
	}
	if(ucRXChar == 'c')
	{
		HAL_GPIO_WritePin(RIN2_GPIO_Port, RIN2_Pin, GPIO_PIN_RESET);
		return;
	}


	if(ucRXChar == 'j')
	{	// set Tim9 PWM duty (++) CHANNEL2
		if(m_ui16DebugTemp2 < 99)
		{
			m_ui16DebugTemp2 ++;
			__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2,m_ui16DebugTemp2);
			sprintf((char *)m_ucDebugBuffer,"Left=%ld Right=%ld, PWML=%d\n\r",__HAL_TIM_GET_COUNTER(&htim4),__HAL_TIM_GET_COUNTER(&htim3),m_ui16DebugTemp2);
			DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
		}
		return;
	}

	if(ucRXChar == 'm')
	{
		if(m_ui16DebugTemp2 > 0)
		{
			m_ui16DebugTemp2 --;
			__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2,m_ui16DebugTemp2);
			sprintf((char *)m_ucDebugBuffer,"Left=%ld Right=%ld, PWML=%d\n\r",__HAL_TIM_GET_COUNTER(&htim4),__HAL_TIM_GET_COUNTER(&htim3),m_ui16DebugTemp2);
			DebugSendData(m_ucDebugBuffer,strlen((char *)m_ucDebugBuffer));
		}
		return;
	}
	if(ucRXChar == 'h')
	{
		HAL_GPIO_WritePin(LIN1_GPIO_Port, LIN1_Pin, GPIO_PIN_SET);
		return;
	}
	if(ucRXChar == 'n')
	{
		HAL_GPIO_WritePin(LIN1_GPIO_Port, LIN1_Pin, GPIO_PIN_RESET);
		return;
	}
	if(ucRXChar == 'g')
	{
		HAL_GPIO_WritePin(LIN2_GPIO_Port, LIN2_Pin, GPIO_PIN_SET);
		return;
	}
	if(ucRXChar == 'b')
	{
		HAL_GPIO_WritePin(LIN2_GPIO_Port, LIN2_Pin, GPIO_PIN_RESET);
		return;
	}

	if(ucRXChar == 'l')
	{	// clear left encoder accumulated counts
		m_EncoderLeft.ui32TotalBackwardPulses = 0;
		m_EncoderLeft.ui32TotalForwardPulses = 0;
		return;
	}

	if(ucRXChar == 'r')
	{	// clear right encoder accumulated counts
		m_EncoderRight.ui32TotalBackwardPulses = 0;
		m_EncoderRight.ui32TotalForwardPulses = 0;
		return;
	}

	*/




	return;
}
#endif
//----------------------------------------------------------------------------------
//==================================================================================================================== SYSTEM_END


// STATIC (INTERN) implementation
//==================================================================================================================== STATIC_START
static void EncoderTimer1KHZ(void)		// used to handle the encoder related real time
{
	if(m_ui16TimerDivider == 10)
	{	// @ 10 read the Right Encoder (interrupt time = 800ns)
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);	// debug on PC13 HIGH

		m_EncoderRight.uiNewEncoderRead = __HAL_TIM_GET_COUNTER(&htim3);	// read the new counter value

		// Right Wheel turning FORWARD  => decrements (1431 pulses / complete rotation
		// Right Wheel turning BACKWARD => increments (1431 pulses / complete rotation
		if( (m_EncoderRight.uiNewEncoderRead > m_EncoderRight.uiOldEncoderRead) )
		{	// 2 situations possible :
			// turning backward if (uiNewEncoderRead - uiOldEncoderRead) < 32000. Ex: (old)61234 --> (new)63456 (increment)
			if( (m_EncoderRight.uiNewEncoderRead - m_EncoderRight.uiOldEncoderRead) < 32000 )
			{	// turning backward
				m_EncoderRight.uiPulseDiference = (m_EncoderRight.uiNewEncoderRead - m_EncoderRight.uiOldEncoderRead);
				m_EncoderRight.ui32TotalBackwardPulses += (uint32_t)m_EncoderRight.uiPulseDiference;
				m_EncoderRight.TurningDirection = WHEEL_TURNING_BACKWARD;
			}
			// turning forward if  (uiNewEncoderRead - uiOldEncoderRead) > 32000. Ex: (old)01000 --> (new)65300 (apparent increment in reality is decrement)
			else
			{	// turning forward
				m_EncoderRight.uiPulseDiference = ((65535 - m_EncoderRight.uiNewEncoderRead) + m_EncoderRight.uiOldEncoderRead);
				m_EncoderRight.ui32TotalForwardPulses += (uint32_t)m_EncoderRight.uiPulseDiference;
				m_EncoderRight.TurningDirection = WHEEL_TURNING_FORWARD;
			}
		}
		else
		{	// 2 situations possible :
			// turning backward if (uiOldEncoderRead - uiNewEncoderRead) < 32000. Ex: (old)63456 --> (new)61234 (decrement)
			if( (m_EncoderRight.uiOldEncoderRead - m_EncoderRight.uiNewEncoderRead) < 32000 )
			{	// turning backward
				m_EncoderRight.uiPulseDiference = (m_EncoderRight.uiOldEncoderRead - m_EncoderRight.uiNewEncoderRead);
				m_EncoderRight.ui32TotalForwardPulses += (uint32_t)m_EncoderRight.uiPulseDiference;
				m_EncoderRight.TurningDirection = WHEEL_TURNING_FORWARD;
			}
			// turning forward if  (uiOldEncoderRead - uiNewEncoderRead) > 32000. Ex: (old)65300 --> (new)01000 (apparent decrement in reality is increment)
			else
			{	// turning forward
				m_EncoderRight.uiPulseDiference = ((65535 - m_EncoderRight.uiOldEncoderRead) + m_EncoderRight.uiNewEncoderRead);
				m_EncoderRight.ui32TotalBackwardPulses += (uint32_t)m_EncoderRight.uiPulseDiference;
				m_EncoderRight.TurningDirection = WHEEL_TURNING_BACKWARD;
			}
		}
		m_EncoderRight.uiOldEncoderRead = m_EncoderRight.uiNewEncoderRead;
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);	// debug on PC13 LOW
		return;
	}

	if(m_ui16TimerDivider == 11)
	{	// @ 11 read the Left Encoder (interrupt time = 800ns)

		m_EncoderLeft.uiNewEncoderRead = __HAL_TIM_GET_COUNTER(&htim4);	// read the new counter value

		/* Left Wheel turning BACKWARD => decrements (1431 pulses / complete rotation */
		/* Left Wheel turning FORWARD  => increments (1431 pulses / complete rotation */
		if( (m_EncoderLeft.uiNewEncoderRead > m_EncoderLeft.uiOldEncoderRead) )
		{	// 2 situations possible :
			// turning backward if (uiNewEncoderRead - uiOldEncoderRead) < 32000. Ex: (old)61234 --> (new)63456 (increment)
			if( (m_EncoderLeft.uiNewEncoderRead - m_EncoderLeft.uiOldEncoderRead) < 32000 )
			{	// turning forward
				m_EncoderLeft.uiPulseDiference = (m_EncoderLeft.uiNewEncoderRead - m_EncoderLeft.uiOldEncoderRead);
				m_EncoderLeft.ui32TotalForwardPulses += (uint32_t)m_EncoderLeft.uiPulseDiference;
				m_EncoderLeft.TurningDirection = WHEEL_TURNING_FORWARD;
			}
			// turning forward if  (uiNewEncoderRead - uiOldEncoderRead) > 32000. Ex: (old)01000 --> (new)65300 (apparent increment in reality is decrement)
			else
			{	// turning backward
				m_EncoderLeft.uiPulseDiference = ((65535 - m_EncoderLeft.uiNewEncoderRead) + m_EncoderLeft.uiOldEncoderRead);
				m_EncoderLeft.ui32TotalBackwardPulses += (uint32_t)m_EncoderLeft.uiPulseDiference;
				m_EncoderLeft.TurningDirection = WHEEL_TURNING_BACKWARD;
			}
		}
		else
		{	// 2 situations possible :
			// turning backward if (uiOldEncoderRead - uiNewEncoderRead) < 32000. Ex: (old)63456 --> (new)61234 (decrement)
			if( (m_EncoderLeft.uiOldEncoderRead- m_EncoderLeft.uiNewEncoderRead ) < 32000 )
			{	// turning forward
				m_EncoderLeft.uiPulseDiference = (m_EncoderLeft.uiOldEncoderRead - m_EncoderLeft.uiNewEncoderRead);
				m_EncoderLeft.ui32TotalBackwardPulses += (uint32_t)m_EncoderLeft.uiPulseDiference;
				m_EncoderLeft.TurningDirection = WHEEL_TURNING_BACKWARD;
			}
			// turning forward if  (uiNewEncoderRead - uiOldEncoderRead) > 32000. Ex: (old)65300 --> (new)01000 (apparent decrement in reality is increment)
			else
			{	// turning backward
				m_EncoderLeft.uiPulseDiference = ((65535 - m_EncoderLeft.uiOldEncoderRead) + m_EncoderLeft.uiNewEncoderRead);
				m_EncoderLeft.ui32TotalForwardPulses += (uint32_t)m_EncoderLeft.uiPulseDiference;
				m_EncoderLeft.TurningDirection = WHEEL_TURNING_FORWARD;
			}
		}
		m_EncoderLeft.uiOldEncoderRead = m_EncoderLeft.uiNewEncoderRead;
		return;
	}

	if(m_ui16TimerDivider == 20)
	{	// counting the speed for the Right Encoder (interrupt time max = 650ns)
		// - the number of pulses between 2 consecutive updates is stored in 			-> uiPulseDiference
		// - the time elapsed between 2 consecutive updates is given by 				-> TIMER_ENCODER_DIVIDER_MAX (in ms)
		// - the length of the wheel is stored in 										-> WHEEL_LENGTH_MM (mm)
		// - the number of pulses for a full wheel turn is kept in						-> ENCODER_PULSES_PER_ROTATION_RIGHT_WHEEL
		//-----------------------------------
		// - v[mm/s] = (1000 * WHEEL_LENGTH_MM * uiPulseDiference) / (ENCODER_PULSES_PER_ROTATION_RIGHT_WHEEL * TIMER_ENCODER_DIVIDER_MAX);
		// ----------------------------------

		uint32_t ui32Temp;

		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);	//  debug on PC13 HIGH

		if(m_EncoderRight.uiPulseDiference == 0)
		{
			m_EncoderRight.uiSpeedMMS = 0;	// speed in mm/sec
			m_EncoderRight.TurningDirection = WHEEL_NOT_TURNING;
			m_EncoderRight.UpdateStatus = NEW_ENCODER_VALUE;
			//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			return;
		}


		ui32Temp  = (uint32_t)(1000 * WHEEL_LENGTH_MM);
		ui32Temp *= (uint32_t)m_EncoderRight.uiPulseDiference;
		ui32Temp /= (uint32_t)TIMER_DIVIDER_MAX;
		ui32Temp /= (uint32_t)ENCODER_PULSES_PER_ROTATION_RIGHT_WHEEL;

		m_EncoderRight.uiSpeedMMS = (uint16_t)ui32Temp;
		m_EncoderRight.UpdateStatus = NEW_ENCODER_VALUE;		// new value available and can be read from the main loop

		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);  //	debug on PC13 LOW
		return;
	}

	if(m_ui16TimerDivider == 30)
	{	// counting the speed for the Left Encoder (interrupt time max = 650ns)
		// - the number of pulses between 2 consecutive updates is stored in 			-> uiPulseDiference
		// - the time elapsed between 2 consecutive updates is given by 				-> TIMER_ENCODER_DIVIDER_MAX (in ms)
		// - the length of the wheel is stored in 										-> WHEEL_LENGTH_MM (mm)
		// - the number of pulses for a full wheel turn is kept in						-> ENCODER_PULSES_PER_ROTATION_LEFT_WHEEL
		//-----------------------------------
		// - v[mm/s] = (1000 * WHEEL_LENGTH_MM * uiPulseDiference) / (ENCODER_PULSES_PER_ROTATION_LEFT_WHEEL * TIMER_ENCODER_DIVIDER_MAX);
		// ----------------------------------

		uint32_t ui32Temp;

		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);	//  debug on PC13 HIGH

		if(m_EncoderLeft.uiPulseDiference == 0)
		{
			m_EncoderLeft.uiSpeedMMS = 0;	// speed in mm/sec
			m_EncoderLeft.TurningDirection = WHEEL_NOT_TURNING;
			m_EncoderLeft.UpdateStatus = NEW_ENCODER_VALUE;
			//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			return;
		}


		ui32Temp  = (uint32_t)(1000 * WHEEL_LENGTH_MM);
		ui32Temp *= (uint32_t)m_EncoderLeft.uiPulseDiference;
		ui32Temp /= (uint32_t)TIMER_DIVIDER_MAX;
		ui32Temp /= (uint32_t)ENCODER_PULSES_PER_ROTATION_RIGHT_WHEEL;

		m_EncoderLeft.uiSpeedMMS = (uint16_t)ui32Temp;
		m_EncoderLeft.UpdateStatus = NEW_ENCODER_VALUE;		// new value available and can be read from the main loop

		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);  //	debug on PC13 LOW
		return;
	}

	return;
}
//----------------------------------------------------------------------------------
static void MotorTimer1KHZ(void)	// used to handle the motors related real time
{
	// RAMP the PWM Speed UP or DOWN
	if( (m_ui16TimerDivider == 15) )		// Ramp the PWM for the RIGHT Motor
	{		// max 2.2uS, typically 400nS
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);	//  debug on PC13 HIGH

		if(m_MotorRight.PWMSpeedStatus != PWM_SPEED_TRACKING)
		{
			//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);  //	debug on PC13 LOW
			return;		// only important during the speed tracking process
		}

		// PWM Speed tracking in progress
		MotorPWMSpeedTracking(&m_MotorRight);
		__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,(uint16_t)m_MotorRight.ucPWMRampCounter);		// set the new PWM value and getting closer to the PWM Target (ucPWMPercentageTarget)
		if(m_MotorRight.ucPWMRampCounter == m_MotorRight.ucPWMPercentageTarget)		m_MotorRight.PWMSpeedStatus = PWM_SPEED_REACHED;		// also the Target PWM Speed was reached
		if(m_MotorRight.ucPWMRampCounter == 0)
		{	// it is also the case when the motor must stop
			// check if it is not a direction change on progress
			if( (m_MotorRight.DirectionChangeWithoutStop == ROTATION_CHANGE_DETECTED) )
			{	// turning direction detected
				m_MotorRight.DirectionChangeWithoutStop = NO_ROTATION_CHANGE_DETECTED;
				m_MotorRight.ucPWMPercentageTarget = m_MotorRight.ucPWMPercentageTarget2;
				m_MotorRight.ucPWMPercentageTarget2 = 0;
				if( (m_MotorRight.TurningDirection == WHEEL_TURNING_BACKWARD) )
				{
					m_MotorRight.TurningDirection = WHEEL_TURNING_FORWARD;
					HAL_GPIO_WritePin(RIN1_GPIO_Port, RIN1_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(RIN1_GPIO_Port, RIN2_Pin, GPIO_PIN_SET);
				}
				else
				{
					m_MotorRight.TurningDirection = WHEEL_TURNING_BACKWARD;
					HAL_GPIO_WritePin(RIN1_GPIO_Port, RIN1_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(RIN1_GPIO_Port, RIN2_Pin, GPIO_PIN_RESET);
				}
				m_MotorRight.PWMSpeedStatus = PWM_SPEED_TRACKING;	// start a new tracking
			}
			else
			{	// just normal STOP without turning direction
				m_MotorRight.TurningDirection = WHEEL_NOT_TURNING;
				HAL_GPIO_WritePin(RIN1_GPIO_Port, RIN1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(RIN1_GPIO_Port, RIN2_Pin, GPIO_PIN_RESET);
			}
		}
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);  //	debug on PC13 LOW
		return;
	}


	if( (m_ui16TimerDivider == 16) )		// Ramp the PWM for the LEFT  Motor
	{	// max 2.2uS, typically 400nS
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);	//  debug on PC13 HIGH
		if(m_MotorLeft.PWMSpeedStatus != PWM_SPEED_TRACKING)
		{
			//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);  //	debug on PC13 LOW
			return;		// only important during the speed tracking process
		}

		// PWM Speed tracking in progress
		MotorPWMSpeedTracking(&m_MotorLeft);
		__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2,(uint16_t)m_MotorLeft.ucPWMRampCounter);		// set the new PWM value and getting closer to the PWM Target (ucPWMPercentageTarget)
		if(m_MotorLeft.ucPWMRampCounter == m_MotorLeft.ucPWMPercentageTarget)		m_MotorLeft.PWMSpeedStatus = PWM_SPEED_REACHED;		// also the Target PWM Speed was reached
		if(m_MotorLeft.ucPWMRampCounter == 0)
		{	// it is also the case when the motor must stop
			// check if it is not a direction change on progress
			if( (m_MotorLeft.DirectionChangeWithoutStop == ROTATION_CHANGE_DETECTED) )
			{	// turning direction detected
				m_MotorLeft.DirectionChangeWithoutStop = NO_ROTATION_CHANGE_DETECTED;
				m_MotorLeft.ucPWMPercentageTarget = m_MotorLeft.ucPWMPercentageTarget2;
				m_MotorLeft.ucPWMPercentageTarget2 = 0;
				if( (m_MotorLeft.TurningDirection == WHEEL_TURNING_BACKWARD) )
				{
					m_MotorLeft.TurningDirection = WHEEL_TURNING_FORWARD;
					HAL_GPIO_WritePin(LIN1_GPIO_Port, LIN1_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(LIN1_GPIO_Port, LIN2_Pin, GPIO_PIN_RESET);
				}
				else
				{
					m_MotorLeft.TurningDirection = WHEEL_TURNING_BACKWARD;
					HAL_GPIO_WritePin(LIN1_GPIO_Port, LIN1_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(LIN1_GPIO_Port, LIN2_Pin, GPIO_PIN_SET);
				}
				m_MotorLeft.PWMSpeedStatus = PWM_SPEED_TRACKING;	// start a new tracking
			}
			else
			{	// just normal STOP without turning direction
				m_MotorLeft.TurningDirection = WHEEL_NOT_TURNING;
				HAL_GPIO_WritePin(LIN1_GPIO_Port, LIN1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LIN1_GPIO_Port, LIN2_Pin, GPIO_PIN_RESET);
			}
		}
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);  //	debug on PC13 LOW
		return;
	}


	if( (m_ui16TimerDivider == 25) )		// Test the RIGHT Motor Stall condition
	{	// max 700nS, typically 500nS
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);	//  debug on PC13 HIGH
		if( (m_MotorRight.TurningDirection == WHEEL_NOT_TURNING) || (m_MotorRight.PWMSpeedStatus != PWM_SPEED_REACHED) )
		{
			//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);  //	debug on PC13 LOW
			return;		// either the wheel is stopped or its speed is not yet stable
		}
		// wheel is not stopped and its PWM speed is stable

		if( (m_EncoderRight.uiSpeedMMS != 0) )	// the wheel speed is not zero
		{	// wheel is not stalled
			m_MotorRight.ucStallCounter = m_MotorRight.ucStallMaxCountValue;
			m_MotorRight.StallCondition = WHEEL_NOT_STALLED;
			m_MotorRight.StallConditionCallback = WHEEL_NOT_STALLED;		// in case it was not cleared in the main loop
			//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);  //	debug on PC13 LOW
			return;
		}

		// the wheel speed is zero
		if( (m_MotorRight.ucStallCounter > 0) )
		{
			m_MotorRight.ucStallCounter --;
			if( (m_MotorRight.ucStallCounter == 0) )
			{	// wheel is stalled
				m_MotorRight.StallCondition = WHEEL_STALLED;
				m_MotorRight.StallConditionCallback = WHEEL_STALLED;	// it will be cleared in the main loop once the callback is called
				//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);  //	debug on PC13 LOW
				return;
			}
		}
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);  //	debug on PC13 LOW
		return;
	}



	if( (m_ui16TimerDivider == 35) )		// Test the LEFT  Motor Stall condition
	{	// max 700nS, typically 500nS
		if( (m_MotorLeft.TurningDirection == WHEEL_NOT_TURNING) || (m_MotorLeft.PWMSpeedStatus != PWM_SPEED_REACHED) )	return;		// either the wheel is stopped or its speed is not yet stable
		// wheel is not stopped and its PWM speed is stable

		if( (m_EncoderLeft.uiSpeedMMS != 0) ) // the wheel speed is not zero
		{	// wheel is not stalled
			m_MotorLeft.ucStallCounter = m_MotorLeft.ucStallMaxCountValue;
			m_MotorLeft.StallCondition = WHEEL_NOT_STALLED;
			m_MotorLeft.StallConditionCallback = WHEEL_NOT_STALLED;		// in case it was not cleared in the main loop
			return;
		}

		// the wheel speed is zero
		if( (m_MotorLeft.ucStallCounter > 0) )
		{
			m_MotorLeft.ucStallCounter --;
			if( (m_MotorLeft.ucStallCounter == 0) )
			{	// wheel is stalled
				m_MotorLeft.StallCondition = WHEEL_STALLED;
				m_MotorLeft.StallConditionCallback = WHEEL_STALLED;	// it will be cleared in the main loop once the callback is called
				return;
			}
		}
		return;
	}


	if( (m_ui16TimerDivider == 37) )		// RIGHT and LEFT Wheel Timeout check and trigger Callback function at timeout
	{	// max 600nS, typically 450nS
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);	//  debug on PC13 HIGH
		if(m_MotorRight.ucTimeToRunHMS > 1) 	m_MotorRight.ucTimeToRunHMS --;
		if(m_MotorLeft.ucTimeToRunHMS > 1) 		m_MotorLeft.ucTimeToRunHMS --;

		if( (m_MotorRight.ucTimeToRunHMS == 1) && (m_MotorLeft.ucTimeToRunHMS == 1) )
		{
			// indicate the callback situation to the main loop function
			m_MotorRight.ucTimeToRunHMS = 255;
			m_MotorLeft.ucTimeToRunHMS  = 255;
			//WheelRobotStop();		// stop the motors
			m_MotorRight.DirectionChangeWithoutStop = NO_ROTATION_CHANGE_DETECTED;
			m_MotorRight.ucPWMPercentageTarget2 = 0;
			m_MotorRight.ucPWMPercentageTarget = 0;
			m_MotorRight.PWMSpeedStatus = PWM_SPEED_TRACKING;	// start a new tracking
			m_MotorLeft.DirectionChangeWithoutStop = NO_ROTATION_CHANGE_DETECTED;
			m_MotorLeft.ucPWMPercentageTarget2 = 0;
			m_MotorLeft.ucPWMPercentageTarget = 0;
			m_MotorLeft.PWMSpeedStatus = PWM_SPEED_TRACKING;	// start a new tracking
		}
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);  //	debug on PC13 LOW
		return;
	}


	if( (m_ui16TimerDivider == 39) )		// RIGHT and LEFT Wheel Distance To Go
	{	// max 1.8uS, typically 430nS
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);	//  debug on PC13 HIGH
		// Right Wheel
		if( (m_MotorRight.ui32DistanceToGoPulses != 0) && (m_MotorRight.ui32DistanceToGoPulses != 0xFFFFFFFF) )
		{	// there is a distance target
			if(m_MotorRight.TurningDirection == WHEEL_TURNING_FORWARD)
			{	// wheel turning forward
				if(m_EncoderRight.ui32TotalForwardPulses >= m_MotorRight.ui32DistanceToGoPulses )
				{	// Distance Target was reached on the RIGHT side
					HAL_GPIO_WritePin(RIN1_GPIO_Port, RIN1_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(RIN1_GPIO_Port, RIN2_Pin, GPIO_PIN_RESET);
					m_MotorRight.ui32DistanceToGoPulses = 0xFFFFFFFF;	// marker for all done and will be handled in the main loop
				}
				else
				{	// the destination is not yet reached
					if( ((m_MotorRight.ui32DistanceToGoPulses - m_EncoderRight.ui32TotalForwardPulses) < 700) && (m_MotorRight.ucPWMPercentageTarget > 15) )
					{	// slow down on the last part
						m_MotorRight.ucPWMPercentageTarget = 15;
						m_MotorRight.PWMSpeedStatus = PWM_SPEED_TRACKING;	// start a new tracking
					}
				}
			}
			else
			{	// wheel turning backward
				if(m_EncoderRight.ui32TotalBackwardPulses >= m_MotorRight.ui32DistanceToGoPulses )
				{	// Distance Target was reached on the RIGHT side
					HAL_GPIO_WritePin(RIN1_GPIO_Port, RIN1_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(RIN1_GPIO_Port, RIN2_Pin, GPIO_PIN_RESET);
					m_MotorRight.ui32DistanceToGoPulses = 0xFFFFFFFF;	// marker for all done and will be handled in the main loop
				}
				else
				{	// the destination is not yet reached
					if( ((m_MotorRight.ui32DistanceToGoPulses - m_EncoderRight.ui32TotalBackwardPulses) < 700) && (m_MotorRight.ucPWMPercentageTarget > 15) )
					{	// slow down on the last part
						m_MotorRight.ucPWMPercentageTarget = 15;
						m_MotorRight.PWMSpeedStatus = PWM_SPEED_TRACKING;	// start a new tracking
					}
				}
			}
		}

		// Left Wheel
		if( (m_MotorLeft.ui32DistanceToGoPulses != 0) && (m_MotorLeft.ui32DistanceToGoPulses != 0xFFFFFFFF) )
		{	// there is a distance target
			if(m_MotorLeft.TurningDirection == WHEEL_TURNING_FORWARD)
			{	// wheel turning forward
				if(m_EncoderLeft.ui32TotalForwardPulses >= m_MotorLeft.ui32DistanceToGoPulses)
				{	// Distance Target was reached on the RIGHT side
					HAL_GPIO_WritePin(LIN1_GPIO_Port, LIN1_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(LIN1_GPIO_Port, LIN2_Pin, GPIO_PIN_RESET);
					m_MotorLeft.ui32DistanceToGoPulses = 0xFFFFFFFF;	// marker for all done and will be handled in the main loop
				}
				else
				{	// the destination is not yet reached
					if( ((m_MotorLeft.ui32DistanceToGoPulses - m_EncoderLeft.ui32TotalForwardPulses) < 700) && (m_MotorLeft.ucPWMPercentageTarget > 15) )
					{	// slow down on the last part
						m_MotorLeft.ucPWMPercentageTarget = 15;
						m_MotorLeft.PWMSpeedStatus = PWM_SPEED_TRACKING;	// start a new tracking
					}
				}
			}
			else
			{	// wheel turning backward
				if(m_EncoderLeft.ui32TotalBackwardPulses >= m_MotorLeft.ui32DistanceToGoPulses)
				{	// Distance Target was reached on the LEFT side
					HAL_GPIO_WritePin(LIN1_GPIO_Port, LIN1_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(LIN1_GPIO_Port, LIN2_Pin, GPIO_PIN_RESET);
					m_MotorLeft.ui32DistanceToGoPulses = 0xFFFFFFFF;	// marker for all done and will be handled in the main loop
				}
				else
				{	// the destination is not yet reached
					if( ((m_MotorLeft.ui32DistanceToGoPulses - m_EncoderLeft.ui32TotalBackwardPulses) < 700) && (m_MotorLeft.ucPWMPercentageTarget > 15) )
					{	// slow down on the last part
						m_MotorLeft.ucPWMPercentageTarget = 15;
						m_MotorLeft.PWMSpeedStatus = PWM_SPEED_TRACKING;	// start a new tracking
					}
				}
			}
		}
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);  //	debug on PC13 LOW
		return;
	}

	if( (m_ui16TimerDivider == 41) )	//  @ 41  =>  MMsSpeed to PWM Feedback loop
	{	// max 1.3uS, typically 500nS
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);	//  debug on PC13 HIGH
		// RIGHT Wheel speed control
		if(m_MotorRight.uiRequestedSpeedMMS != 0)
		{
			if(m_MotorRight.PWMSpeedStatus == PWM_SPEED_REACHED)
			{
				if(m_EncoderRight.uiSpeedMMS != 0)
				{
					if( (m_MotorRight.uiRequestedSpeedMMS > (m_EncoderRight.uiSpeedMMS + 8)) )
					{
						if(m_MotorRight.ucPWMPercentageTarget + 1 < 100)
						{
							m_MotorRight.ucPWMPercentageTarget += 1;
							m_MotorRight.PWMSpeedStatus = PWM_SPEED_TRACKING;	// track a new speed
						}
					}
					else
					{
						if( (m_EncoderRight.uiSpeedMMS > 8) )
						{
							if( (m_MotorRight.uiRequestedSpeedMMS < (m_EncoderRight.uiSpeedMMS - 8)) )
							{
								if(m_MotorRight.ucPWMPercentageTarget > 16)	// 15 is the minimum PWM %
								{
									m_MotorRight.ucPWMPercentageTarget -= 1;
									m_MotorRight.PWMSpeedStatus = PWM_SPEED_TRACKING;	// track a new speed
								}
							}
						}
					}
				}
			}
		}


		// LEFT  Wheel speed control
		if(m_MotorLeft.uiRequestedSpeedMMS != 0)
		{
			if(m_MotorLeft.PWMSpeedStatus == PWM_SPEED_REACHED)
			{	// only make new correction if the last one is settled
				if(m_EncoderLeft.uiSpeedMMS != 0)
				{	// only make correction if the wheel is not stopped
					if( (m_MotorLeft.uiRequestedSpeedMMS > (m_EncoderLeft.uiSpeedMMS + 8)) )
					{	// check if one need to increase the speed
						if(m_MotorLeft.ucPWMPercentageTarget + 1 < 100)
						{	// make sure that the PWM Speed is not already at 100% and it will not overtake 100% after correction
							m_MotorLeft.ucPWMPercentageTarget += 1;
							m_MotorLeft.PWMSpeedStatus = PWM_SPEED_TRACKING;	// track a new speed
						}
					}
					else
					{	// check if one need to decrease the speed
						if( (m_EncoderLeft.uiSpeedMMS > 8) )
						{	// check if one can decrease the speed and after decreasing the value will still be in the range
							if( (m_MotorLeft.uiRequestedSpeedMMS < (m_EncoderLeft.uiSpeedMMS - 8)) )
							{	// check if it is worth adding another correction
								if(m_MotorLeft.ucPWMPercentageTarget > 16)
								{	// check if the PWM setting is large enough for the correction
									m_MotorLeft.ucPWMPercentageTarget -= 1;
									m_MotorLeft.PWMSpeedStatus = PWM_SPEED_TRACKING;	// track a new speed
								}
							}
						}
					}
				}
			}
		}
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);  //	debug on PC13 LOW
		return;
	}



	return;
}
//----------------------------------------------------------------------------------
static void MotorPWMSpeedTracking(WheelMotorTypeDef * MotorData)	// used inside the MotorTimer1KHZ function, during the PWM Speed Tracking process in order to estimate the new PWM setting while
																	// getting closer to the PWM Speed Target
{
	uint8_t ucPWMStep;

	if(MotorData->ucPWMRampCounter == MotorData->ucPWMPercentageTarget)
	{	// the target speed was reached
		//MotorData->PWMSpeedStatus = PWM_SPEED_REACHED;	// it is done in the calling function. in this way in case of motor stop one can also set the corresponding RIN1,RIN2 or LIN1,LIN2 to ZERO
		return;
	}

	// PWM Speed target was not yet reached
	if( (MotorData->ucPWMPercentageTarget > MotorData->ucPWMRampCounter) )
	{	// the target is higher than the current PWM value => the PWM setting on the motor must be increased
		ucPWMStep = GetPWMSpeedStep( (MotorData->ucPWMPercentageTarget - MotorData->ucPWMRampCounter) );
		MotorData->ucPWMRampCounter += ucPWMStep;
		return;		// in the timer function (the calling function) now must set the PWM of the corresponding channel of the Timer9 with the new ucPWMRampCounter value
	}
	else
	{	// the setting is not incremental but decremental
		ucPWMStep = GetPWMSpeedStep( (MotorData->ucPWMRampCounter - MotorData->ucPWMPercentageTarget) );
		MotorData->ucPWMRampCounter -= ucPWMStep;
		return;		// in the timer function (the calling function) now must set the PWM of the corresponding channel of the Timer9 with the new ucPWMRampCounter value
	}
	return;	// just dummy (it will never happen)
}
//----------------------------------------------------------------------------------
static uint8_t GetPWMSpeedStep(uint8_t ucActualSpeedDifference)
{
	if(ucActualSpeedDifference >= 15)											return 15;		// change the PWMRampCounter value with +/- 15 representing +/- 15%
	if( (ucActualSpeedDifference < 15) && (ucActualSpeedDifference >= 10) )		return 10;		// change the PWMRampCounter value with +/- 10 representing +/- 10%
	if( (ucActualSpeedDifference < 10) && (ucActualSpeedDifference >= 5) )		return 5;		// change the PWMRampCounter value with +/- 5 representing +/- 5%
	return 1;		// change the PWMRampCounter value with +/- 1 representing +/- 1%
}
//----------------------------------------------------------------------------------
static void MotorMainLoop(void)	// used in the system main loop to handle thread related activities with the motors
{
	// -------- stall check and callback activation --------------- START
	// RIGHT
	if(m_MotorRight.StallConditionCallback == WHEEL_STALLED)
	{	// Right motor is stalled
		// acknowledge the stall notification and re-arm the stall callback trigger.
		m_MotorRight.StallConditionCallback = WHEEL_STALLED_UNINITIALIZED;	// ready for an eventual new event
		WheelStallRightCallback();
	}
	// LEFT
	if(m_MotorLeft.StallConditionCallback == WHEEL_STALLED)
	{	// Left motor is stalled
		// acknowledge the stall notification and re-arm the stall callback trigger.
		m_MotorLeft.StallConditionCallback = WHEEL_STALLED_UNINITIALIZED;	// ready for an eventual new event
		WheelStallLeftCallback();
	}
	// -------- stall check and callback activation --------------- END

	// --------------- Motor Status Update ------------------------ START
	// --------------- Motor Status Update ------------------------ END

	// ------------ Timeout Command Callback  --------------------- START
	if( (m_MotorRight.ucTimeToRunHMS == 255) && (m_MotorLeft.ucTimeToRunHMS == 255) )
	{
		//m_MotorRight.ucTimeToRunHMS = 0;
		//m_MotorLeft.ucTimeToRunHMS = 0;
		WheelRobotStop();
		WheelRobotCommandEndCallback(TIMEOUT_COMMAND_CALLBACK);		// call the command end callback function
	}
	// ------------ Timeout Command Callback  --------------------- END

	// ------------- Distance to Go Callback  --------------------- START
	if( (m_MotorRight.ui32DistanceToGoPulses == 0xFFFFFFFF) && (m_MotorLeft.ui32DistanceToGoPulses == 0xFFFFFFFF) )
	{
		WheelRobotStop();
		WheelRobotCommandEndCallback(DISTANCE_REACHED_CALLBACK);		// call the command end callback function
	}
	// ------------- Distance to Go Callback  --------------------- END





	return;
}
//----------------------------------------------------------------------------------
static uint8_t MMSSpeedToPWM(uint16_t uiMMSSpeed) // used to estimate the PWMPercentage as function of the MMSSpeed
{
	// mm/s -> PWM estimate
	// if mm/s = 50  ....... PWM ~~ 15
	// if mm/s = 500 ....... PWM ~~ 100
	// PWMTarget = ((85*(SpeedMMS - 50)/450) + 15)
	double dPWMTarget;
	uint8_t ucPWMPercentage;


	dPWMTarget  = (double)(uiMMSSpeed - 50);
	//dPWMTarget *= (double)85;
	dPWMTarget *= (double)65;
	dPWMTarget /= (double)450;
	dPWMTarget += (double)15;
	if(dPWMTarget < 15) 	dPWMTarget = 15;
	if(dPWMTarget > 100)	dPWMTarget = 100;
	ucPWMPercentage = (uint8_t) dPWMTarget;

	return ucPWMPercentage;
}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== STATIC_END
