#ifndef _WHEEL_
#define _WHEEL_


// Settings
/*
 *
 * defined in main.h and related to this module:
 * 	#define RIN1_Pin GPIO_PIN_10
	#define RIN1_GPIO_Port GPIOD
	#define RIN2_Pin GPIO_PIN_11
	#define RIN2_GPIO_Port GPIOD
	#define LC1_Pin GPIO_PIN_12
	#define LC1_GPIO_Port GPIOD
	#define LC2_Pin GPIO_PIN_13
	#define LC2_GPIO_Port GPIOD
	#define LIN1_Pin GPIO_PIN_14
	#define LIN1_GPIO_Port GPIOD
	#define LIN2_Pin GPIO_PIN_15
	#define LIN2_GPIO_Port GPIOD
	#define RC1_Pin GPIO_PIN_6
	#define RC1_GPIO_Port GPIOC
	#define RC2_Pin GPIO_PIN_7
	#define RC2_GPIO_Port GPIOC
 */


//==================================================================================================================== @_SHORT START
/*
 *	--- to command movement @ the unit level ----
 *	- WheelRobotStop()				-> stops the robot
 *	- WheelRobotGoPWM()				-> use just direction and PWM setting for speed control
 *	- WheelRobotGoSpeed()			-> same as above but specify the speed in mm/s instead of PWM percentage of the driver
 *	- WheelRobotGoPWMTmo()			-> add also timeout to stop moving after a certain time -> at timeout a callback function can be used to generate event @ application
 *	- WheelRobotGoSpeedTmo()		-> same as above but specify the speed in mm/s instead of PWM percentage of the driver
 *	- WheelRobotGoPWMTmoDist()		-> specify the distance to run. but also keep the timeout. -> at timeout or distance reached a callback can alert the application
 *	- WheelRobotGoSpeedTmoDist()	-> same as above but specify the speed in mm/s instead of PWM percentage of the driver
 *	- WheelRobotRotate()			-> rotate the robot with the specified angle
 *	- WheelRobotRotateTmo()			-> rotate the robot with the specified angle using timeout
 *
 *	--- to command each wheel separate -----
 *	- WheelRightGo()				-> command the Right Wheel using PWM percentage
 *	- WheelRightStop()				-> stop the Right Wheel
 *	- WheelLeftGo()					-> command the Left Wheel using PWM percentage
 *	- WheelLeftStop()				-> stop the Left Wheel
 *	- WheelLefRighttGo()			-> command both wheels with a PWM percentage but change the speed ratio between the right and left wheel
 *
 *	--- callback function to use in Application ----
 *	- WheelRobotCommandEndCallback()	-> used in case of distance to go reached or if timeout occurred
 *	- WheelStallRightCallback()			-> used to alert the application in case the Right wheel is stalled
 *	- WheelStallLeftCallback()			-> used to alert the application in case the Left wheel is stalled
 *
 * 	--------- Low Level functions -------------------
 * 	- GetWheelSpeed()					-> returns the speed of the selected wheel in mm/s units
 * 	- GetWheelStallCondition()			-> returns if the wheel is stalled or not
 * 	- GetWheelTurningDirection()		-> returns the momentary turning direction of the wheel
 * 	- GetWheelForwardPulses()			-> returns the total forward pulses of that wheel as counted by the wheel encoder since the last reset
 * 	- GetWheelBackwardPulses()			-> returns the total backward pulses of that wheel as counted by the wheel encoder since the last reset
 *
 */
//==================================================================================================================== @_SHORT END


//#define WHEEL_DEBUG_MODE
//====================================================================================================================

// custom data structures
//==================================================================================================================== CUSTOM_DATA_STRUCTS_START

typedef enum
{
	SIDE_UNINITIALIZED		= 0U,		/*  */
	SIDE_LEFT				= 1U,		/* the LEFT side of the Robot */
	SIDE_RIGHT				= 2U		/* the RIGHT side of the Robot */
}WHEEL_SIDE;

typedef enum
{
	WHEEL_NOT_TURNING		= 0U,		/*  */
	WHEEL_TURNING_FORWARD	= 1U,		/* Right Wheel turning FORWARD => decrements (1431 pulses / complete rotation */
	WHEEL_TURNING_BACKWARD	= 2U		/* Left Wheel turning BACKWARD => decrements (1431 pulses / complete rotation */

}WHEEL_TURNING_DIRECTION;

typedef enum
{
	WHEEL_STALLED_UNINITIALIZED		= 0U,		/*  */
	WHEEL_NOT_STALLED				= 1U,		/*  */
	WHEEL_STALLED					= 2U		/*  */
}WHEEL_STALL_STATUS;

typedef enum
{
	PWM_SPEED_STATUS_UNDEFINED		= 0U,		/*  */
	PWM_SPEED_TRACKING				= 1U,		/* every time a new PWM speed target is set to the ucPWMPercentageTartet variable, also the PWMSpeedStatus must be set to TRACKING */
	PWM_SPEED_REACHED				= 2U		/* once the Target speed is reached by the value in the ucPWMRampCounter this PWMSpeedStatus is set to REACHED */
}WHEEL_PWM_SPEED_STATUS;

typedef enum
{
	NO_ROTATION_CHANGE_DETECTED		= 0U,		/* no rotation change is needed */
	ROTATION_CHANGE_DETECTED		= 1U		/* it was a change on the rotation direction without stopping first the wheel => the ucPWMPercentageTartet must be considered only after STOP */
}WHEEL_ROTATION_CHANGE_WITHOUT_STOP;

typedef enum
{
	TIMEOUT_COMMAND_CALLBACK		= 1U,		/*  */
	DISTANCE_REACHED_CALLBACK		= 2U,		/*  */

}COMMAND_CALLBACK_REASON;

typedef enum
{
	ROBOT_DIRECTION_UNINITIALIZED	= 0U,		/*  */
	GOING_FORWARD					= 1U,		/*  */
	GOING_BACKWARD					= 2U		/*  */
}ROBOT_DIRECTION;

typedef enum
{
	STATE_UNKNOWN				= 0U,		/*  */
	STATE_OK					= 1U,		/*  */
	STATE_BUSY					= 2U,		/*  */
	STATE_PARAMETER_ERROR		= 3U		/*  */
}ROBOT_COMMAND_STATUS;

// encoders
typedef enum
{
	UPDATE_STATUS_UNKNOWN		= 0U,		/*  */
	NO_ENCODER_UPDATE			= 1U,		/* there is no new speed value available in the WheelEncoderTypeDef.uiSpeedMMS variable */
	NEW_ENCODER_VALUE			= 2U		/* there is a new speed measurement available in the WheelEncoderTypeDef.uiSpeedMMS variable */
}ENCODER_UPDATE_STATUS;

typedef struct
{
	WHEEL_SIDE							EncoderSide;				/* side of the encoder according to the robot orientation */
	volatile uint16_t  					uiOldEncoderRead;			/* used to store the last reading of the encoder */
	volatile uint16_t  					uiNewEncoderRead;			/* used to store the new counts from the CNT of the timer and the old value will be shifted into the "uiOldEncoderRead" */
	volatile uint16_t					uiPulseDiference;			/* it represents the min(abs(uiNewEncoderRead - uiOldEncoderRead) and abs(uiNewEncoderRead - uiOldEncoderRead +/- 65535)) */
	WHEEL_TURNING_DIRECTION   			TurningDirection;			/* tells the direction of the turning */
	volatile uint16_t					uiSpeedMMS;					/* measured speed in [mm/sec]*/
	ENCODER_UPDATE_STATUS				UpdateStatus;				/* if all is ok is set to NEW_ENCODER_VALUE after each update */
	volatile uint32_t					ui32TotalForwardPulses;		/* the total count of pulses resulted from forward turns */
	volatile uint32_t					ui32TotalBackwardPulses;	/* the total count of pulses resulted from backward turns */

}WheelEncoderTypeDef;


// motors
typedef struct
{
	WHEEL_SIDE							MotorSide;					/* side of the motor according to the robot orientation */
	WHEEL_TURNING_DIRECTION   			TurningDirection;			/* tells the direction of the turning of the motor (wheel) */
	volatile uint8_t					ucPWMPercentageTarget;		/* represents the percentage of the duty cycle of the signal applied to the motor. Range=[15 .... 100] */
	volatile uint8_t					ucPWMPercentageTarget2;		/*  used in case of wheel rotation direction change without previous stop. Range=[15 .... 100] */
	volatile uint8_t					ucPWMRampCounter;			/* ramps to reach the ucPWMPrecentageTarget. this is the actual PWM percentage applied to the motor. it is updated every 1ms */
	WHEEL_PWM_SPEED_STATUS				PWMSpeedStatus;				/* after this variable is set to TRACKING the PWMRampCounter will try to approach the PWMPrecentageTarget and once aporached this variable is set to PWM_SPEED_REACHED */
	WHEEL_ROTATION_CHANGE_WITHOUT_STOP  DirectionChangeWithoutStop;	/* set to "ROTATION_CHANGE_DETECTED" in case a direction change is detected while the wheel is turing in the opposite direction*/
	volatile uint16_t					uiRequestedSpeedMMS;		/* represents the requested speed of the wheel in mm/s. Range=[50 ... 500] */
	volatile uint32_t					ui32DistanceToGoPulses;		/* represents the distance requested for the motor (wheel) to travel until it stops */
	volatile uint8_t					ucTimeToRunHMS;				/* represents the time set for the motor to run in Hundreds of MilliSeconds. Range=[0 ... 250] => max 25.0 seconds */
	// Stall detection
	volatile uint8_t					ucStallCounter;				/* used to count successive zero speed measurements while the driving of the motor is not zero. */
	volatile uint8_t					ucStallMaxCountValue;		/* is is used to store the maximum number of successive zero speed counts in order to consider that the motor is stalled. one unit represents "TIMER_DIVIDER_MAX" ms */
	WHEEL_STALL_STATUS					StallCondition;				/* it reflects the stall status of the motor */
	WHEEL_STALL_STATUS					StallConditionCallback;		/* it is used to trigger the callback function from the main loop once the stall is detected*/
}WheelMotorTypeDef;

//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== CUSTOM_DATA_STRUCTS_END


// API
//==================================================================================================================== API_START
void WheelRobotStop(void);			// stops both motors
//----------------------------------------------------------------------------------
ROBOT_COMMAND_STATUS WheelRobotGoPWM(ROBOT_DIRECTION Direction, uint8_t ucPWMPercentage);	// used to command both wheels for a certain direction. PWM can be between 15% and 100%
//----------------------------------------------------------------------------------
ROBOT_COMMAND_STATUS WheelRobotGoPWMTmo(ROBOT_DIRECTION Direction, uint8_t ucPWMPercentage, uint8_t ucTimeToRunHMS);
// used to command both wheels for a certain direction and a specified time in Hundreds of MilliSeconds.
// - PWM can be between 15% and 100%
// - Time is in range of[1 ... 250] representing max 25.0 Seconds
//----------------------------------------------------------------------------------
ROBOT_COMMAND_STATUS WheelRobotGoPWMTmoDist(ROBOT_DIRECTION Direction, uint8_t ucPWMPercentage, uint8_t ucTimeToRunHMS, uint16_t uiDistanceToGoMM);
// used to command both wheels for a certain direction and a specified time in Hundreds of MilliSeconds or a specific distance given by uiDistanceToGoMM in [mm]
// - PWM can be between 15% and 100%
// - Time is in range of[1 ... 250] representing max 25.0 Seconds
// - Distance is in range of [1 ... 10000] representing max 10 m
//----------------------------------------------------------------------------------
ROBOT_COMMAND_STATUS WheelRobotGoSpeed(ROBOT_DIRECTION Direction, uint16_t uiSpeedMMS);		//  used to command both wheels for a certain direction. the speed is expressed in mm/sec. range [50 .... 500]
//----------------------------------------------------------------------------------
ROBOT_COMMAND_STATUS WheelRobotGoSpeedTmo(ROBOT_DIRECTION Direction, uint16_t uiSpeedMMS, uint8_t ucTimeToRunHMS);
// used to command both wheels for a certain direction and a specified time in Hundreds of MilliSeconds using a specified speed.
// - Speed (uiSpeedMMS) can be between in range [50 .... 500] expressed in mm/s units
// - Time is in range of[1 ... 250] representing max 25.0 Seconds
//----------------------------------------------------------------------------------
ROBOT_COMMAND_STATUS WheelRobotGoSpeedTmoDist(ROBOT_DIRECTION Direction, uint16_t uiSpeedMMS, uint8_t ucTimeToRunHMS, uint16_t uiDistanceToGoMM);
// used to command both wheels for a certain direction and a specified time in Hundreds of MilliSeconds or a specific distance given by uiDistanceToGoMM in [mm] using a specified speed.
// - Speed (uiSpeedMMS) can be between in range [50 .... 500] expressed in mm/s units
// - Time is in range of[1 ... 250] representing max 25.0 Seconds
// - Distance is in range of [1 ... 10000] representing max 10 m
//----------------------------------------------------------------------------------
ROBOT_COMMAND_STATUS WheelRobotRotate(int16_t iRotationAngle);		// used to turn the robot with a certain angle in the range of[-180 ... 180] degrees. trigonometric direction is positive
//----------------------------------------------------------------------------------
ROBOT_COMMAND_STATUS WheelRobotRotateTmo(int16_t iRotationAngle, uint8_t ucPWMPercentage);		// used to turn the robot with a certain angle in the range of[-180 ... 180] degrees. trigonometric direction is positive using timeout
//----------------------------------------------------------------------------------
ROBOT_COMMAND_STATUS WheelRightGo(ROBOT_DIRECTION Direction, uint8_t ucPWMPercentage);	// used to command the Right motor. PWM is in the range of 15% ... 100%
//----------------------------------------------------------------------------------
void WheelRightStop(void);			// used to stop the right motor
//----------------------------------------------------------------------------------
ROBOT_COMMAND_STATUS WheelLeftGo(ROBOT_DIRECTION Direction, uint8_t ucPWMPercentage);	// used to command the Right motor. PWM is in the range of 15% ... 100%
//----------------------------------------------------------------------------------
void WheelLeftStop(void);			// used to stop the left motor
//----------------------------------------------------------------------------------
ROBOT_COMMAND_STATUS WheelLefRighttGo(ROBOT_DIRECTION Direction, uint8_t ucPWMPercentage, uint8_t ucLeftRightBalance);	// used to command the both motors.
// - PWM is in the range of 15% ... 100%
// - ucLeftRightBalance is in the range of [0 ... 200]. neutral is 100 with both engines @ the same speed.
// --- 0 	=> Left motor @ maximum speed and Right motor stopped
// --- 200	=> Left engine is stopped and the Right engine is @ maximum speed
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
void WheelRobotCommandEndCallback(COMMAND_CALLBACK_REASON CallbackReason);			// called after each command finalization [__attribute__((weak)) ]
//----------------------------------------------------------------------------------
void WheelStallRightCallback(void);					// called if the will stalled (driving signal is not zero but the turning speed is zero) for the right motor [__attribute__((weak)) ]
//----------------------------------------------------------------------------------
void WheelStallLeftCallback(void);					// called if the will stalled (driving signal is not zero but the turning speed is zero) for the left motor [__attribute__((weak)) ]
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
uint16_t GetWheelSpeed(WHEEL_SIDE Side);								// returns the speed of the selected wheel in mm/s units
//----------------------------------------------------------------------------------
WHEEL_STALL_STATUS GetWheelStallCondition(WHEEL_SIDE Side);				// returns if the wheel is stalled or not
//----------------------------------------------------------------------------------
WHEEL_TURNING_DIRECTION GetWheelTurningDirection(WHEEL_SIDE Side);		// returns the momentary turning direction of the wheel
//----------------------------------------------------------------------------------
uint32_t GetWheelForwardPulses(WHEEL_SIDE Side);			// returns the total forward pulses of that wheel as counted by the wheel encoder since the last reset
//----------------------------------------------------------------------------------
uint32_t GetWheelBackwardPulses(WHEEL_SIDE Side);			// returns the total backward pulses of that wheel as counted by the wheel encoder since the last reset
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== API_END


// SYSTEM
//==================================================================================================================== SYSTEM_START
void WheelInit(void);				// used to initialize the Wheel Module. Called from main.c file @ init section
//----------------------------------------------------------------------------------
void WheelMainLoop(void);			// loop function of the Wheel Module. Called from the main.c file @ main loop section
//----------------------------------------------------------------------------------
void WheelTimer1KHZISR(void);		// 1KHZ timer function called from System ISR from stm32f4xx_it.c file @ SysTick Function (!!! keep as short as possible !!!)
//----------------------------------------------------------------------------------
#ifdef WHEEL_DEBUG_MODE
void WheelDebugRXString(uint8_t *ucRXBuffer, uint8_t ucRXBufferLength);		// used to receive RX data from the debug interface
void WheelDebugRXChar(uint8_t ucRXChar);		// used to receive RX single char from the debug interface
#endif
//----------------------------------------------------------------------------------
//==================================================================================================================== SYSTEM_END


// STATIC (INTERN)
//==================================================================================================================== STATIC_START
// defined in .c file
//==================================================================================================================== STATIC_END


#endif
