#ifndef _APP_WS2812_
#define _APP_WS2812_

// Settings, resources
/*
 * PWM on Timer1 Channel 4
 * - Input @ Timer1 = 180MHZ
 * - Prescaler = 0 => PSC+1 = 1 => FCNT = 180MHZ
 * - ARR Value = 224 => ARR+1 = aa5 => F_PWM = 800KHZ -> 1.25uS
 * - WS2812 Send High => 66% PWM pulse => 224*0.67 = 150 = value for the TIM1_CCR4 register
 * - WS2812 Send Low  => 33% PWM pulse => 224*0.33 = 74  = value for the TIM1_CCR4 register
 *
 */


// hue to RGB
// https://www.rapidtables.com/convert/color/hsl-to-rgb.html
//==================================================================================================================== USECASE START
/*
 * --------- to use a custom system display: ----------------
 * each LED as color and intensity using one of the functions:
 * -- WS2812SetDisplay(DISPLY_SYSTEM_x,...)
 * -- WS2812SetDisplayHue(DISPLY_SYSTEM_x,...)
 * -- WS2812SetDisplayGR(DISPLY_SYSTEM_x,...)
 * -- WS2812SetDisplayGB(DISPLY_SYSTEM_x,...)
 * after each of the 12 LED's from the display identified by "DISPLY_SYSTEM_x, with x in the range of 1 ... 6 (1 is highest priority)"
 * call the "WS2812ShowDisplay(DISPLY_SYSTEM_x, ui16DisplayFreezTimeout_ms)" to show that display for a time specified by "ui16DisplayFreezTimeout_ms"
 * ----------------------------------------------------------
 * ex: with the LED's electrical position
 *
 *  WS2812SetDisplay(DISPLY_SYSTEM_1, 1 , 255, 16, 0, 0);
	WS2812SetDisplay(DISPLY_SYSTEM_1, 2 , 255, 0, 16, 0);
	WS2812SetDisplay(DISPLY_SYSTEM_1, 3 , 255, 0, 0, 16);

	WS2812SetDisplay(DISPLY_SYSTEM_1, 4 , 255, 64, 0, 0);
	WS2812SetDisplay(DISPLY_SYSTEM_1, 5 , 255, 0, 64, 0);
	WS2812SetDisplay(DISPLY_SYSTEM_1, 6 , 255, 0, 0, 64);

	WS2812SetDisplay(DISPLY_SYSTEM_1, 7 , 255, 128, 0, 0);
	WS2812SetDisplay(DISPLY_SYSTEM_1, 8 , 255, 0, 128, 0);
	WS2812SetDisplay(DISPLY_SYSTEM_1, 9 , 255, 0, 0, 128);

	WS2812SetDisplay(DISPLY_SYSTEM_1, 10, 255, 255, 0, 0);
	WS2812SetDisplay(DISPLY_SYSTEM_1, 11, 255, 0, 255, 0);
	WS2812SetDisplay(DISPLY_SYSTEM_1, 12, 255, 0, 0, 255);

	WS2812ShowDisplay(DISPLY_SYSTEM_1,150);		// show DISPLY_SYSTEM_1 and do not refresh for 150mS
 * ----------------------------------------------------------
 * ex: with the LED's geometrical position
 *
 *  WS2812SetDisplay(DISPLY_SYSTEM_1, WS2812RemapLEDs(1) , 255, 16, 0, 0);
	WS2812SetDisplay(DISPLY_SYSTEM_1, WS2812RemapLEDs(2) , 255, 0, 16, 0);
	WS2812SetDisplay(DISPLY_SYSTEM_1, WS2812RemapLEDs(3) , 255, 0, 0, 16);

	WS2812SetDisplay(DISPLY_SYSTEM_1, WS2812RemapLEDs(4) , 255, 64, 0, 0);
	WS2812SetDisplay(DISPLY_SYSTEM_1, WS2812RemapLEDs(5) , 255, 0, 64, 0);
	WS2812SetDisplay(DISPLY_SYSTEM_1, WS2812RemapLEDs(6) , 255, 0, 0, 64);

	WS2812SetDisplay(DISPLY_SYSTEM_1, WS2812RemapLEDs(7) , 255, 128, 0, 0);
	WS2812SetDisplay(DISPLY_SYSTEM_1, WS2812RemapLEDs(8) , 255, 0, 128, 0);
	WS2812SetDisplay(DISPLY_SYSTEM_1, WS2812RemapLEDs(9) , 255, 0, 0, 128);

	WS2812SetDisplay(DISPLY_SYSTEM_1, WS2812RemapLEDs(10), 255, 255, 0, 0);
	WS2812SetDisplay(DISPLY_SYSTEM_1, WS2812RemapLEDs(11), 255, 0, 255, 0);
	WS2812SetDisplay(DISPLY_SYSTEM_1, WS2812RemapLEDs(12), 255, 0, 0, 255);

	WS2812ShowDisplay(DISPLY_SYSTEM_1,150);		// show DISPLY_SYSTEM_1 and do not refresh for 150mS
 * ----------------------------------------------------------
 *
 * --------------- to use an automation: --------------------
 * animation has priority over system display
 * A) first must edit some of the atomation's static function inside "WS2812.c"
 * 1) ex: for Automation3 -> Animation2Init()
 * ---- define the number of steps inside the automation (m_Animation3.ucAnimationMaxStage = 30; ... for 30 steps)
 * ---- define the time between steps (m_Animation3.ucAnimationPeriod = 50;)
 * 2) ex: for Automation3 -> Animation3Loop()
 * ---- define the behavior or setting of the LED's for each step
 * ---- if necessary the "m_Animation3.ucAnimationPeriod " step time value can also be changed after each step.
 * B) to start an automation must call the start function of the automation mentioning also the number of repetitions:
 * --- ex:for Automation4 -> WS2812Animation4Start(ucRepeatAnimation)
 * ---- ucRepeatAnimation == 0  		=> end current animation
 * ---- ucRepeatAnimation == 1 ... 254	=> repeat animation for this many times
 * ---- ucRepeatAnimation == 255		=> repeat endlessly this animation (or until called with ucRepeatAnimation set to ZERO)
 * ----------------------------------------------------------
 *
 * if needed than use "WS2812RemapLEDs()" function to convert the LED's electrical order (topology) to the real position (geometrical) topology
 *
 */
//==================================================================================================================== USECASE END


//#define WS2812_DEBUG_MODE
//====================================================================================================================


#define DISPLAY_FIRST   0U
typedef enum
{

	DISPLY_ALL_OFF			= DISPLAY_FIRST,
	DISPLY_ANIMATION		,					/* =1U */ /* used for Turn ON / Turn OFF animations */
	DISPLY_SYSTEM_1			,					/* =2U */
	DISPLY_SYSTEM_2			,					/* =3U */
	DISPLY_SYSTEM_3			,					/* =4U */
	DISPLY_SYSTEM_4			,					/* =5U */
	DISPLY_SYSTEM_5			,					/* =6U */
	DISPLY_SYSTEM_6			,					/* =7U */



	DISPALY_LAST											/* last increment */

}WS2812_DISPLY_ID;

typedef enum
{
	NO_ERROR				= 0U,
	RANGE_ERROR				= 1U
}WS2812_STATUS;

typedef enum
{
	DO_NOT_REFRESH_DISPLAY			=0U,
	REFRESH_DISPLAY					=1U
}WS2812_DISPLAY_UPDATE;


typedef struct
{
	uint32_t ui32WSRGBData[12];							/* uiWSRGBData[n]  ->  00000000 Gggggggg Rrrrrrrr Bbbbbbbb  => G,R,B are the MSB bits for the corresponding color information for the (n+1)'th LED, nin the range [0 ... 11]*/
	WS2812_DISPLAY_UPDATE  ucShowNow;					/* if this parameter is set to  REFRESH_DISPLAY than in the main loop the content of the display will be sent to the LEDs and this value will be set to DO_NOT_REFRESH_DISPLAY*/
	uint16_t ui16BKeepDisplayOn_ms;						/* timeout in milliseconds to keep the current display on the LEDs , block the refresh process*/
}WS2812DisplayDataType;


typedef enum
{
	ANIMATION_WAIT					=0U,
	SET_NEXT_ANIMATION				=1U
}WS2812_ANIMATION_UPDATE;

typedef enum
{
	ANIMATION_NOT_RUNNING			=0U,
	ANIMATION_RUNNING				=1U

}WS2812_ANIMATION_STATUS;

typedef struct
{
	WS2812_DISPLY_ID			DispID;							/* used to identify to which display is defined the animation */
	uint8_t						ucAnimationPeriod;				/* used to keep the animation interval, time is considered in [mS]*/
	volatile uint8_t			ucAnimationPeriodCounter;		/* used to count during the period time is considered in [mS]*/
	WS2812_ANIMATION_UPDATE		SetNextAnimation;				/* if is SET_NEXT_ANIMATION the next animation stage is configured into WS2812DisplayDataType[DispID] buffer and at the end the display's ucShowNow variable is set to REFRESH_DISPLAY*/
	uint8_t						ucAnimationMaxStage;			/* represents the number of stages inside the animation */
	volatile uint8_t			ucAnimationStageCounter;		/* represents the counting element for the animation's stages */
	WS2812_ANIMATION_STATUS		AnimationState;					/* while running the animation state is set to ANIMATION_RUNNING. if the stage counter reaches the maximum stages than set to ANIMATION_NOT_RUNNING */
	volatile uint8_t			ucAnimationRepeatCount;			/* it stores how many times the animation will be repeated. 0=means animation stops, 255 means animation counter do not decrement, 1...154 => decremented after each animiation */
}WS2812AnimationDataType;

// API
//==================================================================================================================== API_START
WS2812_STATUS WS2812SetDisplay(WS2812_DISPLY_ID WS2812DisplayID, uint8_t ucLEDIndex, uint8_t ucLEDIntensity, uint8_t ucLEDR, uint8_t ucLEDG, uint8_t ucLEDB);		// used to set the LED from a display
//----------------------------------------------------------------------------------
WS2812_STATUS WS2812SetDisplayHue(WS2812_DISPLY_ID WS2812DisplayID, uint8_t ucLEDIndex, uint8_t ucLEDIntensity, uint16_t uiHue);		// used to set the LED from a display using the Hue magnitude and with Saturation=100%, andIntensity=50%
//----------------------------------------------------------------------------------
WS2812_STATUS WS2812SetDisplayGR(WS2812_DISPLY_ID WS2812DisplayID, uint8_t ucLEDIndex, uint8_t ucLEDIntensity, uint8_t ucGRBalance);	// used to set the LED from a display with the GRBalance such that BLUE is always 0 and RED = ucGRBalance and GREEN  255 - BLUE
//----------------------------------------------------------------------------------
WS2812_STATUS WS2812SetDisplayGB(WS2812_DISPLY_ID WS2812DisplayID, uint8_t ucLEDIndex, uint8_t ucLEDIntensity, uint8_t ucGBBalance);	// used to set the LED from a display with the GBBalance such that RED is always 0 and BLUE = ucGRBalance and GREEN  255 - BLUE
//----------------------------------------------------------------------------------
WS2812_STATUS WS2812ShowDisplay(WS2812_DISPLY_ID WS2812DisplayID, uint16_t ui16DisplayFreezTimeout_ms);			// used to show a display buffer on the WS2812 LEDs
//----------------------------------------------------------------------------------
WS2812_STATUS WS2812EmptyDisplay(WS2812_DISPLY_ID WS2812DisplayID);			// used to clear a display, set all LEDs to OFF
//----------------------------------------------------------------------------------
void WS2812AnimationSwitchONStart(uint8_t ucRepeatAnimation);				// used to start the animation of the SwitchON sequence
//----------------------------------------------------------------------------------
void WS2812AnimationSwitchOFFStart(uint8_t ucRepeatAnimation);				// used to start the animation of the SwitchOFF sequence
//----------------------------------------------------------------------------------
void WS2812Animation1Start(uint8_t ucRepeatAnimation);						// used to start the custom animation 1 (it uses the display buffer identified by: WS2812_DISPLY_ID = DISPLY_ANIMATION_1)
//----------------------------------------------------------------------------------
void WS2812Animation2Start(uint8_t ucRepeatAnimation);						// used to start the custom animation 2 (it uses the display buffer identified by: WS2812_DISPLY_ID = DISPLY_ANIMATION_2)
//----------------------------------------------------------------------------------
void WS2812Animation3Start(uint8_t ucRepeatAnimation);						// used to start the custom animation 1 (it uses the display buffer identified by: WS2812_DISPLY_ID = DISPLY_ANIMATION_1)
//----------------------------------------------------------------------------------
void WS2812Animation4Start(uint8_t ucRepeatAnimation);						// used to start the custom animation 2 (it uses the display buffer identified by: WS2812_DISPLY_ID = DISPLY_ANIMATION_2)
//----------------------------------------------------------------------------------
void WS2812Animation5Start(uint8_t ucRepeatAnimation);						// used to start the custom animation 1 (it uses the display buffer identified by: WS2812_DISPLY_ID = DISPLY_ANIMATION_1)
//----------------------------------------------------------------------------------
void WS2812Animation6Start(uint8_t ucRepeatAnimation);						// used to start the custom animation 2 (it uses the display buffer identified by: WS2812_DISPLY_ID = DISPLY_ANIMATION_2)
//----------------------------------------------------------------------------------
uint8_t WS2812RemapLEDs(uint8_t ucRealLEDOrder);		// used to re-map the LED's according to geometrical position from the electrical order (information spreading order)
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== API_END



// SYSTEM
//==================================================================================================================== SYSTEM_START
void WS2812Init(void);				// used to initialize the WS2812 Module. Called from main.c file @ init section
//----------------------------------------------------------------------------------
void WS2812MainLoop(void);			// loop function of the WS2812 Module. Called from the main.c file @ main loop section
//----------------------------------------------------------------------------------
void WS2812Timer1KHZISR(void);		// 1KHZ timer function called from System ISR from stm32f4xx_it.c file @ SysTick Function
//----------------------------------------------------------------------------------
#ifdef WS2812_DEBUG_MODE
void WS2812DebugRXString(uint8_t *ucRXBuffer, uint8_t ucRXBufferLength);		// used to receive RX data from the debug interface
void WS2812DebugRXChar(uint8_t ucRXChar);		// used to receive RX single char from the debug interface
#endif
//----------------------------------------------------------------------------------
//==================================================================================================================== SYSTEM_END



// STATIC (INTERN)
//==================================================================================================================== STATIC_START
// defined in .c file
//==================================================================================================================== STATIC_END


#endif
