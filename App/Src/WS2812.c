#include "main.h"
#include "WS2812.h"
#include "stdint.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"




#ifdef WS2812_DEBUG_MODE
	char chDebugBuffer[100];
#endif


// STATIC (INTERN) prototypes
//==================================================================================================================== STATIC_START
// variables
extern TIM_HandleTypeDef htim1;


// defined in .c file
volatile uint16_t m_uiWS2812LedDataBuffer[((12 * 24) + 50)];		// buffer to hold the Data to be sent to the LEDs wia the DMA channel
// each LED consists of 3*8=24 data fields G[0-255] R[0-255] B[0-255]
// at the end 50 Zero values are sent as RESET pulse
// LED1  => m_uiWS2812LedDataBuffer[0-7]=G, m_uiWS2812LedDataBuffer[8-15]=R, m_uiWS2812LedDataBuffer[16-23]=B
// LED2  => m_uiWS2812LedDataBuffer[24-31]=G, m_uiWS2812LedDataBuffer[32-39]=R, m_uiWS2812LedDataBuffer[40-47]=B
// LED3  => m_uiWS2812LedDataBuffer[48-55]=G, m_uiWS2812LedDataBuffer[56-63]=R, m_uiWS2812LedDataBuffer[64-71]=B
// LEDn  => m_uiWS2812LedDataBuffer[((n-1)*24+0)-(+7)]=G, m_uiWS2812LedDataBuffer[((n-1)*24+8)-(+7)]=R, m_uiWS2812LedDataBuffer[((n-1)*24+16)-(+7)]=B
// LED12 => m_uiWS2812LedDataBuffer[264-271]=G, m_uiWS2812LedDataBuffer[272-279]=R, m_uiWS2812LedDataBuffer[280-287]=B
// ZEROs => m_uiWS2812LedDataBuffer[288-337]=G

volatile uint16_t m_uiStopDisplayRefresh_ms;		// used to freeze the LED updates for a certain time

volatile WS2812DisplayDataType  WS2812Displays[DISPALY_LAST] = {0};		// the display buffers for all the used displays
//----------------------------------------------------------------------------------
#define WS2812_SKIP_REAL_TIME		2000
uint16_t m_uiWS2812SkipRealTime;	// used in the main loop to skip real time
//----------------------------------------------------------------------------------
WS2812AnimationDataType m_AnimationSwitchON 	= {0};
WS2812AnimationDataType m_AnimationSwitchOFF 	= {0};
WS2812AnimationDataType m_Animation1 = {0};		// used for general purpose animation
WS2812AnimationDataType m_Animation2 = {0};		// used for general purpose animation
WS2812AnimationDataType m_Animation3 = {0};		// used for general purpose animation
WS2812AnimationDataType m_Animation4 = {0};		// used for general purpose animation
WS2812AnimationDataType m_Animation5 = {0};		// used for general purpose animation
WS2812AnimationDataType m_Animation6 = {0};		// used for general purpose animation
//----------------------------------------------------------------------------------
uint8_t m_ucLEDSwapMatrix[12];		// used to rearrange the LEDs according to their actual position
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------


// prototypes
static void SendWS2812Data(volatile uint32_t * ui32ColorBuffer);		// used to sent data to the LEDs
//----------------------------------------------------------------------------------
static void AnimationSwitchONInit(void);			// called @ initialization
//----------------------------------------------------------------------------------
static void AnimationSwitchONLoop(void);			// called in the main AnimationsMainLoop()
//----------------------------------------------------------------------------------
static void AnimationSwitchONTimer1KHZ(void);		// called from the 1KHZ ISR system timer
//----------------------------------------------------------------------------------
static void AnimationSwitchOFFInit(void);			// called @ initialization
//----------------------------------------------------------------------------------
static void AnimationSwitchOFFLoop(void);			// called in the main AnimationsMainLoop()
//----------------------------------------------------------------------------------
static void AnimationSwitchOFFTimer1KHZ(void);		// called from the 1KHZ ISR system timer
//----------------------------------------------------------------------------------
static void Animation1Init(void);			// called @ initialization
//----------------------------------------------------------------------------------
static void Animation1Loop(void);			// called in the main AnimationsMainLoop()
//----------------------------------------------------------------------------------
static void Animation1Timer1KHZ(void);		// called from the 1KHZ ISR system timer
//----------------------------------------------------------------------------------
static void Animation2Init(void);			// called @ initialization
//----------------------------------------------------------------------------------
static void Animation2Loop(void);			// called in the main AnimationsMainLoop()
//----------------------------------------------------------------------------------
static void Animation2Timer1KHZ(void);		// called from the 1KHZ ISR system timer
//----------------------------------------------------------------------------------
static void Animation3Init(void);			// called @ initialization
//----------------------------------------------------------------------------------
static void Animation3Loop(void);			// called in the main AnimationsMainLoop()
//----------------------------------------------------------------------------------
static void Animation3Timer1KHZ(void);		// called from the 1KHZ ISR system timer
//----------------------------------------------------------------------------------
static void Animation4Init(void);			// called @ initialization
//----------------------------------------------------------------------------------
static void Animation4Loop(void);			// called in the main AnimationsMainLoop()
//----------------------------------------------------------------------------------
static void Animation4Timer1KHZ(void);		// called from the 1KHZ ISR system timer
//----------------------------------------------------------------------------------
static void Animation5Init(void);			// called @ initialization
//----------------------------------------------------------------------------------
static void Animation5Loop(void);			// called in the main AnimationsMainLoop()
//----------------------------------------------------------------------------------
static void Animation5Timer1KHZ(void);		// called from the 1KHZ ISR system timer
//----------------------------------------------------------------------------------
static void Animation6Init(void);			// called @ initialization
//----------------------------------------------------------------------------------
static void Animation6Loop(void);			// called in the main AnimationsMainLoop()
//----------------------------------------------------------------------------------
static void Animation6Timer1KHZ(void);		// called from the 1KHZ ISR system timer
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
static void LEDSwapInit(void);						// configure the LED swap matrix
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
static void AnimationsMainLoop(void);		// called in the main loop to handle all thread mode for animations
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------

//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------

//==================================================================================================================== STATIC_END


// API
//==================================================================================================================== API_START
WS2812_STATUS WS2812SetDisplay(WS2812_DISPLY_ID WS2812DisplayID, uint8_t ucLEDIndex, uint8_t ucLEDIntensity, uint8_t ucLEDR, uint8_t ucLEDG, uint8_t ucLEDB)
{	// used to set the RGB and Intensity of an LED from an LED display object
	// - WS2812Display			=  member of "WS2812_DISPLY_ID" enum representing one display arrangement
	// - ucLEDIndex				=  LED from the display arrangement, RANGE=[1 ... 12] representing LED1 ... LED12
	// - ucLEDIntensity			=  LED intensity to scale the next RGB values. RANGE=[0 ... 255], 0=OFF, 255=MAX
	// - ucLEDR					=  LED Red component, 		RANGE=[0 ... 255], 0=OFF, 255=MAX
	// - ucLEDG					=  LED Green component, 	RANGE=[0 ... 255], 0=OFF, 255=MAX
	// - ucLEDB					=  LED Blue component, 		RANGE=[0 ... 255], 0=OFF, 255=MAX
	//---------------------------------------------

	if((WS2812DisplayID < (DISPLAY_FIRST)) || (WS2812DisplayID > (DISPALY_LAST - 1)))				return RANGE_ERROR;
	if((ucLEDIndex < 1) || (ucLEDIndex > 12)) 														return RANGE_ERROR;

	double dTemp;
	uint32_t uiTemp, uiTemp2;

	// GREEN
	dTemp     = (double)ucLEDIntensity;
	dTemp    *= (double)ucLEDG;
	dTemp    /= (double)255U;
	uiTemp2   = (uint32_t)dTemp;
	uiTemp2 <<= (16);
	uiTemp    = uiTemp2;

	// RED
	dTemp     = (double)ucLEDIntensity;
	dTemp    *= (double)ucLEDR;
	dTemp    /= (double)255U;
	uiTemp2   = (uint32_t)dTemp;
	uiTemp2 <<= (8);
	uiTemp   |= uiTemp2;

	// BLUE
	dTemp     = (double)ucLEDIntensity;
	dTemp    *= (double)ucLEDB;
	dTemp    /= (double)255U;
	uiTemp2   = (uint32_t)dTemp;
	uiTemp   |= uiTemp2;

	WS2812Displays[WS2812DisplayID].ui32WSRGBData[(ucLEDIndex - 1)] = uiTemp;

	return NO_ERROR;

}
//----------------------------------------------------------------------------------
WS2812_STATUS WS2812SetDisplayHue(WS2812_DISPLY_ID WS2812DisplayID, uint8_t ucLEDIndex, uint8_t ucLEDIntensity, uint16_t uiHue)
{	// used to set the LED from a display using the Hue magnitude and with Saturation=100%, andIntensity=50%
	// - WS2812Display			=  member of "WS2812_DISPLY_ID" enum representing one display arrangement
	// - ucLEDIndex				=  LED from the display arrangement, RANGE=[1 ... 12] representing LED1 ... LED12
	// - ucLEDIntensity			=  LED intensity to scale the next RGB values. RANGE=[0 ... 255], 0=OFF, 255=MAX
	// - uiHue					=  LED Hue Value, 		RANGE=[0 ... 360], 'S' is set to 100% and 'L' is set to 50%, https://www.rapidtables.com/convert/color/hsl-to-rgb.html
	//---------------------------------------------

	if((WS2812DisplayID < (DISPLAY_FIRST)) || (WS2812DisplayID > (DISPALY_LAST - 1)))				return RANGE_ERROR;
	if((ucLEDIndex < 1) || (ucLEDIndex > 12)) 														return RANGE_ERROR;
	if((uiHue > 360)) 																				return RANGE_ERROR;

	double C, X, m, R1, G1, B1, R, G, B;
	uint8_t ucR, ucG, ucB;

	// S = 1
	// L = 0.5

	//C = (1-|2L-1|) x S
	//C = (1-0)*1;
	C = 1;
	//X = C x (1-|(Hue/60)mod2 - 1|)
	X = (1-abs( ((uiHue/60)%2)-1 ) );
	//m = 0.5-C/2;
	m = 0;

	if( (uiHue < 60) )								{R1=C; G1=X; B1=0;}
	else if( (uiHue > 59 ) && (uiHue < 120) )		{R1=X; G1=C; B1=0;}
	else if( (uiHue > 119) && (uiHue < 180) )		{R1=0; G1=C; B1=X;}
	else if( (uiHue > 179) && (uiHue < 240) )		{R1=0; G1=X; B1=C;}
	else if( (uiHue > 239) && (uiHue < 300) )		{R1=X; G1=0; B1=C;}
	else											{R1=C; G1=0; B1=X;}

	R = (R1+m)*255;
	G = (G1+m)*255;
	B = (B1+m)*255;

	ucR = (uint8_t)R;
	ucG = (uint8_t)G;
	ucB = (uint8_t)B;

#ifdef WS2812_DEBUG_MODE
	sprintf(chDebugBuffer,"ucR=%d, ucG=%d, ucB=%d\r\n",ucR,ucG,ucB);
	DebugSendData((uint8_t*)chDebugBuffer,strlen(chDebugBuffer));
#endif


	return WS2812SetDisplay(WS2812DisplayID, ucLEDIndex, ucLEDIntensity, ucR, ucG, ucB);
}
//----------------------------------------------------------------------------------
WS2812_STATUS WS2812SetDisplayGR(WS2812_DISPLY_ID WS2812DisplayID, uint8_t ucLEDIndex, uint8_t ucLEDIntensity, uint8_t ucGRBalance)
{	// used to set the LED from a display with the GRBalance such that BLUE is always 0 and RED = ucGRBalance and GREEN = 255 - RED
	// this function sets the color of an LED such that BLUE is always set to ZERO and the  ucGRBalance determines the balance between GREEN and RED
	// - WS2812Display			=  member of "WS2812_DISPLY_ID" enum representing one display arrangement
	// - ucLEDIndex				=  LED from the display arrangement, RANGE=[1 ... 12] representing LED1 ... LED12
	// - ucLEDIntensity			=  LED intensity to scale the next RGB values. RANGE=[0 ... 255], 0=OFF, 255=MAX
	// - ucGRBalance			=> represents the color distribution between GREEN and RED, and the Range is [0 ... 255]
	// RED = ucGRBalance, GREEN = 255-ucGRBalance
	// -----------------------------------------------

	if((WS2812DisplayID < (DISPLAY_FIRST)) || (WS2812DisplayID > (DISPALY_LAST - 1)))				return RANGE_ERROR;
	if((ucLEDIndex < 1) || (ucLEDIndex > 12)) 														return RANGE_ERROR;


	return WS2812SetDisplay(WS2812DisplayID, ucLEDIndex, ucLEDIntensity, ucGRBalance, (255 - ucGRBalance), 0);
}
//----------------------------------------------------------------------------------
WS2812_STATUS WS2812SetDisplayGB(WS2812_DISPLY_ID WS2812DisplayID, uint8_t ucLEDIndex, uint8_t ucLEDIntensity, uint8_t ucGBBalance)
{	// used to set the LED from a display with the GRBalance such that RED is always 0 and BLUE = ucGBBalance and GREEN = 255 - BLUE
	// this function sets the color of an LED such that BLUE is always set to ZERO and the  ucGRBalance determines the balance between GREEN and RED
	// - WS2812Display			=  member of "WS2812_DISPLY_ID" enum representing one display arrangement
	// - ucLEDIndex				=  LED from the display arrangement, RANGE=[1 ... 12] representing LED1 ... LED12
	// - ucLEDIntensity			=  LED intensity to scale the next RGB values. RANGE=[0 ... 255], 0=OFF, 255=MAX
	// - ucGBBalance			=> represents the color distribution between GREEN and BLUE, and the Range is [0 ... 255]
	// BLUE = ucGBBalance, GREEN = 255-ucGBBalance
	// -----------------------------------------------

	if((WS2812DisplayID < (DISPLAY_FIRST)) || (WS2812DisplayID > (DISPALY_LAST - 1)))				return RANGE_ERROR;
	if((ucLEDIndex < 1) || (ucLEDIndex > 12)) 														return RANGE_ERROR;


	return WS2812SetDisplay(WS2812DisplayID, ucLEDIndex, ucLEDIntensity, 0, (255 - ucGBBalance), ucGBBalance);
}
//----------------------------------------------------------------------------------
WS2812_STATUS WS2812ShowDisplay(WS2812_DISPLY_ID WS2812DisplayID, uint16_t ui16DisplayFreezTimeout_ms)
{	// used to show a display buffer on the WS2812 LEDs
	if((WS2812DisplayID < (DISPLAY_FIRST)) || (WS2812DisplayID > (DISPALY_LAST - 1)))				return RANGE_ERROR;

	WS2812Displays[WS2812DisplayID].ucShowNow = REFRESH_DISPLAY;
	WS2812Displays[WS2812DisplayID].ui16BKeepDisplayOn_ms = ui16DisplayFreezTimeout_ms;

	return NO_ERROR;
}
//----------------------------------------------------------------------------------
WS2812_STATUS WS2812EmptyDisplay(WS2812_DISPLY_ID WS2812DisplayID)
{	// used to empty the display buffer identified by WS2812DisplayID
	//------------------------------------

	if((WS2812DisplayID < (DISPLAY_FIRST)) || (WS2812DisplayID > (DISPALY_LAST - 1)))				return RANGE_ERROR;

	for(int i=1;i<13;i++) 		WS2812SetDisplay(WS2812DisplayID, i , 0, 0, 0, 0);		// on display with ID = DisplayIndex, clear all the 12 LED data

	return NO_ERROR;
}
//----------------------------------------------------------------------------------
void WS2812AnimationSwitchONStart(uint8_t ucRepeatAnimation)
{	// used to start the animation sequence
	// - ucRepeatAnimation == 0			=> stop the current animation
	// - ucRepeatAnimation == 1...254	=> number to repeat the animation
	// - ucRepeatAnimation == 255		=> the animation is repeated endlessly
	//-----------------------------------

	if(ucRepeatAnimation == 0)
	{	// stop current animation
		m_AnimationSwitchON.ucAnimationRepeatCount = ucRepeatAnimation;
		return;
	}

	AnimationSwitchONInit();
	m_AnimationSwitchON.AnimationState = ANIMATION_RUNNING;
	m_AnimationSwitchON.ucAnimationRepeatCount = ucRepeatAnimation;

	return;
}
//----------------------------------------------------------------------------------
void WS2812AnimationSwitchOFFStart(uint8_t ucRepeatAnimation)
{	// used to start the animation sequence
	// - ucRepeatAnimation == 0			=> stop the current animation
	// - ucRepeatAnimation == 1...254	=> number to repeat the animation
	// - ucRepeatAnimation == 255		=> the animation is repeated endlessly
	//-----------------------------------

	if(ucRepeatAnimation == 0)
	{	// stop current animation
		m_AnimationSwitchOFF.ucAnimationRepeatCount = ucRepeatAnimation;
		return;
	}

	AnimationSwitchOFFInit();
	m_AnimationSwitchOFF.AnimationState = ANIMATION_RUNNING;
	m_AnimationSwitchOFF.ucAnimationRepeatCount = ucRepeatAnimation;

	return;
}
//----------------------------------------------------------------------------------
void WS2812Animation1Start(uint8_t ucRepeatAnimation)
{	// used to start the animation sequence
	// - ucRepeatAnimation == 0			=> stop the current animation
	// - ucRepeatAnimation == 1...254	=> number to repeat the animation
	// - ucRepeatAnimation == 255		=> the animation is repeated endlessly
	//-----------------------------------

	if(ucRepeatAnimation == 0)
	{	// stop current animation
		m_Animation1.ucAnimationRepeatCount = ucRepeatAnimation;
		return;
	}

	Animation1Init();
	m_Animation1.AnimationState = ANIMATION_RUNNING;
	m_Animation1.ucAnimationRepeatCount = ucRepeatAnimation;

	return;
}
//----------------------------------------------------------------------------------
void WS2812Animation2Start(uint8_t ucRepeatAnimation)
{	// used to start the animation sequence
	// - ucRepeatAnimation == 0			=> stop the current animation
	// - ucRepeatAnimation == 1...254	=> number to repeat the animation
	// - ucRepeatAnimation == 255		=> the animation is repeated endlessly
	//-----------------------------------

	if(ucRepeatAnimation == 0)
	{	// stop current animation
		m_Animation2.ucAnimationRepeatCount = ucRepeatAnimation;
		return;
	}


	Animation2Init();
	m_Animation2.AnimationState = ANIMATION_RUNNING;
	m_Animation2.ucAnimationRepeatCount = ucRepeatAnimation;

	return;
}
//----------------------------------------------------------------------------------
void WS2812Animation3Start(uint8_t ucRepeatAnimation)
{	// used to start the animation sequence
	// - ucRepeatAnimation == 0			=> stop the current animation
	// - ucRepeatAnimation == 1...254	=> number to repeat the animation
	// - ucRepeatAnimation == 255		=> the animation is repeated endlessly
	//-----------------------------------

	if(ucRepeatAnimation == 0)
	{	// stop current animation
		m_Animation3.ucAnimationRepeatCount = ucRepeatAnimation;
		return;
	}


	Animation3Init();
	m_Animation3.AnimationState = ANIMATION_RUNNING;
	m_Animation3.ucAnimationRepeatCount = ucRepeatAnimation;

	return;
}
//----------------------------------------------------------------------------------
void WS2812Animation4Start(uint8_t ucRepeatAnimation)
{	// used to start the animation sequence
	// - ucRepeatAnimation == 0			=> stop the current animation
	// - ucRepeatAnimation == 1...254	=> number to repeat the animation
	// - ucRepeatAnimation == 255		=> the animation is repeated endlessly
	//-----------------------------------

	if(ucRepeatAnimation == 0)
	{	// stop current animation
		m_Animation4.ucAnimationRepeatCount = ucRepeatAnimation;
		return;
	}


	Animation4Init();
	m_Animation4.AnimationState = ANIMATION_RUNNING;
	m_Animation4.ucAnimationRepeatCount = ucRepeatAnimation;

	return;
}
//----------------------------------------------------------------------------------
void WS2812Animation5Start(uint8_t ucRepeatAnimation)
{	// used to start the animation sequence
	// - ucRepeatAnimation == 0			=> stop the current animation
	// - ucRepeatAnimation == 1...254	=> number to repeat the animation
	// - ucRepeatAnimation == 255		=> the animation is repeated endlessly
	//-----------------------------------

	if(ucRepeatAnimation == 0)
	{	// stop current animation
		m_Animation5.ucAnimationRepeatCount = ucRepeatAnimation;
		return;
	}


	Animation5Init();
	m_Animation5.AnimationState = ANIMATION_RUNNING;
	m_Animation5.ucAnimationRepeatCount = ucRepeatAnimation;

	return;
}
//----------------------------------------------------------------------------------
void WS2812Animation6Start(uint8_t ucRepeatAnimation)
{	// used to start the animation sequence
	// - ucRepeatAnimation == 0			=> stop the current animation
	// - ucRepeatAnimation == 1...254	=> number to repeat the animation
	// - ucRepeatAnimation == 255		=> the animation is repeated endlessly
	//-----------------------------------

	if(ucRepeatAnimation == 0)
	{	// stop current animation
		m_Animation6.ucAnimationRepeatCount = ucRepeatAnimation;
		return;
	}


	Animation6Init();
	m_Animation6.AnimationState = ANIMATION_RUNNING;
	m_Animation6.ucAnimationRepeatCount = ucRepeatAnimation;

	return;
}
//----------------------------------------------------------------------------------
uint8_t WS2812RemapLEDs(uint8_t ucRealLEDOrder)
{	// used to re-map the LED's according to geometrical position from the electrical order (information spreading order)
	// -- ucRealLEDOrder = led according to electrical order as they are electrically cascaded
	// -- ucRealLEDOrder = range [1 ... 12]
	// -- returned value = the LED from geometrical order perspective => range = [1 ... 12]
	// ----------------------------------------

	if( (ucRealLEDOrder < 1) || (ucRealLEDOrder > 12) )		return 1;		// range error

	return m_ucLEDSwapMatrix[(ucRealLEDOrder - 1)];
}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== API_END



// SYSTEM
//==================================================================================================================== SYSTEM_START
void WS2812Init(void)				// used to initialize the WS2812 Module. Called from main.c file @ init section
{

	// general init
	m_uiWS2812SkipRealTime = WS2812_SKIP_REAL_TIME;

	// clear the WS2812 LED buffer
	memset((uint16_t*)m_uiWS2812LedDataBuffer,0,sizeof(m_uiWS2812LedDataBuffer));
	m_uiStopDisplayRefresh_ms = 0;

	// LED Swap Init
	LEDSwapInit();

	// Animations Init
	AnimationSwitchONInit();
	AnimationSwitchOFFInit();
	Animation1Init();
	Animation2Init();
	Animation3Init();
	Animation4Init();
	Animation5Init();
	Animation6Init();

	// first clear all LEDs @ startup---> temp just until lunching the ON Animation @ startup
	SendWS2812Data(WS2812Displays[DISPLY_ANIMATION].ui32WSRGBData);




	//HAL_TIMEx_PWMN_Start_DMA(htim, Channel, pData, Length)


	/*

	// only for testing
	WS2812SetDisplay(DISPLY_SYSTEM_1, 1 , 255, 16, 0, 0);
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

	SendWS2812Data(WS2812Displays[DISPLY_SYSTEM_1].ui32WSRGBData);
	WS2812ShowDisplay(DISPLY_SYSTEM_1,150);		// show DISPLY_SYSTEM_1 and do not refresh for 150mS
	*/

	return;
}
//----------------------------------------------------------------------------------
void WS2812MainLoop(void)			// loop function of the WS2812 Module. Called from the main.c file @ main loop section
{

	// skip real time
	if(m_uiWS2812SkipRealTime)
	{
		m_uiWS2812SkipRealTime --;
		return;
	}
	m_uiWS2812SkipRealTime = WS2812_SKIP_REAL_TIME;
	//------------------------------------------------------

	// Animations
	AnimationsMainLoop();
	//------------------------------------------------------


	//freeze display
	if(m_uiStopDisplayRefresh_ms)	return;


	// Display Setting
	if(WS2812Displays[DISPLY_ANIMATION].ucShowNow == REFRESH_DISPLAY)
	{	// set LED's according to the display
		SendWS2812Data(WS2812Displays[DISPLY_ANIMATION].ui32WSRGBData);
		WS2812Displays[DISPLY_ANIMATION].ucShowNow = DO_NOT_REFRESH_DISPLAY;
		//m_uiStopDisplayRefresh_ms = WS2812Displays[DISPLY_ANIMATION].ui16BKeepDisplayOn_ms;
		return;
	}
	if(WS2812Displays[DISPLY_SYSTEM_1].ucShowNow == REFRESH_DISPLAY)
	{	// set LED's according to the display
		SendWS2812Data(WS2812Displays[DISPLY_SYSTEM_1].ui32WSRGBData);
		WS2812Displays[DISPLY_SYSTEM_1].ucShowNow = DO_NOT_REFRESH_DISPLAY;
		m_uiStopDisplayRefresh_ms = WS2812Displays[DISPLY_SYSTEM_1].ui16BKeepDisplayOn_ms;
		return;
	}
	if(WS2812Displays[DISPLY_SYSTEM_2].ucShowNow == REFRESH_DISPLAY)
	{	// set LED's according to the display
		SendWS2812Data(WS2812Displays[DISPLY_SYSTEM_2].ui32WSRGBData);
		WS2812Displays[DISPLY_SYSTEM_2].ucShowNow = DO_NOT_REFRESH_DISPLAY;
		m_uiStopDisplayRefresh_ms = WS2812Displays[DISPLY_SYSTEM_2].ui16BKeepDisplayOn_ms;
		return;
	}
	if(WS2812Displays[DISPLY_SYSTEM_3].ucShowNow == REFRESH_DISPLAY)
	{	// set LED's according to the display
		SendWS2812Data(WS2812Displays[DISPLY_SYSTEM_3].ui32WSRGBData);
		WS2812Displays[DISPLY_SYSTEM_3].ucShowNow = DO_NOT_REFRESH_DISPLAY;
		m_uiStopDisplayRefresh_ms = WS2812Displays[DISPLY_SYSTEM_3].ui16BKeepDisplayOn_ms;
		return;
	}
	if(WS2812Displays[DISPLY_SYSTEM_4].ucShowNow == REFRESH_DISPLAY)
	{	// set LED's according to the display
		SendWS2812Data(WS2812Displays[DISPLY_SYSTEM_4].ui32WSRGBData);
		WS2812Displays[DISPLY_SYSTEM_4].ucShowNow = DO_NOT_REFRESH_DISPLAY;
		m_uiStopDisplayRefresh_ms = WS2812Displays[DISPLY_SYSTEM_4].ui16BKeepDisplayOn_ms;
		return;
	}
	if(WS2812Displays[DISPLY_SYSTEM_5].ucShowNow == REFRESH_DISPLAY)
	{	// set LED's according to the display
		SendWS2812Data(WS2812Displays[DISPLY_SYSTEM_5].ui32WSRGBData);
		WS2812Displays[DISPLY_SYSTEM_5].ucShowNow = DO_NOT_REFRESH_DISPLAY;
		m_uiStopDisplayRefresh_ms = WS2812Displays[DISPLY_SYSTEM_5].ui16BKeepDisplayOn_ms;
		return;
	}
	if(WS2812Displays[DISPLY_SYSTEM_6].ucShowNow == REFRESH_DISPLAY)
	{	// set LED's according to the display
		SendWS2812Data(WS2812Displays[DISPLY_SYSTEM_6].ui32WSRGBData);
		WS2812Displays[DISPLY_SYSTEM_6].ucShowNow = DO_NOT_REFRESH_DISPLAY;
		m_uiStopDisplayRefresh_ms = WS2812Displays[DISPLY_SYSTEM_6].ui16BKeepDisplayOn_ms;
		return;
	}

	//------------------------------------------------------

	return;
}
//----------------------------------------------------------------------------------
void WS2812Timer1KHZISR(void)		// 1KHZ timer function called from System ISR from stm32f4xx_it.c file @ SysTick Function
{	// max 2.4 uS
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);	//  debug on PC13 HIGH

	// freeze display
	if(m_uiStopDisplayRefresh_ms)		m_uiStopDisplayRefresh_ms --;


	// Switch OFF Animation
	AnimationSwitchOFFTimer1KHZ();

	// Animation 1
	Animation1Timer1KHZ();

	// Animation 2
	Animation2Timer1KHZ();

	// Animation 3
	Animation3Timer1KHZ();

	// Animation 4
	Animation4Timer1KHZ();

	// Animation 5
	Animation5Timer1KHZ();

	// Animation 6
	Animation6Timer1KHZ();



	// Switch ON Animation
	AnimationSwitchONTimer1KHZ();

	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);  //	debug on PC13 LOW

	return;
}
//----------------------------------------------------------------------------------
#ifdef WS2812_DEBUG_MODE
uint8_t m_ucTemp1, m_ucTemp2;
void WS2812DebugRXString(uint8_t *ucRXBuffer, uint8_t ucRXBufferLength)	// used to receive RX data from the debug interface
{
	return;
}
void WS2812DebugRXChar(uint8_t ucRXChar)	// used to receive RX single char from the debug interface
{
/*
	if(ucRXChar == 'q')
	{
		if(m_ucTemp1 < 255) m_ucTemp1 ++;
		WS2812SetDisplayGR(DISPLY_SYSTEM_1,1,m_ucTemp1,m_ucTemp2);
		WS2812ShowDisplay(DISPLY_SYSTEM_1,0);
		return;
	}
	if(ucRXChar == 'a')
	{
		if(m_ucTemp1 > 0) m_ucTemp1 --;
		WS2812SetDisplayGR(DISPLY_SYSTEM_1,1,m_ucTemp1,m_ucTemp2);
		WS2812ShowDisplay(DISPLY_SYSTEM_1,0);
		return;
	}

	if(ucRXChar == 'w')
	{
		if(m_ucTemp2 < 255) m_ucTemp2 ++;
		WS2812SetDisplayGR(DISPLY_SYSTEM_1,1,m_ucTemp1,m_ucTemp2);
		WS2812ShowDisplay(DISPLY_SYSTEM_1,0);
		return;
	}
	if(ucRXChar == 's')
	{
		if(m_ucTemp2 > 0) m_ucTemp2 --;
		WS2812SetDisplayGR(DISPLY_SYSTEM_1,1,m_ucTemp1,m_ucTemp2);
		WS2812ShowDisplay(DISPLY_SYSTEM_1,0);
		return;
	}
*/


	//m_ucTemp1
	if(ucRXChar == 's') WS2812AnimationSwitchONStart(1);
	if(ucRXChar == 'q') WS2812AnimationSwitchONStart(5);
	if(ucRXChar == 'w') WS2812AnimationSwitchONStart(255);
	if(ucRXChar == 'e') WS2812AnimationSwitchONStart(0);

	if(ucRXChar == 'a') WS2812AnimationSwitchOFFStart(3);
	if(ucRXChar == 'd') WS2812Animation1Start(4);
	if(ucRXChar == 'f') WS2812Animation2Start(5);

	if(ucRXChar == 'z') WS2812SetDisplayHue(DISPLY_SYSTEM_1,1,255,0);
	if(ucRXChar == 'x') WS2812SetDisplayHue(DISPLY_SYSTEM_1,1,255,120);
	if(ucRXChar == 'c') WS2812SetDisplayHue(DISPLY_SYSTEM_1,1,255,240);
	if(ucRXChar == 'v') WS2812SetDisplayHue(DISPLY_SYSTEM_1,1,255,60);
	if(ucRXChar == 'b') WS2812SetDisplayHue(DISPLY_SYSTEM_1,1,255,180);
	if(ucRXChar == 'n') WS2812SetDisplayHue(DISPLY_SYSTEM_1,1,255,300);

	WS2812ShowDisplay(DISPLY_SYSTEM_1,0);



	return;
}
#endif
//----------------------------------------------------------------------------------
//==================================================================================================================== SYSTEM_END



// STATIC (INTERN) implementation
//==================================================================================================================== STATIC_START
// defined in .c file
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{//UNUSED(htim);
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) 		HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_4);
}
//----------------------------------------------------------------------------------
static void SendWS2812Data(volatile uint32_t * ui32ColorBuffer)
{	// using data from ui32ColorBuffer it will prepare and send the m_uiWS2812LedDataBuffer[] data to the LEDs
	// volatile uint16_t m_uiWS2812LedDataBuffer[((12 * 24) + 50)];		// buffer to hold the Data to be sent to the LEDs wia the DMA channel
	// each LED consists of 3*8=24 data fields G[0-255] R[0-255] B[0-255]
	// LED1  => m_uiWS2812LedDataBuffer[0-7]=G, m_uiWS2812LedDataBuffer[8-15]=R, m_uiWS2812LedDataBuffer[16-23]=B
	// LED2  => m_uiWS2812LedDataBuffer[24-31]=G, m_uiWS2812LedDataBuffer[32-39]=R, m_uiWS2812LedDataBuffer[40-47]=B
	// LED3  => m_uiWS2812LedDataBuffer[48-55]=G, m_uiWS2812LedDataBuffer[56-63]=R, m_uiWS2812LedDataBuffer[64-71]=B
	// LEDn  => m_uiWS2812LedDataBuffer[((n-1)*24+0)-(+7)]=G, m_uiWS2812LedDataBuffer[((n-1)*24+8)-(+7)]=R, m_uiWS2812LedDataBuffer[((n-1)*24+16)-(+7)]=B
	// LED12 => m_uiWS2812LedDataBuffer[264-271]=G, m_uiWS2812LedDataBuffer[272-279]=R, m_uiWS2812LedDataBuffer[280-287]=B
	// ZEROs => m_uiWS2812LedDataBuffer[288-337]=G
	//--------------------------------------------
	// ui32ColorBuffer[12] :
	// LEDn  => ui32ColorBuffer[(n-1)] -> 00000000 Gggggggg Rrrrrrrr Bbbbbbbb  => G,R,B are the MSB bits for the corresponding color information

	int i,j;

	for(i=0;i<12;i++)
	{
		for(j=23;j>=0;j--)
		{
			if(ui32ColorBuffer[i] & (1U<<j))	m_uiWS2812LedDataBuffer[((i*24) + (23-j))] = 150;	// -> sending logic 1 = 66% of the period is HIGH
			else								m_uiWS2812LedDataBuffer[((i*24) + (23-j))] = 74;	// -> sending logic 0 = 33% of the period is HIGH
		}
	}
	for(i=288;i<338;i++)						m_uiWS2812LedDataBuffer[i] = 0;						// add ZEROS as reset period after the data was sent to WS2812

	if(HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_4, (uint32_t *)m_uiWS2812LedDataBuffer, ((12 * 24) + 50)) != HAL_OK)
	 {
		Error_Handler2(ERROR_CODE_WS2812_SENDWS2812DATA);
		//Error_Handler();
	 }

}
//----------------------------------------------------------------------------------
static void AnimationSwitchONInit(void)
{	// called @ initialization
	m_AnimationSwitchON.DispID = DISPLY_ANIMATION;
	m_AnimationSwitchON.AnimationState = ANIMATION_NOT_RUNNING;
	m_AnimationSwitchON.SetNextAnimation = ANIMATION_WAIT;
	m_AnimationSwitchON.ucAnimationMaxStage = 30;
	m_AnimationSwitchON.ucAnimationStageCounter = m_AnimationSwitchON.ucAnimationMaxStage ;
	m_AnimationSwitchON.ucAnimationPeriod = 50;
	m_AnimationSwitchON.ucAnimationPeriodCounter = m_AnimationSwitchON.ucAnimationPeriod;
	m_AnimationSwitchON.ucAnimationRepeatCount = 0;	// by default the animation do not repeat

	// clear the display buffer for this animation
	WS2812EmptyDisplay(m_AnimationSwitchON.DispID);

	return;
}
//----------------------------------------------------------------------------------
static void AnimationSwitchONLoop(void)
{	// called from the main loop to handle the animation
	// --------------------------------------
	if(m_AnimationSwitchON.SetNextAnimation == ANIMATION_WAIT)		return;			// waiting for the time elapse

	if(m_AnimationSwitchON.ucAnimationStageCounter == 0)
	{
		if(m_AnimationSwitchON.ucAnimationRepeatCount <= 1)
		{	// end the animation
			m_AnimationSwitchON.AnimationState = ANIMATION_NOT_RUNNING;				// this animation is done
			return;
		}
		// prepare another animation
		if(m_AnimationSwitchON.ucAnimationRepeatCount < 255) m_AnimationSwitchON.ucAnimationRepeatCount --;
		m_AnimationSwitchON.ucAnimationStageCounter = m_AnimationSwitchON.ucAnimationMaxStage ;
		m_AnimationSwitchON.ucAnimationPeriodCounter = m_AnimationSwitchON.ucAnimationPeriod;
	}
	//-------------------------------------

	if( (m_AnimationSwitchON.ucAnimationStageCounter > (m_AnimationSwitchON.ucAnimationMaxStage - 12)) )
	{	// turning LEDs GREEN High intensity ON
		//WS2812SetDisplay(m_AnimationSwitchON.DispID, (m_AnimationSwitchON.ucAnimationMaxStage - m_AnimationSwitchON.ucAnimationStageCounter + 1) , 255, 0, 255, 0);
		WS2812SetDisplay(m_AnimationSwitchON.DispID, m_ucLEDSwapMatrix[(m_AnimationSwitchON.ucAnimationMaxStage - m_AnimationSwitchON.ucAnimationStageCounter)] , 255, 0, 255, 0);
		WS2812ShowDisplay(m_AnimationSwitchON.DispID,0);
	}

	if( (m_AnimationSwitchON.ucAnimationStageCounter < 13) )
	{	// Setting LEDs GREEN Low Intensity
		//WS2812SetDisplay(m_AnimationSwitchON.DispID,(12-m_AnimationSwitchON.ucAnimationStageCounter),4, 0, 255, 0);
		WS2812SetDisplay(m_AnimationSwitchON.DispID,m_ucLEDSwapMatrix[(12-m_AnimationSwitchON.ucAnimationStageCounter)],4, 0, 255, 0);
		WS2812ShowDisplay(m_AnimationSwitchON.DispID,0);
	}

	m_AnimationSwitchON.ucAnimationPeriodCounter = m_AnimationSwitchON.ucAnimationPeriod;	// set the new timer
	m_AnimationSwitchON.SetNextAnimation = ANIMATION_WAIT;									// wait for the time to elapse
	m_AnimationSwitchON.ucAnimationStageCounter --;											// select the next stage of the animation
	return;
}
//----------------------------------------------------------------------------------
static void AnimationSwitchONTimer1KHZ(void)
{// called from the 1KHZ ISR system timer

	if(m_AnimationSwitchON.ucAnimationPeriodCounter > 1) 		m_AnimationSwitchON.ucAnimationPeriodCounter --;
	else
	{
		m_AnimationSwitchON.ucAnimationPeriodCounter = 0;
		m_AnimationSwitchON.SetNextAnimation = SET_NEXT_ANIMATION;
	}

	return;
}
//----------------------------------------------------------------------------------
static void AnimationSwitchOFFInit(void)
{	// called @ initialization
	m_AnimationSwitchOFF.DispID = DISPLY_ANIMATION;
	m_AnimationSwitchOFF.AnimationState = ANIMATION_NOT_RUNNING;
	m_AnimationSwitchOFF.SetNextAnimation = ANIMATION_WAIT;
	m_AnimationSwitchOFF.ucAnimationMaxStage = 30;
	m_AnimationSwitchOFF.ucAnimationStageCounter = m_AnimationSwitchOFF.ucAnimationMaxStage ;
	m_AnimationSwitchOFF.ucAnimationPeriod = 50;
	m_AnimationSwitchOFF.ucAnimationPeriodCounter = m_AnimationSwitchOFF.ucAnimationPeriod;
	m_AnimationSwitchOFF.ucAnimationRepeatCount = 0;

	// clear the display buffer for this animation
	//WS2812EmptyDisplay(m_AnimationSwitchOFF.DispID);

	return;
}
//----------------------------------------------------------------------------------
static void AnimationSwitchOFFLoop(void)
{	// called from the main loop to handle the animation
	// --------------------------------------
	if(m_AnimationSwitchOFF.SetNextAnimation == ANIMATION_WAIT)		return;		// waiting for the time elapse

	if(m_AnimationSwitchOFF.ucAnimationStageCounter == 0)
	{
		if(m_AnimationSwitchOFF.ucAnimationRepeatCount <= 1)
		{	// end the animation
			m_AnimationSwitchOFF.AnimationState = ANIMATION_NOT_RUNNING;				// this animation is done
			return;
		}
		// prepare another animation
		if(m_AnimationSwitchOFF.ucAnimationRepeatCount < 255) m_AnimationSwitchOFF.ucAnimationRepeatCount --;
		m_AnimationSwitchOFF.ucAnimationStageCounter = m_AnimationSwitchOFF.ucAnimationMaxStage ;
		m_AnimationSwitchOFF.ucAnimationPeriodCounter = m_AnimationSwitchOFF.ucAnimationPeriod;
	}
	//-------------------------------------

	if( (m_AnimationSwitchOFF.ucAnimationStageCounter > (m_AnimationSwitchOFF.ucAnimationMaxStage - 12)) )
	{	// turning LEDs RED High intensity ON
		//WS2812SetDisplay(m_AnimationSwitchOFF.DispID, (m_AnimationSwitchOFF.ucAnimationMaxStage - m_AnimationSwitchOFF.ucAnimationStageCounter + 1) , 255, 0, 255, 0);
		WS2812SetDisplay(m_AnimationSwitchOFF.DispID, m_ucLEDSwapMatrix[(11-(m_AnimationSwitchOFF.ucAnimationMaxStage - m_AnimationSwitchOFF.ucAnimationStageCounter))] , 255, 255, 0, 0);
		WS2812ShowDisplay(m_AnimationSwitchOFF.DispID,0);
	}

	if( (m_AnimationSwitchOFF.ucAnimationStageCounter < 13) )
	{	// Setting LEDs OFF
		//WS2812SetDisplay(m_AnimationSwitchOFF.DispID,(12-m_AnimationSwitchOFF.ucAnimationStageCounter),4, 0, 255, 0);
		WS2812SetDisplay(m_AnimationSwitchOFF.DispID,m_ucLEDSwapMatrix[(11-(12-m_AnimationSwitchOFF.ucAnimationStageCounter))],0, 0, 0, 0);
		WS2812ShowDisplay(m_AnimationSwitchOFF.DispID,0);
	}

	m_AnimationSwitchOFF.ucAnimationPeriodCounter = m_AnimationSwitchOFF.ucAnimationPeriod;	// set the new timer
	m_AnimationSwitchOFF.SetNextAnimation = ANIMATION_WAIT;									// wait for the time to elapse
	m_AnimationSwitchOFF.ucAnimationStageCounter --;											// select the next stage of the animation
	return;
}
//----------------------------------------------------------------------------------
static void AnimationSwitchOFFTimer1KHZ(void)
{// called from the 1KHZ ISR system timer

	if(m_AnimationSwitchOFF.ucAnimationPeriodCounter > 1) 		m_AnimationSwitchOFF.ucAnimationPeriodCounter --;
	else
	{
		m_AnimationSwitchOFF.ucAnimationPeriodCounter = 0;
		m_AnimationSwitchOFF.SetNextAnimation = SET_NEXT_ANIMATION;
	}

	return;
}
//----------------------------------------------------------------------------------
static void Animation1Init(void)
{	// called @ initialization
	m_Animation1.DispID = DISPLY_ANIMATION;
	m_Animation1.AnimationState = ANIMATION_NOT_RUNNING;
	m_Animation1.SetNextAnimation = ANIMATION_WAIT;
	m_Animation1.ucAnimationMaxStage = 30;
	m_Animation1.ucAnimationStageCounter = m_Animation1.ucAnimationMaxStage ;
	m_Animation1.ucAnimationPeriod = 100;
	m_Animation1.ucAnimationPeriodCounter = m_Animation1.ucAnimationPeriod;
	m_Animation1.ucAnimationRepeatCount = 0;

	// clear the display buffer for this animation
	WS2812EmptyDisplay(m_Animation1.DispID);

	return;
}
//----------------------------------------------------------------------------------
static void Animation1Loop(void)
{	// called from the main loop to handle the animation
	// --------------------------------------
	if(m_Animation1.SetNextAnimation == ANIMATION_WAIT)			return;		// waiting for the time elapse

	if(m_Animation1.ucAnimationStageCounter == 0)
	{
		if(m_Animation1.ucAnimationRepeatCount <= 1)
		{	// end the animation
			m_Animation1.AnimationState = ANIMATION_NOT_RUNNING;				// this animation is done
			return;
		}
		// prepare another animation
		if(m_Animation1.ucAnimationRepeatCount < 255) m_Animation1.ucAnimationRepeatCount --;
		m_Animation1.ucAnimationStageCounter = m_Animation1.ucAnimationMaxStage ;
		m_Animation1.ucAnimationPeriodCounter = m_Animation1.ucAnimationPeriod;
	}
	//-------------------------------------

	if( (m_Animation1.ucAnimationStageCounter > (m_Animation1.ucAnimationMaxStage - 12)) )
	{	// turning LEDs ON / OFF
		WS2812SetDisplay(m_Animation1.DispID, m_ucLEDSwapMatrix[(11-(m_Animation1.ucAnimationMaxStage - m_Animation1.ucAnimationStageCounter))] , 255, 255, 255, 255);
		WS2812ShowDisplay(m_Animation1.DispID,0);
	}

	if( (m_Animation1.ucAnimationStageCounter < 13) )
	{	// Setting LEDs ON / OFF
		WS2812SetDisplay(m_Animation1.DispID,m_ucLEDSwapMatrix[(11-(12-m_Animation1.ucAnimationStageCounter))],0, 0, 0, 0);
		WS2812ShowDisplay(m_Animation1.DispID,0);
	}

	m_Animation1.ucAnimationPeriodCounter = m_Animation1.ucAnimationPeriod;			// set the new timer
	m_Animation1.SetNextAnimation = ANIMATION_WAIT;									// wait for the time to elapse
	m_Animation1.ucAnimationStageCounter --;										// select the next stage of the animation
	return;
}
//----------------------------------------------------------------------------------
static void Animation1Timer1KHZ(void)
{// called from the 1KHZ ISR system timer

	if(m_Animation1.ucAnimationPeriodCounter > 1) 		m_Animation1.ucAnimationPeriodCounter --;
	else
	{
		m_Animation1.ucAnimationPeriodCounter = 0;
		m_Animation1.SetNextAnimation = SET_NEXT_ANIMATION;
	}

	return;
}
//----------------------------------------------------------------------------------
static void Animation2Init(void)
{	// called @ initialization
	m_Animation2.DispID = DISPLY_ANIMATION;
	m_Animation2.AnimationState = ANIMATION_NOT_RUNNING;
	m_Animation2.SetNextAnimation = ANIMATION_WAIT;
	m_Animation2.ucAnimationMaxStage = 30;
	m_Animation2.ucAnimationStageCounter = m_Animation2.ucAnimationMaxStage ;
	m_Animation2.ucAnimationPeriod = 50;
	m_Animation2.ucAnimationPeriodCounter = m_Animation2.ucAnimationPeriod;
	m_Animation2.ucAnimationRepeatCount = 0;

	// clear the display buffer for this animation
	WS2812EmptyDisplay(m_Animation2.DispID);

	return;
}
//----------------------------------------------------------------------------------
static void Animation2Loop(void)
{	// called from the main loop to handle the animation
	// --------------------------------------
	if(m_Animation2.SetNextAnimation == ANIMATION_WAIT)			return;		// waiting for the time elapse

	if(m_Animation2.ucAnimationStageCounter == 0)
	{
		if(m_Animation2.ucAnimationRepeatCount <= 1)
		{	// end the animation
			m_Animation2.AnimationState = ANIMATION_NOT_RUNNING;				// this animation is done
			return;
		}
		// prepare another animation
		if(m_Animation2.ucAnimationRepeatCount < 255) m_Animation2.ucAnimationRepeatCount --;
		m_Animation2.ucAnimationStageCounter = m_Animation2.ucAnimationMaxStage ;
		m_Animation2.ucAnimationPeriodCounter = m_Animation2.ucAnimationPeriod;
	}
	//-------------------------------------

	if( (m_Animation2.ucAnimationStageCounter > (m_Animation2.ucAnimationMaxStage - 12)) )
	{	// turning LEDs ON / OFF
		WS2812SetDisplay(m_Animation2.DispID, m_ucLEDSwapMatrix[(11-(m_Animation2.ucAnimationMaxStage - m_Animation2.ucAnimationStageCounter))] , 255, 0, 0, 255);
		WS2812ShowDisplay(m_Animation2.DispID,0);
	}

	if( (m_Animation2.ucAnimationStageCounter < 13) )
	{	// Setting LEDs ON / OFF
		WS2812SetDisplay(m_Animation2.DispID,m_ucLEDSwapMatrix[(11-(12-m_Animation2.ucAnimationStageCounter))],0, 0, 0, 0);
		WS2812ShowDisplay(m_Animation2.DispID,0);
	}

	m_Animation2.ucAnimationPeriodCounter = m_Animation2.ucAnimationPeriod;			// set the new timer
	m_Animation2.SetNextAnimation = ANIMATION_WAIT;									// wait for the time to elapse
	m_Animation2.ucAnimationStageCounter --;										// select the next stage of the animation
	return;
}
//----------------------------------------------------------------------------------
static void Animation2Timer1KHZ(void)
{// called from the 1KHZ ISR system timer

	if(m_Animation2.ucAnimationPeriodCounter > 1) 		m_Animation2.ucAnimationPeriodCounter --;
	else
	{
		m_Animation2.ucAnimationPeriodCounter = 0;
		m_Animation2.SetNextAnimation = SET_NEXT_ANIMATION;
	}

	return;
}
//----------------------------------------------------------------------------------
static void Animation3Init(void)
{	// called @ initialization
	m_Animation3.DispID = DISPLY_ANIMATION;
	m_Animation3.AnimationState = ANIMATION_NOT_RUNNING;
	m_Animation3.SetNextAnimation = ANIMATION_WAIT;
	m_Animation3.ucAnimationMaxStage = 30;
	m_Animation3.ucAnimationStageCounter = m_Animation3.ucAnimationMaxStage ;
	m_Animation3.ucAnimationPeriod = 50;
	m_Animation3.ucAnimationPeriodCounter = m_Animation3.ucAnimationPeriod;
	m_Animation3.ucAnimationRepeatCount = 0;

	// clear the display buffer for this animation
	WS2812EmptyDisplay(m_Animation3.DispID);

	return;
}
//----------------------------------------------------------------------------------
static void Animation3Loop(void)
{	// called from the main loop to handle the animation
	// --------------------------------------
	if(m_Animation3.SetNextAnimation == ANIMATION_WAIT)			return;		// waiting for the time elapse

	if(m_Animation3.ucAnimationStageCounter == 0)
	{
		if(m_Animation3.ucAnimationRepeatCount <= 1)
		{	// end the animation
			m_Animation3.AnimationState = ANIMATION_NOT_RUNNING;				// this animation is done
			return;
		}
		// prepare another animation
		if(m_Animation3.ucAnimationRepeatCount < 255) m_Animation3.ucAnimationRepeatCount --;
		m_Animation3.ucAnimationStageCounter = m_Animation3.ucAnimationMaxStage ;
		m_Animation3.ucAnimationPeriodCounter = m_Animation3.ucAnimationPeriod;
	}
	//-------------------------------------

	if( (m_Animation3.ucAnimationStageCounter > (m_Animation3.ucAnimationMaxStage - 12)) )
	{	// turning LEDs ON / OFF
		WS2812SetDisplay(m_Animation3.DispID, m_ucLEDSwapMatrix[(11-(m_Animation3.ucAnimationMaxStage - m_Animation3.ucAnimationStageCounter))] , 255, 0, 0, 255);
		WS2812ShowDisplay(m_Animation3.DispID,0);
	}

	if( (m_Animation3.ucAnimationStageCounter < 13) )
	{	// Setting LEDs ON / OFF
		WS2812SetDisplay(m_Animation3.DispID,m_ucLEDSwapMatrix[(11-(12-m_Animation3.ucAnimationStageCounter))],0, 0, 0, 0);
		WS2812ShowDisplay(m_Animation3.DispID,0);
	}

	m_Animation3.ucAnimationPeriodCounter = m_Animation3.ucAnimationPeriod;			// set the new timer
	m_Animation3.SetNextAnimation = ANIMATION_WAIT;									// wait for the time to elapse
	m_Animation3.ucAnimationStageCounter --;										// select the next stage of the animation
	return;
}
//----------------------------------------------------------------------------------
static void Animation3Timer1KHZ(void)
{// called from the 1KHZ ISR system timer

	if(m_Animation3.ucAnimationPeriodCounter > 1) 		m_Animation3.ucAnimationPeriodCounter --;
	else
	{
		m_Animation3.ucAnimationPeriodCounter = 0;
		m_Animation3.SetNextAnimation = SET_NEXT_ANIMATION;
	}

	return;
}
//----------------------------------------------------------------------------------
static void Animation4Init(void)
{	// called @ initialization
	m_Animation4.DispID = DISPLY_ANIMATION;
	m_Animation4.AnimationState = ANIMATION_NOT_RUNNING;
	m_Animation4.SetNextAnimation = ANIMATION_WAIT;
	m_Animation4.ucAnimationMaxStage = 30;
	m_Animation4.ucAnimationStageCounter = m_Animation4.ucAnimationMaxStage ;
	m_Animation4.ucAnimationPeriod = 50;
	m_Animation4.ucAnimationPeriodCounter = m_Animation4.ucAnimationPeriod;
	m_Animation4.ucAnimationRepeatCount = 0;

	// clear the display buffer for this animation
	WS2812EmptyDisplay(m_Animation4.DispID);

	return;
}
//----------------------------------------------------------------------------------
static void Animation4Loop(void)
{	// called from the main loop to handle the animation
	// --------------------------------------
	if(m_Animation4.SetNextAnimation == ANIMATION_WAIT)			return;		// waiting for the time elapse

	if(m_Animation4.ucAnimationStageCounter == 0)
	{
		if(m_Animation4.ucAnimationRepeatCount <= 1)
		{	// end the animation
			m_Animation4.AnimationState = ANIMATION_NOT_RUNNING;				// this animation is done
			return;
		}
		// prepare another animation
		if(m_Animation4.ucAnimationRepeatCount < 255) m_Animation4.ucAnimationRepeatCount --;
		m_Animation4.ucAnimationStageCounter = m_Animation4.ucAnimationMaxStage ;
		m_Animation4.ucAnimationPeriodCounter = m_Animation4.ucAnimationPeriod;
	}
	//-------------------------------------

	if( (m_Animation4.ucAnimationStageCounter > (m_Animation4.ucAnimationMaxStage - 12)) )
	{	// turning LEDs ON / OFF
		WS2812SetDisplay(m_Animation4.DispID, m_ucLEDSwapMatrix[(11-(m_Animation4.ucAnimationMaxStage - m_Animation4.ucAnimationStageCounter))] , 255, 0, 0, 255);
		WS2812ShowDisplay(m_Animation4.DispID,0);
	}

	if( (m_Animation4.ucAnimationStageCounter < 13) )
	{	// Setting LEDs ON / OFF
		WS2812SetDisplay(m_Animation4.DispID,m_ucLEDSwapMatrix[(11-(12-m_Animation4.ucAnimationStageCounter))],0, 0, 0, 0);
		WS2812ShowDisplay(m_Animation4.DispID,0);
	}

	m_Animation4.ucAnimationPeriodCounter = m_Animation4.ucAnimationPeriod;			// set the new timer
	m_Animation4.SetNextAnimation = ANIMATION_WAIT;									// wait for the time to elapse
	m_Animation4.ucAnimationStageCounter --;										// select the next stage of the animation
	return;
}
//----------------------------------------------------------------------------------
static void Animation4Timer1KHZ(void)
{// called from the 1KHZ ISR system timer

	if(m_Animation4.ucAnimationPeriodCounter > 1) 		m_Animation4.ucAnimationPeriodCounter --;
	else
	{
		m_Animation4.ucAnimationPeriodCounter = 0;
		m_Animation4.SetNextAnimation = SET_NEXT_ANIMATION;
	}

	return;
}
//----------------------------------------------------------------------------------
static void Animation5Init(void)
{	// called @ initialization
	m_Animation5.DispID = DISPLY_ANIMATION;
	m_Animation5.AnimationState = ANIMATION_NOT_RUNNING;
	m_Animation5.SetNextAnimation = ANIMATION_WAIT;
	m_Animation5.ucAnimationMaxStage = 30;
	m_Animation5.ucAnimationStageCounter = m_Animation5.ucAnimationMaxStage ;
	m_Animation5.ucAnimationPeriod = 50;
	m_Animation5.ucAnimationPeriodCounter = m_Animation5.ucAnimationPeriod;
	m_Animation5.ucAnimationRepeatCount = 0;

	// clear the display buffer for this animation
	WS2812EmptyDisplay(m_Animation5.DispID);

	return;
}
//----------------------------------------------------------------------------------
static void Animation5Loop(void)
{	// called from the main loop to handle the animation
	// --------------------------------------
	if(m_Animation5.SetNextAnimation == ANIMATION_WAIT)			return;		// waiting for the time elapse

	if(m_Animation5.ucAnimationStageCounter == 0)
	{
		if(m_Animation5.ucAnimationRepeatCount <= 1)
		{	// end the animation
			m_Animation5.AnimationState = ANIMATION_NOT_RUNNING;				// this animation is done
			return;
		}
		// prepare another animation
		if(m_Animation5.ucAnimationRepeatCount < 255) m_Animation5.ucAnimationRepeatCount --;
		m_Animation5.ucAnimationStageCounter = m_Animation5.ucAnimationMaxStage ;
		m_Animation5.ucAnimationPeriodCounter = m_Animation5.ucAnimationPeriod;
	}
	//-------------------------------------

	if( (m_Animation5.ucAnimationStageCounter > (m_Animation5.ucAnimationMaxStage - 12)) )
	{	// turning LEDs ON / OFF
		WS2812SetDisplay(m_Animation5.DispID, m_ucLEDSwapMatrix[(11-(m_Animation5.ucAnimationMaxStage - m_Animation5.ucAnimationStageCounter))] , 255, 0, 0, 255);
		WS2812ShowDisplay(m_Animation5.DispID,0);
	}

	if( (m_Animation5.ucAnimationStageCounter < 13) )
	{	// Setting LEDs ON / OFF
		WS2812SetDisplay(m_Animation5.DispID,m_ucLEDSwapMatrix[(11-(12-m_Animation5.ucAnimationStageCounter))],0, 0, 0, 0);
		WS2812ShowDisplay(m_Animation5.DispID,0);
	}

	m_Animation5.ucAnimationPeriodCounter = m_Animation5.ucAnimationPeriod;			// set the new timer
	m_Animation5.SetNextAnimation = ANIMATION_WAIT;									// wait for the time to elapse
	m_Animation5.ucAnimationStageCounter --;										// select the next stage of the animation
	return;
}
//----------------------------------------------------------------------------------
static void Animation5Timer1KHZ(void)
{// called from the 1KHZ ISR system timer

	if(m_Animation5.ucAnimationPeriodCounter > 1) 		m_Animation5.ucAnimationPeriodCounter --;
	else
	{
		m_Animation5.ucAnimationPeriodCounter = 0;
		m_Animation5.SetNextAnimation = SET_NEXT_ANIMATION;
	}

	return;
}
//----------------------------------------------------------------------------------
static void Animation6Init(void)
{	// called @ initialization
	m_Animation6.DispID = DISPLY_ANIMATION;
	m_Animation6.AnimationState = ANIMATION_NOT_RUNNING;
	m_Animation6.SetNextAnimation = ANIMATION_WAIT;
	m_Animation6.ucAnimationMaxStage = 30;
	m_Animation6.ucAnimationStageCounter = m_Animation6.ucAnimationMaxStage ;
	m_Animation6.ucAnimationPeriod = 50;
	m_Animation6.ucAnimationPeriodCounter = m_Animation6.ucAnimationPeriod;
	m_Animation6.ucAnimationRepeatCount = 0;

	// clear the display buffer for this animation
	WS2812EmptyDisplay(m_Animation6.DispID);

	return;
}
//----------------------------------------------------------------------------------
static void Animation6Loop(void)
{	// called from the main loop to handle the animation
	// --------------------------------------
	if(m_Animation6.SetNextAnimation == ANIMATION_WAIT)			return;		// waiting for the time elapse

	if(m_Animation6.ucAnimationStageCounter == 0)
	{
		if(m_Animation6.ucAnimationRepeatCount <= 1)
		{	// end the animation
			m_Animation6.AnimationState = ANIMATION_NOT_RUNNING;				// this animation is done
			return;
		}
		// prepare another animation
		if(m_Animation6.ucAnimationRepeatCount < 255) m_Animation6.ucAnimationRepeatCount --;
		m_Animation6.ucAnimationStageCounter = m_Animation6.ucAnimationMaxStage ;
		m_Animation6.ucAnimationPeriodCounter = m_Animation6.ucAnimationPeriod;
	}
	//-------------------------------------

	if( (m_Animation6.ucAnimationStageCounter > (m_Animation6.ucAnimationMaxStage - 12)) )
	{	// turning LEDs ON / OFF
		WS2812SetDisplay(m_Animation6.DispID, m_ucLEDSwapMatrix[(11-(m_Animation6.ucAnimationMaxStage - m_Animation6.ucAnimationStageCounter))] , 255, 0, 0, 255);
		WS2812ShowDisplay(m_Animation6.DispID,0);
	}

	if( (m_Animation6.ucAnimationStageCounter < 13) )
	{	// Setting LEDs ON / OFF
		WS2812SetDisplay(m_Animation6.DispID,m_ucLEDSwapMatrix[(11-(12-m_Animation6.ucAnimationStageCounter))],0, 0, 0, 0);
		WS2812ShowDisplay(m_Animation6.DispID,0);
	}

	m_Animation6.ucAnimationPeriodCounter = m_Animation6.ucAnimationPeriod;			// set the new timer
	m_Animation6.SetNextAnimation = ANIMATION_WAIT;									// wait for the time to elapse
	m_Animation6.ucAnimationStageCounter --;										// select the next stage of the animation
	return;
}
//----------------------------------------------------------------------------------
static void Animation6Timer1KHZ(void)
{// called from the 1KHZ ISR system timer

	if(m_Animation6.ucAnimationPeriodCounter > 1) 		m_Animation6.ucAnimationPeriodCounter --;
	else
	{
		m_Animation6.ucAnimationPeriodCounter = 0;
		m_Animation6.SetNextAnimation = SET_NEXT_ANIMATION;
	}

	return;
}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
static void LEDSwapInit(void)
{// configure the swap matrix
	m_ucLEDSwapMatrix[1 -1] = 7;
	m_ucLEDSwapMatrix[2 -1] = 6;
	m_ucLEDSwapMatrix[3 -1] = 5;
	m_ucLEDSwapMatrix[4 -1] = 4;
	m_ucLEDSwapMatrix[5 -1] = 3;
	m_ucLEDSwapMatrix[6 -1] = 2;
	m_ucLEDSwapMatrix[7 -1] = 1;
	m_ucLEDSwapMatrix[8 -1] = 12;
	m_ucLEDSwapMatrix[9 -1] = 11;
	m_ucLEDSwapMatrix[10-1] = 10;
	m_ucLEDSwapMatrix[11-1] = 9;
	m_ucLEDSwapMatrix[12-1] = 8;
	return;
}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------

static void AnimationsMainLoop(void)
{	// called on the main loop

	// Switch OFF Animation (with highest priority)
	if(m_AnimationSwitchOFF.AnimationState == ANIMATION_RUNNING)
	{
		AnimationSwitchOFFLoop();
		return;
	}

	// Animation1
	if(m_Animation1.AnimationState == ANIMATION_RUNNING)
	{
		Animation1Loop();
		return;
	}

	// Animation2
	if(m_Animation2.AnimationState == ANIMATION_RUNNING)
	{
		Animation2Loop();
		return;
	}

	// Animation3
	if(m_Animation3.AnimationState == ANIMATION_RUNNING)
	{
		Animation3Loop();
		return;
	}

	// Animation4
	if(m_Animation4.AnimationState == ANIMATION_RUNNING)
	{
		Animation4Loop();
		return;
	}

	// Animation5
	if(m_Animation5.AnimationState == ANIMATION_RUNNING)
	{
		Animation5Loop();
		return;
	}

	// Animation6
	if(m_Animation6.AnimationState == ANIMATION_RUNNING)
	{
		Animation6Loop();
		return;
	}






	// Switch ON Animation (with lowest priority)
	if(m_AnimationSwitchON.AnimationState == ANIMATION_RUNNING)
	{
		AnimationSwitchONLoop();
		return;
	}

}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//==================================================================================================================== STATIC_END



// end of the file



