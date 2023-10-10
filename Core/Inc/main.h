/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */




/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void Error_Handler2(uint16_t ui16ErrorCode);		// used to trace error after occurring

// main .c
#define ERROR_CODE_MAIN_DEBUG_INIT					100
#define ERROR_CODE_MAIN_DEBUG_RXCPLTCLBACK			101
#define ERROR_CODE_MAIN_DEBUG_SENDDATA				102

// WS2812.c
#define ERROR_CODE_WS2812_SENDWS2812DATA			200

// Wheel.c
#define ERROR_CODE_WHEEL_INIT_001					300
#define ERROR_CODE_WHEEL_INIT_002					301
#define ERROR_CODE_WHEEL_INIT_003					302
#define ERROR_CODE_WHEEL_INIT_004					303

// Interaction.c
#define ERROR_CODE_INTERACTION_INIT_001				400

#define ERROR_CODE_INTERACTION_DEBUG_001			410

// QRE
#define ERROR_CODE_QRE_INIT_001						500
#define ERROR_CODE_QRE_LOOP_002						501
#define ERROR_CODE_QRE_LOOP_003						502
#define ERROR_CODE_QRE_LOOP_004						503
#define ERROR_CODE_QRE_LOOP_005						504

#define ERROR_CODE_QRE_DEBUG_001					510


// IrTsop
#define ERROR_CODE_IRTSOP_INIT_001					601
#define ERROR_CODE_IRTSOP_INIT_002					602


// Audio
#define ERROR_CODE_AUDIO_INIT_001					700


// I2CDevices
#define ERROR_CODE_I2CDEVICES_DEBUG_001				820
#define ERROR_CODE_I2CDEVICES_DEBUG_002				821

// I2C TSL25911
#define ERROR_CODE_I2CDEVICES_DEBUG_040				840
#define ERROR_CODE_I2CDEVICES_DEBUG_041				841
#define ERROR_CODE_I2CDEVICES_DEBUG_042				842
#define ERROR_CODE_I2CDEVICES_DEBUG_043				843
#define ERROR_CODE_I2CDEVICES_DEBUG_044				844
#define ERROR_CODE_I2CDEVICES_DEBUG_045				845



/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TSOP_Pin GPIO_PIN_3
#define TSOP_GPIO_Port GPIOE
#define TSOP_EXTI_IRQn EXTI3_IRQn
#define PW_OFF_Pin GPIO_PIN_4
#define PW_OFF_GPIO_Port GPIOE
#define RPWM_Pin GPIO_PIN_5
#define RPWM_GPIO_Port GPIOE
#define LPWM_Pin GPIO_PIN_6
#define LPWM_GPIO_Port GPIOE
#define DebugOut_Pin GPIO_PIN_13
#define DebugOut_GPIO_Port GPIOC
#define IR_EN_3_Pin GPIO_PIN_15
#define IR_EN_3_GPIO_Port GPIOC
#define ADC2_QA5_Pin GPIO_PIN_0
#define ADC2_QA5_GPIO_Port GPIOC
#define ADC_CH_5_Pin GPIO_PIN_1
#define ADC_CH_5_GPIO_Port GPIOC
#define ADC_CH_4_Pin GPIO_PIN_2
#define ADC_CH_4_GPIO_Port GPIOC
#define ADC_CH_3_Pin GPIO_PIN_3
#define ADC_CH_3_GPIO_Port GPIOC
#define ADC2_QA6_Pin GPIO_PIN_0
#define ADC2_QA6_GPIO_Port GPIOA
#define ADC2_QA7_Pin GPIO_PIN_1
#define ADC2_QA7_GPIO_Port GPIOA
#define ADC2_QA8_Pin GPIO_PIN_2
#define ADC2_QA8_GPIO_Port GPIOA
#define ADC_CH_1_Pin GPIO_PIN_4
#define ADC_CH_1_GPIO_Port GPIOA
#define ADC_CH_2_Pin GPIO_PIN_5
#define ADC_CH_2_GPIO_Port GPIOA
#define ADC2_QA4_Pin GPIO_PIN_7
#define ADC2_QA4_GPIO_Port GPIOA
#define ADC2_QA2_Pin GPIO_PIN_4
#define ADC2_QA2_GPIO_Port GPIOC
#define ADC2_QA1_Pin GPIO_PIN_5
#define ADC2_QA1_GPIO_Port GPIOC
#define ADC2_QA3_Pin GPIO_PIN_0
#define ADC2_QA3_GPIO_Port GPIOB
#define ADC_CH_0_Pin GPIO_PIN_1
#define ADC_CH_0_GPIO_Port GPIOB
#define INT_CH_2_Pin GPIO_PIN_2
#define INT_CH_2_GPIO_Port GPIOB
#define INT_CH_2_EXTI_IRQn EXTI2_IRQn
#define TX3_XGP_Pin GPIO_PIN_10
#define TX3_XGP_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_14
#define BUZZER_GPIO_Port GPIOB
#define INT_CH_1_Pin GPIO_PIN_8
#define INT_CH_1_GPIO_Port GPIOD
#define INT_CH_1_EXTI_IRQn EXTI9_5_IRQn
#define RX3_XSHUT_Pin GPIO_PIN_9
#define RX3_XSHUT_GPIO_Port GPIOD
#define RIN1_Pin GPIO_PIN_10
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
#define IR_TSOP_Pin GPIO_PIN_8
#define IR_TSOP_GPIO_Port GPIOC
#define INT_CH_0_Pin GPIO_PIN_9
#define INT_CH_0_GPIO_Port GPIOA
#define INT_CH_0_EXTI_IRQn EXTI9_5_IRQn
#define INT_CH_5_Pin GPIO_PIN_10
#define INT_CH_5_GPIO_Port GPIOA
#define INT_CH_5_EXTI_IRQn EXTI15_10_IRQn
#define IR_EN_0_Pin GPIO_PIN_12
#define IR_EN_0_GPIO_Port GPIOA
#define IR_PWM_Pin GPIO_PIN_15
#define IR_PWM_GPIO_Port GPIOA
#define IR_EN_1_Pin GPIO_PIN_10
#define IR_EN_1_GPIO_Port GPIOC
#define IR_EN_2_Pin GPIO_PIN_11
#define IR_EN_2_GPIO_Port GPIOC
#define Q0_Pin GPIO_PIN_12
#define Q0_GPIO_Port GPIOC
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
#define IR_EN_5_Pin GPIO_PIN_8
#define IR_EN_5_GPIO_Port GPIOB
#define IR_EN_4_Pin GPIO_PIN_9
#define IR_EN_4_GPIO_Port GPIOB
#define INT_CH_4_Pin GPIO_PIN_0
#define INT_CH_4_GPIO_Port GPIOE
#define INT_CH_4_EXTI_IRQn EXTI0_IRQn
#define INT_CH_3_Pin GPIO_PIN_1
#define INT_CH_3_GPIO_Port GPIOE
#define INT_CH_3_EXTI_IRQn EXTI1_IRQn
/* USER CODE BEGIN Private defines */







//--------------------------------------------------
// DEBUG
void DebugInit(void);
//--------------------------------------------------

// RX
#define DEBUG_RX_BUFFER_SIZE	250
//--------------------------------------------------


// TX
#define DEBUG_TX_BUSY				100
#define DEBUG_TX_DMA_ERROR			120
#define DEBUG_TX_DATA_ERROR			130
#define DEBUG_TX_OK					0

#define DEBUG_TX_BUFFER_SIZE	250
uint8_t DebugSendData(uint8_t *ucDataBuffer, uint8_t ucDataLength);
//--------------------------------------------------




/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
