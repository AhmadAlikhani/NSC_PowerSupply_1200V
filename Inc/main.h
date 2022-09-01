/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f3xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GPIO_INPUT_Hiccup_Pin GPIO_PIN_2
#define GPIO_INPUT_Hiccup_GPIO_Port GPIOC
#define ADC1_IN2_VOUT_MCU_Pin GPIO_PIN_1
#define ADC1_IN2_VOUT_MCU_GPIO_Port GPIOA
#define ADC1_IN3_IOUT_MCU_Pin GPIO_PIN_2
#define ADC1_IN3_IOUT_MCU_GPIO_Port GPIOA
#define DAC1_OUT1_VLimit_Pin GPIO_PIN_4
#define DAC1_OUT1_VLimit_GPIO_Port GPIOA
#define DAC1_OUT2_CLimit_Pin GPIO_PIN_5
#define DAC1_OUT2_CLimit_GPIO_Port GPIOA
#define DAC2_OUT2_ARC_Pin GPIO_PIN_6
#define DAC2_OUT2_ARC_GPIO_Port GPIOA
#define GPIO_OUTPUT_Enable_MCU_Pin GPIO_PIN_7
#define GPIO_OUTPUT_Enable_MCU_GPIO_Port GPIOA
#define GPIO_OUTPUT_MCU_EN1_Pin GPIO_PIN_0
#define GPIO_OUTPUT_MCU_EN1_GPIO_Port GPIOB
#define GPIO_OUTPUT_MCU_LED1_Pin GPIO_PIN_1
#define GPIO_OUTPUT_MCU_LED1_GPIO_Port GPIOB
#define GPIO_OUTPUT_MCU_LED2_Pin GPIO_PIN_2
#define GPIO_OUTPUT_MCU_LED2_GPIO_Port GPIOB
#define GPIO_OUTPUT_MCU_LED3_Pin GPIO_PIN_10
#define GPIO_OUTPUT_MCU_LED3_GPIO_Port GPIOB
#define GPIO_OUTPUT_MCU_LED4_Pin GPIO_PIN_11
#define GPIO_OUTPUT_MCU_LED4_GPIO_Port GPIOB
#define GPIO_INPUT_MCU_HeatSinkTemp_Pin GPIO_PIN_13
#define GPIO_INPUT_MCU_HeatSinkTemp_GPIO_Port GPIOB
#define GPIO_OUTPUT_Relay_Inrush_Pin GPIO_PIN_14
#define GPIO_OUTPUT_Relay_Inrush_GPIO_Port GPIOB
#define GPIO_OUTPUT_PFC_EN_Pin GPIO_PIN_15
#define GPIO_OUTPUT_PFC_EN_GPIO_Port GPIOB
#define GPIO_EXTI6_Tacho2_Pin GPIO_PIN_6
#define GPIO_EXTI6_Tacho2_GPIO_Port GPIOC
#define GPIO_EXTI6_Tacho2_EXTI_IRQn EXTI9_5_IRQn
#define GPIO_OUTPUT_Relay_AC_Pin GPIO_PIN_7
#define GPIO_OUTPUT_Relay_AC_GPIO_Port GPIOC
#define GPIO_INPUT_MCU_DCLINK_OK_Pin GPIO_PIN_8
#define GPIO_INPUT_MCU_DCLINK_OK_GPIO_Port GPIOA
#define GPIO_OUTPUT_Fan_PWM2_Pin GPIO_PIN_10
#define GPIO_OUTPUT_Fan_PWM2_GPIO_Port GPIOA
#define GPIO_OUTPUT_DE_RE_Pin GPIO_PIN_5
#define GPIO_OUTPUT_DE_RE_GPIO_Port GPIOB
#define GPIO_OUTPUT_MCU_FAN_Pin GPIO_PIN_8
#define GPIO_OUTPUT_MCU_FAN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
