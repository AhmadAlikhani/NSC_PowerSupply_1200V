/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <Sputter.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t index_usart,cn2,cn3;
extern unsigned char state;
extern uint8_t sent_data[16], buffer_usart2[20], rec_D[1],cn1;
extern uint16_t tacho2,Fan_Timer,Fan_Timer_Enable,tacho2_Backup,Usart_Counter;
extern FailureStatus_t failure_status;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern DAC_HandleTypeDef hdac1;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim6;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_EXTI6_Tacho2_Pin);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */
	tacho2=tacho2+1;

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update and TIM16 interrupts.
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */
	htim1.Instance->CNT=0;
	cn1++;
	
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_8);			// MCU Status LED
	
	if (Usart_Counter==0)											// Communication failed
		failure_status.Com_Failure = 1;
	else
		failure_status.Com_Failure = 0;
	
	Usart_Counter=0;
	
	if (Fan_Timer_Enable==1)
		Fan_Timer=Fan_Timer+1;
	
	if (tacho2_Backup<50)
		failure_status.tacho2_Disable=1;
	else
		failure_status.tacho2_Disable=0;
	
	tacho2_Backup=tacho2;
	tacho2=0;

  /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXT line 26.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
	
	cn2++;
	HAL_UART_Receive_IT(&huart2, (uint8_t *) rec_D, 1);
	Usart_Counter++;

	
	switch (state)
	{
		case 'S':
			if (rec_D[0] == 0x05)
				state = 'N';
		break;
			
		case 'N':
			if (rec_D[0] == 'h')
			{
				state = 'D';
				index_usart = 0;
			}
			else
			{
				state = 'S';
			};
		break;
			
		case 'D':
			if (rec_D[0] != 0x04)
			{
				buffer_usart2[index_usart] = rec_D[0];
				index_usart++;
			}
			else if (rec_D[0] == 0x04)
			{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);						// DE_RE = 1 , ADM485 ready to send data
				HAL_UART_Transmit(&huart2, (uint8_t *) sent_data, 16,50);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);					// DE_RE = 1 , ADM485 ready to recieve data
				//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_2);												// Communication LED
				HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_8);												// Test LED

				state = 'S';
			}
		break;
	}
	
	
	
	

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global and DAC1 underrun error interrupts.
  */
void TIM6_DAC1_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC1_IRQn 0 */

  /* USER CODE END TIM6_DAC1_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  HAL_DAC_IRQHandler(&hdac1);
  /* USER CODE BEGIN TIM6_DAC1_IRQn 1 */

  /* USER CODE END TIM6_DAC1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
