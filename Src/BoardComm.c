/*
 * BoardComm.c
 *
 *  Created on: Aug 12, 2022
 *      Author: M.Mozafari
 */

#include "BoardComm.h"

uint8_t buffer_usart[24];
uint8_t Control_Mode;
uint8_t Enable_Sputter;
uint32_t HMI_Current_Setpoint=0;
uint32_t HMI_Voltage_Setpoint=0;
uint32_t Arc_Level;
uint32_t Quanch_Time;
uint32_t Setpoint_Limit_Current;
uint32_t Setpoint_Limit_Voltage;
float Setpoint_Limit_Offset_C1 = 0;//0.25;//0.4983;//0.2611;
float Setpoint_Limit_Coefficient_C =1;//1.0286;//1.329;
float Setpoint_Limit_Offset_V1 = 0;//1.6476;//0.5773;
float Setpoint_Limit_Offset_V2 = 0;
float Setpoint_Limit_Coefficient_V = 1;//0.6258;//0.6243;//0.5626; //DAC-SP=(Nextion-SP*0.5626P-1.5773)

/* USER CODE BEGIN Header_BoardComm */
/**
* @brief Function implementing the BoardCommTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BoardComm */
void BoardCommFunc(void)
{
  /* USER CODE BEGIN BoardComm */
  /* Infinite loop */
  for(;;)
  {

	  //--------------------- Data Gathering ---------------------//
		Enable_Sputter		=	(buffer_usart[0]-48)	;
		HMI_Current_Setpoint=	(buffer_usart[1]-48)*1000 + (buffer_usart[2]-48)*100 +(buffer_usart[3]-48)*10 +(buffer_usart[4]-48)*1	;
		HMI_Voltage_Setpoint=	(buffer_usart[5]-48)*1000 + (buffer_usart[6]-48)*100 +(buffer_usart[7]-48)*10 +(buffer_usart[8]-48)*1	;
		Arc_Level			=	(buffer_usart[9]-48)*1000 + (buffer_usart[10]-48)*100 +(buffer_usart[11]-48)*10 +(buffer_usart[12]-48)*1	;
		Quanch_Time			=	(((buffer_usart[13]-48)*1000 + (buffer_usart[14]-48)*100 +(buffer_usart[15]-48)*10 +(buffer_usart[16]-48)*1	) < 5000 ) ? ((buffer_usart[13]-48)*1000 + (buffer_usart[14]-48)*100 +(buffer_usart[15]-48)*10 +(buffer_usart[16]-48)*1) : Quanch_Time;


		//---------- Calcultaion of Setpoint Limit ------------//
		Setpoint_Limit_Current = (uint32_t)((HMI_Current_Setpoint*Setpoint_Limit_Coefficient_C) - Setpoint_Limit_Offset_C1);
		Setpoint_Limit_Voltage = (uint32_t)((HMI_Voltage_Setpoint*Setpoint_Limit_Coefficient_V) + Setpoint_Limit_Offset_V1);
		//-----------------------------------------------------//
		//----------------------------------------------------------//

    osDelay(1);
  }
  /* USER CODE END BoardComm */
}
