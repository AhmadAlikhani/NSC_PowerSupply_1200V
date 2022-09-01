/*
 * BoardComm.c
 *
 *  Created on: Aug 12, 2022
 *      Author: M.Mozafari
 */

#include "BoardComm.h"
#include "uart_mgr.h"

extern uint8_t buffer_usart2[24];
uint8_t Control_Mode;
float Setpoint_Limit_Offset_C1 = 0;//0.25;//0.4983;//0.2611;
float Setpoint_Limit_Coefficient_C =1;//1.0286;//1.329;
float Setpoint_Limit_Offset_V1 = 0;//1.6476;//0.5773;
float Setpoint_Limit_Offset_V2 = 0;
float Setpoint_Limit_Coefficient_V = 1;//0.6258;//0.6243;//0.5626; //DAC-SP=(Nextion-SP*0.5626P-1.5773)
uint32_t Setpoint_Limit_Current;
uint32_t Setpoint_Limit_Voltage;
hmi_configuration_data_t hmi_config_data;

extern UART_HandleTypeDef huart2;


void BoardCommFunc(void)
{
	memset(&hmi_config_data, 0x0, sizeof(hmi_config_data));

  /* Infinite loop */
  for(;;)
  {

	  	uart_receiver(huart2, (void*)&hmi_config_data);

		//---------- Calcultaion of Setpoint Limit ------------//
		Setpoint_Limit_Current = (hmi_config_data.HMI_Current_Setpoint > 0) ? ((uint32_t)((hmi_config_data.HMI_Current_Setpoint * Setpoint_Limit_Coefficient_C) - Setpoint_Limit_Offset_C1)) : (0);
		Setpoint_Limit_Voltage = (hmi_config_data.HMI_Voltage_Setpoint > 0) ? ((uint32_t)((hmi_config_data.HMI_Voltage_Setpoint * Setpoint_Limit_Coefficient_V) + Setpoint_Limit_Offset_V1)) : (0);

    osDelay(1);
  }
}
