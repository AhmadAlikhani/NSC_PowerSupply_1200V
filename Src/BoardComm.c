/*
 * BoardComm.c
 *
 *  Created on: Aug 12, 2022
 *      Author: M.Mozafari
 */

#include "BoardComm.h"
#include "uart_mgr.h"

extern uint8_t buffer_usart2[24];
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
		Setpoint_Limit_Current = (hmi_config_data.HMI_Current_Setpoint > 0) ? ((uint32_t)((hmi_config_data.HMI_Current_Setpoint))) : (0);
		Setpoint_Limit_Voltage = (hmi_config_data.HMI_Voltage_Setpoint > 0) ? ((uint32_t)((hmi_config_data.HMI_Voltage_Setpoint))) : (0);

    osDelay(1);
  }
}
