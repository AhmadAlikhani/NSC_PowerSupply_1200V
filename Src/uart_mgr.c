/*
 * uart_mgr.c
 *
 *  Created on: Aug 25, 2022
 *      Author: M.Mozafari
 */

#include "uart_mgr.h"

#define MAX_ENABLE_SPUTTER_VALUE 			4095
#define MAX_HMI_CURRENT_SETPOINT_VALUE		4095
#define MAX_HMI_VOLTAGE_SETPOINT_VALUE		4095
#define MAX_ARC_LEVEL_VALUE					4095
#define MAX_QUANCH_TIME_VALUE				5001

uint8_t buffer_usart2[24];
hmi_configuration_data_t * temporary_data;
extern UART_HandleTypeDef huart2;

void uart_receiver(UART_HandleTypeDef uart_index, void* outgoing_data)
{
	if(uart_index.Instance == huart2.Instance)
	{
		//--------------------- Data Gathering ---------------------//
		//store incomming data if they are greater than 0 and lower than max values
		temporary_data = (hmi_configuration_data_t*) outgoing_data;
		temporary_data->Enable_Sputter		=	((buffer_usart2[0]-48) < MAX_ENABLE_SPUTTER_VALUE && (buffer_usart2[0]-48) >= 0) ? ((buffer_usart2[0]-48)) : (temporary_data->Enable_Sputter) ;
		temporary_data->HMI_Current_Setpoint=	(((buffer_usart2[1]-48)*1000 + (buffer_usart2[2]-48)*100 +(buffer_usart2[3]-48)*10 +(buffer_usart2[4]-48)*1) < MAX_HMI_CURRENT_SETPOINT_VALUE && ((buffer_usart2[1]-48)*1000 + (buffer_usart2[2]-48)*100 +(buffer_usart2[3]-48)*10 +(buffer_usart2[4]-48)*1) >= 0) ? (((buffer_usart2[1]-48)*1000 + (buffer_usart2[2]-48)*100 +(buffer_usart2[3]-48)*10 +(buffer_usart2[4]-48)*1)) : (temporary_data->HMI_Current_Setpoint);
		temporary_data->HMI_Voltage_Setpoint=	(((buffer_usart2[5]-48)*1000 + (buffer_usart2[6]-48)*100 +(buffer_usart2[7]-48)*10 +(buffer_usart2[8]-48)*1) < MAX_HMI_VOLTAGE_SETPOINT_VALUE && ((buffer_usart2[5]-48)*1000 + (buffer_usart2[6]-48)*100 +(buffer_usart2[7]-48)*10 +(buffer_usart2[8]-48)*1) >= 0) ? (((buffer_usart2[5]-48)*1000 + (buffer_usart2[6]-48)*100 +(buffer_usart2[7]-48)*10 +(buffer_usart2[8]-48)*1)) : (temporary_data->HMI_Voltage_Setpoint);
		temporary_data->Arc_Level			=	(((buffer_usart2[9]-48)*1000 + (buffer_usart2[10]-48)*100 +(buffer_usart2[11]-48)*10 +(buffer_usart2[12]-48)*1) < MAX_ARC_LEVEL_VALUE && ((buffer_usart2[9]-48)*1000 + (buffer_usart2[10]-48)*100 +(buffer_usart2[11]-48)*10 +(buffer_usart2[12]-48)*1) >= 0) ? (((buffer_usart2[9]-48)*1000 + (buffer_usart2[10]-48)*100 +(buffer_usart2[11]-48)*10 +(buffer_usart2[12]-48)*1)) : (temporary_data->Arc_Level);
		temporary_data->Quanch_Time			=	(((buffer_usart2[13]-48)*1000 + (buffer_usart2[14]-48)*100 +(buffer_usart2[15]-48)*10 +(buffer_usart2[16]-48)*1	) < MAX_QUANCH_TIME_VALUE && ((buffer_usart2[13]-48)*1000 + (buffer_usart2[14]-48)*100 +(buffer_usart2[15]-48)*10 +(buffer_usart2[16]-48)*1	) >= 0) ? ((buffer_usart2[13]-48)*1000 + (buffer_usart2[14]-48)*100 +(buffer_usart2[15]-48)*10 +(buffer_usart2[16]-48)*1) : (temporary_data->Quanch_Time);
	}
}
