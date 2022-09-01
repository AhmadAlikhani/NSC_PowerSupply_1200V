#ifndef UART_MGR_COMM
#define UART_MGR_COMM

#include "main.h"
#include "stdint.h"

typedef struct hmi_configuration_data
{
	uint8_t Enable_Sputter;
	uint32_t HMI_Current_Setpoint;
	uint32_t HMI_Voltage_Setpoint;
	uint32_t Arc_Level;
	uint32_t Quanch_Time;
}hmi_configuration_data_t;

#endif /* UART_MGR_COMM */
