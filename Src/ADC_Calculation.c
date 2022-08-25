/*
 * ADC_Calculation.c
 *
 *  Created on: Aug 12, 2022
 *      Author: M.Mozafari
 */

#include "ADC_Calculation.h"
#include "stdint.h"

uint8_t sent_data[16]={0x06,0x68,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30};
uint32_t Nextion_Setpoint;
uint32_t ADC0_Buffer_Sum;
uint32_t Averaged_ADC0_Buffer;
uint32_t Moving_Average_Buffer_Current;
uint8_t ii, jj, kk, Avg_Cnt1, Avg_Cnt2, Avg_Cnt3, Enable_PFC, Hiccup, PFC_OK, AC_OK, TSW_PFC_Fan, PFC_OTP, FB_OTP, Quanch_Time_Flage, rec_D[1];
uint32_t ADC1_Buffer_Sum, Averaged_ADC1_Buffer, Moving_Average_Buffer_Voltage;
uint32_t Pout=0;
uint32_t Power_Buffer, Averaged_Power, Moving_Average_Power, Output_Power;
float Iout_NotCalibrated;
float Nextion_Current_IL300_Coefficient = 1;
float Iout;
float Current_Calibration_Coefficient = 0.28;//0.2086;
float Current_Calibration_Offset = 2.0234;//1.1862;
float Setpoint_Limit_Offset_C2 = 0;
float Vout_NotCalibrated,Vout;
float Nextion_Voltage_IL300_Coefficient = 1;
float Voltage_Calibration_Coefficient = 0.4607;//0.4941;  //Volatge Read=(0.4941*ADC+4.5743)
float Voltage_Calibration_Offset = 1.4988;//0.4988;//4.5743;
float SP_Step=0.1;
float kasra=0.1;
float CC_Level_Calibration_Coefficient = 1;//2.4363;
float CC_Level_Calibration_Offset = 0;//3.9607;
float Power_Setpoint;
float kP=0.1, kI=0.0001;
char Read_Current[5], Read_Voltage[5];

extern uint32_t ADC_Buffer[5];

/* USER CODE BEGIN Header_ADC_Caculation */
/**
* @brief Function implementing the ADC_Caculation_ thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ADC_Caculation */
void ADC_CaculationFunc(void)
{
  /* USER CODE BEGIN ADC_Caculation */
  /* Infinite loop */
  for(;;)
  {

		//--------------------- Current ADC ---------------------//
		ADC0_Buffer_Sum = 0;
		for (ii=0;ii<50;ii++)
			ADC0_Buffer_Sum = ADC0_Buffer_Sum + ADC_Buffer[2];
		Averaged_ADC0_Buffer = ADC0_Buffer_Sum/50;
		// Moving average filter //
		Moving_Average_Buffer_Current = Moving_Average_Buffer_Current + Averaged_ADC0_Buffer;
		if (Avg_Cnt1==10)
		{
			Iout_NotCalibrated = (uint32_t) ( ((Iout_NotCalibrated*9)+(Moving_Average_Buffer_Current/10))/10 );
			Iout = (uint32_t)(Iout_NotCalibrated*Current_Calibration_Coefficient*Nextion_Current_IL300_Coefficient);
			Moving_Average_Buffer_Current = 0;
			// Offset Calibration //
			if (Iout-Current_Calibration_Offset>0)
				Iout = Iout + Current_Calibration_Offset;
			else
				Iout = 0;
			////////////////////////
			Avg_Cnt1 = 0;
		};
		Avg_Cnt1++;
		i2s(Iout,Read_Current);
		sent_data[2]=Read_Current[0];
		sent_data[3]=Read_Current[1];
		sent_data[4]=Read_Current[2];
		sent_data[5]=Read_Current[3];
		//-------------------------------------------------------//
		//--------------------- Voltage ADC ---------------------//
		ADC1_Buffer_Sum = 0;
		for (jj=0;jj<50;jj++)
			ADC1_Buffer_Sum = ADC1_Buffer_Sum + ADC_Buffer[1];
		Averaged_ADC1_Buffer = ADC1_Buffer_Sum/50;
		// Moving average filter //
		Moving_Average_Buffer_Voltage = Moving_Average_Buffer_Voltage + Averaged_ADC1_Buffer;
		if (Avg_Cnt2==10)
		{
			Vout_NotCalibrated = (uint32_t) ( ((Vout_NotCalibrated*9)+(Moving_Average_Buffer_Voltage/10))/10 );
			Vout = (uint32_t)(Vout_NotCalibrated*Voltage_Calibration_Coefficient*Nextion_Voltage_IL300_Coefficient);
			Moving_Average_Buffer_Voltage = 0;
			// Offset Calibration //
			if (Vout-Voltage_Calibration_Offset>0)
				Vout = Vout + Voltage_Calibration_Offset;
			else
				Vout = 0;
			////////////////////////
			Avg_Cnt2 = 0;
		};
		Avg_Cnt2++;
		i2s(Vout,Read_Voltage);
		sent_data[6]=Read_Voltage[0];
		sent_data[7]=Read_Voltage[1];
		sent_data[8]=Read_Voltage[2];
		sent_data[9]=Read_Voltage[3];
		//-------------------------------------------------------//

		//--------------------- Power ADC ---------------------//
		Pout=Vout*Iout/1000;
		//------ Power Filter ------//
		Power_Buffer = 0;
		for (kk=0;kk<50;kk++)
			Power_Buffer = Power_Buffer + Pout;
		Averaged_Power = Power_Buffer/50;
		// Moving average filter //
		Moving_Average_Power = Moving_Average_Power + Averaged_Power;
		if (Avg_Cnt3==10)
		{
			Output_Power = (uint32_t) ( ((Output_Power*9)+(Moving_Average_Power/10))/10 );
			Moving_Average_Power = 0;
			Avg_Cnt3 = 0;
		};
		Avg_Cnt3++;
		//--------------------------//
		//-------------------------------------------------------//


    osDelay(1);
  }

  /* USER CODE END ADC_Caculation */
}


void i2s(unsigned long int num,char str[5]){
    str[0] = (num/1000)%10 + '0';
    str[1] = (num/100)%10 + '0';
    str[2] = (num/10)%10 + '0';
    str[3] = (num/1)%10 + '0';
    str[4] = '0';
}
