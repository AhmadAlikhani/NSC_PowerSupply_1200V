/*
 * ADC_Calculation.c
 *
 *  Created on: Aug 12, 2022
 *      Author: M.Mozafari
 */

#include "ADC_Calculation.h"
#include "stdint.h"

uint8_t sent_data[16]={0x06,0x68,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30};
uint32_t ADC0_Buffer_Sum;
uint32_t Averaged_ADC0_Buffer;
uint32_t Moving_Average_Buffer_Current;
uint8_t  Avg_Cnt1, Avg_Cnt2, Avg_Cnt3, rec_D[1],cn1;
uint32_t ADC1_Buffer_Sum, Averaged_ADC1_Buffer, Moving_Average_Buffer_Voltage;
uint32_t Pout=0;
uint32_t Power_Buffer, Averaged_Power, Moving_Average_Power, Output_Power;
float Iout_NotCalibrated;
float Iout;
float Vout_NotCalibrated,Vout;
float SP_Step=1;
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
		for (uint8_t i=0; i < 50 ; i++)
			ADC0_Buffer_Sum = ADC0_Buffer_Sum + ADC_Buffer[1];
		Averaged_ADC0_Buffer = ADC0_Buffer_Sum/50;
		// Moving average filter //
		Moving_Average_Buffer_Current = Moving_Average_Buffer_Current + Averaged_ADC0_Buffer;
		if (Avg_Cnt1==10)
		{
			Iout_NotCalibrated = (uint32_t) ( ((Iout_NotCalibrated*9)+(Moving_Average_Buffer_Current/10))/10 );
			Iout = (uint32_t)(Iout_NotCalibrated);
			Moving_Average_Buffer_Current = 0;
			// Offset Calibration //
			if (Iout- 0 >0)
				Iout = Iout +  0 ;
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
		for (uint8_t j = 0; j < 50; j++)
			ADC1_Buffer_Sum = ADC1_Buffer_Sum + ADC_Buffer[0];
		Averaged_ADC1_Buffer = ADC1_Buffer_Sum/50;
		// Moving average filter //
		Moving_Average_Buffer_Voltage = Moving_Average_Buffer_Voltage + Averaged_ADC1_Buffer;
		if (Avg_Cnt2==10)
		{
			Vout_NotCalibrated = (uint32_t) ( ((Vout_NotCalibrated*9)+(Moving_Average_Buffer_Voltage/10))/10 );
			Vout = (uint32_t)(Vout_NotCalibrated* 1 * 1 );
			Moving_Average_Buffer_Voltage = 0;
			// Offset Calibration //
			if (Vout- 0 >0)
				Vout = Vout +  0 ;
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
		for (uint32_t i = 0 ; i < 20 ; i++)
			Power_Buffer = Power_Buffer + Pout;
		Averaged_Power = Power_Buffer/20;
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
