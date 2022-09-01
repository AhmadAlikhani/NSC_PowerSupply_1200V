/*
 * Sputter.c
 *
 *  Created on: Aug 12, 2022
 *      Author: M.Mozafari
 */

#include "Sputter.h"
#include "uart_mgr.h"
#include "main.h"

extern hmi_configuration_data_t hmi_config_data;
unsigned int falg_Fan=0;
unsigned int Cnt_DC_OK=0, Cnt_PFC_OK=0, Cnt_PFC_OTP=0, Cnt_FB_OTP=0, Cnt_Hiccup=0, Cnt_Fan=0;
unsigned char state='S';
signed int Pout_Error, Pout_Total_Error, Pout_P, Pout_I,Pout_PID;
unsigned int flag_DC_OK=0, flag_PFC_OK=0, flag_PFC_OTP=0, flag_FB_OTP=0, flag_Hiccup=0;
uint16_t tacho2,Fan_Timer,Fan_Timer_Enable,tacho2_Backup,tacho2_Disable,Usart_Counter,Com_Failure;
uint32_t CC_Level;
uint32_t CV_Level;
uint32_t My_var;
unsigned char Inrush_OK=0;
unsigned int Inrush_Cnt=0;
float Setpoint,CC_Level_DAC;
float Setpoint_v, SP_Step_v=1;

extern uint8_t PFC_OK;
extern uint8_t DC_OK;
extern uint8_t Quanch_Time_Flage;
extern uint16_t FB_OTP;
extern uint32_t Setpoint_Limit_Current;
extern float SP_Step;
extern uint32_t Setpoint_Limit_Voltage;
extern float Power_Setpoint;
extern uint8_t Hiccup;
extern uint8_t Enable_PFC;
extern uint8_t Control_Mode;
extern uint8_t sent_data[16];
extern float CC_Level_Calibration_Offset;
extern float CC_Level_Calibration_Coefficient;
extern float CC_Level_Calibration_Offset;

extern DAC_HandleTypeDef hdac1;
extern DAC_HandleTypeDef hdac2;


/* USER CODE BEGIN Header_Sputter */
/**
* @brief Function implementing the SputterTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Sputter */
void SputterFunc(void)
{
  /* USER CODE BEGIN Sputter */
  /* Infinite loop */
  for(;;)
  {

		if ((Com_Failure==0) & (hmi_config_data.Enable_Sputter==1) & (Hiccup==0) && (FB_OTP==0))
		{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);						// Relay AC enable
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);						// Fan Enable
			//tacho2_Disable=0;

			if (tacho2_Disable==0)
			{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);				// PFC Enable
				Fan_Timer_Enable=0;
				Fan_Timer=0;

				if (DC_OK==1)// && (Setpoint>0 || Setpoint_v>0))
				{
					if (Inrush_Cnt<1000)
					{
						Inrush_Cnt++;
					}
					else
					{
						Inrush_Cnt=1000;
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);					// Inrush Relay Enable
						Inrush_OK=1;
					};

					if (Inrush_OK==1)
					{
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);						// Enable Sputter
						falg_Fan=1;
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_SET);					// Sputter LED
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET);				// Abnormal condition LED

						//------------------------------ Current Mode ------------------------------//
						if (Setpoint < Setpoint_Limit_Current && Setpoint < 4595)
							Setpoint = Setpoint + SP_Step;
						if (Setpoint >= Setpoint_Limit_Current && Setpoint > 0)
							Setpoint = Setpoint - SP_Step;
						//--------------------------------------------------------------------------//

						//------------------------------ Voltage Mode ------------------------------//
						if (Setpoint_v < Setpoint_Limit_Voltage && Setpoint_v < 4595)
							Setpoint_v = Setpoint_v + SP_Step_v;
						if (Setpoint_v >= Setpoint_Limit_Voltage && Setpoint_v > 0)
							Setpoint_v = Setpoint_v - SP_Step_v;
						//--------------------------------------------------------------------------//
					};
				}
				else  // NOT DC-OK
				{
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);				// PFC Disable
					Setpoint = 0;
					Setpoint_v = 0;
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);					// Inrush Relay Disable
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);						// Disable Sputter
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_SET);						// Abnormal condition LED
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET);					// Enable Status LED Off
					Inrush_OK = 0;
				};
			}
			else  // NOT Tacho
			{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);				// PFC Disable
				Setpoint = 0;
				Setpoint_v = 0;
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);					// Inrush Relay Disable
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);						// Disable Sputter
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_SET);						// Abnormal condition LED
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET);					// Enable Status LED Off
				Inrush_OK = 0;
			};
		} //Hiccup//
		else if ((Com_Failure==0) && (hmi_config_data.Enable_Sputter==1) && (Hiccup==1) && (FB_OTP==0)) //-- Hiccup Condition Begin --//
		{
			Setpoint = 0;
			Setpoint_v = 0;
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);					// Disable Sputter
			Quanch_Time_Flage = 1;
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_SET);					// Abnormal condition LED
		}
		//-- Hiccup Condition End --//
		else
		{
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);					// Disable Sputter
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);				// Inrush Relay Disable
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET); 					// Disable PFC
			Setpoint = 0;
			Setpoint_v = 0;
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET);				// Sputter LED
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET);				// Abnormal condition LED
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);					// Relay AC Disable
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET);				// Enable Status LED Off

			Fan_Timer_Enable=1;
			Inrush_Cnt=0;
			Inrush_OK=0;
			PFC_OK=0;
			if (Fan_Timer>60)
			{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);				// Fan Disable
				Fan_Timer_Enable=0;
				Fan_Timer=0;
			};
		};

		// ---------------------------- Read I/O Condition ----------------------------//
		// --------------------------- Quanch Time ---------------------------------//
		if (Quanch_Time_Flage==1)
		{
			HAL_Delay(hmi_config_data.Quanch_Time);
			Quanch_Time_Flage = 0;
		};

		// ---------------------------- DC-OK --------------------------------------//
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8))
		{
			Cnt_DC_OK++;
			if(Cnt_DC_OK>=1000)
			{
				DC_OK = 0;
				flag_DC_OK=0;
				Cnt_DC_OK=1000;
			};
		}
		else
		{
			DC_OK = 1;
			flag_DC_OK=1;
			Cnt_DC_OK=0;
		};

		// ---------------------------- Full Bridge Over Temprature Protection --------------------------------------//
		if(!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13))
		{
			FB_OTP = 0;
			flag_FB_OTP=0;
			Cnt_FB_OTP=0;
		}
		else
		{
			Cnt_FB_OTP++;
			if (Cnt_FB_OTP>=100)
			{
				FB_OTP = 1;
				flag_FB_OTP=1;
				Cnt_FB_OTP=100;
			};
		};

		// ---------------------------- Hiccup --------------------------------------//
		if (!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2))
		{
			Cnt_Hiccup++;
			if (Cnt_Hiccup>=10)
			{
				Hiccup = 1;
				flag_Hiccup=1;
				Cnt_Hiccup=10;
			};
			if (Control_Mode!=0)
			{
				CC_Level_DAC=0;
			  Cnt_Hiccup=0;
			}
		}
		else
		{
			Hiccup = 0;
			flag_Hiccup=0;
			Cnt_Hiccup=0;
		};
		// ----------------------------------------------------------------------------//

		//------------------- Send flags to HMI -------------------//
		sent_data[10] = flag_DC_OK + '0';
		sent_data[11] = flag_PFC_OK + '0';
		sent_data[12] = flag_PFC_OTP + '0';
		sent_data[13] = flag_FB_OTP + '0';
		sent_data[14] = flag_Hiccup + '0';
		sent_data[15] = falg_Fan + '0';

		//---------------------------------------------------------//
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t) Setpoint_v);
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (uint32_t) Setpoint);
		HAL_DAC_SetValue(&hdac2, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t) ( (hmi_config_data.Arc_Level*CC_Level_Calibration_Coefficient)+CC_Level_Calibration_Offset) );//(Arc_Level/(Current_Calibration_Coefficient*Nextion_Current_IL300_Coefficient)));

    osDelay(1);
  }
  /* USER CODE END Sputter */
}
