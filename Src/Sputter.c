/*
 * Sputter.c
 *
 *  Created on: Aug 12, 2022
 *      Author: M.Mozafari
 */

#include "Sputter.h"
#include "main.h"
extern uint8_t Enable_Sputter;
unsigned int falg_Fan=0;
unsigned int Cnt_AC_OK=0, Cnt_PFC_OK=0, Cnt_PFC_OTP=0, Cnt_FB_OTP=0, Cnt_Hiccup=0, Cnt_Fan=0;
unsigned char state='S';
signed int Pout_Error, Pout_Total_Error, Pout_P, Pout_I,Pout_PID;
unsigned int flag_AC_OK=0, flag_PFC_OK=0, flag_PFC_OTP=0, flag_FB_OTP=0, flag_Hiccup=0;
uint32_t CC_Level;
uint32_t CV_Level;
uint32_t My_var;
float Setpoint,CC_Level_DAC;
float Setpoint_v, SP_Step_v=0.1;


extern uint8_t AC_OK;
extern uint8_t Quanch_Time_Flage;
extern uint32_t Quanch_Time;
extern uint16_t FB_OTP;
extern uint32_t Setpoint_Limit_Current;
extern float SP_Step;
extern uint32_t Setpoint_Limit_Voltage;
extern float Power_Setpoint;
extern uint8_t Hiccup;
extern uint32_t HMI_Current_Setpoint;
extern uint8_t Enable_PFC;
extern uint8_t Control_Mode;
extern uint8_t sent_data[16];
extern uint32_t Arc_Level;
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

	  if (Enable_Sputter==1)
	  			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);						// Relay AC enable
	  		else
	  			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);					// Relay AC Disable

	  		if ((Enable_Sputter==0 && falg_Fan==1)|| (AC_OK==0 && falg_Fan==1))	//-- Manual disable Condition --//
	  		{
	  			Cnt_Fan++;
	  			if (Cnt_Fan>400000)
	  			{
	  				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);						// Fan Disable
	  				Cnt_Fan=0;
	  				falg_Fan=0;
	  			};
	  		};

	  		//--------------------- Sputter -----------------------//
	  		if (Quanch_Time_Flage==1)
	  		{
	  			osDelay(Quanch_Time);
	  			Quanch_Time_Flage = 0;
	  		};
	  		if (AC_OK==0)
	  		{
	  			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);																// Disable PFC
	  			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);																// Disable Sputter
	  		}
	  		else
	  		{
	  			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);															// Enable PFC
	  		};

	  		if (Enable_Sputter==1)// && PFC_OK==1)																									//-- Condition for Relay Enable --//
	  			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
	  		else
	  			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);

	  		//  AC_OK=1; // for Test Kasra
	  		// PFC_OK=1; //for Test Kasra
	  		if (Enable_Sputter==1 && Hiccup==1 &&  AC_OK==1 && FB_OTP==0 )	//-- All Condition OK Begin --//
	  		{
	  			//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET); // Enable PFC
	  			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);	// Enable Sputter
	  			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);	// Fan Enable
	  			falg_Fan=1;
	  			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_SET);							// Sputter LED
	  			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET);					// Abnormal condition LED

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


	  		}
	  		else if (FB_OTP==1 || AC_OK==0)
	  		{
	  			HMI_Current_Setpoint=0;
	  			Pout_Error=0;
	  			Pout_Total_Error =0;
	  			Pout_PID =0;
	  			Setpoint = 0;
	  			Setpoint_v = 0;
	  			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);	// Disable Sputter
	  			//	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET); // Disable PFC
	  			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_SET);					// Abnormal condition LED
	  		}
	  		//--------------------------------------------------------------------------//
	  		//-- All Condition OK End --//
	  		else if (Enable_Sputter==1 && Hiccup==0 && FB_OTP==0) //-- Hiccup Condition Begin --//
	  		{
	  			Setpoint = 0;
	  			Setpoint_v = 0;
	  			CC_Level_DAC=0;
	  			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);	// Disable Sputter
	  			Quanch_Time_Flage = 1;
	  			HMI_Current_Setpoint=0;
	  			Pout_Error=0;
	  			Pout_Total_Error =0;
	  			Pout_PID =0;
	  			Power_Setpoint=0;
	  			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_SET);					// Abnormal condition LED
	  		}
	  		//-- Hiccup Condition End --//

	  		else if (Enable_Sputter==0) 	//-- Disable Sputtering --//
	  		{
	  			//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET); // Disable PFC
	  			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);	// Disable Sputter
	  			Enable_PFC = 0;
	  			Setpoint = 0;
	  			Setpoint_v = 0;
	  			CC_Level_DAC=0;
	  			Power_Setpoint=0;
	  			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET);							// Sputter LED
	  			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET);						// Abnormal condition LED
	  		};

	  		if (Enable_Sputter==0) //-- Manual disable Condition --//
	  		{
	  			//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);	// Fan Disable
	  			CC_Level_DAC=0;
	  			Setpoint = 0;
	  			Setpoint_v = 0;
	  			Pout_Error = 0;
	  			Pout_Total_Error = 0;
	  			HMI_Current_Setpoint=0;
	  			Pout_Error=0;
	  			Pout_Total_Error =0;
	  			Pout_PID =0;
	  			Power_Setpoint=0;
	  			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET);							// Sputter LED
	  			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET);						// Abnormal condition LED
	  		};

	  		// ---------------------------- Read I/O Condition ----------------------------//

	  		// ---------------------------- AC-OK --------------------------------------//
	  		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8))
	  		{
	  			Cnt_AC_OK++;
	  			if(Cnt_AC_OK>=100)
	  			{
	  				AC_OK = 0;
	  				flag_AC_OK=0;
	  				Cnt_AC_OK=100;
	  			};
	  		}
	  		else
	  		{
	  			AC_OK = 1;
	  			flag_AC_OK=1;
	  			Cnt_AC_OK=0;
	  		};

	  		// ---------------------------- Full Bridge Over Temprature Protection --------------------------------------//
	  		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13))
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
	  		if (HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2))
	  		{
	  			Cnt_Hiccup++;
	  			if (Cnt_Hiccup>=10)
	  			{
	  				Hiccup = 1;
	  				flag_Hiccup=1;
	  				Cnt_Hiccup=10;
	  			};
	  			if (Control_Mode != 0)
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
	  		sent_data[10] = flag_AC_OK + '0';
	  		sent_data[11] = flag_PFC_OK + '0';
	  		sent_data[12] = flag_PFC_OTP + '0';
	  		sent_data[13] = flag_FB_OTP + '0';
	  		sent_data[14] = flag_Hiccup + '0';
	  		sent_data[15] = falg_Fan + '0';

	  		//---------------------------------------------------------//
	      /* USER CODE END WHILE */

	      /* USER CODE BEGIN 3 */
	  		if (Enable_Sputter==1)
	  		{
	  			if (CC_Level_DAC < CC_Level && CC_Level_DAC < 4095)
	  				CC_Level_DAC=CC_Level_DAC+0.1;
	  			if (CC_Level_DAC > CC_Level && CC_Level_DAC > 0)
	  				CC_Level_DAC--;
	  		}
	  		else
	  			CC_Level_DAC = 0;

	  		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t) Setpoint_v);
	  		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (uint32_t) Setpoint);
	  		HAL_DAC_SetValue(&hdac2, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t) ( (Arc_Level*CC_Level_Calibration_Coefficient)+CC_Level_Calibration_Offset) );//(Arc_Level/(Current_Calibration_Coefficient*Nextion_Current_IL300_Coefficient)));



    osDelay(1);
  }
  /* USER CODE END Sputter */
}
