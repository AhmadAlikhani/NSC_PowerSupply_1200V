/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "stdint.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CRC_HandleTypeDef hcrc;

DAC_HandleTypeDef hdac1;
DAC_HandleTypeDef hdac2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char Read_Current[5], Read_Voltage[5];
unsigned char state='S';
signed int Pout_Error, Pout_Total_Error, Pout_P, Pout_I,Pout_PID;
unsigned int Cnt_AC_OK=0, Cnt_PFC_OK=0, Cnt_PFC_OTP=0, Cnt_FB_OTP=0, Cnt_Hiccup=0, Cnt_Fan=0;
unsigned int flag_AC_OK=0, flag_PFC_OK=0, flag_PFC_OTP=0, flag_FB_OTP=0, flag_Hiccup=0, falg_Fan=0;
uint8_t sent_data[16]={0x06,0x68,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30},buffer_usart[24],Control_Mode, Enable_Sputter;
uint8_t ii, jj, kk, Avg_Cnt1, Avg_Cnt2, Avg_Cnt3, Enable_PFC, Hiccup, PFC_OK, AC_OK, TSW_PFC_Fan, PFC_OTP, FB_OTP, Quanch_Time_Flage, rec_D[1];
uint32_t Nextion_Setpoint,ADC0_Buffer_Sum,Averaged_ADC0_Buffer,Moving_Average_Buffer_Current,Setpoint_Limit_Current,ADC_Calibration_Factor;
uint32_t ADC1_Buffer_Sum, Averaged_ADC1_Buffer, Moving_Average_Buffer_Voltage, Setpoint_Limit_Voltage, ADC_Buffer[5];
uint32_t Power_Buffer, Averaged_Power, Moving_Average_Power, Output_Power;
uint32_t HMI_Current_Setpoint=0, HMI_Voltage_Setpoint=0, Arc_Level, CC_Level, CV_Level, My_var, Quanch_Time;
uint32_t Pout=0;
float Setpoint,CC_Level_DAC,Iout_NotCalibrated,Iout;
float Setpoint_v, SP_Step_v=0.1;
float Current_Calibration_Coefficient = 0.28;//0.2086;
float Current_Calibration_Offset = 2.0234;//1.1862;
float Nextion_Current_IL300_Coefficient = 1;
float Setpoint_Limit_Offset_C1 = 0;//0.25;//0.4983;//0.2611;
float Setpoint_Limit_Coefficient_C =1;//1.0286;//1.329;
float Setpoint_Limit_Offset_C2 = 0;
float Vout_NotCalibrated,Vout;
float Nextion_Voltage_IL300_Coefficient = 1;
float Voltage_Calibration_Coefficient = 0.4607;//0.4941;  //Volatge Read=(0.4941*ADC+4.5743)
float Voltage_Calibration_Offset = 1.4988;//0.4988;//4.5743;
float Setpoint_Limit_Coefficient_V = 1;//0.6258;//0.6243;//0.5626; //DAC-SP=(Nextion-SP*0.5626P-1.5773)
float Setpoint_Limit_Offset_V1 = 0;//1.6476;//0.5773;
float Setpoint_Limit_Offset_V2 = 0;
float SP_Step=0.1;
float kasra=0.1;
float CC_Level_Calibration_Coefficient = 1;//2.4363;
float CC_Level_Calibration_Offset = 0;//3.9607;
float Power_Setpoint;
float kP=0.1, kI=0.0001;
//float kP=3, kI=0.01;
//float kP=0.1, kI=0.001;
///float kP=0.1, kI=0.0005;

GPIO_PinState flag;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_DAC2_Init(void);
/* USER CODE BEGIN PFP */
void i2s(unsigned long int ,char str[5]);                // Integer to string data conversion

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_USART2_UART_Init();
  MX_CRC_Init();
  MX_DAC2_Init();
  /* USER CODE BEGIN 2 */
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
	HAL_DAC_Start(&hdac2, DAC_CHANNEL_1);
	//HAL_DAC_Start(&hdac2, DAC_CHANNEL_2);
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);	// Receive Data register not empty interrupt
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_TC);		// Transmission complete interrupt
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	ADC_Calibration_Factor = HAL_ADCEx_Calibration_GetValue(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_SetValue(&hadc1, ADC_SINGLE_ENDED, ADC_Calibration_Factor);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADC_Buffer, 3);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);							// DE/RE = 0 ----> ADM485 is in recieve mode
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);							// Disable PFC
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);								// MCU active
	
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//--------------------- Data Gathering ---------------------//
		Enable_Sputter			=	(buffer_usart[0]-48)	;
		HMI_Current_Setpoint=	(buffer_usart[1]-48)*1000 + (buffer_usart[2]-48)*100 +(buffer_usart[3]-48)*10 +(buffer_usart[4]-48)*1	;
		HMI_Voltage_Setpoint=	(buffer_usart[5]-48)*1000 + (buffer_usart[6]-48)*100 +(buffer_usart[7]-48)*10 +(buffer_usart[8]-48)*1	;
		Arc_Level						=	(buffer_usart[9]-48)*1000 + (buffer_usart[10]-48)*100 +(buffer_usart[11]-48)*10 +(buffer_usart[12]-48)*1	;
		Quanch_Time					=	(buffer_usart[13]-48)*1000 + (buffer_usart[14]-48)*100 +(buffer_usart[15]-48)*10 +(buffer_usart[16]-48)*1	;
		
		if(Quanch_Time > 5000)
			Quanch_Time = 0;
		//----------------------------------------------------------//
		
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
		
		//---------- Calcultaion of Setpoint Limit ------------//
		Setpoint_Limit_Current = (uint32_t)((HMI_Current_Setpoint*Setpoint_Limit_Coefficient_C) - Setpoint_Limit_Offset_C1);
		Setpoint_Limit_Voltage = (uint32_t)((HMI_Voltage_Setpoint*Setpoint_Limit_Coefficient_V) + Setpoint_Limit_Offset_V1);
		//-----------------------------------------------------//
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
			HAL_Delay(Quanch_Time);
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
	  
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_181CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  sConfig.DAC_OutputSwitch = DAC_OUTPUTSWITCH_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief DAC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC2_Init(void)
{

  /* USER CODE BEGIN DAC2_Init 0 */

  /* USER CODE END DAC2_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC2_Init 1 */

  /* USER CODE END DAC2_Init 1 */

  /** DAC Initialization
  */
  hdac2.Instance = DAC2;
  if (HAL_DAC_Init(&hdac2) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputSwitch = DAC_OUTPUTSWITCH_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac2, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC2_Init 2 */

  /* USER CODE END DAC2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 19200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_OUTPUT_Enable_MCU_Pin|GPIO_OUTPUT_Fan_PWM2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_OUTPUT_MCU_EN1_Pin|GPIO_OUTPUT_MCU_LED1_Pin|GPIO_OUTPUT_MCU_LED2_Pin|GPIO_OUTPUT_MCU_LED3_Pin
                          |GPIO_OUTPUT_MCU_LED4_Pin|GPIO_OUTPUT_Relay_Inrush_Pin|GPIO_OUTPUT_PFC_EN_Pin|GPIO_OUTPUT_DE_RE_Pin
                          |GPIO_OUTPUT_MCU_FAN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_OUTPUT_Relay_AC_Pin|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : GPIO_INPUT_Hiccup_Pin GPIO_INPUT_TACHO2_Pin */
  GPIO_InitStruct.Pin = GPIO_INPUT_Hiccup_Pin|GPIO_INPUT_TACHO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_OUTPUT_Enable_MCU_Pin GPIO_OUTPUT_Fan_PWM2_Pin */
  GPIO_InitStruct.Pin = GPIO_OUTPUT_Enable_MCU_Pin|GPIO_OUTPUT_Fan_PWM2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_OUTPUT_MCU_EN1_Pin GPIO_OUTPUT_MCU_LED1_Pin GPIO_OUTPUT_MCU_LED2_Pin GPIO_OUTPUT_MCU_LED3_Pin
                           GPIO_OUTPUT_MCU_LED4_Pin GPIO_OUTPUT_Relay_Inrush_Pin GPIO_OUTPUT_PFC_EN_Pin GPIO_OUTPUT_DE_RE_Pin
                           GPIO_OUTPUT_MCU_FAN_Pin */
  GPIO_InitStruct.Pin = GPIO_OUTPUT_MCU_EN1_Pin|GPIO_OUTPUT_MCU_LED1_Pin|GPIO_OUTPUT_MCU_LED2_Pin|GPIO_OUTPUT_MCU_LED3_Pin
                          |GPIO_OUTPUT_MCU_LED4_Pin|GPIO_OUTPUT_Relay_Inrush_Pin|GPIO_OUTPUT_PFC_EN_Pin|GPIO_OUTPUT_DE_RE_Pin
                          |GPIO_OUTPUT_MCU_FAN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_INPUT_MCU_HeatSinkTemp_Pin */
  GPIO_InitStruct.Pin = GPIO_INPUT_MCU_HeatSinkTemp_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_INPUT_MCU_HeatSinkTemp_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_OUTPUT_Relay_AC_Pin PC8 */
  GPIO_InitStruct.Pin = GPIO_OUTPUT_Relay_AC_Pin|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_INPUT_MCU_DCLINK_OK_Pin */
  GPIO_InitStruct.Pin = GPIO_INPUT_MCU_DCLINK_OK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_INPUT_MCU_DCLINK_OK_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void i2s(unsigned long int num,char str[5]){
    str[0] = (num/1000)%10 + '0';
    str[1] = (num/100)%10 + '0';
    str[2] = (num/10)%10 + '0';
    str[3] = (num/1)%10 + '0';
    str[4] = '0';
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
