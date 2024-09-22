/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include<stdio.h>
#include<stdlib.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define vmax_counter 2000
#define vmax 2780//2304//en um/s car 4,34 s/m   // 27.8 m/s en théorie
#define max_accel 10//en um/10ms donc cm/s        10um/10ms soit 1000 um/s
//on actualise ttes les 5ms
//l'accel max dans ce temps est : max_accel*

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
int target_speed = 0;
uint16_t Batt_value;

volatile char cnt_adc_scan = 0;
volatile uint16_t FL_value = 0, BL_value = 0, FR_value = 0, BR_value = 0;
volatile uint16_t ADC_current_value = 0;

bool GO = false;
volatile uint8_t adc_on,nb = 0;

// UART
int flag_uart;
unsigned char Rx_buffer;

char uart_direction;


int d_right, d_left;//distance à droite, distance à gauche
int flag_uart = 0;
int dist_detection = 20;
int nominal_speed;
int speed_left;
int speed_right;
int nominal_target_speed_counter;

bool period_elapsed;

uint32_t last_trigger_bp  =0;


char msg[80];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

void capteur_mur(void);
bool get_direction(int speed);
int compute_speed(int currentSpeed, int speedTarget);
int speed_to_ARR(int speed);
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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  MX_TIM7_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  	nominal_speed = 1000;
	nominal_target_speed_counter = vmax_counter*nominal_speed/vmax;


	//target_speed = 0;

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4, 0);
	//HAL_GPIO_WritePin(Alert_batt_GPIO_Port, Alert_batt_Pin, GPIO_PIN_SET);
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);
	//HAL_UART_Transmit_IT(&huart3, (uint8_t)tab, sizeof(tab));
	HAL_UART_Receive_IT(&huart3, &Rx_buffer, 1);

	d_right = 100;
	d_left = 100;//On initialise dd et dg à une valeur supérieure au seuil. Ils serons modifiés dès le premier lancement de l'ADC.

	bool detection_ended = false;



	//HAL_ADC_Start_IT(&hadc1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	//HAL_ADC_Start_IT(&hadc1);
  while (1)
  {
	  HAL_GPIO_WritePin(IR1_CMD_GPIO_Port, IR1_CMD_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(IR4_CMD_GPIO_Port, IR4_CMD_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(IR2_CMD_GPIO_Port, IR2_CMD_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(IR3_CMD_GPIO_Port, IR3_CMD_Pin, GPIO_PIN_SET);


	  if(GO)
	  {
		  // Les leds seront tout le temps allumées pour éviter les perturbations
		  if(d_right < dist_detection || d_left < dist_detection) // mur détecté
		  {
			  capteur_mur();

			  //On mets brièvement la PWM à 0
			 // __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, 0);//moteur gauche
			 //__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4, 0);//moteur droit

			  detection_ended = true;
		  }
		  else if(period_elapsed || detection_ended)
		  {
			  period_elapsed = false;
			  detection_ended = false;

			switch (uart_direction)
			{
			   case 'F'://Front
			   {
				   speed_left = compute_speed(speed_left, nominal_speed);
				   speed_right = compute_speed(speed_right, nominal_speed);
				  break;
			   }
			   case 'B': //back
			   {
				   speed_left = compute_speed(speed_left, -nominal_speed);
				   speed_right = compute_speed(speed_right, -nominal_speed);

				   break;
			   }
			   case 'L' ://left
			   {
				      speed_left = compute_speed(speed_left, +nominal_speed);
					  speed_right = compute_speed(speed_right, -nominal_speed);



				   break;
			   }
			   case 'R' : //right
			   {
				     speed_left = compute_speed(speed_left, -nominal_speed);
				     speed_right = compute_speed(speed_right, +nominal_speed);

				    break;
			   }
			   case 'W' ://Appel de phare, mode super_vitesse
			   {
				   speed_left = compute_speed(speed_left, vmax);
				   speed_right = compute_speed(speed_right, vmax);
				   break;
			   }
			   case 'X' ://Warning : stop
			   {
				   speed_left = compute_speed(speed_left, 0);
				   speed_right = compute_speed(speed_right, 0);
				  break;
			   }
			   default :
			   {
				   speed_left = compute_speed(speed_left, 0);
				   speed_right = compute_speed(speed_right, 0);
			   }
			}

			// on applique sur la PWM et sur le choix de la direction la vitesse calculée précedemment
			 HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, get_direction(speed_left));//moteur droit
		     HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, get_direction(speed_right));//moteur droit
		     __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, speed_to_ARR(speed_left));//moteur gauche
		     __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4, speed_to_ARR(speed_right));//moteur droit
}
	  }
	  else
	  {
		  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, 0);
		  	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4, 0);
		  	//flag_uart = 0; //plus utile
	  }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
  ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV64;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = DISABLE;
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

  /** Configure Analog WatchDog 1
  */
  AnalogWDGConfig.WatchdogNumber = ADC_ANALOGWATCHDOG_1;
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  AnalogWDGConfig.Channel = ADC_CHANNEL_14;
  AnalogWDGConfig.ITMode = ENABLE;
  AnalogWDGConfig.HighThreshold = 4095;
  AnalogWDGConfig.LowThreshold = 3723;
  if (HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_16;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 13-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 61538-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 1-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 40000;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  huart2.Init.BaudRate = 115200;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_8;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Alert_batt_Pin|IR3_CMD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIR2_Pin|IR1_CMD_Pin|IR4_CMD_Pin|IR2_CMD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BP_Pin */
  GPIO_InitStruct.Pin = BP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Alert_batt_Pin IR3_CMD_Pin */
  GPIO_InitStruct.Pin = Alert_batt_Pin|IR3_CMD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR2_Pin IR1_CMD_Pin IR4_CMD_Pin IR2_CMD_Pin */
  GPIO_InitStruct.Pin = DIR2_Pin|IR1_CMD_Pin|IR4_CMD_Pin|IR2_CMD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DIR1_Pin */
  GPIO_InitStruct.Pin = DIR1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DIR1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//l'origine de l'interruption est donnée
{
	if(htim->Instance == TIM6)
	{
		/*Si nous avions voulu allumer les leds infra, ce serait ici */
		HAL_ADC_Start_IT(&hadc1);
	}
	if(htim->Instance == TIM7)
	{
		period_elapsed = true;
	}
}

int adc_value_to_dist(float a, float b, int p)
{
	//int dist = (int)((1.0f/a)*log(b/((float)p))); // regression exponentielle
	int dist = powf(b/(float)p, 1/a);// regression "puissance"
	return dist;
}



int compute_speed(int currentSpeed, int speedTarget)
{
	int nextSpeed;
	int diff = abs(speedTarget - currentSpeed);
		if(speedTarget > currentSpeed)
		{
			if(speedTarget <= 0)
			{
				if(diff > max_accel) diff = max_accel;
				nextSpeed = currentSpeed + diff;
			}
			else
			{
				if(diff > max_accel) diff = max_accel;

				nextSpeed = currentSpeed + diff;
			}
		}
		else if(speedTarget < currentSpeed)
		{
			if(speedTarget < 0)
			{
				if(diff > max_accel)
					diff = max_accel;
				nextSpeed = currentSpeed - diff;
			}
			else
			{
				if(diff > max_accel)
					diff = max_accel;
				nextSpeed = currentSpeed - diff;
			}
		}
		else
		{
			nextSpeed = currentSpeed;
		}

		/*if(abs(nextSpeed) > maxSpeed) //necessaire si on veut une vitess
		{
			nextSpeed = nextSpeed/abs(nextSpeed) * maxSpeed;
		}*/
		return nextSpeed;
}


int speed_to_ARR(int speed)
{
	int nominal_target_speed_counter = vmax_counter*abs(speed)/vmax;//en valeur absolue
	return nominal_target_speed_counter;
}

bool get_direction(int speed)
{
    return speed > 0 ? 1 : 0;
}




void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
		//sprintf(msg, "Tension batterie = %d, vitesse droite = %d, vitesse gauche = %d\r\n",Batt_value, speed_right, speed_left);
		//HAL_UART_Transmit(&huart2, (uint8_t*) msg, sizeof(msg), HAL_MAX_DELAY);

	  if(cnt_adc_scan == 0)//gauche
		  {
			  FR_value = HAL_ADC_GetValue(&hadc1);
			  d_right = adc_value_to_dist(0.942,4937.5, FR_value);//TODO ici modifier coeffs


		  }
		if(cnt_adc_scan == 1)//droite
		  {
			FL_value = HAL_ADC_GetValue(&hadc1);
			d_left = adc_value_to_dist(0.942,4937.5, FL_value); // TODO ici modifier coeffs
		  }
		if(cnt_adc_scan == 2)
		  {
			  BL_value = HAL_ADC_GetValue(&hadc1);
		  }
		  if(cnt_adc_scan == 3)
		  {
			  BR_value = HAL_ADC_GetValue(&hadc1);
		  }
		  if(cnt_adc_scan == 4)
		  {
			  Batt_value = HAL_ADC_GetValue(&hadc1);//Ce get_value n'est pas nécessaire,
			  //nous le récupérons quand même à des fins de déboguage puisque nous
			  //faisons déjà un scan
			  cnt_adc_scan=-1;
		  }
		cnt_adc_scan = cnt_adc_scan+1;


}


void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc)
{
  HAL_GPIO_WritePin(Alert_batt_GPIO_Port, Alert_batt_Pin, GPIO_PIN_SET);
}

//inversion du booléen GO lors d'un appui sur le bouton bleu
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint32_t actual_time = HAL_GetTick();

	if (GPIO_Pin == BP_Pin  && actual_time - last_trigger_bp > 7)//plus logique vers les centaines de ms
	{
		GO = !GO;
		last_trigger_bp = actual_time;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART3)
	{
		if(flag_uart % 2 ==0)
		{
			if(Rx_buffer != 'S')
				sscanf(&Rx_buffer, "%c",&uart_direction);
		}

		HAL_UART_Receive_IT(&huart3, &Rx_buffer, 1);//on relance après réception
		UNUSED(huart);
		if(Rx_buffer != 'D'){
			//si on se déconnecte et reconnecte il envoie un D pour indiquer qu'il
			//y a eu une connexion. Ce D ne doit donc pas être pris en compte comme une commande
			flag_uart++;
		}

	}
	//uint8_t *coucou;
 //sscanf(Rx_buffer, "%hhu", coucou );
}

void capteur_mur()//d1 = d_left et d2 = d_right
{
	bool avoidance_ended = false;
	int diff;
	while(!avoidance_ended && GO)
	{
		diff = d_left - d_right;//on recalcule à chaque fois
		if(d_left <= dist_detection && d_right <= dist_detection)//bloqué dans un coin
		{
			   HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, 0);//droit
			   HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, 0);//gauche
			   __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, nominal_target_speed_counter);//moteur gauche
			   __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4, nominal_target_speed_counter);//moteur droit
		}
		else if(diff >= 0)//mur à droite, on tourne à gauche
			//Si le mur est en face on choisit d'aller à gauche
		{
			   HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, 1);//droit
			   HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, 0);//gauche
			   __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, nominal_target_speed_counter);//moteur gauche
			   __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4, nominal_target_speed_counter);//moteur droit
		}
		else if(diff < 0) //mur à gauche on tourne à droite
		{
			   HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, 0);//droit
			   HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, 1);//gauche
			   __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, nominal_target_speed_counter);//moteur gauche
			   __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4, nominal_target_speed_counter);//moteur droit
		}
		if(d_right > dist_detection && d_left > dist_detection)
		 	avoidance_ended = true;
	}
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
