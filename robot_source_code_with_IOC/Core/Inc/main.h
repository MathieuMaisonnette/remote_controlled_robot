/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BP_Pin GPIO_PIN_13
#define BP_GPIO_Port GPIOC
#define BP_EXTI_IRQn EXTI15_10_IRQn
#define IR1_IN_Pin GPIO_PIN_2
#define IR1_IN_GPIO_Port GPIOC
#define IR2_IN_Pin GPIO_PIN_3
#define IR2_IN_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define Alert_batt_Pin GPIO_PIN_5
#define Alert_batt_GPIO_Port GPIOA
#define IR4_IN_Pin GPIO_PIN_4
#define IR4_IN_GPIO_Port GPIOC
#define Tension_batt_Pin GPIO_PIN_5
#define Tension_batt_GPIO_Port GPIOC
#define IR3_IN_Pin GPIO_PIN_1
#define IR3_IN_GPIO_Port GPIOB
#define DIR2_Pin GPIO_PIN_2
#define DIR2_GPIO_Port GPIOB
#define PWMD_Pin GPIO_PIN_11
#define PWMD_GPIO_Port GPIOB
#define IR1_CMD_Pin GPIO_PIN_12
#define IR1_CMD_GPIO_Port GPIOB
#define IR4_CMD_Pin GPIO_PIN_14
#define IR4_CMD_GPIO_Port GPIOB
#define IR2_CMD_Pin GPIO_PIN_15
#define IR2_CMD_GPIO_Port GPIOB
#define DIR1_Pin GPIO_PIN_8
#define DIR1_GPIO_Port GPIOC
#define IR3_CMD_Pin GPIO_PIN_12
#define IR3_CMD_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define PWMG_Pin GPIO_PIN_15
#define PWMG_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
