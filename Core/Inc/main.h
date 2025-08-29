/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SW1_Pin GPIO_PIN_0
#define SW1_GPIO_Port GPIOA
#define SW2_Pin GPIO_PIN_1
#define SW2_GPIO_Port GPIOA
#define SW3_Pin GPIO_PIN_2
#define SW3_GPIO_Port GPIOA
#define D0_Pin GPIO_PIN_3
#define D0_GPIO_Port GPIOA
#define A4_Pin GPIO_PIN_4
#define A4_GPIO_Port GPIOA
#define D1_Pin GPIO_PIN_5
#define D1_GPIO_Port GPIOA
#define A5_Pin GPIO_PIN_6
#define A5_GPIO_Port GPIOA
#define D2_Pin GPIO_PIN_7
#define D2_GPIO_Port GPIOA
#define D3_Pin GPIO_PIN_0
#define D3_GPIO_Port GPIOB
#define A2_Pin GPIO_PIN_1
#define A2_GPIO_Port GPIOB
#define D9_Pin GPIO_PIN_2
#define D9_GPIO_Port GPIOB
#define D8_Pin GPIO_PIN_10
#define D8_GPIO_Port GPIOB
#define D7_Pin GPIO_PIN_11
#define D7_GPIO_Port GPIOB
#define ID1_Pin GPIO_PIN_14
#define ID1_GPIO_Port GPIOB
#define ID2_Pin GPIO_PIN_15
#define ID2_GPIO_Port GPIOB
#define A6_Pin GPIO_PIN_8
#define A6_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_11
#define LED_GPIO_Port GPIOA
#define D5_Pin GPIO_PIN_12
#define D5_GPIO_Port GPIOA
#define D6_Pin GPIO_PIN_3
#define D6_GPIO_Port GPIOB
#define D4_Pin GPIO_PIN_4
#define D4_GPIO_Port GPIOB
#define A1_Pin GPIO_PIN_5
#define A1_GPIO_Port GPIOB
#define BUZZ_Pin GPIO_PIN_8
#define BUZZ_GPIO_Port GPIOB
#define A3_Pin GPIO_PIN_9
#define A3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
