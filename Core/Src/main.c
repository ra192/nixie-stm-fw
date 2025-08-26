/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nixie.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DS3231_I2C_ADDR 0x68

#define DS3231_REG_SECOND 	0x00
#define DS3231_REG_DOW 	0x03

#define NIXIE_REFRESH_MS 2
#define TIME_REFRESH_MS 1000
#define TIME_SYNC_MS 60000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

/* USER CODE BEGIN PV */
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
RTC_TimeTypeDef ds3231_getTime(void);
RTC_DateTypeDef ds3231_getDate(void);
void syncTimeWithDS3231(void);
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
  MX_I2C1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  uint32_t currentMs;
  uint32_t nixieLastRefreshMs = 0;
  uint32_t timeLastRefreshMs = 0;
  uint32_t syncTimeLastRefreshMs = 0;

  nixie_setDigits((uint8_t[]){0, 0, 0, 0, 0, 0}); // Set initial digits to display
  syncTimeWithDS3231();                     // Sync time from DS3231 to RTC
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    currentMs = HAL_GetTick();

    if ((currentMs - nixieLastRefreshMs) >= NIXIE_REFRESH_MS)
    {
      nixieLastRefreshMs = currentMs;
      nixie_refresh();
    }

    if ((currentMs - timeLastRefreshMs) >= TIME_REFRESH_MS)
    { // Every second, change the digits
      HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
      nixie_setDigits((uint8_t[]){0, 0, sTime.Minutes / 10, sTime.Minutes % 10, sTime.Seconds / 10, sTime.Seconds % 10});
    }

    if ((currentMs - syncTimeLastRefreshMs) >= TIME_SYNC_MS)
    { // Every minute, sync time from DS3231 to RTC
      syncTimeLastRefreshMs = currentMs;
      syncTimeWithDS3231();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
   */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, D0_Pin | A4_Pin | D1_Pin | A5_Pin | D2_Pin | A6_Pin | LED_Pin | D5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, D3_Pin | A2_Pin | D9_Pin | D8_Pin | D7_Pin | D6_Pin | D4_Pin | A1_Pin | A3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : D0_Pin A4_Pin D1_Pin A5_Pin
                           D2_Pin A6_Pin LED_Pin D5_Pin */
  GPIO_InitStruct.Pin = D0_Pin | A4_Pin | D1_Pin | A5_Pin | D2_Pin | A6_Pin | LED_Pin | D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : D3_Pin A2_Pin D9_Pin D8_Pin
                           D7_Pin D6_Pin D4_Pin A1_Pin
                           A3_Pin */
  GPIO_InitStruct.Pin = D3_Pin | A2_Pin | D9_Pin | D8_Pin | D7_Pin | D6_Pin | D4_Pin | A1_Pin | A3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
RTC_TimeTypeDef ds3231_getTime(void)
{
  RTC_TimeTypeDef time;

  uint8_t addr[]={DS3231_REG_SECOND};
  uint8_t buffer[3];
  
  HAL_I2C_Master_Transmit(&hi2c1, DS3231_I2C_ADDR << 1, addr, 1, HAL_MAX_DELAY);
  HAL_I2C_Master_Receive(&hi2c1, DS3231_I2C_ADDR << 1, buffer, 3, HAL_MAX_DELAY);

  time.Hours = ((buffer[2] >> 4) & 0x03) * 10 + (buffer[2] & 0x0F);
  time.Minutes = ((buffer[1] >> 4) & 0x07) * 10 + (buffer[1] & 0x0F);
  time.Seconds = ((buffer[0] >> 4) & 0x07) * 10 + (buffer[0] & 0x0F);

  return time;
}

RTC_DateTypeDef ds3231_getDate(void)
{
  RTC_DateTypeDef date;

  uint8_t addr[]={DS3231_REG_DOW};
  uint8_t buffer[4];
  
  HAL_I2C_Master_Transmit(&hi2c1, DS3231_I2C_ADDR << 1, addr, 1, HAL_MAX_DELAY);
  HAL_I2C_Master_Receive(&hi2c1, DS3231_I2C_ADDR << 1, buffer, 4, HAL_MAX_DELAY);

  date.Year = ((buffer[3] >> 4) & 0x0F) * 10 + (buffer[3] & 0x0F);
  date.Month = ((buffer[2] >> 4) & 0x01) * 10 + (buffer[2] & 0x0F);
  date.Date = ((buffer[1] >> 4) & 0x03) * 10 + (buffer[1] & 0x0F);
  date.WeekDay = buffer[0] & 0x07;

  return date;
}

void syncTimeWithDS3231(void)
{
  sTime = ds3231_getTime();
  sDate = ds3231_getDate();
  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
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
#ifdef USE_FULL_ASSERT
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
