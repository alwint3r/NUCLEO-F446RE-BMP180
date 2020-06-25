/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct bmp180_cal_struct {
	int16_t ac1;
	int16_t ac2;
	int16_t ac3;
	uint16_t ac4;
	uint16_t ac5;
	uint16_t ac6;
	int16_t b1;
	int16_t b2;
	int16_t mb;
	int16_t mc;
	int16_t md;
} bmp180_cal_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BMP180_CAL_AC1 0xAA
#define BMP180_CAL_AC2 0xAC
#define BMP180_CAL_AC3 0xAE
#define BMP180_CAL_AC4 0xB0
#define BMP180_CAL_AC5 0xB2
#define BMP180_CAL_AC6 0xB4
#define BMP180_CAL_B1 0xB6
#define BMP180_CAL_B2 0xB8
#define BMP180_CAL_MB 0xBA
#define BMP180_CAL_MC 0xBC
#define BMP180_CAL_MD 0xBE
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
bmp180_cal_t cal;
static const uint8_t BMP180_ADDR = 0x77 << 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
HAL_StatusTypeDef BMP180_ReadInt16(uint8_t address, int16_t *result);
HAL_StatusTypeDef BMP180_Write(uint8_t* data, size_t length);
HAL_StatusTypeDef BMP180_ReadByte(uint8_t address, uint8_t *result);
HAL_StatusTypeDef BMP180_Read_Calibration();
HAL_StatusTypeDef BMP180_Read_Temperature(float *result);
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
  uint8_t pbuf[128];
  float temperature;
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  HAL_StatusTypeDef ret = BMP180_Read_Calibration();
  if (ret != HAL_OK)
  {
    sprintf((char*)pbuf, "Failed reading calibration data!\r\n");
  }
  else
  {
    sprintf((char*)pbuf, "Succesfully read calibration data!\r\n");
  }

  HAL_UART_Transmit(&huart2, pbuf, strlen((char*)pbuf), HAL_MAX_DELAY);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    ret = BMP180_Read_Temperature(&temperature);
    if (ret != HAL_OK)
    {
      sprintf((char*)pbuf, "Failed reading temperature!\r\n");
    }
    else
    {
      unsigned int temp = (unsigned int)temperature / 10;
      unsigned int dec = (unsigned int)temperature % 10;
      sprintf((char*)pbuf, "Temperature: %u.%02u C\r\n", temp, dec);
    }

    HAL_UART_Transmit(&huart2, pbuf, strlen((char*)pbuf), HAL_MAX_DELAY);
    HAL_Delay(2000);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
HAL_StatusTypeDef BMP180_Write(uint8_t* data, size_t length)
{
  HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(&hi2c1, BMP180_ADDR, data, length, HAL_MAX_DELAY);

  return ret;
}

HAL_StatusTypeDef BMP180_ReadByte(uint8_t address, uint8_t *result)
{

  HAL_StatusTypeDef ret = BMP180_Write(&address, 1);

  if (ret != HAL_OK)
  {
    return ret;
  }

  uint8_t readResult = 0;
  ret = HAL_I2C_Master_Receive(&hi2c1, BMP180_ADDR, &readResult, 1, HAL_MAX_DELAY);
  if (ret != HAL_OK)
  {
    return ret;
  }

  *result = readResult;

  return HAL_OK;
}

HAL_StatusTypeDef BMP180_ReadInt16(uint8_t address, int16_t *result)
{
  uint8_t buf[2];

  buf[0] = address;
  HAL_StatusTypeDef ret = BMP180_Write(buf, 1);

  if (ret != HAL_OK)
  {
    return ret;
  }

  ret = HAL_I2C_Master_Receive(&hi2c1, BMP180_ADDR, buf, 2, HAL_MAX_DELAY);
  if (ret != HAL_OK)
  {
    return ret;
  }

  *result = ((int16_t)buf[0] << 8 | buf[1]);

  return HAL_OK;
}

HAL_StatusTypeDef BMP180_Read_Calibration()
{
  HAL_StatusTypeDef ret;

  ret = BMP180_ReadInt16(BMP180_CAL_AC1, &cal.ac1);
  if (ret != HAL_OK)
  {
    return ret;
  }

  ret = BMP180_ReadInt16(BMP180_CAL_AC2, &cal.ac2);
  if (ret != HAL_OK)
  {
    return ret;
  }

  ret = BMP180_ReadInt16(BMP180_CAL_AC3, &cal.ac3);
  if (ret != HAL_OK)
  {
    return ret;
  }

  int16_t rshort;

  ret = BMP180_ReadInt16(BMP180_CAL_AC4, &rshort);
  if (ret != HAL_OK)
  {
    return ret;
  }

  cal.ac4 = (uint16_t)rshort;

  ret = BMP180_ReadInt16(BMP180_CAL_AC5, &rshort);
  if (ret != HAL_OK)
  {
    return ret;
  }

  cal.ac5 = (uint16_t)rshort;

  ret = BMP180_ReadInt16(BMP180_CAL_AC6, &rshort);
  if (ret != HAL_OK)
  {
    return ret;
  }

  cal.ac6 = (uint16_t)rshort;

  ret = BMP180_ReadInt16(BMP180_CAL_B1, &cal.b1);
  if (ret != HAL_OK)
  {
    return ret;
  }

  ret = BMP180_ReadInt16(BMP180_CAL_B2, &cal.b2);
  if (ret != HAL_OK)
  {
    return ret;
  }

  ret = BMP180_ReadInt16(BMP180_CAL_MB, &cal.mb);
  if (ret != HAL_OK)
  {
    return ret;
  }

  ret = BMP180_ReadInt16(BMP180_CAL_MC, &cal.mc);
  if (ret != HAL_OK)
  {
    return ret;
  }

  ret = BMP180_ReadInt16(BMP180_CAL_MD, &cal.md);
  if (ret != HAL_OK)
  {
    return ret;
  }

  return HAL_OK;
}

HAL_StatusTypeDef BMP180_Read_Temperature(float *result)
{
  uint8_t buf[2];
  HAL_StatusTypeDef ret;

  buf[0] = 0xF4;
  buf[1] = 0x2E;
  ret = BMP180_Write(buf, 2);
  if (ret != HAL_OK)
  {
    return ret;
  }

  HAL_Delay(5);

  int16_t utemp;
  ret = BMP180_ReadInt16(0xF6, &utemp);
  if (ret != HAL_OK)
  {
    return ret;
  }

  long x1 = (utemp - cal.ac6) * (cal.ac5 / pow(2, 15));
  long x2 = (cal.mc * pow(2, 11)) / (x1 + cal.md);
  long b5 = x1 + x2;
  long temp = (b5 + 8) / pow(2, 4);

  *result = (float)temp;

  return HAL_OK;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
