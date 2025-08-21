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
#include <stdio.h>
#define RTC_ADDR   0x68
uint8_t time_buffer[7];
uint8_t second = 0;
uint8_t tx_buff[27] = "Started To execute\n\r";
char uart_buf[20];
int uart_buf_len;

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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Convert Decimal → BCD
uint8_t dec2bcd(uint8_t val) {
    return ((val / 10) << 4) | (val % 10);
}

// Convert BCD → Decimal
uint8_t bcd2dec(uint8_t val) {
    return ((val >> 4) * 10) + (val & 0x0F);
}

void RTC_SetDateTime(uint16_t year, uint8_t month, uint8_t date,
                     uint8_t hour, uint8_t minute, uint8_t second)
{
    uint8_t set_time[7];

    set_time[0] = dec2bcd(second & 0x7F);   // 0x03 Seconds (bit7 = OS flag, must keep 0)
    set_time[1] = dec2bcd(minute);          // 0x04 Minutes
    set_time[2] = dec2bcd(hour);            // 0x05 Hours
    set_time[3] = dec2bcd(date);            // 0x06 Day
    set_time[4] = dec2bcd(1);               // 0x07 Weekday (dummy or real)
    set_time[5] = dec2bcd(month);           // 0x08 Month
    set_time[6] = dec2bcd(year % 100);      // 0x09 Year (00–99)

    HAL_I2C_Mem_Write(&hi2c1, RTC_ADDR << 1, 0x03, 1, set_time, 7, HAL_MAX_DELAY);
}


void RTC_ReadDateTime(void)
{
    char uart_buf[50];
    int uart_buf_len;

    HAL_I2C_Mem_Read(&hi2c1, RTC_ADDR << 1, 0x03, 1, time_buffer, 7, HAL_MAX_DELAY);

    uint8_t second = bcd2dec(time_buffer[0] & 0x7F);  // mask OS bit
    uint8_t minute = bcd2dec(time_buffer[1]);
    uint8_t hour   = bcd2dec(time_buffer[2] & 0x3F);
    uint8_t date   = bcd2dec(time_buffer[3]);
    uint8_t wday   = bcd2dec(time_buffer[4]);
    uint8_t month  = bcd2dec(time_buffer[5] & 0x1F);
    uint8_t yr = bcd2dec(time_buffer[6]);
    uint16_t year;

    // Decide century (simple rule: 70–99 → 1900s, else → 2000s)
    if (yr >= 70) {
        year = 1900 + yr;
    } else {
        year = 2000 + yr;
    }


		uart_buf_len = sprintf(uart_buf,
							   "%02d:%02d:%04d %02d:%02d:%02d\r\n",
							   date, month, year, hour, minute, second);

    HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, HAL_MAX_DELAY);
}


uint8_t rtc_check_os_flag(void)
{
    uint8_t sec;

    // Read Seconds register (0x03)
    if (HAL_I2C_Mem_Read(&hi2c1, RTC_ADDR << 1, 0x03, I2C_MEMADD_SIZE_8BIT,
                         &sec, 1, HAL_MAX_DELAY) != HAL_OK)
    {
        const char *msg = "I2C Read Error\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
        return 0xFF; // indicate comm error
    }

    if (sec & 0x80) {
        const char *msg = "Oscillator STOP detected (time invalid)\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
        return 1;  // OS flag set
    } else {
        const char *msg = "RTC running normally\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
        return 0;  // OS flag clear
    }
}
// --- I2C helpers ---
static HAL_StatusTypeDef i2c_write_u8(uint8_t reg, uint8_t val) {
    return HAL_I2C_Mem_Write(&hi2c1, RTC_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, HAL_MAX_DELAY);
}
static HAL_StatusTypeDef i2c_read_u8(uint8_t reg, uint8_t *val) {
    return HAL_I2C_Mem_Read(&hi2c1, RTC_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT, val, 1, HAL_MAX_DELAY);
}

void RTC_EnableBatterySwitchover(void)
{
    uint8_t c3 = 0x20;  // PM=010, enable battery switch-over

    if (HAL_I2C_Mem_Write(&hi2c1, RTC_ADDR << 1,
                          0x02, I2C_MEMADD_SIZE_8BIT,
                          &c3, 1, HAL_MAX_DELAY) != HAL_OK) {

        char buf[32];
        sprintf(buf, "Write to C3 failed!\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)buf, sizeof(buf), HAL_MAX_DELAY);
        return;
    }

    HAL_Delay(10);

    // Read back
    if (HAL_I2C_Mem_Read(&hi2c1, RTC_ADDR << 1,
                         0x02, I2C_MEMADD_SIZE_8BIT,
                         &c3, 1, HAL_MAX_DELAY) == HAL_OK) {
        char buf[32];
        sprintf(buf, "After write C3=0x%02X\r\n", c3);
        HAL_UART_Transmit(&huart2, (uint8_t*)buf, sizeof(buf), HAL_MAX_DELAY);
    }
}



// Dump key regs to UART: Seconds(0x03), Control1(0x00), Control2(0x01), Control3(0x02)
void RTC_DumpDebug(void)
{
    char buf[96];
    uint8_t r0=0,r1=0,r2=0,sec=0;
    i2c_read_u8(0x00, &r0);
    i2c_read_u8(0x01, &r1);
    i2c_read_u8(0x02, &r2);
    i2c_read_u8(0x03, &sec);

    int n = snprintf(buf, sizeof(buf),
        "DBG C1=0x%02X C2=0x%02X C3=0x%02X SEC=0x%02X (OS=%d)\r\n",
        r0, r1, r2, sec, (sec & 0x80) ? 1 : 0);
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, n, HAL_MAX_DELAY);
}


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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Transmit(&huart2, tx_buff, sizeof(tx_buff), 10);
  RTC_EnableBatterySwitchover();
  RTC_DumpDebug();
  if (rtc_check_os_flag()) {
      HAL_UART_Transmit(&huart2, (uint8_t *)"RTC lost power, setting default\r\n", 32, HAL_MAX_DELAY);
      RTC_SetDateTime(1999, 1, 1, 12, 0, 0);  // initialize once
  } else {
      HAL_UART_Transmit(&huart2, (uint8_t *)"RTC running, keeping time\r\n", 28, HAL_MAX_DELAY);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 RTC_ReadDateTime();

	 HAL_Delay(1000);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00503D58;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
