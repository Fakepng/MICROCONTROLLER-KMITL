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
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "lux.h"
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
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t rx_indx;
uint8_t rx_data[2];
uint8_t rx_buffer[100];
uint8_t rx_buffer_old[100];
uint8_t transfer_cplt;
uint8_t debug_message[100] = "none";

uint8_t spi_data[8];
uint8_t spi_response[8];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void uart_debug() {

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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, rx_data, 1);
  HAL_SPI_Receive_IT(&hspi1, spi_data, 8);

  HAL_UART_Transmit(&huart2, (uint8_t *)"Start\r\n", 7, 1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  if (spi_data[2] == 0x1) {
//		  HAL_UART_Transmit(&huart2, spi_data, 8, 100);
//		  spi_transmit(&hspi1, spi_data);
//	  }

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, USER_LED_Pin|RELAY_5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RELAY_4_Pin|RELAY_1_Pin|RELAY_3_Pin|RELAY_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_BTN_Pin */
  GPIO_InitStruct.Pin = USER_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_LED_Pin */
  GPIO_InitStruct.Pin = USER_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(USER_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RELAY_4_Pin RELAY_1_Pin RELAY_3_Pin RELAY_2_Pin */
  GPIO_InitStruct.Pin = RELAY_4_Pin|RELAY_1_Pin|RELAY_3_Pin|RELAY_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RELAY_5_Pin */
  GPIO_InitStruct.Pin = RELAY_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RELAY_5_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin) {
	switch (GPIO_Pin) {
	case USER_BTN_Pin:
		// Do something
		break;

	default:
	}
}

void process_uart() {
	strncpy(rx_buffer_old, rx_buffer, 100);
	HAL_UART_Transmit(&huart2, "\r\n", 2, 100);

	if (!strcmp(rx_buffer, (uint8_t *)"?")) {
		const char message[] = "led on, led off, error, spi, clear, ?";
		HAL_UART_Transmit(&huart2, (uint8_t *)message, (int)strlen(message), 1000);
	}

	if (!strcmp(rx_buffer, (uint8_t *)"led on")) {
		HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, 1);
		HAL_UART_Transmit(&huart2, (uint8_t *)"1", 1, 100);
	}

	if (!strcmp(rx_buffer, (uint8_t *)"led off")) {
		HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, 0);
		HAL_UART_Transmit(&huart2, (uint8_t *)"1", 1, 100);
	}

	if (!strcmp(rx_buffer, (uint8_t *)"error")) {
		HAL_UART_Transmit(&huart2, debug_message, strlen(debug_message), 1000);
		strncpy(debug_message, (uint8_t *)"none", 100);
	}

	if (!strcmp(rx_buffer, (uint8_t *)"spi")) {
		uint8_t message[64];
		int message_length = sprintf(message, "0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x", (int)spi_data[0], (int)spi_data[1], (int)spi_data[2], (int)spi_data[3], (int)spi_data[4], (int)spi_data[5], (int)spi_data[6], (int)spi_data[7]);
		HAL_UART_Transmit(&huart2, message, message_length, 1000);
	}

	if (!strcmp(rx_buffer, "clear")) {
		HAL_UART_Transmit(&huart2, (uint8_t *)"\033[2J\033[0;0H", (int)strlen("\033[2J\033[0;0H"), 100);
	}

	HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, 100);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	UNUSED(huart);

	uint8_t i;
	if (huart->Instance == USART2) {
		if (rx_indx == 0) {
			for (i = 0; i < 100; i++) {
				rx_buffer[i] = 0;
			}
		}
		if (rx_data[0] == 8) {
			if (rx_indx > 0) {
				rx_buffer[--rx_indx] = 0;
			}
		} else if (rx_data[0] == 13 && rx_indx == 0) {
			strncpy(rx_buffer, rx_buffer_old, 100);
			process_uart();
		} else if (rx_data[0] != 13) {
			rx_buffer[rx_indx++] = rx_data[0];
		} else {
			rx_indx = 0;
			transfer_cplt = 1;
			process_uart();
		}

		HAL_UART_Receive_IT(&huart2, rx_data, 1);
		HAL_UART_Transmit(&huart2, rx_data, strlen(rx_data), 100);
	}
}

uint16_t crc16_modbus(uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];  // XOR byte into least sig. byte of crc

        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {  // If LSB is set
                crc = (crc >> 1) ^ 0xA001;  // Shift right and XOR with polynomial
            } else {
                crc >>= 1;  // Just shift right
            }
        }
    }

    return crc;  // Return the CRC16 result
}

int validate_crc16_modbus(uint8_t *data, uint16_t length) {
    if (length != 8) {
        return 0;  // Only works for 8-byte packets (6 bytes data + 2 bytes CRC)
    }

    // Last two bytes in the data array are the received CRC (MSB first)
    uint16_t received_crc = (data[length - 2] << 8) | data[length - 1];  // Combine MSB and LSB

    // Calculate the CRC on the first 6 bytes of data
    uint16_t calculated_crc = crc16_modbus(data, length - 2);

    // Compare calculated CRC with received CRC
    return (calculated_crc == received_crc);
}

void spi_transmit(SPI_HandleTypeDef *hspi) {
    uint16_t crc = crc16_modbus(spi_response, 6);  // Calculate CRC for the first 6 bytes
    spi_response[6] = (crc >> 8) & 0xFF;  // CRC MSB
    spi_response[7] = crc & 0xFF;         // CRC LSB

    HAL_SPI_Transmit(hspi, spi_response, 8, 100);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi == &hspi1) {
		int checksum = validate_crc16_modbus(spi_data, 8);
		if (!checksum) {
			strncpy(debug_message, "SPI CRC Error", 100);
		}

		spi_response[0] = spi_data[0];
		spi_response[1] = spi_data[1];
		spi_response[2] = spi_data[2];
		spi_response[3] = spi_data[3];
		spi_response[4] = spi_data[4];
		spi_response[5] = spi_data[5];

		if (spi_data[0] == 1 &&
			spi_data[1] == 1) {
			HAL_GPIO_WritePin(RELAY_5_GPIO_Port, RELAY_5_Pin, (spi_data[5] & (1<<4)) >> 4);
			HAL_GPIO_WritePin(RELAY_4_GPIO_Port, RELAY_4_Pin, (spi_data[5] & (1<<3)) >> 3);
			HAL_GPIO_WritePin(RELAY_3_GPIO_Port, RELAY_3_Pin, (spi_data[5] & (1<<2)) >> 2);
			HAL_GPIO_WritePin(RELAY_2_GPIO_Port, RELAY_2_Pin, (spi_data[5] & (1<<1)) >> 1);
			HAL_GPIO_WritePin(RELAY_1_GPIO_Port, RELAY_1_Pin, (spi_data[5] & (1<<0)) >> 0);
		}


		spi_transmit(&hspi1);

		HAL_SPI_Receive_IT(&hspi1, spi_data, 8);
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
