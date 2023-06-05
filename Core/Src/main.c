/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "STM32_EEPROM_SPI.h"

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

const uint8_t TxBuffer[] = "The MCP 25LC1024 is a specific integrated circuit (IC) from the Microchip Technology's family of serial EEPROM (Electrically Erasable Programmable Read-Only Memory) devices. EEPROM is a \
		non-volatile memory technology that allows for the storage and retrieval of data even when the power is turned off.\
		The 25LC1024 is a serial EEPROM chip with a storage capacity of 1,048,576 bits, equivalent to 128 kilobytes (KB). \
		It uses a standard SPI (Serial Peripheral Interface) bus for communication, which makes it compatible with a \
		wide range of microcontrollers and other digital devices.";

/* Size of Transmission buffer */
#define TXBUFFERSIZE                      (COUNTOF(TxBuffer) - 1)
/* Size of Reception buffer */
#define RXBUFFERSIZE                      TXBUFFERSIZE

uint8_t RxBuffer[RXBUFFERSIZE];

HAL_StatusTypeDef hal_status = HAL_OK;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/************ Byte transfer functions ***************************/
EEPROM_Status EEPROM_WriteByte(uint32_t WriteAddr, uint8_t data);
EEPROM_Status EEPROM_ReadByte(uint32_t ReadAddr, uint8_t *pBuffer);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// WARNING UART2 is connected to st-link
// Properties -> Settings -> MCU_Settings -> select use float with printf
/*
 * USART2, BaudRate = 115200, WordLength = UART_WORDLENGTH_8B, StopBits = UART_STOPBITS_1;
 * */
int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart2,(uint8_t *)ptr, len, HAL_TIMEOUT);
	return len;
}


/**
 * @brief  On Error Handler on condition TRUE.
 * @param  condition : Can be TRUE or FALSE
 * @retval None
 */
static void OnError_Handler(uint32_t condition)
{
	if(condition)
	{
		//BSP_LED_On(LED1);
		while(1) { ; } /* Blocking on error */
	}
}

void transformAddress(uint32_t address, uint8_t byteArray[3]) {
	byteArray[0] = (address >> 16) & 0xFF;  // Extract the most significant byte
	byteArray[1] = (address >> 8) & 0xFF;   // Extract the middle byte
	byteArray[2] =  address & 0xFF;         // Extract the least significant byte
}

static void test_eeprom(void)
{
	/*
	 https://www.digikey.fr/en/maker/projects/getting-started-with-stm32-how-to-use-spi/09eab3dfe74c4d0391aaaa99b0a8ee17
	 */

	// 25LC1024 SPI EEPROM instructions
	const uint8_t __EEPROM_WREN  = 0b00000110; /*!< Write Enable */
//	const uint8_t __EEPROM_WRDI  = 0b00000100; /*!< Write Disable */
	const uint8_t __EEPROM_RDSR  = 0b00000101; /*!< Read Status Register */
//	const uint8_t __EEPROM_WRSR  = 0b00000001; /*!< Write Status Register */
	const uint8_t __EEPROM_READ  = 0b00000011; /*!< Read from Memory Array */
	const uint8_t __EEPROM_WRITE = 0b00000010; /*!< Write to Memory Array */

	char uart_buf[50];
	int uart_buf_len;
	char spi_buf[20];
	uint8_t wip;
	uint32_t addr;
	uint8_t  addrArray[3];

	// CS pin should default high
	EEPROM_CS_HIGH();

	// Say something
	uart_buf_len = sprintf(uart_buf, "SPI Test\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);

	// Enable write enable latch (allow write operations)
	EEPROM_CS_LOW();
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&__EEPROM_WREN, 1, 100);
	EEPROM_CS_HIGH();

	// Read status register
	EEPROM_CS_LOW();
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&__EEPROM_RDSR, 1, 100);
	HAL_SPI_Receive(&hspi1, (uint8_t *)spi_buf, 1, 100);
	EEPROM_CS_HIGH();

	// Print out status register
	uart_buf_len = sprintf(uart_buf,
			"Status: %0x\r\n",
			(unsigned int)spi_buf[0]);
	HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);

	// Test bytes to write to EEPROM
	spi_buf[0] = 0xAB;
	spi_buf[1] = 0xCD;
	spi_buf[2] = 0xEF;

	// Set starting address
	addr = 0x00ABCDEF; // 24-bit address value
	transformAddress(addr, addrArray);

	// Write 3 bytes starting at given address
	EEPROM_CS_LOW();
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&__EEPROM_WRITE, 1, 100);
	HAL_SPI_Transmit(&hspi1, (uint8_t *) addrArray, 3, 100);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)spi_buf, 3, 100);
	EEPROM_CS_HIGH();

	// Clear buffer
	spi_buf[0] = 0;
	spi_buf[1] = 0;
	spi_buf[2] = 0;

	// Wait until WIP bit is cleared
	wip = 1;
	while (wip)
	{
		// Read status register
		EEPROM_CS_LOW();
		HAL_SPI_Transmit(&hspi1, (uint8_t *)&__EEPROM_RDSR, 1, 100);
		HAL_SPI_Receive (&hspi1, (uint8_t *)spi_buf, 1, 100);
		EEPROM_CS_HIGH();

		// Mask out WIP bit
		wip = spi_buf[0] & 0b00000001;
	}

	// Read the 3 bytes back
	EEPROM_CS_LOW();
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&__EEPROM_READ, 1, 100);
	HAL_SPI_Transmit(&hspi1, (uint8_t *) addrArray, 3, 100);
	HAL_SPI_Receive(&hspi1, (uint8_t *)spi_buf, 3, 100);
	EEPROM_CS_HIGH();

	// Print out bytes read
	uart_buf_len = sprintf(uart_buf,
			"%0x %0x %0x\r\n",
			(unsigned int)spi_buf[0],
			(unsigned int)spi_buf[1],
			(unsigned int)spi_buf[2]);
	HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);

	// Read status register
	EEPROM_CS_LOW();
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&__EEPROM_RDSR, 1, 100);
	HAL_SPI_Receive(&hspi1, (uint8_t *)spi_buf, 1, 100);
	EEPROM_CS_HIGH();

	// Print out status register
	uart_buf_len = sprintf(uart_buf,
			"Status: %0x\r\n",
			(unsigned int)spi_buf[0]);
	HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
}

static void test_eeprom_adv(void)
{
	/*
	 https://www.digikey.fr/en/maker/projects/getting-started-with-stm32-how-to-use-spi/09eab3dfe74c4d0391aaaa99b0a8ee17
	 */

	char uart_buf[50];
	int uart_buf_len;
	uint8_t spi_buf[20];
	uint32_t addr;

	// CS pin should default high
	EEPROM_CS_HIGH();

	// Say something
	uart_buf_len = sprintf(uart_buf, "SPI Test\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);

	// Enable write enable latch (allow write operations)
	sEE_WriteEnable();

	// Read status register
	spi_buf[0]=sEE_ReadStatusRegister();

	// Print out status register
	uart_buf_len = sprintf(uart_buf,
			"Status: %0x\r\n",
			(unsigned int)spi_buf[0]);
	HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);

	// Test bytes to write to EEPROM
	spi_buf[0] = 0xAB;
	spi_buf[1] = 0xCD;
	spi_buf[2] = 0xEF;

	// Set starting address
	addr = 0x00ABCDEF; // 24-bit address value

	// Write 3 bytes starting at given address
	EEPROM_SPI_WriteBuffer(spi_buf, addr, 3);

	// Clear buffer
	spi_buf[0] = 0;
	spi_buf[1] = 0;
	spi_buf[2] = 0;

	// Wait until WIP bit is cleared
    EEPROM_SPI_WaitStandbyState();

	// Read the 3 bytes back
    EEPROM_SPI_ReadBuffer(spi_buf, addr, 3);

	// Print out bytes read
	uart_buf_len = sprintf(uart_buf,
			"%0x %0x %0x\r\n",
			(unsigned int)spi_buf[0],
			(unsigned int)spi_buf[1],
			(unsigned int)spi_buf[2]);
	HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);

	// Read status register
	spi_buf[0]=sEE_ReadStatusRegister();

	// Print out status register
	uart_buf_len = sprintf(uart_buf,
			"Status: %0x\r\n",
			(unsigned int)spi_buf[0]);
	HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

	EEPROM_SPI_INIT(&hspi1);
	hal_status = HAL_SPIEx_FlushRxFifo(&hspi1);
	OnError_Handler(hal_status != HAL_OK);

	test_eeprom_adv();

	uint8_t wdata=0x66, rdata=0;
	EEPROM_WriteByte((uint32_t)0x00,  wdata);
	EEPROM_ReadByte( (uint32_t)0x00, &rdata);
	if (wdata != rdata) Error_Handler();


	EEPROM_Status status = EEPROM_SPI_WriteBuffer(TxBuffer, (uint32_t)0x00, (uint16_t)TXBUFFERSIZE);
	printf("string length %d\n\n", TXBUFFERSIZE);
    printf("Writing EEPROM Status %d\n", status);
	OnError_Handler(status != EEPROM_STATUS_COMPLETE);

	for (;;) {
		EEPROM_Status status = EEPROM_SPI_ReadBuffer(RxBuffer, (uint32_t)0x00, (uint16_t)RXBUFFERSIZE);
        printf("Reading EEPROM Status %d\n", status);
    	OnError_Handler(status != EEPROM_STATUS_COMPLETE);
		HAL_Delay(5000);
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  HAL_GPIO_WritePin(EEPROM_CS_GPIO_Port, EEPROM_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EEPROM_CS_Pin */
  GPIO_InitStruct.Pin = EEPROM_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EEPROM_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/************ Byte transfer functions ***************************/
EEPROM_Status EEPROM_WriteByte(uint32_t WriteAddr, uint8_t data) {
	EEPROM_Status pageWriteStatus = EEPROM_STATUS_PENDING;
	uint8_t pBuffer[1];
	pBuffer[0] = data;

	pageWriteStatus = EEPROM_SPI_WriteBuffer(pBuffer, WriteAddr, 1);

	return pageWriteStatus;
}

EEPROM_Status EEPROM_ReadByte(uint32_t ReadAddr, uint8_t *pBuffer) {
	EEPROM_Status pageWriteStatus = EEPROM_STATUS_PENDING;
	pageWriteStatus = EEPROM_SPI_ReadBuffer(pBuffer, ReadAddr, 1);

	return pageWriteStatus;
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
