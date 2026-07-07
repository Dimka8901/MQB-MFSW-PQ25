/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <string.h>
#include "buttons.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

// Set to 1 to self-test on the bench without a car: the board itself sends
// backlight-on and polls the wheel for buttons, filling debug variables
// (visible in Keil's Watch window) so you can confirm the firmware works
// before installing it in the vehicle. Set to 0 for normal operation in the car.
#define DEBUG_STANDALONE 0

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
IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint8_t buttonData[BUTTONS_LEN];
uint8_t statusData[STATUS_LEN];
uint8_t tempData[TEMP_LEN];
uint8_t buttonBuf[BUTTONS_LEN];
uint8_t statusBuf[STATUS_LEN];
uint8_t tempBuf[TEMP_LEN];
uint8_t diagRqBuf[DIAG_RQ_LEN];
HAL_StatusTypeDef sendResult;
HAL_StatusTypeDef readResult;

// Set by the ISR (fast, non-blocking) when the car has asked for fresh data.
// Actually serviced from the main while(1) loop, where blocking calls toward
// the wheel are safe and won't stall the time-critical response to the car.
volatile uint8_t needButtonsRefresh = 0;
volatile uint8_t needTempRefresh = 0;
volatile uint8_t needBacklightRefresh = 0;

// --- Live telemetry for in-car debugging via Watch window (SWD) ---
volatile uint32_t dbgBreakCount = 0;        // LIN breaks detected on UART2 (car side)
volatile uint32_t dbgSyncOkCount = 0;       // sync bytes accepted
volatile uint32_t dbgIdCount = 0;           // IDs received (any)
volatile uint8_t  dbgLastId = 0;            // last raw ID byte from the car
volatile uint32_t dbgIdButtons = 0;         // how many times car asked 0x8E
volatile uint32_t dbgIdStatus = 0;          // ... 0x0D
volatile uint32_t dbgIdTemp = 0;            // ... 0xBA
volatile uint32_t dbgIdDiag = 0;            // ... 0x3C / 0x7D
volatile uint32_t dbgIdUnknown = 0;         // IDs we don't handle
volatile uint32_t dbgWheelPollOk = 0;       // successful button polls toward the wheel
volatile uint32_t dbgWheelPollFail = 0;     // timed-out button polls
volatile uint8_t  dbgWheelBytesGot = 0;     // bytes received on the last failed poll
volatile uint8_t  dbgWheelRaw[9] = {0};     // last raw frame from the wheel
volatile uint8_t  dbgPqOut[9] = {0};        // last PQ frame handed to the car
volatile uint8_t  dbgSlaveStateSnapshot = 0;// slaveState as seen each main loop pass

#if DEBUG_STANDALONE
volatile uint8_t debugButtonFrame[9] = {0};
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
LIN_State slaveState = LIN_RECEIVING_BREAK;
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
	
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_LBD);
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
	__HAL_UART_DISABLE_IT(&huart1, UART_IT_LBD);
	HAL_NVIC_DisableIRQ(USART1_IRQn); // UART1 (toward the wheel) is fully blocking, no interrupts needed

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

#if DEBUG_STANDALONE

  while (1)
  {
		// 1) backlight, simulates ignition on - confirms basic LIN transport works
		uint8_t lightData[4] = { 0xFF, 0xFF, 0x00, 0x00 };
		SendBacklightToWheel(lightData, 4);

		HAL_Delay(20);

		// 2) same code path as in-car operation: poll wheel, decode, build PQ frame.
		//    Watch window: debugButtonFrame = raw frame from wheel,
		//                  buttonData      = converted PQ frame that would go to the car
		RefreshButtonsFromWheel();

		memcpy((void*)debugButtonFrame, buttonBuf, 9);

		HAL_Delay(80);
		HAL_IWDG_Refresh(&hiwdg);
  }

#else

	uint32_t lastHeartbeat = 0;

  while (1)
  {
		/* Normal operation: the car (USART2) drives everything via
		   HAL_UART_RxCpltCallback in stm32f1xx_it.c, which answers the car
		   immediately with cached data and just raises a flag here - actual
		   (blocking) communication with the wheel happens in this loop,
		   outside of any interrupt context, so it never delays the response
		   to the car. */

		dbgSlaveStateSnapshot = (uint8_t)slaveState;

		// non-blocking heartbeat: LED toggles every 500ms = firmware alive
		uint32_t now = HAL_GetTick();
		if (now - lastHeartbeat >= 500)
		{
			lastHeartbeat = now;
			HAL_GPIO_TogglePin(Led_GPIO_Port, Led_Pin);
		}

		if (needBacklightRefresh)
		{
			needBacklightRefresh = 0;
			// translate PQ 0x0D -> MQB 0x0D: live brightness from the car,
			// remaining bytes fixed to known-valid MQB values (from a real
			// MQB gateway capture). VW MQB Evo wheels validate this frame's
			// content and refuse to report buttons if it looks invalid -
			// forwarding the raw PQ bytes breaks them (Skoda wheels don't care).
			uint8_t mqbLight[4];
			mqbLight[0] = statusData[0];  // brightness 0x00-0x64, same semantics both sides
			mqbLight[1] = 0x81;           // ignition on (0x80 = off)
			mqbLight[2] = 0x64;
			mqbLight[3] = 0x40;
			SendBacklightToWheel(mqbLight, 4);
		}
		if (needButtonsRefresh)
		{
			needButtonsRefresh = 0;
			RefreshButtonsFromWheel();
		}
		if (needTempRefresh)
		{
			needTempRefresh = 0;
			RefreshTempFromWheel();
		}
  }

#endif

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
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
}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Reload = 1000;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 19200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  if (HAL_LIN_Init(&huart1, UART_LINBREAKDETECTLENGTH_11B) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  if (HAL_LIN_Init(&huart2, UART_LINBREAKDETECTLENGTH_11B) != HAL_OK)
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SLP2_Pin|SLP1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Led_Pin */
  GPIO_InitStruct.Pin = Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Led_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SLP2_Pin SLP1_Pin */
  GPIO_InitStruct.Pin = SLP2_Pin|SLP1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
HAL_StatusTypeDef linSend(UART_HandleTypeDef *huart, uint8_t id, uint8_t data[], uint8_t len)
{
	uint8_t linSync = LIN_SYNC_BYTE;
	uint8_t buf[len + 2];
	buf[0] = linSync;
	buf[1] = id;
	for (int i = 0; i < len; i++)
	{
		buf[i + 2] = data[i];
	}
	HAL_LIN_SendBreak(huart);
	HAL_StatusTypeDef result = HAL_UART_Transmit_IT(huart, buf, len + 2);
	return result;
}

HAL_StatusTypeDef linRead(UART_HandleTypeDef *huart, uint8_t id, uint8_t data[], uint8_t len)
{
	uint8_t linSync = LIN_SYNC_BYTE;
	uint8_t txbuf[2];
	txbuf[0] = linSync;
	txbuf[1] = id;
	HAL_LIN_SendBreak(huart);
	sendResult = HAL_UART_Transmit_IT(huart, txbuf, 2);
	if (sendResult != HAL_OK) {
		return sendResult;
	}
	readResult = HAL_UART_Receive_IT(huart, data, len);
	return readResult;
}

HAL_StatusTypeDef linRequest(UART_HandleTypeDef *huart, uint8_t id)
{
	uint8_t linSync = LIN_SYNC_BYTE;
	uint8_t txbuf[2];
	txbuf[0] = linSync;
	txbuf[1] = id;
	HAL_LIN_SendBreak(huart);
	sendResult = HAL_UART_Transmit_IT(huart, txbuf, 2);
	return sendResult;
}
uint8_t LIN_CalcCheckSum(uint8_t *id, uint8_t *data, uint8_t len)
{
  uint16_t sum = 0;
	sum += *id;
  
  for (uint8_t i = 0; i < len; i++)
  {
    sum += data[i];
  }
  
  while(sum > 0xFF)
  {
    sum -= 0xFF;
  }
  
  sum = 0xFF - sum;
  
  return (uint8_t)sum;
}

void blink(void)
{
	HAL_GPIO_TogglePin(Led_GPIO_Port, Led_Pin);
}

// --- Blocking master-side transactions toward the wheel (UART1) ---
// Called only from the main while(1) loop (normal mode) or the bench test
// loop (DEBUG_STANDALONE), never from interrupt context - see needXxxRefresh
// flags above for how the car-facing ISR defers this work here.

void RefreshButtonsFromWheel(void)
{
	uint8_t linSync = LIN_SYNC_BYTE;
	uint8_t header[2] = { linSync, BUTTONS_ID };
	uint8_t rxBuf[BUTTONS_LEN] = {0};

	HAL_LIN_SendBreak(&huart1);
	HAL_UART_Transmit(&huart1, header, 2, 10);

	// drain our own echo (break framing-error byte + anything pending) so the
	// received frame starts at the wheel's real byte0
	__HAL_UART_CLEAR_PEFLAG(&huart1);
	while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE))
	{
		volatile uint8_t dump = (uint8_t)(huart1.Instance->DR);
		(void)dump;
	}

	if (HAL_UART_Receive(&huart1, rxBuf, BUTTONS_LEN, 30) == HAL_OK)
	{
		dbgWheelPollOk++;
		memcpy(buttonBuf, rxBuf, BUTTONS_LEN);
		mqbToPq(buttonBuf, buttonData);
		memcpy((void*)dbgWheelRaw, buttonBuf, 9);
		memcpy((void*)dbgPqOut, buttonData, 9);
	}
	else
	{
		dbgWheelPollFail++;
		dbgWheelBytesGot = BUTTONS_LEN - (uint8_t)huart1.RxXferCount;
	}
}

void RefreshTempFromWheel(void)
{
	uint8_t linSync = LIN_SYNC_BYTE;
	uint8_t header[2] = { linSync, TEMP_ID };
	uint8_t rxBuf[TEMP_LEN] = {0};

	HAL_LIN_SendBreak(&huart1);
	HAL_UART_Transmit(&huart1, header, 2, 10);

	// same echo drain as in RefreshButtonsFromWheel
	__HAL_UART_CLEAR_PEFLAG(&huart1);
	while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE))
	{
		volatile uint8_t dump = (uint8_t)(huart1.Instance->DR);
		(void)dump;
	}

	if (HAL_UART_Receive(&huart1, rxBuf, TEMP_LEN, 30) == HAL_OK)
	{
		memcpy(tempBuf, rxBuf, TEMP_LEN);
		memcpy(tempData, tempBuf, TEMP_LEN);
	}
}

void SendBacklightToWheel(uint8_t* data, uint8_t len)
{
	uint8_t linSync = LIN_SYNC_BYTE;
	uint8_t header[2] = { linSync, STATUS_ID };
	uint8_t chkId = STATUS_ID;
	uint8_t checksum = LIN_CalcCheckSum(&chkId, data, len);

	HAL_LIN_SendBreak(&huart1);
	HAL_UART_Transmit(&huart1, header, 2, 10);
	HAL_UART_Transmit(&huart1, data, len, 10);
	HAL_UART_Transmit(&huart1, &checksum, 1, 10);
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
#endif /* USE_FULL_ASSERT */?