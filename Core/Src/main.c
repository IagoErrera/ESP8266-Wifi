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

#define PORT 80
#define MAX_BUFFER_SIZE 500

#define SSID "HOME"
#define PASSWORD "gemeos123"

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
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

char r_data[MAX_BUFFER_SIZE];
char t_data[MAX_BUFFER_SIZE];
uint8_t n = 0;

int buffer_start = 0;
int status = 0;


#define RESPONSE "HTTP/1.1 200 OK\nContent-Type: application/json\nContent-Length: 14\n\n{status: \"OK\"}"
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int find_string(char* a, char* b, int start) {
	int i = start % MAX_BUFFER_SIZE, j;
	int end = (start + MAX_BUFFER_SIZE - 1) % MAX_BUFFER_SIZE;
	while (a[i] != '\0' && i != end) {
		j = 0;
		while (1) {
			if (b[j] == '\0') return i;
			if (a[i + j] != b[j]) break;

			j++;
		}

		i++;
		if (i == MAX_BUFFER_SIZE) i = 0;
	}

	return -1;
}

int wait_ok() {
	int found = -1;
	while(found == -1) {
		found = find_string(r_data, "\r\nOK\r\n", buffer_start);
		if (found > 0) return found + 6;

		found = find_string(r_data, "\r\nno change\r\n", buffer_start);
		if (found > 0) return found + 13;
	}

	return -1;
}

int wait(char* str) {
	int found = -1;

	while (found == -1) {
		found = find_string(r_data, str, buffer_start);
		if (found > 0) return found;
	}

	return -1;
}

void wifi_init(void) {
	HAL_GPIO_WritePin(wifi_rst_GPIO_Port, wifi_rst_Pin, 1);
	HAL_Delay(2000);

	HAL_UART_Receive_DMA(&huart1, (uint8_t*)r_data, MAX_BUFFER_SIZE);

	n = sprintf(t_data, "AT\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)t_data, n, 100);

	buffer_start = wait_ok();

	status = 1;

	n = sprintf(t_data, "AT+CWMODE=3\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)t_data, n, 100);

	buffer_start = wait_ok();

	status = 2;

	n = sprintf(t_data, "AT+CWJAP=\"%s\",\"%s\"\r\n", SSID, PASSWORD);
	HAL_UART_Transmit(&huart1, (uint8_t*)t_data, n, 100);

	buffer_start = wait_ok();

	status = 3;

	n = sprintf(t_data, "AT+CIPMUX=1\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)t_data, n, 100);

	buffer_start = wait_ok();

	status = 4;

	n = sprintf(t_data, "AT+CIPSERVER=1,%d\r\n", PORT);
	HAL_UART_Transmit(&huart1, (uint8_t*)t_data, n, 100);

	buffer_start = wait_ok();

	status = 5;

	for (int i = 0; i < MAX_BUFFER_SIZE; i++) r_data[i] = '\0';
}

// Request -> Link\r\n\r\n+IPD,<channel>,<length>;<Request>\r\n<RequesterInfo>\r\n<Status>\r\n[...]
int handle_request(int* channel, int* length) {
	int found = find_string(r_data,"+IPD", buffer_start);
	if (found == -1) return -1;

	buffer_start = (found + 5) % MAX_BUFFER_SIZE;

	*channel = 0;
	while (r_data[buffer_start] != ',') {
		(*channel) = (*channel) * 10 + r_data[buffer_start++] - 48;
		buffer_start = buffer_start % MAX_BUFFER_SIZE;
	}
	buffer_start++; buffer_start = buffer_start % MAX_BUFFER_SIZE;

	(*length) = 0;
	while (r_data[buffer_start] != ':') {
		(*length) = (*length) * 10 + r_data[buffer_start++] - 48;
		buffer_start = buffer_start % MAX_BUFFER_SIZE;
	}
	buffer_start++; buffer_start = buffer_start % MAX_BUFFER_SIZE;

	return found;
}

void handle_route(char* end_point, int n, int channel, char* response, int rn) {
	int found = -1;
	if ((found = find_string(r_data, end_point, buffer_start)) == -1) return;
	buffer_start = (found + n + 11) % MAX_BUFFER_SIZE;

	char data[32];
	int size = sprintf(data, "AT+CIPSEND=%d,%d\r\n", channel, rn);

	HAL_UART_Transmit(&huart1, data, size, 100);

	found = wait(">");
	buffer_start = (found + 2) % MAX_BUFFER_SIZE;

	HAL_UART_Transmit(&huart1, response, rn, 100);

	found = wait("\r\nSEND OK\r\n");

	buffer_start = (found + 11) % MAX_BUFFER_SIZE;

	for (int i = 0; i < MAX_BUFFER_SIZE; i++) r_data[i] = '\0';
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

	wifi_init();

	char data_response[256];

	int data_len = sprintf(data_response, "AT+CIFSR\r\n");
	HAL_UART_Transmit(&huart1, data_response, data_len, 100);

	data_len = sprintf(data_response, RESPONSE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	int channel, length, found;
	while (1){
		found = -1;
		if ((found = wait("GET /test")) > -1) {
			HAL_Delay(100);
			handle_request(&channel, &length);
			handle_route("GET /test", 9, channel, data_response, data_len);

			status = 10;
		}

		if (handle_request(&channel, &length) == -1) continue;

		handle_route("GET /test", 9, channel, data_response, data_len);

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, wifi_rst_Pin|LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : wifi_rst_Pin LED_Pin */
  GPIO_InitStruct.Pin = wifi_rst_Pin|LED_Pin;
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
