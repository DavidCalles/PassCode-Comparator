/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include <stdio.h>
#include <stdint.h>
#include "lcd.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_PASS_SIZE 4			// Max Length of passcodes
#define NUM_PASSCODES 10		// Length of passcodes database

#define MAX_NAME_SIZE 16		// Max Length of a name
#define MY_EOF 13
/*LED pin*/
#define LED_GPIO GPIOA		// Output GPIO
#define LED_PIN GPIO_PIN_12	// Output pin (Pin D2 on board)

/*LCD screen pins*/
#define D0_GPIO_Port GPIOB	// Data 0 GPIO port
#define D1_GPIO_Port GPIOB	// Data 1 GPIO port
#define D2_GPIO_Port GPIOB	// Data 2 GPIO port
#define D3_GPIO_Port GPIOB	// Data 3 GPIO port
#define D4_GPIO_Port GPIOA	// Data 4 GPIO port
#define D5_GPIO_Port GPIOA	// Data 5 GPIO port
#define D6_GPIO_Port GPIOB	// Data 6 GPIO port
#define D7_GPIO_Port GPIOB	// Data 7 GPIO port

#define D0_Pin GPIO_PIN_0	// Data 0 pin (D3 on board)
#define D1_Pin GPIO_PIN_7	// Data 0 pin (D4 on board)
#define D2_Pin GPIO_PIN_6	// Data 0 pin (D5 on board)
#define D3_Pin GPIO_PIN_1	// Data 0 pin (D6 on board)
#define D4_Pin GPIO_PIN_8	// Data 0 pin (D9 on board)
#define D5_Pin GPIO_PIN_11	// Data 0 pin (D10 on board)
#define D6_Pin GPIO_PIN_5	// Data 0 pin (D11 on board)
#define D7_Pin GPIO_PIN_4	// Data 0 pin (D12 on board)

#define RS_GPIO_Port GPIOA		 // Register select GPIO port
#define RS_Pin 		 GPIO_PIN_9  // Register select pin (D1)
#define EN_GPIO_Port GPIOA		 // Enable GPIO port
#define EN_Pin		 GPIO_PIN_10 // Register select pin (D0)
/* USER CODE END PD */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Structure for each passcode in database
typedef struct {
	char pass[MAX_PASS_SIZE];
	char name[MAX_NAME_SIZE];
}PASSCODE;

// Enum for each state of the fsm
typedef enum {
	RECEIVING_PASS 	= 0,
	CHECKING_PASS	= 1,
	COMPARING_PASS 	= 2,
	GIVING_ACCESS	= 3
}STATE_T;

// Enum for comparating 2 strings
typedef enum {
	EQUAL 		= 0,
	NOT_EQUAL	= 1
}COMPARISON;


/* USER CODE END PTD */

/* Hardware handles ----------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac_ch1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */
COMPARISON StringCompare(char *, char *, uint8_t);

/* USER CODE END PFP */


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	/* USER CODE BEGIN 1 */

	// GPIO Ports for LCD
	Lcd_PortType ports[] = {
		D0_GPIO_Port, D1_GPIO_Port, D2_GPIO_Port, D3_GPIO_Port,
		D4_GPIO_Port, D5_GPIO_Port, D6_GPIO_Port, D7_GPIO_Port
	 };
	// GPIO Pins for LCD
	Lcd_PinType pins[] = {D0_Pin, D1_Pin, D2_Pin, D3_Pin,
						  D4_Pin, D5_Pin, D6_Pin, D7_Pin};

	// Database with stored passcodes and names
	static PASSCODE db[NUM_PASSCODES] = {
			{"0123", "David Calles"},
			{"9999", "Allan Smith"},
			{"0001", "Visitor1"},
			{"0002", "Visitor2"},
			{"0003", "Visitor3"},
			{"0004", "Visitor4"},
			{"0005", "Visitor5"},
			{"0006", "Visitor6"},
			{"0007", "Visitor7"},
			{"0008", "Visitor8"}
	};
	// Initial setup for fsm
	STATE_T state = RECEIVING_PASS;	//Initiate in state 0
	uint8_t n = 0;					//Index of input character
	int16_t ch;						//Input character
	uint8_t cPass = 0;				//Index of checked passcode
	COMPARISON result = NOT_EQUAL;	//Result of comparing 2 strings
	char myPass[MAX_PASS_SIZE];		//Input passcode

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
	/* Configure the system clock */
	SystemClock_Config();
	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_DAC1_Init();
	MX_USART2_UART_Init();
	MX_TIM2_Init();

	// Create handler for the LCD
	Lcd_HandleTypeDef lcd;
	lcd = Lcd_create(ports, pins,
				  RS_GPIO_Port, RS_Pin,
				  EN_GPIO_Port, EN_Pin,
				  LCD_8_BIT_MODE);
	/* USER CODE BEGIN 2 */
	printf("Hello User!\r\n");
	/* USER CODE END 2 */
	Lcd_clear(&lcd);
	Lcd_cursor(&lcd, 0,3);
	Lcd_string(&lcd, "Pass Code");
	Lcd_cursor(&lcd, 1,0);
	Lcd_string(&lcd, "by David Calles!");
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		switch(state){
			// State 0
			case RECEIVING_PASS:{
				// Pass within correct size
				if(n < (MAX_PASS_SIZE+1)){
					ch = getchar();	// Get character
					state = CHECKING_PASS;
				}
				// Pass bigger than correct size
				else{
					printf("Input is too long, Try again!: %c \r\n",ch);
					n = 0;
					state = RECEIVING_PASS;
				}
				break;
			}// State 0

			// State 1
			case CHECKING_PASS:{
				// Normal character received
				if( ch != MY_EOF ){
					myPass[n] = (uint8_t)ch;
					++n;
					state = RECEIVING_PASS;
				}
				// "Enter" received and size is appropriate
				else if( (ch == MY_EOF) && (n == (MAX_PASS_SIZE)) ){
					cPass = 0;
					printf("Input is: %s \r\n", myPass);
					printf("Elements: %d \r\n", sizeof(myPass));
					state = COMPARING_PASS;
				}
				// "Enter" received and size is not appropriate
				else{
					printf("Input is too short, Try again! \r\n");
					n = 0;
					state = RECEIVING_PASS;
				}
				break;
			}// State 1

			// State 2
			case COMPARING_PASS:{
				// Not all passwords have been checked
				if( cPass < NUM_PASSCODES ){
					// Compare strings
					result = StringCompare(myPass, db[cPass].pass, MAX_PASS_SIZE);
					state = GIVING_ACCESS;
				}
				// All passwords have been checked
				else{
					printf("Pass-code Incorrect! \r\n");
					//RED signal
					cPass = 0;
					n = 0;
					state = RECEIVING_PASS;
				}
				break;
			}// State 2

			// State 3
			case GIVING_ACCESS:{
				// Pass-codes are not equal
				if(result == NOT_EQUAL){
					++cPass;
					state = COMPARING_PASS;
				}
				// Pass-codes are equal
				else{
					printf("Pass Accepted, Hello %s! \r\n", db[cPass].name);
					printf("Press any key to reset. \r\n");
					//GREEN signal
					ch = getchar();
					printf("Hello User!. \r\n");
					n = 0;
					cPass = 0;
					result = NOT_EQUAL;
					state = RECEIVING_PASS;
				}
				break;
			}// State 3

			// Default: Go to state 0
			default:
				state = RECEIVING_PASS;
		}

	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}
/*
 *
 * Compare strings
 * */
COMPARISON StringCompare(char *a, char *b, uint8_t n){
	COMPARISON c = EQUAL;
	for (int i=0; i<n; i++){
		if(a[i] != b[i]){
			c = NOT_EQUAL;
		}
	}
	return c;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 32;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|LD3_Pin|GPIO_PIN_4
						  |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
						  |GPIO_PIN_12, GPIO_PIN_RESET);

	/*Configure LED Output level*/
	HAL_GPIO_WritePin(LED_GPIO, LED_PIN, GPIO_PIN_RESET);

	/*Configure GPIO pins : PB0 PB1 LD3_Pin PB4
						   PB5 PB6 PB7 */
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|LD3_Pin|GPIO_PIN_4
						  |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
	if(LED_GPIO == GPIOB) // Add Led pin if part of GPIOB
	  GPIO_InitStruct.Pin |= LED_PIN;

	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PA8 PA9 PA10 PA11
						   PA12 */
	GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
						  |GPIO_PIN_12;
	if(LED_GPIO == GPIOA)// Add Led pin if part of GPIOA
	  GPIO_InitStruct.Pin |= LED_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : VCP_RX_Pin */
	GPIO_InitStruct.Pin = VCP_RX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF3_USART2;
	HAL_GPIO_Init(VCP_RX_GPIO_Port, &GPIO_InitStruct);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
