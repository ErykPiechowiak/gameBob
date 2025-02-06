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
#include "string.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "lcd.h"
#include "ugui.h"
#include "ugui_colors.h"
#include "user.h"
#include "pong.h"
#include "pitches.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DEFAULT_FONT FONT_6X8
#define TIM_FREQ 64000000
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//test
static PLAYER player1, player2;
static BALL ball;
static USER_INPUT uInput;
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );
void ADC_SetActiveChannel(ADC_HandleTypeDef *hadc, uint32_t AdcChannel);
static void initUserInput(USER_INPUT *uInput);



/* TASK FUNCTIONS */
static void testTask( void *pvParameters );
static void taskDisplayUpdate(void *pvParameters);
static void taskGetUserInput(void *pvParameters);
static void taskGameLogic(void *pvParameters);

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
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable DWT
	DWT->CYCCNT = 0;                                // Clear counter
	DWT->CTRL = DWT_CTRL_CYCCNTENA_Msk;             // Enable counter

	BaseType_t xReturned;
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
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  LCD_init();
  initUserInput(&uInput);

/*
  int STmelody[] = {
		  0,0,0,0,0,0,311,0,311,0,246,0,246,0,123,0,123,0,77,0,38,0,38,0,38,0,38,0,51,0,554,0,622,0,659,0,622,0,554,0,622,0,554,0,493,0,466,0,415,0,391,0,415,0,311,0,329,0,311,0,329,0,311,0,329,0,311,0,391,0,493,0,466,0,415,0,415,0,466,0,493,0,466,0,493,0,493,0,523,0,554,0,493,0,466,0,391,0,415,0,466,0,493,0,466,0,415,0,311,0,311,0,329,0,311,0,329,0,311,0,329,0,311,0,493,0,466,0,391,0,415,0,415,0,391,0,415,0,739,0,783,0,830,0,554,0,622,0,659,0,622,0,554,0,622,0,554,0,493,0,466,0,415,0,391,0,415,0,311,0,329,0,311,0,329,0,311,0,329,0,311,0,391,0,493,0,466,0,415,0,415,0,466,0,493,0,466,0,493,0,466,0,523,0,554,0,493,0,466,0,391,0,415,0,466,0,493,0,466,0,415,0,311,0,311,0,329,0,311,0,329,0,311,0,329,0,311,0,493,0,466,0,391,0,415,0,415,0,391,0,415,0,493,0,554,0,622,0,659,0,493,0,415,0,329,0,391,0,311,0,622,0,587,0,554,0,466,0,391,0,311,0,415,0,391,0,415,0,466,0,415,0,466,0,493,0,493,0,493,0,554,0,622,0,739,0,830,0,739,0,659,0,554,0,622,0,783,0,466,0,622,0,554,0,554,0,554,0,659,0,622,0,622,0,622,0,659,0,622,0,587,0,622,0,554,0,622,0,659,0,622,0,554,0,622,0,554,0,493,0,466,0,415,0,391,0,415,0,311,0,329,0,311,0,329,0,311,0,329,0,311,0,391,0,493,0,466,0,415,0,415,0,466,0,493,0,466,0,493,0,493,0,523,0,554,0,493,0,466,0,391,0,415,0,466,0,493,0,466,0,415,0,311,0,311,0,329,0,311,0,329,0,311,0,329,0,311,0,493,0,466,0,391,0,415,0,415,0,391,0,415,0,739,0,783,0,830,0,554,0,622,0,659,0,622,0,554,0,622,0,554,0,493,0,466,0,415,0,391,0,415,0,311,0,329,0,311,0,329,0,311,0,329,0,311,0,391,0,493,0,466,0,415,0,415,0,466,0,493,0,466,0,493,0,466,0,523,0,554,0,493,0,466,0,391,0,415,0,466,0,493,0,466,0,415,0,311,0,311,0,329,0,311,0,329,0,311,0,329,0,311,0,493,0,466,0,391,0,415,0,415,0,391,0,415,0
  };
	int STnoteDurations[] = {
			0,0,0,0,0,0,0,390,391,455,521,651,782,911,1043,1433,1434,1496,1565,2085,2086,2477,2478,2540,2608,2738,2869,2998,3130,3651,4173,4238,4239,4303,4304,4368,4434,4557,4565,4688,4695,4824,4956,5079,5086,5209,5217,5346,5478,5601,5608,5731,5739,5868,5999,6122,6130,6253,6260,6324,6391,6455,6521,6585,6652,6716,6782,6846,6913,7035,7043,7166,7173,7296,7304,7433,7565,7688,7695,7818,7826,7955,8086,8209,8217,8340,8347,8411,8413,8477,8478,8542,8608,8731,8739,8861,8869,8998,9130,9253,9260,9383,9391,9520,9652,9774,9782,9905,9913,10159,10173,10296,10304,10427,10434,10498,10565,10629,10695,10759,10826,10890,10956,11020,11086,11209,11217,11340,11347,11470,11478,11607,11739,11861,11869,11992,11999,12129,12260,12324,12326,12390,12391,12514,12521,12585,12586,12651,12652,12716,12782,12905,12913,13035,13043,13172,13304,13427,13434,13557,13565,13694,13826,13948,13956,14079,14086,14216,14347,14470,14478,14601,14608,14672,14739,14803,14869,14933,14999,15064,15130,15194,15260,15383,15391,15514,15521,15644,15652,15781,15913,16035,16043,16166,16173,16303,16434,16557,16565,16688,16695,16759,16760,16824,16826,16890,16956,17079,17086,17209,17217,17346,17478,17601,17608,17731,17739,17868,17999,18122,18130,18253,18260,18507,18521,18644,18652,18774,18782,18846,18913,18977,19043,19107,19173,19238,19304,19368,19434,19557,19565,19688,19695,19818,19826,20072,20086,20209,20217,20340,20347,20470,20478,20601,20608,20731,20739,20861,20869,21390,21652,21898,21913,22694,22695,22942,22956,23477,23478,23998,23999,24520,24521,25042,25043,25824,25826,26072,26086,26868,26869,27116,27130,27259,27260,27383,27391,27638,27652,27781,27782,27905,27913,28159,28173,28564,28565,28688,28695,28818,28826,28948,28956,29079,29086,29209,29217,29998,29999,30246,30260,31042,31043,31290,31304,31824,31826,32346,32347,32868,32869,33390,33391,34303,34304,34427,34434,34955,34956,35477,35478,36390,36391,36514,36521,36644,36652,36774,36782,36905,36913,37035,37043,37538,37565,37629,37630,37694,37695,37759,37826,37948,37956,38079,38086,38216,38347,38470,38478,38601,38608,38738,38869,38992,38999,39122,39130,39259,39391,39514,39521,39644,39652,39716,39782,39846,39913,39977,40043,40107,40173,40238,40304,40427,40434,40557,40565,40688,40695,40824,40956,41079,41086,41209,41217,41346,41478,41601,41608,41731,41739,41803,41804,41868,41869,41933,41999,42122,42130,42253,42260,42390,42521,42644,42652,42774,42782,42911,43043,43166,43173,43296,43304,43551,43565,43688,43695,43818,43826,43890,43956,44020,44086,44151,44217,44281,44347,44411,44478,44601,44608,44731,44739,44861,44869,44998,45130,45253,45260,45383,45391,45520,45652,45716,45717,45781,45782,45905,45913,45977,45978,46042,46043,46107,46173,46296,46304,46427,46434,46564,46695,46818,46826,46948,46956,47085,47217,47340,47347,47470,47478,47607,47739,47861,47869,47992,47999,48064,48130,48194,48260,48324,48391,48455,48521,48585,48652,48774,48782,48905,48913,49035,49043,49172,49304,49427,49434,49557,49565,49694,49826,49948,49956,50079,50086,50151,50152,50216,50217,50281,50347,50470,50478,50601,50608,50738,50869,50992,50999,51122,51130,51259,51391,51514,51521,51644,51652,51898,51913,52035,52043,52166,52173,52238,52304,52368,52434,52498,52565,52629,52695,52759,52826,52948,52956,53079,53086,53209,53217,53464,53478,53601,53608,53731,53739,54233
	};


	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	playTone(STmelody, STnoteDurations, NULL, (sizeof(STmelody)/sizeof(STmelody[0])));
	*/
  //LCD_Test();

  xReturned = xTaskCreate(testTask, "task", 256, NULL, 1, NULL);
//  xReturned = xTaskCreate(taskDisplayUpdate,"taskDisplayUpdate",256,NULL,3,NULL);
  xReturned = xTaskCreate(taskGetUserInput,"taskGetUserInput",256,NULL,2,NULL);
  xReturned = xTaskCreate(taskGameLogic, "taskGameLogic", 512, NULL, 3, NULL);
  vTaskStartScheduler();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10006;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_DC_Pin|LCD_RST_Pin|LCD_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LEFT_ANALOG_KEY_Pin */
  GPIO_InitStruct.Pin = LEFT_ANALOG_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LEFT_ANALOG_KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RIGHT_ANALOG_KEY_Pin */
  GPIO_InitStruct.Pin = RIGHT_ANALOG_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(RIGHT_ANALOG_KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_DC_Pin */
  GPIO_InitStruct.Pin = LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LCD_DC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RST_Pin LCD_CS_Pin */
  GPIO_InitStruct.Pin = LCD_RST_Pin|LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void ADC_SetActiveChannel(ADC_HandleTypeDef *hadc, uint32_t AdcChannel)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = AdcChannel;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
  {
   Error_Handler();
  }
}


static void testTask( void *pvParameters ){

	for(;;){
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
/*
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	    __HAL_TIM_SET_PRESCALER(&htim1, presForFrequency(1000));
	    vTaskDelay(250);
	    __HAL_TIM_SET_PRESCALER(&htim1, presForFrequency(3000));
	    vTaskDelay(250);
	    __HAL_TIM_SET_PRESCALER(&htim1, presForFrequency(6000));
	    vTaskDelay(250);
	    __HAL_TIM_SET_PRESCALER(&htim1, presForFrequency(9000));
	    vTaskDelay(250);
	    __HAL_TIM_SET_PRESCALER(&htim1, presForFrequency(12000));
*/
	    vTaskDelay(500);

	}
}


static void taskGameLogic(void *pvParameters){
	//initGame(&player1, &player2, &ball);
	initGame(&htim1);
	for(;;){
		gameInput(uInput);
		gameLogic();
		vTaskDelay(33);
	}
}

static void taskDisplayUpdate(void *pvParameters){
//	uint16_t x1 =0;
//	uint16_t x2 =60;
//	uint16_t y1 =120;
//	uint16_t y2 =120;
//	uint16_t acc = 0;
	UG_FillScreen(C_BLACK);
//	UG_DrawLine(x1, y1, x2, y2, C_WHITE);
	UG_Update();
	for(;;){

		if(player1.acc>0){
			UG_DrawLine(player1.x1-player1.acc, player1.y-1, player1.x1-1, player1.y-1, C_BLACK);
			UG_DrawLine(player1.x1-player1.acc, player1.y, player1.x1-1, player1.y, C_BLACK);
		}
		else{
			UG_DrawLine(player1.x2+1, player1.y-1, player1.x2-player1.acc, player1.y-1, C_BLACK);
			UG_DrawLine(player1.x2+1, player1.y, player1.x2-player1.acc, player1.y, C_BLACK);
		}
		UG_DrawLine(player1.x1, player1.y-1, player1.x2, player1.y-1, C_WHITE);
		UG_DrawLine(player1.x1, player1.y, player1.x2, player1.y, C_WHITE);
		UG_Update();
		vTaskDelay(33);


		/*
		if(uInput.leftXAxis > 2100 && x2 < LCD_WIDTH-10){
			acc = (uInput.leftXAxis - 2000)/100;
			x1+=acc;
			x2+=acc;
			UG_DrawLine(x1, y1-1, x2, y2-1, C_WHITE);
			UG_DrawLine(x1, y1, x2, y2, C_WHITE);
			UG_DrawLine(x1-acc, y1-1, x1, y2-1, C_BLACK);
			UG_DrawLine(x1-acc, y1, x1, y2, C_BLACK);
			UG_Update();
			vTaskDelay(33);
		}
		else if(uInput.leftXAxis < 1900 && x1 > 0+10){
			acc = (2000-uInput.leftXAxis)/100;
			x1-=acc;
			x2-=acc;
			UG_DrawLine(x1, y1-1, x2, y2-1, C_WHITE);
			UG_DrawLine(x1, y1, x2, y2, C_WHITE);
			UG_DrawLine(x2, y1-1, x2+acc, y2-1, C_BLACK);
			UG_DrawLine(x2, y1, x2+acc, y2, C_BLACK);
			UG_Update();
			vTaskDelay(33);

			*/



		}
		/*
		while(x2<240){
			x1+=5;
			x2+=5;
			UG_DrawLine(x1, y1, x2, y2, C_WHITE);
			UG_DrawLine(x1-5, y1, x1, y2, C_BLACK);
			UG_Update();
			vTaskDelay(33);
		}
		while(x1>0){
			x1-=5;
			x2-=5;
			UG_DrawLine(x1, y1, x2, y2, C_WHITE);
			UG_DrawLine(x2, y1, x2+5, y2, C_BLACK);
			UG_Update();
			vTaskDelay(33);
		}
		*/


}

static void taskGetUserInput(void *pvParameters){
	uint16_t joystick[4] = {0}; //0 - X axis ; 1 - Y axis
	char buffer[20];
	uint32_t adc_channel = ADC_CHANNEL_6;
	uint32_t adc_channels[4] = {ADC_CHANNEL_6,ADC_CHANNEL_7,ADC_CHANNEL_8,ADC_CHANNEL_9};
	for(;;){
		for(int i=0;i<4;i++){
			//adc_channel = i == 0 ? ADC_CHANNEL_6 : ADC_CHANNEL_7;
			ADC_SetActiveChannel(&hadc1, adc_channels[i]);
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 10);
			joystick[i] = HAL_ADC_GetValue(&hadc1);
		}
		uInput.leftXAxis = joystick[0];
		uInput.leftYAxis = joystick[1];
		uInput.rightXAxis = joystick[2];
		uInput.rightYAxis = joystick[3];
		//sprintf(buffer,"X=%lu Y=%lu",joystick[0],joystick[1]);
		//HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);
		uInput.leftAnalogKey = HAL_GPIO_ReadPin(LEFT_ANALOG_KEY_GPIO_Port, LEFT_ANALOG_KEY_Pin);
		uInput.rightAnalogKey = HAL_GPIO_ReadPin(RIGHT_ANALOG_KEY_GPIO_Port, RIGHT_ANALOG_KEY_Pin);
		vTaskDelay(33);
	}
}

static void initUserInput(USER_INPUT *uInput){
	uInput->leftXAxis = 2000;
	uInput->leftYAxis = 2000;
	uInput->rightXAxis = 2000;
	uInput->rightYAxis = 2000;
}



void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}




/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
