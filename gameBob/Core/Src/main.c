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
#include "images.h"
#include "main_menu.h"
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
#define MAX_OBJECTS 10 //uGUI
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
static USER_INPUT uInputOld;
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];



//uGUI
static UG_WINDOW window_1;
static UG_BUTTON button_start_game, button_select_game;
static UG_TEXTBOX textbox_1;
static UG_OBJECT obj_buff_wnd_1[MAX_OBJECTS];
static UG_PROGRESS pgb;
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
static void updateSelectedOption(enum ActiveGameBobOption selected_menu_option);
static void drawMainMenu();

//uGUI
static void window_1_callback(UG_MESSAGE *msg);


/* TASK FUNCTIONS */
static void testTask( void *pvParameters );
static void taskMainMenu(void *pvParameters);
static void taskGetUserInput(void *pvParameters);
static void taskGameLogic(void *pvParameters);
TaskHandle_t h_task_main_menu = NULL;
TaskHandle_t h_task_game_logic = NULL;

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


  xReturned = xTaskCreate(testTask, "task", 256, NULL, 1, NULL);
  xReturned = xTaskCreate(taskMainMenu,"taskMainMenu",256,NULL,3,&h_task_main_menu);
  xReturned = xTaskCreate(taskGetUserInput,"taskGetUserInput",256,NULL,2,NULL);


  //xReturned = xTaskCreate(taskGameLogic, "taskGameLogic", 512, NULL, 3, NULL);
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
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

  /*Configure GPIO pins : RIGHT_ANALOG_KEY_Pin KEY_DOWN_Pin KEY_UP_Pin */
  GPIO_InitStruct.Pin = RIGHT_ANALOG_KEY_Pin|KEY_DOWN_Pin|KEY_UP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY_B_Pin KEY_A_Pin */
  GPIO_InitStruct.Pin = KEY_B_Pin|KEY_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
	    vTaskDelay(500);
	}
}


static void taskGameLogic(void *pvParameters){

	initGame(h_task_main_menu,&htim1,uInput);
	for(;;){
		gameInput(uInput);
		gameLogic();
		vTaskDelay(33);
	}
}

static void drawMainMenu(){
	UG_FillScreen(C_WHITE);
	UG_FillFrame(0, 0, LCD_WIDTH, 30, C_PURPLE);
	char text_buffer[25];
	sprintf(text_buffer, "Welcome to GameBob!");
	LCD_PutStr(10, 10, text_buffer, FONT_12X20, C_WHITE, C_PURPLE);
	UG_DrawBMP(LCD_WIDTH/2-42, LCD_HEIGHT/2-62, &play_game_pic);
	UG_DrawBMP(LCD_WIDTH/2-42, LCD_HEIGHT/2, &load_game_pic);
	UG_DrawBMP(LCD_WIDTH/2-42, LCD_HEIGHT/2+62, &power_off_pic);
	UG_Update();
}

static void taskMainMenu(void *pvParameters){

	static enum ActiveGameBobOption selected_menu_option = START_GAME;

	drawMainMenu();
	updateSelectedOption(selected_menu_option);

	 UG_WindowShow(&window_1);
	 UG_Update();
	 BaseType_t xReturned;
	 uint8_t game_started_flag = 0;
	 for(;;){

		if(game_started_flag == 0 && selected_menu_option == PLAY_GAME && uInput.keyB == GPIO_PIN_RESET){
			 xReturned = xTaskCreate(taskGameLogic, "taskGameLogic", 512, NULL, 3, &h_task_game_logic);
			 vTaskSuspend(NULL);
			 drawMainMenu();
			 updateSelectedOption(selected_menu_option);
		 }
		if(uInput.keyDown == GPIO_PIN_RESET && uInput.keyDown != uInputOld.keyDown){
			if(selected_menu_option < POWER_OFF){
				selected_menu_option++;
				updateSelectedOption(selected_menu_option);
			}
		}
		else if(uInput.keyUp == GPIO_PIN_RESET && uInput.keyUp != uInputOld.keyUp){
			if(selected_menu_option > PLAY_GAME){
				selected_menu_option--;
				updateSelectedOption(selected_menu_option);
			}
		}



		 vTaskDelay(33);
	 }


}

static void taskGetUserInput(void *pvParameters){
	/* Check analog input */
	uint16_t joystick[4] = {0}; //0 - X axis ; 1 - Y axis
	char buffer[20];
	uint32_t adc_channel = ADC_CHANNEL_6;
	uint32_t adc_channels[4] = {ADC_CHANNEL_6,ADC_CHANNEL_7,ADC_CHANNEL_8,ADC_CHANNEL_9};
	for(;;){
		for(int i=0;i<4;i++){
			ADC_SetActiveChannel(&hadc1, adc_channels[i]);
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 10);
			joystick[i] = HAL_ADC_GetValue(&hadc1);
		}

		uInputOld = uInput;

		uInput.leftXAxis = joystick[0];
		uInput.leftYAxis = joystick[1];
		uInput.rightXAxis = joystick[2];
		uInput.rightYAxis = joystick[3];

		/* Check analog key press */
		uInput.leftAnalogKey = HAL_GPIO_ReadPin(LEFT_ANALOG_KEY_GPIO_Port, LEFT_ANALOG_KEY_Pin);
		uInput.rightAnalogKey = HAL_GPIO_ReadPin(RIGHT_ANALOG_KEY_GPIO_Port, RIGHT_ANALOG_KEY_Pin);

		/* Check buttons state */
		uInput.keyDown = HAL_GPIO_ReadPin(KEY_DOWN_GPIO_Port, KEY_DOWN_Pin);
		uInput.keyUp = HAL_GPIO_ReadPin(KEY_UP_GPIO_Port, KEY_UP_Pin);
		uInput.keyB = HAL_GPIO_ReadPin(KEY_B_GPIO_Port, KEY_B_Pin);
		uInput.keyA = HAL_GPIO_ReadPin(KEY_A_GPIO_Port, KEY_A_Pin);

		/* Get seed */
		ADC_SetActiveChannel(&hadc1, ADC_CHANNEL_10);
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 10);
		uInput.seed = HAL_ADC_GetValue(&hadc1);



		vTaskDelay(33);
	}
}

static void initUserInput(USER_INPUT *uInput){
	uInput->leftXAxis = 2000;
	uInput->leftYAxis = 2000;
	uInput->rightXAxis = 2000;
	uInput->rightYAxis = 2000;
	uInput->keyLeft = GPIO_PIN_SET;
	uInput->keyRight = GPIO_PIN_SET;
	uInput->keyUp = GPIO_PIN_SET;
	uInput->keyDown = GPIO_PIN_SET;
	uInput->keyA = GPIO_PIN_SET;
	uInput->keyB = GPIO_PIN_SET;

	ADC_SetActiveChannel(&hadc1, ADC_CHANNEL_10);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);
	uInput->seed = HAL_ADC_GetValue(&hadc1);


}

static void updateSelectedOption(enum ActiveGameBobOption selected_menu_option){
	uint16_t option_colors[3] = {C_WHITE, C_WHITE, C_WHITE};

	for(int i=0; i<3; i++){
		selected_menu_option == i ? option_colors[i] = C_PURPLE : C_WHITE;
	}

	UG_DrawFrame(LCD_WIDTH/2-42, LCD_HEIGHT/2-62, LCD_WIDTH/2-42+play_game_pic.width, LCD_HEIGHT/2-62+play_game_pic.height, option_colors[0]);
	UG_DrawFrame(LCD_WIDTH/2-42-1, LCD_HEIGHT/2-62-1, LCD_WIDTH/2-42+play_game_pic.width+1, LCD_HEIGHT/2-62+1+play_game_pic.height, option_colors[0]);

	UG_DrawFrame(LCD_WIDTH/2-42, LCD_HEIGHT/2, LCD_WIDTH/2-42+play_game_pic.width, LCD_HEIGHT/2+load_game_pic.height, option_colors[1]);
	UG_DrawFrame(LCD_WIDTH/2-42-1, LCD_HEIGHT/2-1, LCD_WIDTH/2-42+1+play_game_pic.width, LCD_HEIGHT/2+1+load_game_pic.height, option_colors[1]);

	UG_DrawFrame(LCD_WIDTH/2-42, LCD_HEIGHT/2+62, LCD_WIDTH/2-42+play_game_pic.width, LCD_HEIGHT/2+62+power_off_pic.height, option_colors[2]);
	UG_DrawFrame(LCD_WIDTH/2-42-1, LCD_HEIGHT/2+62-1, LCD_WIDTH/2-42+1+play_game_pic.width, LCD_HEIGHT/2+62+1+power_off_pic.height, option_colors[2]);

}

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}


static void window_1_callback(UG_MESSAGE *msg)
{

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
