/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SEED_GENERATOR_Pin GPIO_PIN_0
#define SEED_GENERATOR_GPIO_Port GPIOC
#define LEFT_ANALOG_KEY_Pin GPIO_PIN_4
#define LEFT_ANALOG_KEY_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define LEFT_ANALOG_X_Pin GPIO_PIN_6
#define LEFT_ANALOG_X_GPIO_Port GPIOA
#define LEFT_ANALOG_Y_Pin GPIO_PIN_7
#define LEFT_ANALOG_Y_GPIO_Port GPIOA
#define RIGHT_ANALOG_X_Pin GPIO_PIN_0
#define RIGHT_ANALOG_X_GPIO_Port GPIOB
#define RIGHT_ANALOG_Y_Pin GPIO_PIN_1
#define RIGHT_ANALOG_Y_GPIO_Port GPIOB
#define RIGHT_ANALOG_KEY_Pin GPIO_PIN_2
#define RIGHT_ANALOG_KEY_GPIO_Port GPIOB
#define KEY_DOWN_Pin GPIO_PIN_12
#define KEY_DOWN_GPIO_Port GPIOB
#define KEY_UP_Pin GPIO_PIN_13
#define KEY_UP_GPIO_Port GPIOB
#define KEY_B_Pin GPIO_PIN_6
#define KEY_B_GPIO_Port GPIOC
#define KEY_A_Pin GPIO_PIN_7
#define KEY_A_GPIO_Port GPIOC
#define LCD_DC_Pin GPIO_PIN_6
#define LCD_DC_GPIO_Port GPIOB
#define LCD_RST_Pin GPIO_PIN_7
#define LCD_RST_GPIO_Port GPIOB
#define LCD_CS_Pin GPIO_PIN_8
#define LCD_CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
