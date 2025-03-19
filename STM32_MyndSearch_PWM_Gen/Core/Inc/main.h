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
#include "stm32f4xx_hal.h"

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
#define Ch1Button_Pin GPIO_PIN_3
#define Ch1Button_GPIO_Port GPIOE
#define Ch1LowLED_Pin GPIO_PIN_4
#define Ch1LowLED_GPIO_Port GPIOE
#define Ch1HighLED_Pin GPIO_PIN_5
#define Ch1HighLED_GPIO_Port GPIOE
#define Ch2Button_Pin GPIO_PIN_13
#define Ch2Button_GPIO_Port GPIOC
#define Ch2LowLED_Pin GPIO_PIN_14
#define Ch2LowLED_GPIO_Port GPIOC
#define Ch2HighLED_Pin GPIO_PIN_15
#define Ch2HighLED_GPIO_Port GPIOC
#define Ch3Button_Pin GPIO_PIN_2
#define Ch3Button_GPIO_Port GPIOF
#define Ch3LowLED_Pin GPIO_PIN_3
#define Ch3LowLED_GPIO_Port GPIOF
#define Ch3HighLED_Pin GPIO_PIN_4
#define Ch3HighLED_GPIO_Port GPIOF
#define SetUpStatusLED_Pin GPIO_PIN_3
#define SetUpStatusLED_GPIO_Port GPIOC
#define SetupButton_Pin GPIO_PIN_2
#define SetupButton_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOB
#define Ch7Button_Pin GPIO_PIN_2
#define Ch7Button_GPIO_Port GPIOB
#define Ch7LowLED_Pin GPIO_PIN_11
#define Ch7LowLED_GPIO_Port GPIOF
#define Ch7HighLED_Pin GPIO_PIN_12
#define Ch7HighLED_GPIO_Port GPIOF
#define Ch8Button_Pin GPIO_PIN_14
#define Ch8Button_GPIO_Port GPIOF
#define Ch8LowLED_Pin GPIO_PIN_15
#define Ch8LowLED_GPIO_Port GPIOF
#define Ch8HighLED_Pin GPIO_PIN_0
#define Ch8HighLED_GPIO_Port GPIOG
#define LevelLED1_Pin GPIO_PIN_11
#define LevelLED1_GPIO_Port GPIOE
#define LevelLED2_Pin GPIO_PIN_12
#define LevelLED2_GPIO_Port GPIOE
#define LevelLED3_Pin GPIO_PIN_13
#define LevelLED3_GPIO_Port GPIOE
#define LevelLED4_Pin GPIO_PIN_14
#define LevelLED4_GPIO_Port GPIOE
#define LevelLED5_Pin GPIO_PIN_15
#define LevelLED5_GPIO_Port GPIOE
#define LevelLED6_Pin GPIO_PIN_10
#define LevelLED6_GPIO_Port GPIOB
#define LevelLED7_Pin GPIO_PIN_13
#define LevelLED7_GPIO_Port GPIOB
#define LevelLED8_Pin GPIO_PIN_14
#define LevelLED8_GPIO_Port GPIOB
#define LevelLED9_Pin GPIO_PIN_15
#define LevelLED9_GPIO_Port GPIOB
#define LevelLED10_Pin GPIO_PIN_8
#define LevelLED10_GPIO_Port GPIOD
#define LevelLED11_Pin GPIO_PIN_9
#define LevelLED11_GPIO_Port GPIOD
#define LevelLED12_Pin GPIO_PIN_10
#define LevelLED12_GPIO_Port GPIOD
#define Ch4HighLED_Pin GPIO_PIN_5
#define Ch4HighLED_GPIO_Port GPIOG
#define Ch4LowLED_Pin GPIO_PIN_6
#define Ch4LowLED_GPIO_Port GPIOG
#define Ch4Button_Pin GPIO_PIN_7
#define Ch4Button_GPIO_Port GPIOG
#define LED3_Pin GPIO_PIN_9
#define LED3_GPIO_Port GPIOA
#define Ch6Button_Pin GPIO_PIN_0
#define Ch6Button_GPIO_Port GPIOD
#define Ch6LowLED_Pin GPIO_PIN_1
#define Ch6LowLED_GPIO_Port GPIOD
#define Ch6HighLED_Pin GPIO_PIN_2
#define Ch6HighLED_GPIO_Port GPIOD
#define Ch5HighLED_Pin GPIO_PIN_10
#define Ch5HighLED_GPIO_Port GPIOG
#define Ch5LowLED_Pin GPIO_PIN_11
#define Ch5LowLED_GPIO_Port GPIOG
#define Ch5Button_Pin GPIO_PIN_12
#define Ch5Button_GPIO_Port GPIOG
#define LED2_Pin GPIO_PIN_7
#define LED2_GPIO_Port GPIOB

#define DEBOUNCE_DELAY 50

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
