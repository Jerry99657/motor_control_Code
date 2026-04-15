/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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
void Boot_DebugStageLog(const char *text);
void Boot_DebugFlushStageLogsViaCdc(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Key_Up_Pin GPIO_PIN_0
#define Key_Up_GPIO_Port GPIOF
#define Key_Down_Pin GPIO_PIN_1
#define Key_Down_GPIO_Port GPIOF
#define Key_Left_Pin GPIO_PIN_2
#define Key_Left_GPIO_Port GPIOF
#define Key_Right_Pin GPIO_PIN_3
#define Key_Right_GPIO_Port GPIOF
#define Key_OK_Pin GPIO_PIN_4
#define Key_OK_GPIO_Port GPIOF
#define Key1_Pin GPIO_PIN_5
#define Key1_GPIO_Port GPIOF
#define VOLTAGE_MONITOR_Pin GPIO_PIN_0
#define VOLTAGE_MONITOR_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_2
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_3
#define LED2_GPIO_Port GPIOC
#define SD_Pin GPIO_PIN_0
#define SD_GPIO_Port GPIOB
#define CAMERAPWDN_Pin GPIO_PIN_13
#define CAMERAPWDN_GPIO_Port GPIOF
#define M1_PH_Pin GPIO_PIN_2
#define M1_PH_GPIO_Port GPIOG
#define M2_PH_Pin GPIO_PIN_3
#define M2_PH_GPIO_Port GPIOG
#define M3_PH_Pin GPIO_PIN_4
#define M3_PH_GPIO_Port GPIOG
#define M4_PH_Pin GPIO_PIN_5
#define M4_PH_GPIO_Port GPIOG
#define LED0_Pin GPIO_PIN_7
#define LED0_GPIO_Port GPIOG
#define LCD_BL_Pin GPIO_PIN_12
#define LCD_BL_GPIO_Port GPIOG
#define LCD_DC_Pin GPIO_PIN_15
#define LCD_DC_GPIO_Port GPIOG
#define Key2_Pin GPIO_PIN_8
#define Key2_GPIO_Port GPIOB
#define Key3_Pin GPIO_PIN_9
#define Key3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
