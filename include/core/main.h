/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define V_LV_Pin GPIO_PIN_1
#define V_LV_GPIO_Port GPIOA
#define THROTTLE_POSITION_Pin GPIO_PIN_2
#define THROTTLE_POSITION_GPIO_Port GPIOA
#define IMD_STATE_Pin GPIO_PIN_0
#define IMD_STATE_GPIO_Port GPIOB
#define AMS_STATE_Pin GPIO_PIN_1
#define AMS_STATE_GPIO_Port GPIOB
#define SENSE_12V_1_Pin GPIO_PIN_1
#define SENSE_12V_1_GPIO_Port GPIOG
#define SENSE_12V_2_Pin GPIO_PIN_2
#define SENSE_12V_2_GPIO_Port GPIOG
#define SENSE_12V_3_Pin GPIO_PIN_3
#define SENSE_12V_3_GPIO_Port GPIOG
#define SENSE_12V_4_Pin GPIO_PIN_4
#define SENSE_12V_4_GPIO_Port GPIOG
#define SENSE_12V_5_Pin GPIO_PIN_5
#define SENSE_12V_5_GPIO_Port GPIOG
#define SENSE_12V_6_Pin GPIO_PIN_6
#define SENSE_12V_6_GPIO_Port GPIOG
#define LV_CHARGE_CONTROL_Pin GPIO_PIN_7
#define LV_CHARGE_CONTROL_GPIO_Port GPIOG
#define LV_CHARGE_STATE_Pin GPIO_PIN_8
#define LV_CHARGE_STATE_GPIO_Port GPIOG
#define OUT_nDIS_Pin GPIO_PIN_9
#define OUT_nDIS_GPIO_Port GPIOA
#define OUT_nCS_Pin GPIO_PIN_10
#define OUT_nCS_GPIO_Port GPIOA
#define OUT_nDIAG_Pin GPIO_PIN_11
#define OUT_nDIAG_GPIO_Port GPIOA
#define SENSE_12V_7_Pin GPIO_PIN_9
#define SENSE_12V_7_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
