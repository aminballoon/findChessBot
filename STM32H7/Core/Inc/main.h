/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LM4_Pin GPIO_PIN_6
#define LM4_GPIO_Port GPIOE
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define DIR_5_Pin GPIO_PIN_6
#define DIR_5_GPIO_Port GPIOA
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define DIR_3_Pin GPIO_PIN_9
#define DIR_3_GPIO_Port GPIOE
#define PUL_3_Pin GPIO_PIN_11
#define PUL_3_GPIO_Port GPIOE
#define LM3_Pin GPIO_PIN_12
#define LM3_GPIO_Port GPIOE
#define PUL_2_Pin GPIO_PIN_13
#define PUL_2_GPIO_Port GPIOE
#define DIR_2_Pin GPIO_PIN_14
#define DIR_2_GPIO_Port GPIOE
#define LM5_Pin GPIO_PIN_15
#define LM5_GPIO_Port GPIOE
#define LM1_Pin GPIO_PIN_10
#define LM1_GPIO_Port GPIOB
#define LM2_Pin GPIO_PIN_11
#define LM2_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define USB_OTG_FS_PWR_EN_Pin GPIO_PIN_10
#define USB_OTG_FS_PWR_EN_GPIO_Port GPIOD
#define DIR_4_Pin GPIO_PIN_14
#define DIR_4_GPIO_Port GPIOD
#define PUL_4_Pin GPIO_PIN_15
#define PUL_4_GPIO_Port GPIOD
#define USB_OTG_FS_OVCR_Pin GPIO_PIN_7
#define USB_OTG_FS_OVCR_GPIO_Port GPIOG
#define STLINK_RX1_Pin GPIO_PIN_0
#define STLINK_RX1_GPIO_Port GPIOD
#define STLINK_TX1_Pin GPIO_PIN_1
#define STLINK_TX1_GPIO_Port GPIOD
#define PUL_5_Pin GPIO_PIN_5
#define PUL_5_GPIO_Port GPIOB
#define DIR_1_Pin GPIO_PIN_6
#define DIR_1_GPIO_Port GPIOB
#define PUL_1_Pin GPIO_PIN_7
#define PUL_1_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_1
#define LD2_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
