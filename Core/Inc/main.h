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
#define MCU_CLK_TIM9_CH1_Pin GPIO_PIN_2
#define MCU_CLK_TIM9_CH1_GPIO_Port GPIOA
#define CS0_Pin GPIO_PIN_12
#define CS0_GPIO_Port GPIOB
#define CS1_Pin GPIO_PIN_6
#define CS1_GPIO_Port GPIOC
#define CS2_Pin GPIO_PIN_7
#define CS2_GPIO_Port GPIOC
#define UART4_RTS_Pin GPIO_PIN_15
#define UART4_RTS_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
#define NUMBER_OF_MB_MASTER_TCP_CONN	4
#define NUMBER_OF_ADC_CHANNEL			4

#define len_of_array( arr )			( sizeof( arr ) / sizeof( *arr ) )
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
