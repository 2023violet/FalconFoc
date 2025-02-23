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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
  extern uint8_t Buffer[1];
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
#define MCPWM_CLOCK_HZ 168000000
#define MCPWM_DEADTIME_CLOCKS 20
#define MCPWM_TGRO_TIME MCPWM_PERIOD_CLOCKS-10
#define MCPWM_RCR 0
#define MCPWM_FREQ 20000
#define MCPWM_PERIOD_CLOCKS MCPWM_CLOCK_HZ/2/MCPWM_FREQ
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOC
#define Current_Ic_Pin GPIO_PIN_0
#define Current_Ic_GPIO_Port GPIOA
#define Current_Ib_Pin GPIO_PIN_1
#define Current_Ib_GPIO_Port GPIOA
#define Current_Ia_Pin GPIO_PIN_2
#define Current_Ia_GPIO_Port GPIOA
#define Current_IBUS_Pin GPIO_PIN_3
#define Current_IBUS_GPIO_Port GPIOA
#define WS2812_Pin GPIO_PIN_4
#define WS2812_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOC
#define TEMP_Pin GPIO_PIN_2
#define TEMP_GPIO_Port GPIOB
#define SPI3_CS_Pin GPIO_PIN_12
#define SPI3_CS_GPIO_Port GPIOB
#define LIN3_Pin GPIO_PIN_13
#define LIN3_GPIO_Port GPIOB
#define LIN2_Pin GPIO_PIN_14
#define LIN2_GPIO_Port GPIOB
#define LIN1_Pin GPIO_PIN_15
#define LIN1_GPIO_Port GPIOB
#define HIN3_Pin GPIO_PIN_8
#define HIN3_GPIO_Port GPIOA
#define HIN2_Pin GPIO_PIN_9
#define HIN2_GPIO_Port GPIOA
#define HIN1_Pin GPIO_PIN_10
#define HIN1_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
