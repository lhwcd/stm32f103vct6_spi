/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>
#include "xl355.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum {
    LOG_LEVEL_DEBUG = 1,
    LOG_LEVEL_INFO = 2,
    LOG_LEVEL_WARN = 3,
    LOG_LEVEL_ERROR = 4,
    LOG_LEVEL_NONE = 5
} log_level_e;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define LOG_ERROR(fmt, args...) log_message(LOG_LEVEL_ERROR, "error", fmt, ##args)
#define LOG_WARN(fmt, args...) log_message(LOG_LEVEL_WARN, "warn", fmt, ##args)
#define LOG_INFO(fmt, args...) log_message(LOG_LEVEL_INFO, "info", fmt, ##args)
#define LOG_DEBUG(fmt, args...) log_message(LOG_LEVEL_DEBUG, "debug", fmt, ##args)
#define LOG_BINARY(fmt, args...) log_binary(LOG_LEVEL_DEBUG, fmt, ##args)

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI3_INT_Pin GPIO_PIN_2
#define SPI3_INT_GPIO_Port GPIOE
#define LED_SYNC_Pin GPIO_PIN_4
#define LED_SYNC_GPIO_Port GPIOE
#define LED_RUN_Pin GPIO_PIN_5
#define LED_RUN_GPIO_Port GPIOE
#define WATCHDOG_Pin GPIO_PIN_2
#define WATCHDOG_GPIO_Port GPIOC
#define GPS_UART_RX_Pin GPIO_PIN_2
#define GPS_UART_RX_GPIO_Port GPIOA
#define GPS_UART_TX_Pin GPIO_PIN_3
#define GPS_UART_TX_GPIO_Port GPIOA
#define XL355_SPI_CS_Pin GPIO_PIN_4
#define XL355_SPI_CS_GPIO_Port GPIOA
#define SC200R_UART_TX_Pin GPIO_PIN_10
#define SC200R_UART_TX_GPIO_Port GPIOB
#define SC200R_UART_RX_Pin GPIO_PIN_11
#define SC200R_UART_RX_GPIO_Port GPIOB
#define GPS_EN_Pin GPIO_PIN_12
#define GPS_EN_GPIO_Port GPIOB
#define GPS_RST_Pin GPIO_PIN_13
#define GPS_RST_GPIO_Port GPIOB
#define SC200R_EN_Pin GPIO_PIN_6
#define SC200R_EN_GPIO_Port GPIOC
#define SC200R_RST_Pin GPIO_PIN_7
#define SC200R_RST_GPIO_Port GPIOC
#define DEBUG_UART_TX_Pin GPIO_PIN_9
#define DEBUG_UART_TX_GPIO_Port GPIOA
#define DEBUG_UART_RX_Pin GPIO_PIN_10
#define DEBUG_UART_RX_GPIO_Port GPIOA
#define SPI3_CS_Pin GPIO_PIN_15
#define SPI3_CS_GPIO_Port GPIOA
#define XL355_INT1_Pin GPIO_PIN_11
#define XL355_INT1_GPIO_Port GPIOC
#define XL355_INT1_EXTI_IRQn EXTI15_10_IRQn
#define XL355_SYNC_Pin GPIO_PIN_12
#define XL355_SYNC_GPIO_Port GPIOC
#define GPS_1PPS_Pin GPIO_PIN_2
#define GPS_1PPS_GPIO_Port GPIOD
#define GPS_1PPS_EXTI_IRQn EXTI2_IRQn
#define XL355_EN_Pin GPIO_PIN_3
#define XL355_EN_GPIO_Port GPIOD
#define SC200R_PW_EN_Pin GPIO_PIN_6
#define SC200R_PW_EN_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */


int log_message(log_level_e level, const char *level_str, const char *fmt, ...);
int log_binary(log_level_e level, const char *fmt, ...);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
