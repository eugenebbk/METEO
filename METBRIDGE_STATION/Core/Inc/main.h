/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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

#define INTERRUPT_USB_MASK 1
#define PIN_EN_TRANSMIT_UART4(x) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, x);
#define PIN_EN_TRANSMIT_USART3(x) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, x);

typedef struct
{
	uint8_t USB_int : 1; // 0-1
	uint8_t UART1_int : 1; // 0-1
	uint8_t UART4_int : 1; // 0-1
	uint8_t UART5_int : 1; // 0-1
	uint8_t UART6_int : 1; // 0-1
	uint8_t UART7_int : 1; // 0-1
	uint8_t UART8_int : 1; // 0-1
	uint8_t UART9_int : 1; // 0-1
} flagsInterrupts_t;

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
#define POWER_ON_METEOBLOCK_Pin GPIO_PIN_2
#define POWER_ON_METEOBLOCK_GPIO_Port GPIOC
#define SPI_SS_Pin GPIO_PIN_4
#define SPI_SS_GPIO_Port GPIOA
#define RST_GNSS_Pin GPIO_PIN_8
#define RST_GNSS_GPIO_Port GPIOC
#define POWER_ON_TEMPBOARD_Pin GPIO_PIN_9
#define POWER_ON_TEMPBOARD_GPIO_Port GPIOA
#define UART_EN_Pin GPIO_PIN_10
#define UART_EN_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
