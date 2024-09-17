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
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

  /* Private includes ----------------------------------------------------------*/
  /* USER CODE BEGIN Includes */

#define INTERRUPT_USB_MASK 1
#define PIN_EN_TRANSMIT_UART4(x) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, x);

  /* USER CODE END Includes */

  /* Exported types ------------------------------------------------------------*/
  /* USER CODE BEGIN ET */

  typedef struct
  {
    uint8_t GNSS_Inter : 1; // 0-31
    uint8_t x2 : 1; // 0-7
    uint8_t x3 : 1; // 0-7
    uint8_t x4 : 1; // 0-7
    uint8_t x5 : 1; // 0-7
    uint8_t x6 : 1; // 0-7
    uint8_t x7 : 1; // 0-7
    uint8_t x8 : 1; // 0-7
  } inter_t;

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
