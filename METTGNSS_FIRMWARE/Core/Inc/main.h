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

  typedef struct
  {
    // uint8_t tempSensor[5]:1;   // 0-1
    uint8_t tempSensor0 : 1; // 0-1
    uint8_t tempSensor1 : 1; // 0-1
    uint8_t tempSensor2 : 1; // 0-1
    uint8_t tempSensor3 : 1; // 0-1
    uint8_t tempSensor4 : 1; // 0-1
    uint8_t startBite : 3;   // 0-7
  } startByte_t;


  typedef struct
  {
    // uint8_t tempSensor[5]:1;   // 0-1
    uint8_t TIM10_int : 1; // 0-1
    uint8_t UART1_int : 1; // 0-1
    uint8_t UART4_int : 1; // 0-1
    uint8_t UART5_int : 1; // 0-1
    uint8_t UART6_int : 1; // 0-1
    uint8_t UART7_int : 1; // 0-1
    uint8_t UART8_int : 1; // 0-1
    uint8_t UART9_int : 1; // 0-1
  } flagsInterrupts_t;

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

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
