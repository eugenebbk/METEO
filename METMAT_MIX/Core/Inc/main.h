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
extern "C"{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

  /* Private includes ----------------------------------------------------------*/
  /* USER CODE BEGIN Includes */

#define INTERRUPT_USB_MASK 1

#define SIZE_RX_USB_BUFFER 64
//#define HEADER_TO_DEFAULT_MODE 0x99
#define HEADER_METTMPR_BOARD1 0xFD
#define HEADER_METTMPR_BOARD2 0x22

#define HEADER_EXIT_CMD2 0x19
#define NUMB_FLASH_MEM 1
//#define HEADER_METTMPR_BOARD2 0x55

  /* USER CODE END Includes */

  /* Exported types ------------------------------------------------------------*/
  /* USER CODE BEGIN ET */

  // 48byte - before reserve
  // 64byte - after reserve
  //  typedef struct
  //  {
  //    uint32_t ID;
  //    uint32_t Date;
  //    uint32_t Time;
  //    uint32_t Coordinate_oXoY;
  //    // int32_t Temperature[5];
  //    uint32_t Temperature[5];
  //    uint32_t VoltageBattery;
  //    uint32_t OnPowerMETTMP_Board;
  //    uint32_t Reserved[4];
  //    uint32_t EndOfLog;
  //  } log1_t;

  typedef struct
  {
    uint32_t Date;
    uint32_t Time;
    uint32_t Coordinate_oXoY;
    // int32_t Temperature[5];
    uint32_t Temperature[5];
    uint32_t VoltageBattery;
    uint32_t OnPowerMETTMP_Board;
    uint32_t Reserved[5];
    uint32_t EndOfLog;
  } log1_t;

  typedef struct
  {
    uint16_t periodWritingDataToLog;
    uint8_t currentModeMK;
  } configureModeMK_t;

  
  typedef struct
  {
    uint8_t USB_VCP_int : 1; // 0-1
    uint8_t UART_METEOBLOCK_int : 1;  // 0-1
    uint8_t UART_TEMPERATURE_int : 1;   // 0-1
    uint8_t UART_GNSS_int : 1;    // 0-1
    uint8_t COLLECT_DATA_int : 1;   // 0-1
    uint8_t UART_ACCUM_int : 1;   // 0-1
    uint8_t CLEAR_LOG_int : 1;   // 0-1
    // uint8_t PVD_PWR_int : 1; // 0-1
  } flagsInterrupts_t;

  typedef enum
  {
		DEFAULT_MODE=0,
		COLLECT_DATA_MODE,
		BRIDGE_METEOBLOCK_MODE,
		BRIDGE_GNSS_MODE,
		BRIDGE_ACCUMULATOR_MODE,
		BRIDGE_TMPRTRBRD_MODE
  } eModeMK_t;


  
  typedef struct
  {
    uint8_t COLLECT_GNSS : 1; // 0-1
    uint8_t COLLECT_METEOBLOCK : 1;  // 0-1
    uint8_t COLLECT_TMPRTBRD : 1;   // 0-1
    uint8_t COLLECT_ACCUM : 1;   // 0-1
    // uint8_t PVD_PWR_int : 1; // 0-1
  } flagsCollectData_t;

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
#define PPS_ML_GNSS_Pin GPIO_PIN_13
#define PPS_ML_GNSS_GPIO_Port GPIOC
#define U2_EN_PWR_Pin GPIO_PIN_3
#define U2_EN_PWR_GPIO_Port GPIOC
#define SPI1_SS1_Pin GPIO_PIN_4
#define SPI1_SS1_GPIO_Port GPIOA
#define U3_EN_PWR_METBLCK_Pin GPIO_PIN_4
#define U3_EN_PWR_METBLCK_GPIO_Port GPIOC
#define RST_GNSS_Pin GPIO_PIN_8
#define RST_GNSS_GPIO_Port GPIOC
#define SPI1_SS2_Pin GPIO_PIN_9
#define SPI1_SS2_GPIO_Port GPIOC
#define POWER_ON_TEMPBOARD_Pin GPIO_PIN_9
#define POWER_ON_TEMPBOARD_GPIO_Port GPIOA
#define POWER_ON_12V_XP9_Pin GPIO_PIN_10
#define POWER_ON_12V_XP9_GPIO_Port GPIOA
#define UART_EN_METTMPR_Pin GPIO_PIN_10
#define UART_EN_METTMPR_GPIO_Port GPIOC
#define U1_EN_PWR_AKKUM_Pin GPIO_PIN_5
#define U1_EN_PWR_AKKUM_GPIO_Port GPIOB
  /* USER CODE BEGIN Private defines */

  /* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
