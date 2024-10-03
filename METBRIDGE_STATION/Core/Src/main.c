/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
//#include "parserMETEO.h"
//#include "meteostation.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t str[1];
uint8_t dataReceived = 0;    // ������� ������ ��������
uint8_t dataTransmitted = 1; // ������� ������ ��������
uint8_t *Buf;

uint8_t flagBoard = 0;            // ??????? ?????? ????????
uint8_t receiveRingBuf[32] = {0}; // ??????? ?????? ????????

uint8_t requestTemperature = 0x5e;
uint8_t exitTemperature8[12] = {0};

// uart

uint8_t RxData[256] = {0};
uint8_t RxData6[60] = {0};
int indx = 0;
uint8_t fullPacket[12] = {0xFD, 0x55, 0};

uint8_t RxDataUSART3[32] = {0};
//-------------USB

uint8_t RxDataUSB[255] = {0};
uint32_t RxDataUSB_len = 0;

//--------meteo parser
//errorMeteostation_t errorMeteostation = {0};
//dataMeteostation_t dataMeteostation = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  MX_UART4_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  // __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);

  // HAL_UART_Receive_IT(&huart4, &receiveRingBuf[0], 12);
  HAL_UARTEx_ReceiveToIdle_IT(&huart3, RxDataUSART3, sizeof(RxDataUSART3));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    //    HAL_Delay(1);
    // if (flagBoard & INTERRUPT_USB_MASK)
    if (flagBoard == 1)
    {
      flagBoard = 0;
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_11);

      PIN_EN_TRANSMIT_USART3(1);
      HAL_UART_Transmit_IT(&huart3, RxDataUSB, RxDataUSB_len);
      HAL_Delay(3);
      PIN_EN_TRANSMIT_USART3(0);
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enables the Clock Security System
   */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */

/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
//HAL_Delay(200);
//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);

//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET );
//  if(huart == &huart6) {


//      CDC_Transmit_FS(str,1);

//    }
//    HAL_UART_Receive_IT (&huart6, str, 1);
    HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_3 );
  }*/

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//   // receiveDataBuf = {0}
//   const float DS18B20_T_STEP_float = 0.0625;
//   const uint16_t startWord = {0xFD55};
//   const uint8_t startWord8[2] = {0xFD, 0x55};
//   uint16_t exitTemperature[6] = {0};

//   if (huart->Instance == UART4) // c
//   {
//     /*
//     uint8_t outData_temp[24] = {0};
//     float temperature_fl[5] = {0};

//     for (size_t i = 0; i < 5; i++)
//     {
//       int16_t temp16b = (int16_t)(receiveRingBuf[i * 2 + 2] << 8 | receiveRingBuf[i * 2 + 1 + 2]); //(a << 8) | b
//       temperature_fl[i] = (float)(int32_t)temp16b * DS18B20_T_STEP_float;
//     }

//     //    if (memcmp(receiveRingBuf, &requestTemperature, 1))
//     //    {
//     //      // for (size_t i = 0; i < 5; i++)
//     //      // {
//     //      //   memcpy(&fullPacket[i * 2 + 1], &temperatureSensor[i].sourceTemperature, 2);
//     //      // }

//     //      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
//     //      // HAL_UART_Transmit_IT(&huart1, &fullPacket[0], sizeof(fullPacket));
//     //      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
//     //      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);
//     //      memcpy(&receiveRingBuf[0], huart->pRxBuffPtr, 11);
//     //      CDC_Transmit_FS(&receiveRingBuf[0], 11);
//     //    }

//     HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_11);
//     memcpy(&outData_temp[0], &receiveRingBuf[0], 2);
//     memcpy(&outData_temp[1], &temperature_fl[0], sizeof(float) * 5);
//     CDC_Transmit_FS(&outData_temp[0], sizeof(outData_temp));
//     */

//     HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_11);
//     memcpy(&exitTemperature8[0], &startWord8[0], 2);
//     memcpy(&exitTemperature8[2], &receiveRingBuf[2], 10);
//     CDC_Transmit_FS(&exitTemperature8[0], sizeof(exitTemperature8));
//   }

//   // HAL_UART_Receive_IT(&huart4, &receiveRingBuf[0], 12);
// }

// void HAL_UART_RxIdleCallback(UART_HandleTypeDef *huart)
// {
//   // __HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);

//   const float DS18B20_T_STEP_float = 0.0625;
//   // const uint16_t startWord = {0xFD55};
//   const uint8_t startWord8[2] = {0xFD, 0x55};
//   uint16_t exitTemperature[6] = {0};
//   HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_11);
//   memcpy(&exitTemperature8[0], &startWord8[0], 2);
//   memcpy(&exitTemperature8[2], &receiveRingBuf[2], 10);
//   CDC_Transmit_FS(&exitTemperature8[0], sizeof(exitTemperature8));

//   __HAL_UART_CLEAR_IDLEFLAG(&huart4);
//   __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
// }

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{

  // const float DS18B20_T_STEP_float = 0.0625;
  const uint8_t startWord8[2] = {0xFD, 0x55};
  indx = Size;

  if (huart->Instance == USART3) // check if the interrupt comes from //rs485
  {
    if (memcmp(RxDataUSART3, RxDataUSB, Size)) // echo data
    {
      CDC_Transmit_FS(&RxDataUSART3[0], Size);
//      parserMeteoStation_simple(&RxDataUSART3[0]);
    }
    HAL_UARTEx_ReceiveToIdle_IT(&huart3, RxDataUSART3, sizeof(RxDataUSART3));
  }
  if (huart->Instance == UART4) // check if the interrupt comes from
  {
    // const uint8_t startPacket = 0x5D;
    // const uint8_t startBites = 7;
    // startByte.startBite = startBites;

    // for (size_t i = 0; i < 5; i++)
    // {
    //   startByte.tempSensor[i] = (uint8_t)errorDS18B20[i];
    // }

    // if (memcmp(RxData, &fullPacket[0], Size))
    // {

    // }

//    if (memcmp(RxData, &requestTemperature, 1))
//    {
//      // for (size_t i = 0; i < 5; i++)
//      // {
//      //   memcpy(&fullPacket[i * 2 + 1], &temperatureSensor[i].sourceTemperature, 2);
//      // }

//      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
//      // HAL_UART_Transmit_IT(&huart4, &fullPacket[0], sizeof(fullPacket));
//      // HAL_Delay(5);
//      // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

//      // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_11);
//      memcpy(&exitTemperature8[0], &startWord8[0], 2);
//      memcpy(&exitTemperature8[2], &RxData[2], 10);
//      CDC_Transmit_FS(&exitTemperature8[0], sizeof(exitTemperature8));
//      // CDC_Transmit_FS(&RxData[0], 12);
//    }

    HAL_UARTEx_ReceiveToIdle_IT(&huart4, RxData, sizeof(RxData));
  }

  if (huart->Instance == USART6) // check if the interrupt comes from
  {

    CDC_Transmit_FS(&RxData6[0], sizeof(RxData6));
    HAL_UARTEx_ReceiveToIdle_IT(&huart6, RxData6, sizeof(RxData6));
  }
}

//  uint16_t sign = tRegValue & DS18B20_SIGN_MASK;

//  if (sign != 0)
//  {
//    tRegValue = (0xFFFF - tRegValue + 1);
//  }

//  tRegValue &= DS18B20_12_BITS_DATA_MASK;

//  sensor->temperature = (float)tRegValue * DS18B20_T_STEP;
//  if (sign != 0)
//  {
//    sensor->temperature *= (-1);
//  }

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
