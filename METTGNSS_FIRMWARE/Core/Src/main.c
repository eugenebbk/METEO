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
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "ds18b20.h"
//#include "ds18b20_init.h"
//#include "onewire.h"
#include "string.h"
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
//DS18B20 temperatureSensor[numbSensorsDS18B20] = {0};
//DS18B20_Status errorDS18B20[numbSensorsDS18B20] = {0};

startByte_t startByte = {0};
uint8_t fullPacket[12] = {0xFD, 0x55, 0};
uint8_t receiveByte = {0};

uint8_t RxData[16] = {0};
int indx = 0;
uint8_t flagRxUART = 0;

flagsInterrupts_t flagsInterrupts = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//  if (htim->Instance == TIM10) // check if the interrupt comes from TIM10 //750ms
//  {
//    // HAL_IWDG_Refresh(&hiwdg);
//    HAL_TIM_Base_Stop_IT(&htim10);
//    flagsInterrupts.TIM10_int = 1;
//    __HAL_TIM_SET_COUNTER(&htim10, 0);
//  }
//}

//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
//{

//  uint8_t holeOp = 0;
//  size_t counterTemp = 25000 * 2; // 300ms
//  while (--counterTemp)
//  {
//    holeOp++;
//  }

//  indx = Size;
//  if (huart->Instance == USART1) // check if the interrupt comes from
//  {
//    // if (memcmp(RxData, &fullPacket[0], Size))
//    // {
//    // }

//    flagRxUART = 1;

//    if (memcmp(RxData, &fullPacket[0], sizeof(fullPacket)))
//    {
//      flagsInterrupts.UART1_int = 1;
//    }
//  }
//}

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
  /* USER CODE BEGIN 2 */

  // __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

//  __HAL_UART_CLEAR_IDLEFLAG(&huart1);
//  HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, sizeof(RxData));


//  __HAL_TIM_CLEAR_FLAG(&htim10, TIM_SR_UIF);
//  HAL_TIM_Base_Start_IT(&htim10);

//        int8_t settings[3];
////      settings[0] = (uint8_t)125;
////      settings[1] = (uint8_t)-55;
//      settings[0] = (uint8_t)0;
//      settings[1] = (uint8_t)0;
//      settings[2] = DS18B20_12_BITS_CONFIG;
//			
//			
//   initDS18B20(&temperatureSensor[0], &errorDS18B20[0], numbSensorsDS18B20);
//  // // send request to onewire sensor for measuring temperature
//   requestToCalculationTemperature(&temperatureSensor[0], &errorDS18B20[0], numbSensorsDS18B20);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    //    		while (1)
    //    		{

    //    		}

//    if (flagsInterrupts.TIM10_int)
//    {
//      flagsInterrupts.TIM10_int = 0;
////			initDS18B20(&temperatureSensor[0], &errorDS18B20[0], numbSensorsDS18B20);
//       receiveTemperature(&temperatureSensor[0], &errorDS18B20[0], numbSensorsDS18B20);
//       // writing temperature to exit packet
//       for (size_t i = 0; i < numbSensorsDS18B20; i++)
//       {
//         memcpy(&fullPacket[i * 2 + 2], &temperatureSensor[i].sourceTemperature, 2);
//       }
//       // send request to onewire sensor for measuring temperature
//       requestToCalculationTemperature(&temperatureSensor[0], &errorDS18B20[0], numbSensorsDS18B20);

//      //--debug

//      HAL_TIM_Base_Start_IT(&htim10);
//    }

//    if (flagsInterrupts.UART1_int)
//    {

//      volatile uint8_t holeOp = 0;
//      size_t counterTemp = 10000 * 15; // 15ms
//      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
//			      while (--counterTemp)
//      {
//        //        asm("NOP");
//        holeOp << 1; // nop operation
//      }
//      HAL_UART_Transmit_IT(&huart1, &fullPacket[0], sizeof(fullPacket));
//			HAL_Delay(10);
//      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
//      flagsInterrupts.UART1_int = 0;
//      __HAL_UART_CLEAR_IDLEFLAG(&huart1);
//      HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, sizeof(RxData));
//    }

    // receiving from onewire sensor calculate temperature

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
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
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

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15
                           PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

//  if(huart == &huart1) {
//  HAL_UART_Receive_IT (&huart1, rData, 1);
//	DS18B20_Init(&temperatureSensor, &huart5);

//  DS18B20_InitializationCommand(&temperatureSensor);
//  DS18B20_ReadRom(&temperatureSensor);
//  DS18B20_ReadScratchpad(&temperatureSensor);

//  uint8_t settings[3];
//  settings[0] = temperatureSensor.temperatureLimitHigh;
//  settings[1] = temperatureSensor.temperatureLimitLow;
//  settings[2] = DS18B20_12_BITS_CONFIG;

//    DS18B20_InitializationCommand(&temperatureSensor);
//    DS18B20_SkipRom(&temperatureSensor);
//    DS18B20_WriteScratchpad(&temperatureSensor, settings);
//		DS18B20_InitializationCommand(&temperatureSensor);
//		DS18B20_SkipRom(&temperatureSensor);
//		DS18B20_ConvertT(&temperatureSensor, DS18B20_DATA);
//

//  }
//}
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

#ifdef  USE_FULL_ASSERT
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
