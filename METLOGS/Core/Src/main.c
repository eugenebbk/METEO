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
#include "SST26.h"
#include "LogsSST26.h"

// #include "LogsSST26/LogsSST26.h"
//  #include "crc/crc16.h"
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

uint8_t RxData[12] = {0};
uint8_t RxData6[60] = {0};
int indx = 0;
uint8_t fullPacket[12] = {0xFD, 0x55, 0};

//----------Log
log3_t log3 = {0};
// uint32_t currentID_log_buf = 0;
uint8_t blockInterruptUSB = 0;
uint16_t currentID_log_buf = 0;
uint16_t currentID_log_Reading = 0;
uint32_t currentFreeAddrLog = 0;

#define ziseMassive 256
#define ziseSector 4096
uint8_t rxDataFlash[ziseMassive + ziseMassive] = {0};

uint8_t statusFlashExt = 0;
uint8_t testMassiveExtFlash[ziseMassive] = {0};
uint8_t rxFlashSector[ziseSector] = {0};

#define ziseRBPR 14
uint8_t rxDataFlash_RBPR[ziseRBPR] = {0};

//----debug log

//---------Debug

uint32_t defaultTime = 0x937;
uint32_t defaultDate = 0x0134D879;
uint16_t countLogID = 0;

uint32_t testValue = 0xAAAAAAAA;
uint32_t freeAdressExtFlash = 0;

uint32_t flashID1 = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//----oldVersion log
// void writeTestLog(uint8_t numbOfFlash)
// {
// log3_t log31 = {0};
// // log3_func->
// log31.ID = currentID_log_buf;
// log31.Date = 20240803;
// log31.Time = 181920;
// log31.Coordinate_oXoY = 444444;
// for (size_t i = 0; i < NUMB_TEMPBOARD_SENSOR; i++)
// {
//   log31.Temperature[i] = 20 + i;
// }
// log31.VoltageBattery = 25;
// log31.OnPowerMETTMP_Board = 0;
// log31.ExtMetTMPRTR_Heater = 25;
// log31.ExtMetTMPRTR_Air = 25;
// log31.ExtMetPressureAtmosphere = 25;
// log31.ExtMetHumidity = 25;
// log31.ExtMetSpeedAir_oX = 25;
// log31.ExtMetSpeedAir_oY = 25;
// log31.ExtMetSpeedAir_Result = 25;
// log31.ExtMetDirectAir = 25;
// log31.ExtMetSpeedUltrasound_oX = 25;
// log31.ExtMetSpeedUltrasound_oY = 25;
// for (size_t i = 0; i < 47; i++)
// {
//   log31.Reserved2[i] = i;
// }
// log31.EndOfLog = 0;
// // sFLASH_WritePage((uint8_t *)&log31, sFLASH_SPI_PAGE_SIZE * currentID_log_buf, sizeof(log3_t), numbOfFlash);
// SST26writeLog1(log31, currentID_log_buf, 1);
// currentID_log_buf++;
// }

uint16_t lastLogID = 0;

// new version log
void writeTestLog(uint8_t numbOfFlash)
{

  log3_t log31 = {0};

  // log3_func->
  log31.ID = currentID_log_buf;
  log31.Date[0] = 3;
  log31.Date[1] = 8;
  log31.Date[2] = 24;

  log31.Time[0] = 18;
  log31.Time[1] = 19;
  log31.Time[2] = 20;
  log31.Time[3] = 0;

  // --//--
  log31.EndOfLog = 0;

  SST26writeLog1(log31, currentID_log_buf, 1);

  currentID_log_buf++;
}

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
  MX_UART4_Init();
  MX_USART6_UART_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(2000);
  memset(testMassiveExtFlash, 0xAB, ziseMassive - 6);

  volatile uint32_t sizeLog123 = sizeof(log3_t);
  //------------------v1

  //  sFLASH_TxSimpleCmd(1, FLASH_CMD_ULBPR);
  //  flashID1 = sFLASH_ReadID(1);
  //  // sFLASH_InitSST26(1);
  //  statusFlashExt = sFLASH_ReadSimpleCommand(FLASH_CMD_RDSR, 1);
  //  for (size_t i = 0; i < 2; i++)
  //  {
  //    sFLASH_EraseSector(i * sFLASH_SPI_SECTOR_SIZE, 1);
  //    sFLASH_WritePage(testMassiveExtFlash, i * sFLASH_SPI_PAGE_SIZE, sizeof(testMassiveExtFlash), 1);
  //  }
  //  statusFlashExt = sFLASH_ReadSimpleCommand(FLASH_CMD_RDSR, 1);
  //  for (size_t i = 0; i < 1; i++)
  //  {
  //    sFLASH_ReadBuffer(rxDataFlash, i * sFLASH_SPI_PAGE_SIZE, sFLASH_SPI_PAGE_SIZE+sFLASH_SPI_PAGE_SIZE, 1);
  //  }
  //  statusFlashExt = sFLASH_ReadSimpleCommand(FLASH_CMD_RDSR, 1);
  //  sFLASH_ReadCommand(FLASH_CMD_RBPR, rxDataFlash_RBPR, ziseRBPR, 1);

  //   //--------v2
  //   // #define NUMB_FLASH_FOR_LOGS 1
  //   sFLASH_TxSimpleCmd(1, FLASH_CMD_ULBPR);
  //   flashID1 = sFLASH_ReadID(1);
  //   statusFlashExt = sFLASH_ReadSimpleCommand(FLASH_CMD_RDSR, 1);

  //   sFLASH_EraseSector(0, 1);
  //   sFLASH_ReadBuffer(rxDataFlash, 0, sizeof(rxDataFlash), 1);
  //   currentFreeAddrLog = sFLASH_SearchLastFreePageAdress(&currentID_log_buf, 1);
  // //  sFLASH_EraseSector(0, 1);
  // 	sFLASH_WritePage(testMassiveExtFlash, 0, sizeof(testMassiveExtFlash), 1);
  // //00
  //   memset(rxDataFlash, 0x00, ziseMassive + ziseMassive);
  //   sFLASH_ReadBuffer(rxDataFlash, 0, sizeof(rxDataFlash), 1);
  //   currentFreeAddrLog = sFLASH_SearchLastFreePageAdress(&currentID_log_buf, 1);
  //   writeTestLog(1);

  //   memset(rxDataFlash, 0x00, ziseMassive + ziseMassive);
  //   sFLASH_ReadBuffer(rxDataFlash, 0, sizeof(rxDataFlash), 1);
  //   currentFreeAddrLog = sFLASH_SearchLastFreePageAdress(&currentID_log_buf, 1);

  //-------------------------------
  // test v3
  // write/read data to flash

  sFLASH_TxSimpleCmd(1, FLASH_CMD_ULBPR);
  flashID1 = sFLASH_ReadID(1);
  statusFlashExt = sFLASH_ReadSimpleCommand(FLASH_CMD_RDSR, 1);

  // 1
  SST26clearSectorLogs_sector(0, 1);
  // sFLASH_EraseSector(0, 1);
  currentFreeAddrLog = sFLASH_SearchLastFreePageAdress(&currentID_log_buf, 1);
  sFLASH_ReadBuffer(rxDataFlash, 0, sizeof(rxDataFlash), 1);
  log3 = SST26readIDLogsReturn(currentID_log_buf, 1);
  //	sFLASH_WritePage(testMassiveExtFlash, 0, sizeof(testMassiveExtFlash), 1);
  //  SST26writeLog1(log3, currentID_log_buf, 1);

  // 2
  writeTestLog(1);

  // 00
  // подготовка приемного массива
  //   memset(rxDataFlash, 0x00, ziseMassive + ziseMassive);
  //   sFLASH_ReadBuffer(rxDataFlash, 0, sizeof(rxDataFlash), 1);

  // чтение лога который записали
  // 3
  log3 = SST26readIDLogsReturn(currentID_log_buf - 1, 1);

  // 4 запись следующего лога в след ячейки
  currentFreeAddrLog = sFLASH_SearchLastFreePageAdress(&currentID_log_buf, 1);
  writeTestLog(1);
  currentFreeAddrLog = sFLASH_SearchLastFreePageAdress(&currentID_log_buf, 1);
  writeTestLog(1);
  currentFreeAddrLog = sFLASH_SearchLastFreePageAdress(&currentID_log_buf, 1);
  writeTestLog(1);

  // memset(rxDataFlash, 0x00, ziseMassive + ziseMassive);
  // sFLASH_ReadBuffer(rxDataFlash, 0, sizeof(rxDataFlash), 1);
  log3 = SST26readIDLogsReturn(currentID_log_buf - 1, 1);
  currentFreeAddrLog = sFLASH_SearchLastFreePageAdress(&currentID_log_buf, 1);

  //-----------------

  // 5
  // read Full sector
  memset(rxFlashSector, 0x00, ziseSector);
  sFLASH_ReadBuffer(rxFlashSector, 0, sizeof(rxFlashSector), 1);

  //---------------end

  //-----------------------
  // test 4
  // read all logs to usb

  SST26readAllLogs(0, 1);
  // check data in USB

  //----------------

  // // freeAdressExtFlash = sFLASH_SearchLastFreePageAdress(1);
  // sFLASH_WritePage((uint8_t *)&testValue, 0, 40, 1);
  // // freeAdressExtFlash = sFLASH_SearchLastFreePageAdress(1);
  // // sFLASH_WritePage((uint8_t *)&testValue, freeAdressExtFlash, 4, 1);
  // // freeAdressExtFlash = sFLASH_SearchLastFreePageAdress(1);

  // //  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);

  // // __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);

  // // HAL_UART_Receive_IT(&huart4, &receiveRingBuf[0], 12);

  HAL_UARTEx_ReceiveToIdle_IT(&huart4, RxData, sizeof(RxData));
  //  HAL_UARTEx_ReceiveToIdle_IT(&huart6, RxData6, sizeof(RxData6));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    HAL_Delay(10000);

    SST26readAllLogs(0, 1);
    // if (flagBoard & INTERRUPT_USB_MASK)
    if (flagBoard == 1)
    {
      //          flagBoard &= 0xFE;
      flagBoard = 0;
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_11);
      //      PIN_EN_TRANSMIT_UART4(1);
      //      // HAL_UART_Transmit(&huart4, &requestTemperature, 1, 100);
      //      HAL_UART_Transmit_IT(&huart4, &requestTemperature, 1);
      //      HAL_Delay(10);
      //      PIN_EN_TRANSMIT_UART4(0);
      //      // CDC_Transmit_FS(str, 1);
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

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  const uint8_t startWord8[2] = {0xFD, 0x55};
  indx = Size;

  if (huart->Instance == UART4) // check if the interrupt comes from
  {
    if (memcmp(RxData, &requestTemperature, 1) != 0)
    {
      // for (size_t i = 0; i < 5; i++)
      // {
      //   memcpy(&fullPacket[i * 2 + 1], &temperatureSensor[i].sourceTemperature, 2);
      // }

      memcpy(&exitTemperature8[0], &startWord8[0], 2);
      memcpy(&exitTemperature8[2], &RxData[2], 10);
      CDC_Transmit_FS(&exitTemperature8[0], sizeof(exitTemperature8));

      // writeLog(uint32_t * temperature_ptr);
    }

    HAL_UARTEx_ReceiveToIdle_IT(&huart4, RxData, sizeof(RxData));
  }
}

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
