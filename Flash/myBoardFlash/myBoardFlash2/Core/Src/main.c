/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
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
#include "spi.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usbd_cdc_if.h"
//#include "sst26_v2\SST26_v2.h"
#include "SST26.h"
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
uint8_t dataReceived = 0;    // ??????? ?????? ????????
uint8_t dataTransmitted = 1; // ??????? ?????? ????????
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
//log1_t log1 = {0};
uint32_t currentID_log = 1;
//-----------extern flash

uint16_t numbSectorCurrent = 0;
uint32_t flashID = 0;

//---------Debug

uint32_t defaultTime = 0x937;
uint32_t defaultDate = 0x0134D879;
uint16_t countLogID = 0;

uint8_t testMassiveExtFlash[256] = {0};
//uint8_t testMassiveExtFlash[4096] = {0};
uint8_t testValue8 = 0xAA;
uint32_t testValue = 0xAAAAAAAA;
uint32_t freeAdressExtFlash = 0;
uint8_t rxDataFlash[2048] = {0};
//uint8_t rxDataFlash[4096] = {0};
uint8_t statusFlashExt = 0;

#define ziseRBPR 14
uint8_t rxDataFlash_RBPR[ziseRBPR] = {0};


  uint32_t sectorsNeeded = 0;
  uint32_t currentSector = 0;
  uint32_t remainingBytes = 0;
uint32_t currentID_flash = 1;

//--------

		uint32_t flashID;
		uint8_t status;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

int	my_memcmp(void *dest, void *src, int size)
{
    unsigned char *p = dest;
    unsigned char *q = src;

    while (size > 0)
    {
        if (*p != *q)
        {
            return (*p - *q);
        }
        size--;
        p++;
        q++;
    }
    return 0;
}

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
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  memset(testMassiveExtFlash, testValue8, 128);

////---------------v2

//  flash_GBPU();
//  HAL_Delay(5);
//  flash_sector_erase(107); //let's store data in the memory location 107 of flash
//  HAL_Delay(5);

//  uint8_t tx_buffer[6] = {1,2,3,4,5,6}; //sample data to send to flash
//  uint8_t rx_buffer[6];
//  flash_write(107,tx_buffer,6); //read from tx_buffer...
//  flash_read(107,rx_buffer,6); //...and store it in rx_buffer
//  HAL_Delay(1);
//  HAL_Delay(1);
//  HAL_Delay(1);
	
	//------------------
	//-----------V1 cheker
	
	
	
////  /* 1 - Verify that the identifier is correctly read */

		flashID = sFLASH_ReadIDv2();
		status = sFLASH_ReadStatusRegister();
		HAL_Delay(10);
		sFLASH_GlobalProtectionEnable();
		HAL_Delay(10);
  	sFLASH_EraseSector(107);
		HAL_Delay(10);
//		sFLASH_WritePage(testMassiveExtFlash, 107,sFLASH_SPI_SECTOR_SIZE);
//		HAL_Delay(1000);
		sFLASH_ReadBuffer(rxDataFlash, 107, sFLASH_SPI_SECTOR_SIZE);
		HAL_Delay(1000);
		HAL_Delay(10);
		
		
			//------------------
	//-----------V1 
		
// //  /* 1 - Verify that the identifier is correctly read */
// 		flashID = sFLASH_ReadIDv2();
// 		status = sFLASH_ReadStatusRegister();
// 		HAL_Delay(10);
// 		sFLASH_GlobalProtectionEnable();
// 		HAL_Delay(10);
//   	sFLASH_EraseSector(0);
// 		HAL_Delay(10);
		
// 		sFLASH_WaitForWriteEnd();
// 		HAL_Delay(1000);
// 		sFLASH_WritePage(testMassiveExtFlash, 107,256);
// 		HAL_Delay(1000);
// 		sFLASH_ReadBuffer(rxDataFlash, 0, 2048);
// 		HAL_Delay(1000);
// 		HAL_Delay(10);
// //		
// 		status = sFLASH_ReadStatusRegister();
		
		
//   if(sFLASH_MX25L6433F_ID != sFLASH_ReadID())
//   {
//     Error_Handler();
//   }

//currentID_flash = sFLASH_ReadIDv2();

//  /* 2 - determines how many sectors are needed to write the test image */
//  sectorsNeeded = 1;
//  remainingBytes = (sizeof(testMassiveExtFlash) % sFLASH_SPI_SECTOR_SIZE);

//  /* 3 - erases and writes the needed sectors */
//  for (currentSector = 0; currentSector < sectorsNeeded; currentSector++)
//  {
//    /* 3.1 - erases current sector */
//  	sFLASH_EraseSector(currentSector * sFLASH_SPI_SECTOR_SIZE);
//  	/* 3.2 - writes current sector */
//  	sFLASH_WriteBuffer(&testMassiveExtFlash[currentSector * sFLASH_SPI_SECTOR_SIZE], currentSector * sFLASH_SPI_SECTOR_SIZE, sFLASH_SPI_SECTOR_SIZE);
//  }
//  if(remainingBytes != 0)
//  {
//  	/* 3.3 - erases last sector for the remaining bytes */
//  	sFLASH_EraseSector(currentSector * sFLASH_SPI_SECTOR_SIZE);
//  	/* 3.4 - writes for the remaining bytes */
//  	sFLASH_WriteBuffer(&testMassiveExtFlash[currentSector * sFLASH_SPI_SECTOR_SIZE], currentSector * sFLASH_SPI_SECTOR_SIZE, remainingBytes);
//  }

//    /* 4 - reads and checks the selected sectors */
//  for (currentSector = 0; currentSector < sectorsNeeded; currentSector++)
//  {
//  	/* 4.1 - reads current sector */
//  	sFLASH_ReadBuffer(rxDataFlash, currentSector * sFLASH_SPI_SECTOR_SIZE, sFLASH_SPI_SECTOR_SIZE);
//  	/* 4.2 - checks current sector */
////  	if(my_memcmp((void *)rxDataFlash, (void *)&testMassiveExtFlash[currentSector * sFLASH_SPI_SECTOR_SIZE], sFLASH_SPI_SECTOR_SIZE) != 0)
////  	//if(memcmp(buffer_read, buffer_test, sFLASH_SPI_SECTOR_SIZE) != 0)
////  	{
////  		Error_Handler();
////  	}
//  }
//  if(remainingBytes != 0)
//  {
//    /* 4.3 - reads current sector */
//  	sFLASH_ReadBuffer(rxDataFlash, currentSector * sFLASH_SPI_SECTOR_SIZE, remainingBytes);
////  	/* 4.4 - checks current sector */
////  	if(my_memcmp((void *)rxDataFlash, (void *)&testMassiveExtFlash[currentSector * sFLASH_SPI_SECTOR_SIZE], remainingBytes) != 0)
////  	{
////      Error_Handler();
////  	}
//  }

//volatile uint16_t tempss = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 5;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
