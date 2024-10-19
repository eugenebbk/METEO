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
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "SST26.h"
#include "parserPC.h"
#include "LogsSST26.h"
#include "parserTemperatureBoard.h"
#include "parserMETEO.h"
#include "formCMD.h"
#include "meteostation.h"

#include "parserNMEA.h"
#include "minmea.h"
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

configureModeMK_t configureModeMK = {0};
flagsInterrupts_t flagsInterrupts = {0};
log3_t log3 = {0};

//------------------------usb

uint8_t requestUSB[SIZE_RX_USB_BUFFER] = {0};

//----------------
dataTemperatureMETTMPR_t dataTemperatureMETTMPR = {0};
//------------
uint8_t str[1];
uint8_t dataReceived = 0;    // ������� ������ ��������
uint8_t dataTransmitted = 1; // ������� ������ ��������
uint8_t *Buf;

uint8_t flagBoard = 0;            // ??????? ?????? ????????
uint8_t receiveRingBuf[32] = {0}; // ??????? ?????? ????????

uint8_t requestTemperature = 0x5e;
// uint8_t exitTemperature8[12] = {0};

// uart

#define SIZE_MASSIVE_FOR_ACCUM 768
uint8_t RxDataUSART1[SIZE_MASSIVE_FOR_ACCUM] = {0};
uint8_t RxDataUSART3[32] = {0};
uint8_t RxDataUSART4[12] = {0};
#define SIZE_MASSIVE_FOR_NMEA 128
uint8_t RxDataUSART6[SIZE_MASSIVE_FOR_NMEA] = {0};

uint8_t TxDataUSART3[8] = {0}; // meteostation
// uint8_t TxDataUSART4[12] = {0}; // meteostation
uint8_t TxDataUSART4 = 0; // meteostation

//-------------gnss

//int indxGNSS = 0;
struct minmea_sentence_zda frame_zda = {0};
struct minmea_sentence_gll frame_gll = {0};
uint8_t *massiveParser_ptr = NULL;

//-----------meteo
dataMeteostation_t dataMeteostation = {0};
errorMeteostation_t errorMeteostation = {0};
uint8_t stateCollectData = 0;
// uint8_t RxData[12] = {0};
// uint8_t RxData6[60] = {0};
int indx = 0;
uint8_t fullPacket[12] = {0xFD, 0x55, 0};

//----------Log
// log1_t log1 = {0};
uint32_t currentID_log = 1;
//-----------extern flash

uint16_t numbSectorCurrent = 0;
uint32_t flashID = 0;
//---------mode Work
uint8_t modeMK = 0;
eModeMK_t eModeMK = DEFAULT_MODE;

uint8_t en_dis_interruptsBridges = 0;

//------------USB

uint8_t RxDataUSB[255] = {0};
uint32_t RxDataUSB_len = 0;

//---------Debug

uint32_t defaultTime = 0x937;
uint32_t defaultDate = 0x0134D879;
uint16_t countLogID = 0;

uint8_t testMassiveExtFlash[256] = {0};
uint8_t testValue8 = 0xAA;
uint32_t testValue = 0xAAAAAAAA;
uint32_t freeAdressExtFlash = 0;
uint8_t rxDataFlash[256] = {0};
uint8_t statusFlashExt = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

uint8_t parserRequestPC(uint8_t *messageRequestPC);
uint8_t stateMachineCollectData(void);
uint8_t disableInterruptInterfaces(uint8_t modeMK);
uint8_t enableInterruptInterfaces(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2) // 30s
  {
    if (stateCollectData == 0)
    {
      stateCollectData = 1;
    }
    // flagsInterrupts.COLLECT_DATA_int = 1;
  }
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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_SR_UIF);
  // HAL_TIM_Base_Start_IT(&htim2);
//-------init massive for nmea

  memset(&RxDataUSART6, '$', SIZE_MASSIVE_FOR_NMEA);

  HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxDataUSART1, sizeof(RxDataUSART1));
  HAL_UARTEx_ReceiveToIdle_IT(&huart3, RxDataUSART3, sizeof(RxDataUSART3));
  HAL_UARTEx_ReceiveToIdle_IT(&huart4, RxDataUSART4, sizeof(RxDataUSART4));
  HAL_UARTEx_ReceiveToIdle_IT(&huart6, RxDataUSART6, sizeof(RxDataUSART6));


//---------------
  formMeteoRequestCMD_simple(CMD_METEOSTATION_START, TxDataUSART3);
  PIN_EN_TRANSMIT_UART3(1);
  HAL_UART_Transmit_IT(&huart3, TxDataUSART3, CMD_METEOSTATION_START_SiZE);
  HAL_Delay(10);
  PIN_EN_TRANSMIT_UART3(0);
uint32_t counter_temp = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		counter_temp++;
		if(counter_temp==3600000){ //debug
			counter_temp = 0;
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_11);
		}
		
		if(eModeMK == DEFAULT_MODE){
			if(en_dis_interruptsBridges){
				en_dis_interruptsBridges = 0;
				enableInterruptInterfaces();
			}
			
			if (flagsInterrupts.USB_VCP_int == 1)
			{
				flagsInterrupts.USB_VCP_int = 0;
				parserRequestPC(RxDataUSB);
			}
		}
		else if(eModeMK == COLLECT_DATA_MODE){
			if(en_dis_interruptsBridges){
				en_dis_interruptsBridges = 0;
				enableInterruptInterfaces();
			}
			stateMachineCollectData();
			
			if (flagsInterrupts.USB_VCP_int == 1)
			{
				flagsInterrupts.USB_VCP_int = 0;
//				if(RxDataUSB[0] == HEADER_TO_DEFAULT_MODE && RxDataUSB[1] == HEADER_TO_DEFAULT_MODE){
//					memset(&log3.ID, 0, sizeof(log3_t));
//					enableInterruptInterfaces();
//					eModeMK = DEFAULT_MODE;
////					parserRequestPC(RxDataUSB);
//				}
//				if(RxDataUSB[1] == ExitFromCmdToDefault){
////					enableInterruptInterfaces();
////					eModeMK = DEFAULT_MODE;
//					parserRequestPC(RxDataUSB);
//				}
				parserRequestPC(RxDataUSB);
			}

			if (flagsInterrupts.UART_TEMPERATURE_int == 1)
			{
				flagsInterrupts.UART_TEMPERATURE_int = 0;
				if (!parserTemperatureBoard(RxDataUSART4))
				{
					for (uint8_t i = 0; i < NUMB_TEMPBOARD_SENSOR; i++)
					{
						log3.Temperature[i] = (int32_t)dataTemperatureMETTMPR.temperatureMETTMPR[i];
					}
					stateCollectData = 0;
				}
			}
			if (flagsInterrupts.UART_METEOBLOCK_int == 1)
			{
				flagsInterrupts.UART_METEOBLOCK_int = 0;
				uint8_t resultParser = parserMeteoStation_simple(RxDataUSART3);
				if (resultParser != 1)
				{
					if (resultParser == 0x43)
					{
						memcpy(&log3.ExtMetTMPRTR_Heater, &dataMeteostation.Tmperature_Heater, 20);
						stateCollectData = 2;
					}
				}
				else{
					stateCollectData = 1;
				}
			}
			if (flagsInterrupts.UART_GNSS_int == 1)
			{
				flagsInterrupts.UART_GNSS_int = 0;
				uint8_t successDataGNSS = readData((const uint8_t *)&RxDataUSART6);
				if(successDataGNSS){ //write to log
					//добавить в лог данные с ГНСС //переделать в другие логи???
					memcpy(&log3.Date[0],&frame_zda.date.day ,(12));
					memcpy(&log3.Time[0],&frame_zda.time.hours,(16));
					memcpy(&log3.Coordinate_oX[0],&frame_gll.longitude.value ,(8));
					memcpy(&log3.Coordinate_oY[0],&frame_gll.latitude.value ,(8));
				}
				memset(&RxDataUSART6, '$', SIZE_MASSIVE_FOR_NMEA);
				HAL_UARTEx_ReceiveToIdle_IT(&huart6, RxDataUSART6, sizeof(RxDataUSART6));
			}
		}
		else if(eModeMK == BRIDGE_METEOBLOCK_MODE){
			if(en_dis_interruptsBridges){
				enableInterruptInterfaces();
				disableInterruptInterfaces(BRIDGE_METEOBLOCK_MODE);
				en_dis_interruptsBridges = 0;
			}
			
			if (flagsInterrupts.USB_VCP_int == 1)
			{
				flagsInterrupts.USB_VCP_int = 0;
				if(RxDataUSB[0] == METEOBLOCK_ADDRESS){
					HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_11);
					PIN_EN_TRANSMIT_USART3(1);
					HAL_UART_Transmit_IT(&huart3, RxDataUSB, RxDataUSB_len);
					HAL_Delay(6);
					PIN_EN_TRANSMIT_USART3(0);
					}
//				else if(RxDataUSB[0] == REQUEST_PC_HEADER && RxDataUSB[1] == ExitFromCmdToDefault)
				else{
					parserRequestPC(RxDataUSB);
				}
			}
		}
		else if(eModeMK == BRIDGE_GNSS_MODE){
			if(en_dis_interruptsBridges){
				en_dis_interruptsBridges = 0;
				enableInterruptInterfaces();
				disableInterruptInterfaces(BRIDGE_GNSS_MODE);
			}	
			if (flagsInterrupts.USB_VCP_int == 1)
			{
				flagsInterrupts.USB_VCP_int = 0;
				
//				if(RxDataUSB[0] == REQUEST_PC_HEADER){
//					parserRequestPC(RxDataUSB);
//				}
				parserRequestPC(RxDataUSB);
			}
		}
		else if(eModeMK == BRIDGE_ACCUMULATOR_MODE){
			if(en_dis_interruptsBridges){
				en_dis_interruptsBridges = 0;
				enableInterruptInterfaces();
				disableInterruptInterfaces(BRIDGE_ACCUMULATOR_MODE);
			}
			
			if (flagsInterrupts.USB_VCP_int == 1)
			{
				flagsInterrupts.USB_VCP_int = 0;
				
//				if(RxDataUSB[0] == REQUEST_PC_HEADER && RxDataUSB[1] == HEADER_EXIT_CMD2){
				if(RxDataUSB[0] == REQUEST_PC_HEADER){
					parserRequestPC(RxDataUSB);
				}
				
//				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_11);
//				PIN_EN_TRANSMIT_USART1(1);
//				HAL_UART_Transmit_IT(&huart1, RxDataUSB, RxDataUSB_len);
//				HAL_Delay(3);
//				PIN_EN_TRANSMIT_USART1(0);
			}
		}
		else if(eModeMK == BRIDGE_TMPRTRBRD_MODE){
			if(en_dis_interruptsBridges){
				en_dis_interruptsBridges = 0;
				enableInterruptInterfaces();
				disableInterruptInterfaces(BRIDGE_TMPRTRBRD_MODE);
			}
			
			if (flagsInterrupts.USB_VCP_int == 1)
			{
				flagsInterrupts.USB_VCP_int = 0;
				if(RxDataUSB[0] == HEADER_METTMPR_BOARD1 && RxDataUSB[1] == HEADER_METTMPR_BOARD2){
					
					PIN_EN_TRANSMIT_UART4(1);
					HAL_UART_Transmit_IT(&huart4, RxDataUSB, RxDataUSB_len);
					HAL_Delay(6);
					PIN_EN_TRANSMIT_UART4(0);
					}
				else{
					parserRequestPC(RxDataUSB);
				}
			}		
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
  indx = Size;
  if (huart->Instance == USART1) // accum
  {
		
//		if(eModeMK == COLLECT_DATA_MODE)
//		{
//			if (memcmp(RxDataUSART1, TxDataUSART1, Size)){
//				flagsInterrupts.UART_ACCUM_int = 1;
//			}
//		}
//		else if(eModeMK == BRIDGE_ACCUMULATOR_MODE){
		if(eModeMK == BRIDGE_ACCUMULATOR_MODE){
			if (memcmp(RxDataUSART1, RxDataUSB, Size)){
				CDC_Transmit_FS(&RxDataUSART1[0], Size);
			}		
		}
    else{
    }	
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxDataUSART1, sizeof(RxDataUSART1));
  }
  if (huart->Instance == USART3) // meteoblock
  {
		
		if(eModeMK == COLLECT_DATA_MODE)
		{
			if (memcmp(RxDataUSART3, TxDataUSART3, Size)){
				flagsInterrupts.UART_METEOBLOCK_int = 1;
			}
		}
		else if(eModeMK == BRIDGE_METEOBLOCK_MODE){
			if (memcmp(RxDataUSART3, RxDataUSB, Size)){
				CDC_Transmit_FS(&RxDataUSART3[0], Size);
			}		
		}
    else{
			
    }	
    HAL_UARTEx_ReceiveToIdle_IT(&huart3, RxDataUSART3, sizeof(RxDataUSART3));
  }
  if (huart->Instance == UART4) // temperature
  {
		if(eModeMK == COLLECT_DATA_MODE)
		{
			if (memcmp(RxDataUSART4, &TxDataUSART4, Size)){
				flagsInterrupts.UART_TEMPERATURE_int = 1;
			}
		}
		else if(eModeMK == BRIDGE_TMPRTRBRD_MODE){
			if (memcmp(RxDataUSART4, RxDataUSB, Size)){
				CDC_Transmit_FS(&RxDataUSART4[0], Size);
			}		
		}
    else{
    }	
    HAL_UARTEx_ReceiveToIdle_IT(&huart4, RxDataUSART4, sizeof(RxDataUSART4));
//    if (memcmp(RxDataUSART4, &TxDataUSART4, 1) != 0)
//    {
//      flagsInterrupts.UART_TEMPERATURE_int = 1;
//    }
//    HAL_UARTEx_ReceiveToIdle_IT(&huart4, RxDataUSART4, sizeof(RxDataUSART4));
  }
  
	if (huart->Instance == USART6) // GNSS
  {
		if(eModeMK == BRIDGE_GNSS_MODE){
			CDC_Transmit_FS(&RxDataUSART6[0], Size);
		}
		else if(eModeMK == COLLECT_DATA_MODE){
			flagsInterrupts.UART_GNSS_int = 1;
		}
//    __HAL_UART_CLEAR_IDLEFLAG(&huart6);
    HAL_UARTEx_ReceiveToIdle_IT(&huart6, RxDataUSART6, sizeof(RxDataUSART6));
  }
}

uint8_t parserRequestPC(uint8_t *messageRequestPC)
{
  // uint8_t messageAnswerPC[261] = {0};

  usb_protocol_t usb_protocol = {0};

  usb_protocol.headerConst = REQUEST_PC_HEADER;
  usb_protocol.cmd = messageRequestPC[1];
  usb_protocol.lenghtPayload = 1;

  static uint8_t stateMachineParserPC = 0;
  uint8_t errorParserPC = 0;

  uint8_t payloadLenghtParserPC = messageRequestPC[2];
  uint16_t resultCompareCRC_temp = func_compare_crc16(messageRequestPC, REQUEST_PC_HEADER_SIZE + payloadLenghtParserPC + REQUEST_PC_CRC_SIZE); //

  uint8_t *payload_ptr = messageRequestPC + REQUEST_PC_HEADER_SIZE - 1;
  if (messageRequestPC[0] == REQUEST_PC_HEADER && resultCompareCRC_temp == 0 && payloadLenghtParserPC < 2) // ���� �������� ���� ��������� � ������
  {
    switch (messageRequestPC[1]) // ������ ������������ ������ �������
    {
//      if (stateMachineParserPC == 0) //
//      {
      case WriteTimePeriodEventLog: //
//        configureModeMK.periodWritingDataToLog = (messageRequestPC[3] << 8) && (messageRequestPC[4]);

        // ����� ��������

        usb_protocol.payloadAndCRC[0] = 0;
        break; // ����� �� ����� �������

      case ReadEventLog: //-
        usb_protocol.lenghtPayload = (uint16_t)SIZE_LOG;

        // �������� ����� � ��� ��� �������
        //  ����������
        memcpy(&usb_protocol.payloadAndCRC[0], (uint8_t *)&log3.ID, SIZE_LOG); // ����������� ����
        break;                                                                 // ����� �� ����� �������

      case ClearEventLog:

        // ����������
        usb_protocol.payloadAndCRC[0] = 0;
        break; // ����� �� ����� �������

      case ModeBridgeMeteoblock:
				en_dis_interruptsBridges =1;
//        configureModeMK.currentModeMK = ModeBridgeMeteoblock;
		
			eModeMK = BRIDGE_METEOBLOCK_MODE;
        // ����������
        usb_protocol.payloadAndCRC[0] = 0;
        break; // ����� �� ����� �������

      case ModeBridgeGNSS:
				en_dis_interruptsBridges =1;
//        configureModeMK.currentModeMK = ModeBridgeGNSS;
			eModeMK = BRIDGE_GNSS_MODE;

        // ����������
        usb_protocol.payloadAndCRC[0] = 0;
        break; // ����� �� ����� �������

      case ModeBridgeAccumulator:
				en_dis_interruptsBridges =1;
//        configureModeMK.currentModeMK = ModeBridgeAccumulator;
			eModeMK = BRIDGE_ACCUMULATOR_MODE;

        // ����������
        usb_protocol.payloadAndCRC[0] = 0;
        break; // ����� �� ����� �������
			
			
      case MODE_BRIDGE_TEMPRBOARD:
				en_dis_interruptsBridges =1;
				eModeMK = BRIDGE_TMPRTRBRD_MODE;
        // ����������
        usb_protocol.payloadAndCRC[0] = 0;
        break; // ����� �� ����� �������
			
//      }

    case StopWriteDataToLog:
////      configureModeMK.currentModeMK = StopWriteDataToLog;
//      usb_protocol.lenghtPayload = 1;
//      stateMachineParserPC = 0;
//      //
//      HAL_TIM_Base_Stop_IT(&htim2);
//      __HAL_TIM_SET_COUNTER(&htim2, 0);
//		
//			eModeMK = DEFAULT_MODE;
      // ����������
      usb_protocol.payloadAndCRC[0] = 0;
      break; // ����� �� ����� �������

    case ExitFromCmdToDefault:
      usb_protocol.lenghtPayload = 1;
		
		//vse obnulit`
      stateMachineParserPC = 0;
      HAL_TIM_Base_Stop_IT(&htim2);
      __HAL_TIM_SET_COUNTER(&htim2, 0);
			enableInterruptInterfaces();
		
			eModeMK = DEFAULT_MODE;
      // ����������
      usb_protocol.payloadAndCRC[0] = 0;
      break;

    case StartWriteDataToLog:
			en_dis_interruptsBridges =1;
//      configureModeMK.currentModeMK = StartWriteDataToLog;
      usb_protocol.lenghtPayload = 1;
      stateMachineParserPC = 1;

			eModeMK = COLLECT_DATA_MODE;
      // ���������� �������
      HAL_TIM_Base_Start_IT(&htim2);
      // ����������

      usb_protocol.payloadAndCRC[0] = 0;
      break; // ����� �� ����� �������

      // case AnswerData:
      //     configureModeMK.currentModeMK = StartWriteDataToLog;
      //     stateMachineParserPC = 1;

      //     // ����������
      //     usb_protocol.payloadAndCRC[0] = 0;
      //     break; // ����� �� ����� �������

    default:
      usb_protocol.payloadAndCRC[0] = 2; // data

      uint16_t calcCRC16 = func_calc_crc16((uint8_t *)&usb_protocol.headerConst, REQUEST_PC_HEADER_SIZE + usb_protocol.lenghtPayload);
      usb_protocol.payloadAndCRC[usb_protocol.lenghtPayload] = (uint8_t)(calcCRC16 >> 8);
      usb_protocol.payloadAndCRC[usb_protocol.lenghtPayload+1] = (uint8_t)(calcCRC16 & 0xFF);
      errorParserPC = 2; // error cmd
      break;             // �����

    } // ����� ������������ ������ �������
    uint16_t calcCRC16 = func_calc_crc16((uint8_t *)&usb_protocol.headerConst, REQUEST_PC_HEADER_SIZE + usb_protocol.lenghtPayload);
    usb_protocol.payloadAndCRC[usb_protocol.lenghtPayload] = (uint8_t)(calcCRC16 >> 8);
    usb_protocol.payloadAndCRC[usb_protocol.lenghtPayload+1] = (uint8_t)(calcCRC16 & 0xFF);
  }
  else
  {
    usb_protocol.payloadAndCRC[0] = 1;

    uint16_t calcCRC16 = func_calc_crc16((uint8_t *)&usb_protocol.headerConst, REQUEST_PC_HEADER_SIZE + usb_protocol.lenghtPayload);
    usb_protocol.payloadAndCRC[usb_protocol.lenghtPayload] = (uint8_t)(calcCRC16 >> 8);
    usb_protocol.payloadAndCRC[usb_protocol.lenghtPayload+1] = (uint8_t)(calcCRC16 & 0xFF);

    errorParserPC = 1; // error crc and header
  }

  CDC_Transmit_FS((uint8_t *)&usb_protocol.headerConst, PROTOCOL_USB_FULLSIZE + usb_protocol.lenghtPayload);
  return errorParserPC; // no error or other error
}

uint8_t stateMachineCollectData(void)
{
  switch (stateCollectData)
  {

  // init METEOSTATION
  case 0:
    stateCollectData = 0;
    break;

  // init METEOSTATION
  case 1:
    formMeteoRequestCMD_simple(CMD_READ_DATA, TxDataUSART3);
    PIN_EN_TRANSMIT_UART3(1);
    HAL_UART_Transmit_IT(&huart3, TxDataUSART3, CMD_READ_DATA_SiZE);
    HAL_Delay(3);
    PIN_EN_TRANSMIT_UART3(0);
		stateCollectData = 0;
    // stateCollectData++;
    break;

  //
  case 2:
    // formMeteoRequestCMD_simple(CMD_READ_DATA, TxDataUSART4);
    TxDataUSART4 = 0x5e;
    PIN_EN_TRANSMIT_UART4(1);
    HAL_UART_Transmit_IT(&huart4, &TxDataUSART4, CMD_READ_DATA_SiZE);
    HAL_Delay(10);
    PIN_EN_TRANSMIT_UART4(0);
    stateCollectData++;
    break;
	
//  case 3:
//    // formMeteoRequestCMD_simple(CMD_READ_DATA, TxDataUSART4);
////    TxDataUSART4 = 0x5e;
////    PIN_EN_TRANSMIT_UART4(1);
////    HAL_UART_Transmit_IT(&huart4, &TxDataUSART4, CMD_READ_DATA_SiZE);
////    HAL_Delay(10);
////    PIN_EN_TRANSMIT_UART4(0);
//    stateCollectData++;
//    break;

  default:
    stateCollectData = 0;
    break;
    // return 0xFF; // ready
  }
  return 0x00; // busy
}


uint8_t disableInterruptInterfaces(uint8_t modeMK){

	HAL_StatusTypeDef resultOperation = 0;
	//disable uart
	switch (modeMK) {
		case BRIDGE_METEOBLOCK_MODE:
			resultOperation = HAL_UART_AbortReceive_IT (&huart6);
			resultOperation |= HAL_UART_AbortReceive_IT (&huart1);
			resultOperation |= HAL_UART_AbortReceive_IT (&huart4);
		break;
		
		case BRIDGE_GNSS_MODE:
			resultOperation = HAL_UART_AbortReceive_IT (&huart3);
			resultOperation |= HAL_UART_AbortReceive_IT (&huart1);
			resultOperation |= HAL_UART_AbortReceive_IT (&huart4);
		break;
		
		case BRIDGE_ACCUMULATOR_MODE:
			resultOperation = HAL_UART_AbortReceive_IT (&huart3);
			resultOperation |= HAL_UART_AbortReceive_IT (&huart6);
			resultOperation |= HAL_UART_AbortReceive_IT (&huart4);
		break;
		
		case BRIDGE_TMPRTRBRD_MODE:
			resultOperation = HAL_UART_AbortReceive_IT (&huart3);
			resultOperation |= HAL_UART_AbortReceive_IT (&huart6);
			resultOperation |= HAL_UART_AbortReceive_IT (&huart1);
		break;
		
		/*...*/
		default:
		/*���, ������� ����������, ���� ������ �� ���������� �������� �� ������������� �������� � ���������� variable*/
		return 255;
		break;
		}
	
	//timer
		HAL_TIM_Base_Stop_IT(&htim2);
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		return resultOperation;
}



//void enableInterruptInterfaces(uint8_t modeMK){
uint8_t enableInterruptInterfaces(void){

	HAL_StatusTypeDef resultOperation = 0;
	resultOperation |= HAL_UARTEx_ReceiveToIdle_IT(&huart6, RxDataUSART6, sizeof(RxDataUSART6));
	resultOperation |= HAL_UARTEx_ReceiveToIdle_IT(&huart3, RxDataUSART3, sizeof(RxDataUSART3));
	resultOperation |= HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxDataUSART1, sizeof(RxDataUSART1));
	resultOperation |= HAL_UARTEx_ReceiveToIdle_IT(&huart4, RxDataUSART4, sizeof(RxDataUSART4));
//	
//	//disable uart
//	switch (modeMK) {
//		case BRIDGE_METEOBLOCK_MODE:
//    resultOperation = HAL_UARTEx_ReceiveToIdle_IT(&huart6, RxDataUSART6, sizeof(RxDataUSART6));
//    resultOperation = HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxDataUSART1, sizeof(RxDataUSART1));
//    resultOperation = HAL_UARTEx_ReceiveToIdle_IT(&huart4, RxDataUSART4, sizeof(RxDataUSART4));	
//		break;
//		
//		case BRIDGE_GNSS_MODE:
//    resultOperation = HAL_UARTEx_ReceiveToIdle_IT(&huart3, RxDataUSART3, sizeof(RxDataUSART3));
//    resultOperation = HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxDataUSART1, sizeof(RxDataUSART1));
//    resultOperation = HAL_UARTEx_ReceiveToIdle_IT(&huart4, RxDataUSART4, sizeof(RxDataUSART4));
//		break;
//		
//		case BRIDGE_ACCUMULATOR_MODE:
//    resultOperation = HAL_UARTEx_ReceiveToIdle_IT(&huart6, RxDataUSART6, sizeof(RxDataUSART6));
//    resultOperation = HAL_UARTEx_ReceiveToIdle_IT(&huart3, RxDataUSART3, sizeof(RxDataUSART3));
//    resultOperation = HAL_UARTEx_ReceiveToIdle_IT(&huart4, RxDataUSART4, sizeof(RxDataUSART4));
//		break;
//		
//		case BRIDGE_TMPRTRBRD_MODE:
//    resultOperation = HAL_UARTEx_ReceiveToIdle_IT(&huart6, RxDataUSART6, sizeof(RxDataUSART6));
//    resultOperation = HAL_UARTEx_ReceiveToIdle_IT(&huart3, RxDataUSART3, sizeof(RxDataUSART3));
//    resultOperation = HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxDataUSART1, sizeof(RxDataUSART1));
//		break;
//		
//		/*...*/
//		default:
//		/*���, ������� ����������, ���� ������ �� ���������� �������� �� ������������� �������� � ���������� variable*/
//		return;
//		break;
//		}
	//timer
//      __HAL_TIM_SET_COUNTER(&htim2, 0);
//      HAL_TIM_Base_Start_IT(&htim2);
	return resultOperation;
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
