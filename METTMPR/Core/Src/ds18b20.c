/**
  ******************************************************************************
  * @file           : ds18b20.c
  * @brief          : DS18B20 driver
  * @author         : MicroTechnics (microtechnics.ru)
  ******************************************************************************
  */

//чатГПТ3.5
// CalculateChecksum: Эта функция вычисляет контрольную сумму данных, полученных от датчика. Она использует алгоритм CRC-8 для вычисления байта контрольной суммы.
// ExecuteCommand: Эта функция отправляет команду на датчик DS18B20 и получает ответ. Она обрабатывает как команды чтения, так и записи, парся ответные данные соответственно.
// WaitForConversionFinished: Эта функция ждет завершения преобразования температуры. Она непрерывно проверяет статус преобразования до его завершения.
// DS18B20_ConvertT: Эта функция инициирует преобразование температуры на датчике DS18B20. Она может либо ждать завершения преобразования, либо ввести задержку в зависимости от настройки разрешения датчика.
// DS18B20_ReadScratchpad: Эта функция считывает содержимое регистра Scratchpad датчика DS18B20, который содержит данные о температуре и настройки конфигурации. Затем она разбирает полученные данные для извлечения соответствующей информации, такой как температура и конфигурация.
// DS18B20_WriteScratchpad: Эта функция записывает данные в регистр Scratchpad датчика DS18B20. Она обновляет пределы температуры и настройки конфигурации.
// DS18B20_InitializationCommand: Эта функция отправляет инициализационную команду на датчик DS18B20 для проверки его присутствия на шине OneWire. Она проверяет, подключен ли датчик и правильно ли он отвечает.
// DS18B20_ReadRom: Эта функция считывает уникальный код ROM (только для чтения) датчика DS18B20. Этот код идентифицирует отдельный датчик на шине.
// DS18B20_SkipRom: Эта функция отправляет команду пропуска ROM, чтобы адресовать все датчики DS18B20 на шине одновременно. Она полезна, когда на шине присутствует только один датчик.
// DS18B20_Init: Эта функция инициализирует интерфейс датчика DS18B20. Она устанавливает начальные значения переменных и помечает датчик как инициализированный.


/* Includes ------------------------------------------------------------------*/

#include "ds18b20.h"
#include "onewire.h"

  uint8_t tdata[2];

/* Declarations and definitions ----------------------------------------------*/
// extern UART_HandleTypeDef huart1;

// ROM commands
static DS18B20_Command readRom = {.code = 0x33, .rxBytesNum = 8, .txBytesNum = 0};
static DS18B20_Command skipRom = {.code = 0xCC, .rxBytesNum = 0, .txBytesNum = 0};

// Function commands
static DS18B20_Command readScratchpad = {.code = 0xBE, .rxBytesNum = 9, .txBytesNum = 0};
static DS18B20_Command writeScratchpad = {.code = 0x4E, .rxBytesNum = 0, .txBytesNum = 3};
static DS18B20_Command convertT = {.code = 0x44, .rxBytesNum = 0, .txBytesNum = 0};



/* Functions -----------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static uint8_t CalculateChecksum(uint8_t *data, uint8_t length)
{
  uint8_t checksum = 0;

  while (length--)
  {
    uint8_t currentByte = *data++;

    for (uint8_t i = 8; i; i--)
    {
      uint8_t temp = (checksum ^ currentByte) & 0x01;
      checksum >>= 1;

      if (temp)
      {
        checksum ^= 0x8C;
      }

      currentByte >>= 1;
    }
  }

  return checksum;
}



/*----------------------------------------------------------------------------*/
static DS18B20_Status ExecuteCommand(DS18B20 *sensor, DS18B20_Command command, uint8_t *data)
{
  if (sensor->isConnected == 0)
  {
    return DS18B20_ERROR;
  }

  OneWire_ProcessByte(sensor->uart, command.code);

  if (command.rxBytesNum != 0)
  {
    for (uint8_t i = 0; i < command.rxBytesNum; i++)
    {
      data[i] = OneWire_ProcessByte(sensor->uart, 0xFF);
    }

    uint8_t checkSum = CalculateChecksum(data, command.rxBytesNum - 1);
    if (checkSum != data[command.rxBytesNum - 1])
    {
      return DS18B20_ERROR;
    }
  }
  else
  {
    for (uint8_t i = 0; i < command.txBytesNum; i++)
    {
      OneWire_ProcessByte(sensor->uart, data[i]);
    }
  }

  return DS18B20_OK;
}



/*----------------------------------------------------------------------------*/
static void WaitForConversionFinished(DS18B20 *sensor)
{
  uint8_t data = OneWire_ProcessBit(sensor->uart, 1);
  while(data != 0xFF)
  {
    data = OneWire_ProcessBit(sensor->uart, 1);
  }
}



/*----------------------------------------------------------------------------*/
DS18B20_Status DS18B20_ConvertT(DS18B20 *sensor, DS18B20_WaitCondition waitCondition)
{
  DS18B20_Status result;
  uint8_t rxDummyData;

  result = ExecuteCommand(sensor, convertT, &rxDummyData);

  if (waitCondition == DS18B20_DATA)
  {
    WaitForConversionFinished(sensor);
  }

  if (waitCondition == DS18B20_DELAY)
  {
    uint32_t delayValueMs = 0;

    switch (sensor->configRegister)
    {
      case DS18B20_9_BITS_CONFIG:
        delayValueMs = DS18B20_9_BITS_DELAY_MS;
        break;

      case DS18B20_10_BITS_CONFIG:
        delayValueMs = DS18B20_10_BITS_DELAY_MS;
        break;

      case DS18B20_11_BITS_CONFIG:
        delayValueMs = DS18B20_11_BITS_DELAY_MS;
        break;

      case DS18B20_12_BITS_CONFIG:
        delayValueMs = DS18B20_12_BITS_DELAY_MS;
        break;

      default:
        break;
    }

    HAL_Delay(delayValueMs);
  }

  return result;
}



/*----------------------------------------------------------------------------*/
DS18B20_Status DS18B20_ReadScratchpad(DS18B20 *sensor)
{
  DS18B20_Status result;
  uint8_t rxData[DS18B20_READ_SCRATCHPAD_RX_BYTES_NUM];
  result = ExecuteCommand(sensor, readScratchpad, rxData);

  if (result != DS18B20_OK)
  {
    return result;
  }

  sensor->temperatureLimitHigh = rxData[DS18B20_SCRATCHPAD_T_LIMIT_H_BYTE_IDX];
  sensor->temperatureLimitLow = rxData[DS18B20_SCRATCHPAD_T_LIMIT_L_BYTE_IDX];
  sensor->configRegister = rxData[DS18B20_SCRATCHPAD_CONFIG_BYTE_IDX];
  tdata[1]=rxData[DS18B20_SCRATCHPAD_T_MSB_BYTE_IDX];
	tdata[0]=rxData[DS18B20_SCRATCHPAD_T_LSB_BYTE_IDX];
  uint16_t tRegValue = (rxData[DS18B20_SCRATCHPAD_T_MSB_BYTE_IDX] << 8) | rxData[DS18B20_SCRATCHPAD_T_LSB_BYTE_IDX];
  sensor->sourceTemperature = tRegValue;
  uint16_t sign = tRegValue & DS18B20_SIGN_MASK;

  if (sign != 0)
  {
    tRegValue = (0xFFFF - tRegValue + 1);
  }
//		data[1]=tRegValue;
//	  data[0]=tRegValue>>8;
  switch (sensor->configRegister)
  {
    case DS18B20_9_BITS_CONFIG:
      tRegValue &= DS18B20_9_BITS_DATA_MASK;
      break;

    case DS18B20_10_BITS_CONFIG:
      tRegValue &= DS18B20_10_BITS_DATA_MASK;
      break;

    case DS18B20_11_BITS_CONFIG:
      tRegValue &= DS18B20_11_BITS_DATA_MASK;
      break;

    case DS18B20_12_BITS_CONFIG:
      tRegValue &= DS18B20_12_BITS_DATA_MASK;
      break;

    default:
      tRegValue &= DS18B20_12_BITS_DATA_MASK;
      break;
  }

	
  sensor->temperature = (float)tRegValue * DS18B20_T_STEP;
	
//		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET );
//		HAL_UART_Transmit(&huart1,tdata,2,HAL_MAX_DELAY);
//		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET );

  if (sign != 0)
  {
    sensor->temperature *= (-1);
  }

  return DS18B20_OK;
}



/*----------------------------------------------------------------------------*/
DS18B20_Status DS18B20_WriteScratchpad(DS18B20 *sensor, uint8_t *data)
{
  DS18B20_Status result;

  result = ExecuteCommand(sensor, writeScratchpad, data);

  if (result != DS18B20_OK)
  {
    return result;
  }

  sensor->temperatureLimitHigh = data[0];
  sensor->temperatureLimitLow = data[1];
  sensor->configRegister = data[2];

  return result;
}



/*----------------------------------------------------------------------------*/
DS18B20_Status DS18B20_InitializationCommand(DS18B20 *sensor)
{
  if (sensor->isInitialized == 0)
  {
    return DS18B20_ERROR;
  }

  ONEWIRE_Status status = OneWire_Reset(sensor->uart);

  if (status == ONEWIRE_OK)
  {
    sensor->isConnected = 1;
    return DS18B20_OK;
  }
  else
  {
    sensor->isConnected = 0;
    return DS18B20_ERROR;
  }
}



/*----------------------------------------------------------------------------*/
DS18B20_Status DS18B20_ReadRom(DS18B20 *sensor)
{
  DS18B20_Status result;
  uint8_t rxData[DS18B20_READ_ROM_RX_BYTES_NUM];
  result = ExecuteCommand(sensor, readRom, rxData);

  if (result != DS18B20_OK)
  {
    return result;
  }

  for (uint8_t i = 0; i < DS18B20_SERIAL_NUMBER_LEN_BYTES; i++)
  {
    sensor->serialNumber[i] = rxData[DS18B20_SERIAL_NUMBER_OFFSET_BYTES + i];
  }

  return DS18B20_OK;
}



/*----------------------------------------------------------------------------*/
DS18B20_Status DS18B20_SkipRom(DS18B20 *sensor)
{
  DS18B20_Status result;
  uint8_t rxDummyData;
  result = ExecuteCommand(sensor, skipRom, &rxDummyData);

  if (result != DS18B20_OK)
  {
    return result;
  }

  return DS18B20_OK;
}



/*----------------------------------------------------------------------------*/
void DS18B20_Init(DS18B20 *sensor, UART_HandleTypeDef *huart)
{
  sensor->isConnected = 0;
  sensor->uart = huart;
  sensor->isInitialized = 1;
}



/*----------------------------------------------------------------------------*/
