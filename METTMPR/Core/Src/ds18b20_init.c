#include "ds18b20.h"
#include "onewire.h"
//configure uart
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

void initDS18B20(DS18B20 *temperatureSensor, DS18B20_Status *errorDS18B20, uint8_t numbSensors)
{

    //---- my

    // writing default values to structures //EDIT UART FOR CONFIGURATION
    DS18B20_Init(&temperatureSensor[--numbSensors], &huart7);
    DS18B20_Init(&temperatureSensor[--numbSensors], &huart5);
    DS18B20_Init(&temperatureSensor[--numbSensors], &huart4);
    DS18B20_Init(&temperatureSensor[--numbSensors], &huart3);
    DS18B20_Init(&temperatureSensor[--numbSensors], &huart2);

    //

    for (size_t i = 0; i < numbSensors; i++)
    {

        *(errorDS18B20 + i) |= DS18B20_InitializationCommand(temperatureSensor + i); // reset
        *(errorDS18B20 + i) |= DS18B20_ReadRom(temperatureSensor + i);               // 0x33
        *(errorDS18B20 + i) |= DS18B20_ReadScratchpad(temperatureSensor + i);        // 0xbe func

        // // DS18B20_Init(&temperatureSensor[i], &huart2);
        // errorDS18B20[i] |= DS18B20_InitializationCommand(&temperatureSensor[i]); // reset
        // errorDS18B20[i] |= DS18B20_ReadRom(&temperatureSensor[i]);               // 0x33
        // errorDS18B20[i] |= DS18B20_ReadScratchpad(&temperatureSensor[i]);        // 0xbe func
        // // while (errorDS18B20[i])
        // // {

        // // }
    }

    int8_t settings[3];
    settings[0] = (uint8_t)125;
    settings[1] = (uint8_t)-55;
    settings[2] = DS18B20_12_BITS_CONFIG;

//write configure tresholds
    for (size_t i = 0; i < numbSensors; i++)
    {
        *(errorDS18B20 + i) += DS18B20_InitializationCommand(temperatureSensor + i);
        *(errorDS18B20 + i) += DS18B20_SkipRom(temperatureSensor + i); //cc
        *(errorDS18B20 + i) += DS18B20_WriteScratchpad(temperatureSensor + i, (uint8_t *)&settings[0]);//4e
    }
}

void requestToCalculationTemperature(DS18B20 *temperatureSensor, DS18B20_Status *errorDS18B20, uint8_t numbSensors)
{
    // DS18B20_Init(&temperatureSensor[--numbSensors], &huart7);
    // DS18B20_Init(&temperatureSensor[--numbSensors], &huart5);
    // DS18B20_Init(&temperatureSensor[--numbSensors], &huart4);
    // DS18B20_Init(&temperatureSensor[--numbSensors], &huart3);
    // DS18B20_Init(&temperatureSensor[--numbSensors], &huart2);

    // for (size_t i = 0; i < numbSensors; i++)
    // {

    //     *(errorDS18B20 + i) |= DS18B20_InitializationCommand(temperatureSensor + i); // reset
    //     *(errorDS18B20 + i) |= DS18B20_ReadRom(temperatureSensor + i);               // 0x33
    //     *(errorDS18B20 + i) |= DS18B20_ReadScratchpad(temperatureSensor + i);        // 0xbe func

    // }
    
    // send request to onewire sensor for measuring temperature
    for (size_t i = 0; i < numbSensors; i++)
    {
        *(errorDS18B20 + i) += DS18B20_InitializationCommand(temperatureSensor + i);
        *(errorDS18B20 + i) += DS18B20_SkipRom(temperatureSensor + i);
        *(errorDS18B20 + i) += DS18B20_ConvertT(temperatureSensor + i, DS18B20_NONE);
    }
}

void receiveTemperature(DS18B20 *temperatureSensor, DS18B20_Status *errorDS18B20, uint8_t numbSensors)
{


    // receiving from onewire sensor calculate temperature
    for (size_t i = 0; i < numbSensors; i++)
    {
        *(errorDS18B20 + i) += DS18B20_InitializationCommand(temperatureSensor + i);
        *(errorDS18B20 + i) += DS18B20_SkipRom(temperatureSensor + i);
        *(errorDS18B20 + i) += DS18B20_ReadScratchpad(temperatureSensor + i);
    }
}
