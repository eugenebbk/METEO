#include "crcANSI.h"
#include "parserMETEO.h"
#include "meteostation.h"

extern errorMeteostation_t errorMeteostation;
extern dataMeteostation_t dataMeteostation;

uint8_t parserMeteoStation_simple(uint8_t *receiveData) // вектор обработки прерывания модуля USART_C1 по завершению приёма данных
{
    errorMeteostation_t errorMeteostation_temp;
    dataMeteostation_t dataMeteostation_temp;
    uint8_t resultCompareCRC_temp = 0;

    uint8_t sizePayload = 0;

    if (receiveData[0] == METEOBLOCK_ADDRESS) // если принятый байт совпадает с адресом метеоблока М1
    {
        switch (receiveData[1]) // начало перечисления байтов функций
        {
        case 0x42: // команде старта измерения
            sizePayload = 3;

            resultCompareCRC_temp = func_compare_crc16(receiveData, PACKET_FULLCONSTANT_SIZE + sizePayload); //
            if (resultCompareCRC_temp == 0)                                                                  // если статус ошибки модуля USART равен "0" (нет ошибок)
            {
                errorMeteostation_temp.sensTemperature = receiveData[2];
                errorMeteostation_temp.sensTemperaturePressureHumidity = receiveData[3];
                errorMeteostation_temp.sensAnemometer = receiveData[4];

                errorMeteostation.sensTemperature = receiveData[2];
                errorMeteostation.sensTemperaturePressureHumidity = receiveData[3];
                errorMeteostation.sensAnemometer = receiveData[4];
            }
            else // иначе
            {
                // errorMeteostation_temp.sensTemperature = 0xFF;
                // errorMeteostation_temp.sensTemperaturePressureHumidity = 0xFF;
                // errorMeteostation_temp.sensAnemometer = 0xFF;
            }
            break; // выход из байта функции 0x42

            // если байт кода функции соответствует команде чтения метеоданных
            //-----------------------------------------------------------------начало функции 0x43
        case 0x43:

            sizePayload = 21;
            resultCompareCRC_temp = func_compare_crc16(receiveData, PACKET_FULLCONSTANT_SIZE + sizePayload); //
            if (resultCompareCRC_temp == 0)                                                                  // если статус ошибки модуля USART равен "0" (нет ошибок)
            {
                for (uint8_t i = 0; i < sizePayload; i++)
                {
                    memcpy(&dataMeteostation_temp, &receiveData[2], sizePayload);

                    memcpy(&dataMeteostation, &receiveData[2], sizePayload);
                }
            }
            else // иначе
            {
                // memset(&dataMeteostation_temp, 0xFF, sizePayload);
            }
            break; // выход из байта функции 0x43

                   // если байт кода функции соответствует команде чтения метеоданных-----------------------------------------------------------------начало функции 0x43
        default:
            break; // выход 

        } // конец перечисления байтов функций
    }
    else
    {
        resultCompareCRC_temp = 1;
    }
    
    if (resultCompareCRC_temp)
    {
        return 1;
    }
    else
        return 0;
}