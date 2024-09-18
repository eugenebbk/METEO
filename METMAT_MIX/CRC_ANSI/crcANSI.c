#include "crcANSI.h"

// функция вычисления crc16 (первый аргумент-указатель на элемент массива Mas_crc16, второй - задаёт число элементов массива без CRC)
uint16_t func_calc_crc16(const uint8_t *Mas_crc16, uint8_t sizeDataWithoutCRC)
{
    uint16_t crc16 = 0xFFFF; // объявление переменной для хранения результата вычисления crc16 с заданием начального значения

    while (sizeDataWithoutCRC--) // цикл, пока число элементов массива не равно 0
    {
        crc16 ^= *Mas_crc16++;
        for (uint8_t i = 0; i < 8; i++)
        {
            if (crc16 & 0x01) // если младший бит crc16 равен единице
            {
                crc16 = (crc16 >> 1) ^ 0xA001; // вычисление текущего значения crc16
            }
            else // иначе
            {
                crc16 = crc16 >> 1; // логический сдвиг вправо текущего значения crc16
            }
        }
    }
    return crc16; // возврат функцией значения crc16
}

// —————————————
// функция определения правильности принятого crc16
uint8_t func_compare_crc16(uint8_t *data, uint8_t sizePacketWithCRC)
{
    const uint8_t sizeCRC = 2;
    uint16_t crc16_USART; // локальная переменная для хранения принятого значения crc16_USART
    uint16_t crc16_calc;  // локальная переменная для хранения расчётного значения crc16_calc

    // crc16_USART = ((uint16_t)(data[sizePacketWithCRC - sizeCRC]) << 8) + (uint16_t)(data[sizePacketWithCRC - (sizeCRC - 1)]); // восстановление crc16_USART
    crc16_USART = (data[sizePacketWithCRC - sizeCRC]) << 8;
    crc16_USART += (uint16_t)(data[sizePacketWithCRC - (sizeCRC - 1)]);
    
    // вычисления crc16_calc
    crc16_calc = func_calc_crc16(data, sizePacketWithCRC - sizeCRC);
    
    if (crc16_calc != crc16_USART) // если принятый crc16_USART не равен расчётному значению crc16_calc
    {
        return 1; // запись статуса ошибки модуля USART
    }
    return 0; // обнуление переменной для хранения счётчика принятых/переданных байтов
}
// —————————————