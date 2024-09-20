/*
 * Copyright © 2014 Kosma Moczek <kosma@cloudyourcar.com>
 * This program is free software. It comes without any warranty, to the extent
 * permitted by applicable law. You can redistribute it and/or modify it under
 * the terms of the Do What The Fuck You Want To Public License, Version 2, as
 * published by Sam Hocevar. See the COPYING file for more details.
 */

#include <stdio.h>
#include <string.h>

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

#define SENTENCE_SIZE 3

int main(void)
{

    uint8_t sentenceForCRC[SENTENCE_SIZE] = {0x33, 0x08, 0x00};
    // uint8_t sentenceForCRC[SENTENCE_SIZE] = {0x24, 0x43};
    uint16_t CRC_Answer = 0;

    CRC_Answer = func_calc_crc16(sentenceForCRC, SENTENCE_SIZE);

    return 0;
}
