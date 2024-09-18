#ifndef CRCANSI_H_
#define CRCANSI_H_

#include <stdio.h>
#include <string.h>
#include <stdint.h>

// функция вычисления crc16 (первый аргумент-указатель на элемент массива Mas_crc16, второй - задаёт число элементов массива)
uint16_t func_calc_crc16(const uint8_t *Mas_crc16, uint8_t sizeDataWithoutCRC);
uint8_t func_compare_crc16(uint8_t *data, uint8_t sizePacketWithCRC);


#endif /* CRCANSI_H_ */