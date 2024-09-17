#ifndef FORMCMD_H_
#define FORMCMD_H_

#include <stdio.h>
#include <string.h>
#include <stdint.h>

// // функция вычисления crc16 (первый аргумент-указатель на элемент массива Mas_crc16, второй - задаёт число элементов массива)
// uint16_t func_calc_crc16(const uint8_t *Mas_crc16, uint8_t size);


//form meteo cmd
uint8_t formMeteoCMD_simple(uint8_t cmd, uint8_t *requestDataMeteo);

#endif /* FORMCMD_H_ */