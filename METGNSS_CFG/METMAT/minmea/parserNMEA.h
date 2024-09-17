#ifndef PARSERNMEA_H
#define PARSERNMEA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

//128 for 2 nmea sentence
#define NAX_SIZE_NMEA_SENTENCE 64
#define CNT_NMEA_SENTENCE 2

void clearMassiveForNMEA(UART_HandleTypeDef* huart, uint8_t* buffer, uint8_t sizeBuffer);
uint8_t parserNMEA(uint8_t *sentenceNMEA);
uint8_t readData(const uint8_t *massiveSourceData_ptr);


#ifdef __cplusplus
}
#endif

#endif /* PARSERNMEA_H */
