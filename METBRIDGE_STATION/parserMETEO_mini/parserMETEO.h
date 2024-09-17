
#ifndef PARSERMETEO_H_
#define PARSERMETEO_H_

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#define METEOBLOCK_ADDRESS 0x24
#define PACKET_HEADER_SIZE 2
#define PACKET_CRC_SIZE 2
#define PACKET_FULLCONSTANT_SIZE (PACKET_HEADER_SIZE+PACKET_CRC_SIZE)

uint8_t parserMeteoStation_simple(uint8_t *receiveData);

#endif /* PARSERMETEO_H_ */