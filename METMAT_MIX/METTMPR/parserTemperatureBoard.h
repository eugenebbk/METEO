#ifndef _PARSER_TEMPERATURE_BOARD_H_
#define _PARSER_TEMPERATURE_BOARD_H_

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#define PACKET_METTMPR_SIZE 12
#define PACKET_METTMPR_HEAD_SIZE 2
#define PACKET_METTMPR_HEAD_CONST1 0xFD
#define PACKET_METTMPR_HEAD_CONST2 0x55

#define NUMB_TEMPERATURE_SENSOR 5

typedef struct{
    int16_t temperatureMETTMPR[5];
}dataTemperatureMETTMPR_t;


uint8_t parserTemperatureBoard(uint8_t *receiveDataMETTMPR);

#endif /* _PARSER_TEMPERATURE_BOARD_H_ */
