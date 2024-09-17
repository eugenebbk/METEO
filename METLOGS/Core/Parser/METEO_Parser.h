
#ifndef _METEO_PARSER_H_
#define _METEO_PARSER_H_

#define MAX_DATA_REQUEST 1
#define MAX_DATA_ANSWER 21

typedef struct
{
    uint8_t Date;
    uint8_t CRC16[2];
} payloadCRC_t;

typedef struct
{
    uint8_t hat;
    uint8_t func;
    // payloadCRC_t payloadCRC;
    uint8_t payloadCRC[2 + MAX_DATA_ANSWER];
} messageMETEO_t;

/*
 *  METEO commands
 */

/* METEO comands */
// #define METEO_CMD_START_MEASURE      0x41 // 
#define METEO_CMD_START_MEASURE      0x42 // 
#define METEO_CMD_READ_DATA      0x43 // 

#endif /* _METEO_PARSER_H_ */
