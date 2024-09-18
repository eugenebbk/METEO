#include "formCMD.h"

// uint8_t formMeteoCMD(uint8_t cmd, uint8_t data, uint8_t sizeData){
uint8_t formMeteoRequestCMD_simple(uint8_t cmd, uint8_t *requestDataMeteo)
{
    if (cmd == CMD_METEOSTATION_START)
    {
        requestDataMeteo[0] = 0x24;
        requestDataMeteo[1] = cmd;
        requestDataMeteo[2] = 0;

        uint16_t crc16ANSI = func_calc_crc16(requestDataMeteo, 3);
        requestDataMeteo[3] = (uint8_t)((crc16ANSI >> 8) & 0xFF);
        requestDataMeteo[4] = (uint8_t)(crc16ANSI & 0xFF);
    }
    else if (cmd == CMD_READ_DATA)
    {
        requestDataMeteo[0] = 0x24;
        requestDataMeteo[1] = cmd;
        requestDataMeteo[2] = 0x41;
        requestDataMeteo[3] = 0x5B;
    }
    else
    {
        memset(requestDataMeteo, 0, 5);
        return 1;
    }
    return 0;
}