#include "parserTemperatureBoard.h"

extern dataTemperatureMETTMPR_t dataTemperatureMETTMPR;
uint8_t parserTemperatureBoard(uint8_t *receiveDataMETTMPR)
{
    // dataTemperatureMETTMPR_t dataTemperatureMETTMPR = {0};
    if ((receiveDataMETTMPR[0] == PACKET_METTMPR_HEAD_CONST1) && (receiveDataMETTMPR[1] == PACKET_METTMPR_HEAD_CONST2)) // если принятый байт совпадает с адресом метеоблока М1
    {
        memcpy(dataTemperatureMETTMPR.temperatureMETTMPR, &receiveDataMETTMPR[PACKET_METTMPR_HEAD_SIZE], PACKET_METTMPR_SIZE-PACKET_METTMPR_HEAD_SIZE);
        // for (uint8_t cnt_temp1 = 0; cnt_temp1 < NUMB_TEMPERATURE_SENSOR; cnt_temp1++)
        // {
        //     // memcpy((uint8_t*)&dataTemperatureMETTMPR.temperatureMETTMPR[cnt_temp1], &receiveDataMETTMPR[cnt_temp1 * 2 + 2], 2);
        //     memcpy(dataTemperatureMETTMPR.temperatureMETTMPR, &receiveDataMETTMPR[cnt_temp1 * 2 + 2], 10);
        //     // dataTemperatureMETTMPR[cnt_temp1]
        // }
        return 0;
    }
    return 1;
}