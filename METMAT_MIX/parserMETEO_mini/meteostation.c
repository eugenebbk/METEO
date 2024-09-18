#include "meteostation.h"


void initMeteoStation(void)
{
    uint8_t dataMeteoRequest[5] = {0};
    formMeteoRequestCMD_simple(CMD_METEOSTATION_START, dataMeteoRequest);
}