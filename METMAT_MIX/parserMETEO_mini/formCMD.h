#ifndef FORMCMD_H_
#define FORMCMD_H_

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "crcANSI.h"

#define CMD_METEOSTATION_START 0x42
#define CMD_READ_DATA 0x43



#define CMD_METEOSTATION_START_SiZE 5
#define CMD_READ_DATA_SiZE 4

//form meteo cmd
uint8_t formMeteoRequestCMD_simple(uint8_t cmd, uint8_t *requestDataMeteo);



#endif /* FORMCMD_H_ */