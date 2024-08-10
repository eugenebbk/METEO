#ifndef _LOGSSST26_H_
#define _LOGSSST26_H_

#include "SST26.h"
#include "usbd_cdc_if.h"


#define NUMB_TEMPBOARD_SENSOR 5 

// 48byte - before reserve
// 64byte - after reserve
typedef struct
{
  uint32_t ID;
  uint32_t Date;
  uint32_t Time;
  uint32_t Coordinate_oXoY;
  // int32_t Temperature[5];
  uint32_t Temperature[NUMB_TEMPBOARD_SENSOR];
  uint32_t VoltageBattery;
  uint32_t OnPowerMETTMP_Board;
  uint32_t Reserved[4];
  uint32_t EndOfLog;
} log1_t;

//+8 meteoblock
// 1 akk

// 256b
typedef struct
{
  uint32_t ID;
  uint32_t Date;
  uint32_t Time;
  uint32_t Coordinate_oXoY;
  uint32_t Temperature[NUMB_TEMPBOARD_SENSOR];
  uint32_t VoltageBattery;
  uint32_t OnPowerMETTMP_Board;

  int16_t ExtMetTMPRTR_Heater;
  int16_t ExtMetTMPRTR_Air;
  uint16_t ExtMetPressureAtmosphere;
  uint16_t ExtMetHumidity;
  int16_t ExtMetSpeedAir_oX;
  int16_t ExtMetSpeedAir_oY;
  uint16_t ExtMetSpeedAir_Result;
  int16_t ExtMetDirectAir;
  int16_t ExtMetSpeedUltrasound_oX;
  int16_t ExtMetSpeedUltrasound_oY;

  uint32_t Reserved2[47];
  uint32_t EndOfLog;
} log3_t;

void SST26readIDLogs(uint16_t currentFreeID_func, uint8_t numbFlashMemory);
void SST26readLastLogs(uint16_t currentFreeID_func, uint8_t numbFlashMemory);
log3_t SST26readIDLogsReturn(uint16_t currentFreeID_func, uint8_t numbFlashMemory);

void SST26writeLog1(log3_t log, uint16_t currentFreeID_func, uint8_t numbFlashMemory);

void SST26clearFullBookLogs(uint8_t numbFlashMemory);
void SST26clearSectorLogs(uint8_t numbSector, uint8_t numbFlashMemory);

#endif /* _LOGSSST26_H_ */
