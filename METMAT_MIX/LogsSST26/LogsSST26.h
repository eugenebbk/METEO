#ifndef _LOGSSST26_H_
#define _LOGSSST26_H_

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "SST26.h"
#include "usbd_cdc_if.h"

#define NUMB_TEMPBOARD_SENSOR 5 

// // 48byte - before reserve
// // 64byte - after reserve
// typedef struct
// {
//   uint32_t ID;
//   uint32_t Date;
//   uint32_t Time;
//   uint32_t Coordinate_oXoY;
//   // int32_t Temperature[5];
//   uint32_t Temperature[NUMB_TEMPBOARD_SENSOR];
//   uint32_t VoltageBattery;
//   uint32_t OnPowerMETTMP_Board;
//   uint32_t Reserved[4];
//   uint32_t EndOfLog;
// } log1_t;

//+8 meteoblock
// 1 akk

//struct minmea_date {
//    int day;
//    int month;
//    int year;
//};
//struct minmea_time {
//    int hours;
//    int minutes;
//    int seconds;
//    int microseconds;
//};
//struct minmea_float {
//    int_least32_t value;
//    int_least32_t scale;
//};

// 256b
typedef struct
{
  uint32_t ID;
  uint32_t Date[3];
  uint32_t Time[4];
  uint32_t Coordinate_oX[2];
  uint32_t Coordinate_oY[2];
  int32_t Temperature[NUMB_TEMPBOARD_SENSOR];
  uint32_t VoltageBattery;
  uint32_t OnPowerMETTMP_Board;
//76
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
//96
  uint16_t Reserved1[2];
  uint32_t Reserved2[38];
  uint32_t EndOfLog;
  //158
} log3_t;
//+6b

#define SIZE_LOG (sizeof(log3_t)) 

void SST26readIDLogs(uint16_t currentFreeID_func, uint8_t numbFlashMemory);
void SST26readLastLogs(uint16_t currentFreeID_func, uint8_t numbFlashMemory);
log3_t SST26readIDLogsReturn(uint16_t currentFreeID_func, uint8_t numbFlashMemory);

void SST26writeLog1(log3_t log, uint16_t currentFreeID_func, uint8_t numbFlashMemory);

void SST26clearFullBookLogs(uint8_t numbFlashMemory);
void SST26clearSectorLogs(uint8_t numbSector, uint8_t numbFlashMemory);

#endif /* _LOGSSST26_H_ */
