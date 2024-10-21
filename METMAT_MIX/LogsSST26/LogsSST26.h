#ifndef _LOGSSST26_H_
#define _LOGSSST26_H_

#include "SST26.h"
#include "usbd_cdc_if.h"


#define NUMB_TEMPBOARD_SENSOR 5 

// 48byte - before reserve
// 64byte - after reserve
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

// // 256b
// typedef struct
// {
//   uint32_t ID;
//   uint32_t Date;
//   uint32_t Time;
//   uint32_t Coordinate_oXoY;
//   uint32_t Temperature[NUMB_TEMPBOARD_SENSOR];
//   uint32_t VoltageBattery;
//   uint32_t OnPowerMETTMP_Board;

//   int16_t ExtMetTMPRTR_Heater;
//   int16_t ExtMetTMPRTR_Air;
//   uint16_t ExtMetPressureAtmosphere;
//   uint16_t ExtMetHumidity;
//   int16_t ExtMetSpeedAir_oX;
//   int16_t ExtMetSpeedAir_oY;
//   uint16_t ExtMetSpeedAir_Result;
//   int16_t ExtMetDirectAir;
//   int16_t ExtMetSpeedUltrasound_oX;
//   int16_t ExtMetSpeedUltrasound_oY;

//   uint32_t Reserved2[47];
//   uint32_t EndOfLog;
// } log3_t;



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
  uint16_t Reserved1;
  uint32_t Reserved2[38];
  uint32_t EndOfLog;
} log3_t;

#define SIZE_LOG (sizeof(log3_t)) 
	


log3_t SST26readIDLogsUSB(uint16_t currentFreeID_func, uint8_t numbFlashMemory);
log3_t SST26readLastLogsUSB(uint16_t currentFreeID_func, uint8_t numbFlashMemory);
log3_t SST26readIDLogsReturn(uint16_t currentFreeID_func, uint8_t numbFlashMemory);

void SST26writeLog1(log3_t log, uint16_t currentFreeID_func, uint8_t numbFlashMemory);

void SST26clearFullBookLogs(uint8_t numbFlashMemory);
// void SST26clearSectorLogs(uint8_t numbSector, uint8_t numbFlashMemory);

void SST26clearSectorLogs_sector(uint8_t numbSector, uint8_t numbFlashMemory);
void SST26clearSectorLogs_address(uint8_t adress, uint8_t numbFlashMemory);
void SST26clearSectorLogs_ID(uint8_t ID, uint8_t numbFlashMemory);

log3_t SST26readLastLogs(uint16_t currentFreeID_func, uint8_t numbFlashMemory);
log3_t SST26readIDLogs(uint16_t ID_Log, uint8_t numbFlashMemory);

uint8_t SST26CheckFreeMem(uint16_t ID_Log, uint8_t numbFlashMemory);
uint8_t SST26CheckFreeMem_VarNumb(uint16_t ID_Log, uint8_t numbFlashMemory);
uint8_t writeLogAndCheck(log3_t log, uint16_t ID_Log, uint8_t numbFlashMemory);

void SST26readAllLogs(uint16_t lastFreeID_log, uint8_t numbFlashMemory);

#endif /* _LOGSSST26_H_ */
