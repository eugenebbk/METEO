#include "LogsSST26.h"

//+
void SST26clearFullBookLogs(uint8_t numbFlashMemory)
{
    sFLASH_EraseChip(numbFlashMemory);
}

// void readFullBookLogs(uint8_t numbFlashMemory)
// {
//     // sFLASH_EraseChip(numbFlashMemory);
// }

//+
log3_t SST26readIDLogsReturn(uint16_t currentFreeID_func, uint8_t numbFlashMemory)
{
    log3_t log3_temp = {0};
    sFLASH_ReadBuffer((uint8_t *)&log3_temp, (currentFreeID_func % sFLASH_NUMBER_LOGS_IN_FLASH) * sFLASH_SPI_PAGE_SIZE, sizeof(log3_t), numbFlashMemory);
    return log3_temp;
}

//+
void SST26readIDLogs(uint16_t currentFreeID_func, uint8_t numbFlashMemory)
{
    log3_t log3_temp = {0};
    sFLASH_ReadBuffer((uint8_t *)&log3_temp, (currentFreeID_func % sFLASH_NUMBER_LOGS_IN_FLASH) * sFLASH_SPI_PAGE_SIZE, sizeof(log3_t), numbFlashMemory);
    CDC_Transmit_FS((uint8_t *)&log3_temp, sizeof(log3_t));
}

//+
void SST26readLastLogs(uint16_t currentFreeID_func, uint8_t numbFlashMemory)
{
    log3_t log3_temp = {0};
    sFLASH_ReadBuffer((uint8_t *)&log3_temp, ((currentFreeID_func - 1) % sFLASH_NUMBER_LOGS_IN_FLASH) * sFLASH_SPI_PAGE_SIZE, sizeof(log3_t), numbFlashMemory);
    CDC_Transmit_FS((uint8_t *)&log3_temp, sizeof(log3_t));
}

//+
void SST26writeLog1(log3_t log, uint16_t currentFreeID_func, uint8_t numbFlashMemory)
{
    sFLASH_WritePage((uint8_t *)&log, sFLASH_SPI_PAGE_SIZE * currentFreeID_func, sizeof(log3_t), numbFlashMemory);
}

void SST26clearSectorLogs(uint8_t numbSector, uint8_t numbFlashMemory)
{
    sFLASH_EraseSector(sFLASH_SPI_SECTOR_SIZE * numbSector, numbFlashMemory);
}

#include "usbd_cdc_if.h"
extern uint16_t currentID_log_buf;
extern uint8_t blockInterruptUSB;

//void collectDataForLogs(log3_t *log, uint16_t currentFreeID_func, uint8_t numbFlashMemory)
void collectDataForLogs(log3_t *log)
{

    blockInterruptUSB = 1;
	
    log3_t log31 = {0};

    // log3_func->
    log31.ID = currentID_log_buf;
    log31.Date = 20240803;
    log31.Time = 181920;
    log31.Coordinate_oXoY = 444444;
    for (size_t i = 0; i < NUMB_TEMPBOARD_SENSOR; i++)
    {
        log31.Temperature[i] = 20 + i;
    }
    log31.VoltageBattery = 25;
    log31.OnPowerMETTMP_Board = 0;

    log31.ExtMetTMPRTR_Heater = 25;
    log31.ExtMetTMPRTR_Air = 25;
    log31.ExtMetPressureAtmosphere = 25;
    log31.ExtMetHumidity = 25;
    log31.ExtMetSpeedAir_oX = 25;
    log31.ExtMetSpeedAir_oY = 25;
    log31.ExtMetSpeedAir_Result = 25;
    log31.ExtMetDirectAir = 25;
    log31.ExtMetSpeedUltrasound_oX = 25;
    log31.ExtMetSpeedUltrasound_oY = 25;
    for (size_t i = 0; i < 47; i++)
    {
        log31.Reserved2[i] = i;
    }
    log31.EndOfLog = 0;

    blockInterruptUSB = 0;
    // currentID_log_buf++;
  SST26writeLog1(log31, currentID_log_buf, 1);
}