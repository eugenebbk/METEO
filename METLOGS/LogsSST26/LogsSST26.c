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
log3_t SST26readIDLogsUSB(uint16_t currentFreeID_func, uint8_t numbFlashMemory)
{
    log3_t log3_temp = {0};
    sFLASH_ReadBuffer((uint8_t *)&log3_temp, (currentFreeID_func % sFLASH_NUMBER_LOGS_IN_FLASH) * sFLASH_SPI_PAGE_SIZE, sizeof(log3_t), numbFlashMemory);
    CDC_Transmit_FS((uint8_t *)&log3_temp, sizeof(log3_t));
		return log3_temp;
}

//+
log3_t SST26readLastLogsUSB(uint16_t currentFreeID_func, uint8_t numbFlashMemory)
{
    log3_t log3_temp = {0};
    sFLASH_ReadBuffer((uint8_t *)&log3_temp, ((currentFreeID_func - 1) % sFLASH_NUMBER_LOGS_IN_FLASH) * sFLASH_SPI_PAGE_SIZE, sizeof(log3_t), numbFlashMemory);
    CDC_Transmit_FS((uint8_t *)&log3_temp, sizeof(log3_t));
		return log3_temp;
}

//+
void SST26writeLog1(log3_t log, uint16_t currentFreeID_func, uint8_t numbFlashMemory)
{
    sFLASH_WritePage((uint8_t *)&log, sFLASH_SPI_PAGE_SIZE * currentFreeID_func, sizeof(log3_t), numbFlashMemory);
}

void SST26clearSectorLogs_sector(uint8_t numbSector, uint8_t numbFlashMemory)
{
    sFLASH_EraseSector(sFLASH_SPI_SECTOR_SIZE * numbSector, numbFlashMemory);
}

void SST26clearSectorLogs_address(uint8_t adress, uint8_t numbFlashMemory)
{
    sFLASH_EraseSector(adress, numbFlashMemory);
}

void SST26clearSectorLogs_ID(uint8_t ID, uint8_t numbFlashMemory)
{
    sFLASH_EraseSector(ID * sFLASH_SPI_PAGE_SIZE, numbFlashMemory);
}

//
log3_t SST26readIDLogs(uint16_t ID_Log, uint8_t numbFlashMemory)
{
    log3_t log3_temp = {0};
    sFLASH_ReadBuffer((uint8_t *)&log3_temp, (ID_Log % sFLASH_NUMBER_LOGS_IN_FLASH) * sFLASH_SPI_PAGE_SIZE, sizeof(log3_t), numbFlashMemory);
    // CDC_Transmit_FS((uint8_t *)&log3_temp, sizeof(log3_t));
    return log3_temp;
}

// currentFreeID_func - последний свободный ИД
log3_t SST26readLastLogs(uint16_t currentFreeID_func, uint8_t numbFlashMemory)
{
    log3_t log3_temp = {0};
    sFLASH_ReadBuffer((uint8_t *)&log3_temp, ((currentFreeID_func - 1) % sFLASH_NUMBER_LOGS_IN_FLASH) * sFLASH_SPI_PAGE_SIZE, sizeof(log3_t), numbFlashMemory);
    // CDC_Transmit_FS((uint8_t *)&log3_temp, sizeof(log3_t));
    return log3_temp;
}

#define BYTES_CHECK_FREEMEM 4
uint8_t SST26CheckFreeMem(uint16_t ID_Log, uint8_t numbFlashMemory)
{
    uint8_t flagResult = 1;
    // log3_t log3_temp = {0};
    uint8_t log[sizeof(log3_t)] = {0};
    sFLASH_ReadBuffer(log, ((ID_Log) % sFLASH_NUMBER_LOGS_IN_FLASH) * sFLASH_SPI_PAGE_SIZE, sizeof(log3_t), numbFlashMemory);

    // for (uint8_t cnt2 = 0; cnt2 < sizeof(log3_t); cnt2++) // full check
    for (uint8_t cnt2 = 0; cnt2 < BYTES_CHECK_FREEMEM; cnt2++) // part check
    {
        if (log[cnt2] != 0xFF)
        {
            flagResult = 0;
            return flagResult;
        }
    }
    return flagResult;
}

// return numb don`t free log
// 0xFF-true
//  smb val - false numb ID
#define SIZE_FREE_LOG_CHECK 3
uint8_t SST26CheckFreeMem_VarNumb(uint16_t ID_Log, uint8_t numbFlashMemory)
{
//    log3_t log3_temp = {0};
    uint8_t flagMemCmp = 255;
//    uint8_t *log3_temp_ptr = (uint8_t *)&log3_temp;
    for (uint8_t i = 0; i < SIZE_FREE_LOG_CHECK; i++)
    {
        // sFLASH_ReadBuffer(log3_temp_ptr, ((ID_Log + i) % sFLASH_NUMBER_LOGS_IN_FLASH) * sFLASH_SPI_PAGE_SIZE, sizeof(log3_t), numbFlashMemory);
        // for (uint8_t cnt2 = 0; cnt2 < sizeof(log3_t); cnt2++)
        // {
        //     if (*(log3_temp_ptr + cnt2) != 0xFF)
        //     {
        //         flagMemCmp = i;
        //         return flagMemCmp;
        //     }
        // }

        SST26CheckFreeMem(ID_Log + i, numbFlashMemory);
    }
    return flagMemCmp;
}

// after writing need check free address
uint8_t writeLogAndCheck(log3_t log, uint16_t ID_Log, uint8_t numbFlashMemory)
{
    SST26writeLog1(log, ID_Log, numbFlashMemory);
    // sFLASH_SearchLastFreePageAdress(ID_Log, numbFlashMemory);
    uint8_t CheckFreeMem = SST26CheckFreeMem_VarNumb(ID_Log + 1, numbFlashMemory);
    if (CheckFreeMem < 0xFF)
    {
        SST26clearSectorLogs_ID(ID_Log + 1 + CheckFreeMem, numbFlashMemory);
        if (!(SST26CheckFreeMem(ID_Log + 1 + CheckFreeMem, numbFlashMemory)))
        {
            return 0;
        }
        else
        {
            return 1;
        }
    }
    else
    {
        return 1;
    }
}

// /r /n under question
void SST26readAllLogs(uint16_t lastFreeID_log, uint8_t numbFlashMemory)
{
    // last log
    uint32_t currentFreeAddrLog = sFLASH_SearchLastFreePageAdress(&lastFreeID_log, numbFlashMemory);
    uint16_t ID_log = (currentFreeAddrLog - sizeof(log3_t)) / sizeof(log3_t);

    log3_t logData = SST26readIDLogsUSB(ID_log, numbFlashMemory);
    uint32_t *logData_ptr = (uint32_t *)&logData;

//    while (*logData_ptr[0] != 0xFFFFFFFF)
    while (*logData_ptr != 0xFFFFFFFF)
    {
        logData = SST26readIDLogsUSB(--ID_log, numbFlashMemory);
    }
}
