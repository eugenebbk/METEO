#ifndef _STM32_SPI_FLASH_H_
#define _STM32_SPI_FLASH_H_

#include "main.h"



//--config pin extern
#define FLASH1_CS_Pin GPIO_PIN_4
#define FLASH1_CS_GPIO_Port GPIOA

#define FLASH2_CS_Pin GPIO_PIN_9
#define FLASH2_CS_GPIO_Port GPIOC

#define sFLASH1_CS_LOW() HAL_GPIO_WritePin(FLASH1_CS_GPIO_Port, FLASH1_CS_Pin, GPIO_PIN_RESET)
#define sFLASH1_CS_HIGH() HAL_GPIO_WritePin(FLASH1_CS_GPIO_Port, FLASH1_CS_Pin, GPIO_PIN_SET)

#define sFLASH2_CS_LOW() HAL_GPIO_WritePin(FLASH2_CS_GPIO_Port, FLASH2_CS_Pin, GPIO_PIN_RESET)
#define sFLASH2_CS_HIGH() HAL_GPIO_WritePin(FLASH2_CS_GPIO_Port, FLASH2_CS_Pin, GPIO_PIN_SET)

/*
 *  MX25 series command hex code definition
 */
/* ID comands */
#define    FLASH_CMD_RDID      0x9F    //RDID (Read Identification)
#define    FLASH_CMD_RES       0xAB    //RES (Read Electronic ID)
#define    FLASH_CMD_REMS      0x90    //REMS (Read Electronic & Device ID)

/* Register comands */
#define    FLASH_CMD_WRSR      0x01    //WRSR (Write Status Register)
#define    FLASH_CMD_RDSR      0x05    //RDSR (Read Status Register)
#define    FLASH_CMD_WRSCUR    0x2F    //WRSCUR (Write Security Register)
#define    FLASH_CMD_RDSCUR    0x2B    //RDSCUR (Read Security Register)
#define    FLASH_CMD_RDCR      0x15    //RDCR (Read Configuration Register)

/* READ comands */
#define    FLASH_CMD_READ        0x03    //READ (1 x I/O)
#define    FLASH_CMD_2READ       0xBB    //2READ (2 x I/O)
#define    FLASH_CMD_4READ       0xEB    //4READ (4 x I/O)
#define    FLASH_CMD_FASTREAD    0x0B    //FAST READ (Fast read data)
#define    FLASH_CMD_DREAD       0x3B    //DREAD (1In/2 Out fast read)
#define    FLASH_CMD_QREAD       0x6B    //QREAD (1In/4 Out fast read)
#define    FLASH_CMD_RDSFDP      0x5A    //RDSFDP (Read SFDP)

/* Program comands */
#define    FLASH_CMD_WREN     0x06    //WREN (Write Enable)
#define    FLASH_CMD_WRDI     0x04    //WRDI (Write Disable)
#define    FLASH_CMD_PP       0x02    //PP (page program)
#define    FLASH_CMD_4PP      0x38    //4PP (Quad page program)

/* Erase comands */
#define    FLASH_CMD_SE       0x20    //SE (Sector Erase)
#define    FLASH_CMD_BE32K    0x52    //BE32K (Block Erase 32kb)
#define    FLASH_CMD_BE       0xD8    //BE (Block Erase)
#define    FLASH_CMD_CE       0x60    //CE (Chip Erase) hex code: 60 or C7

/* Mode setting comands */
#define    FLASH_CMD_DP       0xB9    //DP (Deep Power Down)
#define    FLASH_CMD_RDP      0xAB    //RDP (Release form Deep Power Down)
#define    FLASH_CMD_ENSO     0xB1    //ENSO (Enter Secured OTP)
#define    FLASH_CMD_EXSO     0xC1    //EXSO  (Exit Secured OTP)
#ifdef SBL_CMD_0x77
#define    FLASH_CMD_SBL      0x77    //SBL (Set Burst Length) new: 0x77
#else
#define    FLASH_CMD_SBL      0xC0    //SBL (Set Burst Length) Old: 0xC0
#endif

/* Reset comands */
#define    FLASH_CMD_RSTEN     0x66    //RSTEN (Reset Enable)
#define    FLASH_CMD_RST       0x99    //RST (Reset Memory)

/* Security comands */
#ifdef LCR_CMD_0xDD_0xD5
#else
#endif

/* Suspend/Resume comands */
#define    FLASH_CMD_PGM_ERS_S    0xB0    //PGM/ERS Suspend (Suspends Program/Erase)
#define    FLASH_CMD_PGM_ERS_R    0x30    //PGM/ERS Erase (Resumes Program/Erase)
#define    FLASH_CMD_NOP          0x00    //NOP (No Operation)


#define    FLASH_CMD_RBPR          0x72    // Read Block Protection Register

#define    FLASH_CMD_ULBPR      0x98    //

/*
 *  Flash control register mask define
 */
/* status register */
#define    FLASH_WIP_MASK         0x01
#define    FLASH_LDSO_MASK        0x02
#define    FLASH_QE_MASK          0x40
#define    FLASH_BUSY_MASK        0x80
/* security register */
#define    FLASH_OTPLOCK_MASK     0x03
#define    FLASH_4BYTE_MASK       0x04
#define    FLASH_WPSEL_MASK       0x80
/* configuration reigster */
#define    FLASH_DC_MASK          0x80
#define    FLASH_DC_2BIT_MASK     0xC0
/* other */
#define    BLOCK_PROTECT_MASK     0xff
#define    BLOCK_LOCK_MASK        0x01



/*
 * Flash ID, Timing Information Define
 * (The following information could get from device specification)
 */
#define    sFLASH_MX25L6433F_ID          	0xc22017
#define    sFLASH_MX25L6433F_ElectronicID	0x16
#define    sFLASH_MX25L6433F_RESID0			0xc216
#define    sFLASH_MX25L6433F_RESID1			0x16c2



/*
 * Flash memory organization
 */
// #define sFLASH_SPI_FLASH_SIZE	0x200000    /* 8 Mbytes */
#define sFLASH_SPI_FLASH_SIZE	0x400000    /* 4 Mbytes */
#define sFLASH_SPI_SECTOR_SIZE	0x1000      /* 4K Sector size */
#define sFLASH_SPI_PAGE_SIZE	0x100		/* 256 Byte Page size */
#define sFLASH_NUMBER_LOGS_IN_FLASH	(sFLASH_SPI_FLASH_SIZE/sFLASH_SPI_PAGE_SIZE)
#define sFLASH_NUMBER_LOGS_IN_SECTOR	(sFLASH_SPI_SECTOR_SIZE/sFLASH_SPI_PAGE_SIZE)
#define sFLASH_NUMBER_SECTORS	(sFLASH_SPI_FLASH_SIZE/sFLASH_SPI_SECTOR_SIZE)
		
// const size_t numbOfLogs = sFLASH_SPI_FLASH_SIZE/sFLASH_SPI_PAGE_SIZE;

/* High layer functions */
//void sFLASH_DeInit(void);
//void sFLASH_Init(void);
void sFLASH_EraseSector(uint32_t SectorAddr, uint8_t numbFlashMemor);
void sFLASH_EraseChip(uint8_t numbFlashMemor);
void sFLASH_WritePage(uint8_t* pBuffer, uint32_t WriteAddr, uint32_t NumByteToWrite, uint8_t numbFlashMemor);
void sFLASH_WriteBuffer(uint8_t* pBuffer, uint32_t WriteAddr, uint32_t NumByteToWrite, uint8_t numbFlashMemor);
void sFLASH_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead, uint8_t numbFlashMemor);
uint32_t sFLASH_ReadID(uint8_t numbFlashMemor);
void sFLASH_InitSST26(uint8_t numbFlashMemory);
void sFLASH_WaitForWriteEnd(uint8_t numbFlashMemory);
void sFLASH_TxSimpleCmd(uint8_t numbFlashMemory, uint8_t cmd);

void sFLASH_ReadBlockProtection(uint8_t* BlockRegData);
void sFLASH_GlobalBlockProtectionUnlock(uint8_t numbFlashMemory);

uint8_t sFLASH_ReadSimpleCommand(uint8_t command, uint8_t numbFlashMemory);
void sFLASH_ReadCommand(uint8_t command, uint8_t* massiveAnswer, uint8_t sizeAnswer, uint8_t numbFlashMemory);


//----------

uint32_t sFLASH_SearchLastFreePageAdress(uint16_t *ID_log, uint8_t numbFlashMemory);
// uint32_t sFLASH_SearchLastFreePageAdress (uint8_t numbFlashMemory);

#endif /* _STM32_SPI_FLASH_H_ */
