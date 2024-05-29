#include "spi.h"
#include "SST26.h"

// pa4 pb12

extern SPI_HandleTypeDef hspi1;

#define sFLASH1_CS_LOW() HAL_GPIO_WritePin(FLASH1_CS_GPIO_Port, FLASH1_CS_Pin, GPIO_PIN_RESET)
#define sFLASH1_CS_HIGH() HAL_GPIO_WritePin(FLASH1_CS_GPIO_Port, FLASH1_CS_Pin, GPIO_PIN_SET)

#define sFLASH2_CS_LOW() HAL_GPIO_WritePin(FLASH2_CS_GPIO_Port, FLASH2_CS_Pin, GPIO_PIN_RESET)
#define sFLASH2_CS_HIGH() HAL_GPIO_WritePin(FLASH2_CS_GPIO_Port, FLASH2_CS_Pin, GPIO_PIN_SET)

uint8_t sFLASH_ReadByte(void);
uint8_t sFLASH_SendByte(uint8_t byte);
uint16_t sFLASH_SendHalfWord(uint16_t HalfWord);
void sFLASH_WaitForWriteEnd(uint8_t numbFlashMemory);
void sFLASH_WriteEnable(uint8_t numbFlashMemory);


const size_t numbOfLogs = sFLASH_SPI_FLASH_SIZE/sFLASH_SPI_PAGE_SIZE;
// sdelat dlya neskolkih flash

/**
 * @brief  Erases the specified FLASH sector.
 * @param  SectorAddr: address of the sector to erase.
 * @retval None
 */
void sFLASH_EraseSector(uint32_t SectorAddr, uint8_t numbFlashMemory)
{
	uint8_t txData[4];

	txData[0] = FLASH_CMD_SE;
	txData[1] = (SectorAddr & 0xFF0000) >> 16; /* high nibble address byte to write to */
	txData[2] = (SectorAddr & 0xFF00) >> 8;	   /* medium nibble address byte to write to */
	txData[3] = SectorAddr & 0xFF;			   /* low nibble address byte to write to */

	/* Send write enable instruction */
	sFLASH_WriteEnable(numbFlashMemory);

	/* Select the FLASH: Chip Select low */
	// sFLASH_CS_LOW();
	if (numbFlashMemory == 1)
	{
		sFLASH1_CS_LOW();
	}
	else if (numbFlashMemory == 2)
	{
		sFLASH2_CS_LOW();
	}
	else
	{
		sFLASH1_CS_LOW();
	}
	/* Send Sector Erase instruction */
	HAL_SPI_Transmit(&hspi1, txData, sizeof(txData), HAL_MAX_DELAY);

	/* Deselect the FLASH: Chip Select high */
	// sFLASH_CS_HIGH();
	if (numbFlashMemory == 1)
	{
		sFLASH1_CS_HIGH();
	}
	else if (numbFlashMemory == 2)
	{
		sFLASH2_CS_HIGH();
	}
	else
	{
		sFLASH1_CS_HIGH();
	}

	/*!< Wait the end of Flash writing */
	sFLASH_WaitForWriteEnd(numbFlashMemory);
}

/**
 * @brief  Erases the entire FLASH.
 * @param  None
 * @retval None
 */
void sFLASH_EraseChip(uint8_t numbFlashMemory)
{
	uint8_t txData[1];

	txData[0] = FLASH_CMD_CE;

	/* Send write enable instruction */
	sFLASH_WriteEnable(numbFlashMemory);

	/* Select the FLASH: Chip Select low */
	// sFLASH_CS_LOW();
	if (numbFlashMemory == 1)
	{
		sFLASH1_CS_LOW();
	}
	else if (numbFlashMemory == 2)
	{
		sFLASH2_CS_LOW();
	}
	else
	{
		sFLASH1_CS_LOW();
	}
	/* Send Bulk Erase instruction  */
	HAL_SPI_Transmit(&hspi1, txData, sizeof(txData), HAL_MAX_DELAY);

	/* Deselect the FLASH: Chip Select high */
	// sFLASH_CS_HIGH();
	if (numbFlashMemory == 1)
	{
		sFLASH1_CS_HIGH();
	}
	else if (numbFlashMemory == 2)
	{
		sFLASH2_CS_HIGH();
	}
	else
	{
		sFLASH1_CS_HIGH();
	}
	/* Wait the end of Flash writing */
	sFLASH_WaitForWriteEnd(numbFlashMemory);
}

/**
 * @brief  Writes more than one byte to the FLASH with a single WRITE cycle
 *         (Page WRITE sequence).
 * @note   The number of byte can't exceed the FLASH page size.
 * @param  pBuffer: pointer to the buffer containing the data to be written
 *         to the FLASH.
 * @param  WriteAddr: FLASH's internal address to write to.
 * @param  NumByteToWrite: number of bytes to write to the FLASH, must be equal
 *         or less than "sFLASH_PAGESIZE" value.
 * @retval None
 */
void sFLASH_WritePage(uint8_t *pBuffer, uint32_t WriteAddr, uint32_t NumByteToWrite, uint8_t numbFlashMemory)
{
	uint8_t txData[4];

	txData[0] = FLASH_CMD_PP;
	txData[1] = (WriteAddr & 0xFF0000) >> 16; /* high nibble address byte to write to */
	txData[2] = (WriteAddr & 0xFF00) >> 8;	  /* medium nibble address byte to write to */
	txData[3] = WriteAddr & 0xFF;			  /* low nibble address byte to write to */

	/* Enable the write access to the FLASH */
	sFLASH_WriteEnable(numbFlashMemory);

	/* Select the FLASH: Chip Select low */
	// sFLASH_CS_LOW();

	if (numbFlashMemory == 1)
	{
		sFLASH1_CS_LOW();
	}
	else if (numbFlashMemory == 2)
	{
		sFLASH2_CS_LOW();
	}
	else
	{
		sFLASH1_CS_LOW();
	}

	/* Send "Write to Memory " instruction */
	HAL_SPI_Transmit(&hspi1, txData, sizeof(txData), HAL_MAX_DELAY);

	/* Send data */
	HAL_SPI_Transmit(&hspi1, pBuffer, NumByteToWrite, HAL_MAX_DELAY);

	/* Deselect the FLASH: Chip Select high */
	// sFLASH_CS_HIGH();
	if (numbFlashMemory == 1)
	{
		sFLASH1_CS_HIGH();
	}
	else if (numbFlashMemory == 2)
	{
		sFLASH2_CS_HIGH();
	}
	else
	{
		sFLASH1_CS_HIGH();
	}
	/* Wait the end of Flash writing */
	sFLASH_WaitForWriteEnd(numbFlashMemory);
}

/**
 * @brief  Reads a block of data from the FLASH.
 * @param  pBuffer: pointer to the buffer that receives the data read from the FLASH.
 * @param  ReadAddr: FLASH's internal address to read from.
 * @param  NumByteToRead: number of bytes to read from the FLASH.
 * @retval None
 */
void sFLASH_ReadBuffer(uint8_t *pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead, uint8_t numbFlashMemory)
{
	uint8_t txData[4];

	txData[0] = FLASH_CMD_READ;
	txData[1] = (ReadAddr & 0xFF0000) >> 16; /* high nibble address byte to write to */
	txData[2] = (ReadAddr & 0xFF00) >> 8;	 /* medium nibble address byte to write to */
	txData[3] = ReadAddr & 0xFF;			 /* low nibble address byte to write to */

	/* Select the FLASH: Chip Select low */
	//	sFLASH_CS_LOW();

	if (numbFlashMemory == 1)
	{
		sFLASH1_CS_LOW();
	}
	else if (numbFlashMemory == 2)
	{
		sFLASH2_CS_LOW();
	}
	else
	{
		sFLASH1_CS_LOW();
	}

	/* Send "Read from Memory " instruction */
	HAL_SPI_Transmit(&hspi1, txData, sizeof(txData), HAL_MAX_DELAY);

	/* Read data from the FLASH */
	HAL_SPI_Receive(&hspi1, pBuffer, NumByteToRead, HAL_MAX_DELAY);

	/* Deselect the FLASH: Chip Select high */
	// sFLASH_CS_HIGH();
	if (numbFlashMemory == 1)
	{
		sFLASH1_CS_HIGH();
	}
	else if (numbFlashMemory == 2)
	{
		sFLASH2_CS_HIGH();
	}
	else
	{
		sFLASH1_CS_HIGH();
	}
}

/**
 * @brief  Writes block of data to the FLASH. In this function, the number of
 *         WRITE cycles are reduced, using Page WRITE sequence.
 * @param  pBuffer: pointer to the buffer containing the data to be written
 *         to the FLASH.
 * @param  WriteAddr: FLASH's internal address to write to.
 * @param  NumByteToWrite: number of bytes to write to the FLASH.
 * @retval None
 */
void sFLASH_WriteBuffer(uint8_t *pBuffer, uint32_t WriteAddr, uint32_t NumByteToWrite, uint8_t numbFlashMemory)
{
	uint32_t NumOfPage = 0;
	uint32_t NumOfSingle = 0;
	uint32_t Addr = 0;
	uint32_t count = 0;
	uint32_t temp = 0;

	Addr = WriteAddr % sFLASH_SPI_PAGE_SIZE;
	count = sFLASH_SPI_PAGE_SIZE - Addr;
	NumOfPage = NumByteToWrite / sFLASH_SPI_PAGE_SIZE;
	NumOfSingle = NumByteToWrite % sFLASH_SPI_PAGE_SIZE;

	if (Addr == 0) /* WriteAddr is sFLASH_PAGESIZE aligned  */
	{
		if (NumOfPage == 0) /* NumByteToWrite < sFLASH_PAGESIZE */
		{
			sFLASH_WritePage(pBuffer, WriteAddr, NumByteToWrite, numbFlashMemory);
		}
		else /* NumByteToWrite > sFLASH_PAGESIZE */
		{
			while (NumOfPage--)
			{
				sFLASH_WritePage(pBuffer, WriteAddr, sFLASH_SPI_PAGE_SIZE, numbFlashMemory);
				WriteAddr += sFLASH_SPI_PAGE_SIZE;
				pBuffer += sFLASH_SPI_PAGE_SIZE;
			}

			sFLASH_WritePage(pBuffer, WriteAddr, NumOfSingle, numbFlashMemory);
		}
	}
	else /* WriteAddr is not sFLASH_PAGESIZE aligned  */
	{
		if (NumOfPage == 0) /* NumByteToWrite < sFLASH_PAGESIZE */
		{
			if (NumOfSingle > count) /* (NumByteToWrite + WriteAddr) > sFLASH_PAGESIZE */
			{
				temp = NumOfSingle - count;

				sFLASH_WritePage(pBuffer, WriteAddr, count, numbFlashMemory);
				WriteAddr += count;
				pBuffer += count;

				sFLASH_WritePage(pBuffer, WriteAddr, temp, numbFlashMemory);
			}
			else
			{
				sFLASH_WritePage(pBuffer, WriteAddr, NumByteToWrite, numbFlashMemory);
			}
		}
		else /* NumByteToWrite > sFLASH_PAGESIZE */
		{
			NumByteToWrite -= count;
			NumOfPage = NumByteToWrite / sFLASH_SPI_PAGE_SIZE;
			NumOfSingle = NumByteToWrite % sFLASH_SPI_PAGE_SIZE;

			sFLASH_WritePage(pBuffer, WriteAddr, count, numbFlashMemory);
			WriteAddr += count;
			pBuffer += count;

			while (NumOfPage--)
			{
				sFLASH_WritePage(pBuffer, WriteAddr, sFLASH_SPI_PAGE_SIZE, numbFlashMemory);
				WriteAddr += sFLASH_SPI_PAGE_SIZE;
				pBuffer += sFLASH_SPI_PAGE_SIZE;
			}

			if (NumOfSingle != 0)
			{
				sFLASH_WritePage(pBuffer, WriteAddr, NumOfSingle, numbFlashMemory);
			}
		}
	}
}

/**
 * @brief  Reads FLASH identification.
 * @param  None
 * @retval FLASH identification
 */
uint32_t sFLASH_ReadID(uint8_t numbFlashMemory)
{
	uint8_t txData[1];
	uint8_t rxData[3];

	txData[0] = FLASH_CMD_RDID;

	/* Select the FLASH: Chip Select low */
	// sFLASH_CS_LOW();

	if (numbFlashMemory == 1)
	{
		sFLASH1_CS_LOW();
	}
	else if (numbFlashMemory == 2)
	{
		sFLASH2_CS_LOW();
	}
	else
	{
		sFLASH1_CS_LOW();
	}

	/* Send "RDID " instruction */
	HAL_SPI_Transmit(&hspi1, txData, sizeof(txData), HAL_MAX_DELAY);
	/* Receive ID value */
	HAL_SPI_Receive(&hspi1, rxData, sizeof(rxData), HAL_MAX_DELAY);

	/* Deselect the FLASH: Chip Select high */
	// sFLASH_CS_HIGH();
	if (numbFlashMemory == 1)
	{
		sFLASH1_CS_HIGH();
	}
	else if (numbFlashMemory == 2)
	{
		sFLASH2_CS_HIGH();
	}
	else
	{
		sFLASH1_CS_HIGH();
	}
	return (rxData[0] << 16) | (rxData[1] << 8) | rxData[2];
}

/**
 * @brief  Enables the write access to the FLASH.
 * @param  None
 * @retval None
 */
void sFLASH_WriteEnable(uint8_t numbFlashMemory)
{
	uint8_t txData[1];

	txData[0] = FLASH_CMD_WREN;

	/* Select the FLASH: Chip Select low */
	// sFLASH_CS_LOW();

	if (numbFlashMemory == 1)
	{
		sFLASH1_CS_LOW();
	}
	else if (numbFlashMemory == 2)
	{
		sFLASH2_CS_LOW();
	}
	else
	{
		sFLASH1_CS_LOW();
	}

	/* Send "Write Enable" instruction */
	HAL_SPI_Transmit(&hspi1, txData, sizeof(txData), HAL_MAX_DELAY);

	/* Deselect the FLASH: Chip Select high */
	// sFLASH_CS_HIGH();
	if (numbFlashMemory == 1)
	{
		sFLASH1_CS_HIGH();
	}
	else if (numbFlashMemory == 2)
	{
		sFLASH2_CS_HIGH();
	}
	else
	{
		sFLASH1_CS_HIGH();
	}
}

/**
 * @brief  Polls the status of the Write In Progress (WIP) flag in the FLASH's
 *         status register and loop until write operation has completed.
 * @param  None
 * @retval None
 */
void sFLASH_WaitForWriteEnd(uint8_t numbFlashMemory)
{
	uint8_t txData[1];
	uint8_t rxData[1];

	txData[0] = FLASH_CMD_RDSR;

	/* Select the FLASH: Chip Select low */
	// sFLASH_CS_LOW();
	if (numbFlashMemory == 1)
	{
		sFLASH1_CS_LOW();
	}
	else if (numbFlashMemory == 2)
	{
		sFLASH2_CS_LOW();
	}
	else
	{
		sFLASH1_CS_LOW();
	}

	/* Send "Read Status Register" instruction */
	HAL_SPI_Transmit(&hspi1, txData, sizeof(txData), HAL_MAX_DELAY);

	/* Loop as long as the memory is busy with a write cycle */
	do
	{
		/* Receive "Read Status Register" value */
		HAL_SPI_Receive(&hspi1, rxData, sizeof(rxData), HAL_MAX_DELAY);
	} while ((rxData[0] & FLASH_WIP_MASK) == FLASH_WIP_MASK); /* Write in progress */

	/* Deselect the FLASH: Chip Select high */
	// sFLASH_CS_HIGH();
	if (numbFlashMemory == 1)
	{
		sFLASH1_CS_HIGH();
	}
	else if (numbFlashMemory == 2)
	{
		sFLASH2_CS_HIGH();
	}
	else
	{
		sFLASH1_CS_HIGH();
	}
}

/**
 * @brief
 * @param  numbFlashMemory
 * @retval free adress
 */
uint32_t sFLASH_SearchLastFreePageAdress (uint8_t numbFlashMemory)
{
	uint8_t txData[1];
	uint8_t rxData[1];

	txData[0] = FLASH_CMD_RDSR;

	/* Select the FLASH: Chip Select low */
	// sFLASH_CS_LOW();
	if (numbFlashMemory == 1)
	{
		sFLASH1_CS_LOW();
	}
	else if (numbFlashMemory == 2)
	{
		sFLASH2_CS_LOW();
	}
	else
	{
		sFLASH1_CS_LOW();
	}

	uint32_t idSearch = 0;
	size_t i = 0;
//	for (i = 0; i < sFLASH_NUMBER_LOGS; i++)
	for (i = 0; i < numbOfLogs; i++)
	{
		sFLASH_ReadBuffer((uint8_t *)&idSearch, sFLASH_SPI_PAGE_SIZE * i, 4, numbFlashMemory);
		if (idSearch == 0xFFFFFFFF)
		{
			break;
		}
	}

	/* Deselect the FLASH: Chip Select high */
	// sFLASH_CS_HIGH();
	if (numbFlashMemory == 1)
	{
		sFLASH1_CS_HIGH();
	}
	else if (numbFlashMemory == 2)
	{
		sFLASH2_CS_HIGH();
	}
	else
	{
		sFLASH1_CS_HIGH();
	}

	if (idSearch != 0xFFFFFFFF) //have not free memory
	{
		return 0xFFFFFFFF;
	}
	else //adress free memory
		return sFLASH_SPI_PAGE_SIZE * i;
}
