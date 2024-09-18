#include "spi.h"
#include "SST26.h"

//current free id for writing
// id 0..16383
/**
 * @brief
 * @param  
 * @retval free adress
 */
uint32_t sFLASH_SearchLastFreePageAdress(uint16_t *ID_log, uint8_t numbFlashMemory)
{

	uint32_t idSearch = 0;
	size_t id = 0;
	for (id = 0; id < sFLASH_NUMBER_LOGS_IN_FLASH; id++)
	// for (id = 0; id < numbOfLogs; id++)
	{
		/* Select the FLASH: Chip Select low */
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
		sFLASH_ReadBuffer((uint8_t *)&idSearch, sFLASH_SPI_PAGE_SIZE * id, 4, numbFlashMemory);
		/* Deselect the FLASH: Chip Select high */
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

		if (idSearch == 0xFFFFFFFF)
		{
			break;
		}
		// HA
	}
	
	if (idSearch != 0xFFFFFFFF) // have not free memory
	{
		*ID_log = 0xFFFF; //?
		return 0xFFFFFFFF;
	}
	else // adress free memory
	{
		*ID_log = id;
		return sFLASH_SPI_PAGE_SIZE * id;

	}
}
