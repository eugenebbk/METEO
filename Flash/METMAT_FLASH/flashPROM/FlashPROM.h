/*
		https://istarik.ru/blog/stm32/142.html
 */

#ifndef FLASHPROM_H_
#define FLASHPROM_H_

#include "main.h"
#include "string.h"
#include "stdio.h"

#define STARTADDR ((uint32_t)0x00000000)         // адрес, с которого будет начинаться запись во флеш (с начала 126-ой страницы F103)
#define ENDMEMORY ((uint32_t)0x003FFFFF)  // последняя ячейка флеша 
#define PAGES 8                                  // количество страниц для очистки
#define BUFFSIZE 16                               // размер буфера для записи  (в размерность данных буфера(ниже))
#define DATAWIDTH 4                              // размерность данных буфера 16 бит - 2, 32 бита - 4
#define WIDTHWRITE FLASH_TYPEPROGRAM_WORD    // длина слова (16 бит FLASH_TYPEPROGRAM_HALFWORD) для записи в функции HAL_FLASH_Program(...), если 32бита тогда FLASH_TYPEPROGRAM_WORD
#define DEBUG 0

typedef uint32_t myBuf_t;                        // uint16_t либо uint32_t

//typedef struct 
//	{
//			myBuf_t canFilterID1ReceiveEEPROM;
//			myBuf_t canFilterID1TransmitEEPROM;
//			myBuf_t canFilterID2TransmitEEPROM;
//			myBuf_t canFilterID3TransmitEEPROM;
//			myBuf_t canFilterID3ReceiveEEPROM;
//			myBuf_t cycleModeCYMEEPROM;
//			myBuf_t intervalCYTEEPROM;
//			myBuf_t speedCANBusEEPROM;
//			myBuf_t widthAverageWindowEEPROM;
//			myBuf_t offsetAxysXEEPROM;
//			myBuf_t offsetAxysYEEPROM; //11
//			myBuf_t variableFinishStructure;//+1
//	} dataFLASHEEPROM_t;

typedef struct 
	{
			uint32_t canFilterID1ReceiveEEPROM;
			uint32_t canFilterID1TransmitEEPROM;
			uint32_t canFilterID2TransmitEEPROM;
			uint32_t canFilterID3TransmitEEPROM;
			uint32_t canFilterID3ReceiveEEPROM;
			uint32_t cycleModeCYMEEPROM;
			uint32_t intervalCYTEEPROM;
			uint32_t speedCANBusEEPROM;
			uint32_t widthAverageWindowEEPROM;
			int32_t offsetAxysXEEPROM;
			int32_t offsetAxysYEEPROM; //11
			float 	KoeffCompenstionTempX_K0EEPROM; //x^5
			float 	KoeffCompenstionTempX_K1EEPROM;
			float 	KoeffCompenstionTempX_K2EEPROM;
			float 	KoeffCompenstionTempX_K3EEPROM;
			float 	KoeffCompenstionTempX_K4EEPROM;
			float 	KoeffCompenstionTempX_K5EEPROM;
			float 	KoeffCompenstionTempY_K0EEPROM; //x^5
			float 	KoeffCompenstionTempY_K1EEPROM; 
			float 	KoeffCompenstionTempY_K2EEPROM;
			float 	KoeffCompenstionTempY_K3EEPROM;
			float 	KoeffCompenstionTempY_K4EEPROM;
			float 	KoeffCompenstionTempY_K5EEPROM;//21
			uint32_t variableFinishStructure;//+1
	} dataFLASHEEPROM_t;
	
//	typedef struct 
//	{
//			uint32_t canFilterID1ReceiveEEPROM;
//			uint32_t canFilterID1TransmitEEPROM;
//			uint32_t canFilterID2TransmitEEPROM;
//			uint32_t canFilterID3TransmitEEPROM;
//			uint32_t canFilterID3ReceiveEEPROM;
//			uint32_t cycleModeCYMEEPROM;
//			uint32_t intervalCYTEEPROM;
//			uint32_t speedCANBusEEPROM;
//			uint32_t widthAverageWindowEEPROM;
//			uint32_t offsetAxysXEEPROM;
//			uint32_t offsetAxysYEEPROM; //11
//			uint32_t variableFinishStructure;//+1
//			uint32_t cycleModeCYMRAM;//
//			uint32_t intervalCYTRAM;//
//	} dataFLASHEEPROM_t;
	
void erase_flash(void);
uint32_t flash_search_adress(uint32_t address, uint16_t cnt);
void write_to_flash(myBuf_t *buff);
//void write_to_flash(dataFLASHEEPROM_t *buff);
void read_last_data_in_flash(myBuf_t *buff);

#endif /* FLASHPROM_H_ */
