#ifndef _STM32_SPI_FLASH_H_
#define _STM32_SPI_FLASH_H_

#include "main.h"

void send_command(uint8_t cmd);
void send_address(uint32_t addr);
uint8_t busy(void);
void flash_wr_enable(void);
void flash_GBPU(void);
void flash_sector_erase(uint32_t addr);
void flash_write(uint32_t addr, uint8_t* data, uint16_t size);
void flash_read(uint32_t addr, uint8_t* data, uint16_t size);

//uint8_t Read = 0x03;
//uint8_t SE = 0x20; //Sector erase - erase 4KB of memory array
//uint8_t WREN = 0x06; //Write Enable
//uint8_t PP = 0x02; //Page Program (Write)
//uint8_t ULBPR = 0x98; //Global Block Protection Unlock - clears all write protection bits
//uint8_t RDSR = 0x05; //Read Status Register

#define Read 0x03
#define SE 0x20 //Sector erase - erase 4KB of memory array
#define WREN 0x06 //Write Enable
#define PP 0x02 //Page Program (Write)
#define ULBPR 0x98 //Global Block Protection Unlock - clears all write protection bits
#define RDSR 0x05 //Read Status Register


#endif /* _STM32_SPI_FLASH_H_ */
