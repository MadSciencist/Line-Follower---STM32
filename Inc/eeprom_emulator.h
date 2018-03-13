#ifndef eeprom_emulator_H
#define eeprom_emulator_H

#define SECTOR5_FLASH_BEGINING 0x08020000
#include "stm32f4xx.h"

void WriteToFlash(void* ptr, uint16_t size, uint32_t addres, uint32_t sector, uint32_t voltageRange);

void ReadFromFlash(void* ptr, uint16_t size, uint32_t addres);


#endif