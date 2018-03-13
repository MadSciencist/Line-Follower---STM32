#include "eeprom_emulator.h"


void WriteToFlash(void* ptr, uint16_t size, uint32_t addres, uint32_t sector, uint32_t voltageRange ){
  
  __disable_irq();
  
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t SectorError = 0;
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = voltageRange;
  EraseInitStruct.Sector = sector;
  EraseInitStruct.NbSectors = 1; 
  
  HAL_FLASH_Unlock();
  
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
  
  if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK) {
    Error_Handler();
  }
  
  const uint8_t * p = (const uint8_t*) ptr;
  
  for (int i = 0; i < size; i++)
  {
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, i+addres, (*p++)); 
  }
  
  HAL_FLASH_Lock();
  
  __enable_irq();
  
}

void ReadFromFlash(void* ptr, uint16_t size, uint32_t addres)
{

  uint8_t * p = (uint8_t*) ptr;

  for (int j = 0; j < size; j++)
  {
    *p++ = *(uint8_t *)addres++;
  }
}