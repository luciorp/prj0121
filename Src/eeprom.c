/*
 * eeprom.c
 *
 *  Created on: 3 de nov de 2018
 *      Author: lucio
 */
#include "stm32f1xx_hal.h"
#include "eeprom.h"

static FLASH_EraseInitTypeDef EraseInitStruct;

 uint8_t eeprom_erase_data(){

	 uint32_t PAGEError = 0;

	 HAL_FLASH_Unlock();

	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = FLASH_USER_START_ADDR;
	EraseInitStruct.NbPages     = (FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) / FLASH_PAGE_SIZE;

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)   {
		printf("Erro apagar eeprom\n\r");

		return -1;
	}

	HAL_FLASH_Lock();

	return 0;

}



 uint8_t eeprom_write_data(){

  uint32_t Address = FLASH_USER_START_ADDR;

  while (Address < FLASH_USER_END_ADDR)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, 0x22) == HAL_OK)
    {
      Address = Address + 4;
    }

  }

  HAL_FLASH_Lock();

  return 0;

 }

  uint8_t eeprom_read_data(uint32_t *data32){

  uint32_t Address = FLASH_USER_START_ADDR;


  while (Address < FLASH_USER_END_ADDR)   {
    data32 = (__IO uint32_t *)Address;

    Address = Address + 4;
  }
  return 0;
 }

