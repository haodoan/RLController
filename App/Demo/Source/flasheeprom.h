#ifndef __FLASHEEPROM_H
#define __FLASHEEPROM_H

//#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include "stm32f10x_flash.h"

#define FLASH_PAGE_SIZE         ((uint16_t)0x400)
#define BANK1_WRITE_START_ADDR  ((uint32_t)0x08007800)
#define BANK1_WRITE_END_ADDR    ((uint32_t)0x08008000)

FLASH_Status FlashWriteEEprom(void *Data,uint16_t write_number);
void FlashReadEEprom(void *Data,uint16_t write_number);
#endif
