#include "flasheeprom.h"
volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;


FLASH_Status FlashWriteEEprom(void *Data,uint16_t write_number)
{
	uint32_t NbrOfPage = 0x00;
	uint32_t EraseCounter = 0x00, Address = 0x00;
	uint8_t *Tempdata = (uint8_t*)Data;
	/* Porgram FLASH Bank1 ********************************************************/       
	  /* Unlock the Flash Bank1 Program Erase controller */
	  FLASH_UnlockBank1();
	  /* Define the number of page to be erased */
	  NbrOfPage = (BANK1_WRITE_END_ADDR - BANK1_WRITE_START_ADDR) / FLASH_PAGE_SIZE;

	  /* Clear All pending flags */
	  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);	

	  /* Erase the FLASH pages */
	  for(EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
	  {
			FLASHStatus = FLASH_ErasePage(BANK1_WRITE_START_ADDR + (FLASH_PAGE_SIZE * EraseCounter));
	  }
	  
	  /* Program Flash Bank1 */
	  Address = BANK1_WRITE_START_ADDR;

	  while((Address < BANK1_WRITE_START_ADDR + 2*write_number) && (FLASHStatus == FLASH_COMPLETE))
	  {
			FLASHStatus = FLASH_ProgramHalfWord(Address, *Tempdata);
			Address = Address + 2;
			Tempdata++;
	  }
	  FLASH_LockBank1();      
	return FLASHStatus;
}  
void FlashReadEEprom(void *Data,uint16_t write_number)
{
	 uint32_t Address = 0x00;
	 uint8_t *Tempdata = (uint8_t*)Data;
	  /* Check the correctness of written data */
	  Address = BANK1_WRITE_START_ADDR;

	  while(Address < BANK1_WRITE_START_ADDR + 2*write_number)
	  {
         *Tempdata = (*(__IO uint32_t*) Address);
         Address += 2;
         Tempdata++;
	  }
}


