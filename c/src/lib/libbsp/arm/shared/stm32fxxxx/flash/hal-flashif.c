/**
 * @file hal-flashif.c
 *
 * @ingroup flashif
 *
 * @brief Flash interface driver for the STM32xxxx series processors. Provides
 *        a  public routines to manage internal Flash programming (erase and write).
 *
 */

/*
 * Copyright (c) 2015 Vecna Technologies, Inc.
 *
 * Author: Sudarshan Rajagopalan
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.com/license/LICENSE.
 */

/* Includes ------------------------------------------------------------------*/
#include <rtems.h>
#include <hal-utils.h>
#include <stm32f-processor-specific.h>

#include stm_processor_header( TARGET_STM_PROCESSOR_PREFIX )
#include stm_header( TARGET_STM_PROCESSOR_PREFIX, flash )

#include "hal-flashif.h"

/*Variable used for Erase procedure*/
static FLASH_EraseInitTypeDef EraseInitStruct;

uint32_t SECTORError = 0;

/**
  * @brief  Gets the sector of a given address
  * @param  None
  * @retval The sector of a given address
  */
static uint32_t stm32_flash_get_sector(uint32_t Address)
{
  uint32_t sector = 0;

  if((Address < FLASH_SECTOR_1) && (Address >= FLASH_SECTOR_0))
  {
    sector = FLASH_SECTOR_0;
  }
  else if((Address < FLASH_SECTOR_2) && (Address >= FLASH_SECTOR_1))
  {
    sector = FLASH_SECTOR_1;
  }
  else if((Address < FLASH_SECTOR_3) && (Address >= FLASH_SECTOR_2))
  {
    sector = FLASH_SECTOR_2;
  }
  else if((Address < FLASH_SECTOR_4) && (Address >= FLASH_SECTOR_3))
  {
    sector = FLASH_SECTOR_3;
  }
  else if((Address < FLASH_SECTOR_5) && (Address >= FLASH_SECTOR_4))
  {
    sector = FLASH_SECTOR_4;
  }
  else if((Address < FLASH_SECTOR_6) && (Address >= FLASH_SECTOR_5))
  {
    sector = FLASH_SECTOR_5;
  }
  else if((Address < FLASH_SECTOR_7) && (Address >= FLASH_SECTOR_6))
  {
    sector = FLASH_SECTOR_6;
  }
  else /* (Address < FLASH_END_ADDR) && (Address >= FLASH_SECTOR_7) */
  {
    sector = FLASH_SECTOR_7;
  }
  return sector;
}

/**
  * @brief  Unlocks Flash for write access
  * @param  None
  * @retval None
  */
void stm32_flash_if_init(void)
{ 
  HAL_FLASH_Unlock();
}

/**
  * @brief  Locks Flash for write access
  * @param  None
  * @retval None
  */
void stm32_flash_if_lock(void)
{
  HAL_FLASH_Lock();
}

/**
  * @brief  Erases the internal flash area from start_sector to end_sector
  * @param  start_sector: start of flash area to erase
  * @param  end_sector: end of flash area to erase
  * @retval 0: user flash area successfully erased
  *         1: error occurred
  */
uint8_t stm32_flash_if_erase(uint32_t start_sector, uint32_t end_sector)
{
  uint32_t FirstSector = 0, NbOfSectors = 0;

  /* Get the 1st sector to erase */
  FirstSector = stm32_flash_get_sector(start_sector);
  /* Get the number of sector to erase from 1st sector*/
  NbOfSectors = stm32_flash_get_sector(end_sector) - FirstSector + 1;

  EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
  EraseInitStruct.Sector        = start_sector;
  EraseInitStruct.NbSectors     = NbOfSectors;

  /* Device voltage range supposed to be [2.7V to 3.6V], the operation will
     be done by word */ 
 
  if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK)
  {
      return (1);
  }

  return (0);
}
/**
  * @brief  This function writes a data buffer in flash (data are 32-bit aligned).
  * @note   After writing data buffer, the flash content is checked.
  * @param  flash_address: start address for writing data buffer
  * @param  data: pointer on data buffer
  * @param  data_length: length of data buffer (unit is 32-bit word)
  * @retval 0: Data successfully written to Flash memory
  *         1: Error occurred while writing data in Flash memory
  *         2: Written Data in flash memory is different from expected one
  */
uint32_t stm32_flash_if_write( uint8_t* flash_address, uint8_t* data ,uint16_t data_length)
{
  uint32_t i = 0;

  for (i = 0; (i < data_length) && (flash_address <= (USER_FLASH_END_ADDRESS-1)); i++)
  {
    /* Device voltage range supposed to be [2.7V to 3.6V], the operation will
       be done by word */

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, flash_address, *(uint8_t*)(data+i)) == HAL_OK)
    {
     /* Check the written value */
      if (*(uint8_t*)flash_address != *(uint8_t*)(data+i))
      {
        /* Flash content doesn't match SRAM content */
        return(2);
      }
      /* Increment FLASH destination address */
      flash_address += 1;
    }
    else
    {
      /* Error occurred while writing data in Flash memory */
      return (1);
    }
  }

  return (0);
}
