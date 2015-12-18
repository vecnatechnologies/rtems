/**
 * @file hal-flashif.h
 *
 * @ingroup flashif
 *
 * @brief Internal flash interface funtion prototypes
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FLASH_IF_H
#define __FLASH_IF_H


#define USER_FLASH_END_ADDRESS   0x080FFFFF
#define USER_FLASH_SIZE   (USER_FLASH_END_ADDRESS - USER_FLASH_FIRST_PAGE_ADDRESS)


/* Exported functions ------------------------------------------------------- */
void stm32_flash_if_init(void);
void stm32_flash_if_lock(void);
uint8_t stm32_flash_if_erase(uint32_t start_sector, uint32_t end_sector);
uint32_t stm32_flash_if_write( uint8_t* flash_address, uint8_t* data ,uint16_t data_length);

#endif /* __FLASH_IF_H */
