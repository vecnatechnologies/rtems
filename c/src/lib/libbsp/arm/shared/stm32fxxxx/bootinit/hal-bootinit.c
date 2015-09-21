/**
 * @file hal-bootinit.c
 *
 * @ingroup boot
 *
 * @brief Triggers the buit-in bootloader. Strictly processor specific.
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

#include <stdlib.h>
#include <stdio.h>
#include <hal-utils.h>
#include <stm32f-processor-specific.h>
#include <bspopts.h>

#include stm_processor_header( TARGET_STM_PROCESSOR_PREFIX )
#include stm_header( TARGET_STM_PROCESSOR_PREFIX, rcc )

void stm32f_bootloader_init( void )
{
  /* [SYS_MEM_ADDR]         -> The first 4 bytes contains the default stack pointer value
   *                           that the processor picks and sets the SP from MSP during reset.
   *
   *  *[SYS_MEM_ADDR + 0x4] -> This contains the address of the firmware of the built-in bootloader.
   *
   *  IMPORTANT!!    Each processor has a different default MSP and bootloader firmware address.
   *                 These values can/has to be fetched from the system ROM memory i.e. SYS_MEM_ADDR
   */

  /* Get the address of the built-in bootloader firmware */
  /*Address of Bootloader +  4 byte offset for stack pointer*/
  void (*SysBootJump) ( void ) =
    ( void ( * )( void ) )( *( (uint32_t *) ( SYS_MEM_ADDR +
                                              BOOTLOADER_OFFSET ) ) );

  /* Get default MSP value */
  uint32_t default_MSP = *( (uint32_t *) SYS_MEM_ADDR );

  /* Disable all interrupts. Disables the running RTOS */
  __disable_irq();

  /* Disable all peripheral clocks */
  HAL_RCC_DeInit();

  /* Reset the SysTick Timer. SysTick timer is used by the bootloader */
  SysTick->CTRL = 0;
  SysTick->LOAD = 0;
  SysTick->VAL = 0;

  /* Set the Main Stack Pointer (MSP) to its default value */
  __set_MSP( default_MSP );

  /* Jump to bootloader */
  SysBootJump();

  /* Should not return here */
}
