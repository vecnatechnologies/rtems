/**
 * @file hal-startup.c
 * @author Jay M. Doyle
 *
 * @ingroup startup
 *
 * @brief A set of utility functions for configuring clocks, caches,
 * an external memory.
 *
 */

/*
 * Copyright (c) 2015 Vecna Technologies, Inc.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.com/license/LICENSE.
 */

#include <rtems.h>
#include <bspopts.h>
#include <hal-utils.h>
#include <stm32f-processor-specific.h>

#include stm_processor_header( TARGET_STM_PROCESSOR_PREFIX )
#include stm_header( TARGET_STM_PROCESSOR_PREFIX, rcc )
#include stm_header( TARGET_STM_PROCESSOR_PREFIX, conf )

#include <hal-startup-interface.h>
#include <hal-sdram-interface.h>
#include <hal-uart-interface.h>


void bsp_start( void )
{
  // initialize interrupt vectors with default handler
  bsp_interrupt_initialize();
}

void bsp_predriver_hook( void )
{
  //stm32_bsp_register_uart();
  //stm32_bsp_register_can();
  //stm32_bsp_register_i2c();
  //stm32_bsp_register_spi();
  //stm32f_initialize_user_extensions();
}
