/**
 * @file hal-utils.h
 *
 * @ingroup util
 *
 * @brief A set of utility functions used throughout the
 *   STM32F BSP.
 *
 */

/*
 * Copyright (c) 2015 Vecna Technologies, Inc.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.com/license/LICENSE.
 */

#include <hal-utils.h>
#include <stm32f-processor-specific.h>
#include stm_processor_header(TARGET_STM_PROCESSOR_PREFIX)

void HAL_Delay(__IO uint32_t Delay)
{
  //TODO: FIND A BETTER WAY!!!

  volatile uint32_t i;
  volatile uint32_t j;

  for(i = 0; i < Delay; i++) {
    for(j = 0; j < 10000; j++) {
      asm("nop");
    }
  }
}
