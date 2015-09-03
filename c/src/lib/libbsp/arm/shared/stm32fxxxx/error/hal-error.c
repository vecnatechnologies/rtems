/**
 * @file hal-error.c
 *
 * @ingroup error
 *
 * @brief Defines generic error handler referenced throughout the STM32F
 *  HAL code.
 *
 */

/*
 * Copyright (c) 2015 Vecna Technologies, Inc.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.com/license/LICENSE.
 */
void Error_Handler(
  void
)
{
    while(1) {
        ; // Stay here forever
    }
}