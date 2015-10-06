/**
 * @file hal-error.h
 * @author Jay M. Doyle
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

#ifndef INCLUDE_HAL_ERROR_H_
#define INCLUDE_HAL_ERROR_H_

#include <rtems.h>

/**
 * @brief HAL ST32F error handler function.
 */
void stm32f_error_handler( void );

/**
 * @brief HAL ST32F error handler function.
 *
 * @param error_text Text describing the source of the error
 */
void stm32f_error_handler_with_reason( const char *error_text );


void stm32f_error_handler_with_reason_conditional( const char *error_text,
                                                   const bool stop_executing);

#endif /* INCLUDE_HAL_ERROR_H_ */
