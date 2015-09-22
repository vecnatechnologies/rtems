/**
 * @file hal-fatal-error-handler.h
 *
 * @ingroup error
 *
 * @brief Error handler for CPU exceptions
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

#ifndef HAL_FATAL_ERROR_HANDLER_H_
#define HAL_FATAL_ERROR_HANDLER_H_

/**
 * @brief HAL fatal error handler function.
 */
rtems_status_code stm32_initialize_extensions( void );

#endif /* HAL_FATAL_ERROR_HANDLER_H_ */
