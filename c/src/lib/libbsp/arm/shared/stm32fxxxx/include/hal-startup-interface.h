/**
 * @file hal-startup-interface.h
 *
 * @ingroup startup
 *
 * @brief An interface layer to ST's hardware abstraction
 *   layer API functions used in UART driver implementation.
 *
 */

/*
 * Copyright (c) 2015 Vecna Technologies, Inc.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.com/license/LICENSE.
 */

#ifndef RTEMS_C_SRC_LIB_LIBBSP_ARM_STM32F4_HAL_STARTUP_INTERFACE_H
#define RTEMS_C_SRC_LIB_LIBBSP_ARM_STM32F4_HAL_STARTUP_INTERFACE_H

void bsp_start (void);
#endif // RTEMS_C_SRC_LIB_LIBBSP_ARM_STM32F4_HAL_STARTUP_INTERFACE_H
