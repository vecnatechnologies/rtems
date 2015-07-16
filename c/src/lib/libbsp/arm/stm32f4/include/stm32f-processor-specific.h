/**
 * @file stm32f-processor-specific.h
 *
 * @ingroup misc
 *
 * @brief Processor specific #defines for various STM32F microcontrollers
 *
 */

/*
 * Copyright (c) 2015 Vecna Technologies, Inc.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.com/license/LICENSE.
 */

#ifndef RTEMS_C_SRC_LIB_LIBBSP_ARM_STM32F4_INCLUDE_STM32F_PROCESSOR_SPECIFIC_H_
#define RTEMS_C_SRC_LIB_LIBBSP_ARM_STM32F4_INCLUDE_STM32F_PROCESSOR_SPECIFIC_H_

//=========================== STMF32F407 ==============================
#if defined(STM32F407xx)


#define NUM_PROCESSOR_UARTS 6

//=========================== STMF32F745 ==============================
#elif defined(STMF32F745xx)

#error "You need to define NUM_PROCESSOR_UARTS for STMF32F745xx"

//=========================== STMF32F746 ==============================
#elif defined(STMF32F746xx)

#error "You need to define NUM_PROCESSOR_UARTS for STMF32F746xx"

#else
#error "Unspecified processor type!!"
#endif

#endif /* RTEMS_C_SRC_LIB_LIBBSP_ARM_STM32F4_INCLUDE_STM32F_PROCESSOR_SPECIFIC_H_ */
