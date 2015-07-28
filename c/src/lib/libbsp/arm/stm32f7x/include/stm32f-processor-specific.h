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
#if defined(STM32F746xx)

#define NUM_PROCESSOR_UARTS        1
#define ENABLE_PROCESSOR_OVERDRIVE 1
#define ENABLE_PROCESSOR_CACHES    1

//TODO: Move this to some place more universal
#define SYSCLK_FREQUENCY        STM32F4_SYSCLK
#define HSE_FREQUENCY           STM32F4_HSE_OSCILLATOR
#define HSI_FREQUENCY           16000000
#define STM32F_FLASH_LATENCY    FLASH_LATENCY_7
#define HSE_AVAILABLE           ((HSE_FREQUENCY > 0) ? 1 : 0)

#else
#error "Unspecified processor type!!"
#endif

#endif /* RTEMS_C_SRC_LIB_LIBBSP_ARM_STM32F4_INCLUDE_STM32F_PROCESSOR_SPECIFIC_H_ */
