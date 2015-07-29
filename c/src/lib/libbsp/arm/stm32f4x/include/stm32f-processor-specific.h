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

// This processor does not support external SDRAM
#undef EXTERNAL_SDRAM

// This processor does not support overdrive mode
#undef ENABLE_PROCESSOR_OVERDRIVE

// clock configuration
#define SYSCLK_FREQUENCY        STM32F4_SYSCLK
#define HSE_VALUE               STM32F4_HSE_OSCILLATOR
#define HSI_FREQUENCY           16000000
#define STM32F_FLASH_LATENCY    FLASH_LATENCY_7
#define HSE_AVAILABLE           1
#define MAX_SYSCLK              168000000
#define APB1_CLK                STM32F4_PCLK1
#define APB2_CLK                STM32F4_PCLK2

// uart configuration
#define NUM_PROCESSOR_UARTS     6

#else
#error "Unspecified processor type!!"
#endif

#endif /* RTEMS_C_SRC_LIB_LIBBSP_ARM_STM32F4_INCLUDE_STM32F_PROCESSOR_SPECIFIC_H_ */
