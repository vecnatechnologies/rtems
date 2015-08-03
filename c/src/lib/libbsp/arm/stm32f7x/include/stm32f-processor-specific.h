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

// Processor functionality
#define ENABLE_PROCESSOR_OVERDRIVE 1
#define ENABLE_PROCESSOR_CACHES    1

// SDRAM Configuration
#define EXTERNAL_SDRAM                           1
#define SDRAM_BANK_ADDR                          ((uint32_t)0xC0000000)
#define SDRAM_MEMORY_WIDTH                       FMC_SDRAM_MEM_BUS_WIDTH_16
#define SDCLOCK_PERIOD                           FMC_SDRAM_CLOCK_PERIOD_2
#define SDRAM_TIMEOUT                            ((uint32_t)0xFFFF)

#define SDRAM_MODEREG_BURST_LENGTH_1             ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_LENGTH_2             ((uint16_t)0x0001)
#define SDRAM_MODEREG_BURST_LENGTH_4             ((uint16_t)0x0002)
#define SDRAM_MODEREG_BURST_LENGTH_8             ((uint16_t)0x0004)
#define SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL      ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_TYPE_INTERLEAVED     ((uint16_t)0x0008)
#define SDRAM_MODEREG_CAS_LATENCY_2              ((uint16_t)0x0020)
#define SDRAM_MODEREG_CAS_LATENCY_3              ((uint16_t)0x0030)
#define SDRAM_MODEREG_OPERATING_MODE_STANDARD    ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_PROGRAMMED ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_SINGLE     ((uint16_t)0x0200)

// Clock Configuration
#define SYSCLK_FREQUENCY        STM32F7_SYSCLK
#define HSI_FREQUENCY           16000000
#define STM32F_FLASH_LATENCY    FLASH_LATENCY_7
#define HSE_AVAILABLE           1
#define MAX_SYSCLK              216000000
#define APB1_CLK                STM32F7_PCLK1
#define APB2_CLK                STM32F7_PCLK2

// Uart configuration
#define NUM_PROCESSOR_CONSOLE_UARTS 1
#define NUM_PROCESSOR_UARTS         2

#else
#error "Unspecified processor type!!"
#endif

#endif /* RTEMS_C_SRC_LIB_LIBBSP_ARM_STM32F4_INCLUDE_STM32F_PROCESSOR_SPECIFIC_H_ */
