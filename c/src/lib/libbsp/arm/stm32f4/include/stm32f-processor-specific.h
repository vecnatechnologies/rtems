/*
 * stm32f-processor-specific.h
 *
 *  Created on: Jul 14, 2015
 *      Author: jay.doyle
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
