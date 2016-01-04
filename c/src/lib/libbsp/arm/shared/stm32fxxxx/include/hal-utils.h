/**
 * @file hal-utils.h
 * @author Jay M. Doyle
 *
 * @ingroup misc
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

#ifndef HAL_UTILS_H
#define HAL_UTILS_H_

/**
 * A macro that can be used determine the size of any array
 */
#ifndef COUNTOF
#define COUNTOF( x ) ( sizeof( x ) / sizeof( x[ 0 ] ) )
#endif

// The following macros are used to include the correct HAL header files
// based upon the specific type of STM32F processor used in the target
// BSP.  The name of the HAL include files differs slightly between families
// and these macros will ensure the correct header file for the target
// processor are included.

#define __stm_header( x ) #x
#define _stm_header( x, y ) __stm_header( stm32f ## x ## _hal_ ## y.h )
#define stm_header( x, y ) _stm_header( x, y )

#define __stm_ll_header( x ) #x
#define _stm_ll_header( x, y ) __stm_ll_header( stm32f ## x ## _ll_ ## y.h )
#define stm_ll_header( x, y ) _stm_ll_header( x, y )

#define __stm_processor_header( x ) #x
#define _stm_processor_header( x ) __stm_processor_header( stm32f ## x.h )
#define stm_processor_header( x ) _stm_processor_header( x )

#define __stm_hal_header( x ) #x
#define _stm_hal_header( x ) __stm_hal_header( stm32f ## x ## _hal.h )
#define stm_hal_header( x ) _stm_hal_header( x )

#include <stm32f-processor-specific.h>
#include stm_processor_header( TARGET_STM_PROCESSOR_PREFIX )

void HAL_Delay( __IO uint32_t Delay );

#endif /* RTEMS_C_SRC_LIB_LIBBSP_ARM_STM32F4_INCLUDE_VECNA_UTILS_H_ */
