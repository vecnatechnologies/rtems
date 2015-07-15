/*
 * vecna_utils.h
 *
 *  Created on: Jul 14, 2015
 *      Author: jay.doyle
 */

#ifndef RTEMS_C_SRC_LIB_LIBBSP_ARM_STM32F4_INCLUDE_VECNA_UTILS_H_
#define RTEMS_C_SRC_LIB_LIBBSP_ARM_STM32F4_INCLUDE_VECNA_UTILS_H_

#define COUNTOF(x) (sizeof(x)/sizeof(x[0]))

#define __stm_header(x) #x
#define _stm_header(x, y) __stm_header(stm32f##x##_hal_##y.h)
#define stm_header(x,y) _stm_header(x,y)

#define __stm_processor_header(x) #x
#define _stm_processor_header(x) __stm_processor_header(stm32f##x.h)
#define stm_processor_header(x) _stm_processor_header(x)

#endif /* RTEMS_C_SRC_LIB_LIBBSP_ARM_STM32F4_INCLUDE_VECNA_UTILS_H_ */
