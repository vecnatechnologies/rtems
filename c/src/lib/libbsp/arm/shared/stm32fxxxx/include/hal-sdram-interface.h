/**
 * @file hal-sdram-interface.h
 * @author Jay M. Doyle
 *
 * @ingroup sdram
 *
 * @brief An interface layer to ST's hardware abstraction
 *   layer API functions used to configure external SDRAM.
 *
 */

/*
 * Copyright (c) 2015 Vecna Technologies, Inc.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.com/license/LICENSE.
 */

#ifndef INCLUDE_HAL_SDRAM_INTERFACE_H_
#define INCLUDE_HAL_SDRAM_INTERFACE_H_

/**
 * @brief Configures external SDRAM
 *
 * This routine configures the external SDRAM memory. This routine
 * relies upon various #defines that specify various aspects of
 * the STM32F FMC controller.
 *
 */
void BSP_SDRAM_Config( void );

/**
 * @brief Configured ARM Cortex-M MPU (Memory Protection Unit)
 */
void MPU_Config( void );

/**
 * @brief A helper function used to configure the MPU
 *   sections.
 *
 * @param region_size_in_bytes [in] The size of the memory region
 *   in bytes.  This must be power of 2 and must be >= 32 bytes and
 *   <= 4GB.
 *
 * @return The proper value for the Size member of MPU_Region_InitTypeDef
 *   for the specific memory region size.  If the input value is not a
 *   power of 2 then 0 is returned.
 */
uint8_t MPU_Get_Region_Size( const uint64_t region_size_in_bytes );

#endif /* INCLUDE_HAL_SDRAM_INTERFACE_H_ */
