/**
 * @file hal-i2c-init.c
 *
 * @ingroup i2c
 *
 * @brief I2C driver for initializing the I2C instance on the STM32F4x series processors.
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

//================== STMF32 Support Functions =================================
#include <rtems.h>
#include <hal-utils.h>
#include <stm32f-processor-specific.h>
#include <bspopts.h>

#include stm_processor_header( TARGET_STM_PROCESSOR_PREFIX )
#include stm_header( TARGET_STM_PROCESSOR_PREFIX, dma )
#include stm_header( TARGET_STM_PROCESSOR_PREFIX, i2c )
#include stm_header( TARGET_STM_PROCESSOR_PREFIX, rcc )
#include stm_header( TARGET_STM_PROCESSOR_PREFIX, gpio )

#include <hal-i2c.h>
#include <hal-i2c-init.h>

/**
 * Extern the I2C configuration struct
 */
extern stm32_i2c_config i2c_config[ MAX_I2C_INSTANCES ];

/**
 * I2C Set Clock Rate
 */
int stm32_i2c_set_clock(
  i2c_bus *base,
  unsigned long clock
)
{
  I2C_HandleTypeDef i2c_handle;
  rtems_status_code sc = RTEMS_SUCCESSFUL;
  stm32_i2c_bus    *bus = (stm32_i2c_bus *) base;

  i2c_handle = bus->handle;

  HAL_I2C_DeInit( &bus->handle );

  bus->handle = i2c_handle;
  bus->handle.Init.ClockSpeed = (uint32_t) clock;

  return ( HAL_I2C_Init( &bus->handle ) );
}

/**
 *  Initialize the I2C instance
 */
int stm32_i2c_init( stm32_i2c_bus *bus )
{
  bus->handle.Instance = stm32_i2c_get_i2c_instance(bus->instance);
  bus->handle.Init.OwnAddress1 = i2c_config[ bus->instance ].address;
  bus->handle.Init.OwnAddress2 = 0xFE;

  bus->handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  bus->handle.Init.ClockSpeed = 100000;
  bus->handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  bus->handle.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
  bus->handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  bus->handle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  return ( HAL_I2C_Init( &bus->handle ) );
}
