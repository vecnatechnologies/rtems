/**
 * @file hal-i2c.h
 *
 * @ingroup i2c
 *
 * @brief Public I2C driver datatypes
 *
 */

/*
 * Copyright (c) 2015 Vecna Technologies, Inc.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.com/license/LICENSE.
 */

#ifndef STM32_I2C_H
#define STM32_I2C_H

#include <stdint.h>
#include <stdbool.h>

#include <dev/i2c/i2c.h>
#include <linux/i2c.h>

/************************ Defines ******************************/

/**
 *  Alternate Pin Configuration for I2C
 */
#define I2C_AF                     		GPIO_AF4_I2C1

/************************ Typedefs ******************************/

typedef struct i2c_system_bus i2c_system_bus;
typedef struct i2c_base i2c_base;

/**
 * I2C Instance
 */
typedef enum
{
  I2C_ONE = 0,
  I2C_TWO,
  I2C_THREE,
  I2C_FOUR
} I2C_Instance;

/**
 * Stm32 I2C Bus Struct
 */
typedef struct
{
  i2c_bus base;
  I2C_HandleTypeDef handle;
  I2C_Instance instance;
  rtems_id task_id;
}stm32_i2c_bus;


#endif
