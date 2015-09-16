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

#define STRINGIFY(x)                #x

#define _GET_I2C_VEC(x)				I2C##x_EV_IRQn
#define GET_I2C_VEC(__INSTANCE__)	STRINGIFY(_GET_I2C_VEC(__INSTANCE__))

/* I2C1 Pin Configuration */
#if (STM32_ENABLE_I2C1)
#define I2C1_PORT		STM32_I2C1_PORT
#define I2C1_SCK_PIN	STM32_I2C1_SCL_PIN
#define I2C1_SDA_PIN	STM32_I2C1_SDA_PIN
#endif

/* I2C2 Pin Configuration */
#if (STM32_ENABLE_I2C2)
#define I2C2_PORT		STM32_I2C2_PORT
#define I2C2_SCK_PIN	STM32_I2C2_SCL_PIN
#define I2C2_SDA_PIN	STM32_I2C2_SDA_PIN
#endif

/* I2C3 Pin Configuration */
#if (STM32_ENABLE_I2C3)
#define I2C3_PORT		STM32_I2C3_PORT
#define I2C3_SCK_PIN	STM32_I2C3_SCL_PIN
#define I2C3_SDA_PIN	STM32_I2C3_SDA_PIN
#endif

/* I2C4 Pin Configuration */
#if (STM32_ENABLE_I2C4)
#define I2C4_PORT		STM32_I2C4_PORT
#define I2C4_SCK_PIN	STM32_I2C4_SCL_PIN
#define I2C4_SDA_PIN	STM32_I2C4_SDA_PIN
#endif

/**
 *  Max I2C Instances available in system
 */
#define MAX_I2C_INSTANCES 				3

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
 * I2C Pin Config Struct
 */
typedef struct
{
GPIO_TypeDef *port;
uint32_t sck_pin;
uint32_t sda_pin;
uint32_t alternate_func;
}stm32_i2c_pin_config;

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


/**
 * Returns the base address of the I2C instances
 */
I2C_TypeDef* stm32_i2c_get_spi_instance(I2C_Instance i2c_instance);


#endif
