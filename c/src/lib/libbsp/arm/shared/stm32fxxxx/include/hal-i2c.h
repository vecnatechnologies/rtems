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
 * Author: Sudarshan Rajagopalan
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

/* Convert to string */
#define STRINGIFY( x ) #x

/* The following two macro is used to get the IRQ number I2Cx_IRQn, where x=1-4 */
#define _GET_I2C_VEC( x ) I2C##x_EV_IRQn
#define GET_I2C_VEC( instance ) STRINGIFY( _GET_I2C_VEC( instance ) )

/* SPI timeout value */
#define I2C_WAIT_TIMEOUT 10

/* Max charactors for bus path */
#define MAX_PATH_CHAR   12

/* I2C1 Pin Configuration */
#if ( STM32_ENABLE_I2C1 )
#define I2C1_PORT STM32_I2C1_PORT
#define I2C1_SCK_PIN STM32_I2C1_SCL_PIN
#define I2C1_SDA_PIN STM32_I2C1_SDA_PIN
#endif

/* I2C2 Pin Configuration */
#if ( STM32_ENABLE_I2C2 )
#define I2C2_PORT STM32_I2C2_PORT
#define I2C2_SCK_PIN STM32_I2C2_SCL_PIN
#define I2C2_SDA_PIN STM32_I2C2_SDA_PIN
#endif

/* I2C3 Pin Configuration */
#if ( STM32_ENABLE_I2C3 )
#define I2C3_PORT STM32_I2C3_PORT
#define I2C3_SCK_PIN STM32_I2C3_SCL_PIN
#define I2C3_SDA_PIN STM32_I2C3_SDA_PIN
#endif

/* I2C4 Pin Configuration */
#if ( STM32_ENABLE_I2C4 )
#define I2C4_PORT STM32_I2C4_PORT
#define I2C4_SCK_PIN STM32_I2C4_SCL_PIN
#define I2C4_SDA_PIN STM32_I2C4_SDA_PIN
#endif

/* Typedefs */

typedef struct i2c_system_bus i2c_system_bus;
typedef struct i2c_base i2c_base;

/**
 * I2C Instance
 */
typedef enum {
#if (STM32_ENABLE_I2C1)
  /* I2C instance 1 */
  I2C_ONE,
#endif
  /* I2C instance 2 */
#if (STM32_ENABLE_I2C2)
  I2C_TWO,
#endif
  /* I2C instance 3 */
#if (STM32_ENABLE_I2C3)
  I2C_THREE,
#endif
  /* I2C instance 4 */
#if (STM32_ENABLE_I2C4)
  I2C_FOUR,
#endif

  /* Max number of I2C Instances */
  MAX_I2C_INSTANCES
} I2C_Instance;

/**
 * I2C Configuration Struct
 */
typedef struct {
  /* Pointer to the instance base address */
  SPI_TypeDef *instance;
  /* I2C instance number */
  uint8_t instance_num;
  /* IRQ vector number */
  IRQn_Type vector_num;
  /* I2C own address */
  uint32_t address;
  /* GPIO port base address */
  GPIO_TypeDef *port;
  /* SCK pin */
  uint32_t sck_pin;
  /* SDA pin */
  uint32_t sda_pin;
  /* Alternate function */
  uint32_t alternate_func;
} stm32_i2c_config;

/**
 * Stm32 I2C Bus Struct
 */
typedef struct {
  /* I2C base registered to RTEMS fielsystem */
  i2c_bus base;
  /* I2C handle */
  I2C_HandleTypeDef handle;
  /* Instance number */
  I2C_Instance instance;
  /* Task ID */
  rtems_id task_id;
} stm32_i2c_bus;

#endif /* STM32_I2C_H */
