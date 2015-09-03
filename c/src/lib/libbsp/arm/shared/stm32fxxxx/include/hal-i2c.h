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
 *  Max I2C Instances available in system
 */
#define MAX_I2C_INSTANCES 				3

/**
 *  Number of Instances to be initailized
 */
#define I2C_INSTANCES 					3

/**
 *  Device Address for each I2C instances
 */
#define I2C1_ADDRESS        			0x30F
#define I2C2_ADDRESS        			0x31F
#define I2C3_ADDRESS       				0x32F

/**
 *  Definition for I2C1 Pins
 */
#define I2C1_SCL_PIN                    GPIO_PIN_6
#define I2C1_SCL_GPIO_PORT              GPIOB
#define I2C1_SDA_PIN                    GPIO_PIN_7
#define I2C1_SDA_GPIO_PORT              GPIOB

/**
 *  Definition for I2C2 Pins
 */
#define I2C2_SCL_PIN                    GPIO_PIN_10
#define I2C2_SCL_GPIO_PORT              GPIOB
#define I2C2_SDA_PIN                    GPIO_PIN_11
#define I2C2_SDA_GPIO_PORT              GPIOB

/**
 *  Definition for I2C3 Pins
 */
#define I2C3_SCL_PIN                    GPIO_PIN_8
#define I2C3_SCL_GPIO_PORT              GPIOA
#define I2C3_SDA_PIN                    GPIO_PIN_9
#define I2C3_SDA_GPIO_PORT              GPIOC

/**
 *  Alternate Pin Configuration for I2C
 */
#define I2C_AF                     		GPIO_AF4_I2C1

/**
 *  Definition for I2Cx's NVIC
 */
#define I2Cx_EV_IRQn                    I2C1_EV_IRQn
#define I2Cx_EV_IRQHandler              I2C1_EV_IRQHandler
#define I2Cx_ER_IRQn                    I2C1_ER_IRQn
#define I2Cx_ER_IRQHandler              I2C1_ER_IRQHandler

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
  I2C_THREE
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
