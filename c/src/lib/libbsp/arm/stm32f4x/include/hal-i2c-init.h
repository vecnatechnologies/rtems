/**
 * @file hal-i2c-init.h
 *
 * @ingroup misc
 *
 * @brief I2C init function prototype
 *
 */

/*
 * Copyright (c) 2015 Vecna Technologies, Inc.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.com/license/LICENSE.
 */

#ifndef HAL_I2C_INIT_H
#define HAL_I2C_INIT_H

/**
 *  Max I2C Instances available in system
 */
#define MAX_I2C_INSTANCES 				3

/**
 *  Number of Instances to be initailized
 */
#define I2C_INSTANCES 					3

/**
 * Initialize GPIO pins
 */
void stm32_i2c_gpio_init(stm32_i2c_bus * bus);

/**
 *  Initialize the I2C instance
 */
int stm32_i2c_init (stm32_i2c_bus * bus);

#endif /* HAL_I2C_INIT_H */
