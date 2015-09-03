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


/* I2C TIMING Register define when I2C clock source is APB1 (SYSCLK/4) */
/* I2C TIMING is calculated in case of the I2C Clock source is the APB1CLK = 50 MHz */
/* This example use TIMING to 0x40912732 to reach 100 kHz speed (Rise time = 700 ns, Fall time = 100 ns) */
#define I2C_TIMING      0x40912732
//TODO:: Create a function or macro that calculates this value depending on APB1CLK

/**
 *  Max I2C Instances available in system
 */
#define MAX_I2C_INSTANCES 				4

/**
 *  Number of Instances to be initailized
 */
#define I2C_INSTANCES 					4

/**
 * Initialize GPIO pins
 */
void stm32_i2c_gpio_init(stm32_i2c_bus * bus);

/**
 *  Initialize the I2C instance
 */
int stm32_i2c_init (stm32_i2c_bus * bus);

#endif /* HAL_I2C_INIT_H */
