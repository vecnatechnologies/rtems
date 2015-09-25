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
 * Author: Sudarshan Rajagopalan
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.com/license/LICENSE.
 */

#ifndef HAL_I2C_INIT_H
#define HAL_I2C_INIT_H

/**
 * I2C Set Clock Rate
 */
int stm32_i2c_set_clock(
    i2c_bus *base,
    unsigned long clock
);

/**
 *  Initialize the I2C instance
 */
int stm32_i2c_init (stm32_i2c_bus * bus);

#endif /* HAL_I2C_INIT_H */
