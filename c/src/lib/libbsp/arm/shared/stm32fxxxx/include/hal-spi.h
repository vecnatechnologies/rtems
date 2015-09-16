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

#ifndef STM32_SPI_H
#define STM32_SPI_H

#include <stdint.h>
#include <stdbool.h>

#include <dev/spi/spi.h>


/************************ Defines ******************************/
#define STRINGIFY(x)  #x


#define _PIN(x)       GPIO_PIN_##x
#define PIN(x)        STRINGIFY(_PIN(x))

#define _GET_SPI_VEC(x)                SPI##x_IRQn
#define GET_SPI_VEC(__INSTANCE__)      STRINGIFY(_GET_SPI_VEC(__INSTANCE__))


/* Shift left __PIN__ number of times to get the Pin value */
#define GET_PIN(__PIN__)    (GPIO_PIN_0<<__PIN__)

#define SPI_WAIT_TIMEOUT    100

/* SPI1 Pin Configuration */
#if (STM32_ENABLE_SPI1)
#define SPI1_PORT       STM32_SPI1_PORT
#define SPI1_SCK_PIN    STM32_SPI1_SCK_PIN
#define SPI1_MOSI_PIN   STM32_SPI1_MOSI_PIN
#define SPI1_MISO_PIN   STM32_SPI1_MISO_PIN
#endif

/* SPI2 Pin Configuration */
#if (STM32_ENABLE_SPI2)
#define SPI2_PORT       STM32_SPI2_PORT
#define SPI2_SCK_PIN    STM32_SPI2_SCK_PIN
#define SPI2_MOSI_PIN   STM32_SPI2_MOSI_PIN
#define SPI2_MISO_PIN   STM32_SPI2_MISO_PIN
#endif

/* SPI3 Pin Configuration */
#if (STM32_ENABLE_SPI3)
#define SPI3_PORT       STM32_SPI3_PORT
#define SPI3_SCK_PIN    STM32_SPI3_SCK_PIN
#define SPI3_MOSI_PIN   STM32_SPI3_MOSI_PIN
#define SPI3_MISO_PIN   STM32_SPI3_MISO_PIN
#endif

/* SPI4 Pin Configuration */
#if (STM32_ENABLE_SPI4)
#define SPI4_PORT       STM32_SPI4_PORT
#define SPI4_SCK_PIN    STM32_SPI4_SCK_PIN
#define SPI4_MOSI_PIN   STM32_SPI4_MOSI_PIN
#define SPI4_MISO_PIN   STM32_SPI4_MISO_PIN
#endif

/* SPI5 Pin Configuration */
#if (STM32_ENABLE_SPI5)
#define SPI5_PORT        STM32_SPI5_PORT
#define SPI5_SCK_PIN     STM32_SPI5_SCK_PIN
#define SPI5_MOSI_PIN    STM32_SPI5_MOSI_PIN
#define SPI5_MISO_PIN    STM32_SPI5_MISO_PIN
#endif

/* SPI6 Pin Configuration */
#if (STM32_ENABLE_SPI56)
#define SPI56_PORT       STM32_SPI56_PORT
#define SPI56_SCK_PIN    STM32_SPI56_SCK_PIN
#define SPI56_MOSI_PIN   STM32_SPI56_MOSI_PIN
#define SPI56_MISO_PIN   STM32_SPI56_MISO_PIN
#endif

/*********************** Typedefs *******************/

/**
 * SPI Instance
 */
typedef enum
{
  SPI_ONE = 0,
  SPI_TWO,
  SPI_THREE,
  SPI_FOUR,
  SPI_FIVE,
  SPI_SIX,

/* Max number of SPI Instances */
  MAX_SPI_INSTANCES
} SPI_Instance;

/**
 * SPI Pin Config Struct
 */
typedef struct
{
GPIO_TypeDef *port;
uint32_t sck_pin;
uint32_t miso_pin;
uint32_t mosi_pin;
uint32_t alternate_func;
}stm32_spi_pin_config;


/**
 * Stm32 SPI Bus Struct
 */
typedef struct
{
  spi_bus base;
  SPI_HandleTypeDef *handle;
  SPI_Instance instance;
  rtems_id task_id;
}stm32_spi_bus;

#endif /* STM32_SPI_H*/
