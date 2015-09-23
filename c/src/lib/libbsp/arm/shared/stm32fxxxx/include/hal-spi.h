/**
 * @file hal-spi.h
 *
 * @ingroup spi
 *
 * @brief Public SPI driver datatypes
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

#ifndef STM32_SPI_H
#define STM32_SPI_H

#include <stdint.h>
#include <stdbool.h>

#include <dev/spi/spi.h>

/* Convert to string */
#define STRINGIFY( x ) #x

/* The following two macros are used to get the GPIO pin base - GPIO_PIN_x, where x=1-15 */
#define _PIN( x ) GPIO_PIN_##x
#define PIN( x ) STRINGIFY( _PIN( x ) )

/* The followign two macro is used to get the IRQ number SPIx_IRQn, where x=1,2, or 3 4 */
#define _GET_SPI_VEC( x ) SPI##x_IRQn
#define GET_SPI_VEC( instance ) STRINGIFY( _GET_SPI_VEC( instance ) )

/* Shift left "pin" number of times to get the Pin value */
#define GET_PIN( pin ) ( GPIO_PIN_0 << pin )

/* SPI timeout value */
#define SPI_WAIT_TIMEOUT 10

/* Max charactors for bus path */
#define MAX_PATH_CHAR   12

/* SPI1 Pin Configuration */
#if ( STM32_ENABLE_SPI1 )
#define SPI1_PORT STM32_SPI1_PORT
#define SPI1_SCK_PIN STM32_SPI1_SCK_PIN
#define SPI1_MOSI_PIN STM32_SPI1_MOSI_PIN
#define SPI1_MISO_PIN STM32_SPI1_MISO_PIN
#endif

/* SPI2 Pin Configuration */
#if ( STM32_ENABLE_SPI2 )
#define SPI2_PORT STM32_SPI2_PORT
#define SPI2_SCK_PIN STM32_SPI2_SCK_PIN
#define SPI2_MOSI_PIN STM32_SPI2_MOSI_PIN
#define SPI2_MISO_PIN STM32_SPI2_MISO_PIN
#endif

/* SPI3 Pin Configuration */
#if ( STM32_ENABLE_SPI3 )
#define SPI3_PORT STM32_SPI3_PORT
#define SPI3_SCK_PIN STM32_SPI3_SCK_PIN
#define SPI3_MOSI_PIN STM32_SPI3_MOSI_PIN
#define SPI3_MISO_PIN STM32_SPI3_MISO_PIN
#endif

/* SPI4 Pin Configuration */
#if ( STM32_ENABLE_SPI4 )
#define SPI4_PORT STM32_SPI4_PORT
#define SPI4_SCK_PIN STM32_SPI4_SCK_PIN
#define SPI4_MOSI_PIN STM32_SPI4_MOSI_PIN
#define SPI4_MISO_PIN STM32_SPI4_MISO_PIN
#endif

/* SPI5 Pin Configuration */
#if ( STM32_ENABLE_SPI5 )
#define SPI5_PORT STM32_SPI5_PORT
#define SPI5_SCK_PIN STM32_SPI5_SCK_PIN
#define SPI5_MOSI_PIN STM32_SPI5_MOSI_PIN
#define SPI5_MISO_PIN STM32_SPI5_MISO_PIN
#endif

/* SPI6 Pin Configuration */
#if ( STM32_ENABLE_SPI56 )
#define SPI56_PORT STM32_SPI56_PORT
#define SPI56_SCK_PIN STM32_SPI56_SCK_PIN
#define SPI56_MOSI_PIN STM32_SPI56_MOSI_PIN
#define SPI56_MISO_PIN STM32_SPI56_MISO_PIN
#endif

/* Typedefs */

/**
 * SPI Instance
 */
typedef enum {
#if (STM32_ENABLE_SPI1)
  /* SPI Instance 1 */
  SPI_ONE,
#endif
#if (STM32_ENABLE_SPI2)
  /* SPI Instance 2 */
  SPI_TWO,
#endif
#if (STM32_ENABLE_SPI3)
  /* SPI Instance 3 */
  SPI_THREE,
#endif
#if (STM32_ENABLE_SPI4)
  /* SPI Instance 4 */
  SPI_FOUR,
#endif
#if (STM32_ENABLE_SPI5)
  /* SPI Instance 5 */
  SPI_FIVE,
#endif
#if (STM32_ENABLE_SPI6)
  /* SPI Instance 6 */
  SPI_SIX,
#endif

/* Max number of SPI Instances */
  MAX_SPI_INSTANCES
} SPI_Instance;

/**
 * SPI Configuration Struct
 */
typedef struct {
  /* Pointer to the instance base address */
  SPI_TypeDef *instance;
  /* SPI instance number */
  uint8_t instance_num;
  /* IRQ vector number */
  IRQn_Type vector_num;
  /* GPIO port base address */
  GPIO_TypeDef *port;
  /* SCK pin */
  uint32_t sck_pin;
  /* MISO pin */
  uint32_t miso_pin;
  /* MOSI pin */
  uint32_t mosi_pin;
  /* Alternate function */
  uint32_t alternate_func;
} stm32_spi_config;

/**
 * Stm32 SPI Bus Struct
 */
typedef struct {
  /* SPI base registered to RTEMS fielsystem */
  spi_bus base;
  /* SPI handle */
  SPI_HandleTypeDef handle;
  /* Instance number */
  SPI_Instance instance;
  /* task ID */
  rtems_id task_id;
} stm32_spi_bus;

#endif /* STM32_SPI_H*/
