/**
 * @file hal-spi.c
 *
 * @ingroup spi
 *
 * @brief SPI driver for the STM32xxxx series processors. Provides a
 * SPI driver that registers with cpukit/dev/spi as a node in the
 * filesystem.
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
#include stm_header( TARGET_STM_PROCESSOR_PREFIX, spi )
#include stm_header( TARGET_STM_PROCESSOR_PREFIX, rcc )
#include stm_header( TARGET_STM_PROCESSOR_PREFIX, gpio )

#include <hal-spi.h>
#include <dev/spi/spi.h>

#include <math.h>
#include <rtems/irq-extension.h>

/**
 * SPI pin configuration for each instances.
 * Pin and port details to be populated in configure.ac file
 */
static const stm32_spi_config spi_config[ MAX_SPI_INSTANCES ] = {
#if (STM32_ENABLE_SPI1)
  { .instance = SPI1,
    .instance_num = SPI_ONE,
    .vector_num = SPI1_IRQn,
    .port = SPI1_PORT,
    .sck_pin = SPI1_SCK_PIN,
    .miso_pin = SPI1_MISO_PIN,
    .mosi_pin = SPI1_MOSI_PIN,
    .alternate_func = GPIO_AF5_SPI1 },
#endif
#if (STM32_ENABLE_SPI2)
  { .instance = SPI2,
    .instance_num = SPI_TWO,
    .vector_num = SPI2_IRQn,
    .port = SPI2_PORT,
    .sck_pin = SPI2_SCK_PIN,
    .miso_pin = SPI2_MISO_PIN,
    .mosi_pin = SPI2_MOSI_PIN,
    .alternate_func = GPIO_AF5_SPI2 },
#endif
#if (STM32_ENABLE_SPI3)
  { .instance = SPI3,
    .instance_num = SPI_THREE,
    .vector_num = SPI3_IRQn,
    .port = SPI3_PORT,
    .sck_pin = SPI3_SCK_PIN,
    .miso_pin = SPI3_MISO_PIN,
    .mosi_pin = SPI3_MOSI_PIN,
    .alternate_func = GPIO_AF5_SPI3 },
#endif
#if (STM32_ENABLE_SPI4)
  { .instance = SPI4,
    .instance_num = SPI_FOUR,
    .vector_num = SPI4_IRQn,
    .port = SPI4_PORT,
    .sck_pin = SPI4_SCK_PIN,
    .miso_pin = SPI4_MISO_PIN,
    .mosi_pin = SPI4_MOSI_PIN,
    .alternate_func = GPIO_AF5_SPI4 },
#endif
#if (STM32_ENABLE_SPI5)
  { .instance = SPI5,
    .instance_num = SPI_FIVE,
    .vector_num = SPI5_IRQn,
    .port = SPI5_PORT,
    .sck_pin = SPI5_SCK_PIN,
    .miso_pin = SPI5_MISO_PIN,
    .mosi_pin = SPI5_MOSI_PIN,
    .alternate_func = GPIO_AF5_SPI5 },
#endif
#if (STM32_ENABLE_SPI6)
  { .instance = SPI6,
    .instance_num = SPI_THREE,
    .vector_num = SPI6_IRQn,
    .port = SPI6_PORT,
    .sck_pin = SPI6_SCK_PIN,
    .miso_pin = SPI6_MISO_PIN,
    .mosi_pin = SPI6_MOSI_PIN,
    .alternate_func = GPIO_AF5_SPI6 },
#endif
};

/**
 * SPI Bus control
 */
static stm32_spi_bus *bus[ MAX_SPI_INSTANCES ];

/**
 * Returns the base address of the port
 * TODO:: this function is to be pushed to gpio file and used from there
 */
static GPIO_TypeDef *stm32_spi_get_gpio_port( gpio_port port )
{
  switch ( port ) {
    case PORTA:
      return GPIOA;
    case PORTB:
      return GPIOB;
    case PORTC:
      return GPIOC;
    case PORTD:
      return GPIOD;
    case PORTE:
      return GPIOE;
  }
}

/**
 * Initialize port based clock
 */
static void stm32_spi_initialize_gpio_clock( SPI_Instance spi_instance )
{
  switch ( (uint32_t) spi_config[ spi_instance ].port ) {
    case (uint32_t)GPIOA:
      __HAL_RCC_GPIOA_CLK_ENABLE();
      break;
    case (uint32_t)GPIOB:
      __HAL_RCC_GPIOB_CLK_ENABLE();
      break;
    case (uint32_t)GPIOC:
      __HAL_RCC_GPIOC_CLK_ENABLE();
      break;
    case (uint32_t)GPIOD:
      __HAL_RCC_GPIOD_CLK_ENABLE();
      break;
    case (uint32_t)GPIOE:
      __HAL_RCC_GPIOE_CLK_ENABLE();
      break;
    case (uint32_t)GPIOH:
      __HAL_RCC_GPIOH_CLK_ENABLE();
      break;
  }
}

/**
 * Initialize SPI clock
 */
static void stm32_spi_initialize_spi_clock( SPI_Instance spi_instance )
{
  switch ( spi_instance ) {
#if ( STM32_ENABLE_SPI1 )
    case SPI_ONE:
      __HAL_RCC_SPI1_CLK_ENABLE();
      break;
#endif
#if ( STM32_ENABLE_SPI2 )
    case SPI_TWO:
      __HAL_RCC_SPI2_CLK_ENABLE();
      break;
#endif
#if ( STM32_ENABLE_SPI3 )
    case SPI_THREE:
      __HAL_RCC_SPI3_CLK_ENABLE();
      break;
#endif
#if ( STM32_ENABLE_SPI4 )
    case SPI_FOUR:
      __HAL_RCC_SPI4_CLK_ENABLE();
      break;
#endif
#if ( STM32_ENABLE_SPI5 )
    case SPI_FIVE:
      __HAL_RCC_SPI5_CLK_ENABLE();
      break;
#endif
#if ( STM32_ENABLE_SPI6 )
    case SPI_SIX:
      __HAL_RCC_SPI6_CLK_ENABLE();
      break;
#endif
  }
}

/**
 * SPI IOCTL
 */
static int stm32_spi_ioctl(
  spi_bus *base,
  uint32_t ioctl_cmd,
  unsigned long arg
)
{
  _Assert( base != NULL );

  stm32_spi_bus *bus = (stm32_spi_bus *) base;

  switch ( ioctl_cmd ) {
    /* Two line half-duplex mode */
    case SPI_IOCTL_DIR_2LINE:
      __HAL_SPI_DISABLE( bus->handle );
      bus->handle->Instance->CR1 &= ~( SPI_CR1_BIDIMODE );
      __HAL_SPI_ENABLE( bus->handle );
      return 0;

    /* bi-direction line full-duplex mode */
    case SPI_IOCTL_DIR_1LINE:
      __HAL_SPI_DISABLE( bus->handle );
      bus->handle->Instance->CR1 |= SPI_CR1_BIDIMODE;
      __HAL_SPI_ENABLE( bus->handle );
      return 0;

    /* Configure SPI instance to Master mode */
    case SPI_IOCTL_MODE_MASTER:
      __HAL_SPI_DISABLE( bus->handle );
      bus->handle->Instance->CR1 |= ( SPI_CR1_MSTR | SPI_CR1_SSI );
      __HAL_SPI_ENABLE( bus->handle );
      return 0;

    /* Configure SPI instance to Slave mode */
    case SPI_IOCTL_MODE_SLAVE:
      __HAL_SPI_DISABLE( bus->handle );
      bus->handle->Instance->CR1 &= ~( SPI_CR1_MSTR | SPI_CR1_SSI );
      __HAL_SPI_ENABLE( bus->handle );
      return 0;

    /* Enable CRC Calculation */
    case SPI_IOCTL_ENABLE_CRC:
      __HAL_SPI_DISABLE( bus->handle );
      bus->handle->Instance->CR1 |= SPI_CR1_CRCEN;
      __HAL_SPI_ENABLE( bus->handle );
      return 0;

    /* Disable CRC Calculation */
    case SPI_IOCTL_DISABLE_CRC:
      __HAL_SPI_DISABLE( bus->handle );
      bus->handle->Instance->CR1 &= ~( SPI_CR1_CRCEN );
      __HAL_SPI_ENABLE( bus->handle );
      return 0;

    default:

      rtems_set_errno_and_return_minus_one( ENOTTY );
  }
}

/**
 * SPI Interrupt Event ISR
 */
static void stm32_spi_event_irq( void *arg )
{
  rtems_status_code sc;
  stm32_spi_bus * bus = arg;

  HAL_SPI_IRQHandler( (SPI_HandleTypeDef *) &(bus->handle) );

  sc = rtems_event_transient_send( bus->task_id );
  if ( sc != RTEMS_SUCCESSFUL ) {
    rtems_set_errno_and_return_minus_one( EFAULT );
  }
}

/**
 * SPI Initiate Transfer
 */
static int stm32_spi_transfer(
  spi_bus *base,
  spi_msg *msgs,
  uint32_t msg_count
)
{
  rtems_status_code sc = RTEMS_SUCCESSFUL;
  stm32_spi_bus    *bus = (stm32_spi_bus *) base;
  rtems_interval    tickstart;

  _Assert( base != NULL );

  spi_msg *message = msgs;

  tickstart = rtems_clock_get_ticks_since_boot();

  /* Wait for the SPI bus to be ready */
  while ( HAL_SPI_GetState( (SPI_HandleTypeDef *) &bus->handle ) != HAL_SPI_STATE_READY ) {
    if ( ( rtems_clock_get_ticks_since_boot() - tickstart ) >
         SPI_WAIT_TIMEOUT )
      rtems_set_errno_and_return_minus_one( ETIMEDOUT );
  }

  /* Get the task id */
  bus->task_id = rtems_task_self();

  /* Hold CS Pin Low to activate Slave */
  HAL_GPIO_WritePin( stm32_spi_get_gpio_port(
      message->slave_select.port ), GET_PIN(
      message->slave_select.pin ), GPIO_PIN_RESET );

  /* Only Write */
  if ( msgs->flags == SPI_M_WR ) {
    if ( HAL_SPI_Transmit_IT( &bus->handle, (uint8_t *) message->pTxBuf,
           message->len ) != HAL_OK ) {
      rtems_set_errno_and_return_minus_one( EIO );
    }
  }
  /* Only Read */
  else if ( msgs->flags == SPI_M_RD ) {
    if ( HAL_SPI_Receive_IT( &bus->handle, (uint8_t *) message->pRxBuf,
           message->len ) != HAL_OK ) {
      rtems_set_errno_and_return_minus_one( EIO );
    }
  }
  /* Write followed by read */
  else if ( msgs->flags == SPI_M_WR_RD ) {
    if ( HAL_SPI_TransmitReceive_IT( &bus->handle, (uint8_t *) message->pTxBuf,
           (uint8_t *) message->pRxBuf, message->len ) != HAL_OK ) {
      rtems_set_errno_and_return_minus_one( EIO );
    }
  }

  /* Hold CS Pin High to de-activate Slave */
  HAL_GPIO_WritePin( stm32_spi_get_gpio_port(
      message->slave_select.port ), GET_PIN(
      message->slave_select.pin ), GPIO_PIN_SET );

  sc = rtems_event_transient_receive( RTEMS_WAIT, 20 );

  if ( sc != RTEMS_SUCCESSFUL ) {
    rtems_set_errno_and_return_minus_one( ETIMEDOUT );
  }
}

/**
 * Initalize GPIO Pins
 * Note: If software based slave select, gpio pin for the slave select pin
 *       has to be configured and initialized by the application.
 */
static void stm32_spi_gpio_init( SPI_Instance spi_instance )
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* Initalize SPI Clock */
  stm32_spi_initialize_spi_clock( spi_instance );

  /* Initalize GPIO Clock */
  stm32_spi_initialize_gpio_clock( spi_instance );

  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = spi_config[ spi_instance ].alternate_func;

  /* SPI SCK Pin */
  GPIO_InitStruct.Pin = spi_config[ spi_instance ].sck_pin;
  HAL_GPIO_Init( spi_config[ spi_instance ].port, &GPIO_InitStruct );

  /* SPI MOSI Pin */
  GPIO_InitStruct.Pin = spi_config[ spi_instance ].mosi_pin;
  HAL_GPIO_Init( spi_config[ spi_instance ].port, &GPIO_InitStruct );

  /* SPI MISO Pin */
  GPIO_InitStruct.Pin = spi_config[ spi_instance ].miso_pin;
  HAL_GPIO_Init( spi_config[ spi_instance ].port, &GPIO_InitStruct );
}

/**
 * SPI Initalize Instance
 */
static int stm32_spi_init( stm32_spi_bus *bus )
{
  rtems_status_code sc;

  _Assert( bus != NULL );

  /* Initialize GPIO pins for SPI */
  stm32_spi_gpio_init( bus->instance );

  /* Initialize SPI */
  /* Set the SPI parameters */
  bus->handle->Instance = spi_config[ bus->instance ].instance;

  bus->handle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  bus->handle->Init.Direction = SPI_DIRECTION_1LINE;
  bus->handle->Init.CLKPhase = SPI_PHASE_1EDGE;
  bus->handle->Init.CLKPolarity = SPI_POLARITY_LOW;
  bus->handle->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  bus->handle->Init.CRCPolynomial = 7;
  bus->handle->Init.DataSize = SPI_DATASIZE_16BIT;
  bus->handle->Init.FirstBit = SPI_FIRSTBIT_MSB;
  bus->handle->Init.NSS = SPI_NSS_SOFT;
  bus->handle->Init.TIMode = SPI_TIMODE_DISABLE;

  bus->handle->Init.Mode = SPI_MODE_MASTER;

  if ( HAL_SPI_Init( &bus->handle ) != HAL_OK ) {
    ( *bus->base.destroy )( &bus->base );
    rtems_set_errno_and_return_minus_one( EIO );
  }

  /* Install SPI vectors */
  sc = rtems_interrupt_handler_install(
    spi_config[ bus->instance ].vector_num,
    "SPI Event Interrupt",
    RTEMS_INTERRUPT_UNIQUE,
    stm32_spi_event_irq,
    bus
       );

  if ( sc != RTEMS_SUCCESSFUL ) {
    ( *bus->base.destroy )( &bus->base );
    rtems_set_errno_and_return_minus_one( EIO );
  }

  return 0;
}

/**
 * spi Bus Destroy and De-initialise
 */
static int stm32_spi_deinit_destroy( stm32_spi_bus *bus )
{
  rtems_status_code sc;

  _Assert( bus != NULL );

  sc = rtems_interrupt_handler_remove( spi_config[ bus->instance ].vector_num,
    stm32_spi_event_irq,
    bus );
  _Assert( sc == RTEMS_SUCCESSFUL );
  (void) sc;

  spi_bus_destroy_and_free( &bus->base );

  return ( HAL_SPI_DeInit( &bus->handle ) );
}

/* Register SPI Driver to RTEMS */
int stm32_bsp_register_spi( void )
{
  rtems_status_code sc;
  int err = 0;
  int inst_num = 0;
  char bus_path[ MAX_PATH_CHAR ];

  for ( inst_num = 0; inst_num < MAX_SPI_INSTANCES; inst_num++ ) {
    bus[ inst_num ] =
      (stm32_spi_bus *) spi_bus_alloc_and_init( sizeof( *bus[ inst_num ] ) );

    if ( bus[ inst_num ] == NULL ) {
      rtems_set_errno_and_return_minus_one( ENOMEM );
    }

    bus[ inst_num ]->base.init = stm32_spi_init;
    bus[ inst_num ]->base.transfer = stm32_spi_transfer;
    bus[ inst_num ]->base.destroy = stm32_spi_deinit_destroy;
    bus[ inst_num ]->base.de_init = stm32_spi_deinit_destroy;
    bus[ inst_num ]->base.ioctl = stm32_spi_ioctl;

    bus[ inst_num ]->instance = (SPI_Instance) inst_num;

    sprintf( bus_path, "/dev/spi%d", spi_config[ inst_num ].instance_num + 1 );

    err = spi_bus_register( &bus[ inst_num ]->base, bus_path );
  }

  return err;
}
