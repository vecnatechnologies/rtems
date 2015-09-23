/**
 * @file hal-i2c.c
 *
 * @ingroup i2c
 *
 *
 * @brief I2C driver for the STM32xxxx series processors. Provides a
 * I2C driver that registers with cpukit/dev/i2c as a node in the
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
#include stm_header( TARGET_STM_PROCESSOR_PREFIX, i2c )
#include stm_header( TARGET_STM_PROCESSOR_PREFIX, rcc )
#include stm_header( TARGET_STM_PROCESSOR_PREFIX, gpio )

#include <hal-i2c.h>
#include <hal-i2c-init.h>

#include <math.h>
#include <rtems/irq-extension.h>

/**
 * I2C configuration struct
 */
const stm32_i2c_config i2c_config[ MAX_I2C_INSTANCES ] = {
#if (STM32_ENABLE_I2C1)
  { .instance = I2C1,
    .instance_num = I2C_ONE,
    .vector_num = I2C1_EV_IRQn,
    .address = STM32_I2C1_ADDRESS,
    .port = I2C1_PORT,
    .sck_pin = I2C1_SCK_PIN,
    .sda_pin = I2C1_SDA_PIN,
    .alternate_func = GPIO_AF4_I2C1 },
#endif
#if (STM32_ENABLE_I2C2)
  { .instance = I2C2,
    .instance_num = I2C_TWO,
    .vector_num = I2C2_EV_IRQn,
    .address = STM32_I2C2_ADDRESS,
    .port = I2C2_PORT,
    .sck_pin = I2C2_SCK_PIN,
    .sda_pin = I2C2_SDA_PIN,
    .alternate_func = GPIO_AF4_I2C2 },
#endif
#if (STM32_ENABLE_I2C3)
  { .instance = I2C3,
    .instance_num = I2C_THREE,
    .vector_num = I2C3_EV_IRQn,
    .address = STM32_I2C3_ADDRESS,
    .port = I2C3_PORT,
    .sck_pin = I2C3_SCK_PIN,
    .sda_pin = I2C3_SDA_PIN,
    .alternate_func = GPIO_AF4_I2C3 },
#endif
#if (STM32_ENABLE_I2C4)
  { .instance = I2C4,
    .instance_num = I2C_FOUR,
    .vector_num = I2C4_EV_IRQn,
    .address = STM32_I2C4_ADDRESS,
    .port = I2C4_PORT,
    .sck_pin = I2C4_SCK_PIN,
    .sda_pin = I2C4_SDA_PIN,
    .alternate_func = GPIO_AF4_I2C4 },
#endif
};

/**
 * I2C Bus control
 */
static stm32_i2c_bus *bus[ MAX_I2C_INSTANCES ];

/**
 * Initialize I2C clock
 */
static void stm32_i2c_initialize_i2c_clock( I2C_Instance i2c_instance )
{
  switch ( i2c_instance ) {
#if ( STM32_ENABLE_I2C1 )
    case I2C_ONE:
      __HAL_RCC_I2C1_CLK_ENABLE();
      break;
#endif
#if ( STM32_ENABLE_I2C2 )
    case I2C_TWO:
      __HAL_RCC_I2C2_CLK_ENABLE();
      break;
#endif
#if ( STM32_ENABLE_I2C3 )
    case I2C_THREE:
      __HAL_RCC_I2C3_CLK_ENABLE();
      break;
#endif
#if ( STM32_ENABLE_I2C4 )
    case I2C_FOUR:
      __HAL_RCC_I2C4_CLK_ENABLE();
      break;
#endif
  }
}

/**
 * I2C Interrupt Event ISR
 */
static void stm32_i2c_event_irq( void *arg )
{
  rtems_status_code sc;
  stm32_i2c_bus *bus = arg;

  HAL_I2C_EV_IRQHandler( (I2C_HandleTypeDef *) &bus->handle );

  sc = rtems_event_transient_send( bus->task_id );
  if ( sc != RTEMS_SUCCESSFUL ) {
      rtems_set_errno_and_return_minus_one( EFAULT );
    }
}

/**
 * I2C Initiate Transfer
 */
static int stm32_i2c_transfer(
  i2c_bus *base,
  i2c_msg *msgs,
  uint32_t msg_count
)
{
  rtems_status_code sc = RTEMS_SUCCESSFUL;
  stm32_i2c_bus *bus = (stm32_i2c_bus *) base;
  rtems_interval tickstart;

  i2c_msg *message = msgs->buf;

  tickstart = rtems_clock_get_ticks_since_boot();

  /* Wait for the I2C bus to be ready */
  while ( HAL_I2C_GetState( &bus->handle ) != HAL_I2C_STATE_READY ) {
    if ( ( rtems_clock_get_ticks_since_boot() - tickstart ) >
         I2C_WAIT_TIMEOUT )
      rtems_set_errno_and_return_minus_one( ETIMEDOUT );
  }

  bus->task_id = rtems_task_self();

  if ( msgs->flags == 0 ) {
    if ( HAL_I2C_Master_Transmit_IT( &bus->handle, (uint16_t) message->addr,
           (uint8_t *) message->buf, msgs->len ) != HAL_OK ) {
      rtems_set_errno_and_return_minus_one( EIO );
    }
  } else if ( msgs->flags == I2C_M_RD ) {
    if ( HAL_I2C_Master_Receive_IT( &bus->handle, (uint16_t) message->addr,
           (uint8_t *) message->buf, msgs->len ) != HAL_OK ) {
      rtems_set_errno_and_return_minus_one( EIO );
    }
  }

  sc = rtems_event_transient_receive( RTEMS_WAIT, 20 );

  if ( sc != RTEMS_SUCCESSFUL ) {
    rtems_set_errno_and_return_minus_one( ETIMEDOUT );
  }
}

/**
 * I2C Bus Destroy and De-initialise
 */
static int stm32_i2c_deinit_destroy( stm32_i2c_bus *bus )
{
  rtems_status_code sc;

  sc = rtems_interrupt_handler_remove( i2c_config[ bus->instance ].vector_num,
    stm32_i2c_event_irq,
    bus );

  _Assert( sc == RTEMS_SUCCESSFUL );

  i2c_bus_destroy_and_free( &bus->base );

  return ( HAL_I2C_DeInit( &bus->handle ) );
}

/**
 * Initialize GPIO pins
 */
void stm32_i2c_gpio_init( I2C_Instance i2c_instance )
{
  GPIO_InitTypeDef GPIO_InitStruct;

  stm32_i2c_initialize_i2c_clock( i2c_instance );

  /* Initialize GPIO Clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = i2c_config[ i2c_instance ].alternate_func;

  /* I2C SCK GPIO pin configuration  */
  GPIO_InitStruct.Pin = i2c_config[ i2c_instance ].sck_pin;
  HAL_GPIO_Init( i2c_config[ i2c_instance ].port, &GPIO_InitStruct );

  /* I2C SDA GPIO pin configuration  */
  GPIO_InitStruct.Pin = i2c_config[ i2c_instance ].sda_pin;
  HAL_GPIO_Init( i2c_config[ i2c_instance ].port, &GPIO_InitStruct );
}

/* Register I2C Driver to RTEMS */
int stm32_bsp_register_i2c( void )
{
  rtems_status_code sc;
  int err, inst_num = 0;
  char bus_path[ MAX_PATH_CHAR ];

  for ( inst_num = 0; inst_num < MAX_I2C_INSTANCES; inst_num++ ) {
    bus[ inst_num ] =
      (stm32_i2c_bus *) i2c_bus_alloc_and_init( sizeof( *bus[ inst_num ] ) );

    if ( bus[ inst_num ] == NULL ) {
      rtems_set_errno_and_return_minus_one( ENOMEM );
    }

    bus[ inst_num ]->instance = (I2C_Instance) inst_num;

    /* Initialize the GPIO */
    stm32_i2c_gpio_init( inst_num );

    /* Initialize I2C instance */
    if ( stm32_i2c_init( bus[ inst_num ] ) != HAL_OK ) {
      ( *bus[ inst_num ]->base.destroy )( &bus[ inst_num ]->base );
      rtems_set_errno_and_return_minus_one( EIO );
    }

    /* Install I2C vectors */
    sc = rtems_interrupt_handler_install(
      i2c_config[ inst_num ].vector_num,
      "I2C Event Interrupt",
      RTEMS_INTERRUPT_UNIQUE,
      stm32_i2c_event_irq,
      bus[ inst_num ]
         );

    if ( sc != RTEMS_SUCCESSFUL ) {
      ( *bus[ inst_num ]->base.destroy )( &bus[ inst_num ]->base );
      rtems_set_errno_and_return_minus_one( EIO );
    }

    bus[ inst_num ]->base.transfer = stm32_i2c_transfer;
    bus[ inst_num ]->base.destroy = stm32_i2c_deinit_destroy;
    bus[ inst_num ]->base.set_clock = stm32_i2c_set_clock;

    sprintf( bus_path, "/dev/i2c%d", i2c_config[ inst_num ].instance_num + 1 );

    err = i2c_bus_register( &bus[ inst_num ]->base, bus_path );
  }

  return err;
}
