/**
 * @file hal-gpio.c
 *
 * @ingroup gpio
 *
 * @brief GPIO support for STM32F series
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

/* ======================= STMF32 Support Functions ========================= */
#include <rtems.h>
#include <hal-utils.h>
#include <stm32f-processor-specific.h>
#include <bspopts.h>
#include <bsp.h>

#include stm_processor_header( TARGET_STM_PROCESSOR_PREFIX )
#include stm_header( TARGET_STM_PROCESSOR_PREFIX, rcc )
#include stm_header( TARGET_STM_PROCESSOR_PREFIX, gpio )

#include <rtems/irq-extension.h>
#include <hal-gpio.h>

/**
 * @brief GPIO Pins
 */
typedef enum {
  /* Pin 0 */
  PIN0 = 0,
  /* Pin 1 */
  PIN1,
  /* Pin 2 */
  PIN2,
  /* Pin 3 */
  PIN3,
  /* Pin 4 */
  PIN4,
  /* Pin 5 */
  PIN5,
  /* Pin 6 */
  PIN6,
  /* Pin 7 */
  PIN7,
  /* Pin 8 */
  PIN8,
  /* Pin 9 */
  PIN9,
  /* Pin 10 */
  PIN10,
  /* Pin 11 */
  PIN11,
  /* Pin 12 */
  PIN12,
  /* Pin 13 */
  PIN13,
  /* Pin 14 */
  PIN14,
  /* Pin 15 */
  PIN15,

  /* Max number of pins */
  MAX_GPIO_PINS
} stm32_gpio_pin;

/**
 * @brief GPIO Ports
 */
typedef enum {
  /* PORT A */
  PORTA = 0,
  /* PORT B */
  PORTB,
  /* PORT C */
  PORTC,
  /* PORT D */
  PORTD,
  /* PORT E */
  PORTE,
  /* PORT F */
  PORTF,
  /* PORT G */
  PORTG,
  /* PORT H */
  PORTH,
  /* PORT I */
  PORTI,

  /* Max GPIO Ports */
  MAX_GPIO_PORTS
} stm32_gpio_ports;

/**
 * @brief GPIO Struct
 */
typedef struct {
  /* GPIO Ports */
  stm32_gpio_ports port;
  /* GPIO Pin Struct */
  stm32_gpio_pin pin;
} stm32_gpio;

/**
 * @brief Returns the base address of the GPIO port
 */
static stm32_gpio stm32_gpio_arg;

/**
 * @brief Returns the base address of the GPIO port
 */
static rtems_vector_number stm32_gpio_bank_vector[ MAX_GPIO_PORTS ] = { 0 };

/**
 * @brief Status of enabled clock in each GPIO port
 */
static bool stm32_gpio_clock_status[ MAX_GPIO_PORTS ] = { false };

/** Private Function Definitions **/

/**
 * @brief Returns the base address of the GPIO port
 */
static GPIO_TypeDef *stm32_gpio_get_port_base( stm32_gpio_ports bank )
{
  switch ( bank ) {
    case PORTA: return ( GPIOA );
    case PORTB: return ( GPIOB );
    case PORTC: return ( GPIOC );
    case PORTD: return ( GPIOD );
    case PORTE: return ( GPIOE );
    case PORTF: return ( GPIOE );
    case PORTG: return ( GPIOE );
    case PORTH: return ( GPIOH );
    case PORTI: return ( GPIOI );
  }
}

/**
 * @brief Enable the GPIO clock base on port
 */
static void stm32_gpio_clock_enable( stm32_gpio_ports bank )
{
  switch ( bank ) {
    case PORTA:
      __HAL_RCC_GPIOA_CLK_ENABLE();
      break;
    case PORTB:
      __HAL_RCC_GPIOB_CLK_ENABLE();
      break;
    case PORTC:
      __HAL_RCC_GPIOC_CLK_ENABLE();
      break;
    case PORTD:
      __HAL_RCC_GPIOD_CLK_ENABLE();
      break;
    case PORTE:
      __HAL_RCC_GPIOE_CLK_ENABLE();
      break;
    case PORTF:
      __HAL_RCC_GPIOF_CLK_ENABLE();
      break;
    case PORTG:
      __HAL_RCC_GPIOG_CLK_ENABLE();
      break;
    case PORTH:
      __HAL_RCC_GPIOH_CLK_ENABLE();
      break;
    case PORTI:
      __HAL_RCC_GPIOI_CLK_ENABLE();
      break;
  }
}

/**
 * @brief Returns the base address of the GPIO pin
 */
static inline uint32_t stm32_gpio_get_pin_base( stm32_gpio_pin pin )
{
  return ( GPIO_PIN_0 << pin );
}

/**
 * @brief GPIO EXTI Line 0 Interrupt HAndler
 */
static void stm32_gpio_exti_irq( stm32_gpio *arg )
{
  stm32_gpio stm32_gpio_arg = *arg;

  /* Service this interrupt - clear interrupt */
  HAL_GPIO_EXTI_IRQHandler( stm32_gpio_get_pin_base( stm32_gpio_arg.pin ) );

  /* Call application IRQ function - to be called in user space */
  stm32_gpio_exti_callback( stm32_gpio_arg );
}

/**
 *  @brief Called in user space - Application callback function for GPIO interrupt
 */
__weak void stm32_gpio_exti_callback( stm32_gpio stm32_gpio_arg )
{
}

/** GPIO Generic API Functions from gpio.c**/

rtems_status_code rtems_gpio_bsp_select_input(
  uint32_t bank,
  uint32_t pin,
  void    *bsp_specific
)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* Enable GPIO clock if not enabled */
  if ( !( stm32_gpio_clock_status[ bank ] ) ) {
    stm32_gpio_clock_enable( bank );
    stm32_gpio_clock_status[ bank ] = true;
  }

  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Pin = stm32_gpio_get_pin_base( pin );
  GPIO_InitStruct.Alternate = bsp_specific;

  /* Initialize the GPIO pin */
  HAL_GPIO_Init( stm32_gpio_get_port_base( bank ), &GPIO_InitStruct );

  return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_bsp_select_output(
  uint32_t bank,
  uint32_t pin,
  void    *bsp_specific
)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* Enable GPIO clock if not enabled */
  if ( !( stm32_gpio_clock_status[ bank ] ) ) {
    stm32_gpio_clock_enable( bank );
    stm32_gpio_clock_status[ bank ] = true;
  }

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Pin = stm32_gpio_get_pin_base( pin );
  //GPIO_InitStruct.Alternate = bsp_specific;

  /* Initialize the GPIO pin */
  HAL_GPIO_Init( stm32_gpio_get_port_base( bank ), &GPIO_InitStruct );

  return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_bsp_select_specific_io(
  uint32_t bank,
  uint32_t pin,
  uint32_t function,
  void    *pin_data
)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* Enable GPIO clock if not enabled */
  if ( !( stm32_gpio_clock_status[ bank ] ) ) {
    stm32_gpio_clock_enable( bank );
    stm32_gpio_clock_status[ bank ] = true;
  }

  GPIO_InitStruct.Mode = function;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Pin = stm32_gpio_get_pin_base( pin );
  //GPIO_InitStruct.Alternate = bsp_specific;

  /* Initialize the GPIO pin */
  HAL_GPIO_Init( stm32_gpio_get_port_base( bank ), &GPIO_InitStruct );

  return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_bsp_multi_set(
  uint32_t bank,
  uint32_t bitmask
)
{
  uint8_t pin_count;

  bitmask = GPIO_PIN_MASK & bitmask;

  for ( pin_count = 15; pin_count > 0; pin_count -= 1 ) {
    if ( ( GPIO_PIN_15 & bitmask ) == GPIO_PIN_15 ) {
      HAL_GPIO_WritePin( stm32_gpio_get_port_base( bank ),
          stm32_gpio_get_pin_base( pin_count + 1 ), GPIO_PIN_SET );
      bitmask << 1;
    } else
      continue;
  }

  return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_bsp_multi_clear(
  uint32_t bank,
  uint32_t bitmask
)
{
  uint8_t pin_count;

  bitmask = GPIO_PIN_MASK & bitmask;

  for ( pin_count = 15; pin_count > 0; pin_count -= 1 ) {
    if ( ( GPIO_PIN_15 & bitmask ) == GPIO_PIN_15 ) {
      HAL_GPIO_WritePin( stm32_gpio_get_port_base( bank ),
          stm32_gpio_get_pin_base( pin_count + 1 ), GPIO_PIN_RESET );
      bitmask << 1;
    } else
      continue;
  }

  return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_bsp_set(
  uint32_t bank,
  uint32_t pin
)
{
  HAL_GPIO_WritePin( stm32_gpio_get_port_base( bank ),
    stm32_gpio_get_pin_base( pin ), GPIO_PIN_SET );

  return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_bsp_clear(
  uint32_t bank,
  uint32_t pin
)
{
  HAL_GPIO_WritePin( stm32_gpio_get_port_base( bank ),
    stm32_gpio_get_pin_base( pin ), GPIO_PIN_RESET );

  return RTEMS_SUCCESSFUL;
}

uint32_t rtems_gpio_bsp_get_value(
  uint32_t bank,
  uint32_t pin
)
{
  return ( HAL_GPIO_ReadPin( stm32_gpio_get_port_base( bank ),
             stm32_gpio_get_pin_base( pin ) ) );
}

rtems_status_code rtems_bsp_enable_interrupt(
  uint32_t             bank,
  uint32_t             pin,
  rtems_gpio_interrupt interrupt
)
{
  rtems_status_code sc;

  stm32_gpio_arg.pin = pin;
  stm32_gpio_arg.port = bank;

  /* Install GPIO vector */
  sc = rtems_interrupt_handler_install(
    interrupt,
    "GPIO EXTI Line 0 Interrupt",
    RTEMS_INTERRUPT_UNIQUE,
    stm32_gpio_exti_irq,
    &stm32_gpio_arg
       );

  if ( RTEMS_SUCCESSFUL == sc ) {
    stm32_gpio_bank_vector[ bank ] = interrupt;
  }

  return sc;
}

rtems_status_code rtems_bsp_disable_interrupt(
  uint32_t             bank,
  uint32_t             pin,
  rtems_gpio_interrupt interrupt
)
{
  rtems_status_code sc;

  sc = rtems_interrupt_handler_remove(
    interrupt,
    stm32_gpio_exti_irq,
    stm32_gpio_get_pin_base( pin )
       );

  return sc;
}

rtems_vector_number rtems_gpio_bsp_get_vector( uint32_t bank )
{
  return ( stm32_gpio_bank_vector[ bank ] );
}

rtems_status_code rtems_gpio_bsp_set_resistor_mode(
  uint32_t             bank,
  uint32_t             pin,
  rtems_gpio_pull_mode mode
)
{

  /* TODO: Currently not supported. Should figure out a way to
   * get all the pin config status and perform De-init - change reg mode - Init
   */
  return RTEMS_NOT_DEFINED;
}

uint32_t rtems_gpio_bsp_interrupt_line( rtems_vector_number vector )
{
  return RTEMS_NOT_DEFINED;
}

uint32_t rtems_gpio_bsp_multi_read(
  uint32_t bank,
  uint32_t bitmask
)
{
  return RTEMS_NOT_DEFINED;
}

rtems_status_code rtems_gpio_bsp_multi_select(
  rtems_gpio_multiple_pin_select *pins,
  uint32_t                        pin_count,
  uint32_t                        select_bank
)
{
  return RTEMS_NOT_DEFINED;
}

rtems_status_code rtems_gpio_bsp_specific_group_operation(
  uint32_t  bank,
  uint32_t *pins,
  uint32_t  pin_count,
  void     *arg
)
{
  return RTEMS_NOT_DEFINED;
}

