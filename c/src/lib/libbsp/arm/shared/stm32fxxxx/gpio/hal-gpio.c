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
#include <time.h>
#include <math.h>
#include <bsp.h>

#include stm_processor_header( TARGET_STM_PROCESSOR_PREFIX )
#include stm_header( TARGET_STM_PROCESSOR_PREFIX, rcc )
#include stm_header( TARGET_STM_PROCESSOR_PREFIX, gpio )

#include <rtems/irq-extension.h>
#include <hal-gpio.h>

/**
 * @brief Maximum number of pins per port
 */
#define MAX_GPIO_PINS_PER_PORT BSP_GPIO_PINS_PER_BANK

/**
 * @brief Maximum number of port
 */
#define MAX_GPIO_PORTS ( BSP_GPIO_PIN_COUNT / BSP_GPIO_PINS_PER_BANK )

/**
 * @brief GPIO base address offest
 */
#define GPIO_BASE_ADDR_OFFSET 0x0400

/**
 * @brief GPIO Pin
 */
typedef uint8_t stm32_gpio_pin;

/**
 * @brief GPIO Port
 */
typedef uint8_t stm32_gpio_ports;

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
 * @brief Array to store the GPIO port vector numbers.
 *
 * TODO :: This array has to be filled with vector numbers for each bank.
 *         RTEMS GPIO driver uses this array to get the bank specific vectors to
 *         install the interrupt. ST doesn't support bank specific vector
 *         interrupts, they support pin based. This has to be sorted out.
 *         Right now, RTEMS will not register any interrupts for the GPIO.
 */
static rtems_vector_number stm32_gpio_bank_vector[ MAX_GPIO_PORTS ] = { -1 };

/**
 * @brief Status of enabled clock in each GPIO port
 */
static bool stm32_gpio_clock_status[ MAX_GPIO_PORTS ] = { CLOCK_DISABLED };

/** Private Function Definitions **/

/**
 * @brief Returns the base address of the GPIO port
 */
static inline uint32_t stm32_gpio_get_port_base( stm32_gpio_ports bank )
{

  /**
   * Each base address of GPIO port is offset by GPIO_BASE_ADDR_OFFSET from
   * the AHB1PERIPH_BASE address.
   */
  return ( AHB1PERIPH_BASE + ( GPIO_BASE_ADDR_OFFSET * bank ) );
}

/**
 * @brief Enable the GPIO clock based on port
 */
static inline void stm32_gpio_clock_enable( stm32_gpio_ports bank )
{
  RCC->AHB1ENR |= ( RCC_AHB1ENR_GPIOAEN << bank );
}

/**
 * @brief Returns the bit mask of the GPIO pin
 */
static inline uint32_t stm32_gpio_get_pin_bitmask( stm32_gpio_pin pin )
{
  return ( GPIO_PIN_0 << pin );
}

/**
 * @brief Disable GPIO External Interrupt or Event for the defiend pin
 */
static void stm32_gpio_disable_exti( stm32_gpio_pin pin )
{
  uint32_t temp_exti;

  /* Disable the External Interrupt or event for the current IO */
  temp_exti = ( (uint32_t) 0x0F ) << ( 4 * ( pin & 0x03 ) );
  SYSCFG->EXTICR[ pin >> 2 ] &= ~temp_exti;

  /* Clear EXTI line configuration */
  EXTI->IMR &= ~( (uint32_t) stm32_gpio_get_pin_bitmask( pin ) );
  EXTI->EMR &= ~( (uint32_t) stm32_gpio_get_pin_bitmask( pin ) );

  /* Clear Rising Falling edge configuration */
  EXTI->RTSR &= ~( (uint32_t) stm32_gpio_get_pin_bitmask( pin ) );
  EXTI->FTSR &= ~( (uint32_t) stm32_gpio_get_pin_bitmask( pin ) );
}

/** GPIO Generic API Functions from gpio.c **/

rtems_status_code rtems_gpio_bsp_select_input(
  uint32_t bank,
  uint32_t pin,
  void    *bsp_specific
)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  if ( NULL == bsp_specific ) {
    return RTEMS_INVALID_ID;
  }

  stm32_gpio_pin_config *pin_config = (stm32_gpio_pin_config *) bsp_specific;

  /* Enable GPIO clock if not enabled */
  if ( !( stm32_gpio_clock_status[ bank ] ) ) {
    stm32_gpio_clock_enable( bank );
    stm32_gpio_clock_status[ bank ] = CLOCK_ENABLED;
  }

  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = pin_config->pull;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Pin = stm32_gpio_get_pin_bitmask( pin );
  GPIO_InitStruct.Alternate = NULL;

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

  if ( NULL == bsp_specific ) {
    return RTEMS_INVALID_ID;
  }

  stm32_gpio_pin_config *pin_config = (stm32_gpio_pin_config *) bsp_specific;

  if (
    ( pin_config->mode != GPIO_MODE_OUTPUT_PP ) &&
    ( pin_config->mode != GPIO_MODE_OUTPUT_OD )
  ) {
    return RTEMS_INVALID_ID;
  }

  /* Enable GPIO clock if not enabled */
  if ( !( stm32_gpio_clock_status[ bank ] ) ) {
    stm32_gpio_clock_enable( bank );
    stm32_gpio_clock_status[ bank ] = CLOCK_ENABLED;
  }

  GPIO_InitStruct.Mode = pin_config->mode;
  GPIO_InitStruct.Pull = pin_config->pull;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Pin = stm32_gpio_get_pin_bitmask( pin );
  GPIO_InitStruct.Alternate = NULL;

  /* Initialize the GPIO pin */
  HAL_GPIO_Init( stm32_gpio_get_port_base( bank ), &GPIO_InitStruct );

  return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_bsp_select_specific_io(
  uint32_t bank,
  uint32_t pin,
  uint32_t function,
  void    *pin_data
)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  if ( NULL == pin_data ) {
    return RTEMS_INVALID_ID;
  }

  stm32_gpio_pin_config *pin_config = (stm32_gpio_pin_config *) pin_data;

  /* Enable GPIO clock if not enabled */
  if ( !( stm32_gpio_clock_status[ bank ] ) ) {
    stm32_gpio_clock_enable( bank );
    stm32_gpio_clock_status[ bank ] = CLOCK_ENABLED;
  }

  GPIO_InitStruct.Mode = pin_config->mode;
  GPIO_InitStruct.Pull = pin_config->pull;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Pin = stm32_gpio_get_pin_bitmask( pin );
  GPIO_InitStruct.Alternate = pin_config->alternate;

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

  for ( pin_count = 0; pin_count < MAX_GPIO_PINS_PER_PORT; pin_count++ ) {
    if ( ( GPIO_PIN_0 << pin_count ) & bitmask )
      HAL_GPIO_WritePin( stm32_gpio_get_port_base( bank ),
        stm32_gpio_get_pin_bitmask( pin_count ), GPIO_PIN_SET );
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

  for ( pin_count = 0; pin_count < MAX_GPIO_PINS_PER_PORT; pin_count++ ) {
    if ( ( GPIO_PIN_0 << pin_count ) & bitmask )
      HAL_GPIO_WritePin( stm32_gpio_get_port_base( bank ),
        stm32_gpio_get_pin_bitmask( pin_count ), GPIO_PIN_RESET );
  }

  return RTEMS_SUCCESSFUL;
}

uint32_t rtems_gpio_bsp_multi_read(
  uint32_t bank,
  uint32_t bitmask
)
{
  GPIO_TypeDef *gpio_port_base;

  gpio_port_base = stm32_gpio_get_port_base( bank );

  return ( ( gpio_port_base->IDR ) & bitmask );
}

rtems_status_code rtems_gpio_bsp_set(
  uint32_t bank,
  uint32_t pin
)
{
  HAL_GPIO_WritePin( stm32_gpio_get_port_base( bank ),
    stm32_gpio_get_pin_bitmask( pin ), GPIO_PIN_SET );

  return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_bsp_clear(
  uint32_t bank,
  uint32_t pin
)
{
  HAL_GPIO_WritePin( stm32_gpio_get_port_base( bank ),
    stm32_gpio_get_pin_bitmask( pin ), GPIO_PIN_RESET );

  return RTEMS_SUCCESSFUL;
}

uint32_t rtems_gpio_bsp_get_value(
  uint32_t bank,
  uint32_t pin
)
{
  return ( HAL_GPIO_ReadPin( stm32_gpio_get_port_base( bank ),
             stm32_gpio_get_pin_bitmask( pin ) ) );
}

rtems_status_code rtems_gpio_bsp_enable_interrupt(
  uint32_t             bank,
  uint32_t             pin,
  rtems_gpio_interrupt interrupt
)
{
  /* DO NOTHING */

  /**
   * 1. Interrupt vector is installed by rtems_gpio_enable_interrupt().
   * 2. EXTI Line configuration is done in HAL_Init().
   */
  return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_bsp_disable_interrupt(
  uint32_t             bank,
  uint32_t             pin,
  rtems_gpio_interrupt interrupt
)
{
  /* Disable the EXTI configuration of the pin */
  stm32_gpio_disable_exti( pin );

  return RTEMS_SUCCESSFUL;
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
  uint32_t      temp_pull = GPIO_NOPULL;
  uint32_t      pull_mode = GPIO_NOPULL;
  GPIO_TypeDef *gpio_bank_base;

  gpio_bank_base = stm32_gpio_get_port_base( bank );

  if ( NO_PULL_RESISTOR != mode ) {
    pull_mode = ( PULL_UP == mode ) ? GPIO_PULLUP : GPIO_PULLDOWN;
  }

  /* Activate the Pull-up or Pull down resistor */
  temp_pull = gpio_bank_base->PUPDR;
  temp_pull &= ~( GPIO_PUPDR_PUPDR0 << ( pin * 2 ) );
  temp_pull |= ( ( pull_mode ) << ( pin * 2 ) );

  gpio_bank_base->PUPDR = temp_pull;

  return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_bsp_multi_select(
  rtems_gpio_multiple_pin_select *pins,
  uint32_t                        pin_count,
  uint32_t                        select_bank
)
{
  stm32_gpio_pin_config *pin_config;
  GPIO_InitTypeDef       GPIO_InitStruct;
  uint8_t                count;

  for ( count = 0; count < pin_count; count++ ) {
    pin_config = (stm32_gpio_pin_config *) ( pins[ count ].bsp_specific );

    GPIO_InitStruct.Mode = pin_config->mode;
    GPIO_InitStruct.Pull = pin_config->pull;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Pin = stm32_gpio_get_pin_bitmask( pins[ count ].pin_number );
    GPIO_InitStruct.Alternate = NULL;

    /* Initialize the GPIO pin */
    HAL_GPIO_Init( stm32_gpio_get_port_base( select_bank ), &GPIO_InitStruct );
  }

  return RTEMS_SUCCESSFUL;
}

uint32_t rtems_gpio_bsp_interrupt_line( rtems_vector_number vector )
{

  /**
   * The status of the interrupt line can be obtained from the EXTI register,
   * which has the status of all BSP_GPIO_PINS_PER_BANK number of pins. But the
   * pin number cannot be extracted from the argument "vector" being passed,
   * which is vector for each GPIO banks, according to RTEMS GPIO APIs.
   */
  return RTEMS_NOT_DEFINED;
}

rtems_status_code rtems_gpio_bsp_specific_group_operation(
  uint32_t  bank,
  uint32_t *pins,
  uint32_t  pin_count,
  void     *arg
)
{

  /**
   *  Currently not supported. Not clear of any specific GPIO group operation
   *  can be done.
   */
  return RTEMS_NOT_DEFINED;
}
