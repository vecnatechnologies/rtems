/**
 * @file hal-gpio.h
 *
 * @ingroup gpio
 *
 * @brief Public GPIO driver datatypes
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

#ifndef STM32_GPIO_H
#define STM32_GPIO_H
#include <stdint.h>
#include <stdbool.h>
#include <bsp/gpio.h>

/**
 *
 */

/**
 * @brief GPIO Total Pin Enumerations
 */
enum /*No name*/
{
  /*** Port A ***/
  GPIO_PORTA_PIN0, GPIO_PORTA_PIN1, GPIO_PORTA_PIN2, GPIO_PORTA_PIN3,
  GPIO_PORTA_PIN4, GPIO_PORTA_PIN5, GPIO_PORTA_PIN6, GPIO_PORTA_PIN7,
  GPIO_PORTA_PIN8, GPIO_PORTA_PIN9, GPIO_PORTA_PIN10, GPIO_PORTA_PIN11,
  GPIO_PORTA_PIN12, GPIO_PORTA_PIN13, GPIO_PORTA_PIN14, GPIO_PORTA_PIN15,

  /*** Port B ***/
  GPIO_PORTB_PIN0, GPIO_PORTB_PIN1, GPIO_PORTB_PIN2, GPIO_PORTB_PIN3,
  GPIO_PORTB_PIN4, GPIO_PORTB_PIN5, GPIO_PORTB_PIN6, GPIO_PORTB_PIN7,
  GPIO_PORTB_PIN8, GPIO_PORTB_PIN9, GPIO_PORTB_PIN10, GPIO_PORTB_PIN11,
  GPIO_PORTB_PIN12, GPIO_PORTB_PIN13, GPIO_PORTB_PIN14, GPIO_PORTB_PIN15,

  /*** Port C ***/
  GPIO_PORTC_PIN0, GPIO_PORTC_PIN1, GPIO_PORTC_PIN2, GPIO_PORTC_PIN3,
  GPIO_PORTC_PIN4, GPIO_PORTC_PIN5, GPIO_PORTC_PIN6, GPIO_PORTC_PIN7,
  GPIO_PORTC_PIN8, GPIO_PORTC_PIN9, GPIO_PORTC_PIN10, GPIO_PORTC_PIN11,
  GPIO_PORTC_PIN12, GPIO_PORTC_PIN13, GPIO_PORTC_PIN14, GPIO_PORTC_PIN15,

  /*** Port D ***/
  GPIO_PORTD_PIN0, GPIO_PORTD_PIN1, GPIO_PORTD_PIN2, GPIO_PORTD_PIN3,
  GPIO_PORTD_PIN4, GPIO_PORTD_PIN5, GPIO_PORTD_PIN6, GPIO_PORTD_PIN7,
  GPIO_PORTD_PIN8, GPIO_PORTD_PIN9, GPIO_PORTD_PIN10, GPIO_PORTD_PIN11,
  GPIO_PORTD_PIN12, GPIO_PORTD_PIN13, GPIO_PORTD_PIN14, GPIO_PORTD_PIN15,

  /*** Port E ***/
  GPIO_PORTE_PIN0, GPIO_PORTE_PIN1, GPIO_PORTE_PIN2, GPIO_PORTE_PIN3,
  GPIO_PORTE_PIN4, GPIO_PORTE_PIN5, GPIO_PORTE_PIN6, GPIO_PORTE_PIN7,
  GPIO_PORTE_PIN8, GPIO_PORTE_PIN9, GPIO_PORTE_PIN10, GPIO_PORTE_PIN11,
  GPIO_PORTE_PIN12, GPIO_PORTE_PIN13, GPIO_PORTE_PIN14, GPIO_PORTE_PIN15,

  /*** Port F ***/
  GPIO_PORTF_PIN0, GPIO_PORTF_PIN1, GPIO_PORTF_PIN2, GPIO_PORTF_PIN3,
  GPIO_PORTF_PIN4, GPIO_PORTF_PIN5, GPIO_PORTF_PIN6, GPIO_PORTF_PIN7,
  GPIO_PORTF_PIN8, GPIO_PORTF_PIN9, GPIO_PORTF_PIN10, GPIO_PORTF_PIN11,
  GPIO_PORTF_PIN12, GPIO_PORTF_PIN13, GPIO_PORTF_PIN14, GPIO_PORTF_PIN15,

  /*** Port G ***/
  GPIO_PORTG_PIN0, GPIO_PORTG_PIN1, GPIO_PORTG_PIN2, GPIO_PORTG_PIN3,
  GPIO_PORTG_PIN4, GPIO_PORTG_PIN5, GPIO_PORTG_PIN6, GPIO_PORTG_PIN7,
  GPIO_PORTG_PIN8, GPIO_PORTG_PIN9, GPIO_PORTG_PIN10, GPIO_PORTG_PIN11,
  GPIO_PORTG_PIN12, GPIO_PORTG_PIN13, GPIO_PORTG_PIN14, GPIO_PORTG_PIN15,

  /*** Port H ***/
  GPIO_PORTH_PIN0, GPIO_PORTH_PIN1, GPIO_PORTH_PIN2, GPIO_PORTH_PIN3,
  GPIO_PORTH_PIN4, GPIO_PORTH_PIN5, GPIO_PORTH_PIN6, GPIO_PORTH_PIN7,
  GPIO_PORTH_PIN8, GPIO_PORTH_PIN9, GPIO_PORTH_PIN10, GPIO_PORTH_PIN11,
  GPIO_PORTH_PIN12, GPIO_PORTH_PIN13, GPIO_PORTH_PIN14, GPIO_PORTH_PIN15,

  /*** Port I ***/
  GPIO_PORTI_PIN0, GPIO_PORTI_PIN1, GPIO_PORTI_PIN2, GPIO_PORTI_PIN3,
  GPIO_PORTI_PIN4, GPIO_PORTI_PIN5, GPIO_PORTI_PIN6, GPIO_PORTI_PIN7,
  GPIO_PORTI_PIN8, GPIO_PORTI_PIN9, GPIO_PORTI_PIN10, GPIO_PORTI_PIN11,
  GPIO_PORTI_PIN12, GPIO_PORTI_PIN13, GPIO_PORTI_PIN14, GPIO_PORTI_PIN15,
};

typedef struct
{
  /* Operating mode of the pin */
  uint32_t mode;
  /* Resistor mode */
  uint32_t pull;
  /* Alternate function */
  uint32_t alternate;

} stm32_gpio_pin_config;

#endif /* STM32_GPIO_H */
