/*
 * atsam_gpio.h
 *
 *  Created on: Jul 5, 2016
 *      Author: jay.doyle
 */

#ifndef INCLUDE_ATSAM_GPIO_H_
#define INCLUDE_ATSAM_GPIO_H_

#include <rtems/irq-extension.h>
#include <bsp/gpio.h>

typedef enum {
  ATSAM_PIO_PORTA,
  ATSAM_PIO_PORTB,
  ATSAM_PIO_PORTC,
  ATSAM_PIO_PORTD,
  ATSAM_PIO_PORTE
} atsam_pio_ports;


rtems_status_code rtems_gpio_bsp_select_input(
  uint32_t bank,
  uint32_t pin,
  void    *bsp_specific
);

rtems_status_code rtems_gpio_bsp_select_output(
  uint32_t bank,
  uint32_t pin,
  void    *bsp_specific
);

rtems_status_code rtems_gpio_bsp_select_specific_io(
  uint32_t bank,
  uint32_t pin,
  uint32_t function,
  void    *pin_data
);

rtems_status_code rtems_gpio_bsp_multi_set(
  uint32_t bank,
  uint32_t bitmask
);

rtems_status_code rtems_gpio_bsp_multi_clear(
  uint32_t bank,
  uint32_t bitmask
);

uint32_t rtems_gpio_bsp_multi_read(
  uint32_t bank,
  uint32_t bitmask
);

rtems_status_code rtems_gpio_bsp_set(
  uint32_t bank,
  uint32_t pin
);

rtems_status_code rtems_gpio_bsp_clear(
  uint32_t bank,
  uint32_t pin
);

uint32_t rtems_gpio_bsp_get_value(
  uint32_t bank,
  uint32_t pin
);

rtems_status_code rtems_gpio_bsp_enable_interrupt(
  uint32_t             bank,
  uint32_t             pin,
  rtems_gpio_interrupt interrupt
);

rtems_status_code rtems_gpio_bsp_disable_interrupt(
  uint32_t             bank,
  uint32_t             pin,
  rtems_gpio_interrupt interrupt
);

rtems_vector_number rtems_gpio_bsp_get_vector( uint32_t bank );

rtems_status_code rtems_gpio_bsp_set_resistor_mode(
  uint32_t             bank,
  uint32_t             pin,
  rtems_gpio_pull_mode mode
);

rtems_status_code rtems_gpio_bsp_multi_select(
  rtems_gpio_multiple_pin_select *pins,
  uint32_t                        pin_count,
  uint32_t                        select_bank
);

uint32_t rtems_gpio_bsp_interrupt_line( rtems_vector_number vector );

rtems_status_code rtems_gpio_bsp_specific_group_operation(
  uint32_t  bank,
  uint32_t *pins,
  uint32_t  pin_count,
  void     *arg
);




#endif /* INCLUDE_ATSAM_GPIO_H_ */
