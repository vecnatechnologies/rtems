#include <bspopts.h>
#include <time.h>
#include <math.h>
#include <bsp.h>
#include <rtems.h>
#include <libchip/chip.h>
#include "../include/atsam_gpio.h"
#include <pio.h>
#include <pmc.h>



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
#define GPIO_BASE_ADDR_OFFSET 0x0200

#define PIOA_START_ADDR 0x400E0E00


/**
 * @brief Returns the base address of the GPIO port
 */
static inline Pio* atsam_pio_get_port_base( const atsam_pio_ports port )
{

  /**
   * Each base address of GPIO port is offset by GPIO_BASE_ADDR_OFFSET from
   * the AHB1PERIPH_BASE address.
   */
  return (Pio*)( PIOA_START_ADDR + (port * GPIO_BASE_ADDR_OFFSET) );
}

//PIO_PD30 PIO_PD30 PIO_PULLUP PIO_OUTPUT_0
static inline uint8_t atsam_pio_get_port_id(const atsam_pio_ports port){

  switch(port){
  case ATSAM_PIO_PORTA:
    return 10;
    //return ID_PIOA;
    break;
  case ATSAM_PIO_PORTB:
    return ID_PIOB;
    break;
  case ATSAM_PIO_PORTC:
    return ID_PIOC;
    break;
  case ATSAM_PIO_PORTD:
    return 16;
    //return ID_PIOD;
    break;
  case ATSAM_PIO_PORTE:
    return ID_PIOE;
    break;
  default:
    while(1){
      ;
    }
    return 0;
    break;
  }
}


rtems_status_code rtems_gpio_bsp_select_input(
  uint32_t bank,
  uint32_t pin,
  void    *bsp_specific
)
{

  Pin gpio_init;

  gpio_init.mask      = 1 << pin;
  gpio_init.pio       = atsam_pio_get_port_base((atsam_pio_ports)bank);
  gpio_init.id        = atsam_pio_get_port_id((atsam_pio_ports)  bank);
  gpio_init.type      = PIO_INPUT;
  gpio_init.attribute = *((uint8_t*) bsp_specific);

  // Enable MCAN peripheral clock
  PMC_EnablePeripheral(gpio_init.id);

  PIO_Configure(&gpio_init, PIO_LISTSIZE(gpio_init));

  return RTEMS_SUCCESSFUL;
}


rtems_status_code rtems_gpio_bsp_select_output(
  uint32_t bank,
  uint32_t pin,
  void    *bsp_specific
)
{
  Pin gpio_init;

  //#define TRACTION_PWR_EN  {PIO_PD30, PIOD, ID_PIOD, PIO_OUTPUT_0, PIO_PULLUP}
  gpio_init.mask      = 1 << pin;
  gpio_init.pio       = atsam_pio_get_port_base((atsam_pio_ports)bank);
  gpio_init.id        = atsam_pio_get_port_id((atsam_pio_ports)  bank);
  gpio_init.type      = PIO_OUTPUT_0;
  gpio_init.attribute = *((uint8_t*) bsp_specific);

  // Enable MCAN peripheral clock
  PMC_EnablePeripheral(gpio_init.id);

  PIO_Configure(&gpio_init, 1);

  return RTEMS_SUCCESSFUL;
}


rtems_status_code rtems_gpio_bsp_select_specific_io(
  uint32_t bank,
  uint32_t pin,
  uint32_t function,
  void    *pin_data
)
{

  return RTEMS_NOT_IMPLEMENTED;
}


rtems_status_code rtems_gpio_bsp_multi_set(
  uint32_t bank,
  uint32_t bitmask
)
{
  Pin gpio_set;
  uint32_t pin_count;

  gpio_set.pio       = atsam_pio_get_port_base((atsam_pio_ports)bank);
  gpio_set.id        = atsam_pio_get_port_id((atsam_pio_ports)  bank);
  gpio_set.type      = PIO_OUTPUT_0;
  gpio_set.attribute = NULL;

  for ( pin_count = 0; pin_count < MAX_GPIO_PINS_PER_PORT; pin_count++ ) {
    gpio_set.mask = 1 << pin_count;
    if ( gpio_set.mask & bitmask != 0){
      PIO_Set(&gpio_set);
    }
  }

  return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_bsp_multi_clear(
  uint32_t bank,
  uint32_t bitmask
)
{
  Pin gpio_set;
  uint32_t pin_count;

  gpio_set.pio       = atsam_pio_get_port_base((atsam_pio_ports)bank);
  gpio_set.id        = atsam_pio_get_port_id((atsam_pio_ports)  bank);
  gpio_set.type      = PIO_OUTPUT_0;
  gpio_set.attribute = NULL;

  for ( pin_count = 0; pin_count < MAX_GPIO_PINS_PER_PORT; pin_count++ ) {
    gpio_set.mask = 1 << pin_count;
    if ( gpio_set.mask & bitmask != 0){
      PIO_Clear(&gpio_set);
    }
  }

  return RTEMS_SUCCESSFUL;
}

uint32_t rtems_gpio_bsp_multi_read(
  uint32_t bank,
  uint32_t bitmask
)
{
  uint32_t ret = 0;

  Pin gpio_set;
  uint32_t pin_count;

  gpio_set.pio       = atsam_pio_get_port_base((atsam_pio_ports)bank);
  gpio_set.id        = atsam_pio_get_port_id((atsam_pio_ports)  bank);
  gpio_set.type      = PIO_OUTPUT_0;
  gpio_set.attribute = NULL;

  for ( pin_count = 0; pin_count < MAX_GPIO_PINS_PER_PORT; pin_count++ ) {
    gpio_set.mask = 1 << pin_count;
    if ( gpio_set.mask & bitmask != 0){
      if (PIO_Get(&gpio_set) != 0){
        ret |= gpio_set.mask;
      }
    }
  }

  return ret;
}

rtems_status_code rtems_gpio_bsp_set(
  uint32_t bank,
  uint32_t pin
)
{
  return rtems_gpio_bsp_multi_set(bank, 1 << pin);
}

rtems_status_code rtems_gpio_bsp_clear(
  uint32_t bank,
  uint32_t pin
)
{
  return rtems_gpio_bsp_multi_clear(bank, 1 << pin);
}


uint32_t rtems_gpio_bsp_get_value(
  uint32_t bank,
  uint32_t pin
)
{
  Pin gpio_get;

  gpio_get.mask      = 1 << pin;
  gpio_get.pio       = atsam_pio_get_port_base((atsam_pio_ports)bank);
  gpio_get.id        = atsam_pio_get_port_id((atsam_pio_ports)  bank);
  gpio_get.type      = PIO_INPUT;
  gpio_get.attribute = NULL;

  return (uint32_t) PIO_Get(&gpio_get);
}


rtems_status_code rtems_gpio_bsp_enable_interrupt(
  uint32_t             bank,
  uint32_t             pin,
  rtems_gpio_interrupt interrupt
)
{
  return RTEMS_NOT_IMPLEMENTED;
}

rtems_status_code rtems_gpio_bsp_disable_interrupt(
  uint32_t             bank,
  uint32_t             pin,
  rtems_gpio_interrupt interrupt
)
{
  return RTEMS_NOT_IMPLEMENTED;
}

rtems_vector_number rtems_gpio_bsp_get_vector( uint32_t bank )
{
  return 0;
}

rtems_status_code rtems_gpio_bsp_set_resistor_mode(
  uint32_t             bank,
  uint32_t             pin,
  rtems_gpio_pull_mode mode
)
{

  return RTEMS_NOT_IMPLEMENTED;
}

rtems_status_code rtems_gpio_bsp_multi_select(
  rtems_gpio_multiple_pin_select *pins,
  uint32_t                        pin_count,
  uint32_t                        select_bank
)
{
  return RTEMS_NOT_IMPLEMENTED;
}

uint32_t rtems_gpio_bsp_interrupt_line( rtems_vector_number vector )
{
  return RTEMS_NOT_IMPLEMENTED;
}

rtems_status_code rtems_gpio_bsp_specific_group_operation(
  uint32_t  bank,
  uint32_t *pins,
  uint32_t  pin_count,
  void     *arg
)
{
  return RTEMS_NOT_IMPLEMENTED;
}
