/*
 * console-config.h
 *
 *  Created on: Jul 14, 2015
 *      Author: jay.doyle
 */

#ifndef RTEMS_C_SRC_LIB_LIBBSP_ARM_STM32F4_UART_CONSOLE_CONFIG_H_
#define RTEMS_C_SRC_LIB_LIBBSP_ARM_STM32F4_UART_CONSOLE_CONFIG_H_

#include <rtems.h>
#include <rtems/termiostypes.h>
#include <hal-uart-interface.h>

#define SERIAL_FIFO_SIZE 256

typedef enum {
    SERIAL_FIFO_NO_ERROR,
    SERIAL_FIFO_NULL_POINTER,
    SERIAL_FIFO_BUFFER_FULL,
    SERIAL_FIFO_BUFFER_EMPTY
} serial_fifo_error;

typedef struct _serial_fifo {
  uint8_t  buffer[SERIAL_FIFO_SIZE];
  uint32_t head;
  uint32_t count;
} serial_fifo;

typedef struct {
    rtems_termios_device_context base;
    const char*                  device_name;
    struct rtems_termios_tty*    tty;
    UART_HandleTypeDef*          handle;
    serial_fifo*                 fifo;
    IRQn_Type                    UartInterruptNumber;
    DMA_Stream_TypeDef*          RXDMAStream;
    DMA_Stream_TypeDef*          TXDMAStream;
    uint32_t                     initial_baud;
    stm32f_uart_type             uartType;
    stm32f_gpio_pin              TXPin;
    stm32f_gpio_pin              RXPin;
    stm32f_dma_config            TXDMA;
    stm32f_dma_config            RXDMA;
    uint8_t                      altFuncConfg;
} stm32f_uart_driver_entry;


rtems_device_driver console_initialize(
  rtems_device_major_number major,
  rtems_device_minor_number minor,
  void* arg
);

#endif /* RTEMS_C_SRC_LIB_LIBBSP_ARM_STM32F4_UART_CONSOLE_CONFIG_H_ */
