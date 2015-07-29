/**
 * @file hal-uart-interface.h
 *
 * @ingroup uart
 *
 * @brief An interface layer to ST's hardware abstraction
 *   layer API functions used in UART driver implementation.
 *
 */

/*
 * Copyright (c) 2015 Vecna Technologies, Inc.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.com/license/LICENSE.
 */

#ifndef RTEMS_C_SRC_LIB_LIBBSP_ARM_STM32F4_UART_HAL_UART_INTERFACE_H_
#define RTEMS_C_SRC_LIB_LIBBSP_ARM_STM32F4_UART_HAL_UART_INTERFACE_H_

#include <hal-utils.h>

#include stm_processor_header(TARGET_STM_PROCESSOR_PREFIX)
#include stm_header(TARGET_STM_PROCESSOR_PREFIX, uart)

#include <rtems.h>
#include <termios.h>
#include <rtems/irq.h>
#include <rtems/libio.h>
#include <rtems/termiostypes.h>
#include <rtems/irq-extension.h>
#include <rtems/ringbuf.h>

#define SERIAL_FIFO_SIZE 256

typedef enum {
    UartErrorNone = 0,
    UartErrorNoBufferAvailable
} stm32f_uart_error_codes;

typedef enum {
   STM32F_UART_TYPE_POLLING,
   STM32F_UART_TYPE_INT,
   STM32F_UART_TYPE_DMA
} stm32f_uart_type;

typedef enum {
   STM32F_UART1,
   STM32F_UART2,
   STM32F_UART3,
   STM32F_UART4,
   STM32F_UART5,
   STM32F_UART6,
   STM32F_UART7,
   STM32F_UART8,
   STM32F_INVALID_UART
} stm32f_uart;

typedef enum {
    STM32F_GOIO_PORTA,
    STM32F_GOIO_PORTB,
    STM32F_GOIO_PORTC,
    STM32F_GOIO_PORTD,
    STM32F_GOIO_PORTE,
    STM32F_GOIO_PORTF,
    STM32F_GOIO_PORTG,
    STM32F_GOIO_PORTH,
    STM32F_GOIO_PORTI
} stm32f_gpio_port;

typedef struct {
    stm32f_gpio_port port;
    uint32_t         pin;
} stm32f_gpio_pin;

typedef enum {
    STM32F_DMA1_CONTROLLER,
    STM32F_DMA2_CONTROLLER,
} stm32f_dma_controller;

typedef struct {
    stm32f_dma_controller controller;
    IRQn_Type             DMAStreamInterruptNumber;
    uint32_t              channel;
    uint32_t              stream;
} stm32f_dma_config;

typedef struct {
    rtems_termios_device_context base;
    const char*                  device_name;
    struct rtems_termios_tty*    tty;
    UART_HandleTypeDef*          handle;
    Ring_buffer_t*               fifo;
    uint8_t                      tx_buffer[SERIAL_FIFO_SIZE];
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
    stm32f_uart                  uart;
    uint8_t                      rxChar;
} stm32f_uart_driver_entry;

rtems_device_driver console_initialize(
  rtems_device_major_number major,
  rtems_device_minor_number minor,
  void* arg
);

void stm32f_init_uart_clock(
  const stm32f_uart Uart
);

void stmf32_uart_reset(
  const stm32f_uart Uart
);

void stmf32_init_gpio_clock(
  const stm32f_gpio_port port
);

GPIO_TypeDef* stmf32_get_gpio(
  const stm32f_gpio_port port
);

USART_TypeDef* stmf32_uart_get_registers(
  const stm32f_uart Uart
);

void stmf32_init_dma_clock(
  const stm32f_dma_controller controller
);

int stm32f_uart_register_interrupt_handlers(
        stm32f_uart_driver_entry* pUart
);

stm32f_uart_driver_entry* stm32f_get_driver_entry_from_handle(
  const UART_HandleTypeDef *huart
);

#endif /* RTEMS_C_SRC_LIB_LIBBSP_ARM_STM32F4_UART_HAL_UART_INTERFACE_H_ */
