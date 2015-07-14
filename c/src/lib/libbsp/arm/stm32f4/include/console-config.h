/*
 * console-config.h
 *
 *  Created on: Jul 14, 2015
 *      Author: jay.doyle
 */

#ifndef RTEMS_C_SRC_LIB_LIBBSP_ARM_STM32F4_UART_CONSOLE_CONFIG_H_
#define RTEMS_C_SRC_LIB_LIBBSP_ARM_STM32F4_UART_CONSOLE_CONFIG_H_

typedef enum {
    UartErrorNone = 0,
    UartErrorNoBufferAvailable
} STM32FUartErrorCodes;

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


#endif /* RTEMS_C_SRC_LIB_LIBBSP_ARM_STM32F4_UART_CONSOLE_CONFIG_H_ */
