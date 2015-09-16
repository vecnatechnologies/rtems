/*
 * uart-config.c
 *
 *  Created on: Jul 28, 2015
 *      Author: jay.doyle
 */


stm32f_uart_driver_entry stm32f_uart_driver_table[NUM_PROCESSOR_UARTS] = {

  //          UART6
  [0] .base                = RTEMS_TERMIOS_DEVICE_CONTEXT_INITIALIZER("STM32F UART6"),
  [0] .device_name     = "/dev/console",
  [0] .handle          = &(UartHandles[0]),
  [0] .fifo            = &(uart_fifo[0]),
  [0] .UartInterruptNumber = USART6_IRQn,
  [0] .uartType        = STM32F_UART_TYPE_POLLING,
  [0] .TXDMAStream     = DMA2_Stream6,
  [0] .RXDMAStream     = DMA2_Stream2,
  [0] .TXPin           = {STM32F_GOIO_PORTC, 6},
  [0] .RXPin           = {STM32F_GOIO_PORTC, 7},
  [0] .TXDMA           = {STM32F_DMA2_CONTROLLER, DMA2_Stream6_IRQn, DMA_CHANNEL_5, 6},
  [0] .RXDMA           = {STM32F_DMA2_CONTROLLER, DMA2_Stream2_IRQn, DMA_CHANNEL_5, 2},
  [0] .initial_baud    = 115200,
  [0] .altFuncConfg    = GPIO_AF8_USART6,
  [0] .uart            = STM32F_UART6
};

