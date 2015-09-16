/*
 * uart.c
 *
 *  Created on: Jul 28, 2015
 *      Author: jay.doyle
 */


stm32f_uart_driver_entry stm32f_uart_driver_table[NUM_PROCESSOR_UARTS] = {

  //          UART1
  [0] .base                = RTEMS_TERMIOS_DEVICE_CONTEXT_INITIALIZER("STM32F UART1"),
  [0] .device_name         = "/dev/ttyS0",
  [0] .handle              = &(UartHandles[0]),
  [0] .fifo                = &(uart_fifo[0]),
  [0] .UartInterruptNumber = USART1_IRQn,
  [0] .uartType            = STM32F_UART_TYPE_DMA,
  [0] .TXDMAStream         = DMA2_Stream7,
  [0] .RXDMAStream         = DMA2_Stream5,
  [0] .TXPin               = {STM32F_GOIO_PORTB, 6},
  [0] .RXPin               = {STM32F_GOIO_PORTB, 7},
  [0] .TXDMA               = {STM32F_DMA2_CONTROLLER, DMA2_Stream7_IRQn, DMA_CHANNEL_4, 7},
  [0] .RXDMA               = {STM32F_DMA2_CONTROLLER, DMA2_Stream5_IRQn, DMA_CHANNEL_4, 5},
  [0] .initial_baud        = 115200,
  [0] .altFuncConfg        = GPIO_AF7_USART1,
  [0] .uart                = STM32F_UART1,

  //          UART2
  [1] .base                = RTEMS_TERMIOS_DEVICE_CONTEXT_INITIALIZER("STM32F UART2"),
  [1] .device_name     = "/dev/ttyS1",
  [1] .handle          = &(UartHandles[1]),
  [1] .fifo            = &(uart_fifo[1]),
  [1] .UartInterruptNumber = USART2_IRQn,
  [1] .uartType        = STM32F_UART_TYPE_INT,
  [1] .TXDMAStream     = DMA1_Stream6,
  [1] .RXDMAStream     = DMA1_Stream5,
  [1] .TXPin           = {STM32F_GOIO_PORTD, 5},
  [1] .RXPin           = {STM32F_GOIO_PORTD, 6},
  [1] .TXDMA           = {STM32F_DMA1_CONTROLLER, DMA1_Stream6_IRQn, DMA_CHANNEL_4, 6},
  [1] .RXDMA           = {STM32F_DMA1_CONTROLLER, DMA1_Stream5_IRQn, DMA_CHANNEL_4, 5},
  [1] .initial_baud    = 115200,
  [1] .altFuncConfg    = GPIO_AF7_USART2,
  [1] .uart            = STM32F_UART2,

  //          UART3
  [2] .base                = RTEMS_TERMIOS_DEVICE_CONTEXT_INITIALIZER("STM32F UART3"),
  [2] .device_name     = "/dev/ttyS2",
  [2] .handle          = &(UartHandles[2]),
  [2] .fifo            = &(uart_fifo[2]),
  [2] .UartInterruptNumber = USART3_IRQn,
  [2] .uartType        = STM32F_UART_TYPE_INT,
  [2] .TXDMAStream     = DMA1_Stream4,
  [2] .RXDMAStream     = DMA1_Stream1,
  [2] .TXPin           = {STM32F_GOIO_PORTD, 8},
  [2] .RXPin           = {STM32F_GOIO_PORTD, 9},
  [2] .TXDMA           = {STM32F_DMA1_CONTROLLER, DMA1_Stream4_IRQn, DMA_CHANNEL_7, 4},
  [2] .RXDMA           = {STM32F_DMA1_CONTROLLER, DMA1_Stream1_IRQn, DMA_CHANNEL_4, 1},
  [2] .initial_baud    = 115200,
  [2] .altFuncConfg    = GPIO_AF7_USART3,
  [2] .uart            = STM32F_UART3,

  //          UART4
  [3] .base                = RTEMS_TERMIOS_DEVICE_CONTEXT_INITIALIZER("STM32F UART4"),
  [3] .device_name     = "/dev/ttyS3",
  [3] .handle          = &(UartHandles[3]),
  [3] .fifo            = &(uart_fifo[3]),
  [3] .UartInterruptNumber = UART4_IRQn,
  [3] .uartType        = STM32F_UART_TYPE_INT,
  [3] .TXDMAStream     = DMA1_Stream4,
  [3] .RXDMAStream     = DMA1_Stream2,
  [3] .TXPin           = {STM32F_GOIO_PORTC, 10},
  [3] .RXPin           = {STM32F_GOIO_PORTC, 11},
  [3] .TXDMA           = {STM32F_DMA1_CONTROLLER, DMA1_Stream4_IRQn, DMA_CHANNEL_4, 4},
  [3] .RXDMA           = {STM32F_DMA1_CONTROLLER, DMA1_Stream2_IRQn, DMA_CHANNEL_4, 2},
  [3] .initial_baud    = 115200,
  [3] .altFuncConfg    = GPIO_AF8_UART4,
  [3] .uart            = STM32F_UART4,

  //          UART5
  [4] .base                = RTEMS_TERMIOS_DEVICE_CONTEXT_INITIALIZER("STM32F UART5"),
  [4] .device_name     = "/dev/ttyS4",
  [4] .handle          = &(UartHandles[4]),
  [4] .fifo            = &(uart_fifo[4]),
  [4] .UartInterruptNumber = UART5_IRQn,
  [4] .uartType        = STM32F_UART_TYPE_INT,
  [4] .TXDMAStream     = DMA1_Stream7,
  [4] .RXDMAStream     = DMA1_Stream0,
  [4] .TXPin           = {STM32F_GOIO_PORTD, 2},
  [4] .RXPin           = {STM32F_GOIO_PORTC, 12},
  [4] .TXDMA           = {STM32F_DMA1_CONTROLLER, DMA1_Stream7_IRQn, DMA_CHANNEL_4, 7},
  [4] .RXDMA           = {STM32F_DMA1_CONTROLLER, DMA1_Stream0_IRQn, DMA_CHANNEL_4, 0},
  [4] .initial_baud    = 115200,
  [4] .altFuncConfg    = GPIO_AF8_UART5,
  [4] .uart            = STM32F_UART5,

  //          UART6
  [5] .base                = RTEMS_TERMIOS_DEVICE_CONTEXT_INITIALIZER("STM32F UART6"),
  [5] .device_name     = "/dev/console",
  [5] .handle          = &(UartHandles[5]),
  [5] .fifo            = &(uart_fifo[5]),
  [5] .UartInterruptNumber = USART6_IRQn,
  [5] .uartType        = STM32F_UART_TYPE_POLLING,
  [5] .TXDMAStream     = DMA2_Stream6,
  [5] .RXDMAStream     = DMA2_Stream2,
  [5] .TXPin           = {STM32F_GOIO_PORTC, 6},
  [5] .RXPin           = {STM32F_GOIO_PORTC, 7},
  [5] .TXDMA           = {STM32F_DMA2_CONTROLLER, DMA2_Stream6_IRQn, DMA_CHANNEL_5, 6},
  [5] .RXDMA           = {STM32F_DMA2_CONTROLLER, DMA2_Stream2_IRQn, DMA_CHANNEL_5, 2},
  [5] .initial_baud    = 115200,
  [5] .altFuncConfg    = GPIO_AF8_USART6,
  [5] .uart            = STM32F_UART6,
};
