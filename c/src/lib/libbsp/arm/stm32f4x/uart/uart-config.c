/*
 * uart.c
 *
 *  Created on: Jul 28, 2015
 *      Author: jay.doyle
 */


stm32_uart_driver_entry stm32f_uart_driver_table[NUM_PROCESSOR_NON_CONSOLE_UARTS] = {

  //          UART1
  [0] .base_driver_info = {
    .device_name         = "/dev/ttyS0",
    .handle              = &(UartHandles[NUM_PROCESSOR_CONSOLE_UARTS]),
    .interrupt_number = USART1_IRQn,
    .uart_mode            = STM32F_UART_MODE_DMA,
    .tx_dma_stream         = DMA2_Stream7,
    .rx_dma_stream         = DMA2_Stream5,
    .tx_pin               = {STM32F_GPIO_PORTB, 6},
    .rx_pin               = {STM32F_GPIO_PORTB, 7},
    .tx_dma               = {STM32F_DMA2_CONTROLLER, DMA2_Stream7_IRQn, DMA_CHANNEL_4, 7},
    .rx_dma               = {STM32F_DMA2_CONTROLLER, DMA2_Stream5_IRQn, DMA_CHANNEL_4, 5},
    .baud        = 115200,
    .alt_func_config        = GPIO_AF7_USART1,
    .uart                = STM32F_UART1,
  },
/*
  //          UART2
  [1] .base_driver_info = {
    .device_name     = "/dev/ttyS1",
    .handle          = &(UartHandles[NUM_PROCESSOR_CONSOLE_UARTS+1]),
    .interrupt_number = USART2_IRQn,
    .uart_mode        = STM32F_UART_MODE_DMA,
    .tx_dma_stream     = DMA1_Stream6,
    .rx_dma_stream     = DMA1_Stream5,
    .tx_pin           = {STM32F_GPIO_PORTD, 5},
    .rx_pin           = {STM32F_GPIO_PORTD, 6},
    .tx_dma           = {STM32F_DMA1_CONTROLLER, DMA1_Stream6_IRQn, DMA_CHANNEL_4, 6},
    .rx_dma           = {STM32F_DMA1_CONTROLLER, DMA1_Stream5_IRQn, DMA_CHANNEL_4, 5},
    .baud    = 115200,
    .alt_func_config    = GPIO_AF7_USART2,
    .uart            = STM32F_UART2,
  },

  //          UART3
  [2] .base_driver_info = {
    .device_name     = "/dev/ttyS2",
    .handle          = &(UartHandles[NUM_PROCESSOR_CONSOLE_UARTS+2]),
    .interrupt_number = USART3_IRQn,
    .uart_mode        = STM32F_UART_MODE_DMA,
    .tx_dma_stream     = DMA1_Stream4,
    .rx_dma_stream     = DMA1_Stream1,
    .tx_pin           = {STM32F_GPIO_PORTD, 8},
    .rx_pin           = {STM32F_GPIO_PORTD, 9},
    .tx_dma           = {STM32F_DMA1_CONTROLLER, DMA1_Stream4_IRQn, DMA_CHANNEL_7, 4},
    .rx_dma           = {STM32F_DMA1_CONTROLLER, DMA1_Stream1_IRQn, DMA_CHANNEL_4, 1},
    .baud    = 115200,
    .alt_func_config    = GPIO_AF7_USART3,
    .uart            = STM32F_UART3,
  },

  //          UART4
    [3] .base_driver_info = {
    .device_name     = "/dev/ttyS3",
    .handle          = &(UartHandles[NUM_PROCESSOR_CONSOLE_UARTS+3]),
    .interrupt_number = UART4_IRQn,
    .uart_mode        = STM32F_UART_MODE_DMA,
    .tx_dma_stream     = DMA1_Stream4,
    .rx_dma_stream     = DMA1_Stream2,
    .tx_pin           = {STM32F_GPIO_PORTC, 10},
    .rx_pin           = {STM32F_GPIO_PORTC, 11},
    .tx_dma           = {STM32F_DMA1_CONTROLLER, DMA1_Stream4_IRQn, DMA_CHANNEL_4, 4},
    .rx_dma           = {STM32F_DMA1_CONTROLLER, DMA1_Stream2_IRQn, DMA_CHANNEL_4, 2},
    .baud    = 115200,
    .alt_func_config    = GPIO_AF8_UART4,
    .uart            = STM32F_UART4,
  },

  //          UART5
  [4] .base_driver_info = {
    .device_name     = "/dev/ttyS4",
    .handle          = &(UartHandles[NUM_PROCESSOR_CONSOLE_UARTS+4]),
    .interrupt_number = UART5_IRQn,
    .uart_mode        = STM32F_UART_MODE_DMA,
    .tx_dma_stream     = DMA1_Stream7,
    .rx_dma_stream     = DMA1_Stream0,
    .tx_pin           = {STM32F_GPIO_PORTD, 2},
    .rx_pin           = {STM32F_GPIO_PORTC, 12},
    .tx_dma           = {STM32F_DMA1_CONTROLLER, DMA1_Stream7_IRQn, DMA_CHANNEL_4, 7},
    .rx_dma           = {STM32F_DMA1_CONTROLLER, DMA1_Stream0_IRQn, DMA_CHANNEL_4, 0},
    .baud    = 115200,
    .alt_func_config    = GPIO_AF8_UART5,
    .uart            = STM32F_UART5,
  },
  */
};

stm32_uart_device uart_device_table[NUM_PROCESSOR_NON_CONSOLE_UARTS];

