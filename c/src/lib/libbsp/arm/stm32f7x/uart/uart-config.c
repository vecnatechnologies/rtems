/*
 * uart-config.c
 *
 *  Created on: Jul 28, 2015
 *      Author: jay.doyle
 */

stm32f_uart_driver_entry stm32f_uart_driver_table[NUM_PROCESSOR_NON_CONSOLE_UARTS] = {

#if 0
//          UART4
[0] .base_driver_info = {
        .device_name         = "/dev/ttyS3",
        .handle              = &(UartHandles[NUM_PROCESSOR_CONSOLE_UARTS]),
        .interrupt_number    = UART4_IRQn,
        .uart_mode           = STM32F_UART_MODE_INT,
        .tx_dma_stream       = DMA1_Stream4,
        .rx_dma_stream       = DMA2_Stream2,
        .tx_pin              = {STM32F_GPIO_PORTA, 0},
        .rx_pin              = {STM32F_GPIO_PORTA, 1},
        .tx_dma              = {STM32F_DMA1_CONTROLLER, DMA1_Stream4_IRQn, DMA_CHANNEL_4, 4},
        .rx_dma              = {STM32F_DMA1_CONTROLLER, DMA1_Stream2_IRQn, DMA_CHANNEL_4, 2},
        .baud                = 115200,
        .alt_func_config     = GPIO_AF8_UART4,
        .uart                = STM32F_UART4,
},
#endif

//          UART7
[0] .base_driver_info = {
        .device_name         = "/dev/ttyS6",
        .handle              = &(UartHandles[NUM_PROCESSOR_CONSOLE_UARTS+1]),
        .interrupt_number    = UART7_IRQn,
        .uart_mode           = STM32F_UART_MODE_INT,
        .tx_dma_stream       = DMA1_Stream1,
        .rx_dma_stream       = DMA1_Stream3,
        .tx_pin              = {STM32F_GPIO_PORTF, 6},
        .rx_pin              = {STM32F_GPIO_PORTF, 7},
        .tx_dma              = {STM32F_DMA1_CONTROLLER, DMA1_Stream1_IRQn, DMA_CHANNEL_5, 1},
        .rx_dma              = {STM32F_DMA1_CONTROLLER, DMA1_Stream3_IRQn, DMA_CHANNEL_5, 3},
        .baud                = 115200,
        .alt_func_config     = GPIO_AF8_UART7,
        .uart                = STM32F_UART7,
},

};

stm32f_uart_device uart_device_table[NUM_PROCESSOR_NON_CONSOLE_UARTS];

