/*
 * console-config.c
 *
 *  Created on: Aug 4, 2015
 *      Author: jay.doyle
 */


stm32f_console_driver_entry stm32f_console_driver_table[NUM_PROCESSOR_CONSOLE_UARTS] = {

//          UART3
[0] = {
        .base                = RTEMS_TERMIOS_DEVICE_CONTEXT_INITIALIZER("STM32F UART3"),
        .base_driver_info = {
                .device_name         = "/dev/console",
                .handle              = &(UartHandles[0]),
                .interrupt_number    = USART3_IRQn,
                .uart_mode           = STM32F_UART_MODE_POLLING,
                .tx_dma_stream       = DMA2_Stream6,
                .rx_dma_stream       = DMA2_Stream2,
                .tx_pin              = {STM32F_GPIO_PORTC, 10},
                .rx_pin              = {STM32F_GPIO_PORTC, 11},
                .tx_dma              = {STM32F_DMA1_CONTROLLER, DMA1_Stream3_IRQn, DMA_CHANNEL_4, 3},
                .rx_dma              = {STM32F_DMA1_CONTROLLER, DMA1_Stream1_IRQn, DMA_CHANNEL_4, 1},
                .baud                = 115200,
                .alt_func_config     = GPIO_AF7_USART3,
                .uart                = STM32F_UART3,
        }
},

};
