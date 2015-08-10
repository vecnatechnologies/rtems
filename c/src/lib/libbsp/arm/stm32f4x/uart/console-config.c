/*
 * console-config.c
 *
 *  Created on: Aug 4, 2015
 *      Author: jay.doyle
 */


stm32f_console_driver_entry stm32f_console_driver_table[NUM_PROCESSOR_CONSOLE_UARTS] = {

//          UART6
[0] = {
        .base                = RTEMS_TERMIOS_DEVICE_CONTEXT_INITIALIZER("STM32F UART6"),
        .base_driver_info = {
                .device_name         = "/dev/console",
                .handle              = &(UartHandles[0]),
                .interrupt_number    = USART6_IRQn,
                .uart_mode           = STM32F_UART_MODE_POLLING,
                .tx_dma_stream       = DMA2_Stream6,
                .rx_dma_stream       = DMA2_Stream2,
                .tx_pin              = {STM32F_GOIO_PORTC, 6},
                .rx_pin              = {STM32F_GOIO_PORTC, 7},
                .tx_dma              = {STM32F_DMA2_CONTROLLER, DMA2_Stream6_IRQn, DMA_CHANNEL_5, 6},
                .rx_dma              = {STM32F_DMA2_CONTROLLER, DMA2_Stream2_IRQn, DMA_CHANNEL_5, 2},
                .baud                = 115200,
                .alt_func_config     = GPIO_AF8_USART6,
                .uart                = STM32F_UART6,
        }
},

};
