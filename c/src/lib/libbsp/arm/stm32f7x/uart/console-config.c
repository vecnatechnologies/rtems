/*
 * console-config.c
 *
 *  Created on: Jul 30, 2015
 *      Author: jay.doyle
 */

UART_HandleTypeDef UartHandles[NUM_PROCESSOR_CONSOLE_UARTS+NUM_PROCESSOR_UARTS];

stm32f_console_driver_entry stm32f_console_driver_table[NUM_PROCESSOR_CONSOLE_UARTS] = {

//          UART6
[0] = {
        .base                = RTEMS_TERMIOS_DEVICE_CONTEXT_INITIALIZER("STM32F UART6"),
        .base_driver_info = {
                .device_name         = "/dev/console",
                .handle              = &(UartHandles[0]),
                .UartInterruptNumber = USART6_IRQn,
                .uartType            = STM32F_UART_TYPE_POLLING,
                .TXDMAStream         = DMA2_Stream6,
                .RXDMAStream         = DMA2_Stream2,
                .TXPin               = {STM32F_GOIO_PORTC, 6},
                .RXPin               = {STM32F_GOIO_PORTC, 7},
                .TXDMA               = {STM32F_DMA2_CONTROLLER, DMA2_Stream6_IRQn, DMA_CHANNEL_5, 6},
                .RXDMA               = {STM32F_DMA2_CONTROLLER, DMA2_Stream2_IRQn, DMA_CHANNEL_5, 2},
                .baud                = 115200,
                .altFuncConfg        = GPIO_AF8_USART6,
                .uart                = STM32F_UART6
        }
},


};
