/*
 * uart-config.c
 *
 *  Created on: Jul 28, 2015
 *      Author: jay.doyle
 */



stm32_uart_driver_entry stm32f_uart_driver_table[NUM_PROCESSOR_UARTS] = {

//          UART6
[0] .base_driver_info = {
        .device_name         = "/dev/ttyS5",
        .handle              = &(UartHandles[NUM_PROCESSOR_CONSOLE_UARTS]),
        .UartInterruptNumber = USART6_IRQn,
        .uartType            = STM32F_UART_TYPE_INT,
        .TXDMAStream         = DMA2_Stream6,
        .RXDMAStream         = DMA2_Stream2,
        .TXPin               = {STM32F_GOIO_PORTC, 6},
        .RXPin               = {STM32F_GOIO_PORTC, 7},
        .TXDMA               = {STM32F_DMA2_CONTROLLER, DMA2_Stream6_IRQn, DMA_CHANNEL_5, 6},
        .RXDMA               = {STM32F_DMA2_CONTROLLER, DMA2_Stream2_IRQn, DMA_CHANNEL_5, 2},
        .baud                = 115200,
        .altFuncConfg        = GPIO_AF8_USART6,
        .uart                = STM32F_UART6,
},

};

stm32_uart_device uart_device_table[NUM_PROCESSOR_UARTS];

