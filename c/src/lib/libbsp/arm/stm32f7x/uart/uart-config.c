/*
 * uart-config.c
 *
 *  Created on: Jul 28, 2015
 *      Author: jay.doyle
 */


stm32_uart_driver_entry stm32f_uart_driver_table[NUM_PROCESSOR_UARTS] = {

//          UART4
[0] .base_driver_info = {
        .device_name         = "/dev/ttyS3",
        .handle              = &(UartHandles[NUM_PROCESSOR_CONSOLE_UARTS]),
        .UartInterruptNumber = UART4_IRQn,
        .uartType            = STM32F_UART_TYPE_INT,
        .TXDMAStream         = DMA1_Stream4,
        .RXDMAStream         = DMA2_Stream2,
        .TXPin               = {STM32F_GOIO_PORTA, 0},
        .RXPin               = {STM32F_GOIO_PORTA, 1},
        .TXDMA               = {STM32F_DMA1_CONTROLLER, DMA1_Stream4_IRQn, DMA_CHANNEL_4, 4},
        .RXDMA               = {STM32F_DMA1_CONTROLLER, DMA1_Stream2_IRQn, DMA_CHANNEL_4, 2},
        .baud                = 115200,
        .altFuncConfg        = GPIO_AF8_UART4,
        .uart                = STM32F_UART4,
},


//          UART7
[1] .base_driver_info = {
        .device_name         = "/dev/ttyS6",
        .handle              = &(UartHandles[NUM_PROCESSOR_CONSOLE_UARTS+1]),
        .UartInterruptNumber = UART7_IRQn,
        .uartType            = STM32F_UART_TYPE_DMA,
        .TXDMAStream         = DMA1_Stream1,
        .RXDMAStream         = DMA1_Stream3,
        .TXPin               = {STM32F_GOIO_PORTF, 6},
        .RXPin               = {STM32F_GOIO_PORTF, 7},
        .TXDMA               = {STM32F_DMA1_CONTROLLER, DMA1_Stream1_IRQn, DMA_CHANNEL_5, 1},
        .RXDMA               = {STM32F_DMA1_CONTROLLER, DMA1_Stream3_IRQn, DMA_CHANNEL_5, 3},
        .baud                = 115200,
        .altFuncConfg        = GPIO_AF8_UART7,
        .uart                = STM32F_UART7,
},

};

stm32_uart_device uart_device_table[NUM_PROCESSOR_UARTS];

