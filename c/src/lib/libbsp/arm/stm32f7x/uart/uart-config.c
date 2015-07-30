/*
 * uart-config.c
 *
 *  Created on: Jul 28, 2015
 *      Author: jay.doyle
 */



UART_HandleTypeDef UartHandles[NUM_PROCESSOR_UARTS];
stm32_uart stm32f_uart_driver_table_new[NUM_PROCESSOR_UARTS] = {

//          UART1
[0] .device_name         = "/dev/ttyS0",
[0] .handle              = &(UartHandles[0]),
[0] .UartInterruptNumber = USART1_IRQn,
[0] .uartType            = STM32F_UART_TYPE_DMA,
[0] .TXDMAStream         = DMA2_Stream7,
[0] .RXDMAStream         = DMA2_Stream5,
[0] .TXPin               = {STM32F_GOIO_PORTB, 6},
[0] .RXPin               = {STM32F_GOIO_PORTB, 7},
[0] .TXDMA               = {STM32F_DMA2_CONTROLLER, DMA2_Stream7_IRQn, DMA_CHANNEL_4, 7},
[0] .RXDMA               = {STM32F_DMA2_CONTROLLER, DMA2_Stream5_IRQn, DMA_CHANNEL_4, 5},
[0] .baud                = 115200,
[0] .altFuncConfg        = GPIO_AF7_USART1,
[0] .uart                = STM32F_UART1,

};

stm32_uart_device uart_device_table[NUM_PROCESSOR_UARTS];

