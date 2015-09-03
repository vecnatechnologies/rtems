/**
 * @file hal-uart-interface.h
 *
 * @ingroup uart
 *
 * @brief An interface layer to ST's hardware abstraction
 *   layer API functions used in UART driver implementation.
 *
 */

/*
 * Copyright (c) 2015 Vecna Technologies, Inc.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.com/license/LICENSE.
 */

#ifndef RTEMS_C_SRC_LIB_LIBBSP_ARM_STM32F4_UART_HAL_UART_INTERFACE_H_
#define RTEMS_C_SRC_LIB_LIBBSP_ARM_STM32F4_UART_HAL_UART_INTERFACE_H_

#include <rtems.h>
#include <vecna-utils.h>

#include stm_processor_header(TARGET_STM_PROCESSOR_PREFIX)
#include stm_header(TARGET_STM_PROCESSOR_PREFIX, uart)

typedef enum {
    UartErrorNone = 0,
    UartErrorNoBufferAvailable
} stm32f_uart_error_codes;

typedef enum {
   STM32F_UART_TYPE_POLLING,
   STM32F_UART_TYPE_INT,
   STM32F_UART_TYPE_DMA
} stm32f_uart_type;

typedef enum {
   STM32F_UART1,
   STM32F_UART2,
   STM32F_UART3,
   STM32F_UART4,
   STM32F_UART5,
   STM32F_UART6,
   STM32F_INVALID_UART
} stm32f_uart;

typedef enum {
    STM32F_GOIO_PORTA,
    STM32F_GOIO_PORTB,
    STM32F_GOIO_PORTC,
    STM32F_GOIO_PORTD,
    STM32F_GOIO_PORTE,
    STM32F_GOIO_PORTF,
    STM32F_GOIO_PORTG,
    STM32F_GOIO_PORTH,
    STM32F_GOIO_PORTI
} stm32f_gpio_port;

typedef struct {
    stm32f_gpio_port port;
    uint32_t         pin;
} stm32f_gpio_pin;

typedef enum {
    STM32F_DMA1_CONTROLLER,
    STM32F_DMA2_CONTROLLER,
} stm32f_dma_controller;

typedef struct {
    stm32f_dma_controller controller;
    IRQn_Type             DMAStreamInterruptNumber;
    uint32_t              channel;
    uint32_t              stream;
} stm32f_dma_config;

stm32f_uart stm32f_uart_get_uart_from_handle(
  const UART_HandleTypeDef *huart
);

void stm32f_init_uart_clock(
  const stm32f_uart Uart
);

void stmf32_uart_reset(
  const stm32f_uart Uart
);

void stmf32_init_gpio_clock(
  const stm32f_gpio_port port
);

GPIO_TypeDef* stmf32_get_gpio(
  const stm32f_gpio_port port
);

USART_TypeDef* stmf32_uart_get_registers(
  const stm32f_uart Uart
);

void stmf32_init_dma_clock(
  const stm32f_dma_controller controller
);


#endif /* RTEMS_C_SRC_LIB_LIBBSP_ARM_STM32F4_UART_HAL_UART_INTERFACE_H_ */