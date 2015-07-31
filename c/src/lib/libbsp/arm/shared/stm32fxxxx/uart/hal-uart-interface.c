/**
 * @file hal-uart-interface.c
 *
 * @ingroup uart
 *
 * @brief A set of utility functions used in the STM32F UART drivers that provide
 *   an API to the underlying hardware abstraction functions.
 *
 */

/*
 * Copyright (c) 2015 Vecna Technologies, Inc.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.com/license/LICENSE.
 */

//================== STMF32 Support Functions =================================
#include <hal-uart-interface.h>
#include <hal-utils.h>
#include <stm32f-processor-specific.h>


stm32f_base_uart_driver_entry* stm32f_get_driver_entry_from_handle(
  const UART_HandleTypeDef *huart
)
{
   uint32_t i;

   for(i = 0UL; i < COUNTOF(stm32f_console_driver_table) ; i++){
       if(stm32f_console_driver_table[i].base_driver_info.handle == huart){
           return (stm32f_base_uart_driver_entry*) &(stm32f_console_driver_table[i].base_driver_info);
       }
   }

   for(i = 0UL; i < COUNTOF(stm32f_uart_driver_table) ; i++){
       if(stm32f_uart_driver_table[i].base_driver_info.handle == huart){
           return (stm32f_base_uart_driver_entry*) &(stm32f_uart_driver_table[i].base_driver_info);
       }
   }

   return NULL;
}


stm32f_console_driver_entry* stm32f_get_console_driver_entry_from_handle(
  const UART_HandleTypeDef *huart
)
{
   uint32_t i;

   for(i = 0UL; i < COUNTOF(stm32f_console_driver_table) ; i++){
       if(stm32f_console_driver_table[i].base_driver_info.handle == huart){
           return (stm32f_console_driver_entry*) &(stm32f_console_driver_table[i]);
       }
   }

   return NULL;
}

void stm32f_init_uart_clock(
  const stm32f_uart Uart
)
{
    switch(Uart)
    {
    case STM32F_UART1:
        __HAL_RCC_USART1_CLK_ENABLE();
        break;

    case STM32F_UART2:
        __HAL_RCC_USART2_CLK_ENABLE();
        break;

    case STM32F_UART3:
        __HAL_RCC_USART3_CLK_ENABLE();
        break;

    case STM32F_UART4:
        __HAL_RCC_UART4_CLK_ENABLE();
        break;

    case STM32F_UART5:
        __HAL_RCC_UART5_CLK_ENABLE();
        break;

    case STM32F_UART6:
        __HAL_RCC_USART6_CLK_ENABLE();
        break;

#if defined(STM32F7_ENABLE_USART_7)
    case STM32F_UART7:
        __HAL_RCC_UART7_CLK_ENABLE();
        break;
#endif

#if defined(STM32F7_ENABLE_USART_8)
    case STM32F_UART8:
        __HAL_RCC_USART8_CLK_ENABLE();
        break;
#endif
    default:
    case STM32F_INVALID_UART:
        return;
        break;
    }
}


void stmf32_uart_reset(
  const stm32f_uart Uart
)
{
    switch(Uart)
    {
    case STM32F_UART1:
        __HAL_RCC_USART1_FORCE_RESET();
        __HAL_RCC_USART1_RELEASE_RESET();
        break;

    case STM32F_UART2:
        __HAL_RCC_USART2_FORCE_RESET();
        __HAL_RCC_USART2_RELEASE_RESET();
        break;

    case STM32F_UART3:
        __HAL_RCC_USART3_FORCE_RESET();
        __HAL_RCC_USART3_RELEASE_RESET();
        break;

    case STM32F_UART4:
        __HAL_RCC_UART4_FORCE_RESET();
        __HAL_RCC_UART4_RELEASE_RESET();
        break;

    case STM32F_UART5:
        __HAL_RCC_UART5_FORCE_RESET();
        __HAL_RCC_UART5_RELEASE_RESET();
        break;

    case STM32F_UART6:
        __HAL_RCC_USART6_FORCE_RESET();
        __HAL_RCC_USART6_RELEASE_RESET();
        break;

#if defined(STM32F7_ENABLE_USART_7)
    case STM32F_UART7:
        __HAL_RCC_UART7_FORCE_RESET();
        __HAL_RCC_UART7_RELEASE_RESET();
        break;
#endif

#if defined(STM32F7_ENABLE_USART_8)
    case STM32F_UART8:
        __HAL_RCC_USART8_FORCE_RESET();
        __HAL_RCC_USART8_RELEASE_RESET();
        break;
#endif

    default:
    case STM32F_INVALID_UART:
        return;
        break;
    }
}


void stmf32_init_gpio_clock(
  const stm32f_gpio_port port
)
{
    switch(port){

    case STM32F_GOIO_PORTA:
        __HAL_RCC_GPIOA_CLK_ENABLE();
        break;
    case STM32F_GOIO_PORTB:
        __HAL_RCC_GPIOB_CLK_ENABLE();
        break;
    case STM32F_GOIO_PORTC:
        __HAL_RCC_GPIOC_CLK_ENABLE();
        break;
    case STM32F_GOIO_PORTD:
        __HAL_RCC_GPIOD_CLK_ENABLE();
        break;
    case STM32F_GOIO_PORTE:
        __HAL_RCC_GPIOE_CLK_ENABLE();
        break;
    case STM32F_GOIO_PORTF:
        __HAL_RCC_GPIOF_CLK_ENABLE();
        break;
    case STM32F_GOIO_PORTG:
        __HAL_RCC_GPIOG_CLK_ENABLE();
        break;
    case STM32F_GOIO_PORTH:
        __HAL_RCC_GPIOH_CLK_ENABLE();
        break;
    case STM32F_GOIO_PORTI:
        __HAL_RCC_GPIOI_CLK_ENABLE();
        break;

    default:
        break;
    }
}


GPIO_TypeDef* stmf32_get_gpio(
  const stm32f_gpio_port port
)
{
    GPIO_TypeDef * ret = NULL;

    switch(port){

    case STM32F_GOIO_PORTA:
        ret = GPIOA;
        break;
    case STM32F_GOIO_PORTB:
        ret = GPIOB;
        break;
    case STM32F_GOIO_PORTC:
        ret = GPIOC;
        break;
    case STM32F_GOIO_PORTD:
        ret = GPIOD;
        break;
    case STM32F_GOIO_PORTE:
        ret = GPIOE;
        break;
    case STM32F_GOIO_PORTF:
        ret = GPIOF;
        break;
    case STM32F_GOIO_PORTG:
        ret = GPIOG;
        break;
    case STM32F_GOIO_PORTH:
        ret = GPIOH;
        break;
    case STM32F_GOIO_PORTI:
        ret = GPIOI;
        break;

    default:
        break;
    }

    return ret;
}


USART_TypeDef* stmf32_uart_get_registers(
  const stm32f_uart Uart
)
{
    USART_TypeDef* ret = NULL;

    switch(Uart)
    {
    case STM32F_UART1:
        ret = USART1;
        break;

    case STM32F_UART2:
        ret = USART2;
        break;

    case STM32F_UART3:
        ret = USART3;
        break;

    case STM32F_UART4:
        ret = UART4;
        break;

    case STM32F_UART5:
        ret = UART5;
        break;

    case STM32F_UART6:
        ret = USART6;
        break;

#if defined(STM32F7_ENABLE_USART_7)
    case STM32F_UART7:
        ret = UART7;
        break;
#endif

#if defined(STM32F7_ENABLE_USART_8)
    case STM32F_UART8:
        ret = USART8;
        break;
#endif

    default:
    case STM32F_INVALID_UART:
        break;
    }

    return ret;
}


void stmf32_init_dma_clock(
  const stm32f_dma_controller controller
)
{
    if(controller == STM32F_DMA1_CONTROLLER) {
        __HAL_RCC_DMA1_CLK_ENABLE();
    } else if(controller == STM32F_DMA2_CONTROLLER) {
        __HAL_RCC_DMA2_CLK_ENABLE();
    }
}



