/**
 * @file uart_unit.c
 * @author Jay M. Doyle
 *
 * @ingroup test
 *
 * @brief CppUnit test code for testing the functions implemented in
 *   hal-uart*.c
 *
 */

/*
 * Copyright (c) 2015 Vecna Technologies, Inc.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.com/license/LICENSE.
 */

#include <CppUTest/TestHarness.h>
#include <stdio.h>

extern "C"
{
#include <hal-utils.h>
#include <hal-uart-interface.h>
#include <bspopts.h>
}

TEST_GROUP( hal_uart_unit )
{
  void setup()
  {
  }

  void teardown()
  {
  }
};

static void uartRegisterAddAddress(
  USART_TypeDef  *addr,
  USART_TypeDef **addr_list,
  uint32_t        list_size
)
{
  uint32_t i;
  uint32_t insert_index = list_size;

  CHECK_TEXT( addr != NULL, "uartRegisterAddAddress receive a null pointer" );

  // scan for duplicate addresses
  for ( i = 0; i < list_size; i++ ) {
    CHECK_TEXT( addr_list[ i ] != addr,
      "uartRegisterAddAddress, duplicate addresses" );

    if ( ( addr_list[ i ] == NULL ) && ( insert_index == list_size ) ) {
      insert_index = i;
    }
  }

  CHECK_TEXT( insert_index != list_size,
    "uartRegisterAddAddress too many uarts" );

  addr_list[ insert_index ] = addr;
}

TEST( hal_uart_unit, stm32f_uart_get_registers )
{
  USART_TypeDef *register_list[ 8 ] = { 0 };
  USART_TypeDef *reg;

#if defined( STM32_ENABLE_USART_1 )
  reg = stm32f_uart_get_registers( STM32F_UART1 );
  CHECK_TEXT( reg != NULL,
    "stm32f_uart_get_registers returned NULL porter for UART1" );
  uartRegisterAddAddress( reg, (USART_TypeDef **) register_list,
    COUNTOF( register_list ) );
#endif

#if defined( STM32_ENABLE_USART_2 )
  reg = stm32f_uart_get_registers( STM32F_UART2 );
  CHECK_TEXT( reg != NULL,
    "stm32f_uart_get_registers returned NULL porter for UART2" );
  uartRegisterAddAddress( reg, (USART_TypeDef **) register_list,
    COUNTOF( register_list ) );
#endif

#if defined( STM32_ENABLE_USART_3 )
  reg = stm32f_uart_get_registers( STM32F_UART3 );
  CHECK_TEXT( reg != NULL,
    "stm32f_uart_get_registers returned NULL porter for UART3" );
  uartRegisterAddAddress( reg, (USART_TypeDef **) register_list,
    COUNTOF( register_list ) );
#endif

#if defined( STM32_ENABLE_USART_4 )
  reg = stm32f_uart_get_registers( STM32F_UART4 );
  CHECK_TEXT( reg != NULL,
    "stm32f_uart_get_registers returned NULL porter for UART1" );
  uartRegisterAddAddress( reg, (USART_TypeDef **) register_list,
    COUNTOF( register_list ) );
#endif

#if defined( STM32_ENABLE_USART_5 )
  reg = stm32f_uart_get_registers( STM32F_UART5 );
  CHECK_TEXT( reg != NULL,
    "stm32f_uart_get_registers returned NULL porter for UART5" );
  uartRegisterAddAddress( reg, (USART_TypeDef **) register_list,
    COUNTOF( register_list ) );
#endif

#if defined( STM32_ENABLE_USART_6 )
  reg = stm32f_uart_get_registers( STM32F_UART6 );
  CHECK_TEXT( reg != NULL,
    "stm32f_uart_get_registers returned NULL porter for UART6" );
  uartRegisterAddAddress( reg, (USART_TypeDef **) register_list,
    COUNTOF( register_list ) );
#endif

#if defined( STM32F7_ENABLE_USART_7 )
  reg = stm32f_uart_get_registers( STM32F_UART7 );
  CHECK_TEXT( reg != NULL,
    "stm32f_uart_get_registers returned NULL porter for UART7" );
  uartRegisterAddAddress( reg, (USART_TypeDef **) register_list,
    COUNTOF( register_list ) );
#endif

#if defined( STM32_ENABLE_USART_8 )
  reg = stm32f_uart_get_registers( STM32F_UART8 );
  CHECK_TEXT( reg != NULL,
    "stm32f_uart_get_registers returned NULL porter for UART8" );
  uartRegisterAddAddress( reg, (USART_TypeDef **) register_list,
    COUNTOF( register_list ) );
#endif
}


TEST( hal_uart_unit, printk )
{
  CHECK_TEXT( BSP_output_char != NULL, "BSP_output_char is NULL -- printk will not work" );
  CHECK_TEXT( BSP_poll_char   != NULL, "BSP_poll_char is NULL" );
}
