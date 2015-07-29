/**
 * NOTICE: This software  source code and any of  its derivatives are the
 * confidential  and  proprietary   information  of  Vecna  Technologies,
 * Inc. (such source  and its derivatives are hereinafter  referred to as
 * "Confidential Information"). The  Confidential Information is intended
 * to be  used exclusively by  individuals or entities that  have entered
 * into either  a non-disclosure agreement or license  agreement (or both
 * of  these agreements,  if  applicable) with  Vecna Technologies,  Inc.
 * ("Vecna")   regarding  the  use   of  the   Confidential  Information.
 * Furthermore,  the  Confidential  Information  shall be  used  only  in
 * accordance  with   the  terms   of  such  license   or  non-disclosure
 * agreements.   All  parties using  the  Confidential Information  shall
 * verify that their  intended use of the Confidential  Information is in
 * compliance  with and  not in  violation of  any applicable  license or
 * non-disclosure  agreements.  Unless expressly  authorized by  Vecna in
 * writing, the Confidential Information  shall not be printed, retained,
 * copied, or  otherwise disseminated,  in part or  whole.  Additionally,
 * any party using the Confidential  Information shall be held liable for
 * any and  all damages incurred  by Vecna due  to any disclosure  of the
 * Confidential  Information (including  accidental disclosure).   In the
 * event that  the applicable  non-disclosure or license  agreements with
 * Vecna  have  expired, or  if  none  currently  exists, all  copies  of
 * Confidential Information in your  possession, whether in electronic or
 * printed  form, shall be  destroyed or  returned to  Vecna immediately.
 * Vecna  makes no  representations  or warranties  hereby regarding  the
 * suitability  of  the   Confidential  Information,  either  express  or
 * implied,  including  but not  limited  to  the  implied warranties  of
 * merchantability,    fitness    for    a   particular    purpose,    or
 * non-infringement. Vecna  shall not be liable for  any damages suffered
 * by  licensee as  a result  of  using, modifying  or distributing  this
 * Confidential Information.  Please email [info@vecnatech.com]  with any
 * questions regarding the use of the Confidential Information.
 */

/*
 * Test_SCB.cpp
 *
 *  @author jay.doyle
 *
 *  Change Log:
 *    Created Apr 30, 2015 - jay.doyle
 */

#include <CppUTest/TestHarness.h>
#include <stdio.h>
#include <hal-utils.h>
#include <hal-uart-interface.h>
#include <bspopts.h>

TEST_GROUP(hal_uart_interface) {

  void setup() {
    printf("hal_uart_interface setup\n");
  }

  void teardown() {
    printf("hal_uart_interface teardown\n");
  }
};

static void uartRegisterAddAddress(
  USART_TypeDef* addr,
  USART_TypeDef** addr_list,
  uint32_t list_size
)
{
  uint32_t i;
  uint32_t insert_index = list_size;

  CHECK_TEXT(addr != NULL, "uartRegisterAddAddress receive a null pointer");

  // scan for duplicate addresses
  for(i = 0 ; i < list_size; i++) {
    CHECK_TEXT(addr_list[i] != addr, "uartRegisterAddAddress, duplicate addresses");
    if((addr_list[i] == NULL) && (insert_index == list_size)) {
      insert_index = i;
    }
  }

  CHECK_TEXT(insert_index != list_size, "uartRegisterAddAddress too many uarts");

  addr_list[insert_index] = addr;
}


TEST(hal_uart_interface, stmf32_uart_get_registers) {

  USART_TypeDef* register_list[8] = {0};
  USART_TypeDef* reg;

#if defined(STM32F7_ENABLE_USART_1)
  reg = stmf32_uart_get_registers(STM32F_UART1);
  CHECK_TEXT(reg != NULL, "stmf32_uart_get_registers returned NULL porter for UART1");
  uartRegisterAddAddress(reg, (USART_TypeDef**) register_list, COUNTOF(register_list));
#endif

#if defined(STM32F7_ENABLE_USART_2)
  reg = stmf32_uart_get_registers(STM32F_UART2);
  CHECK_TEXT(reg != NULL, "stmf32_uart_get_registers returned NULL porter for UART2");
  uartRegisterAddAddress(reg, (USART_TypeDef**) register_list, COUNTOF(register_list));
#endif


#if defined(STM32F7_ENABLE_USART_3)
  reg = stmf32_uart_get_registers(STM32F_UART3);
  CHECK_TEXT(reg != NULL, "stmf32_uart_get_registers returned NULL porter for UART3");
  uartRegisterAddAddress(reg, (USART_TypeDef**) register_list, COUNTOF(register_list));
#endif


#if defined(STM32F7_ENABLE_USART_4)
  reg = stmf32_uart_get_registers(STM32F_UART4);
  CHECK(reg != NULL, "stmf32_uart_get_registers returned NULL porter for UART1");
  uartRegisterAddAddress(reg, (USART_TypeDef**) register_list, COUNTOF(register_list));
#endif


#if defined(STM32F7_ENABLE_USART_5)
  reg = stmf32_uart_get_registers(STM32F_UART5);
  CHECK_TEXT(reg != NULL, "stmf32_uart_get_registers returned NULL porter for UART5");
  uartRegisterAddAddress(reg, (USART_TypeDef**) register_list, COUNTOF(register_list));
#endif


#if defined(STM32F7_ENABLE_USART_6)
  reg = stmf32_uart_get_registers(STM32F_UART6);
  CHECK_TEXT(reg != NULL, "stmf32_uart_get_registers returned NULL porter for UART6");
  uartRegisterAddAddress(reg, (USART_TypeDef**) register_list, COUNTOF(register_list));
#endif

#if defined(STM32F7_ENABLE_USART_7)
  reg = stmf32_uart_get_registers(STM32F_UART7);
  CHECK_TEXT(reg != NULL, "stmf32_uart_get_registers returned NULL porter for UART7");
  uartRegisterAddAddress(reg, (USART_TypeDef**) register_list, COUNTOF(register_list));
#endif


#if defined(STM32F7_ENABLE_USART_8)
  reg = stmf32_uart_get_registers(STM32F_UART8);
  CHECK_TEXT(reg != NULL, "stmf32_uart_get_registers returned NULL porter for UART8");
  uartRegisterAddAddress(reg, (USART_TypeDef**) register_list, COUNTOF(register_list));
#endif

}




