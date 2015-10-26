/*
 * Copyright (c) 2011-2012 Sebastian Huber.  All rights reserved.
 *
 *  embedded brains GmbH
 *  Obere Lagerstr. 30
 *  82178 Puchheim
 *  Germany
 *  <rtems@embedded-brains.de>
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#include <rtems/score/armv7m.h>

#include <bsp/irq-generic.h>
#include <bsp/armv7m-irq.h>

#ifdef ARM_MULTILIB_ARCH_V7M

void _ARMV7M_NVIC_Interrupt_dispatch(void)
{
  static uint32_t debug_counter = 0UL;
  uint32_t my_counter = debug_counter++;

  rtems_vector_number vector =
    ARMV7M_SCB_ICSR_VECTACTIVE_GET(_ARMV7M_SCB->icsr);

  _ARMV7M_Interrupt_service_enter(debug_counter, ARMV7M_IRQ_OF_VECTOR(vector));
  bsp_interrupt_handler_dispatch(ARMV7M_IRQ_OF_VECTOR(vector));
  _ARMV7M_Interrupt_service_leave(debug_counter, ARMV7M_IRQ_OF_VECTOR(vector));
}

#endif /* ARM_MULTILIB_ARCH_V7M */
