/**
 *  @file
 *
 *  @brief ARMV7M Interrupt Service Enter and Leave
 */

/*
 * Copyright (c) 2011 Sebastian Huber.  All rights reserved.
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

#ifdef HAVE_CONFIG_H
  #include "config.h"
#endif

#include <rtems/score/armv7m.h>
#include <rtems/score/isr.h>
#include <rtems/score/threaddispatch.h>

#ifdef ARM_MULTILIB_ARCH_V7M

typedef enum {
  debug_event_type_none,
  debug_event_type_task_switch,
  debug_event_type_interrupt_enter,
  debug_event_type_interrupt_exit,
} debug_event_type_t;

void add_isr_event(
  const int isr_number,
  const debug_event_type_t type
  );


void _ARMV7M_Interrupt_service_enter( const int debug_counter, const int irq )
{
  static int crazy_counter = 0;

  int initial_level = _Thread_Dispatch_disable_level;

  if(initial_level < 0 ) {
    crazy_counter++;
  }

  ++_Thread_Dispatch_disable_level;
  ++_ISR_Nest_level;

  // this should never be true
  if(_Thread_Dispatch_disable_level == 0) {
    crazy_counter++;
  }

  if (initial_level + 1 != _Thread_Dispatch_disable_level){
    crazy_counter++;
  }

  add_isr_event(irq, debug_event_type_interrupt_enter);
}

void _ARMV7M_Interrupt_service_leave( const int debug_counter, const int irq  )
{

  if((_Thread_Dispatch_disable_level > 0) &&
     (_ISR_Nest_level > 0)){

    --_ISR_Nest_level;
    --_Thread_Dispatch_disable_level;

    if (
      _ISR_Nest_level == 0
        && _Thread_Dispatch_disable_level == 0
        && _Thread_Dispatch_necessary
    ) {
      _ARMV7M_SCB->icsr = ARMV7M_SCB_ICSR_PENDSVSET;
    }

  }

  add_isr_event(irq, debug_event_type_interrupt_exit);
}

#endif /* ARM_MULTILIB_ARCH_V7M */
