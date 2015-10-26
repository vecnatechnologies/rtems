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

#include <rtems.h>
#include <rtems/timecounter.h>
#include <rtems/score/armv7m.h>

#include <bsp.h>

#ifdef ARM_MULTILIB_ARCH_V7M

/* This is defined in clockdrv_shell.h */
static void Clock_isr(void *arg);

static rtems_timecounter_simple _ARMV7M_TC;

static uint32_t _ARMV7M_TC_get(rtems_timecounter_simple *tc)
{
  volatile ARMV7M_Systick *systick = _ARMV7M_Systick;

  return systick->cvr;
}

static bool _ARMV7M_TC_is_pending(rtems_timecounter_simple *tc)
{
  volatile ARMV7M_SCB *scb = _ARMV7M_SCB;

  return ((scb->icsr & ARMV7M_SCB_ICSR_PENDSTSET) != 0);
}

static uint32_t _ARMV7M_TC_get_timecount(struct timecounter *tc)
{
  return rtems_timecounter_simple_downcounter_get(
    tc,
    _ARMV7M_TC_get,
    _ARMV7M_TC_is_pending
  );
}

static void _ARMV7M_TC_tick(void)
{
  rtems_timecounter_simple_downcounter_tick(&_ARMV7M_TC, _ARMV7M_TC_get);
}

static void _ARMV7M_Systick_at_tick(void)
{
  volatile ARMV7M_Systick *systick = _ARMV7M_Systick;

  /* Clear COUNTFLAG */
  systick->csr;
}

static void _ARMV7M_Systick_handler(void)
{
  static int debug_counter = 0;
  int my_counter = debug_counter++;

  _ARMV7M_Interrupt_service_enter(my_counter, -1);
  Clock_isr(NULL);
  _ARMV7M_Interrupt_service_leave(my_counter, -1);
}

static void _ARMV7M_Systick_handler_install(void)
{
  _ARMV7M_Set_exception_priority_and_handler(
    ARMV7M_VECTOR_SYSTICK,
    BSP_ARMV7M_SYSTICK_PRIORITY,
    _ARMV7M_Systick_handler
  );
}

static void _ARMV7M_Systick_initialize(void)
{
  volatile ARMV7M_Systick *systick = _ARMV7M_Systick;
  #ifdef BSP_ARMV7M_SYSTICK_FREQUENCY
    uint64_t freq = BSP_ARMV7M_SYSTICK_FREQUENCY;
  #else
    uint64_t freq = ARMV7M_SYSTICK_CALIB_TENMS_GET(systick->calib) * 100ULL;
  #endif
  uint64_t us_per_tick = rtems_configuration_get_microseconds_per_tick();
  uint64_t interval = (freq * us_per_tick) / 1000000ULL;

  systick->rvr = (uint32_t) interval;
  systick->cvr = 0;
  systick->csr = ARMV7M_SYSTICK_CSR_ENABLE
    | ARMV7M_SYSTICK_CSR_TICKINT
    | ARMV7M_SYSTICK_CSR_CLKSOURCE;

  rtems_timecounter_simple_install(
    &_ARMV7M_TC,
    freq,
    interval,
    _ARMV7M_TC_get_timecount
  );
}

static void _ARMV7M_Systick_cleanup(void)
{
  volatile ARMV7M_Systick *systick = _ARMV7M_Systick;

  systick->csr = 0;
}

#define Clock_driver_timecounter_tick() _ARMV7M_TC_tick()

#define Clock_driver_support_at_tick() \
  _ARMV7M_Systick_at_tick()

#define Clock_driver_support_initialize_hardware() \
  _ARMV7M_Systick_initialize()

#define Clock_driver_support_install_isr(isr, old_isr) \
  do { \
    _ARMV7M_Systick_handler_install(); \
    old_isr = NULL; \
  } while (0)

#define Clock_driver_support_shutdown_hardware() \
  _ARMV7M_Systick_cleanup()

/* Include shared source clock driver code */
#include "../../../../shared/clockdrv_shell.h"

#endif /* ARM_MULTILIB_ARCH_V7M */
