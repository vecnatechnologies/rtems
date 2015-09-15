/**
 * @file hal-bsp-reset.c
 *
 * @ingroup startup
 *
 * @brief An implementation of the RTEMS bsp_reset function.
 *
 */

/*
 * Copyright (c) 2015 Vecna Technologies, Inc.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.com/license/LICENSE.
 */
#include <rtems.h>

#include <bsp/bootcard.h>

void bsp_reset(void)
{
  rtems_interrupt_level level;

  (void) level;
  rtems_interrupt_disable(level);

  while (1);
}
