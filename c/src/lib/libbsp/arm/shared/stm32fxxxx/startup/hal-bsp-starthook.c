/**
 * @file hal-bsp-starthook.c
 *
 * @ingroup startup
 *
 * @brief A set of utility functions called during BSP startup.
 *
 */

/*
 * Copyright (c) 2015 Vecna Technologies, Inc.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.com/license/LICENSE.
 */

#include <bsp.h>
#include <bsp/start.h>

void BSP_START_TEXT_SECTION bsp_start_hook_0(void)
{
  /* Do nothing */
}

void BSP_START_TEXT_SECTION bsp_start_hook_1(void)
{
  bsp_start_copy_sections();
  bsp_start_clear_bss();

  /* At this point we can use objects outside the .start section */
}