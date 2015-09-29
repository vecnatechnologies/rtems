/**
 * @file hal-error.c
 * @author Jay M. Doyle
 *
 * @ingroup error
 *
 * @brief A set of utility functions to handle errors found
 *   while executing the application.
 */

/*
 * Copyright (c) 2015 Vecna Technologies, Inc.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.com/license/LICENSE.
 */

#include <hal-error.h>

void stm32f_error_handler( void )
{
  while ( 1 ) {
    ;     // Stay here forever
  }
}

void stm32f_error_handler_with_reason( const char *error_text )
{
  stm32f_error_handler_with_reason_conditional(error_text, true);
}


void stm32f_error_handler_with_reason_conditional( const char *error_text,
                                                   const bool stop_executing)
{
  printf("\n**** ");
  printf( error_text );
  printf("****\n");

  if(stop_executing) {
    stm32f_error_handler();
  }
}
