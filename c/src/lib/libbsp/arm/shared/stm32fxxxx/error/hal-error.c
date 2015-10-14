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
#include <rtems.h>
#include <rtems/ringbuf.h>
#include <stdio.h>

static Ring_buffer_t critical_error_text_buffer;
static bool critical_error_text_buffer_initialized = false;

static void stm32f_error_handler_init(void) {

  if(critical_error_text_buffer_initialized == false) {
    Ring_buffer_Initialize(&critical_error_text_buffer);
    critical_error_text_buffer_initialized = true;
  }
}

static void stm32f_error_handler_record_error( const char* error_text) {

  uint32_t i;
  uint32_t size_to_copy;

  stm32f_error_handler_init();

  // If the text is larger than the size of the ring buffer then
  // I would rather have the start of the messages than the
  // end.  This also prevents against unterminated input strings.
  size_to_copy = (strlen(error_text) > RINGBUF_QUEUE_LENGTH) ?
      RINGBUF_QUEUE_LENGTH : strlen(error_text);

  for(i = 0; i < size_to_copy; i++) {
    Ring_buffer_Add_character(&critical_error_text_buffer, error_text[i]);
  }
}


void stm32f_error_handler( void )
{
  stm32f_error_handler_with_reason("Unspecificed Error");
}

void stm32f_error_handler_with_reason( const char *error_text )
{
  stm32f_error_handler_with_reason_conditional( error_text, true );
}

void stm32f_error_handler_with_reason_conditional(
  const char *error_text,
  const bool  stop_executing
)
{
  char error_text_to_console[64];

  stm32f_error_handler_record_error(error_text);

  snprintf((char*) error_text_to_console, sizeof(error_text_to_console) - 1, "\nError: %s ", error_text);
  error_text_to_console[sizeof(error_text_to_console) - 1] = '\0';
  printf(error_text_to_console);

  if ( stop_executing ) {
    while (1) {
      // Stay here forever
      ;
    }
  }
}
