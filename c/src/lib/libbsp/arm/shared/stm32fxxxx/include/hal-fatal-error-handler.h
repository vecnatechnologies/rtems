/**
 * @file hal-fatal-error-handler.h
 *
 * @ingroup error
 *
 * @brief Error handler for CPU exceptions
 *
 */

/*
 * Copyright (c) 2015 Vecna Technologies, Inc.
 *
 * Author: Sudarshan Rajagopalan
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.com/license/LICENSE.
 */

#ifndef HAL_FATAL_ERROR_HANDLER_H_
#define HAL_FATAL_ERROR_HANDLER_H_

#define TASK_SWITCH_LOG_LENGTH 40

typedef struct
{
    char interrupt_name[32];
    int  interrupt_number;
} interrupt_name_t;

typedef enum {
  debug_event_type_none,
  debug_event_type_task_switch,
  debug_event_type_interrupt_enter,
  debug_event_type_interrupt_exit,
  debug_event_type_service,
  debug_event_type_supervisor
} debug_event_type_t;

typedef struct  {
    char executing_task[4];
    char heir_task[4];
    int  thread_dispatch_level;
    int  isr_nest_level;
    int  interrupt_number;
    debug_event_type_t event_type;
} debug_event_log_entry_t;

typedef struct  {
    debug_event_log_entry_t event_log_entry[TASK_SWITCH_LOG_LENGTH];
    uint32_t event_log_index;
} debug_event_log_t;

void add_isr_event(
  const int isr_number,
  const debug_event_type_t type
  );

/**
 * @brief HAL fatal error handler function.
 */
rtems_status_code stm32f_initialize_user_extensions( void );

#endif /* HAL_FATAL_ERROR_HANDLER_H_ */
