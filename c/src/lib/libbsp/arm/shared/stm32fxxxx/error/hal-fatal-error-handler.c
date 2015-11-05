/**
 * @file hal-fatal-error-handler.c
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

#include <rtems.h>
#include <rtems/score/userextimpl.h>
#include <rtems/score/cpu.h>
#include <rtems/fatal.h>
#include <rtems/score/thread.h>
#include <hal-utils.h>
#include <stdio.h>
#include <hal-fatal-error-handler.h>
#include <rtems/score/armv7m.h>
#include <rtems/score/isr.h>
#include <rtems/score/threaddispatch.h>

static interrupt_name_t interrupt_name_bindings[] = {{"ETH", 61}, {"Timer", -1}, {"DMA", 15}, {"UART4", 52}};
static debug_event_log_t debug_event_log = {{0},0};

/* Extension name */
rtems_name Extension_name;

/* Extension ID */
rtems_id Extension_id;

typedef struct
{
  uint64_t reg_value;
}register_double;


static bool is_interrupt_event(const debug_event_type_t event) {
  return (event == debug_event_type_interrupt_enter) || (event == debug_event_type_interrupt_exit);
}

static void checkout_for_break(void){

  static uint32_t debug_counter = 0UL;
  static uint32_t discontinuity_count = 0UL;

  uint32_t index = debug_event_log.event_log_index;
  debug_event_log_entry_t* current = &(debug_event_log.event_log_entry[index]);

  if  (index == 0) { index = COUNTOF(debug_event_log.event_log_entry) - 1;}
  else             { index--;}

  debug_event_log_entry_t* last = &(debug_event_log.event_log_entry[index]);

  // Look for back to back interrupt starting from a timer tick
  // and followed by another interrupt
  if((is_interrupt_event(current->event_type)) &&
     (is_interrupt_event(last->event_type))) {

     if(last->interrupt_number == -1) {
       debug_counter++;
     }
  }

  if((is_interrupt_event(current->event_type)) &&
     (current->isr_nest_level != current->thread_dispatch_level)) {
    discontinuity_count++;
  }
}

static char* find_interrupt_name(const int interrupt_number) {

  int i;

  for(i = 0; i < COUNTOF(interrupt_name_bindings); i++) {
    if(interrupt_name_bindings[i].interrupt_number == interrupt_number) {
      return (char*) interrupt_name_bindings[i].interrupt_name;
    }
  }

  return NULL;
}

static debug_event_log_entry_t* get_next_log_entry(void)
{
  debug_event_log_entry_t* ret = NULL;

  if(debug_event_log.event_log_index < COUNTOF(debug_event_log.event_log_entry)) {
    ret = &(debug_event_log.event_log_entry[debug_event_log.event_log_index]);
  }

  return ret;
}

static void increment_log_index(void)
{
  debug_event_log.event_log_index =
      (debug_event_log.event_log_index + 1) %
      COUNTOF(debug_event_log.event_log_entry);
}

static void copy_task_name(
  const uint32_t task_name,
  char* storage_location
)
{
  char* task_name_p = (char*) &task_name;

  if(storage_location != NULL) {
    storage_location[0] = task_name_p[3];
    storage_location[1] = task_name_p[2];
    storage_location[2] = task_name_p[1];
    storage_location[3] = task_name_p[0];
  }
}

static void do_common_event_logging(const uint32_t executing,
                                    const uint32_t heir,
                                    debug_event_log_entry_t* pEvent) {

  if(pEvent != NULL) {

    copy_task_name(executing,(char*) pEvent->executing_task);
    copy_task_name(heir, (char*) pEvent->heir_task);

    pEvent->isr_nest_level = _ISR_Nest_level;
    pEvent->thread_dispatch_level = _Thread_Dispatch_disable_level;

    checkout_for_break();

    increment_log_index();
  }
}

void add_isr_event(
  const int isr_number,
  const debug_event_type_t type
  )
{
  debug_event_log_entry_t* pEvent = get_next_log_entry();

  if(pEvent != NULL) {

    pEvent->event_type = type;
    pEvent->interrupt_number = isr_number;

    do_common_event_logging(0UL, 0UL, pEvent);
  }
}

static void stm32fxxxx_task_switch_extension (
  Thread_Control *executing,
  Thread_Control *heir
)
{
  debug_event_log_entry_t* pEvent = get_next_log_entry();

  if(pEvent != NULL) {

    pEvent->event_type = debug_event_type_task_switch;
    pEvent->interrupt_number = 0;
    do_common_event_logging(executing->Object.name.name_u32,
      heir->Object.name.name_u32, pEvent);
  }
}

void print_event_log(void)
{
  uint32_t i;
  uint32_t log_index;

  log_index = debug_event_log.event_log_index;
  debug_event_log_entry_t* pEvent;

  printk("Event log\n");

  for(i = 0UL; i < COUNTOF(debug_event_log.event_log_entry); i++) {

    if(log_index == 0) {
      log_index = COUNTOF(debug_event_log.event_log_entry) - 1;
    } else {
      log_index--;
    }

    pEvent = &(debug_event_log.event_log_entry[log_index]);

    switch(pEvent->event_type) {

    case debug_event_type_task_switch:
    printk("\t%c%c%c%c -> %c%c%c%c %d %d\n",
      pEvent->executing_task[0],
      pEvent->executing_task[1],
      pEvent->executing_task[2],
      pEvent->executing_task[3],
      pEvent->heir_task[0],
      pEvent->heir_task[1],
      pEvent->heir_task[2],
      pEvent->heir_task[3],
      pEvent->isr_nest_level,
      pEvent->thread_dispatch_level
      );
    break;

    case debug_event_type_interrupt_enter:
    printk("\tEnter Int[%s] %d %d\n",
      find_interrupt_name(pEvent->interrupt_number),
      pEvent->isr_nest_level,
      pEvent->thread_dispatch_level);
    break;

    case debug_event_type_interrupt_exit:
      printk("\tExit Int[%s] %d %d\n",
        find_interrupt_name(pEvent->interrupt_number),
        pEvent->isr_nest_level,
        pEvent->thread_dispatch_level);
      break;

    default:
      break;

    }
  }
}

static void stm32fxxxx_fatal_error_handler(
  Internal_errors_Source source,
  bool is_internal,
  CPU_Exception_frame *frame
)
{
  register_double *fpu_reg;

  int *ptr = (int *)frame;
  int count=0;

  printk ("\n --- Hard Fault Occurred!!! :( --- \n");
  printk ("\n RTEMS Error Code : %s \n", rtems_fatal_source_text((rtems_fatal_source) source));
  printk (" Caused by RTEMS?: ");
  printk (is_internal == false ? "Nope\n" : "Yup\n");
  printk (" Hard fault caused by vector # %X\n", (unsigned int) frame->vector);
  printk ("\n Providing Stacked Register Details... \n");
  printk ("\n General Purpose Registers:\n");

  for(count=0;count<=12;count++)
  {
    printk ("\tr%d\t= 0x%x\n",count, (unsigned int) *(ptr+count));
  }

  printk ("\tsp\t= 0x%x -> Stack Pointer\n", (unsigned int) frame->register_sp);
  printk ("\tlr\t= 0x%x -> Subroutine Call Return Address\n", (unsigned int) frame->register_lr);
  printk ("\tpc\t= 0x%x -> Program Counter\n", (unsigned int) frame->register_pc);
  printk ("\tpsr\t= 0x%x -> Program Status Register\n", (unsigned int) frame->register_xpsr);
  printk ("\tfpexc\t= 0x%x -> Floating-Point Exception Register\n", (unsigned int) frame->register_xpsr);
  printk ("\tfpscr\t= 0x%x -> Floating-Point Status and Control Register\n", (unsigned int) frame->register_xpsr);

  ptr = &(frame->vfp_context->register_d0);

  printk ("\n Single-Precision FPU Registers:\n");

  for(count=0;count<16;count++)
  {
      fpu_reg = (ptr+(count*2));
      printk ("\ts%d\t= 0x%x\n",count, (uint64_t)fpu_reg->reg_value);
  }

  print_event_log();

  __asm__ volatile ("BKPT #01"); /* Break debugger here */
  while(1);
}

static rtems_extensions_table Extensions = {
 NULL,                      /* task create user extension */
 NULL,                      /* task start user extension */
 NULL,                      /* task restart user extension */
 NULL,                      /* task delete user extension */
 stm32fxxxx_task_switch_extension, /* task switch user extension */
 NULL,                      /* task begin user extension */
 NULL,                      /* task exitted user extension */
 stm32fxxxx_fatal_error_handler               /* fatal error user extension */
};

/**
 * Include #define CONFIGURE_MAXIMUM_USER_EXTENSIONS  8
          #define CONFIGURE_RTEMS_INIT_TASKS_TABLE
          in application code
 *
 */
rtems_status_code stm32f_initialize_user_extensions(void){

  rtems_status_code sc;
  Extension_name = rtems_build_name('E', 'X', 'U', '1');

  sc = rtems_extension_create(
      Extension_name,
      &Extensions,
      &Extension_id
  );
  return sc;

}
