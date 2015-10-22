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

#define TASK_SWITCH_LOG_LENGTH 20

typedef struct  {
    char executing_task[4];
    char heir_task[4];
} task_switch_log_entry_t;

typedef struct  {
    task_switch_log_entry_t task_switch_log_entry[TASK_SWITCH_LOG_LENGTH];
    uint32_t task_log_index;
} task_switch_log_t;

static task_switch_log_t task_switch_log = {{0},0};

/* Extension name */
rtems_name Extension_name;

/* Extension ID */
rtems_id Extension_id;

typedef struct
{
  uint64_t reg_value;
}register_double;



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


static void stm32fxxxx_task_switch_extension (
  Thread_Control *executing,
  Thread_Control *heir
)
{
  if(task_switch_log.task_log_index < COUNTOF(task_switch_log.task_switch_log_entry)) {

    copy_task_name(executing->Object.name.name_u32,
      (char*) task_switch_log.task_switch_log_entry[task_switch_log.task_log_index].executing_task);

    copy_task_name(heir->Object.name.name_u32,
      (char*) task_switch_log.task_switch_log_entry[task_switch_log.task_log_index].heir_task);

    task_switch_log.task_log_index = (task_switch_log.task_log_index + 1) % COUNTOF(task_switch_log.task_switch_log_entry);
  }
}

void print_task_switch_log(void)
{
  uint32_t i;
  uint32_t log_index;

  log_index = task_switch_log.task_log_index;

  printf("Context switch log\n");

  for(i = 0UL; i < COUNTOF(task_switch_log.task_switch_log_entry); i++) {

    if(log_index == 0) {
      log_index = COUNTOF(task_switch_log.task_switch_log_entry) - 1;
    } else {
      log_index--;
    }

    printf("\t%c%c%c%c -> %c%c%c%c\n",
      task_switch_log.task_switch_log_entry[log_index].executing_task[0],
      task_switch_log.task_switch_log_entry[log_index].executing_task[1],
      task_switch_log.task_switch_log_entry[log_index].executing_task[2],
      task_switch_log.task_switch_log_entry[log_index].executing_task[3],
      task_switch_log.task_switch_log_entry[log_index].heir_task[0],
      task_switch_log.task_switch_log_entry[log_index].heir_task[1],
      task_switch_log.task_switch_log_entry[log_index].heir_task[2],
      task_switch_log.task_switch_log_entry[log_index].heir_task[3]);
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

  printf ("\n --- Hard Fault Occurred!!! :( --- \n");
  printf ("\n RTEMS Error Code : %s \n", rtems_fatal_source_text((rtems_fatal_source) source));
  printf (" Caused by RTEMS?: ");
  printf (is_internal == false ? "Nope\n" : "Yup\n");
  printf (" Hard fault caused by vector # %X\n", (unsigned int) frame->vector);
  printf ("\n Providing Stacked Register Details... \n");
  printf ("\n General Purpose Registers:\n");

  for(count=0;count<=12;count++)
  {
    printf ("\tr%d\t= 0x%.08X\n",count, (unsigned int) *(ptr+count));
  }

  printf ("\tsp\t= 0x%.08X -> Stack Pointer\n", (unsigned int) frame->register_sp);
  printf ("\tlr\t= 0x%.08X -> Subroutine Call Return Address\n", (unsigned int) frame->register_lr);
  printf ("\tpc\t= 0x%.08X -> Program Counter\n", (unsigned int) frame->register_pc);
  printf ("\tpsr\t= 0x%.08X -> Program Status Register\n", (unsigned int) frame->register_xpsr);
  printf ("\tfpexc\t= 0x%.08X -> Floating-Point Exception Register\n", (unsigned int) frame->register_xpsr);
  printf ("\tfpscr\t= 0x%.08X -> Floating-Point Status and Control Register\n", (unsigned int) frame->register_xpsr);

  ptr = &(frame->vfp_context->register_d0);

  printf ("\n Single-Precision FPU Registers:\n");

  for(count=0;count<16;count++)
  {
      fpu_reg = (ptr+(count*2));
      printf ("\ts%d\t= %-20e\n",count, (uint64_t)fpu_reg->reg_value);
  }

  print_task_switch_log();

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
rtems_status_code stm32_initialize_extensions(void){

  rtems_status_code sc;
  Extension_name = rtems_build_name('E', 'X', 'U', '1');

  sc = rtems_extension_create(
      Extension_name,
      &Extensions,
      &Extension_id
  );
  return sc;
}
