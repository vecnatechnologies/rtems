/*----------------------------------------------------------------------------
 *      CMSIS-RTOS  -  RTX
 *----------------------------------------------------------------------------
 *      Name:    rt_CMSIS.c
 *      Purpose: CMSIS RTOS API
 *      Rev.:    V4.78
 *----------------------------------------------------------------------------
 *
 * Copyright (c) 1999-2009 KEIL, 2009-2015 ARM Germany GmbH
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  - Neither the name of ARM  nor the names of its contributors may be used 
 *    to endorse or promote products derived from this software without 
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *---------------------------------------------------------------------------*/

#define __CMSIS_GENERIC

#define os_thread_cb rtems_id
#define NUM_CMSIS_TASK_PRIORITIES 7

#include <cmsis_os.h>
#include <sched.h>
#include <rtems/score/statesimpl.h>
#include <stdio.h>

#if (osFeature_Semaphore > 65535)
#error Invalid "osFeature_Semaphore" value!
#endif


// ==== Enumeration, structures, defines ====

// Service Calls defines


#define __NO_RETURN __attribute__((noreturn))

typedef uint32_t __attribute__((vector_size(8)))  ret64;
typedef uint32_t __attribute__((vector_size(16))) ret128;

#if 0
#define RET_pointer    __r0
#define RET_int32_t    __r0
#define RET_uint32_t   __r0
#define RET_osStatus   __r0
#define RET_osPriority __r0
#define RET_osEvent    {(osStatus)__r0, {(uint32_t)__r1}, {(void *)__r2}}
#define RET_osCallback {(void *)__r0, (void *)__r1}

#define osEvent_type       __attribute__((pcs("aapcs"))) ret128
#define osEvent_ret_status (ret128){ret.status}
#define osEvent_ret_value  (ret128){ret.status, ret.value.v}
#define osEvent_ret_msg    (ret128){ret.status, ret.value.v, (uint32_t)ret.def.message_id}
#define osEvent_ret_mail   (ret128){ret.status, ret.value.v, (uint32_t)ret.def.mail_id}

#define osCallback_type    __attribute__((pcs("aapcs"))) ret64
#define osCallback_ret     (ret64) {(uint32_t)ret.fp, (uint32_t)ret.arg}

#define SVC_ArgN(n) \
  register int __r##n __asm("r"#n);

#define SVC_ArgR(n,t,a) \
  register t   __r##n __asm("r"#n) = a;

#define SVC_Arg0()                                                             \
  SVC_ArgN(0)                                                                  \
  SVC_ArgN(1)                                                                  \
  SVC_ArgN(2)                                                                  \
  SVC_ArgN(3)

#define SVC_Arg1(t1)                                                           \
  SVC_ArgR(0,t1,a1)                                                            \
  SVC_ArgN(1)                                                                  \
  SVC_ArgN(2)                                                                  \
  SVC_ArgN(3)

#define SVC_Arg2(t1,t2)                                                        \
  SVC_ArgR(0,t1,a1)                                                            \
  SVC_ArgR(1,t2,a2)                                                            \
  SVC_ArgN(2)                                                                  \
  SVC_ArgN(3)

#define SVC_Arg3(t1,t2,t3)                                                     \
  SVC_ArgR(0,t1,a1)                                                            \
  SVC_ArgR(1,t2,a2)                                                            \
  SVC_ArgR(2,t3,a3)                                                            \
  SVC_ArgN(3)

#define SVC_Arg4(t1,t2,t3,t4)                                                  \
  SVC_ArgR(0,t1,a1)                                                            \
  SVC_ArgR(1,t2,a2)                                                            \
  SVC_ArgR(2,t3,a3)                                                            \
  SVC_ArgR(3,t4,a4)

#if (defined (__CORTEX_M0))
#define SVC_Call(f)                                                            \
  __asm volatile                                                               \
  (                                                                            \
    "ldr r7,="#f"\n\t"                                                         \
    "mov r12,r7\n\t"                                                           \
    "svc 0"                                                                    \
    :               "=r" (__r0), "=r" (__r1), "=r" (__r2), "=r" (__r3)         \
    :                "r" (__r0),  "r" (__r1),  "r" (__r2),  "r" (__r3)         \
    : "r7", "r12", "lr", "cc"                                                  \
  );
#else
#define SVC_Call(f)                                                            \
  __asm volatile                                                               \
  (                                                                            \
    "ldr r12,="#f"\n\t"                                                        \
    "svc 0"                                                                    \
    :               "=r" (__r0), "=r" (__r1), "=r" (__r2), "=r" (__r3)         \
    :                "r" (__r0),  "r" (__r1),  "r" (__r2),  "r" (__r3)         \
    : "r12", "lr", "cc"                                                        \
  );
#endif

#define SVC_0_1(f,t,rv)                                                        \
__attribute__((always_inline))                                                 \
static inline  t __##f (void) {                                                \
  SVC_Arg0();                                                                  \
  SVC_Call(f);                                                                 \
  return (t) rv;                                                               \
}

#define SVC_1_0(f,t,t1)                                                        \
__attribute__((always_inline))                                                 \
static inline  t __##f (t1 a1) {                                               \
  SVC_Arg1(t1);                                                                \
  SVC_Call(f);                                                                 \
}

#define SVC_1_1(f,t,t1,rv)                                                     \
__attribute__((always_inline))                                                 \
static inline  t __##f (t1 a1) {                                               \
  SVC_Arg1(t1);                                                                \
  SVC_Call(f);                                                                 \
  return (t) rv;                                                               \
}

#define SVC_2_1(f,t,t1,t2,rv)                                                  \
__attribute__((always_inline))                                                 \
static inline  t __##f (t1 a1, t2 a2) {                                        \
  SVC_Arg2(t1,t2);                                                             \
  SVC_Call(f);                                                                 \
  return (t) rv;                                                               \
}

#define SVC_3_1(f,t,t1,t2,t3,rv)                                               \
__attribute__((always_inline))                                                 \
static inline  t __##f (t1 a1, t2 a2, t3 a3) {                                 \
  SVC_Arg3(t1,t2,t3);                                                          \
  SVC_Call(f);                                                                 \
  return (t) rv;                                                               \
}

#define SVC_4_1(f,t,t1,t2,t3,t4,rv)                                            \
__attribute__((always_inline))                                                 \
static inline  t __##f (t1 a1, t2 a2, t3 a3, t4 a4) {                          \
  SVC_Arg4(t1,t2,t3,t4);                                                       \
  SVC_Call(f);                                                                 \
  return (t) rv;                                                               \
}

#define SVC_1_2 SVC_1_1 
#define SVC_1_3 SVC_1_1 
#define SVC_2_3 SVC_2_1 
#endif


// Callback structure
typedef struct {
  void *fp;             // Function pointer
  void *arg;            // Function argument
} osCallback;


// OS Section definitions
#ifdef OS_SECTIONS_LINK_INFO
extern const uint32_t  os_section_id$$Base;
extern const uint32_t  os_section_id$$Limit;
#endif

// OS Stack Memory for Threads definitions
extern       uint64_t  os_stack_mem[];
extern const uint32_t  os_stack_sz;

// OS Timers external resources
extern const osThreadDef_t   os_thread_def_osTimerThread;
extern       osThreadId      osThreadId_osTimerThread;
extern const osMessageQDef_t os_messageQ_def_osTimerMessageQ;
extern       osMessageQId    osMessageQId_osTimerMessageQ;

static uint8_t* pThreadListBuffer = NULL;

static osStatus rtemsConvertReturnCode(
  const rtems_status_code rtems_ret
)
{
  osStatus ret;

  switch(rtems_ret) {
  case RTEMS_SUCCESSFUL:
    ret = osOK;
    break;

  case RTEMS_INVALID_ID:
    ret = osErrorParameter;
    break;

  case RTEMS_RESOURCE_IN_USE:
    ret = osErrorResource;
    break;

  case RTEMS_ILLEGAL_ON_REMOTE_OBJECT:
    ret = osErrorParameter;
    break;

  case RTEMS_TASK_EXITTED:
  case RTEMS_MP_NOT_CONFIGURED:
  case RTEMS_INVALID_NAME:
  case RTEMS_TOO_MANY:
  case RTEMS_TIMEOUT:
  case RTEMS_OBJECT_WAS_DELETED:
  case RTEMS_INVALID_SIZE:
  case RTEMS_INVALID_ADDRESS:
  case RTEMS_INVALID_NUMBER:
  case RTEMS_NOT_DEFINED:
  case RTEMS_UNSATISFIED:
  case RTEMS_INCORRECT_STATE:
  case RTEMS_ALREADY_SUSPENDED:
  case RTEMS_ILLEGAL_ON_SELF:
  case RTEMS_CALLED_FROM_ISR:
  case RTEMS_INVALID_PRIORITY:
  case RTEMS_INVALID_CLOCK:
  case RTEMS_INVALID_NODE:
  case RTEMS_NOT_CONFIGURED:
  case RTEMS_NOT_OWNER_OF_RESOURCE:
  case RTEMS_NOT_IMPLEMENTED:
  case RTEMS_INTERNAL_ERROR:
  case RTEMS_NO_MEMORY:
  case RTEMS_IO_ERROR:
  case RTEMS_PROXY_BLOCKING:
    ret = osErrorOS;
    break;
  }

  return ret;
}

// ==== Kernel Control ====

uint8_t os_initialized;                         // Kernel Initialized flag
uint8_t os_running;                             // Kernel Running flag


// Kernel Control Public API

/// Initialize the RTOS Kernel for creating objects
osStatus osKernelInitialize (void) {
#if 0
  if (__get_IPSR() != 0) return osErrorISR;     // Not allowed in ISR
  if ((__get_CONTROL() & 1) == 0) {             // Privileged mode
    return   svcKernelInitialize();
  } else {
    return __svcKernelInitialize();
  }
#else
  return (osStatus) 0;
#endif
}

/// Start the RTOS Kernel
osStatus osKernelStart (void) {
#if 0
  uint32_t stack[8];

  if (__get_IPSR() != 0) return osErrorISR;     // Not allowed in ISR
  switch (__get_CONTROL() & 0x03) {
    case 0x00:                                  // Privileged Thread mode & MSP
      __set_PSP((uint32_t)(stack + 8));         // Initial PSP
      if (os_flags & 1) {                       
        __set_CONTROL(0x02);                    // Set Privileged Thread mode & PSP
      } else {
        __set_CONTROL(0x03);                    // Set Unprivileged Thread mode & PSP
      }
      __DSB();
      __ISB();
      break;
    case 0x01:                                  // Unprivileged Thread mode & MSP
      return osErrorOS;
    case 0x02:                                  // Privileged Thread mode & PSP
      if ((os_flags & 1) == 0) {                // Unprivileged Thread mode requested
        __set_CONTROL(0x03);                    // Set Unprivileged Thread mode & PSP
        __DSB();
        __ISB();
      }
      break;
    case 0x03:                                  // Unprivileged Thread mode & PSP
      if  (os_flags & 1) return osErrorOS;      // Privileged Thread mode requested
      break;
  }
  return __svcKernelStart();
#else
  return (osStatus) 0;;
#endif
}

/// Check if the RTOS kernel is already started
int32_t osKernelRunning (void) {

#if 0
  if ((__get_IPSR() != 0) || ((__get_CONTROL() & 1) == 0)) {
    // in ISR or Privileged
    return os_running;
  } else {
    return __svcKernelRunning();
  }
#else
  return 1;
#endif
}

/// Get the RTOS kernel system timer counter
uint32_t osKernelSysTick (void) {
#if 0
  if (__get_IPSR() != 0) return 0;              // Not allowed in ISR
  return __svcKernelSysTick();
#else
  return 0UL;
#endif
}

__NO_RETURN void osThreadExit (void);

// Thread Public API

static rtems_task_priority convertTaskPriority(const osPriority priority) {

  rtems_task_priority ret;
  uint8_t task_priority_step_size = (PRIORITY_MAXIMUM - PRIORITY_MINIMUM) / (uint8_t) (NUM_CMSIS_TASK_PRIORITIES - 1);

  // In RTEMS the higher the numerical value the lower the
  // priority.  RTEMS provide macros for highest and lowest
  // priority level.
  if(priority != osPriorityError) {
    ret = (rtems_task_priority) (PRIORITY_MAXIMUM - ((uint8_t) (priority - osPriorityIdle)* task_priority_step_size));
  } else {
    stm32f_error_handler_with_reason_conditional("convertTaskPriority: Invalid task priority\n", false);
    ret = PRIORITY_DEFAULT_MAXIMUM + 1;
  }

  return ret;
}

/// Create a thread and add it to Active Threads and set it to state READY
osThreadId osThreadCreate (const osThreadDef_t *thread_def, void *argument) {

  static uint8_t cmsis_task_counter = (uint8_t) '1';

  rtems_status_code ret;
  rtems_task_priority task_priority = convertTaskPriority(thread_def->tpriority);
  rtems_id new_task_id = 0;

  if((thread_def->pthread != NULL) &&
     (task_priority  <= PRIORITY_MAXIMUM) &&
     (thread_def->instances > 0) &&
     (thread_def->stacksize > 0)){

    // Create the rtems task.
    ret = rtems_task_create(
      rtems_build_name('C', 'T', 'K', cmsis_task_counter),
      task_priority,
      (size_t) thread_def->stacksize,
      RTEMS_DEFAULT_MODES,
      RTEMS_DEFAULT_ATTRIBUTES,
      &new_task_id
    );

    // Start the task if the create was successful
    if(ret == RTEMS_SUCCESSFUL) {
      cmsis_task_counter++;

      ret = rtems_task_start(
        new_task_id,
        (rtems_task_entry) thread_def->pthread,
        (rtems_task_argument) argument
        );
    }
  }

  return (osThreadId) new_task_id;
}


/// Return the thread ID of the current running thread
osThreadId osThreadGetId (void) {
  return (osThreadId) rtems_task_self();
}


/// Terminate execution of a thread and remove it from ActiveThreads
osStatus osThreadTerminate (osThreadId thread_id) {
  osStatus ret = osOK;

  if(rtems_task_delete((rtems_id) thread_id) != RTEMS_SUCCESSFUL) {
    ret = osErrorParameter;
  }
  return ret;
}


/// Pass control to next thread that is in state READY
osStatus osThreadYield (void) {
  sched_yield();
  return osOK;
}


/// Change priority of an active thread
osStatus osThreadSetPriority (osThreadId thread_id, osPriority priority) {

  rtems_task_priority task_priority = convertTaskPriority(priority);
  rtems_task_priority old_task_priority;
  rtems_status_code   rtems_ret;

  rtems_ret = rtems_task_set_priority(
    (rtems_id) thread_id,
    task_priority,
    &old_task_priority
    );

  return rtemsConvertReturnCode(rtems_ret);
}

/// Get current priority of an active thread
osPriority osThreadGetPriority (osThreadId thread_id) {
#if 0
  if (__get_IPSR() != 0) return osPriorityError;// Not allowed in ISR
  return __svcThreadGetPriority(thread_id);
#else

  return (osPriority) 0;
#endif
}

static void osThreadStateName(
  const States_Control state,
  char* state_name
)
{
  if(_States_Is_ready(state) == true){
    sprintf(state_name, "READY");
  } else if(_States_Is_suspended(state) == true) {
    sprintf(state_name, "SUSPENDED");
  } else if(_States_Is_delaying(state) == true) {
    sprintf(state_name, "DELAYING");
  } else if(_States_Is_waiting_for_buffer(state) == true) {
    sprintf(state_name, "WAIT (BUF)");
  } else if(_States_Is_waiting_for_segment(state) == true) {
    sprintf(state_name, "WAIT (SEG)");
  } else if(_States_Is_waiting_for_message(state) == true) {
    sprintf(state_name, "WAIT (MSG)");
  } else if(_States_Is_waiting_for_event(state) == true) {
    sprintf(state_name, "WAIT (EVT)");
  } else if(_States_Is_waiting_for_system_event(state) == true) {
    sprintf(state_name, "WAIT (SEVT)");
  } else if(_States_Is_waiting_for_mutex(state) == true) {
    sprintf(state_name, "WAIT (MUX)");
  } else if(_States_Is_waiting_for_semaphore(state) == true) {
    sprintf(state_name, "WAIT (SEM)");
  } else if(_States_Is_waiting_for_time(state) == true) {
    sprintf(state_name, "WAIT (TIME)");
  } else if(_States_Is_waiting_for_rpc_reply(state) == true) {
    sprintf(state_name, "WAIT (RPC)");
  } else if(_States_Is_waiting_for_period(state) == true) {
    sprintf(state_name, "WAIT (PER)");
  } else if(_States_Is_interruptible_by_signal(state) == true) {
    sprintf(state_name, "INT (SIG)");
  } else {
    sprintf(state_name, "???");
  }
}

static void osThreadListIterator(
  Thread_Control *the_thread
)
{
  char   NextThreadInfo[128];
  char   thread_name[5];
  char   thread_state[32];

  if ( the_thread ) {

    osThreadStateName(the_thread->current_state, (char*) thread_state);
    rtems_object_get_name( the_thread->Object.id, sizeof(thread_name), thread_name );

    snprintf((char*) NextThreadInfo, sizeof(NextThreadInfo) - 1,
      "%4s\t%16s\t0x%.08X\t%6luKB\t0x%.08X \n\r",
      thread_name,
      thread_state,
      (unsigned int) the_thread->Start.stack,
      (uint32_t)    (the_thread->Start.Initial_stack.size / 1024),
      (unsigned int) the_thread->Object.id
    );
  }

  strcat((char*) pThreadListBuffer, (char*) NextThreadInfo);
}

/**
* @brief   Lists all the current threads, along with their current state
*          and stack usage high water mark.
* @param   buffer   A buffer into which the above mentioned details
*          will be written
* @retval  status code that indicates the execution status of the function.
*/
osStatus osThreadList (uint8_t *buffer)
{
  pThreadListBuffer = buffer;
  rtems_iterate_over_all_threads( osThreadListIterator );

  return osOK;
}

/// INTERNAL - Not Public
/// Auto Terminate Thread on exit (used implicitly when thread exists)
__NO_RETURN void osThreadExit (void) { 

  rtems_id my_task_id;

  if(rtems_task_ident(RTEMS_SELF, RTEMS_SEARCH_ALL_NODES, &my_task_id) == RTEMS_SUCCESSFUL) {
    rtems_task_delete(my_task_id);
  }

  for(;;) {
    // should never get here
  }
}

// Generic Wait API

/// Wait for Timeout (Time Delay)
osStatus osDelay (uint32_t millisec) {
  rtems_interval    ticks_per_millisec;

  ticks_per_millisec = rtems_clock_get_ticks_per_second() / 1000;

  (void) rtems_task_wake_after( millisec * ticks_per_millisec );

  return osOK;
}

/// Wait for Signal, Message, Mail, or Timeout
os_InRegs osEvent osWait (uint32_t millisec) {
#if 0
  osEvent ret;

#if osFeature_Wait == 0
  ret.status = osErrorOS;
  return ret;
#else
  if (__get_IPSR() != 0) {                      // Not allowed in ISR
    ret.status = osErrorISR;
    return ret;
  }
  return __svcWait(millisec);
#endif
#else
  osEvent ret = {osOK, {0},{0}};
  return ret;
#endif
}


// ==== Timer Management ====

// Timer definitions
#define osTimerInvalid  0
#define osTimerStopped  1
#define osTimerRunning  2

// Timer structures 

typedef struct os_timer_cb_ {                   // Timer Control Block
  struct os_timer_cb_ *next;                    // Pointer to next active Timer
  uint8_t             state;                    // Timer State
  uint8_t              type;                    // Timer Type (Periodic/One-shot)
  uint16_t         reserved;                    // Reserved
  uint32_t             tcnt;                    // Timer Delay Count
  uint32_t             icnt;                    // Timer Initial Count 
  void                 *arg;                    // Timer Function Argument
  const osTimerDef_t *timer;                    // Pointer to Timer definition
} os_timer_cb;

// Timer variables
os_timer_cb *os_timer_head;                     // Pointer to first active Timer


// Timer Management Public API

/// Create timer
osTimerId osTimerCreate (const osTimerDef_t *timer_def, os_timer_type type, void *argument) {
#if 0
  if (__get_IPSR() != 0) return NULL;           // Not allowed in ISR
  if (((__get_CONTROL() & 1) == 0) && (os_running == 0)) {
    // Privileged and not running
    return   svcTimerCreate(timer_def, type, argument);
  } else {
    return __svcTimerCreate(timer_def, type, argument);
  }
#else
  return (osTimerId) 0;
#endif
}

/// Start or restart timer
osStatus osTimerStart (osTimerId timer_id, uint32_t millisec) {
#if 0
  if (__get_IPSR() != 0) return osErrorISR;     // Not allowed in ISR
  return __svcTimerStart(timer_id, millisec);
#else
  return (osStatus) 0;
#endif
}

/// Stop timer
osStatus osTimerStop (osTimerId timer_id) {
#if 0
  if (__get_IPSR() != 0) return osErrorISR;     // Not allowed in ISR
  return __svcTimerStop(timer_id);
#else
  return osOK;
#endif
}

/// Delete timer
osStatus osTimerDelete (osTimerId timer_id) {
#if 0
  if (__get_IPSR() != 0) return osErrorISR;     // Not allowed in ISR
  return __svcTimerDelete(timer_id);
#else
  return osOK;
#endif
}


// Timer Thread
#if 0
__NO_RETURN void osTimerThread (void const *argument) {

#if 0
  osCallback cb;
  osEvent    evt;

  for (;;) {
    evt = osMessageGet(osMessageQId_osTimerMessageQ, osWaitForever);
    if (evt.status == osEventMessage) {osMailQId
      cb = osTimerCall(evt.value.p);
      if (cb.fp != NULL) {
        (*(os_ptimer)cb.fp)(cb.arg);
      }
    }
  }
#endif
}
#endif



// Signal Public API

/// Set the specified Signal Flags of an active thread
int32_t osSignalSet (osThreadId thread_id, int32_t signals) {
#if 0
  if (__get_IPSR() != 0) {                      // in ISR
    return   isrSignalSet(thread_id, signals); 
  } else {                                      // in Thread
    return __svcSignalSet(thread_id, signals);
  }
#else
  return 0;
#endif
}

/// Clear the specified Signal Flags of an active thread
int32_t osSignalClear (osThreadId thread_id, int32_t signals) {
#if 0
  if (__get_IPSR() != 0) return 0x80000000;     // Not allowed in ISR
  return __svcSignalClear(thread_id, signals);
#else
  return 0l;
#endif
}

/// Wait for one or more Signal Flags to become signaled for the current RUNNING thread
os_InRegs osEvent osSignalWait (int32_t signals, uint32_t millisec) {
#if 0
  osEvent ret;

  if (__get_IPSR() != 0) {                      // Not allowed in ISR
    ret.status = osErrorISR;
    return ret;
  }
  return __svcSignalWait(signals, millisec);
#else
  osEvent ret = {osOK, {0},{0}};
  return ret;
#endif
}


// Mutex Public API

/// Create and Initialize a Mutex object
osMutexId osMutexCreate (const osMutexDef_t *mutex_def) {

  static uint8_t cmsis_mutex_counter = (uint8_t) 1;

  rtems_attribute   mutex_attributes;
  rtems_status_code ret;
  rtems_id          mutex_id;

   mutex_attributes =  RTEMS_PRIORITY | RTEMS_LOCAL | RTEMS_INHERIT_PRIORITY | RTEMS_BINARY_SEMAPHORE |
                       RTEMS_NO_PRIORITY_CEILING | RTEMS_NO_MULTIPROCESSOR_RESOURCE_SHARING;

   ret = rtems_semaphore_create  (
    rtems_build_name('C', 'M', 'X', cmsis_mutex_counter),
    1,
    mutex_attributes,
    PRIORITY_DEFAULT_MAXIMUM,
    &mutex_id
  );

  if(ret == RTEMS_SUCCESSFUL){
    cmsis_mutex_counter++;
    return (osMutexId) mutex_id;
  } else {
    return 0;
  }
}

/// Wait until a Mutex becomes available
osStatus osMutexWait (osMutexId mutex_id, uint32_t millisec) {
  return osSemaphoreWait((osSemaphoreId) mutex_id, millisec);
}

/// Release a Mutex that was obtained with osMutexWait
osStatus osMutexRelease (osMutexId mutex_id) {
  return osSemaphoreRelease((osSemaphoreId) mutex_id);
}

/// Delete a Mutex that was created by osMutexCreate
osStatus osMutexDelete (osMutexId mutex_id) {
  return osSemaphoreDelete((osSemaphoreId) mutex_id);
}


// Semaphore Public API

/// Create and Initialize a Semaphore object
osSemaphoreId osSemaphoreCreate (const osSemaphoreDef_t *semaphore_def, int32_t count) {
  static uint8_t cmsis_semaphore_counter = (uint8_t) 1;

  rtems_attribute sema_attributes;
  rtems_status_code ret;
  rtems_id          semaphore_id;

  if(count > 1) {
    sema_attributes = RTEMS_DEFAULT_ATTRIBUTES;
  } else {
    sema_attributes = RTEMS_SIMPLE_BINARY_SEMAPHORE;
  }

   ret = rtems_semaphore_create  (
    rtems_build_name('C', 'S', 'M', cmsis_semaphore_counter),
    count,
    sema_attributes,
    RTEMS_NO_PRIORITY,
    &semaphore_id
  );

  if(ret == RTEMS_SUCCESSFUL){
    cmsis_semaphore_counter++;
    return (osSemaphoreId) semaphore_id;
  } else {
    return 0;
  }
}

/// Wait until a Semaphore becomes available
int32_t osSemaphoreWait (osSemaphoreId semaphore_id, uint32_t millisec) {

  rtems_status_code ret;
  rtems_interval ticks_per_millisec = rtems_clock_get_ticks_per_second() / 1000UL;

  // wait for specified semaphore for the maximum amount of time
  ret = rtems_semaphore_obtain ((rtems_id) semaphore_id, RTEMS_WAIT, (ticks_per_millisec * millisec));

  if(ret == RTEMS_SUCCESSFUL) {
    return osOK;
  } else {
    return -1;
  }
}

/// Release a Semaphore
osStatus osSemaphoreRelease (osSemaphoreId semaphore_id) {
  return rtemsConvertReturnCode(rtems_semaphore_release((rtems_id) semaphore_id));
}

/// Delete a Semaphore that was created by osSemaphoreCreate
osStatus osSemaphoreDelete (osSemaphoreId semaphore_id) {
  return rtemsConvertReturnCode(rtems_semaphore_delete((rtems_id) semaphore_id));
}

// ==== Memory Management Functions ====

// Memory Management Public API

/// Create and Initialize memory pool
osPoolId osPoolCreate (const osPoolDef_t *pool_def) {
#if 0
  if (__get_IPSR() != 0) return NULL;           // Not allowed in ISR
  if (((__get_CONTROL() & 1) == 0) && (os_running == 0)) {
    // Privileged and not running
    return   svcPoolCreate(pool_def);
  } else {
    return __svcPoolCreate(pool_def);
  }
#else
  return (osPoolId) 0;
#endif
}

/// Allocate a memory block from a memory pool
void *osPoolAlloc (osPoolId pool_id) {
#if 0
  if ((__get_IPSR() != 0) || ((__get_CONTROL() & 1) == 0)) {    // in ISR or Privileged
    return   sysPoolAlloc(pool_id, 0);
  } else {                                      // in Thread
    return __sysPoolAlloc(pool_id, 0);
  }
#else
  return NULL;
#endif
}

/// Allocate a memory block from a memory pool and set memory block to zero
void *osPoolCAlloc (osPoolId pool_id) {
#if 0
  if ((__get_IPSR() != 0) || ((__get_CONTROL() & 1) == 0)) {    // in ISR or Privileged
    return   sysPoolAlloc(pool_id, 1);
  } else {                                      // in Thread
    return __sysPoolAlloc(pool_id, 1);
  }
#else
  return NULL;
#endif
}

/// Return an allocated memory block back to a specific memory pool
osStatus osPoolFree (osPoolId pool_id, void *block) {
#if 0
  if ((__get_IPSR() != 0) || ((__get_CONTROL() & 1) == 0)) {    // in ISR or Privileged
    return   sysPoolFree(pool_id, block);
  } else {                                      // in Thread
    return __sysPoolFree(pool_id, block);
  }
#else
  return (osStatus) 0;
#endif
}


// ==== Message Queue Management Functions ====


// Message Queue Management Public API

/// Create and Initialize Message Queue
osMessageQId osMessageCreate (const osMessageQDef_t *queue_def, osThreadId thread_id) {
#if 0
  if (__get_IPSR() != 0) return NULL;           // Not allowed in ISR
  if (((__get_CONTROL() & 1) == 0) && (os_running == 0)) {
    // Privileged and not running
    return   svcMessageCreate(queue_def, thread_id);
  } else {
    return __svcMessageCreate(queue_def, thread_id);
  }
#else
  return (osMessageQId) 0;
#endif
}

/// Put a Message to a Queue
osStatus osMessagePut (osMessageQId queue_id, uint32_t info, uint32_t millisec) {
#if 0
  if (__get_IPSR() != 0) {                      // in ISR
    return   isrMessagePut(queue_id, info, millisec);
  } else {                                      // in Thread
    return __svcMessagePut(queue_id, info, millisec);
  }
#else
  return (osStatus) 0;
#endif
}

/// Get a Message or Wait for a Message from a Queue
os_InRegs osEvent osMessageGet (osMessageQId queue_id, uint32_t millisec) {
#if 0
  if (__get_IPSR() != 0) {                      // in ISR
    return   isrMessageGet(queue_id, millisec);
  } else {                                      // in Thread
    return __svcMessageGet(queue_id, millisec);
  }
#else
  osEvent ret = {osOK, {0},{0}};
  return ret;
#endif
}


// ==== Mail Queue Management Functions ====



// Mail Queue Management Public API

/// Create and Initialize mail queue
osMailQId osMailCreate (const osMailQDef_t *queue_def, osThreadId thread_id) {
#if 0
  if (__get_IPSR() != 0) return NULL;           // Not allowed in ISR
  if (((__get_CONTROL() & 1) == 0) && (os_running == 0)) {
    // Privileged and not running
    return   svcMailCreate(queue_def, thread_id);
  } else {
    return __svcMailCreate(queue_def, thread_id);
  }
#else
  return (osMailQId)0;
#endif
}

/// Allocate a memory block from a mail
void *osMailAlloc (osMailQId queue_id, uint32_t millisec) {
#if 0
  if (__get_IPSR() != 0) {                      // in ISR
    return   sysMailAlloc(queue_id, millisec, 1, 0);
  } else {                                      // in Thread
    return __sysMailAlloc(queue_id, millisec, 0, 0);
  }
#else
  return NULL;
#endif
}

/// Allocate a memory block from a mail and set memory block to zero
void *osMailCAlloc (osMailQId queue_id, uint32_t millisec) {
#if 0
  if (__get_IPSR() != 0) {                      // in ISR
    return   sysMailAlloc(queue_id, millisec, 1, 1);
  } else {                                      // in Thread
    return __sysMailAlloc(queue_id, millisec, 0, 1);
  }
#else
  return NULL;
#endif
}

/// Free a memory block from a mail
osStatus osMailFree (osMailQId queue_id, void *mail) {
#if 0
  if (__get_IPSR() != 0) {                      // in ISR
    return   sysMailFree(queue_id, mail, 1);
  } else {                                      // in Thread
    return __sysMailFree(queue_id, mail, 0);
  }
#else
  return osOK;
#endif
}

/// Put a mail to a queue
osStatus osMailPut (osMailQId queue_id, void *mail) {
#if 0
  if (queue_id == NULL) return osErrorParameter;
  if (mail == NULL)     return osErrorValue;
  return osMessagePut(*((void **)queue_id), (uint32_t)mail, 0);
#else
  return osOK;
#endif
}

/// Get a mail from a queue
os_InRegs osEvent osMailGet (osMailQId queue_id, uint32_t millisec) {
#if 0
  osEvent ret;

  if (queue_id == NULL) {
    ret.status = osErrorParameter;
    return ret;
  }

  ret = osMessageGet(*((void **)queue_id), millisec);
  if (ret.status == osEventMessage) ret.status = osEventMail;

  return ret;
#else
  osEvent ret = {osOK, {0},{0}};
  return ret;
  #endif
}


// Public API
#if 0
/// Suspends the OS task scheduler
uint32_t os_suspend (void) {
  //return __rt_suspend();
}

/// Resumes the OS task scheduler
void os_resume (uint32_t sleep_time) {
  //__rt_resume(sleep_time);
}
#endif
