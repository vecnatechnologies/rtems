/**
 * @file cmsis_unit.c
 * @author Jay M. Doyle
 *
 * @ingroup test
 *
 * @brief CppUnit test code for testing the functions implemented in
 *   cmsis-os.c
 *
 */

/*
 * Copyright (c) 2015 Vecna Technologies, Inc.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.com/license/LICENSE.
 */

#include <CppUTest/TestHarness.h>
#include <stdio.h>

extern "C"
{
#include <cmsis_os.h>
}

TEST_GROUP( cmsis_os_unit )
{
  void setup()
  {
  }

  void teardown()
  {
  }
};

static void sample_task( const void *pArg )
{
  printf( "sample_task executing...\n" );
}

TEST( cmsis_os_unit, osSemaphoreRelease )
{
  osSemaphoreId s_xSemaphore = NULL;
  int           ret;

  osSemaphoreDef( SEM );
  s_xSemaphore = osSemaphoreCreate( osSemaphore( SEM ), 0 );
  CHECK_TEXT( s_xSemaphore != 0, "osSemaphore returned an invalid value (0)" );

  // This wait should timeout since semaphore was created with an initial value of 0
  ret = osSemaphoreWait( s_xSemaphore, 1 );
  CHECK_TEXT( ret == osEventTimeout,
    "osSemaphoreWait did not wait as expected" );

  // Release semaphore
  ret = osSemaphoreRelease( s_xSemaphore );
  CHECK_TEXT( ret == osOK,
    "osSemaphoreRelease did not return osOK as expected" );

  // Now check that semaphore is avaiable as expected
  ret = osSemaphoreWait( s_xSemaphore, 1 );
  CHECK_TEXT( ret == osOK,
    "osSemaphoreWait did return immediately as expected" );
}

TEST( cmsis_os_unit, osSemaphoreWait )
{
  osSemaphoreId s_xSemaphore = NULL;

  osSemaphoreDef( SEM );
  s_xSemaphore = osSemaphoreCreate( osSemaphore( SEM ), 0 );
  CHECK_TEXT( s_xSemaphore != 0, "osSemaphore returned an invalid value (0)" );

  // This wait should timeout since semaphore was created with an initial value of 0
  int ret = osSemaphoreWait( s_xSemaphore, 1 );
  CHECK_TEXT( ret == osEventTimeout,
    "osSemaphoreWait did not wait as expected" );
}

TEST( cmsis_os_unit, osSemaphoreCreate )
{
  osSemaphoreId s_xSemaphore = NULL;

  osSemaphoreDef( SEM );
  s_xSemaphore = osSemaphoreCreate( osSemaphore( SEM ), 0 );
  CHECK_TEXT( s_xSemaphore != 0, "osSemaphore returned an invalid value (0)" );
}

TEST( cmsis_os_unit, osThreadGetId )
{
  rtems_id my_thread_id = 0UL;

  my_thread_id = (rtems_id) osThreadGetId();
  UNSIGNED_LONGS_EQUAL_TEXT(
    rtems_task_self(), my_thread_id,
    "osThreadGetId did not returns rtems_id of current task" );
}

TEST( cmsis_os_unit, osThreadSetPriority_osThreadGetPriority )
{
  osThreadDef_t thread_def;
  osThreadId    thread_id;
  osStatus      ret;
  osPriority    return_priority;

  thread_def.pthread = sample_task;
  thread_def.tpriority = osPriorityLow;
  thread_def.instances = 1;
  thread_def.stacksize = 1024;

  // Check for non-null ptThread definition
  thread_id = osThreadCreate( &thread_def, NULL );
  CHECK_TEXT( thread_id != 0, "osThreadCreate returned a null value" );

  ret = osThreadSetPriority( thread_id, osPriorityBelowNormal );
  UNSIGNED_LONGS_EQUAL_TEXT( osOK,
    ret,
    "osThreadSetPriority returned unexpected value" );

  return_priority = osThreadGetPriority( thread_id );
  UNSIGNED_LONGS_EQUAL_TEXT( osPriorityBelowNormal,
    return_priority,
    "osThreadGetPriority returned unexpected value" );

  // Check for valid terminate
  ret = osThreadTerminate( thread_id );
  UNSIGNED_LONGS_EQUAL_TEXT( osOK,
    ret,
    "osThreadTerminate returned non osOK return value" );
}

TEST( cmsis_os_unit, osThreadCreate_and_Terminate )
{
  osThreadDef_t thread_def;
  osThreadId    thread_id;
  osStatus      ret;

  thread_def.pthread = NULL;
  thread_def.tpriority = osPriorityLow;
  thread_def.instances = 1;
  thread_def.stacksize = 1024;

  // Check for non-null ptThread definition
  thread_id = osThreadCreate( &thread_def, NULL );
  CHECK_TEXT( thread_id == 0,
    "osThreadCreate returned a non-null value when created with an invalid pthread value" );

  thread_def.pthread = sample_task;
  thread_def.tpriority = osPriorityError;
  thread_def.instances = 1;
  thread_def.stacksize = 1024;

  // Check for invalid priority
  thread_id = osThreadCreate( &thread_def, NULL );
  CHECK_TEXT( thread_id == 0,
    "osThreadCreate returned a non-null value when created with an invalid priority" );

  thread_def.pthread = sample_task;
  thread_def.tpriority = osPriorityLow;
  thread_def.instances = 0;
  thread_def.stacksize = 1024;

  // Check for invalid instance number
  thread_id = osThreadCreate( &thread_def, NULL );
  CHECK_TEXT( thread_id == 0,
    "osThreadCreate returned a non-null value when created with a 0 instance count" );

  thread_def.pthread = sample_task;
  thread_def.tpriority = osPriorityLow;
  thread_def.instances = 1;
  thread_def.stacksize = 0;

  // Check for invalid stack size
  thread_id = osThreadCreate( &thread_def, NULL );
  CHECK_TEXT( thread_id == 0,
    "osThreadCreate returned a non-null value when created with an invalid stack size" );

  thread_def.pthread = sample_task;
  thread_def.tpriority = osPriorityLow;
  thread_def.instances = 1;
  thread_def.stacksize = 1024;

  // Check for valid thread creation
  thread_id = osThreadCreate( &thread_def, NULL );
  CHECK_TEXT( thread_id != 0,
    "osThreadCreate returned null task id unexpectedly" );

  // Check for valid terminate
  ret = osThreadTerminate( thread_id );
  CHECK_TEXT( ret == osOK,
    "osThreadTerminate returned non osOK return value" );

  // Check for invalid terminate
  ret = osThreadTerminate( thread_id );
  CHECK_TEXT( ret == osErrorParameter,
    "osThreadTerminate osErrorParameter not returned when called on not-existent thread" );
}

TEST( cmsis_os_unit, osDelay )
{
  rtems_interval ticks_per_millisec;
  rtems_interval start_time;
  rtems_interval end_time;

  ticks_per_millisec = rtems_clock_get_ticks_per_second() / 1000;

  start_time = rtems_clock_get_ticks_since_boot();
  osDelay( 100 );
  end_time = rtems_clock_get_ticks_since_boot();
  CHECK_TEXT( ( ( end_time - start_time ) / ticks_per_millisec ) >= 100,
    "osDelay did not delay did not wait 100ms as expected" )
}

