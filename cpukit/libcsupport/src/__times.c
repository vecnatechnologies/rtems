/**
 *  @file
 *
 *  @brief Get Process Times
 *  @ingroup libcsupport
 */

/*
 *  COPYRIGHT (c) 1989-2013.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#if HAVE_CONFIG_H
#include "config.h"
#endif

/*
 *  Needed to get the prototype for this newlib helper method
 */
#define _COMPILING_NEWLIB

#include <rtems.h>

#include <sys/times.h>
#include <sys/time.h>

#include <string.h>
#include <time.h>

#include <rtems/seterr.h>
#include <rtems/score/todimpl.h>
#include <rtems/score/timestamp.h>
#include <rtems/score/threadimpl.h>

/**
 *  POSIX 1003.1b 4.5.2 - Get Process Times
 */
clock_t _times(
   struct tms  *ptms
)
{
  rtems_interval ticks, us_per_tick;

  if ( !ptms )
    rtems_set_errno_and_return_minus_one( EFAULT );

  memset( ptms, 0, sizeof( *ptms ) );

  /*
   *  This call does not depend on TOD being initialized and can't fail.
   */

  ticks = rtems_clock_get_ticks_since_boot();
  us_per_tick = rtems_configuration_get_microseconds_per_tick();

  /*
   *  RTEMS technically has no notion of system versus user time
   *  since there is no separation of OS from application tasks.
   *  But we can at least make a distinction between the number
   *  of ticks since boot and the number of ticks executed by this
   *  this thread.
   */
  {
    Timestamp_Control  cpu_time_used;
    Timestamp_Control  per_tick;
    uint32_t           ticks_of_executing;
    uint32_t           fractional_ticks;

    _Thread_Get_CPU_time_used( _Thread_Get_executing(), &cpu_time_used );
    _Timestamp_Set(
      &per_tick,
      rtems_configuration_get_microseconds_per_tick() /
	  TOD_MICROSECONDS_PER_SECOND,
      (rtems_configuration_get_nanoseconds_per_tick() %
	  TOD_NANOSECONDS_PER_SECOND)
    );
    _Timestamp_Divide(
      &cpu_time_used,
      &per_tick,
      &ticks_of_executing,
      &fractional_ticks
    );

    ptms->tms_utime = ticks_of_executing * us_per_tick;
  }

  ptms->tms_stime  = ticks * us_per_tick;

  return ticks * us_per_tick;
}

/**
 *  times() system call wrapper for _times() above.
 */
clock_t times(
   struct tms  *ptms
)
{
  return _times( ptms );
}

#if defined(RTEMS_NEWLIB)

#include <reent.h>

/**
 *  This is the Newlib dependent reentrant version of times().
 */
clock_t _times_r(
   struct _reent *ptr RTEMS_UNUSED,
   struct tms  *ptms
)
{
  return _times( ptms );
}
#endif
