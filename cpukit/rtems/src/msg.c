/**
 *  @file
 *
 *  @brief Message Queue Manager Initialization
 *  @ingroup ClassicMessageQueue
 */

/*
 *  COPYRIGHT (c) 1989-2008.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#if HAVE_CONFIG_H
#include "config.h"
#endif

#include <rtems/system.h>
#include <rtems/config.h>
#include <rtems/sysinit.h>
#include <rtems/rtems/status.h>
#include <rtems/rtems/attrimpl.h>
#include <rtems/rtems/messageimpl.h>
#include <rtems/rtems/options.h>
#include <rtems/rtems/support.h>

Objects_Information _Message_queue_Information;

static void _Message_queue_Manager_initialization(void)
{
  _Objects_Initialize_information(
    &_Message_queue_Information,  /* object information table */
    OBJECTS_CLASSIC_API,          /* object API */
    OBJECTS_RTEMS_MESSAGE_QUEUES, /* object class */
    Configuration_RTEMS_API.maximum_message_queues,
                                  /* maximum objects of this class */
    sizeof( Message_queue_Control ),
                                  /* size of this object's control block */
    false,                        /* true if names of this object are strings */
    RTEMS_MAXIMUM_NAME_LENGTH,    /* maximum length of each object's name */
    _Message_queue_MP_Send_extract_proxy
                                  /* Proxy extraction support callout */
  );

  /*
   *  Register the MP Process Packet routine.
   */

#if defined(RTEMS_MULTIPROCESSING)
  _MPCI_Register_packet_processor(
    MP_PACKET_MESSAGE_QUEUE,
    _Message_queue_MP_Process_packet
  );
#endif

}

RTEMS_SYSINIT_ITEM(
  _Message_queue_Manager_initialization,
  RTEMS_SYSINIT_CLASSIC_MESSAGE_QUEUE,
  RTEMS_SYSINIT_ORDER_MIDDLE
);
