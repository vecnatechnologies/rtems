/**
 *  @file
 *
 *  @brief Constants and Structures Needed to Declare a Thread Queue
 *
 *  This include file contains all the constants and structures
 *  needed to declare a thread queue.
 */

/*
 *  COPYRIGHT (c) 1989-2014.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#ifndef _RTEMS_SCORE_THREADQ_H
#define _RTEMS_SCORE_THREADQ_H

#include <rtems/score/chain.h>
#include <rtems/score/isrlock.h>
#include <rtems/score/priority.h>
#include <rtems/score/rbtree.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 *  @defgroup ScoreThreadQueue Thread Queue Handler
 *
 *  @ingroup Score
 *
 *  This handler provides the capability to have threads block in
 *  ordered sets. The sets may be ordered using the FIFO or priority
 *  discipline.
 */
/**@{*/

typedef struct _Thread_Control Thread_Control;

/**
 * @brief Thread priority queue.
 */
typedef struct {
#if defined(RTEMS_SMP)
  /**
   * @brief Node to enqueue this queue in the FIFO chain of the corresponding
   * heads structure.
   *
   * @see Thread_queue_Heads::Heads::Fifo.
   */
  Chain_Node Node;
#endif

  /**
   * @brief The actual thread priority queue.
   */
  RBTree_Control Queue;
} Thread_queue_Priority_queue;

/**
 * @brief Thread queue heads.
 *
 * Each thread is equipped with spare thread queue heads in case it is not
 * enqueued on a thread queue.  The first thread enqueued on a thread queue
 * will give its spare thread queue heads to that thread queue.  The threads
 * arriving at the queue will add their thread queue heads to the free chain of
 * the queue heads provided by the first thread enqueued.  Once a thread is
 * dequeued it use the free chain to get new spare thread queue heads.
 *
 * Uses a leading underscore in the structure name to allow forward
 * declarations in standard header files provided by Newlib and GCC.
 */
typedef struct _Thread_queue_Heads {
  /** This union contains the data structures used to manage the blocked
   *  set of tasks which varies based upon the discipline.
   */
  union {
    /**
     * @brief This is the FIFO discipline list.
     *
     * On SMP configurations this FIFO is used to enqueue the per scheduler
     * instance priority queues of this structure.  This ensures FIFO fairness
     * among the highest priority thread of each scheduler instance.
     */
    Chain_Control Fifo;

#if !defined(RTEMS_SMP)
    /**
     * @brief This is the set of threads for priority discipline waiting.
     */
    Thread_queue_Priority_queue Priority;
#endif
  } Heads;

  /**
   * @brief A chain with free thread queue heads providing the spare thread
   * queue heads for a thread once it is dequeued.
   */
  Chain_Control Free_chain;

  /**
   * @brief A chain node to add these thread queue heads to the free chain of
   * the thread queue heads dedicated to the thread queue of an object.
   */
  Chain_Node Free_node;

#if defined(RTEMS_SMP)
  /**
   * @brief One priority queue per scheduler instance.
   */
  Thread_queue_Priority_queue Priority[ RTEMS_ZERO_LENGTH_ARRAY ];
#endif
} Thread_queue_Heads;

#if defined(RTEMS_SMP)
  #define THREAD_QUEUE_HEADS_SIZE( scheduler_count ) \
    ( sizeof( Thread_queue_Heads ) \
      + ( scheduler_count ) * sizeof( Thread_queue_Priority_queue ) )
#else
  #define THREAD_QUEUE_HEADS_SIZE( scheduler_count ) \
    sizeof( Thread_queue_Heads )
#endif

typedef struct {
  /**
   * @brief The thread queue heads.
   *
   * This pointer is NULL, if and only if no threads are enqueued.  The first
   * thread to enqueue will give its spare thread queue heads to this thread
   * queue.
   */
  Thread_queue_Heads *heads;

  /**
   * @brief Lock to protect this thread queue.
   *
   * It may be used to protect additional state of the object embedding this
   * thread queue.
   *
   * @see _Thread_queue_Acquire(), _Thread_queue_Acquire_critical() and
   * _Thread_queue_Release().
   */
#if defined(RTEMS_SMP)
  SMP_ticket_lock_Control Lock;
#endif
} Thread_queue_Queue;

/**
 * @brief Thread queue priority change operation.
 *
 * @param[in] the_thread The thread.
 * @param[in] new_priority The new priority value.
 * @param[in] queue The actual thread queue.
 *
 * @see Thread_queue_Operations.
 */
typedef void ( *Thread_queue_Priority_change_operation )(
  Thread_Control     *the_thread,
  Priority_Control    new_priority,
  Thread_queue_Queue *queue
);

/**
 * @brief Thread queue enqueue operation.
 *
 * @param[in] queue The actual thread queue.
 * @param[in] the_thread The thread to enqueue on the queue.
 *
 * @see _Thread_Wait_set_operations().
 */
typedef void ( *Thread_queue_Enqueue_operation )(
  Thread_queue_Queue *queue,
  Thread_Control     *the_thread
);

/**
 * @brief Thread queue extract operation.
 *
 * @param[in] queue The actual thread queue.
 * @param[in] the_thread The thread to extract from the thread queue.
 *
 * @see _Thread_Wait_set_operations().
 */
typedef void ( *Thread_queue_Extract_operation )(
  Thread_queue_Queue *queue,
  Thread_Control     *the_thread
);

/**
 * @brief Thread queue first operation.
 *
 * @param[in] heads The thread queue heads.
 *
 * @retval NULL No thread is present on the thread queue.
 * @retval first The first thread of the thread queue according to the insert
 * order.  This thread remains on the thread queue.
 *
 * @see _Thread_Wait_set_operations().
 */
typedef Thread_Control *( *Thread_queue_First_operation )(
  Thread_queue_Heads *heads
);

/**
 * @brief Thread queue operations.
 *
 * @see _Thread_wait_Set_operations().
 */
typedef struct {
  /**
   * @brief Thread queue priority change operation.
   *
   * Called by _Thread_Change_priority() to notify a thread about a priority
   * change.  In case this thread waits currently for a resource the handler
   * may adjust its data structures according to the new priority value.  This
   * handler must not be NULL, instead the default handler
   * _Thread_Do_nothing_priority_change() should be used in case nothing needs
   * to be done during a priority change.
   */
  Thread_queue_Priority_change_operation priority_change;

  /**
   * @brief Thread queue enqueue operation.
   *
   * Called by object routines to enqueue the thread.
   */
  Thread_queue_Enqueue_operation enqueue;

  /**
   * @brief Thread queue extract operation.
   *
   * Called by object routines to extract a thread from a thread queue.
   */
  Thread_queue_Extract_operation extract;

  /**
   * @brief Thread queue first operation.
   */
  Thread_queue_First_operation first;
} Thread_queue_Operations;

/**
 *  The following enumerated type details all of the disciplines
 *  supported by the Thread Queue Handler.
 */
typedef enum {
  THREAD_QUEUE_DISCIPLINE_FIFO,     /* FIFO queue discipline */
  THREAD_QUEUE_DISCIPLINE_PRIORITY  /* PRIORITY queue discipline */
}   Thread_queue_Disciplines;

/**
 *  This is the structure used to manage sets of tasks which are blocked
 *  waiting to acquire a resource.
 */
typedef struct {
  /**
   * @brief The actual thread queue.
   */
  Thread_queue_Queue Queue;

#if defined(RTEMS_SMP) && defined(RTEMS_PROFILING)
  SMP_lock_Stats Lock_stats;
#endif

  /**
   * @brief The operations for the actual thread queue.
   */
  const Thread_queue_Operations *operations;
} Thread_queue_Control;

/**@}*/

#ifdef __cplusplus
}
#endif

#endif
/* end of include file */
