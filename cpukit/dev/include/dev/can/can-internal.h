/**
 * @file can.h
 *
 * @brief Control Area Network  (can) Driver API
 *
 * @ingroup can
 */

/*
 * Copyright (c) 2015 Vecna Technologies, Inc.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.com/license/LICENSE.
 */


#ifndef _DEV_CAN_INTERNAL_H
#define _DEV_CAN_INTERNAL_H

#include <rtems.h>
#include <rtems/seterr.h>

#include <sys/ioctl.h>
#include <sys/stat.h>
#include <dev/can/can.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

typedef struct can_bus can_bus;

typedef struct can_rdwr_ioctl_data can_rdwr_ioctl_data;


/**
 * @brief Default can bus clock in Hz.
 */
#define CAN_BUS_CLOCK_DEFAULT 100000

/**
 * @brief can bus control.
 */
struct can_bus {

  int bus_number;

  int oflag;

  int (*init)(can_bus * bus, long buad);

  int (*de_init)(can_bus * bus);

  /*
   * @brief Destroys the bus.
   *
   * @param[in] bus The bus control.
   */
  void (*destroy)(can_bus *bus);

  /*
   * @brief install/set a can_filter
   * 
   * @param[in] can_bus bus instance
   * @param[in] can_filter filter. The filter to be installed or updated.
   *                There is no mechanism for disabling filters. Filters 
   *                should be disabled by setting the mask to all zeros
   *
   */
  int (*set_filter)       (can_bus * bus, can_filter * filter);


  /*
   * @brief Get the number of supported hardware filters
   *
   * @param[in] can_bus bus instance
   * @return number of supported filters
   */
  int (*get_num_filters)  (can_bus * bus);

  /**
   * Task for handling TX responsibilities
   */
  rtems_id tx_task_id;
    
  /**
   * Task for handling RX responsibilities
   */
  rtems_id rx_task_id;

  /**
   * TX task function entry point
   */
  rtems_task_entry rx_task;

  /**
   * RX task function entry point
   */
  rtems_task_entry tx_task;
  
  /**
   * Queues for passing messages
   * to and from CAN driver and
   * low level driver
   */
  rtems_id rx_msg_queue;
  rtems_id tx_msg_queue;

  can_filter default_filter;
  long default_baud;


  long baud;

  /**
   * @brief Mutex to protect the bus access.
   */
  rtems_id mutex;
};

/**
 * @brief Initializes a bus control.
 *
 * After a sucessful initialization the bus control must be destroyed via
 * can_bus_destroy().  A registered bus control will be automatically destroyed
 * in case the device file is unlinked.  Make sure to call can_bus_destroy() in
 * a custom destruction handler.
 *
 * @param[in] bus The bus control.
 *
 * @retval 0 Successful operation.
 * @retval -1 An error occurred.  The errno is set to indicate the error.
 *
 * @see can_bus_register()
 */
int can_bus_init(can_bus *bus);

/**
 * @brief Allocates a bus control from the heap and initializes it.
 *
 * After a sucessful allocation and initialization the bus control must be
 * destroyed via can_bus_destroy_and_free().  A registered bus control will be
 * automatically destroyed in case the device file is unlinked.  Make sure to
 * call can_bus_destroy_and_free() in a custom destruction handler.
 *
 * @param[in] size The size of the bus control.  This enables the addition of
 * bus controller specific data to the base bus control.  The bus control is
 * zero initialized.
 *
 * @retval non-NULL The new bus control.
 * @retval NULL An error occurred.  The errno is set to indicate the error.
 *
 * @see can_bus_register()
 */
can_bus *can_bus_alloc_and_init(size_t size);

/**
 * @brief Destroys a bus control.
 *
 * @param[in] bus The bus control.
 */
void can_bus_destroy(can_bus *bus);

/**
 * @brief Destroys a bus control and frees its memory.
 *
 * @param[in] bus The bus control.
 */
void can_bus_destroy_and_free(can_bus *bus);

/**
 * @brief Registers a bus control.
 *
 * This function claims ownership of the bus control regardless if the
 * registration is successful or not.
 *
 * @param[in] bus The bus control.
 * @param[in] bus_path The path to the bus device file.
 *
 * @retval 0 Successful operation.
 * @retval -1 An error occurred.  The errno is set to indicate the error.
 */
int can_bus_register(
  can_bus *bus
);

/**
 * @brief Obtains the bus.
 *
 * @param[in] bus The bus control.
 */
void can_bus_obtain(can_bus *bus);

/**
 * @brief Releases the bus.
 *
 * @param[in] bus The bus control.
 */
void can_bus_release(can_bus *bus);

/** @} */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _DEV_can_internal_H */
