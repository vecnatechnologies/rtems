/**
 * @file
 *
 * @brief Control Area Network (can) Bus Implementation
 *
 * @ingroup canBus
 */

/*
 * Copyright (c) 2014 embedded brains GmbH.  All rights reserved.
 *
 *  embedded brains GmbH
 *  Dornierstr. 4
 *  82178 Puchheim
 *  Germany
 *  <rtems@embedded-brains.de>
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#if HAVE_CONFIG_H
  #include "config.h"
#endif

#include <dev/can/can-internal.h>


#include <rtems/imfs.h>

#include <stdlib.h>
#include <string.h>

#define CAN_QUEUE_LEN 10
#define CAN_TASK_PRIORITY 70

void can_bus_obtain(can_bus *bus)
{
  rtems_status_code sc;

  sc = rtems_semaphore_obtain(bus->mutex, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
  _Assert(sc == RTEMS_SUCCESSFUL);
  (void) sc;
}

void can_bus_release(can_bus *bus)
{
  rtems_status_code sc;

  sc = rtems_semaphore_release(bus->mutex);
  _Assert(sc == RTEMS_SUCCESSFUL);
  (void) sc;
}

int can_bus_change_baudrate(
  can_bus * bus, 
  uint32_t baud
) 
{
  can_bus_obtain(bus);
  bus->de_init(bus);
  bus->init(bus, baud);
  can_bus_release(bus);
  return 0;
}

int can_bus_set_filter(
  can_bus * bus, 
  can_filter * filter
) 
{
  int err;
  _Assert(filter);
  can_bus_obtain(bus);
  err =  bus->set_filter(bus, filter);
  can_bus_release(bus);
  return err;
}


int can_bus_init_default(
  can_bus * bus,
  long baud
)
{
  (void) bus;
  (void) baud;
  return 0;
  return -EIO;
}

int can_bus_de_init_default(
  can_bus * bus
)
{
  (void) bus;
  return -EIO;
}

int can_bus_set_filter_default(  
  can_bus * bus,
  can_filter * filter
)
{
  (void) bus;
  (void) filter;
  return -EIO;
}

int can_bus_get_num_filters_default(  
  can_bus * bus
)
{
  (void) bus;
  return -EIO;
}

int can_bus_transfer(can_bus *bus, can_msg *msgs, uint32_t msg_count)
{
  int err;

  _Assert(msg_count > 0);
  
  // error checking

  can_bus_obtain(bus);
  can_bus_release(bus);

  return err;
}

static ssize_t can_bus_read(
  rtems_libio_t *iop,
  void *buffer,
  size_t count
)
{
  can_bus *bus = IMFS_generic_get_context_by_iop(iop);
  int err;
  rtems_status_code sc;
  can_msg msg;
  size_t len;


  if (count != sizeof(msg)) {
    return 0; 
  }

  sc = rtems_message_queue_receive(bus->rx_msg_queue, 
      &msg,
      &len,
      RTEMS_WAIT,
      RTEMS_NO_TIMEOUT);
  
  if (RTEMS_SUCCESSFUL != sc) {
    return 0;
  }

  _Assert(len == sizeof(msg));
  memcpy(buffer, &msg, sizeof(msg));
  return sizeof(msg);

  if (err == 0) {
    return 0;
  } else {
    rtems_set_errno_and_return_minus_one(-err);
  }
}

static ssize_t can_bus_write(
  rtems_libio_t *iop,
  const void *buffer,
  size_t count
)
{
  can_bus *bus = IMFS_generic_get_context_by_iop(iop);
  int err;
  rtems_status_code sc;

  can_msg  * msg = (can_msg * ) buffer;

  _Assert(count % sizeof(can_msg) == 0);
  _Assert(msg->len <= 8);

  size_t msg_count = count / sizeof(can_msg);
  int i;
  for (i = 0; i < msg_count; i++) {
      sc = rtems_message_queue_send(bus->tx_msg_queue,
                                    buffer + sizeof(can_msg) * i,
                                    sizeof(can_msg));
  }

  if (RTEMS_SUCCESSFUL != sc) {
    err = EIO; 
  }

  if (err == 0) {
    return 0;
  } else {
    rtems_set_errno_and_return_minus_one(-err);
  }
}

static int can_bus_close(
  rtems_libio_t *iop
)
{
  can_bus *bus = IMFS_generic_get_context_by_iop(iop);
  int err;
  can_bus_obtain(bus);
  err = bus->de_init(bus);
  can_bus_release(bus);
  if (err == 0) {
    return 0;
  } else {
    rtems_set_errno_and_return_minus_one(-err);
  }
}

static int can_bus_open(
  rtems_libio_t *iop,
  const char    *path,
  int            oflag,
  mode_t         mode
)
{
  rtems_status_code sc;
  can_bus *bus = IMFS_generic_get_context_by_iop(iop);
  int err;
  can_bus_obtain(bus);
  err = bus->init(bus, bus->default_baud);
  can_bus_release(bus);

  sc = rtems_task_start(
      bus->rx_task_id,
      bus->rx_task,
      (rtems_task_argument) bus
      );
  if (err == 0) {
    return 0;
  } else {
    rtems_set_errno_and_return_minus_one(-err);
  }
}

static int can_bus_ioctl(
  rtems_libio_t *iop,
  ioctl_command_t command,
  void *arg
)
{
  can_bus *bus = IMFS_generic_get_context_by_iop(iop);
  can_filter * filter;
  int err;

  switch (command) {
    case CAN_SET_FILTER:
      filter = (can_filter*) arg;
      bus->set_filter(bus, filter);
      break;

    case CAN_GET_NUM_FILTERS:
      return bus->get_num_filters(bus);
      break;

    default:
      err = -ENOTTY;
      break;
  }

  if (err == 0) {
    return 0;
  } else {
    rtems_set_errno_and_return_minus_one(-err);
  }
}

static const rtems_filesystem_file_handlers_r can_bus_handler = {
  .open_h = can_bus_open,
  .close_h = can_bus_close,
  .read_h = can_bus_read,
  .write_h = can_bus_write,
  .ioctl_h = can_bus_ioctl,
  .lseek_h = rtems_filesystem_default_lseek,
  .fstat_h = IMFS_stat,
  .ftruncate_h = rtems_filesystem_default_ftruncate,
  .fsync_h = rtems_filesystem_default_fsync_or_fdatasync,
  .fdatasync_h = rtems_filesystem_default_fsync_or_fdatasync,
  .fcntl_h = rtems_filesystem_default_fcntl,
  .kqfilter_h = rtems_filesystem_default_kqfilter,
  .poll_h = rtems_filesystem_default_poll,
  .readv_h = rtems_filesystem_default_readv,
  .writev_h = rtems_filesystem_default_writev
};

static void can_bus_node_destroy(IMFS_jnode_t *node)
{
  can_bus *bus;

  bus = IMFS_generic_get_context_by_node(node);
  (*bus->destroy)(bus);

  IMFS_node_destroy_default(node);
}

static const IMFS_node_control can_bus_node_control = IMFS_GENERIC_INITIALIZER(
  &can_bus_handler,
  IMFS_node_initialize_generic,
  can_bus_node_destroy
);

int can_bus_register(
  can_bus *bus,
  const char *bus_path
)
{
  int rv;
  rtems_status_code sc;

  rv = IMFS_make_generic_node(
    bus_path,
    S_IFCHR | S_IRWXU | S_IRWXG | S_IRWXO,
    &can_bus_node_control,
    bus
  );
  if (rv != 0) {
    (*bus->destroy)(bus);
  }

  sc = rtems_task_start(
      bus->tx_task_id,
      bus->tx_task,
      (rtems_task_argument) bus
      );
  return rv;
}

static int can_bus_do_init(
  can_bus *bus,
  void (*destroy)(can_bus *bus)
)
{
  rtems_status_code sc;

  sc = rtems_semaphore_create(
    rtems_build_name('C', 'A', 'N', ' '),
    1,
    RTEMS_BINARY_SEMAPHORE | RTEMS_INHERIT_PRIORITY | RTEMS_PRIORITY,
    0,
    &bus->mutex
  );

  if (sc != RTEMS_SUCCESSFUL) {
    (*destroy)(bus);
    rtems_set_errno_and_return_minus_one(ENOMEM);
  }

  sc = rtems_message_queue_create(
    rtems_build_name('C', 'A', 'N', 'R'),
    CAN_QUEUE_LEN,
    sizeof(can_msg),
    RTEMS_BINARY_SEMAPHORE | RTEMS_INHERIT_PRIORITY | RTEMS_PRIORITY,
    &bus->rx_msg_queue
  );

  if (sc != RTEMS_SUCCESSFUL) {
    (*destroy)(bus);
    rtems_set_errno_and_return_minus_one(ENOMEM);
  }

  sc = rtems_message_queue_create(
    rtems_build_name('C', 'A', 'N', 'T'),
    CAN_QUEUE_LEN,
    sizeof(can_msg),
    RTEMS_BINARY_SEMAPHORE | RTEMS_INHERIT_PRIORITY | RTEMS_PRIORITY,
    &bus->tx_msg_queue
  );

  if (sc != RTEMS_SUCCESSFUL) {
    (*destroy)(bus);
    rtems_set_errno_and_return_minus_one(ENOMEM);
  }

  sc = rtems_task_create(
    rtems_build_name('C', 'A', 'N', 'r'),
    CAN_TASK_PRIORITY - 1,
    RTEMS_MINIMUM_STACK_SIZE,
    RTEMS_PREEMPT,
    RTEMS_NO_FLOATING_POINT,
    &bus->rx_task_id
  );

  if (sc != RTEMS_SUCCESSFUL) {
    (*destroy)(bus);
    rtems_set_errno_and_return_minus_one(ENOMEM);
  }

  sc = rtems_task_create(
    rtems_build_name('C', 'A', 'N', 't'),
    CAN_TASK_PRIORITY,
    RTEMS_MINIMUM_STACK_SIZE,
    RTEMS_PREEMPT,
    RTEMS_NO_FLOATING_POINT,
    &bus->tx_task_id
  );

  if (sc != RTEMS_SUCCESSFUL) {
    (*destroy)(bus);
    rtems_set_errno_and_return_minus_one(ENOMEM);
  }


  bus->destroy = destroy;
  bus->set_filter = can_bus_set_filter_default;
  bus->init = can_bus_init_default;
  bus->de_init = can_bus_de_init_default;
  bus->set_filter = can_bus_set_filter_default;
  bus->get_num_filters = can_bus_get_num_filters_default;
  return 0;
}

void can_bus_destroy(can_bus *bus)
{
  rtems_status_code sc;

  bus->de_init(bus);

  sc = rtems_semaphore_delete(bus->mutex);
  _Assert(sc == RTEMS_SUCCESSFUL);

  sc = rtems_message_queue_delete(bus->rx_msg_queue);
  _Assert(sc == RTEMS_SUCCESSFUL);

  sc = rtems_message_queue_delete(bus->tx_msg_queue);
  _Assert(sc == RTEMS_SUCCESSFUL);

  sc = rtems_task_delete(bus->rx_task_id);
  _Assert(sc == RTEMS_SUCCESSFUL);

  sc = rtems_task_delete(bus->tx_task_id);
  _Assert(sc == RTEMS_SUCCESSFUL);

  (void) sc;
}

void can_bus_destroy_and_free(can_bus *bus)
{
  can_bus_destroy(bus);
  free(bus);
}

int can_bus_init(can_bus *bus)
{
  memset(bus, 0, sizeof(*bus));

  int err = can_bus_do_init(bus, can_bus_destroy);

  return err;
}

can_bus *can_bus_alloc_and_init(size_t size)
{
  can_bus *bus = NULL;

  if (size >= sizeof(*bus)) {
    bus = calloc(1, size);
    if (bus != NULL) {
      int rv;

      bus->default_baud = 1000000;
      rv = can_bus_do_init(bus, can_bus_destroy_and_free);
      if (rv != 0) {
        return NULL;
      }
    }
  }

  return bus;
}

inline uint32_t can_filter_stdid(uint32_t id) {
  return id << 21;
}
