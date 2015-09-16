/**
 * @file
 *
 * @brief Serial Peripheral Interface (SPI) Bus Implementation
 *
 * @ingroup SPIBus
 */

/*
 * Copyright (c) 2015 Vecna Technologies, Inc.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.com/license/LICENSE.
 */

#if HAVE_CONFIG_H
  #include "config.h"
#endif

#include <dev/spi/spi.h>

#include <rtems/imfs.h>

#include <stdlib.h>
#include <string.h>

void spi_bus_obtain(spi_bus *bus)
{
  rtems_status_code sc;

  sc = rtems_semaphore_obtain(bus->mutex, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
  _Assert(sc == RTEMS_SUCCESSFUL);
  (void) sc;
}

void spi_bus_release(spi_bus *bus)
{
  rtems_status_code sc;

  sc = rtems_semaphore_release(bus->mutex);
  _Assert(sc == RTEMS_SUCCESSFUL);
  (void) sc;
}

int spi_bus_transfer(spi_bus *bus, spi_msg *msgs, uint32_t msg_count)
{
  int err;
  uint32_t i;
  uint32_t j;

  _Assert(msg_count > 0);

  spi_bus_obtain(bus);
  err = (*bus->transfer)(bus, msgs, msg_count);
  spi_bus_release(bus);

  return err;
}

static int spi_bus_open(
		rtems_libio_t *iop,
		const char    *path,
		int            oflag,
		mode_t         mode
)
{
	int err;
	spi_bus *bus = IMFS_generic_get_context_by_iop(iop);
	spi_bus_obtain(bus);
	err = bus->init(bus);
	spi_bus_release(bus);

	if (err == 0) {
		return 0;
	} else {
		rtems_set_errno_and_return_minus_one(-err);
	}

}

static int spi_bus_close(
  rtems_libio_t *iop
)
{
  spi_bus *bus = IMFS_generic_get_context_by_iop(iop);
  spi_bus_obtain(bus);
  int err = bus->de_init(bus);
  spi_bus_release(bus);

  if (err == 0) {
    return 0;
  } else {
    rtems_set_errno_and_return_minus_one(-err);
  }
}

static ssize_t spi_bus_read(
  rtems_libio_t *iop,
  void *buffer,
  size_t count
)
{
  spi_bus *bus = IMFS_generic_get_context_by_iop(iop);
  spi_msg* msg = buffer;
  int err;

  err = spi_bus_transfer(bus, msg, msg->len);
  if (err == 0) {
    return msg->len;
  } else {
    rtems_set_errno_and_return_minus_one(-err);
  }
}

static ssize_t spi_bus_write(
  rtems_libio_t *iop,
  const void *buffer,
  size_t count
)
{
  spi_bus *bus = IMFS_generic_get_context_by_iop(iop);
  spi_msg* msg = buffer;
    int err;

    err = spi_bus_transfer(bus, msg, msg->len);
  if (err == 0) {
    return msg->len;
  } else {
    rtems_set_errno_and_return_minus_one(-err);
  }
}

static int spi_bus_ioctl(
  rtems_libio_t *iop,
  ioctl_command_t command,
  void *arg
)
{
  spi_bus *bus = IMFS_generic_get_context_by_iop(iop);
  int err;

  spi_bus_obtain(bus);
  err = (*bus->ioctl)(bus, (uint32_t) command, (unsigned long) arg);
  spi_bus_release(bus);

  if (err == 0) {
    return 0;
  } else {
    rtems_set_errno_and_return_minus_one(-err);
  }
}

static const rtems_filesystem_file_handlers_r spi_bus_handler = {
  .open_h = spi_bus_open,
  .close_h = spi_bus_close,
  .read_h = spi_bus_read,
  .write_h = spi_bus_write,
  .ioctl_h = spi_bus_ioctl,
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

static void spi_bus_node_destroy(IMFS_jnode_t *node)
{
  spi_bus *bus;

  bus = IMFS_generic_get_context_by_node(node);
  (*bus->destroy)(bus);

  IMFS_node_destroy_default(node);
}

static const IMFS_node_control spi_bus_node_control = IMFS_GENERIC_INITIALIZER(
  &spi_bus_handler,
  IMFS_node_initialize_generic,
  spi_bus_node_destroy
);

int spi_bus_register(
		  spi_bus *bus,
		  const char *bus_path
)
{
  int rv;

  rv = IMFS_make_generic_node(
    bus_path,
    S_IFCHR | S_IRWXU | S_IRWXG | S_IRWXO,
    &spi_bus_node_control,
    bus
  );
  if (rv != 0) {
    (*bus->destroy)(bus);
  }

  return rv;
}

static int spi_bus_transfer_default(
  spi_bus *bus,
  spi_msg *msgs,
  uint32_t msg_count
)
{
  (void) bus;
  (void) msgs;
  (void) msg_count;

  return -EIO;
}

static int spi_bus_do_init(
  spi_bus *bus,
  void (*destroy)(spi_bus *bus)
)
{
  rtems_status_code sc;

  sc = rtems_semaphore_create(
    rtems_build_name('S', 'P', 'I', ' '),
    1,
    RTEMS_BINARY_SEMAPHORE | RTEMS_INHERIT_PRIORITY | RTEMS_PRIORITY,
    0,
    &bus->mutex
  );
  if (sc != RTEMS_SUCCESSFUL) {
    (*destroy)(bus);

    rtems_set_errno_and_return_minus_one(ENOMEM);
  }

  bus->transfer = spi_bus_transfer_default;
  bus->destroy = destroy;

  return 0;
}

void spi_bus_destroy(spi_bus *bus)
{
  rtems_status_code sc;

  sc = rtems_semaphore_delete(bus->mutex);
  _Assert(sc == RTEMS_SUCCESSFUL);
  (void) sc;
}

void spi_bus_destroy_and_free(spi_bus *bus)
{
  spi_bus_destroy(bus);
  free(bus);
}

int spi_bus_init(spi_bus *bus)
{
  memset(bus, 0, sizeof(*bus));

  return spi_bus_do_init(bus, spi_bus_destroy);
}

spi_bus *spi_bus_alloc_and_init(size_t size)
{
  spi_bus *bus = NULL;

  if (size >= sizeof(*bus)) {
    bus = calloc(1, size);
    if (bus != NULL) {
      int rv;

      rv = spi_bus_do_init(bus, spi_bus_destroy_and_free);
      if (rv != 0) {
        return NULL;
      }
    }
  }

  return bus;
}
