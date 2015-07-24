/**
 * @file
 *
 * @brief Control Area Network  (can) Driver API
 *
 * @ingroup can
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

#ifndef _DEV_CAN_H
#define _DEV_CAN_H

#include <rtems.h>
#include <rtems/seterr.h>

#include <sys/ioctl.h>
#include <sys/stat.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define MAX_CAN_MSG_LEN 8

typedef struct {
  uint32_t id;
  uint8_t len;
  uint8_t data[MAX_CAN_MSG_LEN];
} can_msg;

struct can_filter{
  unsigned long mask;
};

typedef struct can_filter can_filter;

typedef struct can_bus can_bus;

typedef struct can_dev can_dev;

typedef struct can_rdwr_ioctl_data can_rdwr_ioctl_data;



/**
 * @brief Default can bus clock in Hz.
 */
#define CAN_BUS_CLOCK_DEFAULT 100000

/**
 * @brief can bus control.
 */
struct can_bus {


  int (*init)(can_bus * bus, long buad);

  int (*de_init)(can_bus * bus);

  /*
   * @brief Destroys the bus.
   *
   * @param[in] bus The bus control.
   */
  void (*destroy)(can_bus *bus);

  int (*set_filter)(can_bus * bus, can_filter * filter);

  rtems_id tx_task_id;

  rtems_id rx_task_id;

  rtems_task_entry rx_task;
  rtems_task_entry tx_task;

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
  can_bus *bus,
  const char *bus_path
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

/**
 * @brief Transfers can messages.
 *
 * The bus is obtained before the transfer and released afterwards.
 *
 * @param[in] bus The bus control.
 * @param[in] msgs The messages to transfer.
 * @param[in] msg_count The count of messages to transfer.  It must be
 * positive.
 *
 * @retval 0 Successful operation.
 * @retval negative Negative error number in case of an error.
 */
int can_bus_transfer(can_bus *bus, can_msg *msgs, uint32_t msg_count);

/** @} */

/**
 * IOCTL Commands
 */
#define IOCTL_CAN_TYPE 72

#define CAN_BAUDRATE  _IOW(IOCTL_CAN_TYPE, 1, unsigned long)
#define CAN_FILTER    _IOW(IOCTL_CAN_TYPE, 2, can_filter)

/**
 * @brief can slave device control.
 */
struct can_dev {
  /**
   * @brief Reads from the device.
   *
   * @retval non-negative Bytes transferred from device.
   * @retval negative Negative error number in case of an error.
   */
  ssize_t (*read)(can_dev *dev, void *buf, size_t n, off_t offset);

  /**
   * @brief Writes to the device.
   *
   * @retval non-negative Bytes transferred to device.
   * @retval negative Negative error number in case of an error.
   */
  ssize_t (*write)(can_dev *dev, const void *buf, size_t n, off_t offset);

  /**
   * @brief Device IO control.
   *
   * @retval 0 Successful operation.
   * @retval negative Negative error number in case of an error.
   */
  int (*iocts)(can_dev *dev, ioctl_command_t command, void *arg);

  /**
   * emud.org/jelson/software/fusd/docs/nbrief Gets the file size.
   */
  off_t (*get_size)(can_dev *dev);

  /**
   * @brief Gets the file block size.
   */
  blksize_t (*get_block_size)(can_dev *dev);

  /**
   * @brief Destroys the device.
   */
  void (*destroy)(can_dev *dev);

  /**
   * @brief The bus control.
   */
  can_bus *bus;

  /**
   * @brief The device address.
   */
  uint16_t address;

  /**
   * @brief File descriptor of the bus.
   *
   * This prevents destruction of the bus since we hold a reference to it with
   * this.
   */
  int bus_fd;
};


/**
 * @brief Initializes a device control.
 *
 * After a sucessful initialization the device control must be destroyed via
 * can_dev_destroy().  A registered device control will be automatically
 * destroyed in case the device file is unlinked.  Make sure to call
 * can_dev_destroy_and_free() in a custom destruction handler.
 *
 * @param[in] device The device control.
 * @param[in] bus_path The path to the bus device file.
 * @param[in] address The address of the device.
 *
 * @retval 0 Successful operation.
 * @retval -1 An error occurred.  The errno is set to indicate the error.
 *
 * @see can_dev_register()
 */
int can_dev_init(can_dev *dev, const char *bus_path, uint16_t address);

/**
 * @brief Allocates a device control from the heap and initializes it.
 *
 * After a sucessful allocation and initialization the device control must be
 * destroyed via can_dev_destroy_and_free().  A registered device control will
 * be automatically destroyed in case the device file is unlinked.  Make sure
 * to call can_dev_destroy_and_free() in a custom destruction handler.
 *
 * @param[in] size The size of the device control.  This enables the addition
 * of device specific data to the base device control.  The device control is
 * zero initialized.
 * @param[in] bus_path The path to the bus device file.
 * @param[in] address The address of the device.
 *
 * @retval non-NULL The new device control.
 * @retval NULL An error occurred.  The errno is set to indicate the error.
 *
 * @see can_dev_register()
 */
can_dev *can_dev_alloc_and_init(
  size_t size,
  const char *bus_path,
  uint16_t address
);

/**
 * @brief Destroys a device control.
 *
 * @param[in] dev The device control.
 */
void can_dev_destroy(can_dev *dev);

/**
 * @brief Destroys a device control and frees its memory.
 *
 * @param[in] dev The device control.
 */
void can_dev_destroy_and_free(can_dev *dev);

/**
 * @brief Registers a device control.
 *
 * This function claims ownership of the device control regardless if the
 * registration is successful or not.
 *
 * @param[in] dev The dev control.
 * @param[in] dev_path The path to the device file of the device.
 *
 * @retval 0 Successful operation.
 * @retval -1 An error occurred.  The errno is set to indicate the error.
 */
int can_dev_register(
  can_dev *dev,
  const char *dev_path
);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _DEV_can_can_H */
