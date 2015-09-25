/**
 * @file
 *
 * @brief Serial Peripheral Interface (SPI) Driver API
 *
 * @ingroup spi
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



#ifndef _SPI_INTERNAL_H
#define _SPI_INTERNAL_H

#include <rtems.h>
#include <rtems/seterr.h>

#include <sys/ioctl.h>
#include <sys/stat.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/********************* Defines *************************/

/* ################### SPI Flags ##################### */
/**
 * @brief SPI Write Flag. Only performs write of defined data size
 */
#define SPI_M_WR    0x0001

/**
 * @brief SPI Read Flag. Only performs read of defined data size
 */
#define SPI_M_RD    0x0010

/**
 * @brief SPI Write and Read Flag. Performs write followed by read of defined size
 */
#define SPI_M_WR_RD    0x0100


/* ################### SPI IOCTL Commands ##################### */

/**
 * @brief IOCTL Command to change the SPI buad rate
 */
#define SPI_IOCTL_CHANGE_BAUD    ((uint32_t)0x0001)

/**
 * @brief IOCTL Command to change data size to 8bits
 */
#define SPI_IOCTL_DATASIZE_8     ((uint32_t)0x0002)

/**
 * @brief IOCTL Command to change data size to 16bits
 */
#define SPI_IOCTL_DATASIZE_16    ((uint32_t)0x0004)

/**
 * @brief IOCTL Command - SPI in 2 line half duplex mode
 */
#define SPI_IOCTL_DIR_2LINE        ((uint32_t)0x0008)

/**
 * @brief IOCTL Command - SPI in 1 line bidirection full duplex mode
 */
#define SPI_IOCTL_DIR_1LINE        ((uint32_t)0x0010)

/**
 * @brief IOCTL Command - change SPI instance to Master mode
 */
#define    SPI_IOCTL_MODE_MASTER    ((uint32_t)0x0020)

/**
 * @brief IOCTL Command - change SPI instance to Slave mode
 */
#define    SPI_IOCTL_MODE_SLAVE    ((uint32_t)0x0040)

/**
 * @brief IOCTL Command - enable CRC
 */
#define    SPI_IOCTL_ENABLE_CRC    ((uint32_t)0x0080)

/**
 * @brief IOCTL Command - disable CRC
 */
#define    SPI_IOCTL_DISABLE_CRC    ((uint32_t)0x0100)

/*********************** Typedefs *******************/
typedef uint16_t gpio_pin;
typedef struct spi_bus spi_bus;

/*********************** Enums *********************/

/**
 * @brief GPIO Port Enumerations
 */
typedef enum
{
  /* PORT A */
  PORTA = 0,
  /* PORT B */
  PORTB,
  /* PORT C */
  PORTC,
  /* PORT D */
  PORTD,
  /* PORT E */
  PORTE
} gpio_port;



/**
 * @brief GPIO Port Struct - Port name and pin
 */
typedef struct
{
  /* GPIO Ports */
  gpio_port port;
  /* GPIO Pin Struct */
  gpio_pin pin;
} gpio;

/**
 * @brief SPI transfer message.
 */
typedef struct {

  /**
   * @brief Pointer to the receive message data.
   */
  uint8_t *pRxBuf;

  /**
     * @brief Pointer to the receive message data.
     */
    uint8_t *pTxBuf;

  /**
   * @brief The message flags.
   *
   * Valid flags are
   * - @ref I2C_M_TEN,
   */
  uint16_t flags;

  /**
   * @brief The message data length in bytes.
   */
  uint16_t len;

  /**
   * Slave Select Pin
   */

  gpio slave_select;
} spi_msg ;


/**
 * @brief spi bus control.
 */
struct spi_bus {

  int (*init)(spi_bus * bus);

  int (*de_init)(spi_bus * bus);

  /**
   * @brief Transfers SPI messages.
   *
   * @param[in] bus The bus control.
   * @param[in] msgs The messages to transfer.
   * @param[in] msg_count The count of messages to transfer.  It must be
   * positive.
   *
   * @retval 0 Successful operation.
   * @retval negative Negative error number in case of an error.
   */
  int (*transfer)(spi_bus *bus, spi_msg *msgs, uint32_t msg_countt);

  /**
   * @brief IOCTL functionality for SPI
   */
  int (*ioctl)(spi_bus *bus, uint32_t ioctl_command, unsigned long arg);
  /*
   * @brief Destroys the bus.
   *
   * @param[in] bus The bus control.
   */
  void (*destroy)(spi_bus *bus);

  rtems_id mutex;
};



/**
 * @brief Initializes a bus control.
 *
 * After a sucessful initialization the bus control must be destroyed via
 * spi_bus_destroy().  A registered bus control will be automatically destroyed
 * in case the device file is unlinked.  Make sure to call spi_bus_destroy() in
 * a custom destruction handler.
 *
 * @param[in] bus The bus control.
 *
 * @retval 0 Successful operation.
 * @retval -1 An error occurred.  The errno is set to indicate the error.
 *
 * @see spi_bus_register()
 */
int spi_bus_init(spi_bus *bus);

/**
 * @brief Allocates a bus control from the heap and initializes it.
 *
 * After a sucessful allocation and initialization the bus control must be
 * destroyed via spi_bus_destroy_and_free().  A registered bus control will be
 * automatically destroyed in case the device file is unlinked.  Make sure to
 * call spi_bus_destroy_and_free() in a custom destruction handler.
 *
 * @param[in] size The size of the bus control.  This enables the addition of
 * bus controller specific data to the base bus control.  The bus control is
 * zero initialized.
 *
 * @retval non-NULL The new bus control.
 * @retval NULL An error occurred.  The errno is set to indicate the error.
 *
 * @see spi_bus_register()
 */
spi_bus *spi_bus_alloc_and_init(size_t size);

/**
 * @brief Destroys a bus control.
 *
 * @param[in] bus The bus control.
 */
void spi_bus_destroy(spi_bus *bus);

/**
 * @brief Destroys a bus control and frees its memory.
 *
 * @param[in] bus The bus control.
 */
void spi_bus_destroy_and_free(spi_bus *bus);

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
int spi_bus_register(
          spi_bus *bus,
          const char *bus_path
);

/**
 * @brief Obtains the bus.
 *
 * @param[in] bus The bus control.
 */
void spi_bus_obtain(spi_bus *bus);

/**
 * @brief Releases the bus.
 *
 * @param[in] bus The bus control.
 */
void spi_bus_release(spi_bus *bus);

/** @} */


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _SPI_INTERNAL_H */
