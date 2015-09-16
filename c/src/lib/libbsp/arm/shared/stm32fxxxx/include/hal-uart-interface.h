/**
 * @file hal-uart-interface.h
 *
 * @ingroup uart
 *
 * @brief An interface layer to ST's hardware abstraction
 *   layer API functions used in UART driver implementation.
 *
 */

/*
 * Copyright (c) 2015 Vecna Technologies, Inc.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.com/license/LICENSE.
 */

#ifndef RTEMS_C_SRC_LIB_LIBBSP_ARM_STM32F4_UART_HAL_UART_INTERFACE_H_
#define RTEMS_C_SRC_LIB_LIBBSP_ARM_STM32F4_UART_HAL_UART_INTERFACE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <hal-utils.h>

#include stm_processor_header(TARGET_STM_PROCESSOR_PREFIX)
#include stm_header(TARGET_STM_PROCESSOR_PREFIX, dma)
#include stm_header(TARGET_STM_PROCESSOR_PREFIX, uart)

#include <rtems.h>
#include <termios.h>
#include <rtems/irq.h>
#include <rtems/libio.h>
#include <rtems/termiostypes.h>
#include <rtems/irq-extension.h>
#include <rtems/ringbuf.h>
#include <bspopts.h>
#include <stm32f-processor-specific.h>

/**
 * The maximum number of characters that can be queued for transmission
 * on a console (termios) UART.
 */
#define SERIAL_TERMIOS_FIFO_SIZE     256

/**
 * The maximum size of single UART TX
 **/
#define MAX_UART_TX_MSG_SIZE 1024

/**
 * The maximum size of data that UART can receive in one call
 **/
#define MAX_UART_RX_MSG_SIZE MAX_UART_TX_MSG_SIZE

/**
 * The maximum number of messages that can be queued for transmission
 * on each UART
 **/
#define UART_TX_QUEUE_LEN       4

/**
 * The RTEMS task priority of the UART TX task
 **/
#define UART_TX_TASK_PRIORITY   100

/**
 * The maximum amount of time the RTOS will wait for a single character to
 * sent or received (in milliseonds.)  This is only applicable when the UART
 * is configured to operate in polling mode.
 */
#define POLLED_TX_TIMEOUT_ms 1000

/**
 * The ioctl type for UARTs.
 */
#define IOCTL_UART_TYPE      73

/**
 * The ioctl baud rate argument definition.
 */
#define UART_BAUDRATE        _IOW(IOCTL_UART_TYPE, 1, uint32_t)

/**
 * The various modes of UART operation supported in the STM32F processor
 */
typedef enum {

  /**
   * The mode in which the UART register are periodically polling to see if
   * either data has been transmitted for received.
   */
  STM32F_UART_MODE_POLLING,

  /**
   * The mode in which the UART transmission and reception is controlled by
   * an interrupt service routine that supply and extract data from the UART
   * data register as necessary.
   */
  STM32F_UART_MODE_INT,

  /**
    * The mode in which the UART transmission and reception is controlled by
    * the UART's associated DMA controller.
    */
  STM32F_UART_MODE_DMA
} stm32f_uart_type;

/**
 * The set of possible UARTs implemented on an STM32F processor.  Not all
 * processor support all UARTs.  Make sure to enable all applicable UARTs via
 * the BSP config.ac file.
 */
typedef enum {

  /** UART 1 (/dev/ttyS0) */
  STM32F_UART1,

  /** UART 2 (/dev/ttyS1) */
  STM32F_UART2,

  /** UART 3 (/dev/ttyS2) */
  STM32F_UART3,

  /** UART 4 (/dev/ttyS3) */
  STM32F_UART4,

  /** UART 5 (/dev/ttyS4) */
  STM32F_UART5,

  /** UART 6 (/dev/ttyS5) */
  STM32F_UART6,

  /** UART 7 (/dev/ttyS6) */
  STM32F_UART7,

  /** UART 8 (/dev/ttyS7) */
  STM32F_UART8,

  /**
   * A placeholder value use when a function is supposed to return
   * a stm32f_uart value and the corresponding UART is not defined on
   * the processor. */
  STM32F_INVALID_UART
} stm32f_uart;

/**
 * The STM32F GPIO ports available on the various STM32F processors.
 * Note: not all processors implement all GPIO ports.
 */
typedef enum {

  /** Port A */
  STM32F_GOIO_PORTA,

  /** Port B */
  STM32F_GOIO_PORTB,

  /** Port C */
  STM32F_GOIO_PORTC,

  /** Port D */
  STM32F_GOIO_PORTD,

  /** Port E */
  STM32F_GOIO_PORTE,

  /** Port F */
  STM32F_GOIO_PORTF,

  /** Port G */
  STM32F_GOIO_PORTG,

  /** Port H */
  STM32F_GOIO_PORTH,

  /** Port I */
  STM32F_GOIO_PORTI
} stm32f_gpio_port;

/**
 * A structure used to identify a particular STM32F GPIO pin.
 */
typedef struct {

  /**
   * The STM32F GPIO port.
   */
  stm32f_gpio_port port;

  /**
   * The STM32F GPIO pin on a GPIO port.
   * This value should be within the range of [0-31].
   */
  uint32_t pin;

} stm32f_gpio_pin;

/**
 * The STM32F DMA controllers.
 */
typedef enum {

  /** DMA controller 1*/
  STM32F_DMA1_CONTROLLER,

  /** DMA controller 2*/
  STM32F_DMA2_CONTROLLER,
} stm32f_dma_controller;

/**
 * This structure contains all the relevant information about the configuration
 * of a particular DMA stream.
 */
typedef struct {

  /**
   * The DMA controller that handles the stream.
   */
  stm32f_dma_controller controller;

  /**
   * The processor interrupt associated with the DMA stream.
   */
  IRQn_Type DMAStreamInterruptNumber;

  /**
   * The DMA channel that controls the stream.
   */
  uint32_t channel;

  /**
   * The stream number within the DMA controller.
   */
  uint32_t stream;

} stm32f_dma_config;

/**
 * A structure that holds all the necessary information to configure
 * and control a UART using the STM32F HAL driver interface.  This
 * structure is defined for both termios and normal UARTs.
 */
typedef struct {

  /**
   * The STM32F HAL UART handle.
   */
  UART_HandleTypeDef* handle;

  /**
   * The path at which the UART should installed in the file system
   * (e.g., /dev/console,  /dev/ttyS0, ...)
   */
  const char* device_name;

  /**
   * The processor interrupt number associated with the UART.
   */
  IRQn_Type interrupt_number;

  /**
   * The DMA stream associated receiving data for this particular UART.
   */
  DMA_Stream_TypeDef* rx_dma_stream;

  /**
   * The DMA stream associated transmitting data for this particular UART.
   */
  DMA_Stream_TypeDef* tx_dma_stream;

  /**
   * The current baud rate of the UART.
   */
  uint32_t baud;

  /**
   * The UART mode of operation (i.e., polling, interrupt, dma).
   */
  stm32f_uart_type uart_mode;

  /**
   * The GPIO pin used to transmit serial data.
   */
  stm32f_gpio_pin tx_pin;

  /**
   * The GPIO pin used to receive serial data.
   */
  stm32f_gpio_pin rx_pin;

  /**
   * The DMA configuration for the UART TX stream.
   */
  stm32f_dma_config tx_dma;

  /**
   * The DMA configuration for the UART RX stream.
   */
  stm32f_dma_config rx_dma;

  /**
   * The alternate GPIO configuration used to initialize the GPIO
   * pins associated with the UART.
   */
  uint8_t alt_func_config;

  /**
   * Identification of which UART in the STM32F processor this instance
   * is associated with.
   */
  stm32f_uart uart;

} stm32f_base_uart_driver_entry;

/**
 * This structure contains all the relevant configuration and state
 * information associated with a console UART using termios.
 */
typedef struct {

  /**
   * The basic termios device context.
   * (This data member must be first, because the device context pointer
   *  is upcasted to a stm32f_console_driver_entry).
   */
  rtems_termios_device_context base;

  /**
   * The basic STM32F configuration data.
   */
  stm32f_base_uart_driver_entry base_driver_info;

  /**
   * The last received character.
   */
  uint8_t rx_char;

  /**
   * The termios tty associated with the UART.
   */
  struct rtems_termios_tty* tty;

  /**
   * A fifo buffer used to queue characters for transmission.
   */
  Ring_buffer_t* fifo;

  /**
   *  The block of data currently being transmitted on the UART.
   */
  uint8_t tx_buffer[SERIAL_TERMIOS_FIFO_SIZE];

} stm32f_console_driver_entry;

/**
 * The structure used hold all RTEMS objects created to service a non-console
 * UART.
 */
typedef struct {

  /**
   * The UART configuration data.
   */
  stm32f_base_uart_driver_entry base_driver_info;

  /**
   * The instance number of this particular non-console UART.  This
   * value should be a number from [0, NUM_PROCESSOR_NON_CONSOLE_UARTS-1].
   */
  uint32_t instance;

  /**
   * The task used to service transmit requests on the UART.
   */
  rtems_task_entry tx_task;

  /**
   * The rtems_id of the tasked used to service transmit requests.
   */
  rtems_id tx_task_id;

  /**
   * The rtems_id of the message queue used to hold transmit requests
   * for the UART.
   */
  rtems_id tx_msg_queue;

  /**
   * The rtems_id of the binary semaphore used to provide mutual exclusion to
   * driver's state variables.
   */
  rtems_id mutex;

} stm32_uart_driver_entry;

/**
 * This structure is used to register a non-console UART device driver.
 */
typedef struct {

  /**
   * The object that describes the configuration and state of the
   * non-console UART.
   */
  stm32_uart_driver_entry* pUart;

  /**
   * The initialization function for the device.
   */
  int (*init)(stm32_uart_driver_entry * pUart, uint32_t baud);

  /**
   * The de-initialization routine that closes the UART.
   */
  int (*de_init)(stm32_uart_driver_entry * pUart);

  /**
   * The destroy routine used to clean up the driver when it is no
   * longer required.
   */
  void (*destroy)(stm32_uart_driver_entry * pUart);
} stm32_uart_device;

/**
 * @brief Initializes console UARTs.
 *
 * This function initializes all console UARTs on the processor.
 *
 * @param[in] major the device driver's major number (not used)
 * @param[in] minor the device driver's minor number (not used)
 */
rtems_device_driver console_initialize(
  rtems_device_major_number major,
  rtems_device_minor_number minor,
  void* arg
);

/**
 * @brief Returns the console_uart from the HAL UART handle
 *
 * This will return a pointer to the console uart from the
 * stm32f_console_driver_table that has the specified HAL UART handle.
 *
 * @param[in] huart A STM32F HAL UART handle
 *
 * @return The address of the specified UART in the stm32f_console_driver_table
 *  if it exists.  If it doesn't exist, then NULL is returned.
 */
stm32f_console_driver_entry* stm32f_get_console_driver_entry_from_handle(
  const UART_HandleTypeDef *huart
);

/**
 * @brief Returns the location of the UART's hardware registers
 *
 * @param[in] Uart A specific STM32F UART
 *
 * @returns The starting address of the UART's hardware registers or
 *  NULL if the processor does not implement the specified UART.
 */
USART_TypeDef* stmf32_uart_get_registers(
  const stm32f_uart Uart
);

/**
 * @brief Installs interrupt handles for particular UART
 *
 * This function installs the generic UART ISR to UART interrupt
 * vector.  If the UART is configured for DMA operation then the DMA
 * TX and RX ISRs are also installed for the associated DMA streams.
 *
 * @param[in] pUart The UART whose interrupts should be enabled.
 *
 * @returns 0 if successful, non 0 otherwise.
 */
int uart_register_interrupt_handlers(
  stm32_uart_driver_entry* pUart
);

/**
 * @brief Removes interrupt handles for a particular UART
 *
 * This function removes the generic UART ISR to UART interrupt
 * vector.  If the UART is configured for DMA operation then the DMA
 * TX and RX ISRs are also removed for the associated DMA streams.
 *
 * @param[in] pUart The UART whose interrupts should be enabled.
 *
 * @returns 0 if successful, non 0 otherwise.
 */
int uart_remove_interrupt_handlers(
  stm32_uart_driver_entry* pUart
);

/**
 * @brief Initializes all non-console UARTs.
 */
void stm32f_uarts_initialize(
  void
);

/**
 * This array will hold all the STM32F HAL UART handles.  The handles are
 * allocated to the console UARTs first, and then to the non-console UARTs.
 */
extern UART_HandleTypeDef UartHandles[NUM_PROCESSOR_CONSOLE_UARTS + NUM_PROCESSOR_NON_CONSOLE_UARTS];

/**
 * The following array should be defined in the specific BSP directory
 * in a file named console-config.c.  This array should contain information
 * about which console UARTs should be defined for the processor.
 */
extern stm32f_console_driver_entry stm32f_console_driver_table[NUM_PROCESSOR_CONSOLE_UARTS];

/**
 * The following array should be defined in the specific BSP directory
 * in a file named uart-config.c.  This array should contain information
 * about which non-console UARTs should be defined for the processor.
 */
extern stm32_uart_driver_entry stm32f_uart_driver_table[NUM_PROCESSOR_NON_CONSOLE_UARTS];

#ifdef __cplusplus
}
#endif

#endif /* RTEMS_C_SRC_LIB_LIBBSP_ARM_STM32F4_UART_HAL_UART_INTERFACE_H_ */
