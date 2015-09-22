/**
 * @file uart.c
 * @author Jay M. Doyle
 *
 * @ingroup uart
 *
 * @brief A universal UART driver implementation for all STM32FXXXX Cortex-M
 *  processors using ST's hardware abstraction layer.
 *
 */

/*
 * Copyright (c) 2015 Vecna Technologies, Inc.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.com/license/LICENSE.
 */
#include <rtems.h>

#include <hal-utils.h>
#include <hal-uart-interface.h>
#include <stm32f-processor-specific.h>
#include <hal-error.h>

#include stm_processor_header( TARGET_STM_PROCESSOR_PREFIX )
#include stm_header( TARGET_STM_PROCESSOR_PREFIX, uart )
#include stm_header( TARGET_STM_PROCESSOR_PREFIX, gpio_ex )

#include <rtems/imfs.h>

//--- Include processor specfic uart configurations
#include <uart-config.c>

typedef struct {
  uint32_t len;
  rtems_interval max_wait_time;
} uart_rx_request;

static uint8_t tx_msg[ NUM_PROCESSOR_NON_CONSOLE_UARTS ][ MAX_UART_TX_MSG_SIZE
];

//============================== ISR Definitions ==========================================
static HAL_StatusTypeDef stm32f_uart_transmit(
  UART_HandleTypeDef    *hUartHandle,
  const stm32f_uart_type uartType,
  uint8_t               *buf,
  uint16_t               len
)
{
  HAL_StatusTypeDef ret = HAL_ERROR;

  if ( ( len > 0 ) && ( buf != NULL ) && ( hUartHandle != NULL ) ) {
    switch ( uartType ) {
      case STM32F_UART_MODE_POLLING:
        ret = HAL_UART_Transmit( hUartHandle, buf, len, POLLED_TX_TIMEOUT_ms );
        break;

      case STM32F_UART_MODE_INT:
        ret = HAL_UART_Transmit_IT( hUartHandle, buf, len );
        break;

      case STM32F_UART_MODE_DMA:
        ret = HAL_UART_Transmit_DMA( hUartHandle, buf, len );
        break;
    }
  }

  return ret;
}

static HAL_StatusTypeDef stm32f_uart_receive(
  UART_HandleTypeDef    *hUartHandle,
  const stm32f_uart_type uartType,
  uint8_t               *buf,
  size_t                 len,
  rtems_interval         timeout
)
{
  HAL_StatusTypeDef         ret = HAL_ERROR;
  stm32f_uart_driver_entry *pUartDriver =
    stm32f_get_uart_driver_entry_from_handle( hUartHandle );

  if ( ( len > 0 ) && ( buf != NULL ) && ( hUartHandle != NULL ) &&
       ( pUartDriver != NULL ) ) {
    // Record calling task rtems_id for RX event routing.
    pUartDriver->rx_task_id = rtems_task_self();

    switch ( uartType ) {
      case STM32F_UART_MODE_POLLING:
        ret = HAL_UART_Receive( hUartHandle, buf, (uint16_t) len, timeout );
        break;

      case STM32F_UART_MODE_INT:
        ret = HAL_UART_Receive_IT( hUartHandle, buf, (uint16_t) len );
        break;

      case STM32F_UART_MODE_DMA:
        ret = HAL_UART_Receive_DMA( hUartHandle, buf, (uint16_t) len );
        break;
    }
  }

  return ret;
}

static rtems_task stm32f_uart_tx_task( rtems_task_argument arg )
{
  size_t                    len;
  rtems_status_code         sc;
  rtems_event_set           events_rx;
  stm32f_uart_driver_entry *pUart = (stm32f_uart_driver_entry *) arg;
  rtems_interval            ticks_per_ms;

  // Calculate the ticks per ms.  This is constant during execution
  // of the application and therefore can be calculated just once
  // at task startup.
  ticks_per_ms = rtems_clock_get_ticks_per_second() / 1000;

  while ( 1 ) {
    // wait until there is a TX request
    sc = rtems_message_queue_receive( pUart->tx_msg_queue,
      (void *) &tx_msg[ pUart->instance ],
      &len,
      RTEMS_WAIT,
      RTEMS_NO_TIMEOUT );
    _Assert( len <= sizeof( tx_msg ) );

    // send tx data to UART hardware
    if ( sc == RTEMS_SUCCESSFUL ) {
      (void) stm32f_uart_transmit( pUart->base_driver_info.handle,
        pUart->base_driver_info.uart_mode,
        (uint8_t *) tx_msg[ pUart->instance ],
        len );
    }

    // Wait for event from TX complete callback before
    // processing any additional messages.
    sc = rtems_event_receive( UART_TX_DONE,
      RTEMS_WAIT | RTEMS_EVENT_ANY,
      ticks_per_ms * STM32F_MAX_UART_TX_TIME_ms,
      &events_rx );
  }
}

static void uart_obtain( stm32f_uart_driver_entry *pUart )
{
  rtems_status_code sc;

  sc = rtems_semaphore_obtain( pUart->mutex, RTEMS_WAIT, RTEMS_NO_TIMEOUT );
  _Assert( sc == RTEMS_SUCCESSFUL );
  (void) sc;
}

static void uart_release( stm32f_uart_driver_entry *pUart )
{
  rtems_status_code sc;

  sc = rtems_semaphore_release( pUart->mutex );
  _Assert( sc == RTEMS_SUCCESSFUL );
  (void) sc;
}

static ssize_t stm32f_uart_read(
  rtems_libio_t *iop,
  void          *buffer,
  size_t         count
)
{
  stm32f_uart_device *pUartDevice =
    (stm32f_uart_device *) IMFS_generic_get_context_by_iop( iop );
  ssize_t           ret_size = 0;
  HAL_StatusTypeDef ret;
  rtems_status_code sc;
  rtems_event_set   events_rx;

  if ( count <= MAX_UART_RX_MSG_SIZE ) {
    // collect the requested number of characters from the UART
    ret = stm32f_uart_receive( pUartDevice->pUart->base_driver_info.handle,
      pUartDevice->pUart->base_driver_info.uart_mode,
      (uint8_t *) buffer,
      count,
      POLLED_TX_TIMEOUT_ms );

    if ( ret == HAL_OK ) {
      // Wait for event from RX complete callback before
      // processing any additional messages.
      sc = rtems_event_receive( UART_RX_DONE,
        RTEMS_WAIT | RTEMS_EVENT_ANY,
        RTEMS_NO_TIMEOUT,
        &events_rx );

      if ( sc == RTEMS_SUCCESSFUL ) {
        ret_size = count;
      } else {
        stm32f_error_handler();
        ret_size = 0;
      }
    }
  }

  return ret_size;
}

static ssize_t stm32f_uart_write(
  rtems_libio_t *iop,
  const void    *buffer,
  size_t         count
)
{
  stm32f_uart_device *pUartDevice =
    (stm32f_uart_device *) IMFS_generic_get_context_by_iop( iop );
  rtems_status_code sc;

  if ( count <= MAX_UART_TX_MSG_SIZE ) {
    sc = rtems_message_queue_send(
      pUartDevice->pUart->tx_msg_queue,
      buffer,
      count
         );

    if ( RTEMS_SUCCESSFUL != sc ) {
      sc = (rtems_status_code) EIO;
    }
  } else {
    sc = (rtems_status_code) EIO;
  }

  return sc;
}

static int stm32f_uart_close( rtems_libio_t *iop )
{
  stm32f_uart_device *pUartDevice =
    (stm32f_uart_device *) IMFS_generic_get_context_by_iop( iop );
  int err;

  uart_obtain( pUartDevice->pUart );
  err = pUartDevice->de_init( pUartDevice->pUart );
  uart_release( pUartDevice->pUart );

  if ( err == 0 ) {
    return 0;
  } else {
    rtems_set_errno_and_return_minus_one( -err );
  }
}

static int stm32f_uart_open(
  rtems_libio_t *iop,
  const char    *path,
  int            oflag,
  mode_t         mode
)
{
  (void) path;
  (void) oflag;
  (void) mode;

  stm32f_uart_device *pUartDevice =
    (stm32f_uart_device *) IMFS_generic_get_context_by_iop( iop );
  rtems_status_code err;

  uart_obtain( pUartDevice->pUart );
  err = (rtems_status_code) pUartDevice->init(
    pUartDevice->pUart,
    pUartDevice->pUart->base_driver_info.baud
        );
  uart_release( pUartDevice->pUart );

  if ( err == RTEMS_SUCCESSFUL ) {
    return RTEMS_SUCCESSFUL;
  } else {
    rtems_set_errno_and_return_minus_one( -err );
  }
}

static int stm32f_uart_ioctl(
  rtems_libio_t  *iop,
  ioctl_command_t command,
  void           *arg
)
{
  stm32f_uart_device *pUartDevice =
    (stm32f_uart_device *) IMFS_generic_get_context_by_iop( iop );
  int      err;
  uint32_t baudrate;

  switch ( command ) {
    case UART_BAUDRATE:
      baudrate = (uint32_t) arg;

      uart_obtain( pUartDevice->pUart );
      pUartDevice->pUart->base_driver_info.baud = baudrate;
      err = pUartDevice->init(
      pUartDevice->pUart,
      pUartDevice->pUart->base_driver_info.baud
            );
      uart_release( pUartDevice->pUart );

      break;

    default:
      err = -ENOTTY;
      break;
  }

  if ( err == 0 ) {
    return 0;
  } else {
    rtems_set_errno_and_return_minus_one( -err );
  }
}

static const rtems_filesystem_file_handlers_r uart_handler = {
  .open_h = stm32f_uart_open,
  .close_h = stm32f_uart_close,
  .read_h = stm32f_uart_read,
  .write_h = stm32f_uart_write,
  .ioctl_h = stm32f_uart_ioctl,
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

static void uart_node_destroy( IMFS_jnode_t *node )
{
  stm32f_uart_device *pUartDevice;

  pUartDevice =
    (stm32f_uart_device *) IMFS_generic_get_context_by_node( node );
  ( *pUartDevice->destroy )( pUartDevice->pUart );

  IMFS_node_destroy_default( node );
}

static int uart_init(
  stm32f_uart_driver_entry *pUart,
  uint32_t                  baud
)
{
  rtems_status_code ret;

  //Configure the UART peripheral
  pUart->base_driver_info.handle->Instance =
    stm32f_uart_get_registers( pUart->base_driver_info.uart );
  pUart->base_driver_info.handle->Init.BaudRate = pUart->base_driver_info.baud;
  pUart->base_driver_info.handle->Init.WordLength = UART_WORDLENGTH_8B;
  pUart->base_driver_info.handle->Init.StopBits = UART_STOPBITS_1;
  pUart->base_driver_info.handle->Init.Parity = UART_PARITY_NONE;
  pUart->base_driver_info.handle->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  pUart->base_driver_info.handle->Init.Mode = UART_MODE_TX_RX;
  pUart->base_driver_info.handle->Init.OverSampling = UART_OVERSAMPLING_16;

  // Initialize UART pins, clocks, and DMA controllers
  if ( HAL_UART_Init( pUart->base_driver_info.handle ) == HAL_OK ) {
    ret = RTEMS_SUCCESSFUL;
  } else {
    ret = RTEMS_UNSATISFIED;
  }

  return ret;
}

static int uart_de_init( stm32f_uart_driver_entry *pUart )
{
  rtems_status_code ret;

  if ( HAL_UART_DeInit( pUart->base_driver_info.handle ) != HAL_OK ) {
    ret = RTEMS_UNSATISFIED;
  } else {
    ret = RTEMS_SUCCESSFUL;
  }

  return ret;
}

static void uart_destroy( stm32f_uart_driver_entry *pUart )
{
  (void) uart_de_init( pUart );
}

static const IMFS_node_control uart_node_control =
  IMFS_GENERIC_INITIALIZER(
    &uart_handler,
    IMFS_node_initialize_generic,
    uart_node_destroy
  );

static int uart_register_device_driver( stm32f_uart_device *pUartDevice )
{
  int               rv;
  rtems_status_code sc;

  rv = IMFS_make_generic_node(
    pUartDevice->pUart->base_driver_info.device_name,
    S_IFCHR | S_IRWXU | S_IRWXG | S_IRWXO,
    &uart_node_control,
    pUartDevice
       );

  if ( rv != 0 ) {
    ( *pUartDevice->destroy )( pUartDevice->pUart );
  } else {
    sc = rtems_task_start(
      pUartDevice->pUart->tx_task_id,
      pUartDevice->pUart->tx_task,
      (rtems_task_argument) pUartDevice->pUart
         );
  }

  return sc;
}

static int uart_create_rtems_objects( stm32f_uart_device *pUartDevice )
{
  rtems_status_code sc;

  char uart_num = '1' + (char) pUartDevice->pUart->base_driver_info.uart;

  sc = rtems_message_queue_create(
    rtems_build_name( 'U', 'T', 'X', uart_num ),
    UART_TX_QUEUE_LEN,
    MAX_UART_TX_MSG_SIZE,
    RTEMS_FIFO | RTEMS_LOCAL,
    &pUartDevice->pUart->tx_msg_queue
       );

  if ( sc != RTEMS_SUCCESSFUL ) {
    uart_destroy( pUartDevice->pUart );
    rtems_set_errno_and_return_minus_one( ENOMEM );
  }

  sc = rtems_task_create(
    rtems_build_name( 'U', 'T', 'X', uart_num ),
    UART_TX_TASK_PRIORITY,
    RTEMS_MINIMUM_STACK_SIZE,
    RTEMS_PREEMPT,
    RTEMS_NO_FLOATING_POINT,
    &pUartDevice->pUart->tx_task_id
       );

  if ( sc != RTEMS_SUCCESSFUL ) {
    uart_destroy( pUartDevice->pUart );
    rtems_set_errno_and_return_minus_one( ENOMEM );
  }

  pUartDevice->pUart->tx_task = stm32f_uart_tx_task;
  pUartDevice->destroy = uart_destroy;
  pUartDevice->init = uart_init;
  pUartDevice->de_init = uart_de_init;

  return 0;
}

void stm32f_uarts_initialize( void )
{
  uint32_t i;

  for ( i = 0; i < COUNTOF( stm32f_uart_driver_table ); i++ ) {
    stm32f_uart_driver_table[ i ].base_driver_info.handle =
      &( UartHandles[ i + NUM_PROCESSOR_CONSOLE_UARTS ] );
    stm32f_uart_driver_table[ i ].instance = i;
    uart_device_table[ i ].pUart = &stm32f_uart_driver_table[ i ];
    uart_create_rtems_objects( &uart_device_table[ i ] );
    stm32f_register_interrupt_handlers(
      uart_device_table[ i ].pUart->base_driver_info.handle );
    uart_register_device_driver( &uart_device_table[ i ] );
  }
}

