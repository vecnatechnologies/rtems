/**
 * @file uart-termios.c
 * @author Jay M. Doyle
 *
 * @ingroup uart
 *
 * @brief A universal termios UART driver implementation for all STM32FXXXX Cortex-M
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
#include <rtems/system.h>
#include <rtems/rtems/status.h>
#include <rtems/score/isr.h>
#include <rtems/rtems/intr.h>
#include <stdio.h>
#include <stdlib.h>
#include <hal-utils.h>
#include <stm32f-processor-specific.h>

#include stm_processor_header( TARGET_STM_PROCESSOR_PREFIX )
#include stm_header( TARGET_STM_PROCESSOR_PREFIX, dma )
#include stm_header( TARGET_STM_PROCESSOR_PREFIX, uart )
#include stm_header( TARGET_STM_PROCESSOR_PREFIX, gpio )
#include stm_header( TARGET_STM_PROCESSOR_PREFIX, gpio_ex )
#include stm_header( TARGET_STM_PROCESSOR_PREFIX, rcc_ex )

#include <termios.h>
#include <rtems/irq.h>
#include <rtems/libio.h>
#include <rtems/termiostypes.h>
#include <rtems/irq-extension.h>
#include <hal-uart-interface.h>

//------------------ Forward declarations ------------------
static bool stm32f_uart_first_open(
  struct rtems_termios_tty      *tty,
  rtems_termios_device_context  *context,
  struct termios                *term,
  rtems_libio_open_close_args_t *args
);

static void stm32f_uart_last_close(
  struct rtems_termios_tty      *tty,
  rtems_termios_device_context  *contex,
  rtems_libio_open_close_args_t *args
);

static bool stm32f_uart_set_attr(
  rtems_termios_device_context *context,
  const struct termios         *term
);

static void stm32f_uart_write(
  rtems_termios_device_context *base,
  const char                   *buf,
  size_t                        len
);

static int stm32f_uart_poll_read( rtems_termios_device_context *context );



//--------------- Processor specific UART configuration
#include <console-config.c>

//--------------- termios handler functions
const rtems_termios_device_handler stm32f_uart_handlers_polling = {
  .first_open = stm32f_uart_first_open,
  .last_close = stm32f_uart_last_close,
  .poll_read = stm32f_uart_poll_read,
  .write = stm32f_uart_write,
  .set_attributes = stm32f_uart_set_attr,
  .mode = TERMIOS_POLLED
};



//============================== CONSOLE API FUNCTION ==========================================
rtems_device_driver console_initialize(
  rtems_device_major_number major,
  rtems_device_minor_number minor,
  void                     *arg
)
{
  rtems_status_code ret = RTEMS_SUCCESSFUL;

  rtems_termios_initialize();

  for ( minor = 0; minor < COUNTOF( stm32f_console_driver_table ); minor++ ) {
    stm32f_console_driver_entry *pNextEntry =
      &stm32f_console_driver_table[ minor ];

    //Configure the UART peripheral
    pNextEntry->base_driver_info.handle->Instance = stm32f_uart_get_registers(
      pNextEntry->base_driver_info.uart );
    pNextEntry->base_driver_info.handle->Init.BaudRate =
      pNextEntry->base_driver_info.baud;
    pNextEntry->base_driver_info.handle->Init.WordLength = UART_WORDLENGTH_8B;
    pNextEntry->base_driver_info.handle->Init.StopBits = UART_STOPBITS_1;
    pNextEntry->base_driver_info.handle->Init.Parity = UART_PARITY_NONE;
    pNextEntry->base_driver_info.handle->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    pNextEntry->base_driver_info.handle->Init.Mode = UART_MODE_TX_RX;
    pNextEntry->base_driver_info.handle->Init.OverSampling =
      UART_OVERSAMPLING_16;

    // Initialize UART pins, clocks, and DMA controllers
    if ( HAL_UART_Init( pNextEntry->base_driver_info.handle ) != HAL_OK ) {
      ret = RTEMS_UNSATISFIED;
    } else {
      ret = rtems_termios_device_install(
        pNextEntry->base_driver_info.device_name,
        major,
        minor,
        &stm32f_uart_handlers_polling,
        NULL,
        (rtems_termios_device_context *) pNextEntry );
    }
  }

  return ret;
}

// =====================  TERMIOS CALLBACK FUNCTIONS ===========================
static bool stm32f_uart_first_open(
  struct rtems_termios_tty      *tty,
  rtems_termios_device_context  *context,
  struct termios                *term,
  rtems_libio_open_close_args_t *args
)
{
  rtems_status_code            ret = RTEMS_SUCCESSFUL;
  stm32f_console_driver_entry *pUart = (stm32f_console_driver_entry *) context;

  // Initialize TTY
  pUart->tty = tty;

  // Configure initial baud rate
  rtems_termios_set_initial_baud( tty, pUart->base_driver_info.baud );

  return ( ret == RTEMS_SUCCESSFUL );
}

static void stm32f_uart_last_close(
  struct rtems_termios_tty      *tty,
  rtems_termios_device_context  *context,
  rtems_libio_open_close_args_t *args
)
{
  stm32f_console_driver_entry *pUart = (stm32f_console_driver_entry *) context;

  pUart->tty = NULL;

  stm32f_remove_interrupt_handlers( pUart->base_driver_info.handle );
}

static bool stm32f_uart_set_attr(
  rtems_termios_device_context *context,
  const struct termios         *term
)
{
  stm32f_console_driver_entry *pUart = (stm32f_console_driver_entry *) context;

  // initialize uart configuration with default values
  uint32_t          parity = UART_PARITY_NONE;
  uint32_t          stop_bits = UART_STOPBITS_1;
  uint32_t          char_size = UART_WORDLENGTH_8B;
  rtems_status_code ret = RTEMS_NOT_CONFIGURED;

  // determine baud rate
  int baud = rtems_termios_baud_to_number( term->c_cflag & CBAUD );

  // determine parity
  if ( term->c_cflag & PARENB ) {
    if ( term->c_cflag & PARODD ) {
      parity = UART_PARITY_ODD;
    } else {
      parity = UART_PARITY_EVEN;
    }
  }

  // determine if two stops bits are requested
  if ( term->c_cflag & CSTOPB ) {
    stop_bits = UART_STOPBITS_2;
  }

  // HAL only supports 8-bit characters
  char_size = UART_WORDLENGTH_8B;

  //##-1- Configure the UART peripheral ######################################
  pUart->base_driver_info.handle->Instance = stm32f_uart_get_registers(
    pUart->base_driver_info.uart );
  pUart->base_driver_info.handle->Init.BaudRate = baud;
  pUart->base_driver_info.handle->Init.WordLength = char_size;
  pUart->base_driver_info.handle->Init.StopBits = stop_bits;
  pUart->base_driver_info.handle->Init.Parity = parity;
  pUart->base_driver_info.handle->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  pUart->base_driver_info.handle->Init.Mode = UART_MODE_TX_RX;
  pUart->base_driver_info.handle->Init.OverSampling = UART_OVERSAMPLING_16;

  // Initialize UART pins, clocks, and DMA controllers
  if ( HAL_UART_Init( pUart->base_driver_info.handle ) != HAL_OK ) {
    ret = RTEMS_UNSATISFIED;
  }

  return (int) ret;
}

static int stm32f_uart_poll_read( rtems_termios_device_context *context )
{
  HAL_StatusTypeDef ret;
  uint8_t           next_char;

  stm32f_console_driver_entry *pUart = (stm32f_console_driver_entry *) context;

  // poll for a single character
  ret = (int) HAL_UART_Receive(
    pUart->base_driver_info.handle,
    &next_char,
    1,
    0 );

  if ( ret == HAL_OK ) {
    return (int) next_char;
  } else {
    return -1;
  }

  return ret;
}

static void stm32f_uart_write(
  rtems_termios_device_context *context,
  const char                   *buf,
  size_t                        len
)
{
  stm32f_console_driver_entry *pUart = (stm32f_console_driver_entry *) context;
  HAL_StatusTypeDef            error;

  if ( ( len > 0 ) && ( buf != NULL ) ) {
    if ( pUart->base_driver_info.uart_mode == STM32F_UART_MODE_POLLING ) {
      error = HAL_UART_Transmit(
        pUart->base_driver_info.handle,
        (uint8_t *) buf,
        len,
        POLLED_TX_TIMEOUT_ms );

      if ( error != HAL_BUSY ) {
        rtems_termios_dequeue_characters( pUart->tty, len );
      }
    }
  }
}

static void _BSP_output_char(char c)
{
  stm32f_console_driver_entry* pConsole =  (stm32f_console_driver_entry*) stm32f_console_driver_table;

  if ( pConsole->base_driver_info.uart_mode == STM32F_UART_MODE_POLLING ) {

    HAL_UART_Transmit(
      pConsole->base_driver_info.handle,
      (uint8_t *) &c,
      1,
      POLLED_TX_TIMEOUT_ms );

    if(c == '\n'){

      c = '\r';

      HAL_UART_Transmit(
        pConsole->base_driver_info.handle,
        (uint8_t *) &c,
        1,
        POLLED_TX_TIMEOUT_ms );
    }
  }
}

static int _BSP_get_char_poll(void)
{
  char next_char;
  HAL_StatusTypeDef ret;
  stm32f_console_driver_entry* pConsole =  (stm32f_console_driver_entry*) stm32f_console_driver_table;

  ret = (int) HAL_UART_Receive(
    pConsole->base_driver_info.handle,
    &next_char,
    1,
    0 );

  if ( ret == HAL_OK ) {
    return (int) next_char;
  } else {
    return -1;
  }

  return ret;
}

// This pointers need to be defined for printk low-level
// kernel printing capabilities.
BSP_output_char_function_type     BSP_output_char = _BSP_output_char;
BSP_polling_getchar_function_type BSP_poll_char   = _BSP_get_char_poll;
