/**
 * @file uart.c
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
#include <rtems/system.h>
#include <rtems/rtems/status.h>
#include <rtems/score/isr.h>
#include <rtems/rtems/intr.h>
#include <stdio.h>
#include <stdlib.h>
#include <hal-utils.h>
#include <stm32f-processor-specific.h>

#include stm_processor_header(TARGET_STM_PROCESSOR_PREFIX)
#include stm_header(TARGET_STM_PROCESSOR_PREFIX, uart)
#include stm_header(TARGET_STM_PROCESSOR_PREFIX, gpio)
#include stm_header(TARGET_STM_PROCESSOR_PREFIX, gpio_ex)

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
  struct rtems_termios_tty *tty,
  rtems_termios_device_context *contex,
  rtems_libio_open_close_args_t *args
);

static bool stm32f_uart_set_attr(
  rtems_termios_device_context *context,
  const struct termios *term
);

static void stm32f_uart_write(
  rtems_termios_device_context *base,
  const char *buf,
  size_t len
);

static int stm32f_uart_poll_read(
  rtems_termios_device_context *context
);

static int stm32f_uart_get_next_tx_buf(stm32f_console_driver_entry* pUart,
                                       uint8_t *buf,
                                       size_t len
);

//--------------- Static data declarations -------------------------
static Ring_buffer_t      uart_fifo[NUM_PROCESSOR_UARTS];


//---------------
BSP_output_char_function_type     BSP_output_char = NULL;
BSP_polling_getchar_function_type BSP_poll_char   = NULL;

//--------------- Processor specific UART configuration
#include <console-config.c>

//--------------- termios handler functions
const rtems_termios_device_handler stm32f_uart_handlers_interrupt = {
  .first_open     = stm32f_uart_first_open,
  .last_close     = stm32f_uart_last_close,
  .poll_read      = stm32f_uart_poll_read,
  .write          = stm32f_uart_write,
  .set_attributes = stm32f_uart_set_attr,
  .mode           = TERMIOS_IRQ_DRIVEN
};

const rtems_termios_device_handler stm32f_uart_handlers_polling = {
  .first_open     = stm32f_uart_first_open,
  .last_close     = stm32f_uart_last_close,
  .poll_read      = stm32f_uart_poll_read,
  .write          = stm32f_uart_write,
  .set_attributes = stm32f_uart_set_attr,
  .mode           = TERMIOS_POLLED
};


//============================== ISR Definitions ==========================================
static void stm32f_uart_dma_tx_isr(
  void* argData
)
{
    stm32f_console_driver_entry* pUART = (stm32f_console_driver_entry*) argData;
    HAL_DMA_IRQHandler(pUART->base_driver_info.handle->hdmatx);
}


static void stm32f_uart_dma_rx_isr(
  void* argData
)
{
    stm32f_console_driver_entry* pUART = (stm32f_console_driver_entry*) argData;
    HAL_DMA_IRQHandler(pUART->base_driver_info.handle->hdmarx);
}


static void stm32f_uart_isr(
  void *arg
)
{
    stm32f_console_driver_entry* pUART = (stm32f_console_driver_entry*) arg;

  uint32_t u32_StartRxCount;

  // Remember how many TX and RX bytes we had before processing the
  // interrupt so that we can determine what happened in the HAL ISR
  u32_StartRxCount = pUART->base_driver_info.handle->RxXferCount;

  HAL_UART_IRQHandler(pUART->base_driver_info.handle);

  // Check to see if we received any characters, if so then
  // enqueue them in termios.  (The RxXferCount counts down from
  // the expected number of characters to receive.)
  if(u32_StartRxCount > pUART->base_driver_info.handle->RxXferCount) {

      int rxCount    = u32_StartRxCount - pUART->base_driver_info.handle->RxXferCount;
      char* pStartRx = (char*) pUART->base_driver_info.handle->pRxBuffPtr;
      pStartRx       -= rxCount;
      rtems_termios_enqueue_raw_characters(pUART->tty, pStartRx, rxCount);

      // re-enable interrupt to receive additional characters
      HAL_UART_Receive_IT(pUART->base_driver_info.handle, &pUART->rxChar, 1);
  }
}


int stm32f_uart_register_interrupt_handlers(stm32f_console_driver_entry* pUart){

    rtems_status_code ret = RTEMS_SUCCESSFUL;

    // Register DMA interrupt handlers (if necessary)
    if(pUart->base_driver_info.uartType == STM32F_UART_TYPE_DMA){
        if(ret == RTEMS_SUCCESSFUL) { ret = rtems_interrupt_handler_install( pUart->base_driver_info.RXDMA.DMAStreamInterruptNumber, NULL, 0, stm32f_uart_dma_rx_isr, pUart);}
        if(ret == RTEMS_SUCCESSFUL) { ret = rtems_interrupt_handler_install( pUart->base_driver_info.TXDMA.DMAStreamInterruptNumber, NULL, 0, stm32f_uart_dma_tx_isr, pUart);}
    }

    // Register UART interrupt handler for either DMA or Interrupt modes
    if(pUart->base_driver_info.uartType != STM32F_UART_TYPE_POLLING){

        if(ret == RTEMS_SUCCESSFUL) { ret = rtems_interrupt_handler_install( pUart->base_driver_info.UartInterruptNumber, NULL, 0, stm32f_uart_isr, pUart);}

        //Enable RX interrupt
        ret = (int) HAL_UART_Receive_IT(pUart->base_driver_info.handle, &pUart->rxChar, 1);
    }

    return (ret == RTEMS_SUCCESSFUL);
}


//============================== CONSOLE API FUNCTION ==========================================
rtems_device_driver console_initialize(
  rtems_device_major_number major,
  rtems_device_minor_number minor,
  void* arg
)
{
    rtems_status_code ret = RTEMS_SUCCESSFUL;

    rtems_termios_initialize();

    for(minor = 0; minor < COUNTOF(stm32f_console_driver_table); minor++){

        stm32f_console_driver_entry* pNextEntry = &stm32f_console_driver_table[minor];

        //Configure the UART peripheral
        pNextEntry->base_driver_info.handle->Instance          = stmf32_uart_get_registers(pNextEntry->base_driver_info.uart);
        pNextEntry->base_driver_info.handle->Init.BaudRate     = pNextEntry->base_driver_info.baud;
        pNextEntry->base_driver_info.handle->Init.WordLength   = UART_WORDLENGTH_8B;
        pNextEntry->base_driver_info.handle->Init.StopBits     = UART_STOPBITS_1;
        pNextEntry->base_driver_info.handle->Init.Parity       = UART_PARITY_NONE;
        pNextEntry->base_driver_info.handle->Init.HwFlowCtl    = UART_HWCONTROL_NONE;
        pNextEntry->base_driver_info.handle->Init.Mode         = UART_MODE_TX_RX;
        pNextEntry->base_driver_info.handle->Init.OverSampling = UART_OVERSAMPLING_16;

        // Initialize UART pins, clocks, and DMA controllers
        if(HAL_UART_Init(pNextEntry->base_driver_info.handle) != HAL_OK) { ret = RTEMS_UNSATISFIED;}

        if(pNextEntry->base_driver_info.uartType == STM32F_UART_TYPE_POLLING)
        {
            ret = rtems_termios_device_install(pNextEntry->base_driver_info.device_name,
                    major,
                    minor,
                    &stm32f_uart_handlers_polling,
                    NULL,
                    (rtems_termios_device_context*) pNextEntry);

        } else {
            ret = rtems_termios_device_install(pNextEntry->base_driver_info.device_name,
                    major,
                    minor,
                    &stm32f_uart_handlers_interrupt,
                    NULL,
                    (rtems_termios_device_context*) pNextEntry);
        }
    }

    return ret;
}


// =====================  TERMIOS CALLBACK FUNCTIONS ===========================
static bool stm32f_uart_first_open(
  struct rtems_termios_tty      *tty,
  rtems_termios_device_context  *context,
  struct termios                *term,
  rtems_libio_open_close_args_t *args){

    rtems_status_code            ret = RTEMS_SUCCESSFUL;
    stm32f_console_driver_entry* pUart = (stm32f_console_driver_entry*) context;

    // Initialize TTY
    pUart->tty = tty;

    // Configure initial baud rate
    rtems_termios_set_initial_baud(tty, pUart->base_driver_info.baud);

    stm32f_uart_register_interrupt_handlers(pUart);

    return (ret == RTEMS_SUCCESSFUL);
}


static void stm32f_uart_last_close(
  struct rtems_termios_tty *tty,
  rtems_termios_device_context *context,
  rtems_libio_open_close_args_t *args
)
{
    stm32f_console_driver_entry* pUart = (stm32f_console_driver_entry*) context;

    pUart->tty = NULL;

    if(pUart->base_driver_info.uartType != STM32F_UART_TYPE_POLLING){
        rtems_interrupt_handler_remove( pUart->base_driver_info.UartInterruptNumber, stm32f_uart_isr, tty);
    }
}


static bool stm32f_uart_set_attr(
  rtems_termios_device_context *context,
  const struct termios *term
)
{
    stm32f_console_driver_entry* pUart = (stm32f_console_driver_entry*) context;

    // initialize uart configuration with default values
    uint32_t parity       = UART_PARITY_NONE;
    uint32_t stop_bits    = UART_STOPBITS_1;
    uint32_t char_size    = UART_WORDLENGTH_8B;
    rtems_status_code ret = RTEMS_NOT_CONFIGURED;

    // determine baud rate
    int baud = rtems_termios_baud_to_number(term->c_cflag & CBAUD);

    // determine parity
    if(term->c_cflag & PARENB){
        if(term->c_cflag & PARODD) {
            parity = UART_PARITY_ODD;
        } else {
            parity = UART_PARITY_EVEN;
        }
    }

    // determine if two stops bits are requested
    if(term->c_cflag & CSTOPB){
        stop_bits = UART_STOPBITS_2;
    }

    // HAL only supports 8-bit characters
    char_size = UART_WORDLENGTH_8B;

    //##-1- Configure the UART peripheral ######################################
    pUart->base_driver_info.handle->Instance          = stmf32_uart_get_registers(pUart->base_driver_info.uart);
    pUart->base_driver_info.handle->Init.BaudRate     = baud;
    pUart->base_driver_info.handle->Init.WordLength   = char_size;
    pUart->base_driver_info.handle->Init.StopBits     = stop_bits;
    pUart->base_driver_info.handle->Init.Parity       = parity;
    pUart->base_driver_info.handle->Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    pUart->base_driver_info.handle->Init.Mode         = UART_MODE_TX_RX;
    pUart->base_driver_info.handle->Init.OverSampling = UART_OVERSAMPLING_16;

    // Initialize UART pins, clocks, and DMA controllers
    if(HAL_UART_Init(pUart->base_driver_info.handle) != HAL_OK) { ret = RTEMS_UNSATISFIED;}

    return (int) ret;
}


static int stm32f_uart_poll_read(
  rtems_termios_device_context *context
)
{
    HAL_StatusTypeDef ret;
    uint8_t next_char;

    stm32f_console_driver_entry* pUart = (stm32f_console_driver_entry*) context;

    // poll for a single character
    ret = (int) HAL_UART_Receive(pUart->base_driver_info.handle, &next_char, 1, 0);

    if(ret == HAL_OK){
        return (int) next_char;
    } else {
        return -1;
    }

    return ret;
}


static void stm32f_uart_write(
  rtems_termios_device_context *context,
  const char *buf,
  size_t len
)
{
    stm32f_console_driver_entry* pUart = (stm32f_console_driver_entry*) context;

    if((len > 0) && (buf != NULL)){
        if(pUart->base_driver_info.uartType == STM32F_UART_TYPE_POLLING){
            HAL_UART_Transmit(pUart->base_driver_info.handle, (uint8_t*) buf, len, POLLED_TX_TIMEOUT_ms);
        } else {
            stm32f_uart_get_next_tx_buf(pUart, (uint8_t*) buf, len);
        }
    }
}


static int stm32f_uart_get_next_tx_buf(stm32f_console_driver_entry* pUart,
                                       uint8_t *buf,
                                       size_t len
)
{
    int i;
    int error = (int) HAL_OK;
    uint16_t txlen;

    if((buf != NULL) || (len == 0)) {

        // First add in any new data
        for(i = 0; i < len; i++) {
            Ring_buffer_Add_character(pUart->fifo, buf[i]);
        }

        // if the uart is ready to transmit then transmit as much
        // data as possible into the tx buffer including any
        // data that was already in the ring buffer from previous
        // calls.
        if((pUart->base_driver_info.handle->State == HAL_UART_STATE_READY) ||
           (pUart->base_driver_info.handle->State == HAL_UART_STATE_BUSY_RX)){
            txlen = 0;

            while((Ring_buffer_Is_empty(pUart->fifo) == false) && (txlen < sizeof(pUart->tx_buffer))) {
                Ring_buffer_Remove_character(pUart->fifo, pUart->tx_buffer[txlen]);
                txlen++;
            }
        }

        // Check to see if it is busy transmitting.
        if(pUart->base_driver_info.uartType == STM32F_UART_TYPE_DMA) {
            if(pUart->tx_buffer != NULL) { error = (int) HAL_UART_Transmit_DMA(pUart->base_driver_info.handle, (uint8_t*) pUart->tx_buffer, txlen);}

        } else if (pUart->base_driver_info.uartType == STM32F_UART_TYPE_INT) {
            if(pUart->tx_buffer != NULL) { error = (int) HAL_UART_Transmit_IT(pUart->base_driver_info.handle, (uint8_t*) pUart->tx_buffer, txlen);}
        }

        // If the HAL Tx call was successful then dequeue characters from the termios queue
        if(error != HAL_BUSY) {
            rtems_termios_dequeue_characters(pUart->tty, txlen);
        }
    } else {
        error = -1;
    }

    return error;
}


void HAL_UART_TxCpltCallback(
  UART_HandleTypeDef *huart
)
{
    stm32f_console_driver_entry* pEntry = stm32f_get_console_driver_entry_from_handle(huart);

    // If there are still characters in TX fifo start sending again...
    if(pEntry != NULL){

        if(Ring_buffer_Is_empty(pEntry->fifo) == false) {
            stm32f_uart_get_next_tx_buf(pEntry, NULL, 0);
        }
    }
}


//=================== HAL UART weak symbol override functions =====================
void HAL_UART_MspInit(
  UART_HandleTypeDef *huart
)
{
  static DMA_HandleTypeDef hdma_tx;
  static DMA_HandleTypeDef hdma_rx;

  GPIO_InitTypeDef  GPIO_InitStruct;
  stm32f_base_uart_driver_entry* pUart;

  pUart = stm32f_get_driver_entry_from_handle(huart);

  //##-1- Enable peripherals and GPIO Clocks #################################

  // Enable Uart clocks
  stm32f_init_uart_clock(pUart->uart);

  // Enable GPIO clocks
  stmf32_init_gpio_clock(pUart->TXPin.port);
  stmf32_init_gpio_clock(pUart->RXPin.port);

  // Enable DMA clocks
  if(pUart->uartType == STM32F_UART_TYPE_DMA){
      stmf32_init_dma_clock(pUart->TXDMA.controller);
      stmf32_init_dma_clock(pUart->RXDMA.controller);
  }

  //##-2- Configure peripheral GPIO ##########################################

  // UART TX GPIO pin configuration
  GPIO_InitStruct.Pin       = (1 << pUart->TXPin.pin);
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = pUart->altFuncConfg;

  HAL_GPIO_Init(stmf32_get_gpio(pUart->TXPin.port), &GPIO_InitStruct);

  // UART RX GPIO pin configuration
  GPIO_InitStruct.Pin       = (1 << pUart->RXPin.pin);
  GPIO_InitStruct.Alternate = pUart->altFuncConfg;

  HAL_GPIO_Init(stmf32_get_gpio(pUart->RXPin.port), &GPIO_InitStruct);

  //##-3- Configure the DMA streams ##########################################

  if(pUart->uartType == STM32F_UART_TYPE_DMA){

      /* Configure the DMA handler for Transmission process */
      hdma_tx.Instance                 = pUart->TXDMAStream;
      hdma_tx.Init.Channel             = pUart->TXDMA.channel;

      hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
      hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
      hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
      hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
      hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
      hdma_tx.Init.Mode                = DMA_NORMAL;
      hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;
      hdma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
      hdma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
      hdma_tx.Init.MemBurst            = DMA_MBURST_INC4;
      hdma_tx.Init.PeriphBurst         = DMA_PBURST_INC4;

      HAL_DMA_Init(&hdma_tx);

      /* Associate the initialized DMA handle to the the UART handle */
      __HAL_LINKDMA(huart, hdmatx, hdma_tx);

      /* Configure the DMA handler for Transmission process */
      hdma_rx.Instance                 = pUart->RXDMAStream; ;
      hdma_rx.Init.Channel             = pUart->RXDMA.channel;

      hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
      hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
      hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
      hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
      hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
      hdma_rx.Init.Mode                = DMA_NORMAL;
      hdma_rx.Init.Priority            = DMA_PRIORITY_HIGH;
      hdma_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
      hdma_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
      hdma_rx.Init.MemBurst            = DMA_MBURST_INC4;
      hdma_rx.Init.PeriphBurst         = DMA_PBURST_INC4;

      HAL_DMA_Init(&hdma_rx);

      // Associate the initialized DMA handle to the the UART handle
      __HAL_LINKDMA(huart, hdmarx, hdma_rx);
  }
}

/**
  * @brief UART MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO, DMA and NVIC configuration to their default state
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspDeInit(
  UART_HandleTypeDef *huart
)
{
  static DMA_HandleTypeDef hdma_tx;
  static DMA_HandleTypeDef hdma_rx;

  stm32f_base_uart_driver_entry* pUart;

  pUart = stm32f_get_driver_entry_from_handle(huart);

  /*##-1- Reset peripherals ##################################################*/
  stmf32_uart_reset(pUart->uart);

  /*##-2- Disable peripherals and GPIO Clocks ################################*/
  /* Configure UART Tx as alternate function  */
  HAL_GPIO_DeInit(stmf32_get_gpio(pUart->TXPin.port), (1 << pUart->TXPin.pin));
  /* Configure UART Rx as alternate function  */
  HAL_GPIO_DeInit(stmf32_get_gpio(pUart->RXPin.port), (1 << pUart->RXPin.pin));

  /*##-3- Disable the DMA Streams ############################################*/
  if(pUart->uartType == STM32F_UART_TYPE_DMA){
      /* De-Initialize the DMA Stream associate to transmission process */
      HAL_DMA_DeInit(&hdma_tx);
      /* De-Initialize the DMA Stream associate to reception process */
      HAL_DMA_DeInit(&hdma_rx);
  }
}

