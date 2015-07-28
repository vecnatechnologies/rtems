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
#include <vecna-utils.h>
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
#include <console-config.h>

#define DMA_BUFFER_LENGTH 128
#define POLLED_TX_TIMEOUT 1000

static uint8_t dummy_char;

//------------------ Forward declarations ------------------
static int STM32FUartFirstOpen(
  int major,
  int minor,
  void *arg
);

static int STM32FUartLastClose(
  int major,
  int minor,
  void *arg
);

static int STM32FUartSetAttr(
  int minor,
  const struct termios *t
);

static int STM32FUartWrite(
  int minor,
  const char* buf,
  size_t bufferSize
);

static int STM32FUartPollRead(
  int minor
);

static bool STM32FUartFirstOpenTTY(
  struct rtems_termios_tty      *tty,
  rtems_termios_device_context  *context,
  struct termios                *term,
  rtems_libio_open_close_args_t *args
);

static void STM32FUartLastCloseTTY(
  struct rtems_termios_tty *tty,
  rtems_termios_device_context *contex,
  rtems_libio_open_close_args_t *args
);

static bool STM32FUartSetAttrTTY(
  rtems_termios_device_context *context,
  const struct termios *term
);

static void STM32FUartWriteTTY(
  rtems_termios_device_context *base,
  const char *buf,
  size_t len
);

static int STM32FUartPollReadTTY(
  rtems_termios_device_context *context
);


//--------------- Static data declarations -------------------------
static rtems_device_minor_number m_consoleOuput = 0;
static Ring_buffer_t uart_fifo[NUM_PROCESSOR_UARTS];
static UART_HandleTypeDef UartHandles[NUM_PROCESSOR_UARTS];

//--------------- Processor specific UART configuration
#include <uart-config.c>


rtems_termios_callbacks stm32f_uart_callbacks = {
    .firstOpen            = STM32FUartFirstOpen,
    .lastClose            = STM32FUartLastClose,
    .pollRead             = NULL,
    .write                = STM32FUartWrite,
    .setAttributes        = STM32FUartSetAttr,
    .stopRemoteTx         = NULL,
    .startRemoteTx        = NULL,
    .outputUsesInterrupts = TERMIOS_IRQ_DRIVEN
};

const rtems_termios_device_handler stm32f_uart_interrupt_handlers = {
  .first_open = STM32FUartFirstOpenTTY,
  .last_close = STM32FUartLastCloseTTY,
  .poll_read = STM32FUartPollReadTTY,
  .write = STM32FUartWriteTTY,
  .set_attributes = STM32FUartSetAttrTTY,
  .mode = TERMIOS_IRQ_DRIVEN
};

rtems_termios_callbacks stm32f_uart_polling_callbacks = {
    .firstOpen            = STM32FUartFirstOpen,
    .lastClose            = STM32FUartLastClose,
    .pollRead             = STM32FUartPollRead,
    .write                = STM32FUartWrite,
    .setAttributes        = STM32FUartSetAttr,
    .stopRemoteTx         = NULL,
    .startRemoteTx        = NULL,
    .outputUsesInterrupts = TERMIOS_POLLED
};

const rtems_termios_device_handler stm32f_uart_polling_handlers = {
  .first_open = STM32FUartFirstOpenTTY,
  .last_close = STM32FUartLastCloseTTY,
  .poll_read = STM32FUartPollReadTTY,
  .write = STM32FUartWriteTTY,
  .set_attributes = STM32FUartSetAttrTTY,
  .mode = TERMIOS_POLLED
};


//============================== ISR Definitions ==========================================
static void uart_dma_tx_handler(
  void* argData
)
{
    stm32f_uart_driver_entry* pUART = (stm32f_uart_driver_entry*) argData;
    HAL_DMA_IRQHandler(pUART->handle->hdmatx);
}


static void uart_dma_rx_handler(
  void* argData
)
{
    stm32f_uart_driver_entry* pUART = (stm32f_uart_driver_entry*) argData;

    HAL_DMA_IRQHandler(pUART->handle->hdmarx);
}


static void rtems_uart_handler(
  void *arg
)
{
  stm32f_uart_driver_entry* pUART = (stm32f_uart_driver_entry*) arg;

  uint32_t u32_StartRxCount;

  // Remember how many TX and RX bytes we had before processing the
  // interrupt so that we can determine what happened in the HAL ISR
  u32_StartRxCount = pUART->handle->RxXferCount;

  HAL_UART_IRQHandler(pUART->handle);

  // Check to see if we received any characters, if so then
  // enqueue them in termios.  (The RxXferCount counts down from
  // the expected number of characters to receive.)
  if(u32_StartRxCount > pUART->handle->RxXferCount) {

      int rxCount = u32_StartRxCount - pUART->handle->RxXferCount;
      char* pStartRx = (char*) pUART->handle->pRxBuffPtr;
      pStartRx -= rxCount;
      rtems_termios_enqueue_raw_characters(pUART->tty, pStartRx, rxCount);

      HAL_UART_Receive_IT(pUART->handle, &dummy_char, 1);
  }
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

    for(minor = 0; minor < COUNTOF(stm32f_uart_driver_table); minor++){

        stm32f_uart_driver_entry* pNextEntry = &stm32f_uart_driver_table[minor];

        //Configure the UART peripheral
        pNextEntry->handle->Instance          = stmf32_uart_get_registers(pNextEntry->uart);
        pNextEntry->handle->Init.BaudRate     = pNextEntry->initial_baud;
        pNextEntry->handle->Init.WordLength   = UART_WORDLENGTH_8B;
        pNextEntry->handle->Init.StopBits     = UART_STOPBITS_1;
        pNextEntry->handle->Init.Parity       = UART_PARITY_NONE;
        pNextEntry->handle->Init.HwFlowCtl    = UART_HWCONTROL_NONE;
        pNextEntry->handle->Init.Mode         = UART_MODE_TX_RX;
        pNextEntry->handle->Init.OverSampling = UART_OVERSAMPLING_16;

        // Initialize UART pins, clocks, and DMA controllers
        if(HAL_UART_Init(pNextEntry->handle) != HAL_OK) { ret = RTEMS_UNSATISFIED;}

        if(pNextEntry->uartType == STM32F_UART_TYPE_POLLING)
        {
            ret = rtems_termios_device_install(pNextEntry->device_name, major, minor, &stm32f_uart_polling_handlers, NULL, (rtems_termios_device_context*) pNextEntry);

        } else {
            ret = rtems_termios_device_install(pNextEntry->device_name, major, minor, &stm32f_uart_interrupt_handlers, NULL, (rtems_termios_device_context*) pNextEntry);
        }
    }

    return ret;
}


static void output_char(
  char c
)
{
    stm32f_uart_driver_entry* pNextEntry = &stm32f_uart_driver_table[m_consoleOuput];

    struct rtems_termios_callbacks* pcallbacks;

    if((pNextEntry->uartType == STM32F_UART_TYPE_INT) ||
       (pNextEntry->uartType == STM32F_UART_TYPE_DMA)){
        pcallbacks = &stm32f_uart_callbacks;
    } else if(pNextEntry->uartType == STM32F_UART_TYPE_POLLING){
        pcallbacks = &stm32f_uart_polling_callbacks;
    }

  if (c == '\n') {
      char CRChar = '\r';
      pcallbacks->write(m_consoleOuput, &CRChar, 1);
  }

  pcallbacks->write(m_consoleOuput, &c, 1);
}

BSP_output_char_function_type BSP_output_char = output_char;
BSP_polling_getchar_function_type BSP_poll_char = NULL;


int STM32FUartRegisterInterruptHandlers(stm32f_uart_driver_entry* pUart){

    rtems_status_code ret = RTEMS_SUCCESSFUL;

    // Register DMA interrupt handlers (if necessary)
    if(pUart->uartType == STM32F_UART_TYPE_DMA){
        if(ret == RTEMS_SUCCESSFUL) { ret = rtems_interrupt_handler_install( pUart->RXDMA.DMAStreamInterruptNumber, NULL, 0, uart_dma_rx_handler, pUart);}
        if(ret == RTEMS_SUCCESSFUL) { ret = rtems_interrupt_handler_install( pUart->TXDMA.DMAStreamInterruptNumber, NULL, 0, uart_dma_tx_handler, pUart);}
    }

    // Register UART interrupt handler for either DMA or Interrupt modes
    if(pUart->uartType != STM32F_UART_TYPE_POLLING){

        if(ret == RTEMS_SUCCESSFUL) { ret = rtems_interrupt_handler_install( pUart->UartInterruptNumber, NULL, 0, rtems_uart_handler, pUart);}

        //Enable RX interrupt
        ret = (int) HAL_UART_Receive_IT(pUart->handle, &dummy_char, 1);
    }

    return (ret == RTEMS_SUCCESSFUL);
}

// =====================  TTY CALLBACK FUNCTIONS ===========================
static bool STM32FUartFirstOpenTTY(
  struct rtems_termios_tty      *tty,
  rtems_termios_device_context  *context,
  struct termios                *term,
  rtems_libio_open_close_args_t *args){

    rtems_status_code ret = RTEMS_SUCCESSFUL;
    stm32f_uart_driver_entry* pUart = (stm32f_uart_driver_entry*) context;

    // Initialize TTY
    pUart->tty = tty;

    // Configure initial baud rate
    rtems_termios_set_initial_baud(tty, pUart->initial_baud);

    STM32FUartRegisterInterruptHandlers(pUart);

    return (ret == RTEMS_SUCCESSFUL);
}


static int STM32FUartFirstOpen(
  int major,
  int minor,
  void *arg
)
{
    stm32f_uart_driver_entry* pSelectedUart = &stm32f_uart_driver_table[minor];
    struct rtems_termios_tty* tty = ((rtems_libio_open_close_args_t *) arg)->iop->data1;

    // Connect TTY data structure
    pSelectedUart->tty = tty;

    return rtems_termios_set_initial_baud(tty, pSelectedUart->initial_baud);
}


static void STM32FUartLastCloseTTY(
  struct rtems_termios_tty *tty,
  rtems_termios_device_context *context,
  rtems_libio_open_close_args_t *args
)
{
    stm32f_uart_driver_entry* pUart = (stm32f_uart_driver_entry*) context;

    pUart->tty = NULL;

    if(pUart->uartType != STM32F_UART_TYPE_POLLING){
        rtems_interrupt_handler_remove( pUart->UartInterruptNumber, rtems_uart_handler, tty);
    }
}


static int STM32FUartLastClose(
  int major,
  int minor,
  void *arg
)
{
    stm32f_uart_driver_entry* pSelectedUart = &stm32f_uart_driver_table[minor];

    pSelectedUart->tty = NULL;

    return 0;
}


static bool STM32FUartSetAttrTTY(
  rtems_termios_device_context *context,
  const struct termios *term
)
{
    stm32f_uart_driver_entry* pUart = (stm32f_uart_driver_entry*) context;

    uint32_t parity    = UART_PARITY_NONE;
    uint32_t stop_bits = UART_STOPBITS_1;
    uint32_t char_size = UART_WORDLENGTH_8B;
    rtems_status_code ret = RTEMS_NOT_CONFIGURED;

    // Determine baud rate
    int baud = rtems_termios_baud_to_number(term->c_cflag & CBAUD);

    // Determine parity
    if(term->c_cflag & PARENB){
        if(term->c_cflag & PARODD) {
            parity = UART_PARITY_ODD;
        } else {
            parity = UART_PARITY_EVEN;
        }
    }

    // Determine if two stops bits are requested
    if(term->c_cflag & CSTOPB){
        stop_bits = UART_STOPBITS_2;
    }

    // Determine character size
    switch(term->c_cflag & CSIZE){
    case CS5:
    case CS6:
    case CS7:
        // not supported
        break;

    case CS8:
        char_size = UART_WORDLENGTH_8B;
        break;
    }

    //##-1- Configure the UART peripheral ######################################
    pUart->handle->Instance          = stmf32_uart_get_registers(pUart->uart);
    pUart->handle->Init.BaudRate     = baud;
    pUart->handle->Init.WordLength   = char_size;
    pUart->handle->Init.StopBits     = stop_bits;
    pUart->handle->Init.Parity       = parity;
    pUart->handle->Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    pUart->handle->Init.Mode         = UART_MODE_TX_RX;
    pUart->handle->Init.OverSampling = UART_OVERSAMPLING_16;

    // Initialize UART pins, clocks, and DMA controllers
    if(HAL_UART_Init(pUart->handle) != HAL_OK) { ret = RTEMS_UNSATISFIED;}

    return (int) ret;
}


static int STM32FUartSetAttr(
  int minor,
  const struct termios *t
)
{
    uint32_t parity    = UART_PARITY_NONE;
    uint32_t stop_bits = UART_STOPBITS_1;
    uint32_t char_size = UART_WORDLENGTH_8B;
    rtems_status_code ret = RTEMS_NOT_CONFIGURED;

    // Determine baud rate
    int baud = rtems_termios_baud_to_number(t->c_cflag & CBAUD);

    // Determine parity
    if(t->c_cflag & PARENB){
        if(t->c_cflag & PARODD) {
            parity = UART_PARITY_ODD;
        } else {
            parity = UART_PARITY_EVEN;
        }
    }

    // Determine if two stops bits are requested
    if(t->c_cflag & CSTOPB){
        stop_bits = UART_STOPBITS_2;
    }

    // Determine character size
    switch(t->c_cflag & CSIZE){
    case CS5:
    case CS6:
    case CS7:
        // not supported
        break;

    case CS8:
        char_size = UART_WORDLENGTH_8B;
        break;
    }

    stm32f_uart_driver_entry* pNextEntry = &stm32f_uart_driver_table[minor];

    //##-1- Configure the UART peripheral ######################################
    pNextEntry->handle->Instance          = stmf32_uart_get_registers(pNextEntry->uart);
    pNextEntry->handle->Init.BaudRate     = baud;
    pNextEntry->handle->Init.WordLength   = char_size;
    pNextEntry->handle->Init.StopBits     = stop_bits;
    pNextEntry->handle->Init.Parity       = parity;
    pNextEntry->handle->Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    pNextEntry->handle->Init.Mode         = UART_MODE_TX_RX;
    pNextEntry->handle->Init.OverSampling = UART_OVERSAMPLING_16;

    // Initialize UART pins, clocks, and DMA controllers
    if(HAL_UART_Init(pNextEntry->handle) != HAL_OK) { ret = RTEMS_UNSATISFIED;}

    return (int) ret;
}


static int STM32FUartPollReadTTY(
  rtems_termios_device_context *context
)
{
    HAL_StatusTypeDef ret;
    uint8_t next_char;
    stm32f_uart_driver_entry* pUart = (stm32f_uart_driver_entry*) context;

    ret = (int) HAL_UART_Receive(pUart->handle, &next_char, 1, 0);

    if(ret == HAL_OK){
        return (int) next_char;
    } else {
        return -1;
    }

    return ret;
}


static int STM32FUartPollRead(
  int minor
)
{

    uint8_t ret = -1;
    stm32f_uart_driver_entry* pSelectedUart = &stm32f_uart_driver_table[minor];

    ret = (int) HAL_UART_Receive(pSelectedUart->handle, &ret, 1, 0UL);

    return ret;
}


static int stm32f_uart_get_next_tx_buf(stm32f_uart_driver_entry* pUart,
                                        uint8_t *buf,
                                        size_t len
)
{
    int i;
    int error = (int) HAL_OK;
    uint16_t txlen;
    static int busyCount = 0UL;

    if((buf != NULL) || (len == 0)) {

        // First add in any new data
        for(i = 0; i < len; i++) {
            Ring_buffer_Add_character(pUart->fifo, buf[i]);
        }

        // if the uart is ready to transmit then as much
        // data as possible into the tx buffer including any
        // data that was already in the ring buffer from previous
        // calls.
        if((pUart->handle->State == HAL_UART_STATE_READY) ||
           (pUart->handle->State == HAL_UART_STATE_BUSY_RX)){
            txlen = 0;

            while((Ring_buffer_Is_empty(pUart->fifo) == false) && (txlen < sizeof(pUart->tx_buffer))) {
                Ring_buffer_Remove_character(pUart->fifo, pUart->tx_buffer[txlen]);
                txlen++;
            }
        }

        // Check to see if it is busy transmitting.
        if(pUart->uartType == STM32F_UART_TYPE_DMA) {
            if(pUart->tx_buffer != NULL) { error = (int) HAL_UART_Transmit_DMA(pUart->handle, (uint8_t*) pUart->tx_buffer, txlen);}

        } else if (pUart->uartType == STM32F_UART_TYPE_INT) {
            if(pUart->tx_buffer != NULL) { error = (int) HAL_UART_Transmit_IT(pUart->handle, (uint8_t*) pUart->tx_buffer, txlen);}
        }

        if(error == HAL_BUSY) {
            busyCount++;
        } else {
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
    static int final = 0;

    stm32f_uart uartTxComplete = stm32f_get_driver_entry_from_handle(huart)->uart;

    // If there are still characters in TX fifo start sending again...
    if(uartTxComplete < COUNTOF(stm32f_uart_driver_table)){

        stm32f_uart_driver_entry* pEntry = &(stm32f_uart_driver_table[uartTxComplete]);

        if(Ring_buffer_Is_empty(pEntry->fifo) == false) {
            stm32f_uart_get_next_tx_buf(pEntry, NULL, 0);
        } else {
            final++;
        }
    }
}


static void STM32FUartWriteTTY(
  rtems_termios_device_context *context,
  const char *buf,
  size_t len
)
{
    stm32f_uart_driver_entry* pUart = (stm32f_uart_driver_entry*) context;

    if(len > 0) {
        if(pUart->uartType == STM32F_UART_TYPE_POLLING){
            HAL_UART_Transmit(pUart->handle, (uint8_t*) buf, len, POLLED_TX_TIMEOUT);
        } else {
            stm32f_uart_get_next_tx_buf(pUart, (uint8_t*) buf, len);
        }
    }
}


static int STM32FUartWrite(
  int minor,
  const char* buf,
  size_t bufferSize
)
{
    int error = (int) HAL_OK;

    // There is no need to check the value of minor number since it is derived
    // from the file descriptor.  The upper layer takes care that it is in a
    // valid range.
    stm32f_uart_driver_entry* pSelectedUart = &stm32f_uart_driver_table[minor];

    // Copy write request contents to TX buffer???

    switch(pSelectedUart->uartType){

    case STM32F_UART_TYPE_DMA:
        // Initiate DMA transfer
        error = (int) HAL_UART_Transmit_DMA(pSelectedUart->handle, (uint8_t*) buf, bufferSize);

        // Immediately dequeue from termios
        if(error == (int) HAL_OK) {
            rtems_termios_dequeue_characters(pSelectedUart->tty, bufferSize);
        }
    break;

    case STM32F_UART_TYPE_INT:
        error = (int) HAL_UART_Transmit_IT(pSelectedUart->handle, (uint8_t*) buf, bufferSize);
        break;

    case STM32F_UART_TYPE_POLLING:
        error = (int) HAL_UART_Transmit(pSelectedUart->handle, (uint8_t*) buf, bufferSize, POLLED_TX_TIMEOUT);
        break;

    default:
        error = -1;
        break;

    }

    return (int) error;
}


//=================== HAL UART weak symbol override functions =====================
void HAL_UART_MspInit(
  UART_HandleTypeDef *huart
)
{
  static DMA_HandleTypeDef hdma_tx;
  static DMA_HandleTypeDef hdma_rx;

  GPIO_InitTypeDef  GPIO_InitStruct;
  stm32f_uart_driver_entry* pUart;

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

  stm32f_uart_driver_entry* pUart;

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

