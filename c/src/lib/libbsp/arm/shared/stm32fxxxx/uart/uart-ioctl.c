/*
 * uart-ioctl.c
 *
 *  Created on: Jul 30, 2015
 *      Author: jay.doyle
 */


#include <rtems.h>

#include <hal-utils.h>
#include <hal-uart-interface.h>
#include <stm32f-processor-specific.h>

#include stm_processor_header(TARGET_STM_PROCESSOR_PREFIX)
#include stm_header(TARGET_STM_PROCESSOR_PREFIX, uart)

#include <rtems/imfs.h>

//--- Include processor specfic uart configurations
#include <uart-config.c>

static uint8_t tx_msg[MAX_UART_TX_MSG_SIZE];
//============================== ISR Definitions ==========================================
static void stm32f_uart_dma_tx_isr(
  void* argData
)
{
    stm32f_base_uart_driver_entry* pUART = (stm32f_base_uart_driver_entry*) argData;
    HAL_DMA_IRQHandler(pUART->handle->hdmatx);
}


static void stm32f_uart_dma_rx_isr(
  void* argData
)
{
    stm32f_base_uart_driver_entry* pUART = (stm32f_base_uart_driver_entry*) argData;
    HAL_DMA_IRQHandler(pUART->handle->hdmarx);
}


static void stm32f_uart_isr(
  void *arg
)
{
  stm32f_base_uart_driver_entry* pUART = (stm32f_base_uart_driver_entry*) arg;
  HAL_UART_IRQHandler(pUART->handle);
}


static HAL_StatusTypeDef stm32_uart_transmit(
        UART_HandleTypeDef* hUartHandle,
        const stm32f_uart_type uartType,
        uint8_t * buf,
        uint16_t len
)
{
    HAL_StatusTypeDef ret = HAL_ERROR;

    if((len > 0) && (buf != NULL) && (hUartHandle != NULL)) {

        switch(uartType) {

        case STM32F_UART_TYPE_POLLING:
            ret = HAL_UART_Transmit(hUartHandle, buf, len, POLLED_TX_TIMEOUT_ms);
            break;

        case STM32F_UART_TYPE_INT:
            ret = HAL_UART_Transmit_IT(hUartHandle, buf, len);
            break;

        case STM32F_UART_TYPE_DMA:
            ret = HAL_UART_Transmit_DMA(hUartHandle, buf, len);
            break;
        }
    }

  return ret;
}


static HAL_StatusTypeDef stm32_uart_receive(
        UART_HandleTypeDef* hUartHandle,
        const stm32f_uart_type uartType,
        uint8_t * buf,
        size_t    len
)
{
    HAL_StatusTypeDef ret = HAL_ERROR;

    if((len > 0) && (buf != NULL) && (hUartHandle != NULL)) {

        switch(uartType) {

        case STM32F_UART_TYPE_POLLING:
            ret = HAL_UART_Receive(hUartHandle, buf, (uint16_t) len, POLLED_TX_TIMEOUT_ms);
            break;

        case STM32F_UART_TYPE_INT:
            ret = HAL_UART_Receive_IT(hUartHandle, buf, (uint16_t) len);
            break;

        case STM32F_UART_TYPE_DMA:
            ret = HAL_UART_Receive_DMA(hUartHandle, buf, (uint16_t) len);
            break;
        }
    }

  return ret;
}


static rtems_task stm32_uart_tx_task(rtems_task_argument arg) {


  size_t len;
  rtems_status_code sc;
  uint32_t tx_count = 0UL;

  stm32_uart_driver_entry * pUart = (stm32_uart_driver_entry *) arg;

  while (1) {

    sc = rtems_message_queue_receive(pUart->tx_msg_queue,
                                     (void*) &tx_msg[tx_count % COUNTOF(tx_msg)],
                                     &len,
                                     RTEMS_WAIT,
                                     RTEMS_NO_TIMEOUT);
    tx_count++;
    _Assert(len <= sizeof(msg));

    if(sc == RTEMS_SUCCESSFUL) {
        (void) stm32_uart_transmit(pUart->base_driver_info.handle, pUart->base_driver_info.uartType, (uint8_t*) &tx_msg[tx_count % COUNTOF(tx_msg)], len);
    }

    while((pUart->base_driver_info.handle->State != HAL_UART_STATE_BUSY_RX) &&
          (pUart->base_driver_info.handle->State != HAL_UART_STATE_READY)) {
        (void) rtems_task_wake_after( 1 );
    }
  }
}


static rtems_task stm32_uart_rx_task(rtems_task_argument arg) {

  //rtems_status_code sc;
  //uint8_t msg[MAX_UART_TX_MSG_SIZE];
  //size_t len;
  //HAL_StatusTypeDef ret;

  rtems_interval    ticks;

  ticks = 5 * rtems_clock_get_ticks_per_second();

  //stm32_uart * pUart = (stm32_uart *) arg;

  while (1) {

    /*
    ret = stm32_uart_receive(pUart->base_driver_info.handle, pUart->base_driver_info.uartType, &msg, len);

    if(ret == HAL_OK) {
        sc = rtems_message_queue_send(pUart->tx_msg_queue,
                                      &msg,
                                      len,
                                      RTEMS_WAIT,
                                      RTEMS_NO_TIMEOUT);
    }
    _Assert(len <= sizeof(msg));


*/
      (void) rtems_task_wake_after( ticks );
  }
}


static void uart_obtain(stm32_uart_driver_entry *pUart)
{
  rtems_status_code sc;

  sc = rtems_semaphore_obtain(pUart->mutex, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
  _Assert(sc == RTEMS_SUCCESSFUL);
  (void) sc;
}


static void uart_release(stm32_uart_driver_entry * pUart)
{
  rtems_status_code sc;

  sc = rtems_semaphore_release(pUart->mutex);
  _Assert(sc == RTEMS_SUCCESSFUL);
  (void) sc;
}


static int uart_change_baudrate(
        stm32_uart_driver_entry * pUart,
        uint32_t baud
)
{
  uart_obtain(pUart);
  rtems_task_restart(pUart->rx_task_id, (uint32_t) pUart);
  rtems_task_restart(pUart->tx_task_id, (uint32_t) pUart);
  uart_release(pUart);
  return 0;
}


static ssize_t stm32_uart_read(
  rtems_libio_t *iop,
  void *buffer,
  size_t count
)
{
  //stm32_uart_device *pUartDevice = IMFS_generic_get_context_by_iop(iop);
  //int err;

  return 0;
/*
  if (err == 0) {
    return 0;
  } else {
    rtems_set_errno_and_return_minus_one(-err);
  }
  */
}


static ssize_t stm32_uart_write(
  rtems_libio_t *iop,
  const void *buffer,
  size_t count
)
{
    stm32_uart_device *pUartDevice = IMFS_generic_get_context_by_iop(iop);
  int err;
  rtems_status_code sc;

  if(count <= MAX_UART_TX_MSG_SIZE) {

      sc = rtems_message_queue_send(pUartDevice->pUart->tx_msg_queue,
                                    buffer,
                                    count);

      if (RTEMS_SUCCESSFUL != sc) {
        err = EIO;
      }
  } else {
      //TODO: What error code should this be
      err = EIO;
  }

  if (err == 0) {
    return 0;
  } else {
    rtems_set_errno_and_return_minus_one(-err);
  }
}


static int stm32_uart_close(
  rtems_libio_t *iop
)
{
    stm32_uart_device *pUartDevice = IMFS_generic_get_context_by_iop(iop);
  int err;

  uart_obtain(pUartDevice->pUart);
  err = pUartDevice->de_init(pUartDevice->pUart);
  uart_release(pUartDevice->pUart);

  if (err == 0) {
    return 0;
  } else {
    rtems_set_errno_and_return_minus_one(-err);
  }
}


static int stm32_uart_open(
  rtems_libio_t *iop,
  const char    *path,
  int            oflag,
  mode_t         mode
)
{
  stm32_uart_device *pUartDevice = IMFS_generic_get_context_by_iop(iop);
  int err;

  uart_obtain(pUartDevice->pUart);
  err = pUartDevice->init(pUartDevice->pUart, pUartDevice->pUart->base_driver_info.baud);
  uart_release(pUartDevice->pUart);

  if (err == 0) {
    return 0;
  } else {
    rtems_set_errno_and_return_minus_one(-err);
  }
}

static int stm32_uart_ioctl(
  rtems_libio_t *iop,
  ioctl_command_t command,
  void *arg
)
{
  stm32_uart_device *pUartDevice = IMFS_generic_get_context_by_iop(iop);
  int err;
  uint32_t baudrate;

  switch (command) {

    case UART_BAUDRATE:
      baudrate = *((uint32_t*) arg);
      uart_change_baudrate(pUartDevice->pUart, baudrate);
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


static const rtems_filesystem_file_handlers_r uart_handler = {
  .open_h      = stm32_uart_open,
  .close_h     = stm32_uart_close,
  .read_h      = stm32_uart_read,
  .write_h     = stm32_uart_write,
  .ioctl_h     = stm32_uart_ioctl,
  .lseek_h     = rtems_filesystem_default_lseek,
  .fstat_h     = IMFS_stat,
  .ftruncate_h = rtems_filesystem_default_ftruncate,
  .fsync_h     = rtems_filesystem_default_fsync_or_fdatasync,
  .fdatasync_h = rtems_filesystem_default_fsync_or_fdatasync,
  .fcntl_h     = rtems_filesystem_default_fcntl,
  .kqfilter_h  = rtems_filesystem_default_kqfilter,
  .poll_h      = rtems_filesystem_default_poll,
  .readv_h     = rtems_filesystem_default_readv,
  .writev_h    = rtems_filesystem_default_writev
};

static void uart_node_destroy(IMFS_jnode_t *node)
{
  stm32_uart_device *pUartDevice;

  pUartDevice = IMFS_generic_get_context_by_node(node);
  (*pUartDevice->destroy)(pUartDevice->pUart);

  IMFS_node_destroy_default(node);
}

int uart_register_interrupt_handlers(stm32_uart_driver_entry* pUart){

    rtems_status_code ret = RTEMS_SUCCESSFUL;

    // Register DMA interrupt handlers (if necessary)
    if(pUart->base_driver_info.uartType == STM32F_UART_TYPE_DMA){
        if(ret == RTEMS_SUCCESSFUL) { ret = rtems_interrupt_handler_install( pUart->base_driver_info.RXDMA.DMAStreamInterruptNumber, NULL, 0, stm32f_uart_dma_rx_isr, &(pUart->base_driver_info));}
        if(ret == RTEMS_SUCCESSFUL) { ret = rtems_interrupt_handler_install( pUart->base_driver_info.TXDMA.DMAStreamInterruptNumber, NULL, 0, stm32f_uart_dma_tx_isr, &(pUart->base_driver_info));}
    }

    // Register UART interrupt handler for either DMA or Interrupt modes
    if(pUart->base_driver_info.uartType != STM32F_UART_TYPE_POLLING){

        if(ret == RTEMS_SUCCESSFUL) { ret = rtems_interrupt_handler_install( pUart->base_driver_info.UartInterruptNumber, NULL, 0, stm32f_uart_isr, &(pUart->base_driver_info));}

        //Enable RX interrupt
        //ret = (int) HAL_UART_Receive_IT(pUart->handle, &pUart->rxChar, 1);
    }

    return (ret == RTEMS_SUCCESSFUL);
}

int uart_remove_interrupt_handlers(stm32_uart_driver_entry* pUart){

    rtems_status_code ret = RTEMS_SUCCESSFUL;

    // Register DMA interrupt handlers (if necessary)
    if(pUart->base_driver_info.uartType == STM32F_UART_TYPE_DMA){
        if(ret == RTEMS_SUCCESSFUL) { ret = rtems_interrupt_handler_remove( pUart->base_driver_info.RXDMA.DMAStreamInterruptNumber, stm32f_uart_dma_rx_isr, &pUart->base_driver_info);}
        if(ret == RTEMS_SUCCESSFUL) { ret = rtems_interrupt_handler_remove( pUart->base_driver_info.TXDMA.DMAStreamInterruptNumber, stm32f_uart_dma_tx_isr, &pUart->base_driver_info);}
    }

    // Register UART interrupt handler for either DMA or Interrupt modes
    if(pUart->base_driver_info.uartType != STM32F_UART_TYPE_POLLING){

        if(ret == RTEMS_SUCCESSFUL) { ret = rtems_interrupt_handler_remove( pUart->base_driver_info.UartInterruptNumber, stm32f_uart_isr, &pUart->base_driver_info);}

    }

    return (ret == RTEMS_SUCCESSFUL);
}


static int uart_init(
  stm32_uart_driver_entry * pUart,
  uint32_t baud
)
{
  rtems_status_code ret;

  //Configure the UART peripheral
  pUart->base_driver_info.handle->Instance          = stmf32_uart_get_registers(pUart->base_driver_info.uart);
  pUart->base_driver_info.handle->Init.BaudRate     = pUart->base_driver_info.baud;
  pUart->base_driver_info.handle->Init.WordLength   = UART_WORDLENGTH_8B;
  pUart->base_driver_info.handle->Init.StopBits     = UART_STOPBITS_1;
  pUart->base_driver_info.handle->Init.Parity       = UART_PARITY_NONE;
  pUart->base_driver_info.handle->Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  pUart->base_driver_info.handle->Init.Mode         = UART_MODE_TX_RX;
  pUart->base_driver_info.handle->Init.OverSampling = UART_OVERSAMPLING_16;

  // Initialize UART pins, clocks, and DMA controllers
  if(HAL_UART_Init(pUart->base_driver_info.handle) != HAL_OK) {

      ret = uart_register_interrupt_handlers(pUart);
  }
  else {
      ret = RTEMS_SUCCESSFUL;
  }

  return ret;
}


static int uart_de_init(
        stm32_uart_driver_entry * pUart
)
{
  rtems_status_code ret;

  if(HAL_UART_DeInit(pUart->base_driver_info.handle) != HAL_OK) {
      ret = uart_remove_interrupt_handlers(pUart);
  }
  else {
      ret = RTEMS_SUCCESSFUL;
  }

  return ret;
}

static void uart_destroy(
  stm32_uart_driver_entry * pUart
)
{
    (void) uart_de_init(pUart);
}


static const IMFS_node_control uart_node_control = IMFS_GENERIC_INITIALIZER(
  &uart_handler,
  IMFS_node_initialize_generic,
  uart_node_destroy
);

int uart_register(
    stm32_uart_device * pUartDevice
)
{
  int rv;
  rtems_status_code sc;

  rv = IMFS_make_generic_node(
          pUartDevice->pUart->base_driver_info.device_name,
          S_IFCHR | S_IRWXU | S_IRWXG | S_IRWXO,
          &uart_node_control,
          pUartDevice
  );

  if (rv != 0) {

    (*pUartDevice->destroy)(pUartDevice->pUart);

  } else {

      sc = rtems_task_start(
              pUartDevice->pUart->tx_task_id,
              pUartDevice->pUart->tx_task,
          (rtems_task_argument) pUartDevice->pUart
          );

      if(sc == RTEMS_SUCCESSFUL) {
          sc = rtems_task_start(
                  pUartDevice->pUart->rx_task_id,
                  pUartDevice->pUart->rx_task,
              (rtems_task_argument) pUartDevice->pUart
              );
      }
  }
  return sc;
}


static int uart_do_init(
        stm32_uart_device *pUartDevice
)
{
  rtems_status_code sc;

  char uart_num = '1'+ (char) pUartDevice->pUart->base_driver_info.uart;

  sc = rtems_semaphore_create(
    rtems_build_name('U', 'A', 'R', uart_num),
    1,
    RTEMS_BINARY_SEMAPHORE | RTEMS_INHERIT_PRIORITY | RTEMS_PRIORITY,
    0,
    &pUartDevice->pUart->mutex
  );

  if (sc != RTEMS_SUCCESSFUL) {
    uart_destroy(pUartDevice->pUart);
    rtems_set_errno_and_return_minus_one(ENOMEM);
  }

  sc = rtems_message_queue_create(
    rtems_build_name('U', 'R', 'X', uart_num),
    UART_QUEUE_LEN,
    MAX_UART_RX_MSG_SIZE,
    RTEMS_FIFO | RTEMS_LOCAL,
    &pUartDevice->pUart->rx_msg_queue
  );

  if (sc != RTEMS_SUCCESSFUL) {
    uart_destroy(pUartDevice->pUart);
    rtems_set_errno_and_return_minus_one(ENOMEM);
  }

  sc = rtems_message_queue_create(
    rtems_build_name('U', 'T', 'X', uart_num),
    UART_QUEUE_LEN,
    MAX_UART_TX_MSG_SIZE,
    RTEMS_FIFO | RTEMS_LOCAL,
    &pUartDevice->pUart->tx_msg_queue
  );

  if (sc != RTEMS_SUCCESSFUL) {
    uart_destroy(pUartDevice->pUart);
    rtems_set_errno_and_return_minus_one(ENOMEM);
  }

  sc = rtems_task_create(
    rtems_build_name('U', 'R', 'X', uart_num),
    UART_TASK_PRIORITY,
    RTEMS_MINIMUM_STACK_SIZE,
    RTEMS_PREEMPT,
    RTEMS_NO_FLOATING_POINT,
    &pUartDevice->pUart->rx_task_id
  );

  if (sc != RTEMS_SUCCESSFUL) {
    uart_destroy(pUartDevice->pUart);
    rtems_set_errno_and_return_minus_one(ENOMEM);
  }

  pUartDevice->pUart->rx_task = stm32_uart_rx_task;

  sc = rtems_task_create(
    rtems_build_name('U', 'T', 'X', uart_num),
    UART_TASK_PRIORITY,
    RTEMS_MINIMUM_STACK_SIZE,
    RTEMS_PREEMPT,
    RTEMS_NO_FLOATING_POINT,
    &pUartDevice->pUart->tx_task_id
  );

  if (sc != RTEMS_SUCCESSFUL) {
    uart_destroy(pUartDevice->pUart);
    rtems_set_errno_and_return_minus_one(ENOMEM);
  }

  pUartDevice->pUart->tx_task = stm32_uart_tx_task;
  pUartDevice->destroy        = uart_destroy;
  pUartDevice->init           = uart_init;
  pUartDevice->de_init        = uart_de_init;

  return 0;
}


void __uarts_initialize(
        void
)
{
  int i;

  for( i = 0 ; i < COUNTOF(stm32f_uart_driver_table); i++) {
      stm32f_uart_driver_table[i].base_driver_info.handle = &(UartHandles[i+NUM_PROCESSOR_CONSOLE_UARTS]);
      uart_device_table[i].pUart =  &stm32f_uart_driver_table[i];
      uart_do_init(&uart_device_table[i]);
      uart_register(&uart_device_table[i]);
  }
}

