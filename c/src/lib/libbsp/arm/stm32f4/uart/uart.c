#include <rtems.h>
#include <rtems/system.h>
#include <rtems/rtems/status.h>
#include <rtems/score/isr.h>
#include <rtems/rtems/intr.h>
#include <stdio.h>
#include <stdlib.h>
#include <stm32f4xx_hal_conf.h>
#include <stm32f4xx_hal.h>
#include <stm32f407xx.h>
#include <termios.h>
#include <rtems/irq.h>
#include <rtems/libio.h>
#include <rtems/termiostypes.h>
#include <rtems/irq-extension.h>
#include <console-config.h>


#define DMA_BUFFER_LENGTH 128
//TODO: Define this value by processor
#define NUM_PROCESSOR_UARTS 6
#define POLLED_TX_TIMEOUT 1000

//TODO: Move these somewhere else
#define COUNTOF(x) (sizeof(x)/sizeof(x[0]))
uint32_t SystemCoreClock = 16000000UL;

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

static serial_fifo uart_fifo[NUM_PROCESSOR_UARTS] = {
   [0 ... NUM_PROCESSOR_UARTS-1] .count = 0UL,
   [0 ... NUM_PROCESSOR_UARTS-1] .head  = 0UL
};

static UART_HandleTypeDef UartHandles[NUM_PROCESSOR_UARTS];

static stm32f_uart_driver_entry stm32f_uart_driver_table[NUM_PROCESSOR_UARTS] = {

  //          UART1
  [0] .base                = RTEMS_TERMIOS_DEVICE_CONTEXT_INITIALIZER("STM32F UART1"),
  [0] .device_name         = "/dev/ttyS0",
  [0] .handle              = &(UartHandles[0]),
  [0] .fifo                = &(uart_fifo[0]),
  [0] .UartInterruptNumber = USART1_IRQn,
  [0] .uartType            = STM32F_UART_TYPE_INT,
  [0] .TXDMAStream         = DMA2_Stream7,
  [0] .RXDMAStream         = DMA2_Stream5,
  [0] .TXPin               = {STM32F_GOIO_PORTD, 6},
  [0] .RXPin               = {STM32F_GOIO_PORTD, 7},
  [0] .TXDMA               = {STM32F_DMA2_CONTROLLER, DMA2_Stream7_IRQn, DMA_CHANNEL_4, 7},
  [0] .RXDMA               = {STM32F_DMA2_CONTROLLER, DMA2_Stream5_IRQn, DMA_CHANNEL_4, 5},
  [0] .initial_baud        = 115200,
  [0] .altFuncConfg        = GPIO_AF7_USART1,

  //          UART2
  [1] .base                = RTEMS_TERMIOS_DEVICE_CONTEXT_INITIALIZER("STM32F UART2"),
  [1] .device_name     = "/dev/ttyS1",
  [1] .handle          = &(UartHandles[1]),
  [1] .fifo            = &(uart_fifo[1]),
  [1] .UartInterruptNumber = USART2_IRQn,
  [1] .uartType        = STM32F_UART_TYPE_DMA,
  [1] .TXDMAStream     = DMA1_Stream6,
  [1] .RXDMAStream     = DMA1_Stream5,
  [1] .TXPin           = {STM32F_GOIO_PORTD, 5},
  [1] .RXPin           = {STM32F_GOIO_PORTD, 6},
  [1] .TXDMA           = {STM32F_DMA1_CONTROLLER, DMA1_Stream6_IRQn, DMA_CHANNEL_4, 6},
  [1] .RXDMA           = {STM32F_DMA1_CONTROLLER, DMA1_Stream5_IRQn, DMA_CHANNEL_4, 5},
  [1] .initial_baud    = 115200,
  [1] .altFuncConfg    = GPIO_AF7_USART2,

  //          UART3
  [2] .base                = RTEMS_TERMIOS_DEVICE_CONTEXT_INITIALIZER("STM32F UART3"),
  [2] .device_name     = "/dev/ttyS2",
  [2] .handle          = &(UartHandles[2]),
  [2] .fifo            = &(uart_fifo[2]),
  [2] .UartInterruptNumber = USART3_IRQn,
  [2] .uartType        = STM32F_UART_TYPE_INT,
  [2] .TXDMAStream     = DMA1_Stream4,
  [2] .RXDMAStream     = DMA1_Stream1,
  [2] .TXPin           = {STM32F_GOIO_PORTD, 8},
  [2] .RXPin           = {STM32F_GOIO_PORTD, 9},
  [2] .TXDMA           = {STM32F_DMA1_CONTROLLER, DMA1_Stream4_IRQn, DMA_CHANNEL_7, 4},
  [2] .RXDMA           = {STM32F_DMA1_CONTROLLER, DMA1_Stream1_IRQn, DMA_CHANNEL_4, 1},
  [2] .initial_baud    = 115200,
  [2] .altFuncConfg    = GPIO_AF7_USART3,

  //          UART4
  [3] .base                = RTEMS_TERMIOS_DEVICE_CONTEXT_INITIALIZER("STM32F UART4"),
  [3] .device_name     = "/dev/ttyS3",
  [3] .handle          = &(UartHandles[3]),
  [3] .fifo            = &(uart_fifo[3]),
  [3] .UartInterruptNumber = UART4_IRQn,
  [3] .uartType        = STM32F_UART_TYPE_INT,
  [3] .TXDMAStream     = DMA1_Stream4,
  [3] .RXDMAStream     = DMA1_Stream2,
  [3] .TXPin           = {STM32F_GOIO_PORTC, 10},
  [3] .RXPin           = {STM32F_GOIO_PORTC, 11},
  [3] .TXDMA           = {STM32F_DMA1_CONTROLLER, DMA1_Stream4_IRQn, DMA_CHANNEL_4, 4},
  [3] .RXDMA           = {STM32F_DMA1_CONTROLLER, DMA1_Stream2_IRQn, DMA_CHANNEL_4, 2},
  [3] .initial_baud    = 115200,
  [3] .altFuncConfg    = GPIO_AF8_UART4,

  //          UART5
  [4] .base                = RTEMS_TERMIOS_DEVICE_CONTEXT_INITIALIZER("STM32F UART5"),
  [4] .device_name     = "/dev/ttyS4",
  [4] .handle          = &(UartHandles[4]),
  [4] .fifo            = &(uart_fifo[4]),
  [4] .UartInterruptNumber = UART5_IRQn,
  [4] .uartType        = STM32F_UART_TYPE_INT,
  [4] .TXDMAStream     = DMA1_Stream7,
  [4] .RXDMAStream     = DMA1_Stream0,
  [4] .TXPin           = {STM32F_GOIO_PORTD, 2},
  [4] .RXPin           = {STM32F_GOIO_PORTC, 12},
  [4] .TXDMA           = {STM32F_DMA1_CONTROLLER, DMA1_Stream7_IRQn, DMA_CHANNEL_4, 7},
  [4] .RXDMA           = {STM32F_DMA1_CONTROLLER, DMA1_Stream0_IRQn, DMA_CHANNEL_4, 0},
  [4] .initial_baud    = 115200,
  [4] .altFuncConfg    = GPIO_AF8_UART5,

  //          UART6
  [5] .base                = RTEMS_TERMIOS_DEVICE_CONTEXT_INITIALIZER("STM32F UART6"),
  [5] .device_name     = "/dev/console",
  [5] .handle          = &(UartHandles[5]),
  [5] .fifo            = &(uart_fifo[5]),
  [5] .UartInterruptNumber = USART6_IRQn,
  [5] .uartType        = STM32F_UART_TYPE_POLLING,
  [5] .TXDMAStream     = DMA2_Stream6,
  [5] .RXDMAStream     = DMA2_Stream2,
  [5] .TXPin           = {STM32F_GOIO_PORTC, 6},
  [5] .RXPin           = {STM32F_GOIO_PORTC, 7},
  [5] .TXDMA           = {STM32F_DMA2_CONTROLLER, DMA2_Stream6_IRQn, DMA_CHANNEL_5, 6},
  [5] .RXDMA           = {STM32F_DMA2_CONTROLLER, DMA2_Stream2_IRQn, DMA_CHANNEL_5, 2},
  [5] .initial_baud    = 115200,
  [5] .altFuncConfg    = GPIO_AF8_USART6,
};


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
  .poll_read = NULL,
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


//================== FIFO functions =================================
static serial_fifo_error serialFifoAppend(
  serial_fifo* pfifo,
  uint8_t* buf,
  const uint32_t len
)
{

    int i;
    serial_fifo_error ret = SERIAL_FIFO_NO_ERROR;

    if((pfifo != NULL) && (buf != NULL)){

        for(i = 0; i < len; i++){
          if(pfifo->count < COUNTOF(pfifo->buffer)){
              pfifo->buffer[(pfifo->head + pfifo->count + i) % COUNTOF(pfifo->buffer)] = buf[i];
              pfifo->count++;
          } else {
              ret = SERIAL_FIFO_BUFFER_FULL;
              break;
          }
        }
    } else {
        ret = SERIAL_FIFO_NULL_POINTER;
    }

    return ret;
}

static serial_fifo_error serialFifoRemove(
  serial_fifo* pfifo,
  uint8_t* buf,
  const uint32_t len,
  const bool copyData
)
{
    int i;
    serial_fifo_error ret = SERIAL_FIFO_NO_ERROR;

    if(!((pfifo == NULL) || ((copyData == true) && (buf == NULL)))){

        for(i = 0; i < len; i++){
          if(pfifo->count > 0UL){
              if(copyData == true){
                  buf[i] = pfifo->buffer[pfifo->head];
              }
              pfifo->count--;
              pfifo->head = (pfifo->head + 1) % COUNTOF(pfifo->buffer);
          } else {
              ret = SERIAL_FIFO_BUFFER_EMPTY;
              break;
          }
        }
    } else {
        ret = SERIAL_FIFO_NULL_POINTER;
    }

    return ret;
}


static uint8_t* serialFifoGetNextContiguousBlock(
  serial_fifo* pfifo,
  uint32_t* len
)
{

    uint8_t* ret = NULL;

    if((pfifo != NULL) && (len != NULL)){
          if(pfifo->count > 0UL){

            // the start of the contiguous block is always the head
            ret = &(pfifo->buffer[pfifo->head]);

            // find the maximum number of bytes to remove (either
            // the number of bytes or the number of bytes until the end of the buffer.
            if((pfifo->head + pfifo->count) <= COUNTOF(pfifo->buffer) ) {
                *len = pfifo->count;
            } else {
                *len = COUNTOF(pfifo->buffer) - pfifo->head;
            }

          }
    }

    return ret;
}


//================== STMF32 Support Functions =================================
static stm32f_uart STM32FUartGetUartFromHandle(
  const UART_HandleTypeDef *huart
)
{
   uint32_t i;
   stm32f_uart ret = STM32F_INVALID_UART;

   for(i = 0UL; i < COUNTOF(stm32f_uart_driver_table) ; i++){
       if(stm32f_uart_driver_table[i].handle == huart){
           ret = (stm32f_uart) i;
           break;
       }
   }

   return ret;
}


static void STM32FInitUartClock(
  const stm32f_uart Uart
)
{
    switch(Uart)
    {
    case STM32F_UART1:
        __HAL_RCC_USART1_CLK_ENABLE();
        break;

    case STM32F_UART2:
        __HAL_RCC_USART2_CLK_ENABLE();
        break;

    case STM32F_UART3:
        __HAL_RCC_USART3_CLK_ENABLE();
        break;

    case STM32F_UART4:
        __HAL_RCC_UART4_CLK_ENABLE();
        break;

    case STM32F_UART5:
        __HAL_RCC_UART5_CLK_ENABLE();
        break;

    case STM32F_UART6:
        __HAL_RCC_USART6_CLK_ENABLE();
        break;

    case STM32F_INVALID_UART:
        return;
        break;
    }
}


static void STM32FUartReset(
  const stm32f_uart Uart
)
{
    switch(Uart)
    {
    case STM32F_UART1:
        __HAL_RCC_USART1_FORCE_RESET();
        __HAL_RCC_USART1_RELEASE_RESET();
        break;

    case STM32F_UART2:
        __HAL_RCC_USART2_FORCE_RESET();
        __HAL_RCC_USART2_RELEASE_RESET();
        break;

    case STM32F_UART3:
        __HAL_RCC_USART3_FORCE_RESET();
        __HAL_RCC_USART3_RELEASE_RESET();
        break;

    case STM32F_UART4:
        __HAL_RCC_UART4_FORCE_RESET();
        __HAL_RCC_UART4_RELEASE_RESET();
        break;

    case STM32F_UART5:
        __HAL_RCC_UART5_FORCE_RESET();
        __HAL_RCC_UART5_RELEASE_RESET();
        break;

    case STM32F_UART6:
        __HAL_RCC_USART6_FORCE_RESET();
        __HAL_RCC_USART6_RELEASE_RESET();
        break;

    case STM32F_INVALID_UART:
        return;
        break;
    }
}


static void STM32FInitGPIOClock(
  const stm32f_gpio_port port
)
{
    switch(port){

    case STM32F_GOIO_PORTA:
        __HAL_RCC_GPIOA_CLK_ENABLE();
        break;
    case STM32F_GOIO_PORTB:
        __HAL_RCC_GPIOB_CLK_ENABLE();
        break;
    case STM32F_GOIO_PORTC:
        __HAL_RCC_GPIOC_CLK_ENABLE();
        break;
    case STM32F_GOIO_PORTD:
        __HAL_RCC_GPIOD_CLK_ENABLE();
        break;
    case STM32F_GOIO_PORTE:
        __HAL_RCC_GPIOE_CLK_ENABLE();
        break;
    case STM32F_GOIO_PORTF:
        __HAL_RCC_GPIOF_CLK_ENABLE();
        break;
    case STM32F_GOIO_PORTG:
        __HAL_RCC_GPIOG_CLK_ENABLE();
        break;
    case STM32F_GOIO_PORTH:
        __HAL_RCC_GPIOH_CLK_ENABLE();
        break;
    case STM32F_GOIO_PORTI:
        __HAL_RCC_GPIOI_CLK_ENABLE();
        break;

    default:
        break;
    }
}


static GPIO_TypeDef* STM32FGetGPIO(
  const stm32f_gpio_port port
)
{
    GPIO_TypeDef * ret = NULL;

    switch(port){

    case STM32F_GOIO_PORTA:
        ret = GPIOA;
        break;
    case STM32F_GOIO_PORTB:
        ret = GPIOB;
        break;
    case STM32F_GOIO_PORTC:
        ret = GPIOC;
        break;
    case STM32F_GOIO_PORTD:
        ret = GPIOD;
        break;
    case STM32F_GOIO_PORTE:
        ret = GPIOE;
        break;
    case STM32F_GOIO_PORTF:
        ret = GPIOF;
        break;
    case STM32F_GOIO_PORTG:
        ret = GPIOG;
        break;
    case STM32F_GOIO_PORTH:
        ret = GPIOH;
        break;
    case STM32F_GOIO_PORTI:
        ret = GPIOI;
        break;

    default:
        break;
    }

    return ret;
}


static USART_TypeDef* STM32FUartGetRegisters(
  const stm32f_uart Uart
)
{
    USART_TypeDef* ret = NULL;

    switch(Uart)
    {
    case STM32F_UART1:
        ret = USART1;
        break;

    case STM32F_UART2:
        ret = USART2;
        break;

    case STM32F_UART3:
        ret = USART3;
        break;

    case STM32F_UART4:
        ret = UART4;
        break;

    case STM32F_UART5:
        ret = UART5;
        break;

    case STM32F_UART6:
        ret = USART6;
        break;

    case STM32F_INVALID_UART:
        break;
    }

    return ret;
}


static void STM32FInitDMAClock(
  const stm32f_dma_controller controller
)
{

    if(controller == STM32F_DMA1_CONTROLLER) {
        __HAL_RCC_DMA1_CLK_ENABLE();
    } else if(controller == STM32F_DMA2_CONTROLLER) {
        __HAL_RCC_DMA2_CLK_ENABLE();
    }
}


//============================== ISR Definitions ==========================================
static void UART_DMA_TX_Handler(
  void* argData
)
{

    stm32f_uart_driver_entry* pUART = (stm32f_uart_driver_entry*) argData;
    uint32_t u32_StartCount = pUART->handle->TxXferCount;

    HAL_DMA_IRQHandler(pUART->handle->hdmatx);

    // Check to see if we have transmitted any characters, if so them
    // remove them from the
    if(u32_StartCount > pUART->handle->TxXferCount) {
        serialFifoRemove(pUART->fifo, NULL, (u32_StartCount - pUART->handle->TxXferCount), false);
        rtems_termios_dequeue_characters(pUART->tty, (u32_StartCount - pUART->handle->TxXferCount));
    }
}


static void UART_DMA_RX_Handler(
  void* argData
)
{
    stm32f_uart_driver_entry* pUART = (stm32f_uart_driver_entry*) argData;

    HAL_DMA_IRQHandler(pUART->handle->hdmarx);
}


static void RTEMS_UART_Handler(
  void *arg
)
{
  stm32f_uart_driver_entry* pUART = (stm32f_uart_driver_entry*) arg;

  uint32_t u32_StartRxCount;
  uint32_t u32_StartTxCount;

  // Remember how many TX and RX bytes we had before processing the
  // interrupt so that we can determine what happened in the HAL ISR
  u32_StartRxCount = pUART->handle->RxXferCount;
  u32_StartTxCount = pUART->handle->TxXferCount;

  HAL_UART_IRQHandler(pUART->handle);

  // Check to see if we received any characters, if so then
  // enqueue them in termios.  (The RxXferCount counts down from
  // the expected number of characters to receive.)
  if(u32_StartRxCount > pUART->handle->RxXferCount) {
      rtems_termios_enqueue_raw_characters(pUART->tty, (char*) &(pUART->handle->pRxBuffPtr[u32_StartRxCount]), pUART->handle->RxXferCount - u32_StartRxCount);
  }

  // Check to see if we have transmitted any characters, if so them
  // remove them from the =
  if(u32_StartTxCount > pUART->handle->TxXferCount) {

      // If we have finished transferring then HAL_UART_IRQHandler has reset TxXferCount to 0
      //txCount = (pUART->handle->TxXferCount == 0) ? u32_StartTxCount : ( pUART->handle->TxXferCount - u32_StartTxCount);

      serialFifoRemove(pUART->fifo, NULL, (u32_StartTxCount - pUART->handle->TxXferCount), false);
      rtems_termios_dequeue_characters(pUART->tty, (u32_StartTxCount - pUART->handle->TxXferCount));
  }
}


static void UART_Handler(
  rtems_vector_number vector,
  void* argData
)
{
    uint32_t u32_StartRxCount;
    uint32_t u32_StartTxCount;

    //TODO: what is type of argData?  It might be rtems_termios_tty*

    stm32f_uart_driver_entry* pUART = (stm32f_uart_driver_entry*) argData;

    // Remember how many TX and RX bytes we had before processing the
    // interrupt so that we can determine what happened in the HAL ISR
    u32_StartRxCount = pUART->handle->RxXferCount;
    u32_StartTxCount = pUART->handle->TxXferCount;

    HAL_UART_IRQHandler(pUART->handle);

    // Check to see if we received any characters, if so then
    // enqueue them in termios
    if(u32_StartRxCount > pUART->handle->RxXferCount) {
        rtems_termios_enqueue_raw_characters(pUART->tty, (char*) &(pUART->handle->pRxBuffPtr[u32_StartRxCount]), pUART->handle->RxXferCount - u32_StartRxCount);
    }

    // Check to see if we have transmitted any characters, if so them
    // remove them from the =
    if(u32_StartTxCount > pUART->handle->TxXferCount) {
        rtems_termios_dequeue_characters(pUART->tty, pUART->handle->TxXferCount - u32_StartTxCount);
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
        pNextEntry->handle->Instance          = STM32FUartGetRegisters(STM32FUartGetUartFromHandle(pNextEntry->handle));
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

    // Register DMA interrupt handlers (if necessary)
    if(pUart->uartType == STM32F_UART_TYPE_DMA){
        if(ret == RTEMS_SUCCESSFUL) { ret = rtems_interrupt_handler_install( pUart->RXDMA.DMAStreamInterruptNumber, NULL, 0, UART_DMA_RX_Handler, pUart->handle->hdmarx);}
        if(ret == RTEMS_SUCCESSFUL) { ret = rtems_interrupt_handler_install( pUart->TXDMA.DMAStreamInterruptNumber, NULL, 0, UART_DMA_TX_Handler, pUart->handle->hdmatx);}
    }

    // Register UART interrupt handler fpr either DMA or Interrupt modes
    if(pUart->uartType != STM32F_UART_TYPE_POLLING){
        if(ret == RTEMS_SUCCESSFUL) { ret = rtems_interrupt_handler_install( pUart->UartInterruptNumber, NULL, 0, RTEMS_UART_Handler, pUart);}
    }

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
        rtems_interrupt_handler_remove( pUart->UartInterruptNumber, RTEMS_UART_Handler, tty);
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
    pUart->handle->Instance          = STM32FUartGetRegisters(STM32FUartGetUartFromHandle(pUart->handle));
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
    pNextEntry->handle->Instance          = STM32FUartGetRegisters(STM32FUartGetUartFromHandle(pNextEntry->handle));
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

    ret = (int) HAL_UART_Receive(pUart->handle, &next_char, 1, HAL_MAX_DELAY);

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


void HAL_UART_TxCpltCallback(
  UART_HandleTypeDef *huart
)
{
    static int final = 0;

    stm32f_uart uartTxComplete = STM32FUartGetUartFromHandle(huart);

    // If there are still characters in TX fifo start sending again...
    if(uartTxComplete < COUNTOF(stm32f_uart_driver_table)){

        stm32f_uart_driver_entry* pEntry = &(stm32f_uart_driver_table[uartTxComplete]);

        if(pEntry->fifo->count > 0){

            uint8_t* txbuf = NULL;
            uint16_t txlen = 0UL;

            txbuf = serialFifoGetNextContiguousBlock(pEntry->fifo, &txlen);
            if(txbuf != NULL) { HAL_UART_Transmit_IT(pEntry->handle, txbuf, txlen);}
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
    static int busyCount = 0UL;
    int error = (int) HAL_OK;
    uint8_t* txbuf;
    uint16_t txlen;
    stm32f_uart_driver_entry* pUart = (stm32f_uart_driver_entry*) context;

    if(len > 0) {

        switch(pUart->uartType){

        case STM32F_UART_TYPE_DMA:
            // Initiate DMA transfer
            error = (int) HAL_UART_Transmit_DMA(pUart->handle, (uint8_t*) buf, len);

            /*
            // Immediately dequeue from termios
            if(error == (int) HAL_OK) {
                rtems_termios_dequeue_characters(pUart->tty, len);
            }
            */
        break;

        case STM32F_UART_TYPE_INT:
            serialFifoAppend(pUart->fifo, (uint8_t*) buf, len);
            txbuf = serialFifoGetNextContiguousBlock(pUart->fifo, &txlen);
            if(txbuf != NULL) { error = (int) HAL_UART_Transmit_IT(pUart->handle, txbuf, txlen);}
            if(error == HAL_BUSY) {
                busyCount++;
            }
            break;

        case STM32F_UART_TYPE_POLLING:
            error = (int) HAL_UART_Transmit(pUart->handle, (uint8_t*) buf, len, POLLED_TX_TIMEOUT);
            break;

        default:
            error = -1;
            break;

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
  stm32f_uart Uart = STM32FUartGetUartFromHandle(huart);

  //##-1- Enable peripherals and GPIO Clocks #################################

  // Get driver table entry
  pUart = (stm32f_uart_driver_entry*) &(stm32f_uart_driver_table[Uart]);

  // Enable Uart clocks
  STM32FInitUartClock(Uart);

  // Enable GPIO clocks
  STM32FInitGPIOClock(pUart->TXPin.port);
  STM32FInitGPIOClock(pUart->RXPin.port);

  // Enable DMA clocks
  if(pUart->uartType == STM32F_UART_TYPE_DMA){
      STM32FInitDMAClock(pUart->TXDMA.controller);
      STM32FInitDMAClock(pUart->RXDMA.controller);
  }

  //##-2- Configure peripheral GPIO ##########################################

  // UART TX GPIO pin configuration
  GPIO_InitStruct.Pin       = (1 << pUart->TXPin.pin);
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = pUart->altFuncConfg;

  HAL_GPIO_Init(STM32FGetGPIO(pUart->TXPin.port), &GPIO_InitStruct);

  // UART RX GPIO pin configuration
  GPIO_InitStruct.Pin       = (1 << pUart->RXPin.pin);
  GPIO_InitStruct.Alternate = pUart->altFuncConfg;

  HAL_GPIO_Init(STM32FGetGPIO(pUart->RXPin.port), &GPIO_InitStruct);

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
  stm32f_uart Uart = STM32FUartGetUartFromHandle(huart);
  pUart = (stm32f_uart_driver_entry*) &(stm32f_uart_driver_table[Uart]);

  /*##-1- Reset peripherals ##################################################*/
  STM32FUartReset(Uart);

  /*##-2- Disable peripherals and GPIO Clocks ################################*/
  /* Configure UART Tx as alternate function  */
  HAL_GPIO_DeInit(STM32FGetGPIO(pUart->TXPin.port), (1 << pUart->TXPin.pin));
  /* Configure UART Rx as alternate function  */
  HAL_GPIO_DeInit(STM32FGetGPIO(pUart->RXPin.port), (1 << pUart->RXPin.pin));

  /*##-3- Disable the DMA Streams ############################################*/
  if(pUart->uartType == STM32F_UART_TYPE_DMA){
      /* De-Initialize the DMA Stream associate to transmission process */
      HAL_DMA_DeInit(&hdma_tx);
      /* De-Initialize the DMA Stream associate to reception process */
      HAL_DMA_DeInit(&hdma_rx);
  }
}


/*
rtems_device_driver console_initialize(rtems_device_major_number major,
                                       rtems_device_minor_number minor,
                                       void* arg);

rtems_device_driver console_open(rtems_device_major_number major,
                                  rtems_device_minor_number minor,
                                  void* arg);
rtems_device_driver console_close(rtems_device_major_number major,
                                  rtems_device_minor_number minor,
                                  void* arg);
rtems_device_driver console_read(rtems_device_major_number major,
                                  rtems_device_minor_number minor,
                                  void* arg);
rtems_device_driver console_write(rtems_device_major_number major,
                                  rtems_device_minor_number minor,
                                  void* arg);
rtems_device_driver console_control(rtems_device_major_number major,
                                  rtems_device_minor_number minor,
                                  void* arg);


rtems_device_driver console_open(rtems_device_major_number major,
                                 rtems_device_minor_number minor,
                                 void* arg){

    struct rtems_termios_callbacks* pcallbacks;

    stm32f_uart_driver_entry* pSelectedUart = &stm32f_uart_driver_table[minor];

    if((pSelectedUart->uartType == STM32F_UART_TYPE_INT) ||
       (pSelectedUart->uartType == STM32F_UART_TYPE_DMA)){
        pcallbacks = &stm32f_uart_callbacks;
    } else if(pSelectedUart->uartType == STM32F_UART_TYPE_POLLING){
        pcallbacks = &stm32f_uart_polling_callbacks;
    }

    return rtems_termios_open(major, minor, arg, pcallbacks);
}


rtems_device_driver console_close(rtems_device_major_number major,
                                  rtems_device_minor_number minor,
                                  void* arg){

    return rtems_termios_close(arg);
}


rtems_device_driver console_read(rtems_device_major_number major,
                                  rtems_device_minor_number minor,
                                  void* arg){

    return rtems_termios_read(arg);
}


rtems_device_driver console_write(rtems_device_major_number major,
                                  rtems_device_minor_number minor,
                                  void* arg){

    return rtems_termios_write(arg);
}


rtems_device_driver console_control(rtems_device_major_number major,
                                    rtems_device_minor_number minor,
                                    void* arg){

    return rtems_termios_ioctl(arg);
}
rtc_open
*/
