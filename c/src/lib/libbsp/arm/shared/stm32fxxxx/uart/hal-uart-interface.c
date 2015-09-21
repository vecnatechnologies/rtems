/**
 * @file hal-uart-interface.c
 *
 * @ingroup uart
 *
 * @brief A set of utility functions used in the STM32F UART drivers that provide
 *   an API to the underlying hardware abstraction functions.
 *
 */

/*
 * Copyright (c) 2015 Vecna Technologies, Inc.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.com/license/LICENSE.
 */

//================== STMF32 Support Functions =================================
#include <hal-uart-interface.h>
#include <hal-utils.h>
#include <stm32f-processor-specific.h>
#include <hal-error.h>
#include <bspopts.h>

#include stm_processor_header(TARGET_STM_PROCESSOR_PREFIX)
#include stm_header(TARGET_STM_PROCESSOR_PREFIX, rcc)
#include stm_header(TARGET_STM_PROCESSOR_PREFIX, rcc_ex)
#include stm_header(TARGET_STM_PROCESSOR_PREFIX, gpio)

// A set of UART handle used in the HAL code.  These handles are used
// for both the termios console uarts as well as the non-console uarts.
UART_HandleTypeDef UartHandles[NUM_PROCESSOR_CONSOLE_UARTS+NUM_PROCESSOR_NON_CONSOLE_UARTS];

typedef enum {
  UARTCallbackType_TX,
  UARTCallbackType_RX
} UARTCallbackType;


stm32f_base_uart_driver_entry* stm32f_get_base_uart_driver_entry_from_handle(
  const UART_HandleTypeDef *huart
)
{
  uint32_t i;

  for ( i = 0UL; i < COUNTOF(stm32f_console_driver_table); i++ ) {
    if ( stm32f_console_driver_table[i].base_driver_info.handle == huart ) {
      return (stm32f_base_uart_driver_entry*) &(stm32f_console_driver_table[i].base_driver_info);
    }
  }

  for ( i = 0UL; i < COUNTOF(stm32f_uart_driver_table); i++ ) {
    if ( stm32f_uart_driver_table[i].base_driver_info.handle == huart ) {
      return (stm32f_base_uart_driver_entry*) &(stm32f_uart_driver_table[i].base_driver_info);
    }
  }

  return NULL;
}

stm32f_console_driver_entry* stm32f_get_console_driver_entry_from_handle(
  const UART_HandleTypeDef *huart
)
{
  uint32_t i;

  for ( i = 0UL; i < COUNTOF(stm32f_console_driver_table); i++ ) {
    if ( stm32f_console_driver_table[i].base_driver_info.handle == huart ) {
      return (stm32f_console_driver_entry*) &(stm32f_console_driver_table[i]);
    }
  }

  return NULL;
}


stm32_uart_driver_entry* stm32f_get_uart_driver_entry_from_handle(
  const UART_HandleTypeDef *huart
)
{
  uint32_t i;

  for ( i = 0UL; i < COUNTOF(stm32f_uart_driver_table); i++ ) {
    if ( stm32f_uart_driver_table[i].base_driver_info.handle == huart ) {
      return (stm32_uart_driver_entry*) &(stm32f_uart_driver_table[i]);
    }
  }

  return NULL;
}


static void stm32f_init_uart_clock(
  const stm32f_uart Uart
)
{
  switch ( Uart )
  {
#if defined(STM32_ENABLE_USART_1)
  case STM32F_UART1:
    __HAL_RCC_USART1_CLK_ENABLE();
    break;
#endif

#if defined(STM32_ENABLE_USART_2)
  case STM32F_UART2:
    __HAL_RCC_USART2_CLK_ENABLE();
    break;
#endif

#if defined(STM32_ENABLE_USART_3)
  case STM32F_UART3:
    __HAL_RCC_USART3_CLK_ENABLE();
    break;
#endif

#if defined(STM32_ENABLE_USART_4)
  case STM32F_UART4:
    __HAL_RCC_UART4_CLK_ENABLE();
    break;
#endif

#if defined(STM32_ENABLE_USART_5)
  case STM32F_UART5:
    __HAL_RCC_UART5_CLK_ENABLE();
    break;
#endif

#if defined(STM32_ENABLE_USART_6)
  case STM32F_UART6:
    __HAL_RCC_USART6_CLK_ENABLE();
    break;
#endif

#if defined(STM32_ENABLE_USART_7)
    case STM32F_UART7:
    __HAL_RCC_UART7_CLK_ENABLE();
    break;
#endif

#if defined(STM32_ENABLE_USART_8)
    case STM32F_UART8:
    __HAL_RCC_UART8_CLK_ENABLE();
    break;
#endif
  default:
  case STM32F_INVALID_UART:
    stm32f_error_handler();
    break;
  }
}

static void stmf32_uart_reset(
  const stm32f_uart Uart
)
{
  switch ( Uart )
  {
#if defined(STM32_ENABLE_USART_1)
  case STM32F_UART1:
    __HAL_RCC_USART1_FORCE_RESET();
    __HAL_RCC_USART1_RELEASE_RESET();
    break;
#endif

#if defined(STM32_ENABLE_USART_2)
  case STM32F_UART2:
    __HAL_RCC_USART2_FORCE_RESET();
    __HAL_RCC_USART2_RELEASE_RESET();
    break;
#endif

#if defined(STM32_ENABLE_USART_3)
  case STM32F_UART3:
    __HAL_RCC_USART3_FORCE_RESET();
    __HAL_RCC_USART3_RELEASE_RESET();
    break;
#endif

#if defined(STM32_ENABLE_USART_4)
  case STM32F_UART4:
    __HAL_RCC_UART4_FORCE_RESET();
    __HAL_RCC_UART4_RELEASE_RESET();
    break;
#endif

#if defined(STM32_ENABLE_USART_5)
  case STM32F_UART5:
    __HAL_RCC_UART5_FORCE_RESET();
    __HAL_RCC_UART5_RELEASE_RESET();
    break;
#endif

#if defined(STM32_ENABLE_USART_6)
  case STM32F_UART6:
    __HAL_RCC_USART6_FORCE_RESET();
    __HAL_RCC_USART6_RELEASE_RESET();
    break;
#endif

#if defined(STM32_ENABLE_USART_7)
    case STM32F_UART7:
    __HAL_RCC_UART7_FORCE_RESET();
    __HAL_RCC_UART7_RELEASE_RESET();
    break;
#endif

#if defined(STM32_ENABLE_USART_8)
    case STM32F_UART8:
    __HAL_RCC_UART8_FORCE_RESET();
    __HAL_RCC_UART8_RELEASE_RESET();
    break;
#endif

  default:
  case STM32F_INVALID_UART:
    stm32f_error_handler();
    break;
  }
}

static void stmf32_init_gpio_clock(
  const stm32f_gpio_port port
)
{
  switch ( port ) {

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
    stm32f_error_handler();
    break;
  }
}

static GPIO_TypeDef* stmf32_get_gpio(
  const stm32f_gpio_port port
)
{
  GPIO_TypeDef * ret = NULL;

  switch ( port ) {

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
    stm32f_error_handler();
    break;
  }

  return ret;
}

USART_TypeDef* stmf32_uart_get_registers(
  const stm32f_uart Uart
)
{
  USART_TypeDef* ret = NULL;

  switch ( Uart )
  {
#if defined(STM32_ENABLE_USART_1)
  case STM32F_UART1:
    ret = USART1;
    break;
#endif

#if defined(STM32_ENABLE_USART_2)
  case STM32F_UART2:
    ret = USART2;
    break;
#endif

#if defined(STM32_ENABLE_USART_3)
  case STM32F_UART3:
    ret = USART3;
    break;
#endif

#if defined(STM32_ENABLE_USART_4)
  case STM32F_UART4:
    ret = UART4;
    break;
#endif

#if defined(STM32_ENABLE_USART_5)
  case STM32F_UART5:
    ret = UART5;
    break;
#endif

#if defined(STM32_ENABLE_USART_6)
  case STM32F_UART6:
    ret = USART6;
    break;
#endif

#if defined(STM32_ENABLE_USART_7)
    case STM32F_UART7:
    ret = UART7;
    break;
#endif

#if defined(STM32_ENABLE_USART_8)
    case STM32F_UART8:
    ret = UART8;
    break;
#endif

  default:
    case STM32F_INVALID_UART:
    stm32f_error_handler();
    break;
  }

  return ret;
}

static void stmf32_init_dma_clock(
  const stm32f_dma_controller controller
)
{
  if ( controller == STM32F_DMA1_CONTROLLER ) {
    __HAL_RCC_DMA1_CLK_ENABLE();
  } else if ( controller == STM32F_DMA2_CONTROLLER ) {
    __HAL_RCC_DMA2_CLK_ENABLE();
  }
}


static void stm32f_uart_dma_rx_isr(
  void* argData
)
{
  stm32f_base_uart_driver_entry* pUART =
    (stm32f_base_uart_driver_entry*) argData;

  HAL_DMA_IRQHandler(pUART->handle->hdmarx);
}


static void stm32f_uart_dma_tx_isr(
  void* argData
)
{
  stm32f_base_uart_driver_entry* pUART =
    (stm32f_base_uart_driver_entry*) argData;

  HAL_DMA_IRQHandler(pUART->handle->hdmatx);
}

static void stm32f_uart_isr_termios(
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
  if ( u32_StartRxCount > pUART->base_driver_info.handle->RxXferCount ) {

    int rxCount = u32_StartRxCount
      - pUART->base_driver_info.handle->RxXferCount;
    char* pStartRx = (char*) pUART->base_driver_info.handle->pRxBuffPtr;
    pStartRx -= rxCount;
    rtems_termios_enqueue_raw_characters(pUART->tty, pStartRx, rxCount);

    // re-enable interrupt to receive additional characters
    HAL_UART_Receive_IT(pUART->base_driver_info.handle, &pUART->rx_char, 1);
  }
}

static void stm32f_uart_isr(
  void *arg
)
{
  stm32f_base_uart_driver_entry* pUART = (stm32f_base_uart_driver_entry*) arg;

  HAL_UART_IRQHandler(pUART->handle);
}

static rtems_status_code stm32f_get_uart_isr( UART_HandleTypeDef* huart, void (** isr_routine)(void*)){

  rtems_status_code ret = RTEMS_SUCCESSFUL;

  if(stm32f_get_uart_driver_entry_from_handle(huart) != NULL) {
    *isr_routine = &stm32f_uart_isr;
  }  else if(stm32f_get_console_driver_entry_from_handle(huart) != NULL) {
    *isr_routine = &stm32f_uart_isr_termios;
  } else {
    stm32f_error_handler_with_reason("UART handle is not a console or standard uart.");
    ret = RTEMS_UNSATISFIED;
  }

  return ret;
}


int stm32f_register_interrupt_handlers(
  UART_HandleTypeDef* huart
)
{
  rtems_status_code ret = RTEMS_SUCCESSFUL;

  stm32f_base_uart_driver_entry* pUart = stm32f_get_base_uart_driver_entry_from_handle(huart);

  // Register DMA interrupt handlers (if necessary).  If they have been
  // previously installed then update the status to successful.
  if ( pUart->uart_mode == STM32F_UART_MODE_DMA ) {
    if ( ret == RTEMS_SUCCESSFUL ) {
      ret = rtems_interrupt_handler_install(
        pUart->rx_dma.DMAStreamInterruptNumber,
        NULL,
        RTEMS_INTERRUPT_UNIQUE,
        stm32f_uart_dma_rx_isr,
        pUart
        );

      if ( ret == RTEMS_TOO_MANY ) {
        ret = RTEMS_SUCCESSFUL;
      }
    }

    if ( ret == RTEMS_SUCCESSFUL ) {
      ret = rtems_interrupt_handler_install(
        pUart->tx_dma.DMAStreamInterruptNumber,
        NULL,
        RTEMS_INTERRUPT_UNIQUE,
        stm32f_uart_dma_tx_isr,
        pUart
        );

      if ( ret == RTEMS_TOO_MANY ) {
        ret = RTEMS_SUCCESSFUL;
      }
    }
  }

  // Decide which flavor of isr to install based upon whether the uart is
  // termios console or normal uart.
  void (* uart_isr_to_install) (void*);

  if(stm32f_get_uart_isr(huart, &uart_isr_to_install) == RTEMS_SUCCESSFUL) {

    // Register UART interrupt handler for either DMA or Interrupt modes.  If it has been
    // previously installed then update the status to successful.
    if ( pUart->uart_mode != STM32F_UART_MODE_POLLING ) {

      if ( ret == RTEMS_SUCCESSFUL ) {
        ret = rtems_interrupt_handler_install(
          pUart->interrupt_number,
          NULL,
          RTEMS_INTERRUPT_UNIQUE,
          uart_isr_to_install,
          pUart
          );
      }

      if ( ret == RTEMS_TOO_MANY ) {
        ret = RTEMS_SUCCESSFUL;
      }
    }
  }

  return ret;
}


int stm32f_remove_interrupt_handlers(
  UART_HandleTypeDef* huart
)
{
  rtems_status_code ret = RTEMS_SUCCESSFUL;
  stm32f_base_uart_driver_entry* pUart = stm32f_get_base_uart_driver_entry_from_handle(huart);

  // Register DMA interrupt handlers (if necessary)
  if ( pUart->uart_mode == STM32F_UART_MODE_DMA ) {
    if ( ret == RTEMS_SUCCESSFUL ) {
      ret = rtems_interrupt_handler_remove(
        pUart->rx_dma.DMAStreamInterruptNumber,
        stm32f_uart_dma_rx_isr,
        pUart
        );
    }
    if ( ret == RTEMS_SUCCESSFUL ) {
      ret = rtems_interrupt_handler_remove(
        pUart->tx_dma.DMAStreamInterruptNumber,
        stm32f_uart_dma_tx_isr,
        pUart
        );
    }
  }

  // Decide which flavor of isr to remove based upon whether the uart is
  // termios console or normal uart.
  void (* uart_isr_to_remove) (void*);

  if(stm32f_get_uart_isr(huart, &uart_isr_to_remove) == RTEMS_SUCCESSFUL) {

    // Register UART interrupt handler for either DMA or Interrupt modes
    if ( pUart->uart_mode != STM32F_UART_MODE_POLLING ) {

      if ( ret == RTEMS_SUCCESSFUL ) {
        ret = rtems_interrupt_handler_remove(
          pUart->interrupt_number,
          uart_isr_to_remove,
          pUart
          );
      }
    }
  }

  return (ret == RTEMS_SUCCESSFUL);
}


//=================== HAL UART weak symbol override functions =====================
void HAL_UART_MspInit(
  UART_HandleTypeDef *huart
)
{
  static DMA_HandleTypeDef hdma_tx;
  static DMA_HandleTypeDef hdma_rx;

  GPIO_InitTypeDef GPIO_InitStruct;
  stm32f_base_uart_driver_entry* pUart;

  pUart = stm32f_get_base_uart_driver_entry_from_handle(huart);

  //##-1- Enable peripherals and GPIO Clocks #################################

  // Enable Uart clocks
  stm32f_init_uart_clock(pUart->uart);

  // Enable GPIO clocks
  stmf32_init_gpio_clock(pUart->tx_pin.port);
  stmf32_init_gpio_clock(pUart->rx_pin.port);

  // Enable DMA clocks
  if ( pUart->uart_mode == STM32F_UART_MODE_DMA ) {
    stmf32_init_dma_clock(pUart->tx_dma.controller);
    stmf32_init_dma_clock(pUart->rx_dma.controller);
  }

  //##-2- Configure peripheral GPIO ##########################################

  // UART TX GPIO pin configuration
  GPIO_InitStruct.Pin = (1 << pUart->tx_pin.pin);
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = pUart->alt_func_config;

  HAL_GPIO_Init(stmf32_get_gpio(pUart->tx_pin.port), &GPIO_InitStruct);

  // UART RX GPIO pin configuration
  GPIO_InitStruct.Pin = (1 << pUart->rx_pin.pin);
  GPIO_InitStruct.Alternate = pUart->alt_func_config;

  HAL_GPIO_Init(stmf32_get_gpio(pUart->rx_pin.port), &GPIO_InitStruct);

  //##-3- Configure the DMA streams ##########################################

  if ( pUart->uart_mode == STM32F_UART_MODE_DMA ) {

    /* Configure the DMA handler for Transmission process */
    hdma_tx.Instance = pUart->tx_dma_stream;
    hdma_tx.Init.Channel = pUart->tx_dma.channel;

    hdma_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_tx.Init.Mode = DMA_NORMAL;
    hdma_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    hdma_tx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_tx.Init.MemBurst = DMA_MBURST_INC4;
    hdma_tx.Init.PeriphBurst = DMA_PBURST_INC4;

    HAL_DMA_Init(&hdma_tx);

    /* Associate the initialized DMA handle to the the UART handle */
    __HAL_LINKDMA(huart, hdmatx, hdma_tx);

    /* Configure the DMA handler for Transmission process */
    hdma_rx.Instance = pUart->rx_dma_stream;
    hdma_rx.Init.Channel = pUart->rx_dma.channel;

    hdma_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_rx.Init.Mode = DMA_NORMAL;
    hdma_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    hdma_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_rx.Init.MemBurst = DMA_MBURST_INC4;
    hdma_rx.Init.PeriphBurst = DMA_PBURST_INC4;

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

  pUart = stm32f_get_base_uart_driver_entry_from_handle(huart);

  /*##-1- Reset peripherals ##################################################*/
  stmf32_uart_reset(pUart->uart);

  /*##-2- Disable peripherals and GPIO Clocks ################################*/
  /* Configure UART Tx as alternate function  */
  HAL_GPIO_DeInit(stmf32_get_gpio(pUart->tx_pin.port), (1 << pUart->tx_pin.pin));
  /* Configure UART Rx as alternate function  */
  HAL_GPIO_DeInit(stmf32_get_gpio(pUart->rx_pin.port), (1 << pUart->rx_pin.pin));

  /*##-3- Disable the DMA Streams ############################################*/
  if ( pUart->uart_mode == STM32F_UART_MODE_DMA ) {
    /* De-Initialize the DMA Stream associate to transmission process */
    HAL_DMA_DeInit(&hdma_tx);
    /* De-Initialize the DMA Stream associate to reception process */
    HAL_DMA_DeInit(&hdma_rx);
  }
}

static rtems_status_code stm32f_send_uart_event(UART_HandleTypeDef *huart, const UARTCallbackType type) {

  rtems_status_code sc = RTEMS_UNSATISFIED;

  stm32_uart_driver_entry* pUartDriver = stm32f_get_uart_driver_entry_from_handle (huart);

  if(pUartDriver != NULL) {

    switch(type){

    case UARTCallbackType_TX:
      // Double check driver state for TX done
      if((pUartDriver->base_driver_info.handle->State == HAL_UART_STATE_BUSY_RX) ||
         (pUartDriver->base_driver_info.handle->State == HAL_UART_STATE_READY)) {

        // Send event to TX task to indicate that transmit is complete.
        sc = rtems_event_send(pUartDriver->tx_task_id, UART_TX_DONE);
      }
      break;

    case UARTCallbackType_RX:
      // Double check driver state for RX done
      if((pUartDriver->base_driver_info.handle->State == HAL_UART_STATE_BUSY_TX) ||
         (pUartDriver->base_driver_info.handle->State == HAL_UART_STATE_READY)) {

        // Send event to TX task to indicate that transmit is complete.
        sc = rtems_event_send(pUartDriver->rx_task_id, UART_RX_DONE);
      }
      break;
    }
  }

  return sc;
}

void HAL_UART_TxCpltCallback(
  UART_HandleTypeDef *huart
)
{
  stm32f_console_driver_entry* pEntry =
    stm32f_get_console_driver_entry_from_handle(huart);

  // If there are still characters in TX fifo start sending again...
  if ( pEntry != NULL ) {

    if ( Ring_buffer_Is_empty(pEntry->fifo) == false ) {
      stm32f_uart_get_next_tx_buf(pEntry, NULL, 0);
    }
  } else {

    // If this is non-console uart then send event to TX task to
    // indicate that we are done transmitting.
    stm32f_send_uart_event(huart, UARTCallbackType_TX);
  }
}

/**
  * @brief Rx Transfer completed callbacks
  * @param huart: uart handle
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  stm32f_send_uart_event(huart, UARTCallbackType_RX);
}
