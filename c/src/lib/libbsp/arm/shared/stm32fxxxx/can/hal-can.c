/**
 * @file hal-can.c
 *
 * @ingroup can
 *
 * @brief CAN driver for the STM32xxxx series processors. Provides a 
 * CAN driver that registers with cpukit/dev/can as a node in the 
 * filesystem.
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
#include <hal-can.h>
#include <dev/can/can-internal.h>
#include <rtems.h>
#include <hal-utils.h>
#include <stm32f-processor-specific.h>

#include stm_processor_header(TARGET_STM_PROCESSOR_PREFIX)
#include stm_header(TARGET_STM_PROCESSOR_PREFIX, can)
#include stm_header(TARGET_STM_PROCESSOR_PREFIX, rcc)
#include stm_header(TARGET_STM_PROCESSOR_PREFIX, gpio)

#include <math.h>
#include <rtems/irq-extension.h>

// Internal functions to perform RX and TX
static int                stm32_can_init              (can_bus * self, long baudRate);
static int                stm32_can_de_init           (can_bus * self);
static void               stm32_can_gpio_init         (CAN_Instance canInstance);
static CAN_Timing_Values  rtems_can_get_timing_values (uint32_t desired_baud);
static CAN_Status         stm32_can_start_tx          (CAN_HandleTypeDef* hCanHandle, can_msg * msg);
static CAN_Status         stm32_can_start_rx          (CAN_HandleTypeDef *hCanHandle);
// Tasks
static rtems_task         stm32_rx_task               (rtems_task_argument arg);
static rtems_task         stm32_tx_task               (rtems_task_argument arg);
// ISRs
static void               stm32_can_isr               (void * arg);
static void               stm32_can_rx0_isr           (void * arg);
static void               stm32_can_rx1_isr           (void * arg);

typedef struct stm32_can_bus stm32_can_bus;
typedef struct HandleWrapper HandleWrapper;

struct HandleWrapper {
  CAN_HandleTypeDef handle;
  stm32_can_bus * bus;
};

struct stm32_can_bus {
  can_bus base;
  HandleWrapper wrapper;
  CAN_Instance instance;
  CanTxMsgTypeDef TxMessage;
  CanRxMsgTypeDef RxMessage;
};

/* Definition for CAN1 Pins */
//TODO Make this configurable and work for F7
#define CAN1_TX_PIN                    GPIO_PIN_9
#define CAN1_RX_PIN                    GPIO_PIN_8
#define CAN1_GPIO_PORT                 GPIOB
#define CAN1_AF                        GPIO_AF9_CAN1


/* Definition for CAN2 Pins */
#define CAN2_TX_PIN                    GPIO_PIN_6
#define CAN2_RX_PIN                    GPIO_PIN_5
#define CAN2_GPIO_PORT                 GPIOB
#define CAN2_AF                        GPIO_AF9_CAN2


CAN_Status stm32_can_start_tx
(
  CAN_HandleTypeDef* hCanHandle, 
  can_msg * msg
)
{
  //TODO refactor this to not have so much STM32 stuff
  CAN_Status Status;

  hCanHandle->pTxMsg->RTR     = CAN_RTR_DATA;
  hCanHandle->pTxMsg->IDE     = CAN_ID_STD;

  hCanHandle->pTxMsg->StdId   = msg->id;
  hCanHandle->pTxMsg->DLC     = msg->len;
  memcpy(hCanHandle->pTxMsg->Data, msg->data, 8);

  Status = HAL_CAN_Transmit_IT(hCanHandle);
  return Status;
}


CAN_Status stm32_can_start_rx
(
  CAN_HandleTypeDef *hCanHandle
)
{
  //TODO use both FIFOs
  CAN_Status Status;
  Status = HAL_CAN_Receive_IT(hCanHandle, 0);
  return Status;
}


void HAL_CAN_RxCpltCallback
(
  CAN_HandleTypeDef *hCanHandle
)
{
  // Wakeup RX Task
  rtems_status_code sc;
  stm32_can_bus * bus = (((HandleWrapper *) hCanHandle)->bus);
  volatile int a;
  sc = rtems_event_transient_send(bus->base.rx_task_id);
  a++;
}


rtems_task stm32_rx_task
(
  rtems_task_argument arg
)
{
  rtems_status_code sc;
  can_msg msg;
  size_t len;

  stm32_can_bus * bus = (stm32_can_bus *) arg;
  CAN_HandleTypeDef * hCanHandle = (CAN_HandleTypeDef * ) &bus->wrapper;

  while (1) {
    // Schedule a read
    stm32_can_start_rx(hCanHandle);



    // Wait for interrupt
    sc = rtems_event_transient_receive(RTEMS_WAIT, RTEMS_NO_TIMEOUT);
    _Assert(sc == RTEMS_SUCCESSFUL);

    //TODO consider 29 bit ids
    msg.id = hCanHandle->pRxMsg->StdId;
    msg.len = hCanHandle->pRxMsg->DLC;
    memcpy(&msg.data, &hCanHandle->pRxMsg->Data, msg.len);
    // Pass message along to stack
    rtems_message_queue_send(bus->base.rx_msg_queue, &msg, sizeof(msg));
  }
}


rtems_task stm32_tx_task
(
  rtems_task_argument arg
)
{
  rtems_status_code sc;
  can_msg msg;
  size_t len;

  stm32_can_bus * bus = (stm32_can_bus *) arg;
  CAN_HandleTypeDef * hCanHandle = &(bus->wrapper.handle);
  while (1) {
    //TODO abstract away into
    // can_msg_queue_receive
    sc = rtems_message_queue_receive( 
        bus->base.tx_msg_queue, 
        &msg,
        &len,
        RTEMS_WAIT,
        RTEMS_NO_TIMEOUT
      );
    _Assert(len == sizeof(msg));

    stm32_can_start_tx(hCanHandle, &msg);
  }
}


void stm32_can_gpio_init
(
  CAN_Instance canInstance
)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  // Enable the clock CAN Core //
  __HAL_RCC_CAN1_CLK_ENABLE();

  if (canInstance == CAN_TWO)
  {
    __HAL_RCC_CAN2_CLK_ENABLE();	// CAN2 requires clock for CAN1 enabled too
  }

  // CAN GPIO Clock Enable//
  __GPIOB_CLK_ENABLE();

  /* CAN2 TX RX GPIO pin configuration */

  GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
  GPIO_InitStructure.Pull = GPIO_PULLDOWN;

  if (canInstance == CAN_ONE)
  {
    GPIO_InitStructure.Pin = CAN1_TX_PIN | CAN1_RX_PIN;
    GPIO_InitStructure.Alternate =  CAN1_AF;
    HAL_GPIO_Init(CAN1_GPIO_PORT, &GPIO_InitStructure);
  }

  else if (canInstance == CAN_TWO)
  {
    GPIO_InitStructure.Pin = CAN2_TX_PIN | CAN2_RX_PIN;
    GPIO_InitStructure.Alternate =  CAN2_AF;
    HAL_GPIO_Init(CAN2_GPIO_PORT, &GPIO_InitStructure);
  }
}

/**
 * Calculate the baud timing values for the 
 * STM32 BxCAN peripheral
 *
 * @param desired_baud desired buad rate
 *
 * @return CAN_Timing_Values.s1         - TQ in first bit segment
 *                          .s2         - TQ in second bit segment
 *                          .prescaler  - Preescaler
 *                          .error      - Difference from requested baud
 */
CAN_Timing_Values rtems_can_get_timing_values
(
  uint32_t desired_baud
)
{

  //TODO:: Determine best value.
  //
  // 1 BS = 1 Time Quata, or TQ.
  //
  // A TQ is the smallest unit of time that can be used to 
  // create a CAN timing profile. It is Preescaler * (1 / Peripheral Clock)
  //
  // A CAN bit is divided up as follows:
  //
  //   +---+-------------+-------+
  //   |SS |     BS1     | BS2   |
  //   +---+-------------+-------+
  //    ^  ^             ^       ^
  //    |  |             |       |
  //    |  \ Start Bit   |       \ End of Bit
  //    \ Synq Seg      \ Sampling Point
  //
  //  There are also a Sync Segment that is 1 TQ
  //
  //
  //  Therefore the baud rate is the 1/(1 TQ + BS1 + BS2)
  //
  //  Keep in mind that in calculations, we are storing BS1
  //  and BS2 as multiples of 1TQ, a unitless quantity.
  //
  //  Keeping this in mind, the actual equation is
  //
  //  baud = 1 / (TQ * (1 + BS1 + BS2))
  //

  CAN_Timing_Values s_timeValues;

  // We want to calculate the best
  // possible values for BS1 BS2 and 
  // the Prescalar. We are going to do 
  // 2D optimization using a bruteforce
  // algorithm, simply trying combinations
  // and comparing them against our previous 
  // best

  float test_baud;

  // Rather than optimizing for three variables,
  // we are combining x = BS1 + BS2.
  //
  // Now our optimization is the best combination 
  // of X and the Prescaler.
  //
  // We can calculate BS1 and BS2 later to 
  // pick a good sampling point
  int x;
  int p;

  float error = 1000000000;

  // Store a very large error term
  float best_error = 1000000000;
  float best_x;
  float best_p;

  float clock_frequency = HAL_RCC_GetPCLK1Freq();
  float tq = 1.0 / clock_frequency;

  // x = S1 + S2
  for (x = 9; x < 28; x++)
  {
    for (p = 0; p < 1025; p++)
    {
      test_baud = 1.0/ (tq * (x + 1) * (p) );

      error = fabs(desired_baud- test_baud);
      if (error < best_error) 
      {
        best_error = error;
        best_x = x;
        best_p = p;
      }
    }
  }

  // Calculate S1 and S2
  // x = S1 + S2
  int s1_plus_s2 = best_x;
  int s1 = roundf(.875 * s1_plus_s2);
  int s2 = s1_plus_s2 - s1;
  s_timeValues.s1 = s1;
  s_timeValues.s2 = s2;

  s_timeValues.prescaler = best_p;
  s_timeValues.error = best_error;
  return s_timeValues;
}

void stm32_can_isr
(
  void * arg
)
{
  HAL_CAN_IRQHandler((CAN_HandleTypeDef *) arg);
}

void stm32_can_rx0_isr
(
 void * arg
)
{
  HAL_CAN_IRQHandler((CAN_HandleTypeDef *) arg);
}

void stm32_can_rx1_isr 
( 
  void * arg
)
{
  HAL_CAN_IRQHandler((CAN_HandleTypeDef *) arg);
}

int stm32_can_set_filter
(
  can_bus * self,
  can_filter * filter
)
{
  stm32_can_bus * bus = (stm32_can_bus * ) self;
  CAN_HandleTypeDef * hCanHandle = (CAN_HandleTypeDef *) &(bus->wrapper);


  CAN_FilterConfTypeDef sFilterConfig;
  sFilterConfig.FilterNumber = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = 0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.BankNumber = 14;
  
}

int stm32_can_init
(
    can_bus * self,
    long baudRate
)
{

  stm32_can_bus * bus = (stm32_can_bus * ) self;
  CAN_HandleTypeDef * hCanHandle = (CAN_HandleTypeDef *) &(bus->wrapper);
  bus->wrapper.bus = bus;
  CAN_Instance canInstance = bus->instance;

  CAN_Timing_Values s_timeValues;

  /*Initialize the clock and GPIO pins for the CAN driver*/
  stm32_can_gpio_init(canInstance);

  /*Configure the CAN peripherals*/

  if (CAN_ONE == canInstance)
    hCanHandle->Instance = CAN1;
  else
    hCanHandle->Instance = CAN2;

  hCanHandle->pTxMsg = &bus->TxMessage;
  hCanHandle->pRxMsg = &bus->RxMessage;

  hCanHandle->Init.TTCM = DISABLE;
  hCanHandle->Init.ABOM = DISABLE;
  hCanHandle->Init.AWUM = DISABLE;
  hCanHandle->Init.NART = DISABLE;
  hCanHandle->Init.RFLM = DISABLE;
  hCanHandle->Init.TXFP = DISABLE;
  hCanHandle->Init.SJW = CAN_SJW_1TQ;
  hCanHandle->Init.Mode = CAN_MODE_NORMAL;

  s_timeValues = rtems_can_get_timing_values(baudRate);

  const uint32_t CAN_BS1_OFFSET = 16;
  const uint32_t CAN_BS2_OFFSET = 20;
  const uint32_t BS1_MAX = 16;
  const uint32_t BS2_MAX = 8;

  if (   s_timeValues.s1 > BS1_MAX 
      || s_timeValues.s1 < 1) 
  {
    return CAN_ERROR;
  }

  if (   s_timeValues.s2 > BS2_MAX 
      || s_timeValues.s2 < 1) 
  {
    return CAN_ERROR;
  }
    
  // The STM32 HAL Code expects Init->BS1 and Init->BS2
  // to contain the appropriate value so they can simply be 
  // OR'd into CAN_BTR. 
  //
  // We subtract 1, since 1 TQ is represented by zero, but 
  // our timing function returns the actual number of time 
  // quanta for S1 and S2
  hCanHandle->Init.BS1 = (s_timeValues.s1 - 1) << CAN_BS1_OFFSET;
  hCanHandle->Init.BS2 = (s_timeValues.s2 - 1) << CAN_BS2_OFFSET;

  // The Preescaler however, is expected to be passed in
  // as is, so we don't need to subtract 1 here.
  //
  // See stm32f4xx_hal_can.c, line 309
  hCanHandle->Init.Prescaler = s_timeValues.prescaler; //s_timeValues.prescaler; //s_timeValues.prescaler;

  if (HAL_CAN_Init(hCanHandle) != HAL_OK)
  {
    /* Initialization Error */
    //TODO::
    return CAN_ERROR;
  }
  CAN_FilterConfTypeDef sFilterConfig;
  sFilterConfig.FilterNumber = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = 0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.BankNumber = 14;

  if(HAL_CAN_ConfigFilter(&hCanHandle, &sFilterConfig) != HAL_OK) {
    //TODO fill in this body
  }

  rtems_status_code sc;

  if (CAN_ONE == canInstance)
  {
    sc = rtems_interrupt_handler_install(
        CAN1_TX_IRQn,
        "CAN1 TX Interrupt",
        RTEMS_INTERRUPT_UNIQUE,
        stm32_can_isr,
        hCanHandle);
    sc = rtems_interrupt_handler_install(
        CAN1_RX0_IRQn,
        "CAN1 RX Interrupt 0",
        RTEMS_INTERRUPT_UNIQUE,
        stm32_can_rx0_isr,
        hCanHandle);
    sc = rtems_interrupt_handler_install(
        CAN1_RX1_IRQn,
        "CAN1 RX Interrupt 1",
        RTEMS_INTERRUPT_UNIQUE,
        stm32_can_rx1_isr,
        hCanHandle);
  } else 
  {
    sc = rtems_interrupt_handler_install(
        CAN2_TX_IRQn,
        "CAN2 TX Interrupt",
        RTEMS_INTERRUPT_UNIQUE,
        stm32_can_isr,
        hCanHandle);
    sc = rtems_interrupt_handler_install(
        CAN2_RX0_IRQn,
        "CAN2 RX Interrupt 0",
        RTEMS_INTERRUPT_UNIQUE,
        stm32_can_rx0_isr,
        hCanHandle);
    sc = rtems_interrupt_handler_install(
        CAN2_RX1_IRQn,
        "CAN2 RX Interrupt 1",
        RTEMS_INTERRUPT_UNIQUE,
        stm32_can_rx1_isr,
        hCanHandle);
  }
  return CAN_OK;
}


int stm32_can_de_init
(
    can_bus * self
)
{
  stm32_can_bus * bus = (stm32_can_bus * ) self;
  CAN_HandleTypeDef * hCanHandle = &(bus->wrapper.handle);
  rtems_status_code sc;

  if (CAN_ONE == hCanHandle->Instance)
  {
    sc = rtems_interrupt_handler_remove(
        CAN1_TX_IRQn, 
        stm32_can_isr,     
        hCanHandle
        );
    sc = rtems_interrupt_handler_remove(
        CAN1_RX0_IRQn, 
        stm32_can_rx0_isr, 
        hCanHandle
        );
    sc = rtems_interrupt_handler_remove(
        CAN1_RX1_IRQn, 
        stm32_can_rx1_isr,
        hCanHandle);
  } else 
  {
    sc = rtems_interrupt_handler_remove(
        CAN2_TX_IRQn, 
        stm32_can_isr,     
        hCanHandle
        );
    sc = rtems_interrupt_handler_remove(
        CAN2_RX0_IRQn, 
        stm32_can_rx0_isr, 
        hCanHandle
        );
    sc = rtems_interrupt_handler_remove(
        CAN2_RX1_IRQn, 
        stm32_can_rx1_isr,
        hCanHandle);
  }

  HAL_CAN_DeInit(hCanHandle);
  // TODO error codes
  return 0;
}

stm32_can_bus * bus1;
stm32_can_bus * bus2;


int stm32_bsp_register_can
(
  void
)
{
  int error;

  bus1 = (stm32_can_bus *) can_bus_alloc_and_init(sizeof(*bus1));
  bus2 = (stm32_can_bus *) can_bus_alloc_and_init(sizeof(*bus2));

  //
  bus1->base.init     = stm32_can_init;
  bus1->base.de_init  = stm32_can_de_init;
  bus1->base.tx_task  = stm32_tx_task;
  bus1->base.rx_task  = stm32_rx_task;
  bus1->base.set_filter = stm32_can_set_filter;
  bus1->instance      = CAN_ONE;

  bus2->base.init     = stm32_can_init;
  bus2->base.de_init  = stm32_can_de_init;
  bus2->base.tx_task  = stm32_tx_task;
  bus2->base.rx_task  = stm32_rx_task;
  bus2->base.set_filter = stm32_can_set_filter;
  bus2->instance      = CAN_TWO;



  if (bus1 == NULL) {
    return -ENOMEM;
  }

  if (bus2 == NULL) {
    return -ENOMEM;
  }

  error = can_bus_register(&bus1->base, "/dev/can1");

  error = can_bus_register(&bus2->base, "/dev/can2");
  return error;
}
