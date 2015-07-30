#include <can.h>
#include <dev/can/can.h>
#include <rtems.h>
#include <stm32f4xx_hal_can.h>
#include <stm32f4xx_hal_rcc.h>
#include <stm32f4xx_hal_gpio.h>
#include <math.h>
#include <rtems/irq-extension.h>

void stm32_can_gpio_init(CAN_Instance canInstance);
CAN_Timing_Values rtems_can_get_timing_values(uint32_t desired_baud);
CAN_Status stm32_can_transmit(CAN_HandleTypeDef* hCanHandle, can_msg * msg);


typedef struct stm32_can_bus stm32_can_bus;
typedef struct HandleWrapper HandleWrapper;

struct HandleWrapper{
  CAN_HandleTypeDef handle;
  stm32_can_bus * bus;
};

 struct stm32_can_bus{
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



CAN_Status stm32_can_transmit(CAN_HandleTypeDef* hCanHandle, can_msg * msg)
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

CAN_Status stm32_start_rx(CAN_HandleTypeDef *hCanHandle)
{
  CAN_Status Status;
  //TODO use real fifo numbers
  Status = HAL_CAN_Receive_IT(hCanHandle, 0);

  return Status;
}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef *hCanHandle) {
  // Wakeup RX Task
  rtems_status_code sc;
  stm32_can_bus * bus = (((HandleWrapper *) hCanHandle)->bus);
  volatile int a;
  sc = rtems_event_transient_send(bus->base.rx_task_id);
  a++;

}


rtems_task stm32_rx_task(rtems_task_argument arg) {
  rtems_status_code sc;
  can_msg msg;
  size_t len;

  stm32_can_bus * bus = (stm32_can_bus *) arg;
  CAN_HandleTypeDef * hCanHandle = (CAN_HandleTypeDef * ) &bus->wrapper;

  while (1) {
    // Schedule a read
    stm32_start_rx(hCanHandle);



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


rtems_task stm32_tx_task(rtems_task_argument arg) {
  rtems_status_code sc;
  can_msg msg;
  size_t len;

  stm32_can_bus * bus = (stm32_can_bus *) arg;
  CAN_HandleTypeDef * hCanHandle = &(bus->wrapper.handle);
  while (1) {
    //TODO abstract away into
    // can_msg_queue_receive
    sc = rtems_message_queue_receive(bus->base.tx_msg_queue, 
        &msg,
        &len,
        RTEMS_WAIT,
        RTEMS_NO_TIMEOUT);
    _Assert(len == sizeof(msg));

    stm32_can_transmit(hCanHandle, &msg);


    // Is it okay to send?
    // Schedule a TX
  }
}


void stm32_can_gpio_init(CAN_Instance canInstance)
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


CAN_Timing_Values rtems_can_get_timing_values(
    uint32_t desired_baud
){

  //TODO:: Determine best value.

  CAN_Timing_Values s_timeValues;

  double test_baud;
  double tpclk = 1.0/58000000;

  int x;
  int p;
  double error = 1000000000;
  double best_error = 1000000000;
  double best_x;
  double best_p;
  double clock_frequency = HAL_RCC_GetPCLK1Freq();
  tpclk = 1.0 / clock_frequency;

  /* Perform Computation */
  // x = S1 + S2
  for (x = 4; x < 28; x++)
  {
    for (p = 1; p < 1025; p++)
    {
      test_baud = 1.0/(tpclk * (x + 3) * (p+1));

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
  // x = S1 + S2 + 3
  int s1_plus_s2 = best_x;
  int s1 = .75 * s1_plus_s2;
  int s2 = s1_plus_s2 - s1;
  s_timeValues.s1 = 6;//s1;
  s_timeValues.s2 = 1;//  s2;

  s_timeValues.prescaler = 10; //best_p; //10; // best_p;
  s_timeValues.error = best_error;
  return s_timeValues;
}

void stm32_can_isr(void * arg) {

  HAL_CAN_IRQHandler((CAN_HandleTypeDef *) arg);
}

void stm32_can_rx0_isr(void * arg) {

  HAL_CAN_IRQHandler((CAN_HandleTypeDef *) arg);
}

void stm32_can_rx1_isr(void * arg) {

  HAL_CAN_IRQHandler((CAN_HandleTypeDef *) arg);
}

int stm32_can_init(
    can_bus * self,
    long baudRate
){

  stm32_can_bus * bus = (stm32_can_bus * ) self;
  CAN_HandleTypeDef * hCanHandle = &(bus->wrapper);
  bus->wrapper.bus = bus;
  CAN_Instance canInstance = bus->instance;
  //CAN_HandleTypeDef hCanHandle = *hnCanHandle;

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
  hCanHandle->Init.Mode = CAN_MODE_LOOPBACK;

  s_timeValues = rtems_can_get_timing_values(baudRate);

  const uint32_t CAN_BS1_OFFSET = 16;
  const uint32_t CAN_BS2_OFFSET = 20;
  const uint32_t BS1_MAX = 16;
  const uint32_t BS2_MAX = 8;

  if (   s_timeValues.s1 > BS1_MAX 
      || s_timeValues.s1 < 1) {
    return CAN_ERROR;
  }

  if (   s_timeValues.s2 > BS2_MAX 
      || s_timeValues.s2 < 1) {
    return CAN_ERROR;
  }

  hCanHandle->Init.BS1 = (s_timeValues.s1 - 1) << CAN_BS1_OFFSET;
  hCanHandle->Init.BS2 = (s_timeValues.s2 - 1) << CAN_BS2_OFFSET;

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

    }

  rtems_status_code sc;
  sc = rtems_interrupt_handler_install(19, "Info", RTEMS_INTERRUPT_UNIQUE, stm32_can_isr, hCanHandle);
  sc = rtems_interrupt_handler_install(27, "Info", RTEMS_INTERRUPT_UNIQUE, stm32_can_rx0_isr, hCanHandle);
  sc = rtems_interrupt_handler_install(28, "Info", RTEMS_INTERRUPT_UNIQUE, stm32_can_rx1_isr, hCanHandle);
  return CAN_OK;
}


int stm32_can_de_init(
    can_bus * self
){
  stm32_can_bus * bus = (stm32_can_bus * ) self;
  CAN_HandleTypeDef * hCanHandle = &(bus->wrapper.handle);
  HAL_CAN_DeInit(hCanHandle);
  // TODO error codes
  return 0;
}

stm32_can_bus * bus1;
stm32_can_bus * bus2;


int stm32_init_can(
    void
){
  int error;

  bus1 = (stm32_can_bus *) can_bus_alloc_and_init(sizeof(*bus1));
  bus2 = (stm32_can_bus *) can_bus_alloc_and_init(sizeof(*bus2));

  //
  bus1->base.init     = stm32_can_init;
  bus1->base.de_init  = stm32_can_de_init;
  bus1->base.tx_task = stm32_tx_task;
  bus1->base.rx_task = stm32_rx_task;

  bus1->instance = CAN_ONE;

  bus2->base.init     = stm32_can_init;
  bus2->base.de_init  = stm32_can_de_init;
  bus2->base.tx_task = stm32_tx_task;
  bus2->instance = CAN_TWO;

  if (bus1 == NULL) {
    return -ENOMEM;
  }

  if (bus2 == NULL) {
    return -ENOMEM;
  }

  error = can_bus_register(&bus1->base,
      "/dev/can1");

  //error = can_bus_register(&bus2->base, "/dev/can2");
  return error;
}
