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
#include <can-internal.h>
#include <rtems.h>
#include "../include/atsam_can.h"
#include <mcan.h>
#include <libchip/chip.h>
#include <rtems/irq-extension.h>

static uint32_t     txdCntr = 0;
static uint32_t     rxdCntr = 0;
static uint32_t*    txMailbox0;
static uint32_t*    txMailbox1;
static Mailbox8Type rxMailbox0;
static Mailbox8Type rxMailbox1;
static Mailbox8Type rxMailbox2;

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/
#define PIN_MCAN1_TXD {PIO_PC14C_CANTX1, PIOC, ID_PIOC, PIO_PERIPH_C, PIO_DEFAULT}
#define PIN_MCAN1_RXD {PIO_PC12C_CANRX1, PIOC, ID_PIOC, PIO_PERIPH_C, PIO_DEFAULT}

static const Pin pinsMcan1[] =  {PIN_MCAN1_TXD, PIN_MCAN1_RXD };

const uint32_t CAN_STANDARD = 0;
const uint32_t CAN_FD = 1;
static const uint32_t ADD_MSG_FAILURE = 255;
static const uint32_t ATSAM_NUM_CAN_FILTERS = 128;

#define TX_BUFFER_0  0
#define TX_BUFFER_1  1
#define RX_BUFFER_0  0
#define RX_BUFFER_1  1
#define ACCEPT_ALL         0x000
#define FILTER_0           0
#define FILTER_1           1
#define MCAN_LINE1_IRQn    38

// Internal functions to perform RX and TX
static int atsam_can_init(
  can_bus *self,
  long     baudRate
);

int atsam_can_set_filter
(
  can_bus    *self,
  can_filter *filter
);

int atsam_can_set_flags
(
  can_bus *self,
  uint32_t flags
);

void MCAN_Handler(void * arg);

static int  atsam_can_de_init( can_bus *self );

static CAN_Status atsam_can_start_tx(
  MCan_ConfigType *hCanHandle,
  can_msg           *msg
);

static CAN_Status atsam_can_start_rx( MCan_ConfigType *hCanHandle );
// Tasks
static rtems_task atsam_rx_task( rtems_task_argument arg );
static rtems_task atsam_tx_task( rtems_task_argument arg );

typedef struct atsam_can_bus atsam_can_bus;
typedef struct HandleWrapper HandleWrapper;

// CAN bus metrics
static uint64_t can_tx_count[2] = {0};
static uint64_t can_rx_count[2] = {0};

struct HandleWrapper {
  MCan_ConfigType* handle;
  atsam_can_bus*   bus;
};

struct atsam_can_bus {
  can_bus base;
  HandleWrapper wrapper;
  CAN_Instance instance;
};

// Two static CAN bus instances to be
// registered with RTEMS
// They are declared here for easier reference
atsam_can_bus *bus1;
atsam_can_bus *bus2;

/* Definition for CAN1 Pins */
//TODO Make this configurable and work for F7
#define CAN1_TX_PIN ATSAM_CAN1_TX_PIN
#define CAN1_RX_PIN ATSAM_CAN1_RX_PIN
#define CAN1_GPIO_PORT ATSAM_CAN1_GPIO_PORT
#define CAN1_AF GPIO_AF9_CAN1

/* Definition for CAN2 Pins */
#define CAN2_TX_PIN ATSAM_CAN2_TX_PIN
#define CAN2_RX_PIN ATSAM_CAN2_RX_PIN
#define CAN2_GPIO_PORT ATSAM_CAN2_GPIO_PORT
#define CAN2_AF GPIO_AF9_CAN2


/**
 * \brief Enable MCAN interrupt.
 *
 * \param[in] module_inst  Pointer to the MCAN software instance struct
 * \param[in] source  Interrupt source type
 */
static inline void mcan_enable_interrupt(Mcan* module_inst,
    const enum mcan_interrupt_source source)
{
  module_inst->MCAN_IE |= source;
}

/**
 * \brief Disable MCAN interrupt.
 *
 * \param[in] module_inst  Pointer to the MCAN software instance struct
 * \param[in] source  Interrupt source type
 */
static inline void mcan_disable_interrupt(Mcan* module_inst,
    const enum mcan_interrupt_source source)
{
  module_inst->MCAN_IE &= ~source;
}

/**
 * \brief Get MCAN interrupt status.
 *
 * \param[in] module_inst  Pointer to the MCAN software instance struct
 */
static inline uint32_t mcan_read_interrupt_status(
  Mcan* module_inst)
{
  return module_inst->MCAN_IR;
}

/**
 * \brief Get MCAN interrupt status.
 *
 * \param[in] module_inst  Pointer to the MCAN software instance struct
 */
static inline uint32_t mcan_read_interrupt_enable(
  Mcan* module_inst)
{
  return module_inst->MCAN_IE;
}


/**
 * \brief Clear MCAN interrupt status.
 *
 * \param[in] module_inst  Pointer to the MCAN software instance struct
 * \param[in] source  Interrupt source type
 *
 * \return Bit mask of interrupt status value.
 */
static inline void mcan_clear_interrupt_status(
    Mcan* module_inst,
    const enum mcan_interrupt_source source)
{
  module_inst->MCAN_IR = source;
}

static MCan_DlcType atsam_get_dlc(const uint32_t length){

  MCan_DlcType ret;

  switch(length){

  case 0:
    ret = CAN_DLC_0;
    break;
  case 1:
    ret = CAN_DLC_1;
    break;
  case 2:
    ret = CAN_DLC_2;
    break;
  case 3:
    ret = CAN_DLC_3;
    break;
  case 4:
    ret = CAN_DLC_4;
    break;
  case 5:
    ret = CAN_DLC_5;
    break;
  case 6:
    ret = CAN_DLC_6;
    break;
  case 7:
    ret = CAN_DLC_7;
    break;
  case 8:
    ret = CAN_DLC_8;
    break;
  case 12:
    ret = CAN_DLC_12;
    break;
  case 16:
    ret = CAN_DLC_16;
    break;
  case 20:
    ret = CAN_DLC_20;
    break;
  case 24:
    ret = CAN_DLC_24;
    break;
  case 32:
    ret = CAN_DLC_32;
    break;
  case 48:
    ret = CAN_DLC_48;
    break;
  case 64:
    ret = CAN_DLC_64;
    break;
  default:
    ret = CAN_DLC_0;
    break;
  }

  return ret;
}


CAN_Status atsam_can_start_tx
(
  MCan_ConfigType*   hCanHandle,
  can_msg*           msg
)
{
  CAN_Status ret = CAN_OK;

  // get the correct enumeration for the specified length
  MCan_DlcType length = atsam_get_dlc(msg->len);

  // add message to TX FIFO
  if (MCAN_AddToTxFifoQ(hCanHandle, msg->id,
                        CAN_STD_ID,
                        length,
                        (uint8_t*) msg->data) == ADD_MSG_FAILURE){
    ret = CAN_ERROR;
  }

  return ret;
}


CAN_Status atsam_can_start_rx
  ( MCan_ConfigType *hCanHandle )
{

  return CAN_OK;
}

#if 0
HAL_CAN_TxCpltCallback(MCan_ConfigType* hCanHandle) {

  atsam_can_bus    *bus = ( ( (HandleWrapper *) hCanHandle )->bus );

  if(bus->instance < COUNTOF(can_tx_count)) {
    can_tx_count[bus->instance]++;
  }
}

void HAL_CAN_RxCpltCallback
  ( MCan_ConfigType *hCanHandle )
{
  // Wakeup RX Task
  rtems_status_code sc;
  atsam_can_bus    *bus = ( ( (HandleWrapper *) hCanHandle )->bus );

  if(bus->instance < COUNTOF(can_tx_count)) {
    can_rx_count[bus->instance]++;
  }

  sc = rtems_event_transient_send( bus->base.rx_task_id );
}
#endif

rtems_task atsam_rx_task
  ( rtems_task_argument arg )
{
  rtems_status_code sc = RTEMS_SUCCESSFUL;
  can_msg           msg;
  uint32_t          fifo_entries;

  atsam_can_bus   *bus = (atsam_can_bus *) arg;
  MCan_ConfigType *hCanHandle = bus->wrapper.handle;
  Mcan* mcan = bus->wrapper.handle->pMCan;

  // at least one filter must be installed to receive anything.
  // Initially install a filter to accept everything
  MCAN_ConfigRxClassicFilter(hCanHandle,
                             CAN_FIFO_0,
                             FILTER_1,
                             0x000,
                             CAN_STD_ID,
                             ACCEPT_ALL);

  while ( 1 ) {

    // Wait for interrupt to indicate that there are received
    // CAN packets to process
    sc = rtems_event_transient_receive( RTEMS_WAIT, RTEMS_NO_TIMEOUT );

    if(sc == RTEMS_SUCCESSFUL){

      // Process all queued RX message in RX FIFO
      do {
        fifo_entries = MCAN_GetRxFifoBuffer(&mcan1Config, CAN_FIFO_0,
            (Mailbox64Type *) &rxMailbox2);

        if (fifo_entries > 0){
          rxdCntr++;

          msg.id = rxMailbox2.info.id;
          msg.len = rxMailbox2.info.length;
          memcpy( &msg.data, &rxMailbox2.data, msg.len );

          // Pass message along to stack
          rtems_message_queue_send( bus->base.rx_msg_queue, &msg, sizeof( msg ) );
        }
      } while (fifo_entries > 1);

    }
  }
}


rtems_task atsam_tx_task
  ( rtems_task_argument arg )
{
  rtems_status_code sc;
  can_msg           msg;
  size_t            len;

  atsam_can_bus*   bus = (atsam_can_bus *) arg;
  MCan_ConfigType* hCanHandle = bus->wrapper.handle;

  while ( 1 ) {

    sc = rtems_message_queue_receive(
      bus->base.tx_msg_queue,
      &msg,
      &len,
      RTEMS_WAIT,
      RTEMS_NO_TIMEOUT
         );
    _Assert( len == sizeof( msg ) );

    if(sc == RTEMS_SUCCESSFUL){
      CAN_Status status =  atsam_can_start_tx( hCanHandle, &msg );

      while (status != CAN_OK) {
          rtems_task_wake_after(1);
          status =  atsam_can_start_tx( hCanHandle, &msg );
      }
    }
  }
}


int atsam_can_get_num_filters(
  can_bus *self
)
{
  return ATSAM_NUM_CAN_FILTERS;
}


int atsam_can_set_filter
(
  can_bus    *self,
  can_filter *filter
)
{
  atsam_can_bus* bus = (atsam_can_bus *) self;
  MCan_ConfigType* hCanHandle = bus->wrapper.handle;

  if(filter->number < ATSAM_NUM_CAN_FILTERS){

    MCAN_ConfigRxClassicFilter(hCanHandle,
                               CAN_FIFO_0,
                               filter->number,
                               filter->filter,
                               CAN_STD_ID,
                               filter->mask);
  } else {
    return 1;
  }

  return 0;
}


int atsam_can_set_flags
(
  can_bus *self,
  uint32_t flags
)
{

#if 0
  atsam_can_bus     *bus = (atsam_can_bus *) self;
  MCan_ConfigType *hCanHandle = bus->wrapper.handle;

  if ( flags & CAN_FLAG_LOOPBACK_MODE ) {
    hCanHandle->Instance->BTR |= CAN_BTR_LBKM;
  } else {
    hCanHandle->Instance->BTR &= ~CAN_BTR_LBKM;
  }
#endif

  return 0;
}


static void configure_can_clocking(void) {

  // Select source as main clk (12 MHz)
  PMC->PMC_PCK[5] &= ~(PMC_PCK_CSS_Msk);
  PMC->PMC_PCK[5] |= ((1 << PMC_PCK_CSS_Pos) & PMC_PCK_CSS_Msk);

  // Divide the source by 1 to get a 12 MHz source clock to CAN
  PMC->PMC_PCK[5] &= ~(PMC_PCK_PRES_Msk );
  PMC->PMC_PCK[5] |= ((0 << PMC_PCK_PRES_Pos ) & PMC_PCK_PRES_Msk);

  uint32_t can1_pid = MCAN1_IRQn;
  PMC_EnablePeripheral(ID_MCAN1);
  PMC->PMC_PCR |= (PMC_PCR_CMD);
  PMC->PMC_PCR |= (PMC_PCR_PID(can1_pid));

  Mcan       *mcan = MCAN1;

  mcan->MCAN_BTP &= ~(MCAN_BTP_BRP_Msk);
  mcan->MCAN_BTP |= MCAN_BTP_BRP(0);

  mcan->MCAN_BTP &= ~MCAN_BTP_TSEG1_Msk;
  mcan->MCAN_BTP &= ~MCAN_BTP_TSEG2_Msk;

  mcan->MCAN_BTP |= (5 << MCAN_BTP_TSEG1_Pos) & MCAN_BTP_TSEG1_Msk;
  mcan->MCAN_BTP |= (4  << MCAN_BTP_TSEG2_Pos) & MCAN_BTP_TSEG2_Msk;

  mcan->MCAN_BTP &= ~(MCAN_BTP_SJW_Msk);
  mcan->MCAN_BTP |= MCAN_BTP_SJW(2);

}


/**
 *  The MCAN only allows configuration if these bit are set in the CCCR
 *  register. They must be clear before the can bus is operational.
 */
static inline void mcan_start_initialization(Mcan* module_inst)
{
  volatile uint32_t* pCCR = (volatile uint32_t*) 0x40034018;

  module_inst->MCAN_CCCR |= MCAN_CCCR_INIT_ENABLED;
  *pCCR = MCAN_CCCR_INIT_ENABLED;
  rtems_task_wake_after(1);

  module_inst->MCAN_CCCR |= MCAN_CCCR_CCE_CONFIGURABLE;
  *pCCR = MCAN_CCCR_INIT_ENABLED | MCAN_CCCR_CCE_CONFIGURABLE;
  rtems_task_wake_after(1);
  printf("CCCR = 0x%.08X (0x%.08X)\n", module_inst->MCAN_CCCR, &module_inst->MCAN_CCCR);
}


static inline void mcan_end_initialization(Mcan* module_inst)
{
  module_inst->MCAN_CCCR &= ~(MCAN_CCCR_INIT_ENABLED | MCAN_CCCR_CCE_CONFIGURABLE);
}



/**
 * \brief Interrupt handler for MCAN,
 *   including RX,TX,ERROR and so on processes.
 */
void MCAN_Handler(void * arg){

  atsam_can_bus* bus = (atsam_can_bus* )arg;
  Mcan* mcan = bus->wrapper.handle->pMCan;
  static volatile uint32_t rx_interrupt_count = 0;
  static volatile uint32_t error_count = 0;

  volatile uint32_t status;
  status = mcan_read_interrupt_status(mcan);

  if(status & MCAN_RX_FIFO_0_NEW_MESSAGE){

    rtems_event_transient_send(bus->base.rx_task_id);

    // Clear interrupt
    mcan_clear_interrupt_status(mcan, MCAN_RX_FIFO_0_NEW_MESSAGE);
  }
}


int atsam_can_init
(
  can_bus *self,
  long     baudRate
)
{
  atsam_can_bus   *bus = (atsam_can_bus *) self;
  MCan_ConfigType *hCanHandle = bus->wrapper.handle ;
  rtems_status_code  sc;

  MCAN_Init(hCanHandle);

  //TODO: Actually configure the clock rate correctly.
  configure_can_clocking();

  mcan_start_initialization(bus->wrapper.handle->pMCan);

  // Enable RX FIF0 interrupt
  mcan_enable_interrupt(bus->wrapper.handle->pMCan, MCAN_RX_FIFO_0_NEW_MESSAGE);
  printf("Interrupt Enable: 0x%.08X\n", mcan_read_interrupt_enable(bus->wrapper.handle->pMCan));

  // Associate interrupts to CAN line 0
  bus->wrapper.handle->pMCan->MCAN_ILS  |= MCAN_ILS_RF0NL;

  // Enable CAN Line 1 interrupt
  bus->wrapper.handle->pMCan->MCAN_ILE  |= MCAN_ILE_EINT0;

  PIO_Configure(pinsMcan1, PIO_LISTSIZE(pinsMcan1));

  MCAN_RequestIso11898_1(hCanHandle);
  MCAN_Enable(hCanHandle);
  MCAN_IEnableMessageStoredToRxDedBuffer(hCanHandle, CAN_INTR_LINE_1);
  MCAN_RequestIso11898_1( hCanHandle );

  mcan_end_initialization(bus->wrapper.handle->pMCan);

  sc = rtems_interrupt_handler_install(MCAN_LINE1_IRQn,
    "CAN1 ISR",
    RTEMS_INTERRUPT_UNIQUE,
    MCAN_Handler,
    bus);

  if (sc != RTEMS_SUCCESSFUL){

  }

  return CAN_OK;
}


int atsam_can_de_init
  ( can_bus *self )
{
  atsam_can_bus*   bus = (atsam_can_bus *) self;
  MCan_ConfigType* hCanHandle = bus->wrapper.handle;
  rtems_status_code  sc = RTEMS_SUCCESSFUL;
  int ret = 1;

  if ( &mcan1Config == hCanHandle ) {
    sc = rtems_interrupt_handler_remove(MCAN1_LINE1_IRQn,
      MCAN_Handler, hCanHandle);

    if(sc == RTEMS_SUCCESSFUL){
      ret = 0;
    }
  }

  return ret;
}


int atsam_bsp_register_can
  ( void )
{
  int error = 0;

  if(error == 0 ) {

    bus1 = (atsam_can_bus *) can_bus_alloc_and_init( sizeof( *bus1 ) );

    bus1->base.init            = atsam_can_init;
    bus1->base.de_init         = atsam_can_de_init;
    bus1->base.tx_task         = atsam_tx_task;
    bus1->base.rx_task         = atsam_rx_task;
    bus1->base.set_filter      = atsam_can_set_filter;
    bus1->base.get_num_filters = atsam_can_get_num_filters;
    bus1->base.set_flags       = atsam_can_set_flags;
    bus1->instance             = CAN_ONE;
    bus1->wrapper.handle       = &mcan1Config;

    if ( bus1 == NULL ) {
      return -ENOMEM;
    }

    error = can_bus_register( &bus1->base );
  }


#if atsam_ENABLE_CAN2

  if(error == 0 ) {

    bus2 = (atsam_can_bus *) can_bus_alloc_and_init( sizeof( *bus2 ) );

    bus2->base.init            = atsam_can_init;
    bus2->base.de_init         = atsam_can_de_init;
    bus2->base.tx_task         = atsam_tx_task;
    bus2->base.rx_task         = atsam_rx_task;
    bus2->base.get_num_filters = atsam_can_get_num_filters;
    bus2->base.set_filter      = atsam_can_set_filter;
    bus2->base.set_flags       = atsam_can_set_flags;
    bus2->instance             = CAN_TWO;
    bus2->wrapper = &mcan2Config;

    if ( bus2 == NULL ) {
      return -ENOMEM;
    }

    error = can_bus_register( &bus2->base );
  }

#endif

  return error;
}


uint64_t atsam_can_get_tx_count(const CAN_Instance can_bus) {

  // Increment RX metrics
  if(can_bus == CAN_ONE ) {
    return can_tx_count[0];
  } else if(can_bus == CAN_TWO ){
    return can_tx_count[1];
  }

  return (uint64_t) 0;
}


uint64_t atsam_can_get_rx_count(const CAN_Instance can_bus) {

  // Increment RX metrics
  if(can_bus == CAN_ONE ) {
    return can_rx_count[0];
  } else if(can_bus == CAN_TWO ){
    return can_rx_count[1];
  }

  return (uint64_t) 0;
}
