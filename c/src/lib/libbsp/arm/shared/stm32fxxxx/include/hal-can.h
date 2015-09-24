/**
 * @file hal-can.h
 *
 * @ingroup can
 *
 * @brief Public CAN driver datatypes
 *
 */

/*
 * Copyright (c) 2015 Vecna Technologies, Inc.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.com/license/LICENSE.
 */

#ifndef STM32_CAN_H
#define STM32_CAN_H
#include <stdint.h>
#include <stdbool.h>

/* CAN Status*/
typedef enum {
  CAN_OK = 0,
  CAN_ERROR,
  CAN_BUSY,
  CAN_TIMEOUT
} CAN_Status;

/*CAN Instance*/
typedef enum {
  CAN_ONE = 0,
  CAN_TWO,

  NUM_CAN_INSTANCES
} CAN_Instance;

/* Interrupt Enale */
typedef enum {
  DISABLE_INT = 0,
  ENABLE_INT
} CAN_InterruptEnable;

/* CAN Hardware FIFOs*/
typedef enum {
  FIFO0 = 0,
  FIFO1
} CAN_FIFO;

/**
 *
 * Timing information for configuring
 * baud rate on the BxCAN interface.
 *
 * See the STM32F4 Technical Reference Manaul
 *
 * 1 BS = 1 Time Quata, or TQ.
 *
 * A TQ is the smallest unit of time that can be used to
 * create a CAN timing profile. It is Preescaler * (1 / Peripheral Clock)
 *
 * A CAN bit is divided up as follows:
 *
 *   +---+-------------+-------+
 *   |SS |     BS1     | BS2   |
 *   +---+-------------+-------+
 *    ^  ^             ^       ^
 *    |  |             |       |
 *    |  \ Start Bit   |       \ End of Bit
 *    \ Synq Seg       \ Sampling Point
 *
 *  There are also a Sync Segment that is 1 TQ
 *
 *  Therefore the baud rate is the 1/(1 TQ + BS1 + BS2)
 *
 *  Keep in mind that in calculations, we are storing BS1
 *  and BS2 as multiples of 1TQ, a unitless quantity.
 *
 *  Keeping this in mind, the actual equation is
 *
 *  baud = 1 / (TQ * (1 + BS1 + BS2))
 *
 */
typedef struct {
  /**
   * Bit segment 1
   */
  uint32_t s1;

  /**
   * Bit segment 2
   */
  uint32_t s2;

  /**
   * CAN Prescaler
   */
  uint8_t prescaler;

  bool error;
} CAN_Timing_Values;

int      stm32_bsp_register_can( void );
uint64_t stm32_can_get_tx_count( const CAN_Instance can_bus );
uint64_t stm32_can_get_rx_count( const CAN_Instance can_bus );

#endif
