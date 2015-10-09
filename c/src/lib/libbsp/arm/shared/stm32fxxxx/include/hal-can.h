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
#include <dev/can/can-internal.h>

/**
 *  CAN Status
 **/
typedef enum {
  CAN_OK = 0,
  CAN_ERROR,
  CAN_BUSY,
  CAN_TIMEOUT
} CAN_Status;

/**
 * CAN Instance
 **/
typedef enum {
  CAN_ONE = 0,
  CAN_TWO,

  NUM_CAN_INSTANCES
} CAN_Instance;

/**
 *  Interrupt Enable
 **/
typedef enum {
  DISABLE_INT = 0,
  ENABLE_INT
} CAN_InterruptEnable;

/**
 * CAN Hardware FIFOs
 **/
typedef enum {
  FIFO0 = 0,
  FIFO1
} CAN_FIFO;



/**
 * The maximum bit segment 1 setting for the CAN
 * clock configuration.
 */
#define BS1_MAX  16UL

/**
 * The maximum bit segment 2 setting for the CAN
 * clock configuration.
 */
#define BS2_MAX  8UL

/**
 *
 * Timing information for configuring
 * baud rate on the BxCAN interface.
 *
 * See the STM32F4 Technical Reference Manual
 *
 * 1 BS = 1 Time Quanta, or TQ.
 *
 * A TQ is the smallest unit of time that can be used to
 * create a CAN timing profile. It is Pre-scaler * (1 / Peripheral Clock)
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

  /**
   * The error between the desired baud rate and the actual
   * generated baud rate in bits per second.
   */
  float   error;
} CAN_Timing_Values;


/**
 * @brief This function registers any CAN buses that were
 *   enabled in the BSP configure.ac file via STM32_ENABLE_CANX
 *   defines.
 *
 * @returns 0 If all configured CAN bus were registered with RTEMS
 *   succesfully, non-zero otherwise.
 */
int stm32_bsp_register_can(
  void
);

/**
 * @brief This function returns then number of CAN bus messages
 *   transmitted on the specified CAN bus since the system started up.
 *
 * @param can_bus The target can bus
 *
 * @return The total number of CAN bus messages transmitted.
 *  If the input can_bus is invalid then 0 is returned.
 */
uint64_t stm32_can_get_tx_count(
  const CAN_Instance can_bus
);

/**
 * @brief This function returns then number of CAN bus messages
 *   received on the specified CAN bus since the system started up.
 *
 * @param can_bus The target can bus
 *
 * @return The total number of receive CAN bus messages received
 *  (not including messages masked out by HW filters).  If the
 *  input can_bus is invalid then 0 is returned.
 */
uint64_t stm32_can_get_rx_count(
  const CAN_Instance can_bus
);

/**
 * @brief This function returns the number of CAN bus HW filters
 *        available for the specified CAN bus.
 *
 * @param self The target CAN bus.
 *
 * @return The number of hardware filters available for the
 *   specified CAN bus.  It the input argument is invalid then 0
 *   is returned.
 */
int stm32_can_get_num_filters(
  can_bus *self
);

/**
 *  @brief Calculate the can timing values used to initialize
 *   the internal clock configuration for the CAN bus.
 *
 *  @param desired_baud The desired baud rate for the CAN bus in
 *    bits per second.
 *
 *  @return The set of CAN clock configuration settings that best fits
 *    the desired bit rate based upon the current processor clock
 *    configuration.  This structure includes Bit segment 1, bit segment 2,
 *    and prescaler values.
 */
CAN_Timing_Values rtems_can_get_timing_values(
  uint32_t desired_baud
);

#endif
