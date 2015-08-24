/**
 * @file can.h
 *
 * @brief Control Area Network  (can) Driver API
 *
 * @ingroup can
 */

/*
 * Copyright (c) 2015 Vecna Technologies, Inc.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.com/license/LICENSE.
 */


#ifndef _DEV_CAN_H
#define _DEV_CAN_H


#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define MAX_CAN_MSG_LEN 8

typedef struct {
  uint32_t id;
  uint8_t len;
  uint8_t data[MAX_CAN_MSG_LEN];
} can_msg;

struct can_filter{
  /**
   * Filter number in can 
   * hardware
   */
  uint16_t number;

  /**
   * 32 bit mask.
   *
   * Use can_filter_stdid to create 
   * a mask for an 11 bit ID/mask.
   */
  uint32_t mask;

  /**
   * 32 bit filter.
   *
   * Use can_filter_stdid to create
   * a filter for an 11 bit id
   */
  uint32_t filter;
};

typedef struct can_filter can_filter;


/**
 * IOCTL Commands
 */
#define IOCTL_CAN_TYPE 72

#define CAN_SET_BAUDRATE        _IOW(IOCTL_CAN_TYPE, 1, unsigned long)
#define CAN_GET_NUM_FILTERS     _IOR(IOCTL_CAN_TYPE, 2, void )
#define CAN_SET_FILTER          _IOW(IOCTL_CAN_TYPE, 3, can_filter)

/**
 * @brief Create an mask/filter for an 11
 * bit CAN id.
 *
 * @param[in] 11 bit CAN id
 * @return 32 bit filter for use in a 
 *            can_filter struct.
 */
uint32_t can_filter_stdid(uint32_t id);



#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _DEV_can_can_H */
