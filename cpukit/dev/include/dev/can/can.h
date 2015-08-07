/**
 * @file
 *
 * @brief Control Area Network  (can) Driver API
 *
 * @ingroup can
 */

/*
 * Copyright (c) 2014 embedded brains GmbH.  All rights reserved.
 *
 *  embedded brains GmbH
 *  Dornierstr. 4
 *  82178 Puchheim
 *  Germany
 *  <rtems@embedded-brains.de>
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */


/**
 * Overview
 * ========
 *
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
  uint16_t number;
  uint32_t mask;
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





#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _DEV_can_can_H */
