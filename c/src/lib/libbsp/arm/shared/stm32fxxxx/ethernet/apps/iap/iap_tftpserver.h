/**
 * @file iap_tftpserver.h
 *
 * @ingroup iap
 *
 * @brief TFTP Server for In-Application Programming. Based on LWIP.
 *
 */

/*
 * Copyright (c) 2015 Vecna Technologies, Inc.
 *
 * Author: Sudarshan Rajagopalan
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.com/license/LICENSE.
 */

#ifndef __TFTPSERVER_H_
#define __TFTPSERVER_H_

#include <lwip/mem.h>
#include <lwip/udp.h>
#include <lwip/ip4_addr.h>

#define TFTP_OPCODE_LEN         2
#define TFTP_BLKNUM_LEN         2
#define TFTP_DATA_LEN_MAX       512
#define TFTP_DATA_PKT_HDR_LEN   (TFTP_OPCODE_LEN + TFTP_BLKNUM_LEN)
#define TFTP_ERR_PKT_HDR_LEN    (TFTP_OPCODE_LEN + TFTP_ERRCODE_LEN)
#define TFTP_ACK_PKT_LEN        (TFTP_OPCODE_LEN + TFTP_BLKNUM_LEN)
#define TFTP_DATA_PKT_LEN_MAX   (TFTP_DATA_PKT_HDR_LEN + TFTP_DATA_LEN_MAX)
#define TFTP_MAX_RETRIES        3
#define TFTP_TIMEOUT_INTERVAL   5

/* Device Network Address Info

 /*Static IP ADDRESS*/
#define IP_ADDR0   192
#define IP_ADDR1   168
#define IP_ADDR2   0
#define IP_ADDR3   12

/*NETMASK*/
#define NETMASK_ADDR0   255
#define NETMASK_ADDR1   255
#define NETMASK_ADDR2   255
#define NETMASK_ADDR3   0

/*Gateway Address*/
#define GW_ADDR0   192
#define GW_ADDR1   168
#define GW_ADDR2   0
#define GW_ADDR3   150

#define MAC_ADDR0   2
#define MAC_ADDR1   0
#define MAC_ADDR2   0
#define MAC_ADDR3   5
#define MAC_ADDR4   0
#define MAC_ADDR5   0

typedef struct
{
  /* Status of firmware image being downloaded */
  bool firmware_corrupted;

  /* Valid firmware flashed or not */
  bool firmware_flashed;

  /* Firmware image to be flashed requested */
    bool firmware_flash_requested;

}tftp_firmware_image_info;


typedef struct
{
  int op;    /*WRQ */
  /* last block read */
  char data[TFTP_DATA_PKT_LEN_MAX];
  int  data_len;
  /* destination ip:port */
  struct ip4_addr to_ip;
  int to_port;
  /* next block number */
  int block;
  /* total number of bytes transferred */
  int tot_bytes;
  /* timer interrupt count when last packet was sent */
  /* this should be used to resend packets on timeout */
  unsigned long long last_time;
 
}tftp_connection_args;


/* TFTP opcodes as specified in RFC1350   */
typedef enum {
  TFTP_RRQ = 1,
  TFTP_WRQ = 2,
  TFTP_DATA = 3,
  TFTP_ACK = 4,
  TFTP_ERROR = 5
} tftp_opcode;


/* TFTP error codes as specified in RFC1350  */
typedef enum {
  TFTP_ERR_NOTDEFINED,
  TFTP_ERR_FILE_NOT_FOUND,
  TFTP_ERR_ACCESS_VIOLATION,
  TFTP_ERR_DISKFULL,
  TFTP_ERR_ILLEGALOP,
  TFTP_ERR_UKNOWN_TRANSFER_ID,
  TFTP_ERR_FILE_ALREADY_EXISTS,
  TFTP_ERR_NO_SUCH_USER,
} tftp_errorcode;


void IAP_tftpd_init(void);

#endif

