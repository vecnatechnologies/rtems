/**
 * NOTICE: This software  source code and any of  its derivatives are the
 * confidential  and  proprietary   information  of  Vecna  Technologies,
 * Inc. (such source  and its derivatives are hereinafter  referred to as
 * "Confidential Information"). The  Confidential Information is intended
 * to be  used exclusively by  individuals or entities that  have entered
 * into either  a non-disclosure agreement or license  agreement (or both
 * of  these agreements,  if  applicable) with  Vecna Technologies,  Inc.
 * ("Vecna")   regarding  the  use   of  the   Confidential  Information.
 * Furthermore,  the  Confidential  Information  shall be  used  only  in
 * accordance  with   the  terms   of  such  license   or  non-disclosure
 * agreements.   All  parties using  the  Confidential Information  shall
 * verify that their  intended use of the Confidential  Information is in
 * compliance  with and  not in  violation of  any applicable  license or
 * non-disclosure  agreements.  Unless expressly  authorized by  Vecna in
 * writing, the Confidential Information  shall not be printed, retained,
 * copied, or  otherwise disseminated,  in part or  whole.  Additionally,
 * any party using the Confidential  Information shall be held liable for
 * any and  all damages incurred  by Vecna due  to any disclosure  of the
 * Confidential  Information (including  accidental disclosure).   In the
 * event that  the applicable  non-disclosure or license  agreements with
 * Vecna  have  expired, or  if  none  currently  exists, all  copies  of
 * Confidential Information in your  possession, whether in electronic or
 * printed  form, shall be  destroyed or  returned to  Vecna immediately.
 * Vecna  makes no  representations  or warranties  hereby regarding  the
 * suitability  of  the   Confidential  Information,  either  express  or
 * implied,  including  but not  limited  to  the  implied warranties  of
 * merchantability,    fitness    for    a   particular    purpose,    or
 * non-infringement. Vecna  shall not be liable for  any damages suffered
 * by  licensee as  a result  of  using, modifying  or distributing  this
 * Confidential Information.  Please email [info@vecnatech.com]  with any
 * questions regarding the use of the Confidential Information.
 */

/*
 * ethernet_fit.cpp
 *
 *  @author jay.doyle
 *
 *  Change Log:
 *    Created Oct. 5, 2015 - jay.doyle
 */

#include <CppUTest/TestHarness.h>
#include <stdio.h>

extern "C" {
#include <hal-utils.h>
#include <bspopts.h>
#include <string.h>
#include <lwip/netif.h>
#include <lwip/tcpip.h>
#include <hal-ethernetif.h>
#include <lwip/sockets.h>
#include <lwip/ip_addr.h>
#include <httpserver-socket.h>
#include <cmsis_os.h>
}

/*Static IP ADDRESS*/
#define IP_ADDR0 192
#define IP_ADDR1 168
#define IP_ADDR2 0
#define IP_ADDR3 11

/*NETMASK*/
#define NETMASK_ADDR0 255
#define NETMASK_ADDR1 255
#define NETMASK_ADDR2 255
#define NETMASK_ADDR3 0

/*Gateway Address*/
#define GW_ADDR0 192
#define GW_ADDR1 168
#define GW_ADDR2 0
#define GW_ADDR3 150

TEST_GROUP( hal_ethernet_fit )
{
  void setup()
  {
  }

  void teardown()
  {
  }
};

TEST( hal_ethernet_fit, check_ethernetif_init )
{
  err_t ret;

  ip_addr_t    ipaddr;
  ip_addr_t    netmask;
  ip_addr_t    gw;
#if 0
  uint8_t      link_status;
  struct netif gnetif; /* network interface structure */
#endif

  /* IP address setting */
  IP4_ADDR( &ipaddr, IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3 );
  IP4_ADDR( &netmask,
    NETMASK_ADDR0,
    NETMASK_ADDR1,
    NETMASK_ADDR2,
    NETMASK_ADDR3 );
  IP4_ADDR( &gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3 );

  // Keep track of the initial number of tasks
  uint32_t init_num_tasks = tasks_get_num();

  // Create tcp_ip stack thread
  tcpip_init( NULL, NULL );
  LONGS_EQUAL_TEXT( ( init_num_tasks + 1UL ),
    tasks_get_num(), "TCP/IP task not created as expected" );

  ret = ethernetif_init( NULL );
  LONGS_EQUAL_TEXT( ERR_ARG, ret, "Invalid handling of NULL netif argument" );

#if 0
  netif_add( &gnetif,
    &ipaddr,
    &netmask,
    &gw,
    NULL,
    &ethernetif_init,
    &tcpip_input );

  /*  Registers the default network interface. */
  netif_set_default( &gnetif );

  CHECK_TEXT( gnetif.output != NULL,
    "etharp_output not initialized as expected" );
  CHECK_TEXT( gnetif.linkoutput != NULL,
    "low level output function not initialized as expected" );

  link_status = netif_is_link_up( &gnetif );
  LONGS_EQUAL_TEXT( link_status, 1UL, "Unable to establish Ethernet link" );
#endif
}

TEST( hal_ethernet_fit, check_http_server_socket_init )
{
  // Keep track of the initial number of tasks
  uint32_t init_num_tasks = tasks_get_num();

  // Create http_server thread
  http_server_socket_init();
  LONGS_EQUAL_TEXT( ( init_num_tasks + 1UL ),
    tasks_get_num(), "HTTP server task not created as expected" );
}

