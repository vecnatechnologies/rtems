/**
 * @file iap_init.c
 *
 * @ingroup iap
 *
 * @brief Initialization routines for IAP.
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


#include <lwip/netif.h>
#include <lwip/tcpip.h>
#include <lwip/udp.h>
#include <lwip/mem.h>
#include <lwip/sockets.h>
#include <lwip/ip_addr.h>

#include "iap_tftpserver.h"
#include <hal-ethernetif.h>

  uint8_t stm32_device_mac_address[ 6 ];
  uint8_t stm32_device_ip_address[ 6 ];

struct netif gnetif; /* network interface structure */

//bool tcpip_init_done = false;

void stm32f_set_mac_addr(uint8_t* macaddress){

  macaddress[0] = MAC_ADDR0;
  macaddress[1] = MAC_ADDR1;
  macaddress[2] = MAC_ADDR2;
  macaddress[3] = MAC_ADDR3;
  macaddress[4] = MAC_ADDR4;
  macaddress[5] = MAC_ADDR5;
}

/**
  * @brief  Initializes the lwIP stack
  * @param  None
  * @retval None
  */
static void stm32_config_netif(void)
{
  ip_addr_t ipaddr;
  ip_addr_t netmask;
  ip_addr_t gw;

  /* IP address setting */
  IP4_ADDR(&ipaddr, IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
  IP4_ADDR(&netmask, NETMASK_ADDR0, NETMASK_ADDR1 , NETMASK_ADDR2, NETMASK_ADDR3);
  IP4_ADDR(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);

  /* - netif_add(struct netif *netif, struct ip_addr *ipaddr,
  struct ip_addr *netmask, struct ip_addr *gw,
  void *state, err_t (* init)(struct netif *netif),
  err_t (* input)(struct pbuf *p, struct netif *netif))

  Adds your network interface to the netif_list. Allocate a struct
  netif and pass a pointer to this structure as the first argument.
  Give pointers to cleared ip_addr structures when using DHCP,
  or fill them with sane numbers otherwise. The state pointer may be NULL.

  The init function pointer must point to a initialization function for
  your ethernet netif interface. The following code illustrates it's use.*/

  netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &tcpip_input);

  /*  Registers the default network interface. */
  netif_set_default(&gnetif);

  //TODO: Is this correct?
  //autoip_start(&gnetif);

  if (netif_is_link_up(&gnetif))
  {
    /* When the netif is fully configured this function must be called.*/
    netif_set_up(&gnetif);
  }
  else
  {
    /* When the netif link is down this function must be called */
    netif_set_down(&gnetif);
  }
}

rtems_task stm32_ethernet_iap_init(
  rtems_task_argument task_index
)
{
   /*Create tcp_ip stack thread*/
  tcpip_init(NULL, NULL);

  /*Initialize the LwIP stack*/
  stm32_config_netif();

  //tcpip_init_done = true;

  /* Initialize the TFTP server demo */
  IAP_tftpd_init();

  rtems_task_delete(RTEMS_SELF);
}
