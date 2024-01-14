/**
 ******************************************************************************
  * File Name          : LWIP.c
  * Description        : This file provides initialization code for LWIP
  *                      middleWare.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "lwip.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#if defined ( __CC_ARM )  /* MDK ARM Compiler */
#include "lwip/sio.h"
#endif /* MDK ARM Compiler */
#include "ethernetif.h"
#include "priv/tcpip_priv.h"
#include <string.h>

/*Static IP ADDRESS*/
#define IP_ADDR0   192
#define IP_ADDR1   168
#define IP_ADDR2   100
#define IP_ADDR3   20
/*NETMASK*/
#define NETMASK_ADDR0   255
#define NETMASK_ADDR1   255
#define NETMASK_ADDR2   255
#define NETMASK_ADDR3   0
/*Gateway Address*/
#define GW_ADDR0   0
#define GW_ADDR1   0
#define GW_ADDR2   0
#define GW_ADDR3   0

/* Variables Initialization */
struct netif gnetif;
ip4_addr_t ipaddr;
ip4_addr_t netmask;
ip4_addr_t gw;

static osThreadAttr_t attributes;
static struct link_str link_arg; // Ethernet link thread Argument

void MX_LWIP_Init(void)
{	/* Initilialize the LwIP stack with RTOS */
	tcpip_init( NULL, NULL );
	LOCK_TCPIP_CORE();
#if LWIP_DHCP
	/* IP addresses initialization with DHCP (IPv4) */
	ipaddr.addr = 0;
	netmask.addr = 0;
	gw.addr = 0;
#else
	IP4_ADDR(&ipaddr, IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
	IP4_ADDR(&netmask, NETMASK_ADDR0, NETMASK_ADDR1, NETMASK_ADDR2, NETMASK_ADDR3);
	IP4_ADDR(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
#endif
	/* add the network interface (IPv4/IPv6) with RTOS */
	netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &tcpip_input);
	netif_set_down(&gnetif);
	netif_set_link_down(&gnetif);
	/* Registers the default network interface */
	netif_set_default(&gnetif);
	netif_set_addr(&gnetif, &ipaddr, &netmask, &gw);
	/* Set the link callback function, this function is called on change of link status*/
	netif_set_link_callback(&gnetif, ethernetif_update_config);
	httpd_init();
#if LWIP_DHCP
	//dhcp_start(&gnetif);
#endif /* LWIP_DHCP */
	/* Create the Ethernet link handler thread */
	link_arg.netif = &gnetif;
	link_arg.semaphore = osSemaphoreNew(1, 1, NULL);
	memset(&attributes, 0x0, sizeof(osThreadAttr_t));
	attributes.name = "LinkThr";
	attributes.stack_size = 2048;
	attributes.priority = osPriorityNormal;
	osThreadNew(ethernetif_set_link, &link_arg, &attributes);
	UNLOCK_TCPIP_CORE();
}
