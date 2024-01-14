/**
  ******************************************************************************
  * File Name          : ethernetif.c
  * Description        : This file provides code for the configuration
  *                      of the Target/ethernetif.c MiddleWare.
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
#include <string.h>

#include "main.h"
#include "cmsis_os.h"
#include "semphr.h"

#include "lwip/opt.h"
#include "lwip/timeouts.h"
#include "netif/ethernet.h"
#include "netif/etharp.h"
#include "lwip/ethip6.h"
#include "ethernetif.h"

#include "lwip/tcpip.h"
#include "priv/tcpip_priv.h"

#include "../../lwip/Target/enc424j600/enc424j600.h"

/* Network interface name */
#define IFNAME0 's'
#define IFNAME1 't'

extern osSemaphoreId_t myBinarySemSpiHandle;

__ALIGN_BEGIN uint8_t Rx_Buff[ETH_RX_BUF_SIZE] __ALIGN_END; /* Ethernet Receive Buffer */
//__ALIGN_BEGIN uint8_t Tx_Buff[ETH_TX_BUF_SIZE] __ALIGN_END; /* Ethernet Transmit Buffer */

/* Semaphore to signal incoming packets */
osSemaphoreId RxPktSemaphore = NULL;
/*******************************************************************************
                       LL Driver Interface ( LwIP stack --> ETH)
*******************************************************************************/
static StaticTask_t  Ethernetif_InputTaskCB;
static StackType_t   Ethernetif_InputTaskStk[512];
const osThreadAttr_t Ethernetif_InputTask_attributes =
{
	.name       = "EthIf",
	.cb_mem     = &Ethernetif_InputTaskCB,
	.cb_size    = sizeof(Ethernetif_InputTaskCB),
	.stack_mem  = &Ethernetif_InputTaskStk,
	.stack_size = sizeof(Ethernetif_InputTaskStk),
	.priority   = (osPriority_t) osPriorityNormal,
};

static struct tcpip_api_call_data tcpip_api_call_dhcp_start_data;

/**
 * In this function, the hardware should be initialized.
 * Called from ethernetif_init().
 *
 * @param netif the already initialized lwip network interface structure
 *        for this ethernetif
 */
static void low_level_init(struct netif *netif)
{
	uint8_t MACAddr[6] = { 0 };

	if( 0 == enc424j600Init(MACAddr) )
	{	/* Set netif link flag */
		netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;
	}
	/* Initialize the RX POOL */
	//LWIP_MEMPOOL_INIT(RX_POOL);
#if LWIP_ARP || LWIP_ETHERNET
	/* set MAC hardware address length */
	netif->hwaddr_len = ETH_HWADDR_LEN;
	/* set MAC hardware address */
	netif->hwaddr[0] = MACAddr[0];
	netif->hwaddr[1] = MACAddr[1];
	netif->hwaddr[2] = MACAddr[2];
	netif->hwaddr[3] = MACAddr[3];
	netif->hwaddr[4] = MACAddr[4];
	netif->hwaddr[5] = MACAddr[5];
	/* maximum transfer unit */
	netif->mtu = 1500;
	/* Accept broadcast address and ARP traffic */
	/* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
#if LWIP_ARP
	netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;
#else
	netif->flags |= NETIF_FLAG_BROADCAST;
#endif /* LWIP_ARP */
	/* create a binary semaphore used for informing ethernetif of frame reception */
	RxPktSemaphore = osSemaphoreNew(1, 1, NULL);
	/* create the task that handles the ETH_MAC */
	osThreadNew(ethernetif_input, netif, &Ethernetif_InputTask_attributes);
#endif /* LWIP_ARP || LWIP_ETHERNET */
}

/**
 * This function should do the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
 * @return ERR_OK if the packet could be sent
 *         an err_t value if the packet couldn't be sent
 *
 * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
 *       strange results. You might consider waiting for space in the DMA queue
 *       to become available since the stack doesn't retry to send a packet
 *       dropped because of memory failure (except for the TCP timers).
 */
/*static err_t low_level_output_(struct netif *netif, struct pbuf *p)
{
	uint16_t framelength = 0;
	struct pbuf *q;
	err_t errval;

	xSemaphoreTake(myBinarySemSpiHandle, (TickType_t)portMAX_DELAY);
#if ETH_PAD_SIZE
	pbuf_header(p, -ETH_PAD_SIZE); // drop the padding word
#endif
	for(q = p; q != NULL; q = q->next )
	{	// copy frame from pbufs to driver buffers
		memcpy( &Tx_Buff[framelength], q->payload, q->len );
		framelength += q->len;
	}
	errval = enc424j600PacketSend(Tx_Buff, p->tot_len) ? !ERR_OK:ERR_OK;
#if ETH_PAD_SIZE
	pbuf_header(p, ETH_PAD_SIZE); // reclaim the padding word
#endif
	LINK_STATS_INC(link.xmit);
	xSemaphoreGive(myBinarySemSpiHandle);

	return errval;
}*/

static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
	err_t errval = ERR_OK;
	struct pbuf *q;

	xSemaphoreTake(myBinarySemSpiHandle, (TickType_t)portMAX_DELAY);
#if ETH_PAD_SIZE
	pbuf_header(p, -ETH_PAD_SIZE); // drop the padding word
#endif
	enc424j600PacketBegin();
	for(q = p; q != NULL; q = q->next )
	{	// copy frame from pbufs to driver buffers
		enc424j600PacketSendPart( q->payload, q->len );
	}
	enc424j600PacketEnd( p->tot_len );
#if ETH_PAD_SIZE
	pbuf_header(p, ETH_PAD_SIZE); // reclaim the padding word
#endif
	LINK_STATS_INC(link.xmit);
	xSemaphoreGive(myBinarySemSpiHandle);

	return errval;
}

/**
 * Should allocate a pbuf and transfer the bytes of the incoming
 * packet from the interface into the pbuf.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return a pbuf filled with the received packet (including MAC header)
 *         NULL on memory error
   */
static struct pbuf * low_level_input(struct netif *netif)
{
	struct pbuf *p = NULL;
	struct pbuf *q = NULL;
	uint16_t len = 0;

	xSemaphoreTake(myBinarySemSpiHandle, (TickType_t)portMAX_DELAY);
	len = enc424j600PacketReceive( Rx_Buff, 2*ETH_RX_BUF_SIZE );
	if( len<4 )
	{
		xSemaphoreGive(myBinarySemSpiHandle);
		return NULL;
	}
#if ETH_PAD_SIZE
	len += ETH_PAD_SIZE; /* allow room for Ethernet padding */
#endif
	p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);
	if( p != NULL )
	{
		uint32_t framelen = 0, offset = 0;
#if ETH_PAD_SIZE
		pbuf_header(p, -ETH_PAD_SIZE); /* drop the padding word */
#endif
		for(q = p; q != NULL; q = q->next)
		{
			memcpy(q->payload, &Rx_Buff[offset+framelen], q->len);
			framelen += q->len;
		}
#if ETH_PAD_SIZE
		pbuf_header(p, ETH_PAD_SIZE); /* reclaim the padding word */
#endif
		LINK_STATS_INC(link.recv);
	} else {
		/* drop packet(); */
		LINK_STATS_INC(link.memerr);
		LINK_STATS_INC(link.drop);
	}
	xSemaphoreGive(myBinarySemSpiHandle);

	return p;
}

static struct pbuf * low_level_input_(struct netif *netif)
{
	struct pbuf *p = NULL;
	struct pbuf *q = NULL;
	uint16_t len = 0;

	xSemaphoreTake(myBinarySemSpiHandle, (TickType_t)portMAX_DELAY);
	len = enc424j600PacketReceiveBegin();
	if( len<4 )
	{
		xSemaphoreGive(myBinarySemSpiHandle);
		return NULL;
	}
#if ETH_PAD_SIZE
	len += ETH_PAD_SIZE; /* allow room for Ethernet padding */
#endif
	p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);
	if( p != NULL )
	{
#if ETH_PAD_SIZE
		pbuf_header(p, -ETH_PAD_SIZE); /* drop the padding word */
#endif
		for(q = p; q != NULL; q = q->next)
		{
			enc424j600PacketReceivePart(q->payload, q->len);
		}
		enc424j600PacketReceiveEnd(p->tot_len);
#if ETH_PAD_SIZE
		pbuf_header(p, ETH_PAD_SIZE); /* reclaim the padding word */
#endif
		LINK_STATS_INC(link.recv);
	} else {
		/* drop packet(); */
		LINK_STATS_INC(link.memerr);
		LINK_STATS_INC(link.drop);
	}
	xSemaphoreGive(myBinarySemSpiHandle);

	return p;
}

/**
 * This function should be called when a packet is ready to be read
 * from the interface. It uses the function low_level_input() that
 * should handle the actual reception of bytes from the network
 * interface. Then the type of the received packet is determined and
 * the appropriate input function is called.
 *
 * @param netif the lwip network interface structure for this ethernetif
 */
void ethernetif_input(void* argument)
{
	struct netif *netif = (struct netif *)argument;
	struct pbuf *p;

	for( ;; )
	{
		osSemaphoreAcquire(RxPktSemaphore, portMAX_DELAY);
		do
		{
			LOCK_TCPIP_CORE();
			p = low_level_input(netif);
			if( NULL != p )
			{
				if( netif->input(p, netif) != ERR_OK )
				{
					pbuf_free(p);
				}
			}
			UNLOCK_TCPIP_CORE();
		} while( NULL != p );
	}
}

#if !LWIP_ARP
/**
 * This function has to be completed by user in case of ARP OFF.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if ...
 */
static err_t low_level_output_arp_off(struct netif *netif, struct pbuf *q, const ip4_addr_t *ipaddr)
{
  err_t errval;
  errval = ERR_OK;

/* USER CODE BEGIN 5 */

/* USER CODE END 5 */

  return errval;

}
#endif /* LWIP_ARP */

/**
 * Should be called at the beginning of the program to set up the
 * network interface. It calls the function low_level_init() to do the
 * actual setup of the hardware.
 *
 * This function should be passed as a parameter to netif_add().
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if the loopif is initialized
 *         ERR_MEM if private data couldn't be allocated
 *         any other err_t on error
 */
err_t ethernetif_init(struct netif *netif)
{
  LWIP_ASSERT("netif != NULL", (netif != NULL));
#if LWIP_NETIF_HOSTNAME
  /* Initialize interface hostname */
  netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */
  netif->name[0] = IFNAME0;
  netif->name[1] = IFNAME1;
  /* We directly use etharp_output() here to save a function call.
   * You can instead declare your own function an call etharp_output()
   * from it if you have to do some checks before sending (e.g. if link
   * is available...) */
#if LWIP_IPV4
#if LWIP_ARP || LWIP_ETHERNET
#if LWIP_ARP
  netif->output = etharp_output;
#else
  /* The user should write ist own code in low_level_output_arp_off function */
  netif->output = low_level_output_arp_off;
#endif /* LWIP_ARP */
#endif /* LWIP_ARP || LWIP_ETHERNET */
#endif /* LWIP_IPV4 */
#if LWIP_IPV6
  netif->output_ip6 = ethip6_output;
#endif /* LWIP_IPV6 */
  netif->linkoutput = low_level_output;
  /* initialize the hardware */
  low_level_init(netif);
  return ERR_OK;
}

/* USER CODE BEGIN 6 */

/**
* @brief  Returns the current time in milliseconds
*         when LWIP_TIMERS == 1 and NO_SYS == 1
* @param  None
* @retval Time
*/
u32_t sys_jiffies(void)
{
  return HAL_GetTick();
}

/**
* @brief  Returns the current time in milliseconds
*         when LWIP_TIMERS == 1 and NO_SYS == 1
* @param  None
* @retval Time
*/
u32_t sys_now(void)
{
  return HAL_GetTick();
}

err_t tcpip_api_call_dhcp_start(struct tcpip_api_call_data *call)
{
	extern struct netif gnetif;
	dhcp_start(&gnetif);
	return ERR_OK;
}

void ethernetif_set_link(void* argument)
{
	struct link_str *link_arg = (struct link_str *)argument;
	uint32_t is_link_up;

	for(;;)
	{
		osDelay(500);

		xSemaphoreTake( myBinarySemSpiHandle, (TickType_t)portMAX_DELAY );
		is_link_up = enc424j600MACIsLinked();
		xSemaphoreGive( myBinarySemSpiHandle );

		osSemaphoreAcquire(link_arg->semaphore, osWaitForever);
		/* Check whether the netif link down and the PHY link is up */
		if( !netif_is_link_up(link_arg->netif) && is_link_up )
		{	/* network cable is connected */
			netif_set_up(link_arg->netif);
			netif_set_link_up(link_arg->netif);
			tcpip_api_call(tcpip_api_call_dhcp_start, &tcpip_api_call_dhcp_start_data);
		} else
		if( netif_is_link_up(link_arg->netif) && !is_link_up )
		{	/* network cable is dis-connected */
			netif_set_down(link_arg->netif);
			netif_set_link_down(link_arg->netif);
		}
		osSemaphoreRelease(link_arg->semaphore);
	}
}

#if LWIP_NETIF_LINK_CALLBACK
/**
  * @brief  Link callback function, this function is called on change of link status
  *         to update low level driver configuration.
* @param  netif: The network interface
  * @retval None
  */
void ethernetif_update_config(struct netif *netif)
{
	if(netif_is_link_up(netif))
	{
		//uint8_t MACAddr[6];
		//enc424j600Init( MACAddr );
	}
	else
	{	/* Stop MAC interface */
		//HAL_ETH_Stop(&heth);
	}
	ethernetif_notify_conn_changed(netif);
}

/* USER CODE BEGIN 8 */
/**
  * @brief  This function notify user about link status changement.
  * @param  netif: the network interface
  * @retval None
  */
__weak void ethernetif_notify_conn_changed(struct netif *netif)
{
  /* NOTE : This is function could be implemented in user file
            when the callback is needed,
  */
}
#endif /* LWIP_NETIF_LINK_CALLBACK */
