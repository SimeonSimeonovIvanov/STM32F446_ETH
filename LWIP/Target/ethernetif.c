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
#include "main.h"
#include "lwip/opt.h"

#include "lwip/timeouts.h"
#include "netif/ethernet.h"
#include "netif/etharp.h"
#include "lwip/ethip6.h"
#include "ethernetif.h"
#include <string.h>
#include "cmsis_os.h"
#include "lwip/tcpip.h"

#include "../../lwip/Target/enc424j600/enc424j600.h"

#include "cmsis_os.h"
#include "semphr.h"

extern osSemaphoreId_t myBinarySemSpiHandle;

/* Within 'USER CODE' section, code will be kept by default at each generation */
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* Private define ------------------------------------------------------------*/
#define TIME_WAITING_FOR_INPUT ( portMAX_DELAY )
/* USER CODE BEGIN OS_THREAD_STACK_SIZE_WITH_RTOS */
/* Stack size of the interface thread */
#define INTERFACE_THREAD_STACK_SIZE ( 1024 )

/* Network interface name */
#define IFNAME0 's'
#define IFNAME1 't'

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
  #pragma data_alignment=4
#endif
//__ALIGN_BEGIN ETH_DMADescTypeDef  DMARxDscrTab[ETH_RXBUFNB] __ALIGN_END;/* Ethernet Rx MA Descriptor */

#if defined ( __ICCARM__ ) /*!< IAR Compiler */
  #pragma data_alignment=4
#endif
//__ALIGN_BEGIN ETH_DMADescTypeDef  DMATxDscrTab[ETH_TXBUFNB] __ALIGN_END;/* Ethernet Tx DMA Descriptor */

#if defined ( __ICCARM__ ) /*!< IAR Compiler */
  #pragma data_alignment=4
#endif
__ALIGN_BEGIN uint8_t Rx_Buff[2*ETH_RX_BUF_SIZE] __ALIGN_END; /* Ethernet Receive Buffer */

#if defined ( __ICCARM__ ) /*!< IAR Compiler */
  #pragma data_alignment=4
#endif
__ALIGN_BEGIN uint8_t Tx_Buff[2*ETH_TX_BUF_SIZE] __ALIGN_END; /* Ethernet Transmit Buffer */

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

/* Semaphore to signal incoming packets */
osSemaphoreId s_xSemaphore = NULL;
/* Global Ethernet handle */
ETH_HandleTypeDef heth;

/* USER CODE BEGIN 3 */

/* USER CODE END 3 */

/* Private functions ---------------------------------------------------------*/

void HAL_ETH_MspInit(ETH_HandleTypeDef* ethHandle)
{
}

void HAL_ETH_MspDeInit(ETH_HandleTypeDef* ethHandle)
{
}

/**
  * @brief  Ethernet Rx Transfer completed callback
  * @param  heth: ETH handle
  * @retval None
  */
void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *heth)
{
  osSemaphoreRelease(s_xSemaphore);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/*******************************************************************************
                       LL Driver Interface ( LwIP stack --> ETH)
*******************************************************************************/
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
  HAL_StatusTypeDef hal_eth_init_status;
/* USER CODE BEGIN OS_THREAD_ATTR_CMSIS_RTOS_V2 */
  osThreadAttr_t attributes;
/* USER CODE END OS_THREAD_ATTR_CMSIS_RTOS_V2 */

/* Init ETH */

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  //hal_eth_init_status = HAL_ETH_Init(&heth);

  enc424j600Init( MACAddr );
  hal_eth_init_status = HAL_OK;
  //if (hal_eth_init_status == HAL_OK)
  {
    /* Set netif link flag */
	  netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;
  }

  /* Initialize the RX POOL */
  //LWIP_MEMPOOL_INIT(RX_POOL);

#if LWIP_ARP || LWIP_ETHERNET
  /* set MAC hardware address length */
  netif->hwaddr_len = ETH_HWADDR_LEN;

  /* set MAC hardware address */
  netif->hwaddr[0] =  MACAddr[0];
  netif->hwaddr[1] =  MACAddr[1];
  netif->hwaddr[2] =  MACAddr[2];
  netif->hwaddr[3] =  MACAddr[3];
  netif->hwaddr[4] =  MACAddr[4];
  netif->hwaddr[5] =  MACAddr[5];
  /*netif->hwaddr[0] =  MACAddr[5];
  netif->hwaddr[1] =  MACAddr[4];
  netif->hwaddr[2] =  MACAddr[3];
  netif->hwaddr[3] =  MACAddr[2];
  netif->hwaddr[4] =  MACAddr[1];
  netif->hwaddr[5] =  MACAddr[0];*/

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
  s_xSemaphore = osSemaphoreNew(1, 1, NULL);

/* create the task that handles the ETH_MAC */
/* USER CODE BEGIN OS_THREAD_NEW_CMSIS_RTOS_V2 */
  memset(&attributes, 0x0, sizeof(osThreadAttr_t));
  attributes.name = "EthIf";
  attributes.stack_size = INTERFACE_THREAD_STACK_SIZE;
  attributes.priority = osPriorityRealtime;
  osThreadNew(ethernetif_input, netif, &attributes);
/* USER CODE END OS_THREAD_NEW_CMSIS_RTOS_V2 */
  /* Enable MAC and DMA transmission and reception */
  //HAL_ETH_Start(&heth);

/* USER CODE BEGIN PHY_PRE_CONFIG */

/* USER CODE END PHY_PRE_CONFIG */

  /**** Configure PHY to generate an interrupt when Eth Link state changes ****/
  /* Read Register Configuration */
  //HAL_ETH_ReadPHYRegister(&heth, PHY_MICR, &regvalue);

  //regvalue |= (PHY_MICR_INT_EN | PHY_MICR_INT_OE);

  /* Enable Interrupts */
  //HAL_ETH_WritePHYRegister(&heth, PHY_MICR, regvalue );

  /* Read Register Configuration */
  //HAL_ETH_ReadPHYRegister(&heth, PHY_MISR, &regvalue);

  //regvalue |= PHY_MISR_LINK_INT_EN;

  /* Enable Interrupt on change of link status */
  //HAL_ETH_WritePHYRegister(&heth, PHY_MISR, regvalue);

/* USER CODE BEGIN PHY_POST_CONFIG */

/* USER CODE END PHY_POST_CONFIG */

#endif /* LWIP_ARP || LWIP_ETHERNET */

/* USER CODE BEGIN LOW_LEVEL_INIT */

  netif_set_up(netif);
  netif_set_link_up(netif);
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

static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
	uint16_t framelength = 0;
	struct pbuf *q;
	err_t errval;

	xSemaphoreTake( myBinarySemSpiHandle, (TickType_t)portMAX_DELAY );
#if ETH_PAD_SIZE
	pbuf_header(p, -ETH_PAD_SIZE); /* drop the padding word */
#endif
	for(q = p; q != NULL; q = q->next )
	{	/* copy frame from pbufs to driver buffers */
		memcpy( &Tx_Buff[framelength], q->payload, q->len );
		framelength += q->len;
	}
	errval = enc424j600PacketSend( Tx_Buff, p->tot_len/*framelength*/ ) ? !ERR_OK:ERR_OK;
#if ETH_PAD_SIZE
	pbuf_header(p, ETH_PAD_SIZE); /* reclaim the padding word */
#endif
	LINK_STATS_INC(link.xmit);
	xSemaphoreGive( myBinarySemSpiHandle );

	return errval;
}

static err_t low_level_output_(struct netif *netif, struct pbuf *p)
{
	err_t errval = ERR_OK;
	struct pbuf *q;

	xSemaphoreTake( myBinarySemSpiHandle, (TickType_t)portMAX_DELAY );
#if ETH_PAD_SIZE
	pbuf_header(p, -ETH_PAD_SIZE); /* drop the padding word */
#endif
	enc424j600PacketBegin();
	for(q = p; q != NULL; q = q->next )
	{	/* copy frame from pbufs to driver buffers */
		enc424j600PacketSendPart( q->payload, q->len );
	}
	enc424j600PacketEnd( p->tot_len );
#if ETH_PAD_SIZE
	pbuf_header(p, ETH_PAD_SIZE); /* reclaim the padding word */
#endif
	LINK_STATS_INC(link.xmit);

	xSemaphoreGive( myBinarySemSpiHandle );
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

	xSemaphoreTake( myBinarySemSpiHandle, (TickType_t)portMAX_DELAY );
	len = enc424j600PacketReceive( Rx_Buff, 2*ETH_RX_BUF_SIZE );
	if( len<4 )
	{
		xSemaphoreGive( myBinarySemSpiHandle );
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

	xSemaphoreGive( myBinarySemSpiHandle );
	return p;
}

static struct pbuf * low_level_input_(struct netif *netif)
{
	struct pbuf *p = NULL;
	struct pbuf *q = NULL;
	uint16_t len = 0;

	xSemaphoreTake( myBinarySemSpiHandle, (TickType_t)portMAX_DELAY );

	len = enc424j600PacketReceiveBegin();
	if( len<4 )
	{
		xSemaphoreGive( myBinarySemSpiHandle );
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

	xSemaphoreGive( myBinarySemSpiHandle );
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
  struct pbuf *p;
  struct netif *netif = (struct netif *) argument;

  for( ;; )
  {
    if( osSemaphoreAcquire(s_xSemaphore, TIME_WAITING_FOR_INPUT) == osOK )
    {
      do
      {
        LOCK_TCPIP_CORE();
        p = low_level_input( netif );
        if (p != NULL)
        {
          if (netif->input( p, netif) != ERR_OK )
          {
            pbuf_free(p);
          }
        }
        UNLOCK_TCPIP_CORE();
      } while(p!=NULL);
    }
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

/* USER CODE END 6 */

/**
  * @brief  This function sets the netif link status.
  * @param  netif: the network interface
  * @retval None
  */
void ethernetif_set_link(void* argument)
{
	struct link_str *link_arg = (struct link_str *)argument;
	uint32_t regvalue = 0;

	for(;;)
	{
		if( xSemaphoreTake( myBinarySemSpiHandle, (TickType_t)portMAX_DELAY ) == pdTRUE )
		{
			if( enc424j600MACIsLinked() )
			{
				regvalue = 1;
			} else {
				regvalue = 0;
			}
			xSemaphoreGive( myBinarySemSpiHandle );
		}

		/* Check whether the netif link down and the PHY link is up */
		if(!netif_is_link_up(link_arg->netif) && regvalue)
		{	/* network cable is connected */
			netif_set_up(link_arg->netif);
			netif_set_link_up(link_arg->netif);

//#ifdef USE_DHCP
			extern struct netif gnetif;
  dhcp_start(&gnetif);
//#endif
		}
		else if(netif_is_link_up(link_arg->netif) && !regvalue)
		{	/* network cable is dis-connected */
			netif_set_down(link_arg->netif);
			netif_set_link_down(link_arg->netif);
		}

		osDelay(200);
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
  __IO uint32_t tickstart = 0;
  uint32_t regvalue = 0;

  if(netif_is_link_up(netif))
  {
	uint8_t MACAddr[6];
	//enc424j600Init( MACAddr );

  }
  else
  {
    /* Stop MAC interface */
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
/* USER CODE END 8 */
#endif /* LWIP_NETIF_LINK_CALLBACK */

/* USER CODE BEGIN 9 */

/* USER CODE END 9 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

