/*
 * FreeModbus Libary: Atmel AT91SAM3S Demo Application
 * Copyright (C) 2010 Christian Walter <cwalter@embedded-solutions.at>
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * IF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * File: $Id$
 */
/* ----------------------- System includes ----------------------------------*/
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include "main.h"
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
/* ----------------------- Variables ----------------------------------------*/
extern stModbusConn arrModbusConn[NUMBER_OF_MB_MASTER_TCP_CONN];
extern stModbusConn *lpMbConn;
/* ----------------------- Start implementation -----------------------------*/
BOOL xMBPortEventInit( void )
{
	for(int i = 0; i < len_of_array(arrModbusConn); i++)
	{
		if( NULL == (arrModbusConn[i].xQueueMbRX = xQueueCreate(1, sizeof(eMBEventType))) )
		{
			return FALSE;
		}
		if( NULL == (arrModbusConn[i].xQueueMbTX = xQueueCreate(1, sizeof(eMBEventType))) )
		{
			return FALSE;
		}
	}
	return TRUE;
}

void vMBPortEventClose( void )
{
	for(int i = 0; i < len_of_array(arrModbusConn); i++)
	{
		if( NULL != arrModbusConn[i].xQueueMbRX )
		{
			vQueueDelete(arrModbusConn[i].xQueueMbRX);
			arrModbusConn[i].xQueueMbRX = NULL;
		}
		if( NULL != arrModbusConn[i].xQueueMbTX )
		{
			vQueueDelete(arrModbusConn[i].xQueueMbTX);
			arrModbusConn[i].xQueueMbTX = NULL;
		}
	}
}
///////////////////////////////////////////////////////////////////////////////
BOOL xMBPortEventPost(eMBEventType eEvent)
{
	return xQueueSend(lpMbConn->xQueueMbRX, (const void *)&eEvent, portMAX_DELAY);
}

BOOL xMBPortEventGet(eMBEventType * peEvent)
{
	static uint8_t i = 0;
	BOOL res = FALSE;

	if( xQueueReceive(arrModbusConn[i].xQueueMbRX, peEvent, 0) )
	{
		lpMbConn = &arrModbusConn[i];
		res = TRUE;
	}

	if( ++i >= len_of_array(arrModbusConn) )
	{
		i = 0;
	}

	return res;
}
///////////////////////////////////////////////////////////////////////////////
BOOL xMBPortEventPostTX(stModbusConn *lpMbConn, eMBEventType eEvent)
{
	return xQueueSend(lpMbConn->xQueueMbTX, (const void *)&eEvent, portMAX_DELAY);
}

BOOL xMBPortEventGetTX(stModbusConn *lpMbConn, eMBEventType *peEvent)
{
	if( xQueueReceive(lpMbConn->xQueueMbTX, peEvent, portTICK_RATE_MS * 1000) )
	{
		return TRUE;
	}
	return FALSE;
}
///////////////////////////////////////////////////////////////////////////////
