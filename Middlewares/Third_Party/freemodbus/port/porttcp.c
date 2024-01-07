/*
 * FreeModbus Libary: Win32 Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id$
 */

#include <stdio.h>

#include "main.h"
#include "port.h"
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
/* ----------------------- MBAP Header --------------------------------------*/
#define MB_TCP_UID          6
#define MB_TCP_LEN          4
#define MB_TCP_FUNC         7
/* ----------------------- Defines  -----------------------------------------*/
#define MB_TCP_DEFAULT_PORT 502 /* TCP listening port. */
#define MB_TCP_POOL_TIMEOUT 50  /* pool timeout for event waiting. */
#define MB_TCP_READ_TIMEOUT 1000        /* Maximum timeout to wait for packets. */
#define MB_TCP_READ_CYCLE   100 /* Time between checking for new data. */

#define MB_TCP_DEBUG        1   /* Set to 1 for additional debug output. */

#define EV_CONNECTION       0
#define EV_CLIENT           1
#define EV_NEVENTS          EV_CLIENT + 1
/* ----------------------- External variables -------------------------------*/
stModbusConn arrModbusConn[5];
stModbusConn *lpMbConn;
/* ----------------------- Static variables ---------------------------------*/

/* ----------------------- External functions -------------------------------*/

/* ----------------------- Static functions ---------------------------------*/

/* ----------------------- Begin implementation -----------------------------*/
BOOL xMBTCPPortInit(USHORT usTCPPort)
{
    BOOL bOkay = TRUE;
    return bOkay;
}

void vMBTCPPortClose(void)
{
}

void vMBTCPPortDisable(void)
{
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
BOOL xMBTCPPortGetRequest(UCHAR ** ppucMBTCPFrame, USHORT * usTCPLength)
{
	if(NULL == lpMbConn)
	{
		return FALSE;
	}

    *ppucMBTCPFrame = &lpMbConn->aucTCPBuf[0];
    *usTCPLength = lpMbConn->len;

    /* Reset the buffer. */
    lpMbConn->usTCPBufPos = 0;
    lpMbConn->usTCPFrameBytesLeft = MB_TCP_FUNC;

    return TRUE;
}

BOOL xMBTCPPortSendResponse(const UCHAR * pucMBTCPFrame, USHORT usTCPLength)
{
	if(NULL == lpMbConn)
	{
		xMBPortEventPostTX(lpMbConn, EV_READY);
		return FALSE;
	}

	lpMbConn->len = usTCPLength;
	xMBPortEventPostTX(lpMbConn, EV_FRAME_SENT);

	return TRUE;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
eMBErrorCode xMBTCPPortGetRequestNoPoll(stModbusConn *lpMbConn1, UCHAR ** ppucMBTCPFrame, USHORT * usTCPLength)
{
    *ppucMBTCPFrame = &lpMbConn1->aucTCPBuf[0];
    *usTCPLength = lpMbConn1->len;
    /* Reset the buffer. */
    lpMbConn1->usTCPBufPos = 0;
    lpMbConn1->usTCPFrameBytesLeft = MB_TCP_FUNC;
    return MB_ENOERR;
}

eMBErrorCode eMBTCPSendNoPoll(stModbusConn *lpMbConn, const UCHAR * pucFrame, USHORT usLength )
{	// From mbtcp.c -> eMBTCPSend(...)
    eMBErrorCode eStatus = MB_ENOERR;
    UCHAR        *pucMBTCPFrame = ( UCHAR * ) pucFrame - MB_TCP_FUNC;
    USHORT       usTCPLength = usLength + MB_TCP_FUNC;
    /* The MBAP header is already initialized because the caller calls this
     * function with the buffer returned by the previous call. Therefore we
     * only have to update the length in the header. Note that the length
     * header includes the size of the Modbus PDU and the UID Byte. Therefore
     * the length is usLength plus one.
     */
    pucMBTCPFrame[MB_TCP_LEN] = ( usLength + 1 ) >> 8U;
    pucMBTCPFrame[MB_TCP_LEN + 1] = ( usLength + 1 ) & 0xFF;
    lpMbConn->len = usTCPLength;
    return eStatus;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
BOOL xMBTCPPortSendResponseNull(void)
{
	if(NULL == lpMbConn)
	{
		return FALSE;
	}

	xMBPortEventPostTX(lpMbConn, EV_READY);

	return TRUE;
}
