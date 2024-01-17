/*
 * FreeModbus Libary: BARE Port
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

#ifndef _PORT_H
#define _PORT_H

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include <cmsis_os2.h>

#include <assert.h>
#include <inttypes.h>

#define	INLINE                      inline
#define PR_BEGIN_EXTERN_C           extern "C" {
#define	PR_END_EXTERN_C             }

#define ENTER_CRITICAL_SECTION( )   
#define EXIT_CRITICAL_SECTION( )    

typedef uint8_t BOOL;

typedef unsigned char UCHAR;
typedef char CHAR;

typedef uint16_t USHORT;
typedef int16_t SHORT;

typedef uint32_t ULONG;
typedef int32_t LONG;

#ifndef TRUE
#define TRUE            1
#endif

#ifndef FALSE
#define FALSE           0
#endif

#define MB_TCP_BUF_SIZE     ( 256 + 7 ) /* Must hold a complete Modbus TCP frame. */

typedef struct
{
	struct netconn *newconn;
	xQueueHandle xQueueMbRX;
	xQueueHandle xQueueMbTX;
	uint16_t len;
	void *hTask;

	uint8_t  aucTCPBuf[MB_TCP_BUF_SIZE];
	uint16_t usTCPFrameBytesLeft;
	uint16_t usTCPBufPos;
} stModbusConn;

#endif
