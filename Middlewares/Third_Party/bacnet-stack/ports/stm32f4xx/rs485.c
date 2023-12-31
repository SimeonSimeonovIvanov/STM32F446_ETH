/*
 * @file
 * @brief RS-485 Interface
 *
 * Copyright (C) 2020 Steve Karg <skarg@users.sourceforge.net>
 *
 * @page License
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f4xx.h"
#include "bacnet/basic/sys/mstimer.h"
#include "bacnet/bits.h"
#include "bacnet/basic/sys/fifo.h"
#include "bacnet/datalink/dlmstp.h"
#include "bacnet/datalink/mstpdef.h"
#include "rs485.h"

#include "cmsis_os.h"
#include "main.h"

extern UART_HandleTypeDef huart4;

/* buffer for storing received bytes - size must be power of two */
/* BACnet DLMSTP_MPDU_MAX for MS/TP is 1501 bytes */
static uint8_t Receive_Queue_Data[NEXT_POWER_OF_2(DLMSTP_MPDU_MAX)];
static FIFO_BUFFER Receive_Queue;

/* buffer for storing bytes to transmit */
/* BACnet DLMSTP_MPDU_MAX for MS/TP is 1501 bytes */
static uint8_t Transmit_Queue_Data[NEXT_POWER_OF_2(DLMSTP_MPDU_MAX)];
static FIFO_BUFFER Transmit_Queue;

/* baud rate of the UART interface */
static uint32_t Baud_Rate = 38400;
/* flag to track RTS status */
static volatile bool Transmitting;

/* statistics */
static volatile uint32_t RS485_Transmit_Bytes;
static volatile uint32_t RS485_Receive_Bytes;

/* amount of silence on the wire */
static struct mstimer Silence_Timer;

/**
 * @brief Reset the silence on the wire timer.
 */
void rs485_silence_reset(void)
{
    mstimer_set(&Silence_Timer, 0);
}

/**
 * @brief Determine the amount of silence on the wire from the timer.
 * @param amount of time that might have elapsed
 * @return true if the amount of time has elapsed
 */
bool rs485_silence_elapsed(uint32_t interval)
{
    return (mstimer_elapsed(&Silence_Timer) > interval);
}

/**
 * @brief Determine the turnaround time
 * @return amount of milliseconds
 */
static uint16_t rs485_turnaround_time(void)
{
    /* delay after reception before transmitting - per MS/TP spec */
    /* wait a minimum  40 bit times since reception */
    /* at least 2 ms for errors: rounding, clock tick */
    if (Baud_Rate) {
        return (2 + ((Tturnaround * 1000UL) / Baud_Rate));
    } else {
        return 2;
    }
}

/**
 * @brief Use the silence timer to determine turnaround time
 * @return true if turnaround time has expired
 */
bool rs485_turnaround_elapsed(void)
{
    return (mstimer_elapsed(&Silence_Timer) > rs485_turnaround_time());
}

/**
 * @brief Determines if an error occured while receiving
 * @return true an error occurred
 */
bool rs485_receive_error(void)
{
    return false;
}

static void ClearUartErrors( UART_HandleTypeDef *huart )
{
	if( __HAL_UART_GET_FLAG( huart, UART_FLAG_ORE ) )
	{
		__HAL_UART_CLEAR_OREFLAG( huart );
	}

	if( __HAL_UART_GET_FLAG( huart, UART_FLAG_NE ) )
	{
		__HAL_UART_CLEAR_NEFLAG( huart );
	}

	if( __HAL_UART_GET_FLAG( huart, UART_FLAG_FE ) )
	{
		__HAL_UART_CLEAR_FEFLAG( huart );
	}

	if( __HAL_UART_GET_FLAG( huart, UART_FLAG_PE ) )
	{
		__HAL_UART_CLEAR_PEFLAG( huart );
	}
}

/**
 * @brief USARTx interrupt handler sub-routine
 */
//__HAL_UART_DISABLE_IT(&huart4, UART_IT_RXNE);
//__HAL_UART_ENABLE_IT(&huart4, UART_IT_TXE);
void rs485_Bacnet_Handler(void)
{
	uint8_t data_byte;
	if( SET == __HAL_UART_GET_FLAG( &huart4, UART_FLAG_RXNE ) )
	{	/* Read one byte from the receive data register */
		data_byte = __HAL_UART_FLUSH_DRREGISTER(&huart4);
		if (!Transmitting)
		{
			FIFO_Put(&Receive_Queue, data_byte);
			RS485_Receive_Bytes++;
		}
		__HAL_UART_CLEAR_FLAG( &huart4, UART_FLAG_RXNE );
	}
    if( SET == __HAL_UART_GET_FLAG( &huart4, UART_FLAG_TXE ) )
    {
        if (FIFO_Count(&Transmit_Queue))
        {
            //USART_SendData(USART6, FIFO_Get(&Transmit_Queue));
        	__HAL_UART_FLUSH_DRREGISTER(&huart4) = FIFO_Get(&Transmit_Queue);
            RS485_Transmit_Bytes += 1;
            rs485_silence_reset();
        }
        else
        {	/* disable the USART to generate interrupts on TX empty */
        	__HAL_UART_DISABLE_IT(&huart4, UART_IT_TXE);
            /* enable the USART to generate interrupts on TX complete */
        	__HAL_UART_ENABLE_IT(&huart4, UART_IT_TC);
        }
        //USART_ClearITPendingBit(USART6, USART_IT_TXE);
    }
    if( SET == __HAL_UART_GET_FLAG( &huart4, UART_FLAG_TC ) )
    {
        rs485_rts_enable(false);
        /* disable the USART to generate interrupts on TX complete */
        __HAL_UART_DISABLE_IT(&huart4, UART_IT_TC);
        /* enable the USART to generate interrupts on RX not empty */
        __HAL_UART_ENABLE_IT(&huart4, UART_IT_RXNE);
        __HAL_UART_CLEAR_FLAG( &huart4, UART_FLAG_TC );
    }
    /* check for errors and clear them */
    if( SET == __HAL_UART_GET_FLAG( &huart4, UART_FLAG_ORE ) )
	{	/* note: enabling RXNE interrupt also enables the ORE interrupt! */
        /* dummy read to clear error state */
    	data_byte = __HAL_UART_FLUSH_DRREGISTER(&huart4);
    	__HAL_UART_CLEAR_FLAG( &huart4, UART_FLAG_TC );
    }
    ClearUartErrors(&huart4);
}

/**
 * @brief Control the DE and /RE pins on the RS-485 transceiver
 * @param enable - true to set DE and /RE high, false to set /DE and RE low
 */
void rs485_rts_enable(bool enable)
{
    Transmitting = enable;
    if (Transmitting)
    {
    	UART4_RTS_GPIO_Port->BSRR |= UART4_RTS_Pin;
    } else {
    	UART4_RTS_GPIO_Port->BSRR |= UART4_RTS_Pin<<16;
    }
}

/**
 * @brief Determine the status of the transmit-enable line on the RS-485
 *  transceiver
 * @return true if RTS is enabled, false if RTS is disabled
 */
bool rs485_rts_enabled(void)
{
    return Transmitting;
}

/**
 * @brief Return true if a byte is available
 * @param data_register - byte in this parameter if there is one available
 * @return true if a byte is available, with the byte in the parameter
 */
bool rs485_byte_available(uint8_t *data_register)
{
    bool data_available = false; /* return value */

    if (!FIFO_Empty(&Receive_Queue)) {
        if (data_register) {
            *data_register = FIFO_Get(&Receive_Queue);
        }
        rs485_silence_reset();
        data_available = true;
    }

    return data_available;
}

/**
 * @brief Transmit one or more bytes on RS-485.
 * @param buffer - array of one or more bytes to transmit
 * @param nbytes - number of bytes to transmit
 * @return true if added to queue
 */
bool rs485_bytes_send(uint8_t *buffer, uint16_t nbytes)
{
    bool status = false;

    if (buffer && (nbytes > 0))
    {
        if (FIFO_Add(&Transmit_Queue, buffer, nbytes))
        {
            rs485_silence_reset();
            rs485_rts_enable(true);
            /* disable the USART to generate interrupts on RX not empty */
            __HAL_UART_DISABLE_IT(&huart4, UART_IT_RXNE);
            /* enable the USART to generate interrupts on TX empty */
            __HAL_UART_ENABLE_IT(&huart4, UART_IT_TXE);
            /* TXE interrupt will load the first byte */
            status = true;
        }
    }

    return status;
}

/**
 * @brief Configures the baud rate of the USART
 */
static void rs485_baud_rate_configure(void)
{
    /*USART_InitTypeDef USART_InitStructure;

    USART_InitStructure.USART_BaudRate = Baud_Rate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl =
        USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;*/
    /* Configure USARTx */
    //USART_Init(USART6, &USART_InitStructure);
}

/**
 * @brief Initialize the RS-485 baud rate
 * @param baudrate - RS-485 baud rate in bits per second (bps)
 * @return true if set and valid
 */
bool rs485_baud_rate_set(uint32_t baud)
{
    bool valid = true;

    switch (baud) {
        case 9600:
        case 19200:
        case 38400:
        case 57600:
        case 76800:
        case 115200:
            Baud_Rate = baud;
            rs485_baud_rate_configure();
            break;
        default:
            valid = false;
            break;
    }

    return valid;
}

/**
 * @brief Return the RS-485 baud rate
 * @return baud - RS-485 baud rate in bits per second (bps)
 */
uint32_t rs485_baud_rate(void)
{
    return Baud_Rate;
}

/**
 * @brief Return the RS-485 statistics for transmit bytes
 * @return number of bytes transmitted
 */
uint32_t rs485_bytes_transmitted(void)
{
    return RS485_Transmit_Bytes;
}

/**
 * @brief Return the RS-485 statistics for receive bytes
 * @return number of bytes received
 */
uint32_t rs485_bytes_received(void)
{
    return RS485_Receive_Bytes;
}

/**
 * @brief Initialize the USART for RS485
 */
void rs485_init(void)
{
	/* initialize the Rx and Tx byte queues */
    FIFO_Init(&Receive_Queue, &Receive_Queue_Data[0],
        (unsigned)sizeof(Receive_Queue_Data));
    FIFO_Init(&Transmit_Queue, &Transmit_Queue_Data[0],
        (unsigned)sizeof(Transmit_Queue_Data));
    /* enable the USART to generate interrupts on RX */
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_RXNE);
    rs485_baud_rate_set(Baud_Rate);
    rs485_silence_reset();
}
