/*
 * hmi_task.c
 *
 *  Created on: Dec 27, 2020
 *      Author: Simo
 */

#include <string.h>
#include "cmsis_os.h"
#include "semphr.h"

#include "main.h"
#include "bit-array.h"

#include "hmi_task.h"

extern osSemaphoreId_t myBinarySemSpiHandle;
extern SPI_HandleTypeDef hspi2;

static uint8_t hmi_address, hmi_leds[40];
static uint8_t ledsMap[48] =
{
	 4,  5,  6,  7, 12, 13, 14, 23,
	22, 21, 15, 20, 28, 29, 31, 30,
	39, 38, 37, 36,  1,  2,  3,  0,
	 9,  8, 10, 19, 18, 17, 11, 16,
	24, 25, 26, 27, 35, 34, 32, 33,
	40, 41, 42, 43, 44, 45, 46, 47
};

void HmiTask( void *argument )
{
	LP_HMI_TASK_ARG arg = (LP_HMI_TASK_ARG)argument;

	hmiInitLeds();

	for(;;)
	{
		hmiSetLeds( arg->arrInput, 0, 20  );
		hmiSetLeds( arg->arrOutput, 20, 16  );
		hmiSetLeds( &arg->arrInput[25], 36, 1  );
		hmiSetLeds( arg->lpRS485PortLed[0], 37, 1 );
		hmiSetLeds( arg->lpRS485PortLed[1], 38, 1 );
		hmiUpdateLeds();
		osDelay(2);
	}
}

void hmiSetAddress( uint8_t address )
{
	hmi_address = address;
}

void hmiSetLeds( uint8_t *lpLeds, uint8_t index, uint8_t size )
{
	memcpy( &hmi_leds[ index ], lpLeds, size );
}

void hmiUpdateLeds( void )
{
	uint8_t i, temp, txBuffer[6] = { 0 };

	for( i = 0; i < 40; i++ )
	{
		bitarr_write( txBuffer, ledsMap[ i ], 1 & hmi_leds[ i ] );
	}

	temp = hmi_address;
	for( i = 0; i < 8; i++ )
	{
		bitarr_write( txBuffer, ledsMap[ 40 + i ], 1 & temp );
		temp = temp>>1;
	}

	xSemaphoreTake(myBinarySemSpiHandle, (TickType_t)portMAX_DELAY);
	HAL_SPI_Transmit( &hspi2, txBuffer, 6, 100 );

	HAL_GPIO_WritePin( CS0_GPIO_Port, CS0_Pin, GPIO_PIN_SET );
	HAL_GPIO_WritePin( CS1_GPIO_Port, CS1_Pin, GPIO_PIN_SET );

	HAL_GPIO_WritePin( CS0_GPIO_Port, CS0_Pin, GPIO_PIN_RESET );
	HAL_GPIO_WritePin( CS1_GPIO_Port, CS1_Pin, GPIO_PIN_RESET );
	xSemaphoreGive(myBinarySemSpiHandle);
}

void hmiInitLeds( void )
{
	hmi_address = 0;
	memset( hmi_leds, 0, sizeof( hmi_leds ) );
}
