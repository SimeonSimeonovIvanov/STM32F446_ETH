/*
 * bacnet_task.c
 *
 *  Created on: Dec 30, 2023
 *      Author: Simeon
 */
#include "cmsis_os.h"
#include "semphr.h"

#include "bacnet_task.h"
#include "main.h"

#include "bacnet/basic/sys/mstimer.h"
#include "rs485.h"
#include "led.h"
#include "bacnet.h"

#include "bit-array.h"

void BacnetTask( void *argument )
{
	struct mstimer Blink_Timer;

	vTaskDelay(100);

	mstimer_init();
	led_init();
	rs485_init();
	bacnet_init();
	mstimer_set(&Blink_Timer, 125);

	for (;;)
	{
		if (mstimer_expired(&Blink_Timer))
		{
			mstimer_reset(&Blink_Timer);
			led_toggle(LED_LD3);
			led_toggle(LED_RS485);
		}
		led_task();
		bacnet_task();
		portYIELD();
	}
}
