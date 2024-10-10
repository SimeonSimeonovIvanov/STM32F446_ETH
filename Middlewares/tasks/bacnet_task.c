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

#include "bacnet/basic/object/device.h"
#include "bacnet/basic/sys/mstimer.h"
#include "bacnet/basic/sys/ringbuf.h"
#include "bacnet/datalink/datalink.h"
#include "bacnet/datalink/dlmstp.h"
#include "bacnet/datalink/mstp.h"
#include "rs485.h"
#include "led.h"
#include "bacnet.h"

#include "bit-array.h"

/* MS/TP port */
static struct mstp_port_struct_t MSTP_Port;
static struct dlmstp_rs485_driver RS485_Driver =
{
    .init = rs485_init,
    .send = rs485_bytes_send,
    .read = rs485_byte_available,
    .transmitting = rs485_rts_enabled,
    .baud_rate = rs485_baud_rate,
    .baud_rate_set = rs485_baud_rate_set,
    .silence_milliseconds = rs485_silence_milliseconds,
    .silence_reset = rs485_silence_reset
};
static struct dlmstp_user_data_t MSTP_User_Data;
static uint8_t Input_Buffer[DLMSTP_MPDU_MAX];
static uint8_t Output_Buffer[DLMSTP_MPDU_MAX];

void BacnetTask( void *argument )
{
	struct mstimer Blink_Timer;

	vTaskDelay(100);

	mstimer_init();
	led_init();
	rs485_init();
	mstimer_set(&Blink_Timer, 500);
	/* FIXME: get the device ID from EEPROM */
	Device_Set_Object_Instance_Number(103);
	/* seed stdlib rand() with device-id to get pseudo consistent
	 * zero-config poll slot, or use hardware RNG to get a more random slot
	 */
#ifdef BACNET_ZERO_CONFIG_RNG_HARDWARE
	/* enable the random number generator hardware */
	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);
	RNG_Cmd(ENABLE);
	while (RNG_GetFlagStatus(RNG_FLAG_DRDY) == RESET)
	{
		/* wait for 32-bit random number to generate */
	}
srand(RNG_GetRandomNumber());
#else
	srand(Device_Object_Instance_Number());
#endif
    /* initialize the Device UUID from rand() */
    Device_UUID_Init();
    Device_UUID_Get(MSTP_Port.UUID, sizeof(MSTP_Port.UUID));
    /* initialize MSTP datalink layer */
    MSTP_Port.Nmax_info_frames = DLMSTP_MAX_INFO_FRAMES;
    MSTP_Port.Nmax_master = DLMSTP_MAX_MASTER;
    MSTP_Port.InputBuffer = Input_Buffer;
    MSTP_Port.InputBufferSize = sizeof(Input_Buffer);
    MSTP_Port.OutputBuffer = Output_Buffer;
    MSTP_Port.OutputBufferSize = sizeof(Output_Buffer);
    /* user data */
    MSTP_Port.ZeroConfigEnabled = true;
    MSTP_Port.SlaveNodeEnabled = false;
    MSTP_User_Data.RS485_Driver = &RS485_Driver;
    MSTP_Port.UserData = &MSTP_User_Data;
    dlmstp_init((char *)&MSTP_Port);
    if (MSTP_Port.ZeroConfigEnabled) {
        /* set node to monitor address */
        dlmstp_set_mac_address(255);
    } else {
        /* FIXME: get the address from hardware DIP or from EEPROM */
        dlmstp_set_mac_address(1);
    }
    /* FIXME: get the baud rate from hardware DIP or from EEPROM */
    dlmstp_set_baud_rate(DLMSTP_BAUD_RATE_DEFAULT);
    /* initialize application layer*/
    bacnet_init();

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
