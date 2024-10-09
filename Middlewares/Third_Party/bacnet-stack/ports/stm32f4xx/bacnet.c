/**************************************************************************
 *
 * Copyright (C) 2011 Steve Karg <skarg@users.sourceforge.net>
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
 *
 *********************************************************************/
#include <stdint.h>
#include <stdbool.h>
/* hardware layer includes */
#include "bacnet/basic/sys/mstimer.h"
#include "rs485.h"
/* BACnet Stack includes */
#include "bacnet/datalink/datalink.h"
#include "bacnet/npdu.h"
#include "bacnet/basic/services.h"
#include "bacnet/basic/services.h"
#include "bacnet/basic/tsm/tsm.h"
#include "bacnet/dcc.h"
#include "bacnet/iam.h"
/* BACnet objects */
#include "bacnet/basic/object/device.h"
#include "bacnet/basic/object/ai.h"
#include "bacnet/basic/object/ao.h"
#include "bacnet/basic/object/av.h"
#include "bacnet/basic/object/bi.h"
#include "bacnet/basic/object/bo.h"
#include "bacnet/basic/object/bv.h"
#include "bacnet/basic/object/ms-input.h"
#include "bacnet/basic/object/mso.h"
#include "bacnet/basic/object/msv.h"
/* me */
#include "bacnet.h"

/* timer for device communications control */
static struct mstimer DCC_Timer;
#define DCC_CYCLE_SECONDS 1

#include "bacnet/datalink/mstp.h"
static struct mstp_port_struct_t MSTP_Port;
static struct dlmstp_user_data_t UserData;
static struct dlmstp_rs485_driver rs485_driver;

static uint8_t InputBuffer[1000];
static uint8_t OutputBuffer[1000];

#ifndef BACNET_ANALOG_INPUTS_MAX
#define BACNET_ANALOG_INPUTS_MAX 12
#endif
#ifndef BACNET_ANALOG_OUTPUTS_MAX
#define BACNET_ANALOG_OUTPUTS_MAX 12
#endif
#ifndef BACNET_ANALOG_VALUES_MAX
#define BACNET_ANALOG_VALUES_MAX 12
#endif
#ifndef BACNET_BINARY_INPUTS_MAX
#define BACNET_BINARY_INPUTS_MAX 12
#endif
#ifndef BACNET_BINARY_OUTPUTS_MAX
#define BACNET_BINARY_OUTPUTS_MAX 12
#endif
#ifndef BACNET_BINARY_VALUES_MAX
#define BACNET_BINARY_VALUES_MAX 12
#endif
#ifndef BACNET_MULTISTATE_INPUTS_MAX
#define BACNET_MULTISTATE_INPUTS_MAX 12
#endif
#ifndef BACNET_MULTISTATE_OUTPUTS_MAX
#define BACNET_MULTISTATE_OUTPUTS_MAX 12
#endif
#ifndef BACNET_MULTISTATE_VALUES_MAX
#define BACNET_MULTISTATE_VALUES_MAX 12
#endif

void bacnet_init(void)
{
	uint32_t instance;

    dlmstp_set_mac_address(2);
    dlmstp_set_max_master(127);
    /* initialize datalink layer */
    rs485_driver.init = rs485_init;
	rs485_driver.send = rs485_bytes_send;
    rs485_driver.read = rs485_byte_available;
    rs485_driver.transmitting = rs485_rts_enabled;
	rs485_driver.baud_rate = rs485_baud_rate;
	rs485_driver.baud_rate_set = rs485_baud_rate_set;
	rs485_driver.silence_milliseconds = rs485_silence_milliseconds;
	rs485_driver.silence_reset = rs485_silence_reset;

	UserData.RS485_Driver = (void *)&rs485_driver;

    MSTP_Port.Nmax_info_frames = DLMSTP_MAX_INFO_FRAMES;
    MSTP_Port.Nmax_master = DLMSTP_MAX_MASTER;
    MSTP_Port.ZeroConfigEnabled = true;
    MSTP_Port.SlaveNodeEnabled = false;

    MSTP_Port.UserData = (void *)&UserData;
    MSTP_Port.InputBuffer = InputBuffer;
    MSTP_Port.OutputBuffer = OutputBuffer;
    MSTP_Port.InputBufferSize = sizeof(InputBuffer) - 1;
    MSTP_Port.OutputBufferSize = sizeof(OutputBuffer) - 1;

    dlmstp_init((char *)&MSTP_Port);
    /* initialize objects */
    Device_Init(NULL);

    for (instance = 1; instance <= BACNET_ANALOG_INPUTS_MAX; instance++) {
        Analog_Input_Create(instance);
    }
    for (instance = 1; instance <= BACNET_ANALOG_OUTPUTS_MAX; instance++) {
        Analog_Output_Create(instance);
    }
    for (instance = 1; instance <= BACNET_ANALOG_VALUES_MAX; instance++) {
        Analog_Value_Create(instance);
    }
    for (instance = 1; instance <= BACNET_BINARY_INPUTS_MAX; instance++) {
        Binary_Input_Create(instance);
    }
    for (instance = 1; instance <= BACNET_BINARY_OUTPUTS_MAX; instance++) {
        Binary_Output_Create(instance);
    }
    for (instance = 1; instance <= BACNET_BINARY_VALUES_MAX; instance++) {
        Binary_Value_Create(instance);
    }
    for (instance = 1; instance <= BACNET_MULTISTATE_INPUTS_MAX; instance++) {
        Multistate_Input_Create(instance);
    }
    for (instance = 1; instance <= BACNET_MULTISTATE_OUTPUTS_MAX; instance++) {
        Multistate_Output_Create(instance);
    }
    for (instance = 1; instance <= BACNET_MULTISTATE_VALUES_MAX; instance++) {
        Multistate_Value_Create(instance);
    }

    /* set up our confirmed service unrecognized service handler - required! */
    apdu_set_unrecognized_service_handler_handler(handler_unrecognized_service);
    /* we need to handle who-is to support dynamic device binding */
    apdu_set_unconfirmed_handler(SERVICE_UNCONFIRMED_WHO_IS, handler_who_is);
    apdu_set_unconfirmed_handler(SERVICE_UNCONFIRMED_WHO_HAS, handler_who_has);
    /* Set the handlers for any confirmed services that we support. */
    /* We must implement read property - it's required! */
    apdu_set_confirmed_handler(
        SERVICE_CONFIRMED_READ_PROPERTY, handler_read_property);
    apdu_set_confirmed_handler(
        SERVICE_CONFIRMED_READ_PROP_MULTIPLE, handler_read_property_multiple);
    apdu_set_confirmed_handler(
        SERVICE_CONFIRMED_REINITIALIZE_DEVICE, handler_reinitialize_device);
    apdu_set_confirmed_handler(
        SERVICE_CONFIRMED_WRITE_PROPERTY, handler_write_property);
    /* handle communication so we can shutup when asked */
    apdu_set_confirmed_handler(SERVICE_CONFIRMED_DEVICE_COMMUNICATION_CONTROL,
        handler_device_communication_control);
    /* start the cyclic 1 second timer for DCC */
    mstimer_set(&DCC_Timer, DCC_CYCLE_SECONDS * 1000);
    /* Hello World! */
    Send_I_Am(&Handler_Transmit_Buffer[0]);
}

static uint8_t PDUBuffer[MAX_MPDU];
void bacnet_task(void)
{
    uint16_t pdu_len;
    BACNET_ADDRESS src; /* source address */

    /* handle the communication timer */
    if (mstimer_expired(&DCC_Timer)) {
        mstimer_reset(&DCC_Timer);
        dcc_timer_seconds(DCC_CYCLE_SECONDS);
    }
    /* handle the messaging */
    pdu_len = dlmstp_receive(&src, &PDUBuffer[0], sizeof(PDUBuffer), 0);
    if (pdu_len) {
        npdu_handler(&src, &PDUBuffer[0], pdu_len);
    }
}
