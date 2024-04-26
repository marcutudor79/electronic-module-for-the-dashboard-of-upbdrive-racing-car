/******************************************************************************/
/* UPBDRIVE ECU-emulator Firmware                                             */
/* All rights reserverd 2023                                                  */
/******************************************************************************/

/******************************************************************************/
/* UPBDRIVE ECU-Emulator Firmware                                             */
/* All rights reserverd 2023                                                  */
/******************************************************************************/

/* Include components */
#include "../components/adc/inc/adc.h"
#include "../components/can/inc/can.h"
#include "../components/heartbeat/inc/heartbeat.h"
#include "../components/signal-generation/inc/random-signal.h"
#include "../components/signal-generation/inc/rpm.h"
#include "../components/signal-generation/inc/sawtooth-signal.h"

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/adc.h"
#include <string.h>


int main() {

    /* declare data array, see can */
    uint8_t data[20];

    /* sawtooth signal variable */
    uint8_t sawtooth = 0U;

    /* set heartbeat led state to OFF */
    uint8_t heartbeat_state = 0U;

    /* initialize usb uart communication */
    stdio_init_all();

    /* initialize adc and shift button */
    init_adc_and_buttons();

    /* initialize the can bus */
    //canbus_setup();
    mcp2515_setup();

    struct can_frame rx;

    /* initialize heartbeat led */
    init_heartbeat(&heartbeat_state);

    while(true) {

        read_adc_and_button(&data[0]);
        sawtooth = generate_sawtooth(sawtooth);
        generate_rpm(&data[0]);

        for (uint8_t i = 4; i < 20; i++)
        {
            data[i] = sawtooth;
        }

        send_can(&data[0]);

        //send via USB acceleration and suspension position transformed from raw to %
        //printf("rpm  = %d\n", (((*data << 8) & 0xFF00) + *(data + 1)));
        //printf("gear = %d\n", *(data + 2));

        /* toggle the heartbeat LED */
        toggle_heartbeat(&heartbeat_state);

    }

    return 0;
}




