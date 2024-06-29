/******************************************************************************/
/* UPBDRIVE ECU-Emulator Firmware                                             */
/* All rights reserverd 2023                                                  */
/******************************************************************************/

/* Include components */
#include "adc.h"
#include "can.h"
#include "heartbeat.h"
#include "sawtooth-signal.h"

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/adc.h"
#include <string.h>

/* Global data structure for display data */
static display_data_t display_data = {0};

int main() {

    /* This 2 values need to be init to nominal
       values in order to avoid dashboard showing error */
    display_data.battery_voltage     = 123U;
    display_data.oil_pressure        = 100U;

    /* This values are emulating the DAT system
        that will send a CAN packet with the time */
    display_data.time_hour = 12U;
    display_data.time_minute = 30U;
    display_data.time_second = 45U;

    /* sawtooth signal variable */
    uint8_t sawtooth = 0U;

    /* set heartbeat led state to OFF */
    uint8_t heartbeat_state = 0U;

    /* initialize usb uart communication */
    stdio_init_all();

    /* initialize adc and shift button */
    init_adc_and_buttons();

    /* initialize the can bus */
    #ifdef SUPPORT_CAN2040
    canbus_setup();
    #endif /* SUPPORT_CAN2040 */

    #ifdef SUPPORT_MCP2515
    mcp2515_setup();
    #endif /* SUPPORT_MCP2515 */

    /* initialize heartbeat led */
    init_heartbeat(&heartbeat_state);

    while(true) {
        /* fill display_data structure */
        read_adc_and_button(&display_data);
        fill_with_sawtooth(&display_data);

        /* send the display_data struct fields via CAN */
        send_can(&display_data);

        /* toggle the heartbeat LED */
        toggle_heartbeat(&heartbeat_state);

        #ifdef ENABLE_DEBUG_PRINTF
        //send via USB UART the data to be sent to CAN bus
        printf("rpm          = %d\n", display_data.rpm);
        printf("oil_temp     = %d\n", display_data.oil_temperature);
        printf("coolant_temp = %d\n", display_data.coolant_temperature);
        printf("gear         = %d\n", display_data.current_gear);
        printf("battery      = %d\n", display_data.battery_voltage);
        printf("oil_pressure = %d\n", display_data.oil_pressure);
        sleep_ms(250);
        #endif /* ENABLE_DEBUG_PRINTF */
    }

    return 0;
}




