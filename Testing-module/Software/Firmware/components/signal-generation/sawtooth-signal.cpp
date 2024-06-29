/******************************************************************************/
/* UPBDRIVE ECU-Emulator Firmware                                             */
/* All rights reserverd 2023                                                  */
/******************************************************************************/

#include "sawtooth-signal.h"

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL VARIABLES                                 ////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL FUNCTIONS                                 ////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////                       LOCAL MACROS                                     ////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////                       LOCAL CONSTANTS                                  ////
////////////////////////////////////////////////////////////////////////////////

void generate_sawtooth(uint8_t* data) {
    *data = *data + 1;

    if (*data > 250) {
        *data = 0;
    }

    return;
}

void fill_with_sawtooth(display_data_t* display_data) {

    generate_sawtooth(&display_data->tps);
    generate_sawtooth(&display_data->fuel_pressure);
    generate_sawtooth(&display_data->lambda);
    generate_sawtooth((uint8_t*)&display_data->egt[0]);
    generate_sawtooth((uint8_t*)&display_data->egt[1]);
    generate_sawtooth((uint8_t*)&display_data->egt[2]);
    generate_sawtooth((uint8_t*)&display_data->egt[3]);
    generate_sawtooth(&display_data->map);
    generate_sawtooth(&display_data->brake_pressure_raw);

    return;
}