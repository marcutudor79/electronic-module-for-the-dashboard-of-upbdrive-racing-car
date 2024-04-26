/******************************************************************************/
/* UPBDRIVE ECU-Emulator Firmware                                             */
/* All rights reserverd 2023                                                  */
/******************************************************************************/

#include "inc/heartbeat.h"

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

void init_heartbeat(uint8_t* heartbeat_state)
{
    /* initialize the heartbeat led pin */
    gpio_init(GPIO_NUM_HEARTBEAT);
    gpio_set_dir(GPIO_NUM_HEARTBEAT, GPIO_OUT);

    /* set the initial state of the heartbeat_state variable */
    gpio_put(GPIO_NUM_HEARTBEAT, heartbeat_state);
}

void toggle_heartbeat(uint8_t *heartbeat_state)
{
    /* invert the heartbeat_state value such that the LED
       will have an ON-OFF blink */
    *heartbeat_state = !(*heartbeat_state);

    /* change the state of the LED based on the heartbeat_state value */
    gpio_put(GPIO_NUM_HEARTBEAT, *heartbeat_state);
}