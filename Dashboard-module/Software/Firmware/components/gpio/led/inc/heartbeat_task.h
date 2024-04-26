/******************************************************************************/
/* UPBDRIVE Dashboard Firmware                                                */
/* All rights reserverd 2023                                                  */
/******************************************************************************/
#ifndef HEARTBEAT_TASK_H
#define HEARTBEAT_TASK_H

#include <common.h>
#include <driver/gpio.h>
#include <stdio.h>

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL VARIABLES                                 ////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL MACROS                                    ////
////////////////////////////////////////////////////////////////////////////////

#define HEARTBEAT_LED_RATE  (1000/portTICK_PERIOD_MS)

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL CONSTANTS                                 ////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////                       FUNCTION PROTOYPES                               ////
////////////////////////////////////////////////////////////////////////////////

/*
    * @brief This function will setup the gpio used for heartbeat led as an output
    *
    * @param void
    *
    * @return esp_err_t
*/
esp_err_t heartbeat_setup(void);

/*
    * @brief This task will update the heartbeat 1HZ refresh rate
    *
    * @param[in] pvParameters IN
    *
    * @return void
*/
void heartbeat(void *pvParameters);

#endif /* HEARTBEAT_TASK_H */
