/******************************************************************************/
/* UPBDRIVE Dashboard Firmware                                                */
/* All rights reserverd 2023                                                  */
/******************************************************************************/

#ifndef BUTTONS_TASK_H
#define BUTTONS_TASK_H

#include <common.h>


////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL VARIABLES                                 ////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL MACROS                                    ////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL CONSTANTS                                 ////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////                       FUNCTION PROTOYPES                               ////
////////////////////////////////////////////////////////////////////////////////

/*
    * @brief This function will setup the LCD change page button and ventilator ON/OFF button
    *
    * @param void
    *
    * @return esp_response

*/
esp_err_t buttons_setup(void);

/*
    * @brief This task will poll the LCD change page button and ventilator ON/OFF button at BUTTON_POLLING_RATE refresh
    *        rate
    *
    * @param[in] pvParameters IN
    *
    * @return void
*/
void buttons_read(void *pvParameters);


#endif /* BUTTONS_TASK_H */