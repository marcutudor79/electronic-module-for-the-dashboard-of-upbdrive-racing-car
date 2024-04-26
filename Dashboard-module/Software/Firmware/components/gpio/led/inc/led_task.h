/******************************************************************************/
/* UPBDRIVE Dashboard Firmware                                                */
/* All rights reserverd 2023                                                  */
/******************************************************************************/
#ifndef SHIFTLIGHT_TASK_H
#define SHIFTLIGHT_TASK_H

#include <common.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <driver/rmt_tx.h>
#include <led_strip_encoder.h>

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL VARIABLES                                 ////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL MACROS                                    ////
////////////////////////////////////////////////////////////////////////////////

#define SHIFTLED_THRESHOLD  (SHIFT_THRESHOLD)

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL CONSTANTS                                 ////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////                       FUNCTION PROTOYPES                               ////
////////////////////////////////////////////////////////////////////////////////

/*
    * @brief This function will setup the shift led and neutral led GPIOs as output
    *
    * @param void
    *
    * @return esp_err_t
*/
esp_err_t shift_neutral_led_setup(void);

/*
    * @brief This function will setup the adafruit shift led strip GPIO as output
    *
    * @param void
    *
    * @return esp_err_t
*/
esp_err_t shift_strip_led_setup(void);

/*
    * @brief This task will update the shift led based on the rpm value at 4Hz
    *        and the neutral led based on the gear value at the same frequency
    *
    * @param[in] pvParameters
    *
    * @return esp_err_t
*/
void shift_neutral_led_update(void *pvParameters);

/*
    * @brief This task will update the shift strip at 30HZ refresh rate
    *
    * @param pvParameters IN
    *
    * @return void
*/
void shift_strip_led_update(void *pvParameters);


#endif /* BUTTON_TASK_H */
