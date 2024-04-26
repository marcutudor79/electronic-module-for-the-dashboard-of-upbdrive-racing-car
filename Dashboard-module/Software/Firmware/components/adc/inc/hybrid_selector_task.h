/******************************************************************************/
/* UPBDRIVE Dashboard Firmware                                                */
/* All rights reserverd 2023                                                  */
/******************************************************************************/

#ifndef HYBRID_SELECTOR_TASK_H
#define HYBRID_SELECTOR_TASK_H

#include <common.h>
#include <esp_adc/adc_oneshot.h>

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
    * @brief This function setups the gpio used for the hybrid selector as output
    *        and the ADC_UNIT_1 in oneshot mode with CHANNEL 6 attenuation 11db
    *
    * @note ADC_UNIT_2 cannot be used because it is used by Wi-Fi
    *
    * @param void
    *
    * @return esp_err_t
*/
esp_err_t hybrid_selector_setup(void);


/*
    * @brief This task will read the hybrid selector
    *
    * @param[in] pvParameters IN
    *
    * @return void
*/
void hybrid_selector_read(void *pvParameters);

#endif /* HYBRID_SELECTOR_TASK_H */