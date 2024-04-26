/******************************************************************************/
/* UPBDRIVE Dashboard Firmware                                                */
/* All rights reserverd 2023                                                  */
/******************************************************************************/

#ifndef ACCELEROMETER_TASK_H
#define ACCELEROMETER_TASK_H

#include <common.h>
#include <driver/i2c.h>

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
    * @brief This function will setup the I2C bus for communication with MPU6050
    *
    * @param void
    *
    * @return void
*/
esp_err_t accelerometer_setup(void);

/*
    * @brief This task will read the accelerometer data
    *
    * @param[in] pvParameters IN
    *
    * @return void
*/
void accelerometer_read(void *pvParameters);

#endif /* ACCELEROMETER_TASK_H */