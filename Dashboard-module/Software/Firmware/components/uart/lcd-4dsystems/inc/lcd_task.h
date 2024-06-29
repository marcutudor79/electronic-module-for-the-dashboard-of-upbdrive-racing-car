/******************************************************************************/
/* UPBDRIVE Dashboard Firmware                                                */
/* All rights reserverd 2023                                                  */
/******************************************************************************/
#ifndef LCD_TASK_H
#define LCD_TASK_H

#include <common.h>
#include <driver/uart.h>

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL VARIABLES                                 ////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL MACROS                                    ////
////////////////////////////////////////////////////////////////////////////////

#define LCD_START_UP_PAGE           (0x00)
#define LCD_SECONDARY_PAGE          (0x01)
#define LCD_THIRD_PAGE              (0x02)
#define LCD_MAIN_PAGE               (0x03)
#define LCD_FOURTH_PAGE             (0x04)
#define LCD_LOW_12V_BATTERY_PAGE    (0x05)
#define LCD_LOW_OIL_PRESSURE_PAGE   (0x06)
#define LCD_OVERHEAT_PAGE           (0x07)

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL CONSTANTS                                 ////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////                       FUNCTION PROTOYPES                               ////
////////////////////////////////////////////////////////////////////////////////

/*
    * @brief This function will setup the lcd display from 4DSystems
    *
    * @param[in] void
    *
    * @return esp_err_t
*/
esp_err_t lcd_setup(void);

/*
    * @brief This task will update the display objects @ LCD_REFRESH_RATE frequency
    *
    * @param[in] pvParameters
    *
    * @return void
*/
void lcd_update(void *pvParameters);

#endif /* LCD_TASK_H */
