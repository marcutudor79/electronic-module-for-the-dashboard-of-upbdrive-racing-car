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

#define LCD_MAIN_PAGE        (0x00)
#define LCD_SECONDARY_PAGE   (0x11)
#define LCD_THIRD_PAGE       (0x22)
#define LCD_FOURTH_PAGE      (0x33)
#define LCD_START_UP_PAGE    (0xAA)

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
