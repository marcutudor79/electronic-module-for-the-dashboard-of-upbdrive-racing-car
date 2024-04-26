/******************************************************************************/
/* UPBDRIVE ECU-Emulator Firmware                                             */
/* All rights reserverd 2023                                                  */
/******************************************************************************/

#ifndef COMMON_H
#define COMMON_H

#include <stdio.h>

////////////////////////////////////////////////////////////////////////////////
////                      PINS SETTINGS                                     ////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL MACROS                                    ////
////////////////////////////////////////////////////////////////////////////////

/* Pins used in adc module */
#define GPIO_NUM_POTENTIOMETER   (26)
#define GPIO_NUM_BUTTON          (22)
#define GPIO_NUM_HEARTBEAT       PICO_DEFAULT_LED_PIN


/* Get the first 8 bits in uint16_t */
#define GET_MSB(x) (uint8_t)((x >> 8) & 0xFF)

/* Get the last 8 bits in uint16_t */
#define GET_LSB(x) (uint8_t)(x & 0xFF)




#endif /* COMMON_H */