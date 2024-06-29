/******************************************************************************/
/* UPBDRIVE ECU-Emulator Firmware                                             */
/* All rights reserverd 2023                                                  */
/******************************************************************************/

#ifndef CAN_H
#define CAN_H

#include "common.h"
#include "can2040.h"
#include "RP2040.h"
#include "pico/stdlib.h"
#include "pico/binary_info.h"

#include "mcp2515.h"

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL VARIABLES                                 ////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL MACROS                                    ////
////////////////////////////////////////////////////////////////////////////////

/*
    * @brief data structure:
    * data[0]: RPM / 100
    * data[1]: CurrGear
    * data[2]: TPS
    * data[3]: Oil pressure
    * data[4]: Water temperature
    * data[5]: Fuel pressure
    * data[6]: Lamda
    * data[7]: IAT
*/
#define CAN_PACKET_ECU_1 (0x3E8)

/*
    * @brief data structure:
    * data[0]: EGT1 temp / 100
    * data[1]: EGT2 temp / 100
    * data[2]: EGT3 temp / 100
    * data[3]: EGT4 temp / 100
    * data[4]: Vehicle speed
    * data[5]: Manifold Air Pressure (MAP)
    * data[6]: BPS raw value
    * data[7]: Oil temperature
*/
#define CAN_PACKET_ECU_2 (0x3E9)

/*
    * @brief data structure:
    * data[0]: LV battery voltage
    * data[1]: Input voltage
    * data[2]: Input current battery
    * data[3]: Input current alternator
*/
#define CAN_PACKET_PMU_1 (0x3ED)

/*
    * @brief data structure:
    * data[4]: Tyre pressure Left-Front
	* data[5]: Tyre pressure Right-Front
	* data[6]: Tyre pressure Left-Rear
	* data[7]: Tyre pressure Right-Rear
*/
#define CAN_PACKET_VDU (0x3EC)

/*
    * @brief data structure:
    * data[0]: Selector value
*/
#define CAN_PACKET_DASH (0x384)

/*
    * @brief data structure:
    * data[0]: HOUR value
    * data[1]: MINUTE value
    * data[2]: SECOND value
*/
#define CAN_PACKET_DAT (0x10)

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL CONSTANTS                                 ////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////                       FUNCTION PROTOTYPES                              ////
////////////////////////////////////////////////////////////////////////////////

//generate and send CAN package
void send_can(display_data_t *data);

//function to setup the CAN bus
void canbus_setup();
void mcp2515_setup();

#endif /* CAN_H */


