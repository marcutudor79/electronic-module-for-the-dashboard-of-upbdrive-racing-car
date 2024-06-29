/******************************************************************************/
/* UPBDRIVE ECU-Emulator Firmware                                             */
/* All rights reserverd 2023                                                  */
/******************************************************************************/

#ifndef COMMON_H
#define COMMON_H

#include <stdio.h>

////////////////////////////////////////////////////////////////////////////////
////                      ECU EMULATOR SETTINGS                             ////
////////////////////////////////////////////////////////////////////////////////
//#define ENABLE_DEBUG_PRINTF

/* Select the library to be used in order to send / receive can packets */
//#define SUPPORT_CAN2040
#define SUPPORT_MCP2515

/* SPI0 bus speed, do not modify! */
#define SPI0_SPEED          (10000000)

////////////////////////////////////////////////////////////////////////////////
////                      PINS SETTINGS                                     ////
////////////////////////////////////////////////////////////////////////////////

#define GPIO_CS_MCP2515          (17)
#define GPIO_MOSI_MCP2515        (19)
#define GPIO_MISO_MCP2515        (16)
#define GPIO_SCK_MCP2515         (18)

#define GPIO_NUM_POTENTIOMETER      (26)
#define GPIO_NUM_OIL_TEMP           (27)
#define GPIO_NUM_COOLANT_TEMP       (28)
#define GPIO_NUM_GEAR_SHIFT         (22)
#define GPIO_NUM_LOW_BATTERY        (14)
#define GPIO_NUM_LOW_OIL_PRESSUIRE  (15)
#define GPIO_NUM_HEARTBEAT          PICO_DEFAULT_LED_PIN

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL MACROS                                    ////
////////////////////////////////////////////////////////////////////////////////

/* Get the first 8 bits in uint16_t */
#define GET_MSB(x) (uint8_t)((x >> 8) & 0xFF)

/* Get the last 8 bits in uint16_t */
#define GET_LSB(x) (uint8_t)(x & 0xFF)

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL TYPEDEFS                                  ////
////////////////////////////////////////////////////////////////////////////////
/* Structure that is used in dashboard firmware to save the data */
typedef struct display_data_t {

   /*************************************************************
    *                  MAIN DISPLAY PAGE                        *
    *************************************************************/
   /* Expected rpm is between 0 - 12.000 (uint16_t max 65.536) */
   uint16_t rpm;

   /* Current gear is between 0 - 5 (uint8_t max 255) */
   uint8_t current_gear;

   /* Coolant temperature is between 0 - 100 (uint8_t max 255) */
   uint8_t coolant_temperature;

   /* Radiator Fan state is either true for ON or false for OFF */
   uint8_t fan_state;

   /* Oil temperature value is between 0 - 150 (uint8_t max 255) */
   uint8_t oil_temperature;


   /***********************************************************
    *              SECOND DISPLAY PAGE for debug              *
    ***********************************************************/
   /* CAN bus status is 0x00 for ON or 0xFF for OFF */
   uint8_t can_status;

   /* Hybrid system status is 0x00 for OK or 0xFF for FAULT */
   uint8_t hybrid_status;

   /* Safety circuit status is 0x00 for OK or 0xFF for FAULT */
   uint8_t safety_circuit_status;

   /* Oil pressure value is between 0 - 150 (uint8_t max 255) */
   uint8_t oil_pressure;

   /* Raw bps value from the sensor ToDo */
   uint8_t brake_pressure_raw;

   /* Throttle Position Sensor value is between 0 - 100 (uint8_t max 255) */
   uint8_t tps;

   /* Oil temperature value is between 0 - 150 (uint8_t max 255)

      @note: the field oil_temperature is scaled before being sent to MAIN_PAGE
             this field will contain the unscaled version of oil_temperature from
             the CAN message
   */
   uint8_t oil_temperature_2;

   /* Hybrid status value HYBRID_STATE_1 - HYBRID_STATE_11 or HYBRID_STATE_ERROR */
   uint8_t hybrid_selector_value;

   /* Battery voltage is between 0 - 240 (uint8_t max 255) */
   uint8_t battery_voltage;


   /**********************************************************
   *                THIRD DISPLAY PAGE for debug             *
   ***********************************************************/
   /* Lambda value between 0 - 155 (uint8_t max 255) */
   uint8_t lambda;

   /* Manifold Absolute Pressure MAP value 0 - 45 (uint8_t max 255) */
   uint8_t map;

   /* Fuel pressure value is between 0 - 45 (uint8_t max 255) */
   uint8_t fuel_pressure;

   /* EGT values between 0 - 450 (uint8_t max 255) */
   uint16_t egt[4];

   /* Voltage on the PMU input rail 0 - 14 (uint8_t max 255) */
   uint8_t input_voltage_pmu;

   /* Input current from alternator 0 - 10 (uint8_t max 255) */
   uint8_t input_current_altr_pmu;


   /***********************************************************
    *             FOURTH DISPLAY PAGE for debug               *
    ***********************************************************/
   /* Tyre pressure values for each wheel between 0 - 30 bar (uint8_t max 255)*/
   uint8_t tyre_pressure[4];


   /*********************************************************
    *            EXTRA - EMULATION PURPOSES                 *
    ********************************************************/
   /* Store the time from the RTC of the car */
   /* time in seconds */
   uint8_t time_second;

   /* time in minutes */
   uint8_t time_minute;

   /* time in hours */
   uint8_t time_hour;

} display_data_t ;


#endif /* COMMON_H */