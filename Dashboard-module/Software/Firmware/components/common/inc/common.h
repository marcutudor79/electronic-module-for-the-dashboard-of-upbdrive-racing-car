/******************************************************************************/
/* UPBDRIVE Dashboard Firmware                                                */
/* All rights reserverd 2023                                                  */
/******************************************************************************/
#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/timers.h>
#include <driver/gpio.h>
#include <esp_log.h>

////////////////////////////////////////////////////////////////////////////////
////                               DEBUG                                    ////
////////////////////////////////////////////////////////////////////////////////
/* By uncommenting one macro, debug console output will be enabled for the selected
   functionality */
//#define ENABLE_DEBUG_ACCELEROMETER

//#define ENABLE_DEBUG_BUTTONS
/* Button polling rate, update at 60hz just for debug */
#define BUTTON_POLLING_RATE  (17/portTICK_PERIOD_MS)

//#define ENABLE_CAN_READ_DEBUG
//#define ENABLE_CAN_SEND_DEBUG
//#define ENABLE_DEBUG_DISPLAY
//#define ENABLE_DEBUG_HEARTBEAT
//#define ENABLE_DEBUG_HYBRID_SELECTOR
//#define ENABLE_DEBUG_SDCARD

////////////////////////////////////////////////////////////////////////////////
////                            CAR SETTINGS                                ////
////////////////////////////////////////////////////////////////////////////////

/*  @brief: Set the redline of the car
    @note:  possible values are 0 - 12000
*/
#define SHIFT_THRESHOLD      (10000)

/* @brief: Set the gear in which the car is considered to be in neutral
   @note: possible values: 0 - 4 */
#define NEUTRAL_GEAR        (0)

/* @brief: Set the display data transmission rate,
   @note: possible values are:
    60 HZ: 17/portTICK_PERIOD_MS
    30 HZ: 34/portTICK_PERIOD_MS
*/
#define LCD_REFRESH_RATE     (17/portTICK_PERIOD_MS)

/* @brief: Set the start-up screen delay in ms
   @note: possible values are:
   0   seconds:     0/portTICK_PERIOD_MS
   2.5 seconds:     2500/portTICK_PERIOD_MS
   5   seconds:     5000/portTICK_PERIOD_MS
   7.5 seconds:     7500/portTICK_PERIOD_MS
   10  seconds:     10000/portTICK_PERIOD_MS
*/
#define LCD_START_UP_SCREEN_DELAY (5000/portTICK_PERIOD_MS)

/* @brief: Set the baudrate of the lcd screen in bps, keep in mind that the
           baudrate must be the same with the one set in the 4d systems workshop
           project
   @note: possible values are:
   9600 bps:     9600
   115200 bps:   115200
*/
#define LCD_BAUDRATE         (115200)

/* @brief: Hybrid selector polling rate, note that this polling rate is divided
           by the number of samples of the moving average filter in order to obtain
           the actual value renewal rate
   @note: possible values are:
   100 HZ:  10/portTICK_PERIOD_MS
   60  HZ:  17/portTICK_PERIOD_MS
   30  HZ:  34/portTICK_PERIOD_MS
*/
#define HYBRID_SELECTOR_POLLING_RATE (10/portTICK_PERIOD_MS)

/* No of samples of the moving average filter */
#define HYBRID_SELECTOR_MA_FILTER_SAMPLES (50)

/* @brief: Accelerometer polling rate
   @note: possible values are:
   100 HZ:  10/portTICK_PERIOD_MS
   60  HZ:  17/portTICK_PERIOD_MS
   30  HZ:  34/portTICK_PERIOD_MS
*/
#define MPU6050_POLLING_RATE         (10/portTICK_PERIOD_MS)

/* @brief: Set the maximum value in G for the accelerometer reading
   @note: possible value are:
          2,
          4,
          8 or
          16
*/
#define MPU6050_ACCEL_SCALE          (2)
/* DO NOT CHANGE MPU6050_AXIS_NUM */
#define MPU6050_AXIS_NUM             (3)

/* @brief: Set the blink period for the SHIFT LEDs on the dashboard
   @note: possible value are:
   1  HZ:   1000/portTICK_PERIOD_MS
   5  HZ:   200/portTICK_PERIOD_MS
   10 HZ:   100/portTICK_PERIOD_MS
*/
#define SHIFTLED_RATE         (250/portTICK_PERIOD_MS)

/* @brief: Set the refresh rate for the SHIFT STRIP on the dashboard
   @note: possible value are:
   10  HZ:   100/portTICK_PERIOD_MS
   15  HZ:   67/portTICK_PERIOD_MS
   20  Hz:   50/portTICK_PERIOD_MS
   30  HZ:   34/portTICK_PERIOD_MS
*/
#define SHIFTLED_STRIP_RATE   (100/portTICK_PERIOD_MS)

/* @brief: Set the can bus speed in kpbs
   @note: possible value are:
          250,
          500,
          800 or
          1000
*/
#define CAN_SPEED           (500)

/* @brief: Set the can packet transmit rate of the dashboard
   @note: possible value are:
   100 HZ:  10/portTICK_PERIOD_MS
   60  HZ:  17/portTICK_PERIOD_MS
   30  HZ:  34/portTICK_PERIOD_MS
*/
#define CAN_TRANSMIT_RATE     (500/portTICK_PERIOD_MS)

/* @brief: Set the log frequency on the sdcard of the display_data_t struct
   @note: possible value are:
   60  HZ:  17/portTICK_PERIOD_MS
   30  HZ:  34/portTICK_PERIOD_MS
   15  HZ:  58/portTICK_PERIOD_MS
   1   HZ:  1000/portTICK_PERIOD_MS
*/
#define SDCARD_LOGGING_RATE   (1000/portTICK_PERIOD_MS)

/* @brief: Set the SSID of the wifi network from dashboard's AP
   @note: possible value are:
   "dashboard_dr-05"
*/
#define ESP_WIFI_SSID "dashboard_dr-05"

/* @brief: Set the password of the wifi network from dashboard's AP
   @note: possible value are:
   "FlatOut5"
*/
#define ESP_WIFI_PASS "FlatOut5"

////////////////////////////////////////////////////////////////////////////////
////                           GPIO PINS USAGE                              ////
////////////////////////////////////////////////////////////////////////////////

/* Define the pins used in the firmware */
#define GPIO_LED_STATUS         GPIO_NUM_32
#define GPIO_LED_SHIFT          GPIO_NUM_25
#define GPIO_LED_NEUTRAL        GPIO_NUM_26
#define GPIO_LED_STRIP          GPIO_NUM_33

#define GPIO_LCD_RX             GPIO_NUM_16
#define GPIO_LCD_TX             GPIO_NUM_17
#define GPIO_LCD_RST            GPIO_NUM_18 /* ToDo pull up in code */

#define GPIO_SDCARD_MISO        GPIO_NUM_4
#define GPIO_SDCARD_MOSI        GPIO_NUM_15
#define GPIO_SDCARD_CLK         GPIO_NUM_14
#define GPIO_SDCARD_CS          GPIO_NUM_13

#define GPIO_ACCEL_SDA          GPIO_NUM_21
#define GPIO_ACCEL_SCL          GPIO_NUM_22

#define GPIO_CAN_RX             GPIO_NUM_19
#define GPIO_CAN_TX             GPIO_NUM_27

#define GPIO_BUTTON_RAD_FAN     GPIO_NUM_23
#define GPIO_BUTTON_LCD_PAGE    GPIO_NUM_12
#define GPIO_BUTTON_SAFETY_CIRC GPIO_NUM_35

/* GPIO pin for the hybrid selector */
#if 0 // do not compile this code, only for documentation purposes
#define GPIO_HYBRID_SELECTOR  IS  ADC_CHANNEL_6  ON GPIO_NUM_34
#endif

////////////////////////////////////////////////////////////////////////////////
////                          GLOBAL STRUCTURE TYPES                        ////
////////////////////////////////////////////////////////////////////////////////
/*
    * @brief: This structure contains the data that is displayed on the lcd screen
    *         as well as the data that is logged on the sdcard.
    * @note: The data is real-time, meaning that all of it, besides the accelerometer
    *        and current_page fields, the rest of the data is updated at the
    *        CAN_POLLING_RATE frequency. The accelerometer data is updated at
    *        the MPU6050_POLLING_RATE frequency.
    *
    *        This approach has data loss if the frequency of the SDCARD logging rate is
    *        lower than CAN_POLLING_RATE or MPU6050_POLLING_RATE.
    *
*/
typedef struct display_data_t {

   /* Keeps track of the current_page shown on the display */
   uint8_t current_page;


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


   /**********************************************************
    *                    NOT IN A PAGE                        *
    ***********************************************************/

   /* Raw accelerometer value on X, Y and Z, note that the max value is
      adjusted considering the MPU6050_ACCEL_SCALE macro. The formula is
      ((accelerometer[i] / 2^15) * MPU6050_ACCEL_SCALE). It is divided to
      2^15 since the values are integer ones on 16 bit, so the max value of
      int16_t is 2^15 */
   int16_t accelerometer[MPU6050_AXIS_NUM];

   /* Intake Air Temperature (IAT) is between 0 - 100 (uint8_t max 255) */
   uint8_t iat;

   /* This is a byte dedicated to error flags.
   Bits in error flags:
   [0]: coolant_temperature higher than 100 degrees
   [1]: oil_temperature higher than 150 degrees
   [2]: low oil pressure error
   [3]: hybrid_status unknown
   [4]: safety_circuit_status unknown
   [5]: can_status unknown
   [6]: tps application is higher than 100 %
   */
   uint8_t error_flags;
} display_data_t ;

/*
    * @brief This structure contains pointers to other relevant data structures
    *        that are used in the firmware as well as to the Semaphores that
    *        are used to handle concurrent access to them.
    *
    * @param uint8_t display_page_num: number of the display page
    *
*/
typedef struct status_firmware_t {

    /* pointer to the structure containing all the data to be
       send to the display and logged on the sdcard */
    display_data_t *display_data;

    /* gloabl semaphore handle for the display data struct that will handle
       concurrent access requests from the:
       - can_send   task
       - lcd_update task
       - can_read   task
       - sdcard_write task
       - shift_neutral_led_update task
       - shift_strip_led_update task
       - hybrid_selector_read task
       - acceleromter_read task */
    SemaphoreHandle_t xSemaphore_display_data;

    /* flag that signals a change_page request. It is handled in lcd_update task */
    uint8_t signal_change_page;

} status_firmware_t ;

#endif /* COMMON_TYPES_H */