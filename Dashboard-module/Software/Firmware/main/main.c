/******************************************************************************/
/* UPBDRIVE Dashboard Firmware                                                */
/* All rights reserverd 2023                                                  */
/******************************************************************************/

/* Include tasks */
#include <lcd_task.h>
#include <led_task.h>
#include <heartbeat_task.h>
#include <can_task.h>
#include <accelerometer_task.h>
#include <hybrid_selector_task.h>
#include <sdcard_task.h>
#include <buttons_task.h>
#include <wifi_task.h>
#include <timer_task.h>

#include <common.h>
#include <stdio.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL VARIABLES                                 ////
////////////////////////////////////////////////////////////////////////////////

/* Global semaphore for display_data struct */
SemaphoreHandle_t xSemaphore_display_data;

display_data_t display_data =
{
    /* Initialize the structure members with 0 */
    0
};

status_firmware_t general_status =
{
    /* Initialize the structure members with 0 */
    0
};

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL FUNCTIONS                                 ////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////                       LOCAL MACROS                                     ////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////                       LOCAL CONSTANTS                                  ////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////                       LOCAL FUNCTIONS                                 ////
////////////////////////////////////////////////////////////////////////////////

esp_err_t dashboard_init()
{
    esp_err_t esp_response                    = ESP_FAIL;
    twai_mode_t can_mode                      = TWAI_MODE_NORMAL;
    #if (CAN_SPEED == 250)
    twai_timing_config_t can_speed            = TWAI_TIMING_CONFIG_250KBITS();
    #elif (CAN_SPEED == 500)
    twai_timing_config_t can_speed            = {.clk_src = TWAI_CLK_SRC_APB, .quanta_resolution_hz = 8000000, .brp = 10, .tseg_1 = 13, .tseg_2 = 2, .sjw = 1, .triple_sampling = false};
    #elif (CAN_SPEED == 800)
    twai_timing_config_t can_speed            = TWAI_TIMING_CONFIG_800KBITS();
    #elif (CAN_SPEED == 1000)
    twai_timing_config_t can_speed            = TWAI_TIMING_CONFIG_1MBITS();
    #endif /* CAN_SPEED */
    twai_filter_config_t packet_filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    general_status.display_data               = &display_data;
    general_status.time_second                = 99U;
    general_status.time_minute                = 99U;
    general_status.time_hour                  = 99U;
    general_status.sdcard_logging             = false;

    /* setup the lcd display */
    ESP_ERROR_CHECK(lcd_setup());

    /* setup can bus */
    ESP_ERROR_CHECK(can_setup(can_mode, &can_speed, &packet_filter_config));

    /* setup buttons */
    ESP_ERROR_CHECK(buttons_setup());

    /* setup led status */
    ESP_ERROR_CHECK(heartbeat_setup());

    /* setup the Semaphore for display data */
    xSemaphore_display_data                = xSemaphoreCreateBinary();
    general_status.xSemaphore_display_data = xSemaphore_display_data;

    /* Because for some reason it is initialized to 0 instead of 1*/
    xSemaphoreGive(xSemaphore_display_data);

    /* Init RTC data in general_status with 99 which is an imposible value */

    /* setup the shift and neutral led */
    ESP_ERROR_CHECK(shift_neutral_led_setup());

    /* setup shift strip */
    ESP_ERROR_CHECK(shift_strip_led_setup());

    /* setup the accelerometer module */
    ESP_ERROR_CHECK(accelerometer_setup());

    /* setup the hybrid selector */
    ESP_ERROR_CHECK(hybrid_selector_setup());

    /* setup the sdcard module */
    ESP_ERROR_CHECK(sdcard_setup());

    /* setup wifi */
    wifi_setup();

    /* setup http server */
    ESP_ERROR_CHECK(http_server_setup());

    /* ToDo override time check */
    general_status.time_second = 0;
    general_status.time_minute = 30;
    general_status.time_hour   = 12;

    /* setup timer for data logging - every second increments the time value
       stored in general_status struct */
    ESP_ERROR_CHECK(timer_setup());

    esp_response = ESP_OK;
    return esp_response;
}


void app_main(void)
{
    esp_err_t esp_response = ESP_FAIL;
    esp_response = dashboard_init();

    if (esp_response != ESP_OK)
    {
        while(true)
        {
            printf("Dashboard failed to initialize\n");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            /* Infinite loop caused by dash perpiherals
                init failure */
        }
    }

    /* CORE 0 tasks*/
    xTaskCreatePinnedToCore(heartbeat,            "heartbeat",            1024, NULL,                             0, NULL, 0);
    xTaskCreatePinnedToCore(can_read,             "can_read",             2048, &general_status,                  3, NULL, 0);
    xTaskCreatePinnedToCore(can_send,             "can_send",             2048, &general_status,                  3, NULL, 0);
    xTaskCreatePinnedToCore(accelerometer_read,   "accelerometer_read",   2048, &general_status,                  1, NULL, 0);
    xTaskCreatePinnedToCore(hybrid_selector_read, "hybrid_selector_read", 2048, &general_status,                  2, NULL, 0);

    /* CORE 1 tasks */
    xTaskCreatePinnedToCore(neutral_led_update,       "shift_neutral_led_update",   1024, &general_status,                  0, NULL, 1);
    xTaskCreatePinnedToCore(shift_strip_led_update,   "shift_strip_led_update",     4096, &general_status,                  1, NULL, 1);
    xTaskCreatePinnedToCore(lcd_update,               "lcd_update",                 4096, &general_status,                  3, NULL, 1);
    xTaskCreatePinnedToCore(sdcard_write,             "sdcard_write",               4096, &general_status,                  2, NULL, 1);

    /* DEBUG tasks */
    #ifdef ENABLE_DEBUG_BUTTONS
    xTaskCreatePinnedToCore(buttons_read,         "buttons_task",         1024, NULL,                             0, NULL, 0);
    #endif /* ENABLE_DEBUG_BUTTONS */

}
