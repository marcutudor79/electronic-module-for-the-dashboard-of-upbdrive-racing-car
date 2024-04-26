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

#include <common.h>
#include <stdio.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL VARIABLES                                 ////
////////////////////////////////////////////////////////////////////////////////

// obsolete: static Adafruit_NeoPixel pixels = Adafruit_NeoPixel(8, GPIO_NUM_12, NEO_GRB + NEO_KHZ800);

/* Global semaphore for display_data struct */
SemaphoreHandle_t xSemaphore_display_data;

display_data_t display_data =
{
    /* Initialize the structure members with 0 */
    0
};

status_firmware_t general_status =
{
    /* Pointer to the display data struct */
    &display_data
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

    /* setup can bus */
    esp_response = can_setup(can_mode, &can_speed, &packet_filter_config);
    if (esp_response != ESP_OK)
    {
        return esp_response;
    }

    /* setup buttons */
    esp_response = buttons_setup();
    if (esp_response != ESP_OK)
    {
        return esp_response;
    }

    /* setup led status */
    esp_response = heartbeat_setup();
    if (esp_response != ESP_OK)
    {
        return esp_response;
    }

    /* setup the lcd display */
    esp_response = lcd_setup();
    if (esp_response != ESP_OK)
    {
        return esp_response;
    }

    /* setup the Semaphore for display data */
    xSemaphore_display_data                = xSemaphoreCreateBinary();
    general_status.xSemaphore_display_data = xSemaphore_display_data;

    /* Because for some fcking reason it is initialized to 0 instead of 1*/
    xSemaphoreGive(xSemaphore_display_data);

    /* setup the shift and neutral led */
    esp_response = shift_neutral_led_setup();
    if (esp_response != ESP_OK)
    {
        return esp_response;
    }

    /* setup shift strip */
    esp_response = shift_strip_led_setup();
    if (esp_response != ESP_OK)
    {
        return esp_response;
    }

    /* setup the accelerometer module */
    esp_response = accelerometer_setup();
    if (esp_response != ESP_OK)
    {
        return esp_response;
    }

    /* setup the hybrid selector */
    esp_response = hybrid_selector_setup();
    if (esp_response != ESP_OK)
    {
        return esp_response;
    }

    /* setup the sdcard module */
    esp_response = sdcard_setup();
    if (esp_response != ESP_OK)
    {
        return esp_response;
    }

    /* setup wifi */
    wifi_setup();

    /* setup http server */
    esp_response = http_server_setup();
    if (esp_response != ESP_OK)
    {
        return esp_response;
    }

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
    xTaskCreatePinnedToCore(can_read,             "can_read",             2048, &general_status,                  2, NULL, 0);
    xTaskCreatePinnedToCore(can_send,             "can_send",             2048, &general_status,                  2, NULL, 0);
    xTaskCreatePinnedToCore(accelerometer_read,   "accelerometer_read",   2048, &general_status,                  1, NULL, 0);
    xTaskCreatePinnedToCore(hybrid_selector_read, "hybrid_selector_read", 2048, &general_status,                  1, NULL, 0);

    /* CORE 1 tasks */
    xTaskCreatePinnedToCore(shift_neutral_led_update, "shift_neutral_led_update",   1024, &general_status,                  1, NULL, 1);
    xTaskCreatePinnedToCore(shift_strip_led_update,   "shift_strip_led_update",     4096, &general_status,                  1, NULL, 1);
    xTaskCreatePinnedToCore(lcd_update,               "lcd_update",                 4096, &general_status,                  2, NULL, 1);
    xTaskCreatePinnedToCore(sdcard_write,             "sdcard_write",               4096, &general_status,                  1, NULL, 1);

    /* DEBUG tasks */
    #ifdef ENABLE_DEBUG_BUTTONS
    xTaskCreatePinnedToCore(buttons_read,         "buttons_task",         1024, NULL,                             0, NULL, 0);
    #endif /* ENABLE_DEBUG_BUTTONS */

}
