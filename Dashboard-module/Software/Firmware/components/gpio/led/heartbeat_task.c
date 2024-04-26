/******************************************************************************/
/* UPBDRIVE Dashboard Firmware                                                */
/* All rights reserverd 2023                                                  */
/******************************************************************************/

#include <heartbeat_task.h>

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL VARIABLES                                 ////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL FUNCTIONS                                 ////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////                       LOCAL MACROS                                     ////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////                       LOCAL CONSTANTS                                  ////
////////////////////////////////////////////////////////////////////////////////


esp_err_t heartbeat_setup(void)
{
    esp_err_t esp_response = ESP_FAIL;
    gpio_config_t gpio_led_status =
    {
        .pin_bit_mask = (1ULL << GPIO_LED_STATUS),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };

    esp_response = gpio_config(&gpio_led_status);
    return esp_response;
}


void heartbeat(void *pvParameters)
{
    esp_err_t esp_response      = ESP_FAIL;
    uint8_t heartbeat_led_state = 0;

    while(true)
    {
        esp_response = gpio_set_level(GPIO_LED_STATUS, heartbeat_led_state);
        if(esp_response != ESP_OK)
        {
            #ifdef ENABLE_DEBUG_HEARTBEAT
            printf("Failed to set the level of the heartbeat LED\n");
            #endif /* ENABLE_DEBUG_HEARTBEAT */

            vTaskDelay(HEARTBEAT_LED_RATE);
        }

        /* toggle the level of the heartbeat */
        heartbeat_led_state = !heartbeat_led_state;

        /* delay the loop for debugging purposes */
        vTaskDelay(HEARTBEAT_LED_RATE);
    }
}