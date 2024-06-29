/******************************************************************************/
/* UPBDRIVE Dashboard Firmware                                                */
/* All rights reserverd 2023                                                  */
/******************************************************************************/

#include <led_task.h>

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL VARIABLES                                 ////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL FUNCTIONS                                 ////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////                       LOCAL MACROS                                     ////
////////////////////////////////////////////////////////////////////////////////

#define RMT_LED_STRIP_RESOLUTION_HZ (10000000) // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)

#define STRIP_LED_NUMBER                (8)
#define EXAMPLE_CHASE_SPEED_MS          (0)

/*******************************************************************************
 *                            LED STRIP RPM CASE                               *
 * @note: The maximum RPM of the vehicle is 12000 RPM, the LED strip has 8 LEDs*
 *        therefore an LED might indicate a value <= 12000 / 8 = 1500 RPM      *
 ******************************************************************************/

#define LOWER_THAN_1500             (0)
#define BETWEEN_1500_3000           (1)
#define BETWEEN_3000_4500           (2)
#define BETWEEN_4500_6000           (3)
#define BETWEEN_6000_7500           (4)
#define BETWEEN_7500_9000           (5)
#define BETWEEN_9000_10500          (6)
#define BETWEEN_10500_12000         (7)
#define IS_12000                    (8)

////////////////////////////////////////////////////////////////////////////////
////                       LOCAL CONSTANTS                                  ////
////////////////////////////////////////////////////////////////////////////////
static rmt_channel_handle_t led_chan           = NULL;
static  rmt_tx_channel_config_t tx_chan_config =
{
        .clk_src            = RMT_CLK_SRC_DEFAULT, // select source clock
        .gpio_num           = GPIO_LED_STRIP,
        .mem_block_symbols  = 128,                 // increase the block size can make the LED less flickering
        .resolution_hz      = RMT_LED_STRIP_RESOLUTION_HZ,
        .trans_queue_depth  = 4, // set the number of transactions that can be pending in the background
};
static rmt_encoder_handle_t led_encoder        = NULL;
static rmt_transmit_config_t tx_config = {
        .loop_count = 0, // no transfer loop
};


esp_err_t shift_neutral_led_setup(void)
{
    esp_err_t esp_response = ESP_FAIL;

    /* Configuration of the neutral led gpio */
    gpio_config_t gpio_neutral =
    {
        .pin_bit_mask = (1ULL << GPIO_LED_NEUTRAL),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };

    /* Configure the gpio based on the struct */
    ESP_ERROR_CHECK(gpio_config(&gpio_neutral));

    esp_response = ESP_OK;
    return esp_response;
}

esp_err_t shift_strip_led_setup(void)
{
    esp_err_t esp_response = ESP_FAIL;

    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));

    led_strip_encoder_config_t encoder_config =
    {
        .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
    };

    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &led_encoder));

    ESP_ERROR_CHECK(rmt_enable(led_chan));

    esp_response = ESP_OK;
    return esp_response;
}

void neutral_led_update(void *pvParameters)
{
    status_firmware_t *general_status           = (status_firmware_t*) pvParameters;
    display_data_t  *display_data               = general_status->display_data;
    SemaphoreHandle_t xSemaphore_display_data   = general_status->xSemaphore_display_data;

    while(true)
    {
        if (gpio_get_level(GPIO_LED_NEUTRAL) == 0)
        {
            if( xSemaphoreTake( xSemaphore_display_data, ( TickType_t ) 10 ) == pdTRUE )
            {
                display_data->current_gear = NEUTRAL_GEAR;
                xSemaphoreGive( xSemaphore_display_data );
            }
        }

        vTaskDelay(NEUTRAL_LED_RATE);
    }
}

void shift_strip_led_update(void *pvParameters)
{
    status_firmware_t *general_status           = (status_firmware_t*) pvParameters;
    display_data_t  *display_data               = general_status->display_data;
    SemaphoreHandle_t xSemaphore_display_data   = general_status->xSemaphore_display_data;

    /* By using this local stack variables, one can give faster the semaphore
       for the global struct display_data */
    uint16_t rpm                                = 0U;
    uint8_t rpm_case                            = 0U;

    /* There are 3 bytes needed for each LED:
       first    byte for GREEN
       second   byte for BLUE
       third    byte for RED */
    uint8_t led_strip_pixels[STRIP_LED_NUMBER * 3]  = {0U};

    while(true)
    {
        /* Set the data about the LEDs to OFF (0U) */
        memset(led_strip_pixels, 0U, sizeof(led_strip_pixels));

        if( xSemaphoreTake( xSemaphore_display_data, ( TickType_t ) 10 ) == pdTRUE )
        {
            /* Find the number of leds on the strip to be lit based on the current rpm */
            rpm = display_data->rpm;
            xSemaphoreGive( xSemaphore_display_data );

            /* Sanitize the rpm, keep it capped to 12000 RPM */
            if (rpm > 12000U)
            {
                rpm = 12000U;
            }

            rpm_case = rpm / 1500;

            switch(rpm_case)
            {
                case LOWER_THAN_1500:
                {
                    /*  There are 3 values corresponding to the first LED
                        led_strip_pixels[0] encoding GREEN
                        led_strip_pixels[1] encoding BLUE
                        led_strip_pixels[2] encoding RED

                        then the values
                        led_strip_pixels[3]
                        led_strip_pixels[4]
                        led_strip_pixels[5] correspond to the second LED and so on
                    */
                    /* Set the first LED in the STRIP to be GREEN */
                    led_strip_pixels[0] = 255;
                    break;
                }
                case BETWEEN_1500_3000:
                {
                    /* Set the first LED on the STRIP to be GREEN */
                    led_strip_pixels[0] = 255;
                    break;
                }
                case BETWEEN_3000_4500:
                {
                    /* Set the first LED on the STRIP to be GREEN */
                    led_strip_pixels[0] = 255;
                    /* Set the second LED on the STRIP to be GREEN */
                    led_strip_pixels[3] = 255;
                    break;
                }
                case BETWEEN_4500_6000:
                {
                    /* Set the first LED on the STRIP to be GREEN */
                    led_strip_pixels[0] = 255;
                    /* Set the second LED on the STRIP to be GREEN */
                    led_strip_pixels[3] = 255;
                    /* Set the third LED on the STRIP to be GREEN */
                    led_strip_pixels[6] = 255;
                    break;
                }
                case BETWEEN_6000_7500:
                {
                    /* Set the first LED on the STRIP to be GREEN */
                    led_strip_pixels[0] = 255;
                    /* Set the second LED on the STRIP to be GREEN */
                    led_strip_pixels[3] = 255;
                    /* Set the third LED on the STRIP to be GREEN */
                    led_strip_pixels[6] = 255;
                    /* Set the fourth LED on the STRIP to be BLUE */
                    led_strip_pixels[11] = 255;
                    break;
                }
                case BETWEEN_7500_9000:
                {
                    /* Set the first LED on the STRIP to be GREEN */
                    led_strip_pixels[0] = 255;
                    /* Set the second LED on the STRIP to be GREEN */
                    led_strip_pixels[3] = 255;
                    /* Set the third LED on the STRIP to be GREEN */
                    led_strip_pixels[6] = 255;
                    /* Set the fourth LED on the STRIP to be BLUE */
                    led_strip_pixels[11] = 255;
                    /* Set the fifth LED on the STRIP to be BLUE */
                    led_strip_pixels[14] = 255;
                    break;
                }
                case BETWEEN_9000_10500:
                {
                    /* Set the first LED on the STRIP to be GREEN */
                    led_strip_pixels[0] = 255;
                    /* Set the second LED on the STRIP to be GREEN */
                    led_strip_pixels[3] = 255;
                    /* Set the third LED on the STRIP to be GREEN */
                    led_strip_pixels[6] = 255;
                    /* Set the fourth LED on the STRIP to be BLUE */
                    led_strip_pixels[11] = 255;
                    /* Set the fifth LED on the STRIP to be BLUE */
                    led_strip_pixels[14] = 255;
                    /* Set the sixth LED on the STRIP to be BLUE */
                    led_strip_pixels[17] = 255;

                    break;
                }
                case BETWEEN_10500_12000:
                {
                    /* Set the first LED on the STRIP to be GREEN */
                    led_strip_pixels[0] = 255;
                    /* Set the second LED on the STRIP to be GREEN */
                    led_strip_pixels[3] = 255;
                    /* Set the third LED on the STRIP to be GREEN */
                    led_strip_pixels[6] = 255;
                    /* Set the fourth LED on the STRIP to be BLUE */
                    led_strip_pixels[11] = 255;
                    /* Set the fifth LED on the STRIP to be BLUE */
                    led_strip_pixels[14] = 255;
                    /* Set the sixth LED on the STRIP to be BLUE */
                    led_strip_pixels[17] = 255;
                    /* Set the seventh LED on the STRIP to be RED */
                    led_strip_pixels[19] = 255;
                    break;
                }

                case IS_12000:
                {
                    /* Set the first LED on the STRIP to be GREEN */
                    led_strip_pixels[0] = 255;
                    /* Set the second LED on the STRIP to be GREEN */
                    led_strip_pixels[3] = 255;
                    /* Set the third LED on the STRIP to be GREEN */
                    led_strip_pixels[6] = 255;
                    /* Set the fourth LED on the STRIP to be BLUE */
                    led_strip_pixels[11] = 255;
                    /* Set the fifth LED on the STRIP to be BLUE */
                    led_strip_pixels[14] = 255;
                    /* Set the sixth LED on the STRIP to be BLUE */
                    led_strip_pixels[17] = 255;
                    /* Set the seventh LED on the STRIP to be RED */
                    led_strip_pixels[19] = 255;
                    /* Set the eighth LED on the STRIP to be RED */
                    led_strip_pixels[22] = 255;
                }
                default:
                {
                    break;
                }
            }

            /* Send the values to the led strip */
            ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));

            /* Blink the LED strip if the RPM is in the good shift range */
            if (rpm_case >= BETWEEN_10500_12000 && rpm_case <= IS_12000)
            {
                /* Wait before turning off the strip */
                vTaskDelay(SHIFTLED_STRIP_RATE);

                /* Set the led strip to OFF */
                memset(led_strip_pixels, 0U, sizeof(led_strip_pixels));
                ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
                ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
            }
        }

        /* Update the SHIFT STRIP at SHIFTLED_STRIP_RATE set in common_types */
        vTaskDelay(SHIFTLED_STRIP_RATE);
    }
}

