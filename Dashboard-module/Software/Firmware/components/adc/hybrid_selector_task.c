/******************************************************************************/
/* UPBDRIVE Dashboard Firmware                                                */
/* All rights reserverd 2023                                                  */
/******************************************************************************/

#include <hybrid_selector_task.h>

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL FUNCTIONS                                 ////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////                       LOCAL VARIABLES                                 ////
////////////////////////////////////////////////////////////////////////////////

static adc_oneshot_unit_handle_t adc1_handle;

////////////////////////////////////////////////////////////////////////////////
////                       LOCAL MACROS                                     ////
////////////////////////////////////////////////////////////////////////////////

/* Error state */
#define HYBRID_STATE_ERROR (0)

/* Voltage value between 0.4V - 0.6V */
#define HYBRID_STATE_1 (1)

/* Voltage value between 0.7V - 0.9V */
#define HYBRID_STATE_2 (2)

/* Voltage value between 1.0V - 1.2V */
#define HYBRID_STATE_3 (3)

/* Voltage value between 1.3V - 1.5V */
#define HYBRID_STATE_4 (4)

/* Voltage value between 1.6V - 1.8V */
#define HYBRID_STATE_5 (5)

/* Voltage value between 1.9V - 2.1V */
#define HYBRID_STATE_6 (6)

/* Voltage value between 2.1V - 2.3V */
#define HYBRID_STATE_7 (7)

/* Volatage value between 2.4V - 2.6V */
#define HYBRID_STATE_8 (8)

/* Voltage value between 2.7V - 2.9V */
#define HYBRID_STATE_9 (9)

/* Voltage value between 2.9V - 3.1V */
#define HYBRID_STATE_10 (10)

/* Voltage value between 3.2V - 3.3V */
#define HYBRID_STATE_11 (11)

/* Numeric digital outputs of the 12bit adc conversion for different voltages*/
#define RAW_VOLTAGE_0V4 (496)
#define RAW_VOLTAGE_0V6 (744)
#define RAW_VOLTAGE_0V7 (868)
#define RAW_VOLTAGE_0V9 (1117)
#define RAW_VOLTAGE_1V0 (1241)
#define RAW_VOLTAGE_1V2 (1489)
#define RAW_VOLTAGE_1V3 (1613)
#define RAW_VOLTAGE_1V5 (1861)
#define RAW_VOLTAGE_1V6 (1985)
#define RAW_VOLTAGE_1V8 (2234)
#define RAW_VOLTAGE_1V9 (2358)
#define RAW_VOLTAGE_2V1 (2606)
#define RAW_VOLTAGE_2V3 (2854)
#define RAW_VOLTAGE_2V4 (2978)
#define RAW_VOLTAGE_2V6 (3227)
#define RAW_VOLTAGE_2V7 (3351)
#define RAW_VOLTAGE_2V9 (3599)
#define RAW_VOLTAGE_3V1 (3847)
#define RAW_VOLTAGE_3V2 (3971)

////////////////////////////////////////////////////////////////////////////////
////                       LOCAL CONSTANTS                                  ////
////////////////////////////////////////////////////////////////////////////////

esp_err_t hybrid_selector_setup(void)
{
    esp_err_t esp_response                  = ESP_FAIL;

    adc_oneshot_unit_init_cfg_t init_adc1_config =
    {
        .unit_id = ADC_UNIT_1
    };

    adc_oneshot_chan_cfg_t channel_adc1_config = {
        .atten    = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_12
    };

    esp_response = adc_oneshot_new_unit(&init_adc1_config, &adc1_handle);
    if (esp_response != ESP_OK)
    {
        printf("Failed to configure the ADC1 unit\n");
        return esp_response;
    }

    esp_response = adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_6, &channel_adc1_config);
    if (esp_response != ESP_OK)
    {
        printf("Failed to configure the ADC1 channel\n");
        return esp_response;
    }

    esp_response = ESP_OK;
    return esp_response;
}

void hybrid_selector_read(void *pvParameters)
{
    status_firmware_t *general_status         = (status_firmware_t*)pvParameters;
    display_data_t    *display_data           = general_status->display_data;
    SemaphoreHandle_t xSemaphore_display_data = general_status->xSemaphore_display_data;
    int       adc_buffer                      = 0;
    uint8_t   i                               = 0U;
    uint8_t   hybrid_selector_value           = HYBRID_STATE_ERROR;
    uint32_t  hybrid_selector_raw             = 0UL;

    while(true)
    {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_6, &adc_buffer)) ;

        /* Add the value to the hybrid selector raw buffer */
        hybrid_selector_raw = hybrid_selector_raw + adc_buffer;

        /* Check if the iterator reached the maximum or overflowed it */
        if( i >= (HYBRID_SELECTOR_MA_FILTER_SAMPLES) )
        {
            /* Compute the moving average result of the raw buffer */
            hybrid_selector_raw = hybrid_selector_raw / (HYBRID_SELECTOR_MA_FILTER_SAMPLES+1);

            /* Update the state of the of the hybrid selector */
            if (hybrid_selector_raw >= RAW_VOLTAGE_0V4 && hybrid_selector_raw <= RAW_VOLTAGE_0V6)
            {
                hybrid_selector_value = HYBRID_STATE_1;
            }
            else if (hybrid_selector_raw >= RAW_VOLTAGE_0V7 && hybrid_selector_raw <= RAW_VOLTAGE_0V9)
            {
                hybrid_selector_value = HYBRID_STATE_2;
            }
            else if (hybrid_selector_raw >= RAW_VOLTAGE_1V0 && hybrid_selector_raw <= RAW_VOLTAGE_1V2)
            {
                hybrid_selector_value = HYBRID_STATE_3;
            }
            else if (hybrid_selector_raw >= RAW_VOLTAGE_1V3 && hybrid_selector_raw <= RAW_VOLTAGE_1V5)
            {
                hybrid_selector_value = HYBRID_STATE_4;
            }
            else if (hybrid_selector_raw >= RAW_VOLTAGE_1V6 && hybrid_selector_raw <= RAW_VOLTAGE_1V8)
            {
                hybrid_selector_value = HYBRID_STATE_5;
            }
            else if (hybrid_selector_raw >= RAW_VOLTAGE_1V9 && hybrid_selector_raw <= RAW_VOLTAGE_2V1)
            {
                hybrid_selector_value = HYBRID_STATE_6;
            }
            else if (hybrid_selector_raw >= RAW_VOLTAGE_2V1 && hybrid_selector_raw <= RAW_VOLTAGE_2V3)
            {
                hybrid_selector_value = HYBRID_STATE_7;
            }
            else if (hybrid_selector_raw >= RAW_VOLTAGE_2V4 && hybrid_selector_raw <= RAW_VOLTAGE_2V6)
            {
                hybrid_selector_value = HYBRID_STATE_8;
            }
            else if (hybrid_selector_raw >= RAW_VOLTAGE_2V7 && hybrid_selector_raw <= RAW_VOLTAGE_2V9)
            {
                hybrid_selector_value = HYBRID_STATE_9;
            }
            else if (hybrid_selector_raw >= RAW_VOLTAGE_2V9 && hybrid_selector_raw <= RAW_VOLTAGE_3V1)
            {
                hybrid_selector_value = HYBRID_STATE_10;
            }
            else if (hybrid_selector_raw >= RAW_VOLTAGE_3V2)
            {
                hybrid_selector_value = HYBRID_STATE_11;
            }
            else
            {
                hybrid_selector_value = HYBRID_STATE_ERROR;
            }

            /* Reset hybrid buffer and iterator */
            hybrid_selector_raw = 0UL;
            i                   = 0U;

            if (xSemaphoreTake(xSemaphore_display_data, (TickType_t)10) == pdTRUE)
            {
                display_data->hybrid_selector_value = hybrid_selector_value;
                xSemaphoreGive(xSemaphore_display_data);
            }

            #ifdef ENABLE_DEBUG_HYBRID_SELECTOR
            printf("hybrid_selector_value: %d, iterator: %d\n", hybrid_selector_value, i);
            #endif /* ENABLE_DEBUG_HYBRID_SELECTOR */
        }

        /* Increment the iterator */
        i++;

        /* put the task in blocking state */
        vTaskDelay(HYBRID_SELECTOR_POLLING_RATE);
    }
}