/******************************************************************************/
/* UPBDRIVE Dashboard Firmware                                                */
/* All rights reserverd 2023                                                  */
/******************************************************************************/

#include <buttons_task.h>

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL VARIABLES                                 ////
////////////////////////////////////////////////////////////////////////////////

extern status_firmware_t general_status;
extern display_data_t    display_data;
extern uint8_t change_page_ready;

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
////                       FUNCTION DEFINITIONS                             ////
////////////////////////////////////////////////////////////////////////////////

void IRAM_ATTR button_change_page_isr(void)
{
    /* Software debounce consisting in:
       - taking into consideration bouncing effect of ns order
       - gpio level should be 0 if the button is pressed
       - bouncing effect happens at button release, mitigated with gpio level
       - should change page at a defined speed, does not care if button is pressed faster */
    if ((gpio_get_level(GPIO_BUTTON_LCD_PAGE) == 0) && (change_page_ready == 1))
    {
        general_status.signal_change_page = 0xFF;
    }
}

void IRAM_ATTR button_rad_fan_isr(void)
{
    /* Recheck the status of radiator button if it is ON or OFF */
    if(gpio_get_level(GPIO_BUTTON_RAD_FAN) == 1)
    {
        display_data.fan_state = true;
    }
    else
    {
        display_data.fan_state = false;
    }
}

void IRAM_ATTR button_safety_circ_isr(void)
{
    /* Recheck the status of safety circ button if it is ON or OFF */
    if(gpio_get_level(GPIO_BUTTON_SAFETY_CIRC) == 1)
    {
        display_data.safety_circuit_status = true;
    }
    else
    {
        display_data.safety_circuit_status = false;
    }
}

esp_err_t buttons_setup(void)
{
    esp_err_t esp_response = ESP_FAIL;

    /* When the button is pressed, gpio pin is connected with 2.71V, therefore,
       enable PULLDOWN when for the button not pressed state */
    gpio_config_t gpio_button_rad_fan =
    {
        .pin_bit_mask = (1ULL << GPIO_BUTTON_RAD_FAN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_ANYEDGE
    };

    /* When the button is pressed, gpio pin is connected with GND, therefore,
       enable PULLUP when for the button not pressed state */
    gpio_config_t gpio_button_lcd_page =
    {
        .pin_bit_mask = (1ULL << GPIO_BUTTON_LCD_PAGE),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE
    };

    /* When the button is pressed, gpio pin is connected with 2.71V */
    gpio_config_t gpio_button_safety_circuit =
    {
        .pin_bit_mask = (1ULL << GPIO_BUTTON_SAFETY_CIRC),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_ANYEDGE
    };

    esp_response = gpio_config(&gpio_button_rad_fan);
    if(esp_response != ESP_OK)
    {
        return esp_response;
    }

    esp_response = gpio_config(&gpio_button_lcd_page);
    if(esp_response != ESP_OK)
    {
        return esp_response;
    }

    esp_response = gpio_config(&gpio_button_safety_circuit);
    if(esp_response != ESP_OK)
    {
        return esp_response;
    }

    esp_response = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if(esp_response != ESP_OK)
    {
        return esp_response;
    }

    /* Add isr handler for GPIO_BUTTON_LCD_PAGE */
    esp_response = gpio_isr_handler_add(GPIO_BUTTON_LCD_PAGE, (gpio_isr_t)button_change_page_isr, NULL);
    if(esp_response != ESP_OK)
    {
        return esp_response;
    }

    /* Add isr handler for GPIO_BUTTON_RAD_FAN */
    esp_response = gpio_isr_handler_add(GPIO_BUTTON_RAD_FAN, (gpio_isr_t)button_rad_fan_isr, NULL);
    if(esp_response != ESP_OK)
    {
        return esp_response;
    }

    /* Add isr handler for GPIO_BUTTON_SAFETY_CIRC */
    esp_response = gpio_isr_handler_add(GPIO_BUTTON_SAFETY_CIRC, (gpio_isr_t)button_safety_circ_isr, NULL);
    if(esp_response != ESP_OK)
    {
        return esp_response;
    }

    /* Avoid unknown boot-up state for SAFETY CIRCUIT */
    if(gpio_get_level(GPIO_BUTTON_SAFETY_CIRC) == 1)
    {
        display_data.safety_circuit_status = true;
    }
    else
    {
        display_data.safety_circuit_status = false;
    }

    return esp_response;
}



#ifdef ENABLE_DEBUG_BUTTONS
void buttons_read(void *pvParameters)
{
    while(true)
    {

        /* Debug print for button change page */
        if(general_status.signal_change_page == 0xFF)
        {
            #ifdef ENABLE_DEBUG_DISPLAY
            general_status.signal_change_page = 0x00;
            #endif /* ENABLE_DEBUG_DISPLAY */

            printf("Button change page!\n");
        }

        /* Debug print for fan state button */
        if(display_data.fan_state == true)
        {
            printf("Fan is ON!\n");
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
#endif /* ENABLE_DEBUG_BUTTONS */