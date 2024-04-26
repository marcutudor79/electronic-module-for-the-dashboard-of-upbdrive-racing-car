/******************************************************************************/
/* UPBDRIVE Dashboard Firmware                                                */
/* All rights reserverd 2023                                                  */
/******************************************************************************/

#include <lcd_task.h>

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL VARIABLES                                 ////
////////////////////////////////////////////////////////////////////////////////

/* Timer handle for changing page at 250ms rate */
static TimerHandle_t xTimer_change_page;

/* Uart queue handle used in lcd_setup function */
static QueueHandle_t uart_queue;

/* Variable to avoid change page faster than 250ms */
uint8_t change_page_ready = true;

/*
    @brief: This struct holds the sanitized data to be displayed on the LCD
            display, it is a local struct in which it is copied the data from
            the global display_data struct
*/
static display_data_t sanitized_display_data = {0};

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL FUNCTIONS                                 ////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////                       LOCAL MACROS                                     ////
////////////////////////////////////////////////////////////////////////////////

#define GENIE_READ_OBJ          0
#define GENIE_WRITE_OBJ         1
#define GENIE_WRITE_STR         2
#define GENIE_WRITE_STRU        3
#define GENIE_WRITE_CONTRAST    4
#define GENIE_REPORT_OBJ        5
#define GENIE_REPORT_EVENT      7
#define GENIEM_WRITE_BYTES      8
#define GENIEM_WRITE_DBYTES     9
#define GENIEM_REPORT_BYTES     10
#define GENIEM_REPORT_DBYTES    11
#define GENIE_WRITE_INH_LABEL   12

#define GENIE_OBJ_DIPSW                 0
#define GENIE_OBJ_KNOB                  1
#define GENIE_OBJ_ROCKERSW              2
#define GENIE_OBJ_ROTARYSW              3
#define GENIE_OBJ_SLIDER                4
#define GENIE_OBJ_TRACKBAR              5
#define GENIE_OBJ_WINBUTTON             6
#define GENIE_OBJ_ANGULAR_METER         7
#define GENIE_OBJ_COOL_GAUGE            8
#define GENIE_OBJ_CUSTOM_DIGITS         9
#define GENIE_OBJ_FORM                  10
#define GENIE_OBJ_GAUGE                 11
#define GENIE_OBJ_IMAGE                 12
#define GENIE_OBJ_KEYBOARD              13
#define GENIE_OBJ_LED                   14
#define GENIE_OBJ_LED_DIGITS            15
#define GENIE_OBJ_METER                 16
#define GENIE_OBJ_STRINGS               17
#define GENIE_OBJ_THERMOMETER           18
#define GENIE_OBJ_USER_LED              19
#define GENIE_OBJ_VIDEO                 20
#define GENIE_OBJ_STATIC_TEXT           21
#define GENIE_OBJ_SOUND                 22
#define GENIE_OBJ_TIMER                 23
#define GENIE_OBJ_SPECTRUM              24
#define GENIE_OBJ_SCOPE                 25
#define GENIE_OBJ_TANK                  26
#define GENIE_OBJ_USERIMAGES            27
#define GENIE_OBJ_PINOUTPUT             28
#define GENIE_OBJ_PININPUT              29
#define GENIE_OBJ_4DBUTTON              30
#define GENIE_OBJ_ANIBUTTON             31
#define GENIE_OBJ_COLORPICKER           32
#define GENIE_OBJ_USERBUTTON            33
// reserved for magic functions         34
#define GENIE_OBJ_SMARTGAUGE            35
#define GENIE_OBJ_SMARTSLIDER           36
#define GENIE_OBJ_SMARTKNOB             37
// Not advisable to use the below 3, use the above 3 instead.
#define GENIE_OBJ_ISMARTGAUGE           35 // Retained for backwards compatibility, however Users should use SMARTGAUGE instead of ISMARTGAUGE
#define GENIE_OBJ_ISMARTSLIDER          36 // Retained for backwards compatibility, however Users should use SMARTSLIDER instead of ISMARTSLIDER
#define GENIE_OBJ_ISMARTKNOB            37 // Retained for backwards compatibility, however Users should use SMARTKNOB instead of ISMARTKNOB
// Comment end
#define GENIE_OBJ_ILED_DIGITS_H         38
#define GENIE_OBJ_IANGULAR_METER        39
#define GENIE_OBJ_IGAUGE                40
#define GENIE_OBJ_ILABELB               41
#define GENIE_OBJ_IUSER_GAUGE           42
#define GENIE_OBJ_IMEDIA_GAUGE          43
#define GENIE_OBJ_IMEDIA_THERMOMETER    44
#define GENIE_OBJ_ILED                  45
#define GENIE_OBJ_IMEDIA_LED            46
#define GENIE_OBJ_ILED_DIGITS_L         47
#define GENIE_OBJ_ILED_DIGITS           47
#define GENIE_OBJ_INEEDLE               48
#define GENIE_OBJ_IRULER                49
#define GENIE_OBJ_ILED_DIGIT            50
#define GENIE_OBJ_IBUTTOND              51
#define GENIE_OBJ_IBUTTONE              52
#define GENIE_OBJ_IMEDIA_BUTTON         53
#define GENIE_OBJ_ITOGGLE_INPUT         54
#define GENIE_OBJ_IDIAL                 55
#define GENIE_OBJ_IMEDIA_ROTARY         56
#define GENIE_OBJ_IROTARY_INPUT         57
#define GENIE_OBJ_ISWITCH               58
#define GENIE_OBJ_ISWITCHB              59
#define GENIE_OBJ_ISLIDERE              60
#define GENIE_OBJ_IMEDIA_SLIDER         61
#define GENIE_OBJ_ISLIDERH              62
#define GENIE_OBJ_ISLIDERG              63
#define GENIE_OBJ_ISLIDERF              64
#define GENIE_OBJ_ISLIDERD              65
#define GENIE_OBJ_ISLIDERC              66
#define GENIE_OBJ_ILINEAR_INPUT         67

#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))

#define LCD_CHECK_IF_COOLANT_TEMP_LOWER_85(coolant_temperature)     (coolant_temperature <= 85)
#define LCD_CHECK_IF_COOLANT_TEMP_BTWEEN_85_95(coolant_temperature) ((coolant_temperature > 85) && (coolant_temperature <= 95))
#define LCD_CHECK_IF_COOLANT_TEMP_HIGHER_95(coolant_temperature)    (coolant_temperature > 95)

#define LCD_CHECK_IF_OIL_TEMP_LOWER_100(oil_temperature)            (oil_temperature <= 100)
#define LCD_CHECK_IF_OIL_TEMP_BWTEEN_100_130(oil_temperature)       ((oil_temperature > 100) && (oil_temperature <= 130))
#define LCD_CHECK_IF_OIL_TEMP_HIGHER_130(oil_temperature)           (oil_temperature > 130)

/* Error flag register bits mask*/
#define LCD_LOG_COOLANT_ERROR               (0b00000001)
#define LCD_LOG_OIL_TEMP_ERROR              (0b00000010)
#define LCD_LOG_OIL_PRESSURE_ERROR          (0b00000100)
#define LCD_LOG_HYBRID_STATUS_ERROR         (0b00001000)
#define LCD_LOG_SAFETY_STATUS_ERROR         (0b00010000)
#define LCD_LOG_CAN_STATUS_ERROR            (0b00100000)
#define LCD_LOG_TPS_ERROR                   (0b01000000)

////////////////////////////////////////////////////////////////////////////////
////                       LOCAL CONSTANTS                                  ////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////                       LOCAL FUNCTIONS                                  ////
////////////////////////////////////////////////////////////////////////////////

/*
    @brief: This function is used to write data to the LCD 4DSystems display
    via the UART port,

    @note: The uart port is hardcoded to UART_NUM_2 because the code is
    written for ESP32 WROOM 32 mcu. Change it accourdingly if you are not
    using the same mcu.
*/
static esp_err_t lcd_write_object (uint8_t object, uint8_t index, uint16_t data) {
    uint8_t msb, lsb;
    uint8_t checksum, write_signal;
    write_signal = GENIE_WRITE_OBJ;

    lsb = lowByte(data);
    msb = highByte(data);

    uart_write_bytes(UART_NUM_2, &write_signal, 1);
    checksum  = GENIE_WRITE_OBJ ;
    uart_write_bytes(UART_NUM_2, &object, 1);
    checksum ^= object ;
    uart_write_bytes(UART_NUM_2, &index, 1);
    checksum ^= index ;
    uart_write_bytes(UART_NUM_2, &msb, 1);
    checksum ^= msb;
    uart_write_bytes(UART_NUM_2, &lsb, 1);
    checksum ^= lsb;
    uart_write_bytes(UART_NUM_2, &checksum, 1);

    uart_read_bytes(UART_NUM_2, &checksum, 1, 1000 / portTICK_PERIOD_MS);

    #ifdef ENABLE_DEBUG_DISPLAY
    printf("response: %d\n", checksum);
    #endif /* ENABLE_DEBUG_DISPLAY */

    return 0;
}

/*
    @brief: This is a callback function to set the variable change_page_ready as
            true after 250ms

    @note: When you press a button, it will have a bouncing effect, this timer
    mitigates that at button press, the debounce at button release is mitigated in
    the buttton ISR
*/
void change_page_timer_callback()
{
    change_page_ready = true;
}

/*
    @brief: This function is used to change the page of the LCD display
            based on the signal_change_page flag and on the current_page variable

    @note: The signal_change_page variable is set in the button ISR and current_page
           is a local variable of the lcd_update task
*/
static esp_err_t change_page_handler(uint8_t* signal_change_page, uint8_t* current_page)
{
    esp_err_t esp_response = ESP_FAIL;

    /* Check if the change page button was pressed */
    if ((0xFF == *signal_change_page) && (true == change_page_ready))
    {
        /* Set this variable to false to avoid multiple changes */
        change_page_ready = false;
        xTimerStart(xTimer_change_page, 0);

        /* Check what is the current page (MAIN, SECONDARY, THIRD) */
        switch (*current_page)
        {
            /* If the page is MAIN switch to SECONDARY */
            case LCD_MAIN_PAGE:
            {
                *current_page = LCD_SECONDARY_PAGE;
                esp_response = lcd_write_object(GENIE_OBJ_FORM, 0x01, 0);
                break;
            }

            /* If the page is SECONDARY switch to THIRD */
            case LCD_SECONDARY_PAGE:
            {
                *current_page = LCD_THIRD_PAGE;
                esp_response = lcd_write_object(GENIE_OBJ_FORM, 0x02, 0);
                break;
            }

            /* If the page is THIRD switch to FOURTH */
            case LCD_THIRD_PAGE:
            {
                *current_page = LCD_FOURTH_PAGE;
                esp_response = lcd_write_object(GENIE_OBJ_FORM, 0x04, 0);
                break;
            }

            /* If the page is FOURTH switch to MAIN */
            case LCD_FOURTH_PAGE:
            {
                *current_page = LCD_MAIN_PAGE;
                esp_response = lcd_write_object(GENIE_OBJ_FORM, 0x00, 0);
                break;
            }

            /* In case of unknown current page state, or start-up
                switch to MAIN */
            default:
            {
                *current_page = LCD_MAIN_PAGE;
                esp_response = lcd_write_object(GENIE_OBJ_FORM, 0x00, 0);
                break;
            }
        }

        /* Set the signal to 0x00, to mark that the page change was processed */
        *signal_change_page = 0x00;
    }

    esp_response = ESP_OK;
    return esp_response;
}

/*  @brief: This function sanitizes the data in the display_data struct before
           sending it to the LCD display

    @note: If a value higher then expected is sent to the LCD display, it will
           crash and it will need a power cycle to recover, this function mitigates
           that

    @param[IN]: display_data_t* display_data pointer to the global struct that holds
                the data from the CAN bus and the ACCELEROMETER

    @param[IN]: SemaphoreHandle_t xSemaphore_display_data semaphore to protect the
                display_data struct from being accessed by multiple tasks at the same time

    @param[OUT]: esp_err_t esp_response
*/
static esp_err_t lcd_data_check_sanity(display_data_t* display_data, display_data_t* sanitized_display_data, SemaphoreHandle_t xSemaphore_display_data)
{
    esp_err_t esp_response = ESP_FAIL;

    /* Wait for the semaphore to be available, this is a blocking point in the code */
    if(xSemaphoreTake( xSemaphore_display_data, portMAX_DELAY ) == pdTRUE )
    {
        /* Make a local copy of the display_data struct in the sanitized_display_data */
        memcpy(sanitized_display_data, display_data, sizeof(display_data_t));

        xSemaphoreGive( xSemaphore_display_data );
    }

    /* Check if the RPM is higher then 12000, if it is, set it to 1200 */
    if (sanitized_display_data->rpm > 12000)
    {
        sanitized_display_data->rpm = 12000;
    }

    /* Check if the COOLANT is higher than 100, if it is, set it to 100

        @note: That the coolant temperature is represented with a progress bar
        on the LCD display, which is not symmetrical therefore the coolant temperature
        needs to be readjusted before being sent to the display

        0 - 85 degrees celsius are represented between 0 - 40 on the progress bar
        0 being 0 and 85 being 40

        85 - 95 degrees celsius are represented between 40 - 70 on the progress
        bar

        95 - 100 degrees celsius are represented between 70 - 100 on the progress
        bar
    */
    if (sanitized_display_data->coolant_temperature > 100)
    {
        sanitized_display_data->coolant_temperature = 100;
    }

    /* This is the blue area of the progress bar, engine is cold  */
    else if (LCD_CHECK_IF_COOLANT_TEMP_LOWER_85(sanitized_display_data->coolant_temperature))
    {
        sanitized_display_data->coolant_temperature = (uint8_t)((sanitized_display_data->coolant_temperature / 85.0) * 40.0);
    }
    /* This is the green area of the progress bar, considered to be optimal */
    else if (LCD_CHECK_IF_COOLANT_TEMP_BTWEEN_85_95(sanitized_display_data->coolant_temperature))
    {
        sanitized_display_data->coolant_temperature = 40 + (uint8_t)((sanitized_display_data->coolant_temperature - 85.0) / 10.0 * 30.0);
    }
    /* This is the red area of the progress bar, considered dangerous */
    else if (LCD_CHECK_IF_COOLANT_TEMP_HIGHER_95(sanitized_display_data->coolant_temperature))
    {
        sanitized_display_data->coolant_temperature = 70 + (uint8_t)((sanitized_display_data->coolant_temperature - 95) / 5.0 * 30.0);
    }

    /* Check if oil temparature is higher than 100, if it is, set it to 100

        @note: That the oil temperature is represented with a progress bar
        on the LCD display, which is not symmetrical therefore the oil temperature
        needs to be readjusted before being sent to the display

        0 - 100 degress celsius are represented between 0 - 50 on the progress bar

        100 - 130 degrees celsius are represented between 50 - 80 on the progress
        bar

        130 - 150 degrees celsius are represented between 80 - 100 on the progress
    */
    if (sanitized_display_data->oil_temperature > 150)
    {
        sanitized_display_data->oil_temperature = 150;
    }

    /* Copy the unscaled version of oil_temperature in oil_temperature_2 */
    sanitized_display_data->oil_temperature_2 = sanitized_display_data->oil_temperature;

    /* This is the blue area on the display, engine is cold */
    if (LCD_CHECK_IF_OIL_TEMP_LOWER_100(sanitized_display_data->oil_temperature))
    {
        sanitized_display_data->oil_temperature = sanitized_display_data->oil_temperature / 2;
    }
    /* This is the green area on the display, it is considered optimal */
    else if (LCD_CHECK_IF_OIL_TEMP_BWTEEN_100_130(sanitized_display_data->oil_temperature))
    {
        sanitized_display_data->oil_temperature = 50 + (sanitized_display_data->oil_temperature - 100);
    }
    /* This is the red area on the display, it is considered dangerous */
    else if (LCD_CHECK_IF_OIL_TEMP_HIGHER_130(sanitized_display_data->oil_temperature))
    {
        sanitized_display_data->oil_temperature = 80 + (sanitized_display_data->oil_temperature - 130);
    }

    /* Check if the fan_state is either true or false, set it to false to signal error  */
    if ((true != sanitized_display_data->fan_state) && (false != sanitized_display_data->fan_state))
    {
        sanitized_display_data->fan_state = false;
    }

    /* Check if the can_state is either true or false, set error flag */
    if ((true != sanitized_display_data->can_status) && (false != sanitized_display_data->can_status))
    {
        sanitized_display_data->can_status = false;

        /* Log error */
        sanitized_display_data->error_flags = sanitized_display_data->error_flags | LCD_LOG_CAN_STATUS_ERROR;
    }

    /* Check if the hybrid_status is either true or false, set error flag */
    if ((true != sanitized_display_data->hybrid_status) && (false != sanitized_display_data->hybrid_status))
    {
        /* Set it to false */
        sanitized_display_data->hybrid_status = false;

        /* Log error */
        sanitized_display_data->error_flags = sanitized_display_data->error_flags | LCD_LOG_HYBRID_STATUS_ERROR;
    }

    /* Check if the safety_circuit_status is either true or false */
    if ((true != sanitized_display_data->safety_circuit_status) && (false != sanitized_display_data->safety_circuit_status))
    {
        /* Set it to false */
        sanitized_display_data->safety_circuit_status = false;

        /* Log error */
        sanitized_display_data->error_flags = sanitized_display_data->error_flags | LCD_LOG_SAFETY_STATUS_ERROR;
    }

    /* Check if the TPS exceeds 100, set it to 100

        @note: the TPS should have maximum 100% application
    */
    if (sanitized_display_data->tps > 100)
    {
        sanitized_display_data->tps = 100;

        /* Log error */
        sanitized_display_data->error_flags = sanitized_display_data->error_flags | LCD_LOG_TPS_ERROR;
    }

    if (0U == sanitized_display_data->error_flags)
    {
        esp_response = ESP_OK;
    }

    return esp_response;
}

esp_err_t lcd_setup(void)
{
    esp_err_t esp_response = ESP_FAIL;
    uart_config_t uart_config = {
                                    /* Set the LCD baudarate according to common.h headfer */
                                    .baud_rate = LCD_BAUDRATE,
                                    /* The lcd communicates on 8 bit data without parity */
                                    .data_bits = UART_DATA_8_BITS,
                                    .parity = UART_PARITY_DISABLE,
                                    .stop_bits = UART_STOP_BITS_1,
                                    /* There is no flow control needed to communicate with
                                       the lcd */
                                    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                                    .rx_flow_ctrl_thresh = 122,
                                };

    /* Setup the configuration parameters on UART_2, since UART_1 is used
       to flash the ESP32 microcontroller */
    esp_response = uart_param_config(UART_NUM_2, &uart_config);
    if(esp_response != ESP_OK)
    {
        return esp_response;
    }

    /* Setup UART2 buffered IO with event queue */
    const int uart_buffer_size = (1024 * 2);

    /* Install UART2 driver using an event queue here */
    esp_response = uart_driver_install(UART_NUM_2, uart_buffer_size, \
                                        uart_buffer_size, 10, &uart_queue, 0);
    if(esp_response != ESP_OK)
    {
        return esp_response;
    }

    /* Configure the GPIO_LCD_TX and GPIO_LCD_RX as UART2 pins */
    esp_response = uart_set_pin(UART_NUM_2, GPIO_LCD_TX, GPIO_LCD_RX, -1, -1);
    if(esp_response != ESP_OK)
    {
        return esp_response;
    }

    /* Set the entrance page to LCD_PAGE_START_UP correspondint to 0x03 in 4DSystems
       project FORM settings */
    lcd_write_object(GENIE_OBJ_FORM, 0x03, 0);

    /* Create a software timer for change_page_ready variable, wait 250ms before being able to change the page again */
    xTimer_change_page = xTimerCreate("change_page", (250/portTICK_PERIOD_MS), pdFALSE, (void *)0, change_page_timer_callback);

    return esp_response;
}

void lcd_update(void *pvParameters)
{
    esp_err_t esp_response                    = ESP_FAIL;
    status_firmware_t *general_status         = (status_firmware_t*)pvParameters;
    display_data_t    *display_data           = general_status->display_data;
    SemaphoreHandle_t xSemaphore_display_data = general_status->xSemaphore_display_data;

    /* start on the main page */
    uint8_t current_page                      = LCD_MAIN_PAGE;

    /* Wait the LCD_START_UP_SCREEN_DELAY time then change to MAIN_PAGE */
    vTaskDelay(LCD_START_UP_SCREEN_DELAY);
    lcd_write_object(GENIE_OBJ_FORM, 0x00, 0);

    #ifdef ENABLE_DEBUG_DISPLAY
    /* initialize the object for display control */
    uint8_t value = 0;
    #endif /* ENABLE_DEBUG_DISPLAY */

    while(true)
    {
        #ifndef ENABLE_DEBUG_DISPLAY
        esp_response = change_page_handler(&general_status->signal_change_page, &current_page);
        if (esp_response != ESP_OK)
        {
            ESP_LOGE("change_page_handler", "Failed to change page\n");
        }

        esp_response = lcd_data_check_sanity(display_data, &sanitized_display_data, xSemaphore_display_data);
        if (esp_response != ESP_OK)
        {
            ESP_LOGE("display_data_process", "Failed to process the display data\n");
        }

        switch(current_page)
        {

            /*************************************************************
             *                    MAIN PAGE UPDATE LOGIC                 *
            **************************************************************/
            case LCD_MAIN_PAGE:
            {
                /* Set the RPM digits */
                lcd_write_object(GENIE_OBJ_LED_DIGITS, 1, sanitized_display_data.rpm);

                /* Set the RPM bar */
                lcd_write_object(GENIE_OBJ_IGAUGE,     0, sanitized_display_data.rpm/100);

                /* Set the GEAR digit */
                lcd_write_object(GENIE_OBJ_LED_DIGITS, 0, sanitized_display_data.current_gear);

                /* Set the COOLANT bar */
                lcd_write_object(GENIE_OBJ_IGAUGE,     1, sanitized_display_data.coolant_temperature);

                /* Set the FAN led status */
                lcd_write_object(GENIE_OBJ_IMEDIA_LED, 0, sanitized_display_data.fan_state);

                /* Set the OIL temperature bar */
                lcd_write_object(GENIE_OBJ_IGAUGE,     2, sanitized_display_data.oil_temperature);

                break;
            }

            /*************************************************************
             *                    SECONDARY PAGE UPDATE LOGIC            *
            **************************************************************/
            case LCD_SECONDARY_PAGE:
            {
                /* Set the CAN led status */
                lcd_write_object(GENIE_OBJ_IMEDIA_LED, 1, sanitized_display_data.can_status);

                /* Set the HYBRID system led status */
                lcd_write_object(GENIE_OBJ_IMEDIA_LED, 2, sanitized_display_data.hybrid_status);

                /* Set the SAFETY circuit led status */
                lcd_write_object(GENIE_OBJ_IMEDIA_LED, 3, sanitized_display_data.safety_circuit_status);

                /* Set the OIL PRESSURE digits */
                lcd_write_object(GENIE_OBJ_LED_DIGITS, 5, sanitized_display_data.oil_pressure);

                /* Set the BRAKE PRESSURE digits */
                lcd_write_object(GENIE_OBJ_LED_DIGITS, 6, sanitized_display_data.brake_pressure_raw);

                /* Set the TPS % digits */
                lcd_write_object(GENIE_OBJ_LED_DIGITS, 7, sanitized_display_data.tps);

                /* Set the OIL TEMPERATURE digits */
                lcd_write_object(GENIE_OBJ_LED_DIGITS, 8, sanitized_display_data.oil_temperature_2);

                /* Set the MAP digits */
                lcd_write_object(GENIE_OBJ_LED_DIGITS, 9, sanitized_display_data.hybrid_selector_value);

                /* Set the LV BATT VOLTAGE digits */
                lcd_write_object(GENIE_OBJ_LED_DIGITS, 10, sanitized_display_data.battery_voltage);
                break;
            }

            /*************************************************************
             *                    THIRD PAGE UPDATE LOGIC                *
            **************************************************************/
            case LCD_THIRD_PAGE:
            {
                /* Set the LAMBDA digits */
                lcd_write_object(GENIE_OBJ_LED_DIGITS, 2, sanitized_display_data.lambda);

                /* Set the MASS AIRFLOW PREESURE digits */
                lcd_write_object(GENIE_OBJ_LED_DIGITS, 3, sanitized_display_data.map);

                /* Set the FUEL PRESSURE digits */
                lcd_write_object(GENIE_OBJ_LED_DIGITS, 4, sanitized_display_data.fuel_pressure);

                /* Set the EGT #1 digits */
                lcd_write_object(GENIE_OBJ_LED_DIGITS, 11, sanitized_display_data.egt[0]);

                /* Set the EGT #2 digits */
                lcd_write_object(GENIE_OBJ_LED_DIGITS, 12, sanitized_display_data.egt[1]);

                /* Set the EGT #3 digits */
                lcd_write_object(GENIE_OBJ_LED_DIGITS, 13, sanitized_display_data.egt[2]);

                /* Set the EGT #4 digits */
                lcd_write_object(GENIE_OBJ_LED_DIGITS, 14, sanitized_display_data.egt[3]);

                /* Set the IN CURRENT ALTERN digits */
                lcd_write_object(GENIE_OBJ_LED_DIGITS, 16, sanitized_display_data.input_current_altr_pmu);

                /* Set the IN VOLTAGE PMU digits */
                lcd_write_object(GENIE_OBJ_LED_DIGITS, 15, sanitized_display_data.input_voltage_pmu);
                break;
            }

            /*************************************************************
             *                    FOURTH PAGE UPDATE LOGIC               *
            **************************************************************/
            case LCD_FOURTH_PAGE:
            {
                /* Set the TYRE LEFT FRONT digits */
                lcd_write_object(GENIE_OBJ_LED_DIGITS, 17, sanitized_display_data.tyre_pressure[0]);

                /* Set the TYRE LEFT BACK digits */
                lcd_write_object(GENIE_OBJ_LED_DIGITS, 18, sanitized_display_data.tyre_pressure[1]);

                /* Set the TYRE RIGHT FRONT digits */
                lcd_write_object(GENIE_OBJ_LED_DIGITS, 20, sanitized_display_data.tyre_pressure[2]);

                /* Set the TYRE RIGHT BACK digits */
                lcd_write_object(GENIE_OBJ_LED_DIGITS, 19, sanitized_display_data.tyre_pressure[3]);
                break;
            }

            /* Enter here only if current_page is corrupted, force it to MAIN */
            default:
            {
                current_page = LCD_MAIN_PAGE;
                break;
            }
        }

        /* This code is used for debug purposes, it will increment the gear
            as well as the rpm */
        #else
        printf("lcd update\n");
        lcd_write_object(GENIE_OBJ_LED_DIGITS, 0, value);
        lcd_write_object(GENIE_OBJ_LED_DIGITS, 1, value);
        lcd_write_object(GENIE_OBJ_IGAUGE,     0, value);
        value += 1;
        #endif /* ENABLE_DEBUG_DISPLAY */

        /* Put the task in blocking state */
        vTaskDelay(LCD_REFRESH_RATE);
    }
}