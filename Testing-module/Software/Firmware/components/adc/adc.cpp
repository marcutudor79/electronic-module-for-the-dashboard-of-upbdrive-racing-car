/******************************************************************************/
/* UPBDRIVE ECU-Emulator Firmware                                             */
/* All rights reserverd 2023                                                  */
/******************************************************************************/

#include "adc.h"

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL VARIABLES                                 ////
////////////////////////////////////////////////////////////////////////////////

static uint8_t button_state_gearshift      = 0U;
static uint8_t button_state_lowbattery     = 0U;
static uint8_t button_state_lowoilpressure = 0U;

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL FUNCTIONS                                 ////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////                       LOCAL MACROS                                     ////
////////////////////////////////////////////////////////////////////////////////

/* The ADC inputs are connected to 3 potentiometers
    - ADC0 input emulates the RPM value
    - ADC1 input emulates the OIL_TEMPERATURE value
    - ADC2 input emulates the COOLANT_TEMPERATURE value */
#define ADC_INPUT_RPM               (0)
#define ADC_INPUT_OIL_TEMP          (1)
#define ADC_INPUT_COOLANT_TEMP      (2)

////////////////////////////////////////////////////////////////////////////////
////                       LOCAL CONSTANTS                                  ////
////////////////////////////////////////////////////////////////////////////////

void init_adc_and_buttons()
{
    /* initialise ADC */
    adc_init();
    adc_gpio_init(GPIO_NUM_POTENTIOMETER);
    adc_gpio_init(GPIO_NUM_OIL_TEMP);
    adc_gpio_init(GPIO_NUM_COOLANT_TEMP);

    /* initialize button gear shift */
    gpio_init(GPIO_NUM_GEAR_SHIFT);
    gpio_set_dir(GPIO_NUM_GEAR_SHIFT, GPIO_IN);
    gpio_set_pulls(GPIO_NUM_GEAR_SHIFT, true, false);

    /* initialize button low battery */
    gpio_init(GPIO_NUM_LOW_BATTERY);
    gpio_set_dir(GPIO_NUM_LOW_BATTERY, GPIO_IN);
    gpio_set_pulls(GPIO_NUM_LOW_BATTERY, true, false);

    /* initialize button low oil pressure */
    gpio_init(GPIO_NUM_LOW_OIL_PRESSUIRE);
    gpio_set_dir(GPIO_NUM_LOW_OIL_PRESSUIRE, GPIO_IN);
    gpio_set_pulls(GPIO_NUM_LOW_OIL_PRESSUIRE, true, false);
}

void read_adc_and_button(struct display_data_t* display_data)
{
    //initialise buffers for adc
    uint16_t rpm_value     = 0UL;
    uint16_t oil_temp      = 0UL;
    uint16_t coolant_temp  = 0UL;

    //read the rpm potentiometer
    adc_select_input(ADC_INPUT_RPM);
    rpm_value = adc_read();

    //read the oil_temp potentiometer
    adc_select_input(ADC_INPUT_OIL_TEMP);
    oil_temp = adc_read();

    //read the coolant_temp potentiometer
    adc_select_input(ADC_INPUT_COOLANT_TEMP);
    coolant_temp = adc_read();

    /* Set the ADC data in the display_data structure, after normalizing it  */
    display_data->rpm                 = (uint16_t)((rpm_value * 13000) / ADC_RANGE);
    display_data->oil_temperature     = (uint8_t)((oil_temp * 170) / ADC_RANGE);
    display_data->coolant_temperature = (uint8_t)((coolant_temp * 120) / ADC_RANGE);

    /* If the gear shift button is pressed, increase the gear value */
    if ((gpio_get(GPIO_NUM_GEAR_SHIFT) == 0U) && (button_state_gearshift == 0U))
    {
        button_state_gearshift = 0xFF;

        display_data->current_gear++;

        if (display_data->current_gear >= 5U)
        {
            display_data->current_gear = 0U;
        }
    }
    /* This else branch is meant to protect the incrementing from happening
    unless the button is released */
    else if ((gpio_get(GPIO_NUM_GEAR_SHIFT) != 0U) && (button_state_gearshift == 0xFF))
    {
        button_state_gearshift = 0U;
    }


    /* If the low battery button is pressed, set the battery_voltage to 11V */
    if ((gpio_get(GPIO_NUM_LOW_BATTERY) == 0U) && (button_state_lowbattery == 0U))
    {
        button_state_lowbattery = 0xFF;

        display_data->battery_voltage = 110U;
    }
    /* This else branch is meant to protect the incrementing from happening
    unless the button is released */
    else if ((gpio_get(GPIO_NUM_LOW_BATTERY) != 0U) && (button_state_lowbattery == 0xFF))
    {
        button_state_lowbattery = 0x00;
        display_data->battery_voltage = 123U;
    }


    /* If the low oil pressure is pressed, set the oil_pressure to 0U */
    if ((gpio_get(GPIO_NUM_LOW_OIL_PRESSUIRE) == 0U) && (button_state_lowoilpressure == 0U))
    {
        button_state_lowoilpressure = 0xFF;

        display_data->oil_pressure = 0U;
    }
    /* This else branch is meant to protect the incrementing from happening
    unless the button is released */
    else if ((gpio_get(GPIO_NUM_LOW_OIL_PRESSUIRE) != 0U) && (button_state_lowoilpressure == 0xFF))
    {
        button_state_lowoilpressure = 0x00;
        display_data->oil_pressure = 100U;
    }
 }
