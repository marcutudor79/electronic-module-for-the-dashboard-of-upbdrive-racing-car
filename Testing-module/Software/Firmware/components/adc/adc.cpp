/******************************************************************************/
/* UPBDRIVE ECU-Emulator Firmware                                             */
/* All rights reserverd 2023                                                  */
/******************************************************************************/

#include "inc/adc.h"

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL VARIABLES                                 ////
////////////////////////////////////////////////////////////////////////////////

static uint8_t button_state = 0;

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL FUNCTIONS                                 ////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////                       LOCAL MACROS                                     ////
////////////////////////////////////////////////////////////////////////////////

#define ADC_INPUT_POTENTIOMETER               (0)


////////////////////////////////////////////////////////////////////////////////
////                       LOCAL CONSTANTS                                  ////
////////////////////////////////////////////////////////////////////////////////

void init_adc_and_buttons()
{
    /* initialise ADC */
    adc_init();
    adc_gpio_init(GPIO_NUM_POTENTIOMETER);

    /* initialize button */
    gpio_init(GPIO_NUM_BUTTON);
    gpio_set_dir(GPIO_NUM_BUTTON, GPIO_IN);
    gpio_set_pulls(GPIO_NUM_BUTTON, true, false);
}

void read_adc_and_button(uint8_t *data) {

    //initialise buffers for adc
    uint16_t potentiometer = 0;
    uint8_t button         = 0;

    //read the tps potentiometer
    adc_select_input(ADC_INPUT_POTENTIOMETER);
    potentiometer = adc_read();

    //read the oil potentiometer
    button = gpio_get(GPIO_NUM_BUTTON);

    /* If the button is pressed, increase the CurrGear */
    if ((button == 0U) && (button_state == 0U))
    {
        *(data + 2) = *(data + 2) + 1;
        button_state = 0xFF;
    }
    else if (button != 0U && button_state == 0xFF)
    {
        button_state = 0U;
    }

    /* keep the currGear within 0-4 range */
    if (*(data + 2) > 4)
    {
        *(data + 2) = 0;
    }

    /* Set the adc data into the rpm slot */
    *data       = GET_MSB(potentiometer);
    *(data + 1) = GET_LSB(potentiometer);

 }
