/******************************************************************************/
/* UPBDRIVE Dashboard Firmware                                                */
/* All rights reserverd 2023                                                  */
/******************************************************************************/

#include <timer_task.h>

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL VARIABLES                                 ////
////////////////////////////////////////////////////////////////////////////////

extern status_firmware_t general_status;

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
////                       LOCAL FUNCTIONS                                  ////
////////////////////////////////////////////////////////////////////////////////

static bool IRAM_ATTR timer_callback(void *arg)
{
    general_status.time_second += 1;

    if (general_status.time_second >= 60)
    {
        general_status.time_second = 0;
        general_status.time_minute += 1;
    }

    if (general_status.time_minute >= 60)
    {
        general_status.time_minute = 0;
        general_status.time_hour += 1;
    }

    return true;
}

esp_err_t timer_setup(void)
{
    gptimer_handle_t gptimer      = NULL;
    gptimer_config_t timer_config =
    {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000 // 1MHz, 1 tick=1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    gptimer_event_callbacks_t cbs =
    {
        .on_alarm = timer_callback
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));

    ESP_ERROR_CHECK(gptimer_enable(gptimer));

    gptimer_alarm_config_t alarm_config =
    {
        .reload_count = 0,
        .alarm_count = 1000000, // period = 1s
        .flags.auto_reload_on_alarm = true

    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));

    ESP_ERROR_CHECK(gptimer_start(gptimer));

    return ESP_OK;
}



