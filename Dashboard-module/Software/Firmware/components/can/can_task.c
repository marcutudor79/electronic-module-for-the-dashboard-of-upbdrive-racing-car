/******************************************************************************/
/* UPBDRIVE Dashboard Firmware                                                */
/* All rights reserverd 2023                                                  */
/******************************************************************************/

#include <can_task.h>

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL VARIABLES                                 ////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL FUNCTIONS                                 ////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////                       LOCAL MACROS                                     ////
////////////////////////////////////////////////////////////////////////////////

/*
    * @brief data structure:
    * data[0]: RPM / 100
    * data[1]: CurrGear
    * data[2]: TPS
    * data[3]: Oil pressure
    * data[4]: Water temperature
    * data[5]: Fuel pressure
    * data[6]: Lamda
    * data[7]: IAT
*/
#define CAN_PACKET_ECU_1 0x3E8

/*
    * @brief data structure:
    * data[0]: EGT1 temp / 100
    * data[1]: EGT2 temp / 100
    * data[2]: EGT3 temp / 100
    * data[3]: EGT4 temp / 100
    * data[4]: Vehicle speed
    * data[5]: Manifold Air Pressure (MAP)
    * data[6]: BPS raw value
    * data[7]: Oil temperature
*/
#define CAN_PACKET_ECU_2 0x3E9

/*
    * @brief data structure:
    * data[0]: LV battery voltage
    * data[1]: Input voltage
    * data[2]: Input current battery
    * data[3]: Input current alternator
*/
#define CAN_PACKET_PMU_1 0x3ED

/*
    * @brief data structure:
    * data[0]: Safety circuit status
*/
#define CAN_PACKET_EDU 0x3F0

/*
    * @brief data structure:
    * data[0]: Selector value
*/
#define CAN_PACKET_DASH 0x384
////////////////////////////////////////////////////////////////////////////////
////                       LOCAL CONSTANTS                                  ////
////////////////////////////////////////////////////////////////////////////////

esp_err_t can_setup(twai_mode_t can_mode, twai_timing_config_t *can_speed, twai_filter_config_t *packet_filter_config)
{
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_CAN_TX, GPIO_CAN_RX, can_mode);
    twai_timing_config_t  t_config;
    twai_filter_config_t  f_config;
    esp_err_t esp_response = ESP_FAIL;

    /* Copy the configuration parameters locally */
    memcpy(&t_config, can_speed,            sizeof(twai_timing_config_t));
    memcpy(&f_config, packet_filter_config, sizeof(twai_filter_config_t));

    /* Install the can driver */
    esp_response = twai_driver_install(&g_config, &t_config, &f_config);

    /* Check that the driver was installed succesfully */
    if (esp_response != ESP_OK)
    {
        return esp_response;
    }

    /* Start CAN driver */
    esp_response = twai_start();

    /* Check that the driver started succesfully */
    if (esp_response == ESP_OK)
    {
        return esp_response;
    }

    return esp_response;
}


void can_read(void *pvParameters)
{
    status_firmware_t *general_status         = (status_firmware_t*)pvParameters;
    display_data_t *display_data              = general_status->display_data;
    SemaphoreHandle_t xSemaphore_display_data = general_status->xSemaphore_display_data;
    twai_message_t message                    = {0};
    esp_err_t esp_response                    = ESP_FAIL;
    uint8_t counter_no_packet_recieved        = 0;

    while(true)
    {
        #ifdef ENABLE_CAN_READ_DEBUG
        printf("I'm in the can read task\n");
        #endif /* ENABLE_CAN_READ_DEBUG */

        esp_response = twai_receive(&message, CAN_POLLING_RATE);

        if (esp_response == ESP_OK)
        {
            #ifdef ENABLE_CAN_READ_DEBUG
            printf("Message received\n");
            printf("ID is %ld\n", message.identifier);
            if (!(message.rtr))
            {
                for (int i = 0; i < message.data_length_code; i++)
                {
                    printf("Data byte %d = %d\n", i, message.data[i]);
                }
            }
            #endif /* ENABLE_CAN_READ_DEBUG */


            if( xSemaphoreTake( xSemaphore_display_data, ( TickType_t ) 10 ) == pdTRUE )
            {
                /* Update display_data structure based on the CAN packet id */
                switch(message.identifier)
                {
                    case CAN_PACKET_ECU_1:
                    {
                        display_data->rpm                 = (uint16_t)message.data[0] * 100U;
                        display_data->current_gear        = message.data[1];
                        display_data->tps                 = message.data[2];
                        display_data->oil_pressure        = message.data[3];
                        display_data->coolant_temperature = message.data[4];
                        //display_data->fuel_pressure       = message.data[5];
                        display_data->lambda              = message.data[6];
                        display_data->iat                 = message.data[7];
                        break;
                    }

                    case CAN_PACKET_ECU_2:
                    {
                        display_data->egt[0]              = (uint16_t)((float)(message.data[0] & 0xF0U) + ((message.data[0] & 0x0FU) * 0.0625)) * 100U;
                        display_data->egt[1]              = (uint16_t)((float)(message.data[1] & 0xF0U) + ((message.data[1] & 0x0FU) * 0.0625)) * 100U;
                        display_data->egt[2]              = (uint16_t)((float)(message.data[2] & 0xF0U) + ((message.data[2] & 0x0FU) * 0.0625)) * 100U;
                        display_data->egt[3]              = (uint16_t)((float)(message.data[3] & 0xF0U) + ((message.data[3] & 0x0FU) * 0.0625)) * 100U;
                        display_data->map                 = message.data[5];
                        display_data->brake_pressure_raw  = message.data[6];
                        display_data->oil_temperature     = message.data[7];
                        break;
                    }

                    case CAN_PACKET_PMU_1:
                    {
                        display_data->battery_voltage         = message.data[0];
                        display_data->input_voltage_pmu       = message.data[1];
                        display_data->input_current_altr_pmu  = message.data[3];
                        break;
                    }

                    case CAN_PACKET_EDU:
                    {
                        display_data->safety_circuit_status = message.data[0];
                        break;
                    }

                    default:
                        break;
                }

                xSemaphoreGive( xSemaphore_display_data );

                /* If a packet was received, then the CAN bus is available */
                display_data->can_status   = true;
                /* Reset the fault counter */
                counter_no_packet_recieved = 0U;
            }
            else
            {
                vTaskDelay(CAN_POLLING_RATE);
            }
        }

        else if (esp_response == ESP_ERR_TIMEOUT)
        {
            /* No packet was received, increment the fault counter */
            counter_no_packet_recieved = counter_no_packet_recieved + 1;

            #ifdef ENABLE_CAN_READ_DEBUG
            printf("Timeout occurred, no message received\n");
            #endif /* ENABLE_CAN_READ_DEBUG */
        }

        else
        {
            /* Other error has occured, increment the fault counter */
            counter_no_packet_recieved = counter_no_packet_recieved + 1;

            #ifdef ENABLE_CAN_READ_DEBUG
            printf("Failed to receive message\n");
            continue;
            #endif /* ENABLE_CAN_READ_DEBUG */
        }

        /* If more than 200 packets were missed, then signal CAN bus FAULT */
        if (counter_no_packet_recieved == 200U )
        {
            display_data->can_status   = false;
            counter_no_packet_recieved = 200U;
        }

        vTaskDelay(CAN_POLLING_RATE);
    }
}

void can_send(void *pvParameters)
{
    esp_err_t esp_response                    = ESP_FAIL;
    status_firmware_t *general_status         = (status_firmware_t*)pvParameters;
    display_data_t *display_data              = general_status->display_data;
    SemaphoreHandle_t xSemaphore_display_data = general_status->xSemaphore_display_data;
    twai_message_t message                    = {0};

    while(true)
    {
        if( xSemaphoreTake( xSemaphore_display_data, ( TickType_t ) 10 ) == pdTRUE )
        {
            /* Send the data to the CAN bus */
            message.identifier       = CAN_PACKET_DASH;
            message.data_length_code = 1;
            message.data[0]          = display_data->hybrid_selector_value;
            message.rtr              = 0;
            message.flags            = 0;
            xSemaphoreGive( xSemaphore_display_data );
        }

        esp_response = twai_transmit(&message, CAN_TRANSMIT_RATE);
        if(esp_response != ESP_OK)
        {
            #ifdef ENABLE_CAN_SEND_DEBUG
            printf("Failed to send message with id 0x384\n");
            #endif /* ENABLE_CAN_SEND_DEBUG */
        }
        else
        {
            #ifdef ENABLE_CAN_SEND_DEBUG
            printf("Message with id 0x384 was sent\n");
            #endif /* ENABLE_CAN_SEND_DEBUG */
        }

        vTaskDelay(CAN_TRANSMIT_RATE);
    }
}