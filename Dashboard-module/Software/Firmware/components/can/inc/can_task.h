/******************************************************************************/
/* UPBDRIVE Dashboard Firmware                                                */
/* All rights reserverd 2023                                                  */
/******************************************************************************/

#ifndef CAN_TASK_H
#define CAN_TASK_H

#include <common.h>
#include <driver/gpio.h>
#include <driver/twai.h>
#include <string.h>

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL VARIABLES                                 ////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL MACROS                                    ////
////////////////////////////////////////////////////////////////////////////////

#define CAN_POLLING_RATE      (20/portTICK_PERIOD_MS)

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL CONSTANTS                                 ////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////                       FUNCTION PROTOYPES                               ////
////////////////////////////////////////////////////////////////////////////////

/*
    * @brief This function will setup the CAN bus
    *
    * @param[in] can_mode    IN configures if the CAN driver sends ack signals upon receiving a message or not
    *
    * @note: if the can_mode is set to TWAI_MODE_LISTEN_ONLY and there are not any other nodes on the bus, then the
    *           sender node will softlock itself in retrying to send their message
    *
    * @param[in] can_speed   IN sets the bitrate of the can bus
    * @param[in] packet_filter_config IN sets to filter certain messages
    *
    * @return esp_err_t
*/
esp_err_t can_setup(twai_mode_t can_mode, twai_timing_config_t *can_speed, twai_filter_config_t *packet_filter_config);

/*
    * @brief This task will read packets from the CAN bus
    *
    * @param[in] pvParameters IN
    *
    * @return void
*/
void can_read(void *pvParameters);


/*
    * @brief This task will send packets on the CAN bus
    *
    * @param[in] pvParameters IN
    *
    * @return void
*/
void can_send(void* pvParameters);



#endif /* CAN_TASK_H */