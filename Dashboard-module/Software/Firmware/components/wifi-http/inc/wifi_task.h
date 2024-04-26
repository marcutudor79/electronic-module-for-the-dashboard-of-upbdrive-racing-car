/******************************************************************************/
/* UPBDRIVE Dashboard Firmware                                                */
/* All rights reserverd 2023                                                  */
/******************************************************************************/

#ifndef WIFI_TASK_H
#define WIFI_TASK_H

#include <common.h>
#include <file_server.h>

#include <esp_mac.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include <esp_http_server.h>
#include <esp_random.h>
#include <esp_tls_crypto.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_err.h>


////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL VARIABLES                                 ////
////////////////////////////////////////////////////////////////////////////////

typedef struct {
    char    *username;
    char    *password;
} basic_auth_info_t;

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL MACROS                                    ////
////////////////////////////////////////////////////////////////////////////////

#define ESP_WIFI_CHANNEL 5
#define ESP_MAXIMUM_RETRY 10

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

#define EXAMPLE_HTTP_QUERY_KEY_MAX_LEN  (64)
#define HTTPD_401      "401 UNAUTHORIZED"           /*!< HTTP Response 401 */
#define MIN(a, b) (((a) < (b)) ? (a) : (b))

/* Mount point of the sdcard */
#define MOUNT_POINT "/sdcard"

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL CONSTANTS                                 ////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////                       FUNCTION PROTOYPES                               ////
////////////////////////////////////////////////////////////////////////////////

/*
    * @brief This function will setup the wifi access point to ESP32
    *
    * @param void
    *
    * @return void
*/
void wifi_setup(void);

/*
    * @brief This function will setup the http server on the ESP32 for
    *        file serving the log file on the SDCARD
    *
    * @param void
    *
    * @return esp_err_t
*/
esp_err_t http_server_setup(void);


#endif /* WIFI_TASK_H */