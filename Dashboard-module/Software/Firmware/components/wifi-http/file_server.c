/******************************************************************************/
/* UPBDRIVE Dashboard Firmware                                                */
/* All rights reserverd 2023                                                  */
/******************************************************************************/

#include <common.h>

#include <file_server.h>
#include <esp_http_server.h>
#include <esp_vfs.h>
#include <image.h>

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL VARIABLES                                 ////
////////////////////////////////////////////////////////////////////////////////

extern status_firmware_t general_status;

////////////////////////////////////////////////////////////////////////////////
////                       LOCAL MACROS                                     ////
////////////////////////////////////////////////////////////////////////////////

/* Scratch buffer size */
#define SCRATCH_BUFSIZE     (8192)

#define FILE_PATH_MAX       (150)

////////////////////////////////////////////////////////////////////////////////
////                       LOCAL CONSTANTS                                  ////
////////////////////////////////////////////////////////////////////////////////

static const char *TAG = "file_server";

/* The index.html file is embedded into the binary file uploaded in the ESP32
   flash memory */
extern const char index_start[] asm ("_binary_index_html_start");

/* The path to dashboard log */
static const char *dashboard_log = MOUNT_POINT"/log.txt";

////////////////////////////////////////////////////////////////////////////////
////                       LOCAL VARIABLES                                 ////
////////////////////////////////////////////////////////////////////////////////

/*
    Allocate the file server data struct containing:
    - the base path of the files on the server, in this case MOUNT_POINT
    - a scratchpad for file transfer
*/
static struct file_server_data {
    /* Base path of file storage */
    char base_path[ESP_VFS_PATH_MAX + 1];

    /* Scratch buffer for temporary storage during file transfer */
    char scratch[SCRATCH_BUFSIZE];
} server_data;

static char http_response[200] = {0};

////////////////////////////////////////////////////////////////////////////////
////                       LOCAL FUNCTIONS                                  ////
////////////////////////////////////////////////////////////////////////////////

/* Handler to respond with index.html page */
static esp_err_t index_html_get_handler(httpd_req_t *req)
{
    return httpd_resp_send(req, index_start, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t logo_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "image/jpeg");
    return httpd_resp_send(req, image, image_len);
}

/* Handler to download a the log file from sdcard */
static esp_err_t download_get_handler(httpd_req_t *req)
{
    FILE *fd = NULL;
    /* Retrieve the pointer to scratch buffer for temporary storage */
    char *chunk = ((struct file_server_data *)req->user_ctx)->scratch;
    size_t chunksize;

    fd = fopen(dashboard_log, "r");
    if (NULL == fd)
    {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read existing file");

        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "text/plain");
    do {
        /* Read file in chunks into the scratch buffer */
        chunksize = fread(chunk, 1, SCRATCH_BUFSIZE, fd);

        if (chunksize > 0) {
            /* Send the buffer contents as HTTP response chunk */
            if (httpd_resp_send_chunk(req, chunk, chunksize) != ESP_OK) {
                fclose(fd);
                ESP_LOGE(TAG, "File sending failed!");
                /* Abort sending file */
                httpd_resp_sendstr_chunk(req, NULL);
                /* Respond with 500 Internal Server Error */
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send file");
               return ESP_FAIL;
           }
        }

        /* Keep looping till the whole file is sent */
    } while (chunksize != 0);

    /* Close file after sending complete */
    fclose(fd);
    ESP_LOGI(TAG, "File sending complete");

    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t data_get_handler(httpd_req_t *req)
{
    status_firmware_t* general_status = (status_firmware_t*)req->user_ctx;
    display_data_t* display_data = general_status->display_data;

    if (xSemaphoreTake (general_status->xSemaphore_display_data, ( TickType_t ) 10) == pdTRUE)
    {
        sprintf(http_response, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %f %f %f %d %d %d %d %d",
                display_data->rpm,
                display_data->coolant_temperature,
                display_data->oil_temperature,
                display_data->battery_voltage,
                display_data->can_status,
                display_data->hybrid_status,
                display_data->safety_circuit_status,
                display_data->fan_state,
                display_data->tps,
                display_data->brake_pressure_raw,
                display_data->map,
                display_data->lambda,
                display_data->tyre_pressure[0],
                display_data->tyre_pressure[1],
                display_data->tyre_pressure[2],
                display_data->tyre_pressure[3],
                display_data->accelerometer_g[0],
                display_data->accelerometer_g[1],
                display_data->accelerometer_g[2],
                general_status->time_hour,
                general_status->time_minute,
                general_status->time_second,
                general_status->sdcard_logging,
                display_data->hybrid_selector_value
                );

        xSemaphoreGive (general_status->xSemaphore_display_data);
    }

    return httpd_resp_send(req, http_response, HTTPD_RESP_USE_STRLEN);
}

/* Function to start the file server */
esp_err_t start_file_server(const char *base_path)
{
    httpd_handle_t http_server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    /* Use the URI wildcard matching function in order to
     * allow the same handler to respond to multiple different
     * target URIs which match the wildcard scheme */
    config.uri_match_fn = httpd_uri_match_wildcard;

    ESP_LOGI(TAG, "Starting HTTP Server on port: '%d'", config.server_port);
    if (httpd_start(&http_server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start file server!");
        return ESP_FAIL;
    }

    /* Start page URI */
    httpd_uri_t index_html_uri = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = index_html_get_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(http_server, &index_html_uri);

    /* Log download URI */
    httpd_uri_t log_download_uri = {
        .uri       = "/sdcard/log.txt",
        .method    = HTTP_GET,
        .handler   = download_get_handler,
        .user_ctx  = &server_data
    };
    httpd_register_uri_handler(http_server, &log_download_uri);

    /* Log live data URI */
    httpd_uri_t live_data_uri = {
        .uri       = "/data",
        .method    = HTTP_GET,
        .handler   = data_get_handler,
        .user_ctx  = &general_status
    };
    httpd_register_uri_handler(http_server, &live_data_uri);

    /* Image URI */
    httpd_uri_t image_uri = {
        .uri       = "/UPBDRIVE_Logo_Horizontal.jpg",
        .method    = HTTP_GET,
        .handler   = logo_get_handler,
        .user_ctx  = &general_status
    };
    httpd_register_uri_handler(http_server, &image_uri);

    return ESP_OK;
}