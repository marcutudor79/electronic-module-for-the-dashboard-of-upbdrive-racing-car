/******************************************************************************/
/* UPBDRIVE Dashboard Firmware                                                */
/* All rights reserverd 2023                                                  */
/******************************************************************************/

#include <sdcard_task.h>

////////////////////////////////////////////////////////////////////////////////
////                       LOCAL VARIABLES                                  ////
////////////////////////////////////////////////////////////////////////////////

/*
    Local structure used to store temporarly the data from the global
    structure display_data

    @note:  this is done in order to have the semaphore taken for
            as little as possible in order to not block other tasks
*/
static display_data_t log_data = {0};

extern status_firmware_t general_status;

////////////////////////////////////////////////////////////////////////////////
////                       LOCAL MACROS                                     ////
////////////////////////////////////////////////////////////////////////////////

#define COOLANT_LOG_POINT       (0)
#define OIL_PRESSURE_LOG_POINT  (1)
#define BATTERY_LOG_POINT       (2)
#define RPM_LOG_POINT           (3)
#define TPS_LOG_POINT           (4)
#define ACCEL_X_LOG_POINT       (5)
#define ACCEL_Y_LOG_POINT       (6)
#define ACCEL_Z_LOG_POINT       (7)

////////////////////////////////////////////////////////////////////////////////
////                       LOCAL CONSTANTS                                  ////
////////////////////////////////////////////////////////////////////////////////

static const char *TAG = "example";
static const char *dashboard_log = MOUNT_POINT"/log.txt";
static const char *log_name[]  = {  "COOLANT",
                                    "OIL PRESSURE",
                                    "BATTERY",
                                    "RPM",
                                    "TPS",
                                    "ACCEL_X",
                                    "ACCEL_Y",
                                    "ACCEL_Z"
                                };

////////////////////////////////////////////////////////////////////////////////
////                       LOCAL FUNCTIONS                                  ////
////////////////////////////////////////////////////////////////////////////////

static esp_err_t sdcard_write_log_point(FILE* log_file, uint8_t log_point, uint16_t log_value)
{
    esp_err_t esp_response   = ESP_FAIL;
    int16_t printed          = 0UL;

    printed = fprintf(log_file, "%d:%d:%d %s: %d\n",
                        general_status.time_hour,
                        general_status.time_minute,
                        general_status.time_second,
                        log_name[log_point],
                        log_value
                    );
    if (printed < 0)
    {
        ESP_LOGE(TAG, "Failed to write to file");

        /* Also signal an error on the webpage */
        general_status.sdcard_logging = false;
        return esp_response;
    }

    esp_response = ESP_OK;
    return esp_response;
}

static esp_err_t sdcard_write_file(const char *path, display_data_t* display_data)
{
    esp_err_t esp_response = ESP_FAIL;

    FILE *log_file = fopen(path, "a+");
    if (log_file == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for writing");

        general_status.sdcard_logging = false;
        esp_response = ESP_FAIL;
        return esp_response;
    }

    ESP_ERROR_CHECK(sdcard_write_log_point(log_file, COOLANT_LOG_POINT, display_data->coolant_temperature));
    ESP_ERROR_CHECK(sdcard_write_log_point(log_file, OIL_PRESSURE_LOG_POINT, display_data->oil_pressure));
    ESP_ERROR_CHECK(sdcard_write_log_point(log_file, BATTERY_LOG_POINT, display_data->battery_voltage));
    ESP_ERROR_CHECK(sdcard_write_log_point(log_file, RPM_LOG_POINT, display_data->rpm));
    ESP_ERROR_CHECK(sdcard_write_log_point(log_file, TPS_LOG_POINT, display_data->tps));
    ESP_ERROR_CHECK(sdcard_write_log_point(log_file, ACCEL_X_LOG_POINT, display_data->accelerometer[0]));
    ESP_ERROR_CHECK(sdcard_write_log_point(log_file, ACCEL_Y_LOG_POINT, display_data->accelerometer[1]));
    ESP_ERROR_CHECK(sdcard_write_log_point(log_file, ACCEL_Z_LOG_POINT, display_data->accelerometer[2]));

    fclose(log_file);
    ESP_LOGI(TAG, "File written");

    general_status.sdcard_logging = true;
    return ESP_OK;
}

esp_err_t sdcard_setup(void)
{
    esp_err_t esp_response                        = ESP_FAIL;
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {0};
    sdmmc_card_t *card                            = NULL;
    sdmmc_host_t host                             = SDSPI_HOST_DEFAULT();
    const char mount_point[]                      = MOUNT_POINT;
    spi_bus_config_t bus_cfg                      = {0};
    sdspi_device_config_t slot_config             = SDSPI_DEVICE_CONFIG_DEFAULT();

    mount_config.format_if_mount_failed           = false;
    mount_config.max_files                        = 5;
    mount_config.allocation_unit_size             = 16 * 1024;

    bus_cfg.intr_flags                            = 0;
    bus_cfg.isr_cpu_id                            = INTR_CPU_ID_AUTO;
    bus_cfg.flags                                 = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_GPIO_PINS;
    bus_cfg.mosi_io_num                           = GPIO_SDCARD_MOSI;
    bus_cfg.miso_io_num                           = GPIO_SDCARD_MISO;
    bus_cfg.sclk_io_num                           = GPIO_SDCARD_CLK;
    bus_cfg.quadwp_io_num                         = -1;
    bus_cfg.quadhd_io_num                         = -1;
    bus_cfg.max_transfer_sz                       = 4000;

    ESP_ERROR_CHECK(spi_bus_initialize((spi_host_device_t)host.slot, &bus_cfg, SDSPI_DEFAULT_DMA));

    slot_config.gpio_cs = GPIO_SDCARD_CS;
    slot_config.host_id = (spi_host_device_t)host.slot;
    ESP_ERROR_CHECK(esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card));

    esp_response = ESP_OK;
    return esp_response;
}

void sdcard_write(void *pvParameters)
{
    status_firmware_t   *general_status         = (status_firmware_t*)pvParameters;
    display_data_t      *display_data           = general_status->display_data;
    SemaphoreHandle_t   xSemaphore_display_data = general_status->xSemaphore_display_data;

    while(true)
    {
        /* Check if the RTC data was received on the CAN bus */
        while( general_status->time_hour == 99U && general_status->time_minute == 99U && general_status->time_second == 99U)
        {
            /* Stay in this loop */
        }

        if (xSemaphoreTake(xSemaphore_display_data, portMAX_DELAY) == pdTRUE)
        {
            /* Make a local copy of the global data structure display_data */
            memcpy(&log_data, display_data, sizeof(display_data_t));
            xSemaphoreGive(xSemaphore_display_data);
        }

        ESP_ERROR_CHECK(sdcard_write_file(dashboard_log, &log_data));

        vTaskDelay(SDCARD_LOGGING_RATE);
    }
}
