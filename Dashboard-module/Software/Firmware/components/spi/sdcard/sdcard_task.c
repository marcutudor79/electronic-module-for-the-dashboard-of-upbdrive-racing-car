/******************************************************************************/
/* UPBDRIVE Dashboard Firmware                                                */
/* All rights reserverd 2023                                                  */
/******************************************************************************/

#include <sdcard_task.h>

////////////////////////////////////////////////////////////////////////////////
////                       LOCAL VARIABLES                                 ////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////                       LOCAL MACROS                                     ////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////                       LOCAL CONSTANTS                                  ////
////////////////////////////////////////////////////////////////////////////////

static const char *TAG = "example";
static const char *dashboard_log = MOUNT_POINT"/log.txt";

////////////////////////////////////////////////////////////////////////////////
////                       LOCAL FUNCTIONS                                  ////
////////////////////////////////////////////////////////////////////////////////

static esp_err_t s_example_write_file(const char *path, char *data)
{
    ESP_LOGI(TAG, "Opening file %s", path);
    FILE *f = fopen(path, "a+");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return ESP_FAIL;
    }
    fprintf(f, data);
    fclose(f);
    ESP_LOGI(TAG, "File written");

    return ESP_OK;
}

static esp_err_t s_example_read_file(const char *path)
{
    ESP_LOGI(TAG, "Reading file %s", path);
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return ESP_FAIL;
    }
    char line[MAX_CHAR_SIZE];
    fgets(line, sizeof(line), f);
    fclose(f);

    // strip newline
    char *pos = strchr(line, '\n');
    if (pos) {
        *pos = '\0';
    }
    ESP_LOGI(TAG, "Read from file: '%s'", line);

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

    esp_response = spi_bus_initialize((spi_host_device_t)host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (esp_response != ESP_OK)
    {
        #ifdef ENABLE_DEBUG_SDCARD
        printf("Failed to initialize bus \n");
        #endif /* ENABLE_DEBUG_SDCARD */

        return esp_response;
    }

    slot_config.gpio_cs = GPIO_SDCARD_CS;
    slot_config.host_id = (spi_host_device_t)host.slot;
    esp_response = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);
    if (esp_response != ESP_OK)
    {
        return esp_response;
    }
    #ifdef ENABLE_DEBUG_SDCARD
    printf("Filesystem mounted");
    sdmmc_card_print_info(stdout, card);
    #endif /* ENABLE_DEBUG_SDCARD */

    char data[MAX_CHAR_SIZE];

    snprintf(data, MAX_CHAR_SIZE, "%s %u %s", "Setup was succesfull for sdcard with size ", (card->csd.capacity * card->csd.sector_size), " bytes\n");

    esp_response = s_example_write_file(dashboard_log, data);
    if (esp_response != ESP_OK) {
        return esp_response;
    }

    esp_response = ESP_OK;
    return esp_response;
}

void sdcard_write(void *pvParameters)
{
    esp_err_t           esp_response            = ESP_FAIL;
    status_firmware_t   *general_status         = (status_firmware_t*)pvParameters;
    display_data_t      *display_data           = general_status->display_data;
    SemaphoreHandle_t   xSemaphore_display_data = general_status->xSemaphore_display_data;
    uint8_t data[MAX_CHAR_SIZE]                 = {0};

    while(true)
    {
        if (xSemaphoreTake(xSemaphore_display_data, portMAX_DELAY) == pdTRUE)
        {
            snprintf((char *)&data[0], MAX_CHAR_SIZE,  "%s %u %s %s %u %s ",
                                           "RPM: "              ,display_data->rpm,                     "\n",
                                           "GEAR: "             ,display_data->current_gear,            "\n");
            xSemaphoreGive(xSemaphore_display_data);
        }

        esp_response = s_example_write_file(dashboard_log, (char *)&data[0]);
        if (esp_response != ESP_OK)
        {
            #ifdef ENABLE_DEBUG_SDCARD
            printf("Failed to write to the sdcard\n");
            #endif /* ENABLE_DEBUG_SDCARD */
        }

        vTaskDelay(SDCARD_LOGGING_RATE);
    }
}
