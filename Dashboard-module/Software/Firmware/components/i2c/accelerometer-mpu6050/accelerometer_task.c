/******************************************************************************/
/* UPBDRIVE Dashboard Firmware                                                */
/* All rights reserverd 2023                                                  */
/******************************************************************************/

#include <accelerometer_task.h>

////////////////////////////////////////////////////////////////////////////////
////                       LOCAL VARIABLES                                 ////
////////////////////////////////////////////////////////////////////////////////

static i2c_port_t i2c_master_port = I2C_NUM_0;

////////////////////////////////////////////////////////////////////////////////
////                       LOCAL FUNCTIONS                                 ////
////////////////////////////////////////////////////////////////////////////////

static esp_err_t accelerometer_reset(void);

////////////////////////////////////////////////////////////////////////////////
////                       LOCAL MACROS                                     ////
////////////////////////////////////////////////////////////////////////////////

/* i2c BUS speed */
#define I2C_FREQ_400khz             (400 * 1000)

/* MPU6050 i2c address */
#define MPU6050_I2C_ADDRESS         (0x68)

/* MPU6050 register addresses */
/* Bits in MPU6050_PWR_MGMT_1
   [7]: DEVICE_RESET, when set to 1 resets all internal registers to default value
   [6]: SLEEP, when set to 1 MPU6050 enters into sleep mode
   [5]: CYCLE, when set to 1 MPU6050 cycles between sleep mode and waking up to
        take a single sample of data from active sensors
   [4]: - not used
   [3]: TEMP_DIS, when set to 1 disables the temperature sensor
   [2]: CLKSEL, specifies the clock source of the device, when set to 0 the internal
                oscillator of 8 MHz is used
   [1]: CLKSEL
   [0]: CLKSEL
*/
#define MPU6050_PWR_MGMT_1          (0x6B)
/* Bits in MPU6050_SIGNAL_PATH_RESET
   [7]: - not used
   [6]: - not used
   [5]: - not used
   [4]: - not used
   [3]: - not used
   [2]: GYRO_RESET, when set to 1 resets the gyro sens analog and digital parts
   [1]: ACCEL_RESET, when set to 1 resets the accel sens analog and digital parts
   [0]: TEMP_RESET, when set to 1 resets the temp sens analog and digital parts
*/
#define MPU6050_SIGNAL_PATH_RESET   (0x68)
/* Bits in MPU6050_ACCEL_CONFIG
   [7]: XA_ST, when set to 1 the self-test of the accel on X axis is enabled
   [6]: YA_ST, when set to 1 the self-test of the accel on Y axis is enabled
   [5]: ZA_ST, when set to 1 the self-test of the accel on Z axis is enabled
   [4]: AFS_SEL, when set to 00 full scale range is +/- 2g, 01 +/- 4g, 10- +/- 8g
   [3]: AFS_SEL, 11 +/- 16g
   [2]: - not used
   [1]: - not used
   [0]: - not used
*/
#define MPU6050_ACCEL_CONFIG        (0x1C)

/* Bits in MPU6050_ACCEL_XOUT_H
   MSB of X axis acceleration reading */
#define MPU6050_ACCEL_XOUT_H        (0x3B)

/* Accelerometer scale selection based on the MPU6050_ACCEL_SCALE macro
   defined in common/common.h header file */
#if (MPU6050_ACCEL_SCALE == 2)

#define MPU6050_ACCEL_SET_RANGE     (0b00000000)

/* RAW_TO_G macro divides the raw acceleration to 32768 since it is a int16 value
   which has a -32768 up to 32767 range */
#define RAW_TO_G(raw_acceleration)  ((raw_acceleration / 32768.0) * 2)

#elif (MPU6050_ACCEL_SCALE == 4)

#define MPU6050_ACCEL_SET_RANGE     (0b00001000)
#define RAW_TO_G(raw_acceleration)  ((raw_acceleration / 32768.0) * 4)

#elif (MPU6050_ACCEL_SCALE == 8)

#define MPU6050_ACCEL_SET_RANGE     (0b00010000)
#define RAW_TO_G(raw_acceleration)  ((raw_acceleration / 32768.0) * 8)

#elif (MPU6050_ACCEL_SCALE == 16)

#define MPU6050_ACCEL_SET_RANGE     (0b00011000)
#define RAW_TO_G(raw_acceleration)  ((raw_acceleration / 32768.0) * 16)

#endif /* MPU6050_ACCEL_RANGE */

/* MPU6050 trigger device reset register value*/
#define MPU6050_DEVICE_RESET        (0x80)
#define MPU6050_DEVICE_ENABLED      (0x00)

/* MPU6050 trigger gyro / accel / temp reset*/
#define MPU6050_GYRO_RESET          (0b00000100)
#define MPU6050_ACCEL_RESET         (0b00000010)
#define MPU6050_TEMP_RESET          (0b00000001)

/* Macro that returns the size of the stream to be sent */
#define STREAM_SIZE(transmit_stream) (sizeof(transmit_stream)/sizeof((transmit_stream)[0]))

////////////////////////////////////////////////////////////////////////////////
////                       LOCAL CONSTANTS                                  ////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////                       FUNCTION DEFINITIONS                             ////
////////////////////////////////////////////////////////////////////////////////

esp_err_t accelerometer_setup(void)
{
    esp_err_t esp_response             = ESP_FAIL;
    i2c_config_t configuration_options = {0};

    /* Setup ESP as master on the i2c bus */
    configuration_options.mode = I2C_MODE_MASTER;

    /* GPIO_NUM_21 is the only SDA pin on ESP32_WROOM_32 */
    configuration_options.sda_io_num    = GPIO_NUM_21;
    configuration_options.sda_pullup_en = GPIO_PULLUP_ENABLE;

    /* GPIO_NUM_22 is the only SCL pin on ESP32_WROOM_32 */
    configuration_options.scl_io_num    = GPIO_NUM_22;
    configuration_options.scl_pullup_en = GPIO_PULLUP_ENABLE;

    /* Setup the bus speed to 400 kHz, I2C fast mode */
    configuration_options.master.clk_speed = I2C_FREQ_400khz;

    /* No clk flags are used */
    configuration_options.clk_flags = 0;

    /* Configure the bus and install the driver */
    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &configuration_options));

    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, configuration_options.mode, 0, 0, 0));

    ESP_ERROR_CHECK(accelerometer_reset());

    esp_response = ESP_OK;
    return esp_response;
}

static esp_err_t accelerometer_reset(void)
{
    esp_err_t esp_response     = ESP_FAIL;
    uint8_t transmit_stream[2] = {MPU6050_PWR_MGMT_1, MPU6050_DEVICE_RESET};

    /***************************************************************************
     *                        MPU6050 reset sequence                           *
     *  1. Reset the mpu6050 peripheral                                        *
     *  2. Reset the signal path of acc, gyro and temp sensor in mpu6050       *
     *  3. Reset the mpu6050 peripheral again                                  *
    ***************************************************************************/

    /* Trigger a reset by setting PWR_MGMT_1 bit7 DEVICE_RESET bit  */
    ESP_ERROR_CHECK(i2c_master_write_to_device(i2c_master_port, MPU6050_I2C_ADDRESS, &transmit_stream[0], STREAM_SIZE(transmit_stream), pdMS_TO_TICKS(100)));

    /* Wait 100ms in order for the issued reset to complete */
    vTaskDelay(100 / portTICK_PERIOD_MS);

    transmit_stream[0] = MPU6050_SIGNAL_PATH_RESET;
    transmit_stream[1] = MPU6050_ACCEL_RESET | MPU6050_GYRO_RESET | MPU6050_TEMP_RESET;

    /* Trigger a reset of accelerometer, gyro and temp sensor on MPU6050 */
    ESP_ERROR_CHECK(i2c_master_write_to_device(i2c_master_port, MPU6050_I2C_ADDRESS, &transmit_stream[0], STREAM_SIZE(transmit_stream), pdMS_TO_TICKS(100)));

    /* Wait 100ms in order for the issued reset to complete */
    vTaskDelay(100 / portTICK_PERIOD_MS);

    transmit_stream[0] = MPU6050_PWR_MGMT_1;
    transmit_stream[1] = MPU6050_DEVICE_ENABLED;

    /* Finally, reset the PWR_MGMT_1 bit7 DEVICE_RESET bit 7 (it should automatically
       clear to 0, but this write forces it to 0) */
    ESP_ERROR_CHECK(i2c_master_write_to_device(i2c_master_port, MPU6050_I2C_ADDRESS, &transmit_stream[0], STREAM_SIZE(transmit_stream), pdMS_TO_TICKS(100)));

    /* Wait 100ms in order for the issued reset to complete */
    vTaskDelay(100 / portTICK_PERIOD_MS);

    /***************************************************************************
     *                       MPU6050 configuration sequence                    *
     * 1. Set the accelerometer range in the ACCEL_CONFIG register of mpu6050  *
    ***************************************************************************/

    transmit_stream[0] = MPU6050_ACCEL_CONFIG;

    /* The MPU6050_ACCEL_SET_RANGE macro is defined based on the settings from
       common/common.h header file */
    transmit_stream[1] = MPU6050_ACCEL_SET_RANGE;

    ESP_ERROR_CHECK(i2c_master_write_to_device(i2c_master_port, MPU6050_I2C_ADDRESS, &transmit_stream[0], STREAM_SIZE(transmit_stream), pdMS_TO_TICKS(100)));

    /* Wait 100ms in order for the configuratio to complete  */
    vTaskDelay(100 / portTICK_PERIOD_MS);

    esp_response = ESP_OK;
    return esp_response;
}

void accelerometer_read(void *pvParameters)
{
    status_firmware_t *general_status         = (status_firmware_t*)pvParameters;
    display_data_t *display_data              = general_status->display_data;
    SemaphoreHandle_t xSemaphore_display_data = general_status->xSemaphore_display_data;

    /* MPU6050 has 8 bit registers, but the acceleration resolution is on
        16bit for each axis */
    uint8_t read_stream[MPU6050_AXIS_NUM * 2] = {0};
    uint8_t transmit_stream                   = MPU6050_ACCEL_XOUT_H;
    while(true)
    {
        /* Start reading acceleration registers from register 0x3B for 6 bytes */
        ESP_ERROR_CHECK(i2c_master_write_read_device( i2c_master_port,
                                                    MPU6050_I2C_ADDRESS,
                                                    &transmit_stream,
                                                    1,
                                                    &read_stream[0],
                                                    STREAM_SIZE(read_stream),
                                                    MPU6050_POLLING_RATE));

        /* Catch MPU6050 error or entering sleep mode
        Typically it manifests in reading 0 on all accelerometer axis */
        if ((read_stream[0] == 0U) && (read_stream[1] == 0U)
            && (read_stream[2] == 0U) && (read_stream[3] == 0U))
        {
            accelerometer_reset();
            vTaskDelay(MPU6050_POLLING_RATE);
        }

        /* Check if the data structure is used by other tasks */
        if ( xSemaphoreTake(xSemaphore_display_data, portMAX_DELAY) == pdTRUE)
        {
            /* Combine the High and Low bytes from read_stream into a single variable  */
            for (uint_fast8_t i = 0; i < MPU6050_AXIS_NUM; i++)
            {
                *(display_data->accelerometer + i)   = (read_stream[i * 2] << 8 | read_stream[(i * 2) + 1]);
                *(display_data->accelerometer_g + i) = (float)RAW_TO_G((*(display_data->accelerometer + i)));
            }
            xSemaphoreGive(xSemaphore_display_data);
        }

        #ifdef ENABLE_DEBUG_ACCELEROMETER
        /* Transform the value in G's into a m/s^2 one */
        printf("X: %f m/s^2, Y: %f  m/s^2, Z: %f  m/s^2\n", (RAW_TO_G(*(display_data->accelerometer)) * 9.81), (RAW_TO_G(*(display_data->accelerometer+1)) * 9.81), (RAW_TO_G(*(display_data->accelerometer+2)) * 9.81));
        #endif /* ENABLE_DEBUG_ACCELEROMETER */

        vTaskDelay(MPU6050_POLLING_RATE);
    }
}
