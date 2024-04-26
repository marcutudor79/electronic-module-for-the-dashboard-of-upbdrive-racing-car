/******************************************************************************/
/* UPBDRIVE Dashboard Firmware                                                */
/* All rights reserverd 2023                                                  */
/******************************************************************************/

#ifndef SDCARD_TASK_H
#define SDCARD_TASK_H

#include <common.h>
#include <string.h>
#include <esp_vfs_fat.h>
#include <sdmmc_cmd.h>

#include <sys/unistd.h>
#include <sys/stat.h>

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL VARIABLES                                 ////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL MACROS                                    ////
////////////////////////////////////////////////////////////////////////////////

/* Max char size */
#define MAX_CHAR_SIZE   (1024)

/* Mount point of the sdcard */
#define MOUNT_POINT "/sdcard"

////////////////////////////////////////////////////////////////////////////////
////                       GLOBAL CONSTANTS                                 ////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
////                       FUNCTION PROTOYPES                               ////
////////////////////////////////////////////////////////////////////////////////

/*
    * @brief This function will setup the sdcard on SPI bus
    *
    * @param void
    *
    * @return esp_err_t
*/
esp_err_t sdcard_setup(void);

/*
    * @brief This task will write a file to the sdcard and log the display data at 1 Hz
    *
    * @param[in] pvParameters
    *
    * @return void
*/
void sdcard_write(void *pvParameters);

#endif /* SDCARD_TASK_H */