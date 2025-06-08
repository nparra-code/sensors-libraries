/**
 * @file main.c
 * @author MaverickST
 * @brief Main file for the AS5600 library
 * @details This file contains the main function and the calibration process for the AS5600 sensor.
 * @version 0.1
 * @date 2025-04-08
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <stdio.h>

#include "as5600_lib.h"

#define I2C_MASTER_SCL_GPIO 4       /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_GPIO 5       /*!< gpio number for I2C master data  */
#define AS5600_OUT_GPIO 6           /*!< gpio number for OUT signal */
#define I2C_MASTER_NUM 1            /*!< I2C port number for master dev */

AS5600_t gAs5600;

void app_main(void)
{

    ///< ---------------------- AS5600 -------------------
    AS5600_Init(&gAs5600, I2C_MASTER_NUM, I2C_MASTER_SCL_GPIO, I2C_MASTER_SDA_GPIO, AS5600_OUT_GPIO);

    // Set some configurations to the AS5600
    AS5600_config_t conf = {
        .PM = AS5600_POWER_MODE_NOM, ///< Normal mode
        .HYST = AS5600_HYSTERESIS_2LSB, ///< Hysteresis 2LSB
        .OUTS = AS5600_OUTPUT_STAGE_ANALOG_RR, ///< Analog output 10%-90%
        .PWMF = AS5600_PWM_FREQUENCY_115HZ, ///< PWM frequency 115Hz
        .SF = AS5600_SLOW_FILTER_8X, ///< Slow filter 8x
        .FTH = AS5600_FF_THRESHOLD_6LSB, ///< Fast filter threshold 6LSB
        .WD = AS5600_WATCHDOG_OFF, ///< Watchdog off
    };
    AS5600_SetConf(&gAs5600, conf);
    
    // Read the configuration
    uint16_t conf_reg;
    AS5600_ReadReg(&gAs5600, AS5600_REG_CONF_H, &conf_reg);
    printf("Configuration register readed: 0x%04X\n", conf_reg);
    printf("Configuration register written: 0x%04X\n", conf.WORD);

    ///< ------------- For calibration process. -------------
    AS5600_SetStartPosition(&gAs5600, 0x0000); ///< Set start position to 0 degrees
    AS5600_SetStopPosition(&gAs5600, 0x0FFF); ///< Set stop position to 360 degrees
    ///< -------------

    ///< Burn commands
    // AS5600_BurnSettingCommand(&gAs5600); TAKE CARE! This command just works once. It will burn the configuration to the EEPROM of the AS5600 sensor.
    // AS5600_BurnAngleCommand(&gAs5600); TAKE CARE! This command just works three times. It will burn the start and end angles to the EEPROM of the AS5600 sensor.
}
