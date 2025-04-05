#include <stdio.h>

#include "as5600_lib.h"

#define I2C_MASTER_SCL_GPIO 4       /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_GPIO 5       /*!< gpio number for I2C master data  */
#define AS5600_OUT_GPIO 6           /*!< gpio number for OUT signal */
#define I2C_MASTER_NUM 1            /*!< I2C port number for master dev */

AS5600_t gAs5600;

///< ------------- For calibration process. -------------
///< Calibration process is made for esp32s3 only. The AS5600 sensor has a range of 10%-90% of VCC.
#include "esp_timer.h"
esp_timer_handle_t gOneshotTimer;
uint8_t cnt_cali; ///< Counter for the calibration process

/**
 * @brief Callback for the AS5600 sensor calibration
 * 
 * @param arg 
 */
void sensor_calibration_cb(void *arg);
///< -------------

void app_main(void)
{

    ///< ---------------------- AS5600 -------------------
    AS5600_Init(&gAs5600, I2C_MASTER_NUM, I2C_MASTER_SCL_GPIO, I2C_MASTER_SDA_GPIO, AS5600_OUT_GPIO);

    // Set some configurations to the AS5600
    AS5600_config_t conf = {
        .PM = AS5600_POWER_MODE_NOM, ///< Normal mode
        .HYST = AS5600_HYSTERESIS_OFF, ///< Hysteresis off
        .OUTS = AS5600_OUTPUT_STAGE_ANALOG_RR, ///< Analog output 10%-90%
        .PWMF = AS5600_PWM_FREQUENCY_115HZ, ///< PWM frequency 115Hz
        .SF = AS5600_SLOW_FILTER_16X, ///< Slow filter 16x
        .FTH = AS5600_FF_THRESHOLD_SLOW_FILTER_ONLY, ///< Slow filter only
        .WD = AS5600_WATCHDOG_ON, ///< Watchdog on
    };
    AS5600_SetConf(&gAs5600, conf);
    
    // Read the configuration
    uint16_t conf_reg;
    AS5600_ReadReg(&gAs5600, AS5600_REG_CONF_H, &conf_reg);
    printf("Configuration register readed: 0x%04X\n", conf_reg);
    printf("Configuration register written: 0x%04X\n", conf.WORD);


    ///< ------------- For calibration process. -------------
    // Create a one-shot timer to control the sequence
    const esp_timer_create_args_t oneshot_timer_args = {
        .callback = &sensor_calibration_cb,
        .arg = NULL, ////< argument specified here will be passed to timer callback function
        .name = "as5600_cali-one-shot" ///< name is optional, but may help identify the timer when debugging
    };
    esp_timer_handle_t oneshot_timer;
    ESP_ERROR_CHECK(esp_timer_create(&oneshot_timer_args, &oneshot_timer));
    gOneshotTimer = oneshot_timer;
    cnt_cali = 0; ///< Initialize the counter for the calibration process

    ESP_ERROR_CHECK(esp_timer_start_once(gOneshotTimer, 500*1000)); ///< Start the timer to calibrate the AS5600 sensor
    ESP_LOGI("app_main", "AS5600 calibration timer started");
    ///< -------------


    // Burn commands
    // AS5600_BurnSettingCommand(&gAs5600); TAKE CARE! This command just works once. It will burn the configuration to the EEPROM of the AS5600 sensor.
    // AS5600_BurnAngleCommand(&gAs5600); TAKE CARE! This command just works once. It will burn the start and end angles to the EEPROM of the AS5600 sensor.
}

void sensor_calibration_cb(void *arg)
{
    switch (cnt_cali) {
        case 0:
            printf("AS5600 calibration step 1. As step 4 in page 23 of the datasheet, move the magnet (or wheel) to the MAX position.\n");

            AS5600_DeinitGPIO(&gAs5600); ///< Deinitialize the GPIO driver
            cnt_cali++;
            esp_timer_start_once(gOneshotTimer, 5*1000*1000); ///< Start the timer to calibrate the AS5600 sensor
            break;
        case 1:
            printf("AS5600 calibration step 2\n");

            AS5600_InitGPIO(&gAs5600); ///< Initialize the GPIO driver
            AS5600_SetGPIO(&gAs5600, 0); ///< Set the GPIO pin to low (GND)
            cnt_cali++;
            esp_timer_start_once(gOneshotTimer, 500*1000); ///< Start the timer to calibrate the AS5600 sensor
            break;
        case 2:
            printf("AS5600 calibration step 3\n");

            AS5600_DeinitGPIO(&gAs5600); ///< Deinitialize the GPIO driver
            AS5600_InitADC(&gAs5600); ///< Initialize the ADC driver

            /**
             * @brief If the readed voltage is always 0, this indicates an error occurred during calibration.
             * 
             */
            float angle = AS5600_ADC_GetAngle(&gAs5600); ///< Get the angle from the ADC
            printf("angle-> %0.2f\n", angle);

            // Read n times the angle.
            for (int i = 0; i < 100; i++) {
                vTaskDelay(1000 / portTICK_PERIOD_MS); ///< Wait 1s
                angle = AS5600_ADC_GetAngle(&gAs5600); ///< Get the angle from the ADC
                printf("angle-> %0.2f\n", angle);
            }

            esp_timer_delete(gOneshotTimer); ///< Delete the timer
            break;
        default:
            printf("AS5600 calibration finished");
            break;
    }
}
