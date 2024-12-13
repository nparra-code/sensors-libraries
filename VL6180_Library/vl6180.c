#include "vl6180.h"
#include "i2c_api.h"

bool wait_sensor_booted() {
    // Reads SYSTEM_FRESH_OUT_OF_RESET register to check if the sensor is booted
    uint8_t frsh;
    do {
        frsh = i2c0_read_byte(SYSTEM_FRESH_OUT_OF_RESET);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    } while (frsh != 0x01);
    return true;
}

bool VL6180_Init(){
    i2c0_init(VL6180_I2C_ADDRESS, 400*1000);

    ESP_LOGI("VL6180x", "Waiting for sensor to boot");
    wait_sensor_booted();
    
    i2c0_write_byte(0x003F, 0x46); // Sets the light and dark gain

    return true;
}

uint8_t VL6180_GetRange(){

    return i2c0_read_byte(RESULT_RANGE_VAL); // Read the range value

}

bool VL6180_SetInterruptConfig(irq_config_t *config) {

    /**
     * The following block of code configures the measurement period
     * The measurement period is in 10ms units, so the maximum value is 2550ms
     * The minimum value is 10ms */
    if (config->measurement_period > 2550)
        return 0;
    else if (config->measurement_period < 10)
        config->measurement_period = 10;

    uint8_t period = config->measurement_period / 10;
    i2c0_write_byte(SYSRANGE_INTERMEASUREMENT_PERIOD, period); // Set the measurement period

    /**
     * The following block of code configures the interrupt pin
     * It configures GPIO 1 as the interrupt pin
     */
    i2c0_write_byte(SYSTEM_MODE_GPIO1, 0x20 | (0x08 << 1));

    
    /**
     * The following block of code configures the threshold values for the interrupt
     * The threshold values are in mm units, so the maximum value is 255mm
     */
    i2c0_write_byte(SYSTEM_GROUPED_PARAMETER_HOLD, 0x01); // Hold the parameter
    switch(config->condition) {
        case(NEW_SAMPLE_READY):
            i2c0_write_byte(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x4); // Configures interrupt on new sample ready
            break;
        case(LEVEL_LOW):
            i2c0_write_byte(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x1); // Set the interrupt on level low
            i2c0_write_byte(SYSRARANGE_THRESH_LOW, config->threshold_low); // Set the low threshold
            break;
        case(LEVEL_HIGH):
            i2c0_write_byte(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x2); // Set the interrupt on level high
            i2c0_write_byte(SYSRARANGE_THRESH_HIGH, config->threshold_high); // Set the high threshold
            break;
        case(OUT_OF_WINDOW):
            i2c0_write_byte(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x3); // Configures interrupt on new sample ready
            i2c0_write_byte(SYSRARANGE_THRESH_LOW, config->threshold_low); // Set the low threshold
            i2c0_write_byte(SYSRARANGE_THRESH_HIGH, config->threshold_high); // Set the high threshold
            break;
        default:
            return 0;
    }
    i2c0_write_byte(SYSTEM_GROUPED_PARAMETER_HOLD, 0x00); // Release the parameter

    i2c0_write_byte(SYSRANGE_START, 0x01 | 0x02); // Start the measurement

    return 1;
}