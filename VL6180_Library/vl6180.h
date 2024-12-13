#ifndef VL6180_H
#define VL6180_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"

#define VL6180_I2C_ADDRESS 0x29

#define SYSTEM_FRESH_OUT_OF_RESET 0x16
#define SYSTEM_INTERRUPT_CONFIG_GPIO 0x14
#define SYSTEM_GROUPED_PARAMETER_HOLD 0x17
#define SYSRANGE_START 0x18
#define SYSRARANGE_THRESH_HIGH 0x19
#define SYSRARANGE_THRESH_LOW 0x1A
#define SYSRANGE_INTERMEASUREMENT_PERIOD 0x1B
#define SYSTEM_MODE_GPIO1 0x11
#define RESULT_RANGE_VAL 0x62


/**
 * @brief The conditions for the interrupt
 * 
 */
typedef enum {
    NEW_SAMPLE_READY = 0,
    LEVEL_LOW = 1,
    LEVEL_HIGH = 2,
    OUT_OF_WINDOW = 3,
} irq_condition_t;

/**
 * @brief Configuration structure for the interrupt
 * 
 * @param condition the condition for the interrupt
 * @param threshold_low the low threshold for the interrupt
 * @param threshold_high the high threshold for the interrupt
 * @param measurement_period the measurement period for the interrupt
 */
typedef struct {
    irq_condition_t condition;
    uint8_t threshold_low;
    uint8_t threshold_high;
    uint16_t measurement_period;
} irq_config_t;

/**
 * @brief Wait for the sensor to boot by checking the SYSTEM_FRESH_OUT_OF_RESET register
 * 
 * @return true if the sensor is booted
 */
bool wait_sensor_booted();

/**
 * @brief Initialize the VL6180 sensor
 * 
 * @return true if the sensor is initialized
 */
bool VL6180_Init();



/*<------------ INTERRUPT MODE ------------>*/

/**
 * @brief Get the range from the VL6180 sensor
 * 
 * @return uint8_t the range
 */
uint8_t VL6180_GetRange();

/**
 * @brief Set the interrupt configuration for the VL6180 sensor
 * 
 * @param config the configuration for the interrupt
 * @return true if the configuration was successful
 */
bool VL6180_SetInterruptConfig(irq_config_t *config);

#endif