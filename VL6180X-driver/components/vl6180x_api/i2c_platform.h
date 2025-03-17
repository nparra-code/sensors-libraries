/**
 * @file        esp32s3_i2c_platform.h
 * @author      Nelson Parra (nelson.parra@udea.edu.co)
 * @brief       This file implements the I2C master driver for the ESP32-S3
 * @date        2024-15-09

 * @copyright   Unlicensed
 */

#ifndef __HAL_I2C_ESP32S3__
#define __HAL_I2C_ESP32S3__

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"

#define I2C_TIMEOUT_MS 100

/**
 * @brief I2C master driver structure
 * 
 */
typedef struct 
{
    uint8_t addr;           // I2C device address
    uint32_t clk_speed_hz;  // I2C clock speed

    i2c_port_t i2c_num;     // I2C port number
    uint8_t gpio_scl;       // GPIO pin for SCL
    uint8_t gpio_sda;       // GPIO pin for SDA

    i2c_master_dev_handle_t dev_handle;
    i2c_master_bus_handle_t bus_handle;
} i2c_t;

/**
 * @brief Initialize the I2C master driver
 * 
 * @param i2c Device to initialize
 * @param i2c_num I2C port number (I2C_NUM_0 or I2C_NUM_1)
 * @param gpio_scl GPIO pin for SCL
 * @param gpio_sda GPIO pin for SDA
 * @param clk_speed_hz Clock speed in Hz
 * @param addr Address of the I2C slave device
 * @return true if the I2C is initialized correctly
 * @return false if the I2C initialization failed
 */
bool i2c_init(i2c_t *i2c, i2c_port_t i2c_num, uint8_t gpio_scl, uint8_t gpio_sda, uint32_t clk_speed_hz, uint16_t addr);

/**
 * @brief Deinitialize the I2C master driver
 * 
 * @param i2c Device to deinitialize
 * @return esp_err_t (int)
 */
esp_err_t i2c_deinit(i2c_t *i2c);

/**
 * @brief Read the data.
 * 
 * @param i2c I2C device to read from
 * @param data Buffer to store the data
 * @param len Length of the data to read
 * @return esp_err_t (int)
 */
esp_err_t i2c_read(i2c_t i2c, uint8_t *data, size_t len);

/**
 * @brief Write data to the I2C bus.
 * 
 * @param i2c  I2C device to write to
 * @param data Data to write
 * @param len  Length of the data to write
 * @return esp_err_t (int)
 */ 
esp_err_t i2c_write(i2c_t i2c, uint8_t *data, size_t len);


#endif // __HAL_I2C_ESP32__
