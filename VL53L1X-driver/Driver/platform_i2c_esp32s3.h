/**
 * \file        platform_i2c_esp32s3.h
 * \brief
 * \details     I2C HAL for ESP32-S3
 * 
 * \author      MaverickST
 * \version     0.0.3
 * \date        05/12/2024
 * \copyright   Unlicensed
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
#include "freertos/FreeRTOS.h"

#define I2C_TIMEOUT_MS 500

static const char* TAG_I2C = "i2c";

/**
 * @brief I2C master driver structure
 * 
 */
typedef struct 
{
    uint8_t addr; // I2C device address
    uint32_t clk_speed_hz;

    i2c_port_t i2c_num; // I2C port number
    uint8_t gpio_scl;
    uint8_t gpio_sda;

    // I2C configuration structure for ESP32 - S3
    i2c_master_dev_handle_t dev_handle;
    i2c_master_bus_handle_t bus_handle;
} i2c_t;

/**
 * @brief Initialize the I2C master driver
 * 
 * @param i2c 
 * @param i2c_num 
 * @param gpio_scl 
 * @param gpio_sda 
 * @param clk_speed_hz 
 * @param addr 
 * @return true if the I2C is initialized correctly
 * @return false if the I2C initialization failed
 */
bool i2c_init(i2c_t *i2c, i2c_port_t i2c_num, uint8_t gpio_scl, uint8_t gpio_sda, uint32_t clk_speed_hz, uint16_t addr);

/**
 * @brief Deinitialize the I2C master driver
 * 
 * @param i2c 
 */
void i2c_deinit(i2c_t *i2c);
/**
 * @brief Given a register address, read the data from the register.
 * 
 * @param i2c 
 * @param reg 
 * @param data 
 * @param len 
 */
void i2c_read_reg(i2c_t *i2c, uint8_t reg, uint8_t *data, size_t len);
/**
 * @brief Given a 2 bytes register address, read the data from the register.
 * 
 * @param i2c 
 * @param reg 
 * @param data 
 * @param len 
 */
void i2c_read_reg16bits(i2c_t *i2c, uint16_t reg, uint8_t *data, size_t len);

/**
 * @brief Given a register address, write the data to the register.
 * 
 * @param i2c 
 * @param reg 
 * @param data 
 * @param len 
 */
void i2c_write_reg(i2c_t *i2c, uint8_t reg, uint8_t *data, size_t len);

/**
 * @brief Write data to the I2C bus
 * 
 * @param i2c 
 * @param data 
 * @param len 
 */
void i2c_write(i2c_t *i2c, uint8_t *data, size_t len);

/**
 * @brief Given a 2 bytes register address, write the data to the register.
 * 
 * @param i2c 
 * @param reg 
 * @param data 
 * @param len 
 */
void i2c_write_reg16bits(i2c_t *i2c, uint16_t reg, uint8_t *data, size_t len);

#endif // __HAL_I2C_ESP32__
