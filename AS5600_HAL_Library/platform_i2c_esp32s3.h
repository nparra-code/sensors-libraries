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

#define I2C_TIMEOUT_MS 100

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
bool i2c_init(i2c_t *i2c, i2c_port_t i2c_num, uint8_t gpio_scl, uint8_t gpio_sda, uint32_t clk_speed_hz, uint16_t addr)
{
    esp_err_t ret = ESP_OK;

    i2c->addr = addr;
    i2c->clk_speed_hz = clk_speed_hz;
    i2c->i2c_num = i2c_num;
    i2c->gpio_scl = gpio_scl;
    i2c->gpio_sda = gpio_sda;

    // ------------- I2C master configuration ------------- //
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = i2c_num,
        .scl_io_num = gpio_scl,
        .sda_io_num = gpio_sda,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_GOTO_ON_ERROR(i2c_new_master_bus(&i2c_mst_config, &bus_handle), err, TAG_I2C, "i2c io to channel failed");

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = clk_speed_hz,
    };

    static i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
    i2c->dev_handle = dev_handle;

    return true;
err:
    return ret == ESP_OK;
}

/**
 * @brief Deinitialize the I2C master driver
 * 
 * @param i2c 
 */
void i2c_deinit(i2c_t *i2c)
{
    i2c_del_master_bus(i2c->bus_handle);
}

/**
 * @brief Given a register address, read the data from the register.
 * 
 * @param i2c 
 * @param reg 
 * @param data 
 * @param len 
 */
void i2c_read_reg(i2c_t *i2c, uint8_t reg, uint8_t *data, size_t len)
{
    uint8_t write_buffer[] = {reg};
    i2c_master_transmit_receive(i2c->dev_handle, write_buffer, 1, (uint8_t *)data, len, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief Given a register address, write the data to the register.
 * 
 * @param i2c 
 * @param reg 
 * @param data 
 * @param len 
 */
void i2c_write_reg(i2c_t *i2c, uint8_t reg, uint8_t *data, size_t len)
{
    uint8_t write_buffer[len + 1];
    write_buffer[0] = reg;
    memcpy(&write_buffer[1], data, len);
    i2c_master_transmit(i2c->dev_handle, write_buffer, len + 1, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief Write data to the I2C bus
 * 
 * @param i2c 
 * @param data 
 * @param len 
 */
void i2c_write(i2c_t *i2c, uint8_t *data, size_t len)
{
    i2c_master_transmit(i2c->dev_handle, data, len, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}


#endif // __HAL_I2C_ESP32__
