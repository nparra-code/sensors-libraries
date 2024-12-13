#ifndef I2C_PLATFORM_H
#define I2C_PLATFORM_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"

/**
 * @brief Initialize the I2C bus
 * 
 * @param address the address of the device
 * @param frequency the frequency of the I2C bus
 */
void i2c0_init(uint8_t address, uint32_t frequency);

/**
 * @brief Read a byte from the I2C device
 * 
 * @param reg the register to read from
 * @return the byte read
 */
uint8_t i2c0_read_byte(uint8_t reg);

/**
 * @brief Write a byte to the I2C device
 * 
 * @param reg the register to write to
 * @param data the data to write
 */
void i2c0_write_byte(uint8_t reg, uint8_t data);

#endif