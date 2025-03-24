/**
 * \file        as5600_lib.h
 * \brief       AS5600 library
 * \details
 * 
 *          About the OUT pin in the AS5600 sensor:
 * The ADC of the ESP32 is connected to the OUT pin of the AS5600 sensor.
 * The OUT pin can be configured to output a 10%-90% (VCC) analog signal.
 * Since the ESP32 ADC can only read 0-3.3V, the VCC of the AS5600 sensor must be 3.3V.
 * But there is another problem. The characteristic graft of the ADC (Voltage vs. Digital Value) is not linear on all
 * the range (0-3.3V). It is linear only on the 5%-90% range, aproximately.
 * That is why the OUT pin must be configured to output a 10%-90% signal.
 * 
 * \author      MaverickST
 * \version     0.0.3
 * \date        05/10/2024
 * \copyright   Unlicensed
 */

#ifndef __AS5600_H__
#define __AS5600_H__

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "as5600_defs.h"
#include "platform_esp32s3.h"

#define VCC_3V3_MV          3300        /*!< VCC in mV */
#define VCC_3V3_MIN_RR_MV   330         /*!< VCC minimum range in mV -> 10% of VCC */
#define VCC_3V3_MAX_RR_MV   2970        /*!< VCC maximum range in mV -> 90% of VCC */

#define MAP(val, in_min, in_max, out_min, out_max) ((val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min) /*!< Map function */
#define ADC_TO_VOLTAGE(val) MAP(val, 0, AS5600_ADC_RESOLUTION_12_BIT, 0, VCC_3V3_MV) /*!< ADC to voltage conversion */
#define LIMIT(a, min, max) (a < min ? min : (a > max ? max : a)) /*!< Limit a value between min and max */

#define I2C_MASTER_FREQ_HZ  400*1000    /*!< I2C master clock frequency */
#define AS5600_SENSOR_ADDR  0x36        /*!< slave address for AS5600 sensor */

typedef struct
{
    AS5600_config_t conf; ///< AS5600 configuration
    AS5600_reg_t reg;

    // Peripheral handles
    i2c_t i2c_handle;
    adc_t adc_handle;

} AS5600_t;

/**
 * @brief Initialize the I2C master driver
 * 
 * @param i2c_num I2C port number
 */
void AS5600_Init(AS5600_t *as5600, i2c_port_t i2c_num, uint8_t scl, uint8_t sda, uint8_t out);

/**
 * @brief Deinitialize the I2C master driver
 * 
 */
void AS5600_Deinit(AS5600_t *as5600);

/**
 * @brief Get angle in degrees from the AS5600 sensor by ADC.
 * Also take into account the range of the OUT pin of the AS5600 sensor, which is 10%-90% of VCC.
 * 
 * @param as5600 
 */
float AS5600_ADC_GetAngle(AS5600_t *as5600);

/**
 * @brief The host microcontroller can perform a permanent programming of ZPOS and MPOS with a BURN_ANGLE command.
 * To perform a BURN_ANGLE command, write the value 0x80 into register 0xFF. 
 * The BURN_ANGLE command can be executed up to 3 times
 * ZMCO shows how many times ZPOS and MPOS have been permanently written. 
 * This command may only be executed if the presence of the magnet is detected (MD = 1).
 * 
 * @param as5600 
 */
void AS5600_BurnAngleCommand(AS5600_t *as5600);

/**
 * @brief The host microcontroller can perform a permanent writing of MANG and CONFIG with a BURN_SETTING command. 
 * To perform a BURN_SETTING command, write the value 0x40 into register 0xFF. 
 * MANG can be written only if ZPOS and MPOS have never been permanently written (ZMCO = 00). 
 * The BURN_ SETTING command can be performed only one time.
 * 
 * @param as5600 
 */
void AS5600_BurnSettingCommand(AS5600_t *as5600);

/**
 * @brief Convert register string to register address
 * 
 * @param reg_str Register string
 * @return as5600_reg_t Register address
 */
AS5600_reg_t AS5600_RegStrToAddr(AS5600_t *as5600, const char *reg_str);

// -------------------------------------------------------------
// ---------------------- I2C FUNCTIONS ------------------------
// -------------------------------------------------------------

/**
 * @brief Read register
 * 
 * @param reg Register address
 * @param data Pointer to the data
 */
void AS5600_ReadReg(AS5600_t *as5600, AS5600_reg_t reg, uint16_t *data);

/**
 * @brief Write register
 * 
 * @param reg Register address
 * @param data Data to write
 */
void AS5600_WriteReg(AS5600_t *as5600, AS5600_reg_t reg, uint16_t data);

/**
 * @brief Check if the register is valid for reading
 * 
 * @param reg Register address
 * @return true if the register is valid
 * @return false if the register is invalid
 */
bool AS5600_IsValidReadReg(AS5600_t *as5600, AS5600_reg_t reg);

/**
 * @brief Check if the register is valid for writing
 * 
 * @param reg Register address
 * @return true if the register is valid
 * @return false if the register is invalid
 */
bool AS5600_IsValidWriteReg(AS5600_t *as5600, AS5600_reg_t reg);

// -------------------------------------------------------------
// ---------------------- CONFIG REGISTERS ---------------------
// -------------------------------------------------------------

/**
 * @brief Set the start position by writing the ZPOS register
 * 
 * @param start_position 
 */
void AS5600_SetStartPosition(AS5600_t *as5600, uint16_t start_position);

/**
 * @brief Get the start position by reading the ZPOS register
 * 
 * @param start_position 
 */
void AS5600_GetStartPosition(AS5600_t *as5600, uint16_t *start_position);

/**
 * @brief Set the stop position by writing the MPOS register
 * 
 * @param stop_position 
 */
void AS5600_SetStopPosition(AS5600_t *as5600, uint16_t stop_position);

/**
 * @brief Get the stop position by reading the MPOS register
 * 
 * @param stop_position 
 */
void AS5600_GetStopPosition(AS5600_t *as5600, uint16_t *stop_position);

/**
 * @brief Set the maximum angle by writing the MANG register
 * 
 * @param max_angle 
 */
void AS5600_SetMaxAngle(AS5600_t *as5600, uint16_t max_angle);

/**
 * @brief Get the maximum angle by reading the MANG register
 * 
 * @param max_angle 
 */
void AS5600_GetMaxAngle(AS5600_t *as5600, uint16_t *max_angle);

/**
 * @brief Set the configuration by writing the CONF register
 * 
 * @param conf Configuration
 */
void AS5600_SetConf(AS5600_t *as5600, AS5600_config_t conf);

/**
 * @brief Get the configuration by reading the CONF register
 * 
 * @param conf Configuration
 */
void AS5600_GetConf(AS5600_t *as5600, AS5600_config_t *conf);


// -------------------------------------------------------------
// ---------------------- OUTPUT REGISTERS ---------------------
// -------------------------------------------------------------

/**
 * @brief Read RAW ANGLE register
 * 
 * @param reg buffer to store the register value
 * @param data Pointer to the data
 */
void AS5600_GetRawAngle(AS5600_t *as5600, uint16_t *raw_angle);

/**
 * @brief Read ANGLE register
 * 
 * @param reg buffer to store the register value
 * @param data Pointer to the data
 */
void AS5600_GetAngle(AS5600_t *as5600, uint16_t *angle);

// -------------------------------------------------------------
// ---------------------- STATUS REGISTERS ---------------------
// -------------------------------------------------------------

/**
 * @brief Read STATUS register
 * 
 * @param reg buffer to store the register value
 * @param data Pointer to the data
 */
void AS5600_GetStatus(AS5600_t *as5600, uint8_t *status);

/**
 * @brief Read AGC register
 * 
 * @param reg buffer to store the register value
 * @param data Pointer to the data
 */
void AS5600_GetAgc(AS5600_t *as5600, uint8_t *agc);

/**
 * @brief Read MAGNITUDE register
 * 
 * @param reg buffer to store the register value
 * @param data Pointer to the data
 */
void AS5600_GetMagnitude(AS5600_t *as5600, uint16_t *magnitude);


#endif // __AS5600_H__
