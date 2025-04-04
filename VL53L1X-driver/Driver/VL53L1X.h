/**
 * \file        VL53L1X.h
 * \brief       VL53L1X driver library
 * \details     VL53L1X is a long distance ranging Time-of-Flight sensor, ranging up to 4 m and fast ranging frequency up to 50 Hz.
 * 
 * \author      Jose Rivera
 * \version     0.0.3
 * \date        20/10/2024
 * \copyright   Unlicensed
 */

#ifndef __VL53L1X_H__
#define __VL53L1X_H__

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "esp_timer.h"

#include "VL53L1X_types.h"
#include "platform_i2c_esp32s3.h"

#include "driver/gpio.h"
#include "esp_err.h"

//#define GPIO_INPUT_IO  18   // GPIO pin for interrupt
#define ESP_INTR_FLAG_DEFAULT 0


#define I2C_MASTER_FREQ_HZ  400000        /*!< I2C master clock frequency */
#define VL53L1X_SENSOR_ADDR  0x29         /*!< slave address for sensor */

static const char* TAG_VL53L1X = "VL53L1X";

static uint8_t TARGET_RATE[] = {0x0A,0x00};
static uint8_t GPIO__TIO_HV_STATUS_VALUE[] = {0x02};
static uint8_t SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS_VALUE[] = {8};
static uint8_t SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS_VALUE[] = {16};
static uint8_t ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM_VALUE[] = {0x01};
static uint8_t ALGO__RANGE_IGNORE_VALID_HEIGHT_MM_VALUE[] = {0xFF};
static uint8_t ALGO__RANGE_MIN_CLIP_VALUE[] = {0x0};
static uint8_t ALGO__CONSISTENCY_CHECK__TOLERANCE_VALUE[] = {2};

static uint8_t SYSTEM__THRESH_RATE_HIGH_VALUE[] = {0x00,0x00};
static uint8_t SYSTEM__THRESH_RATE_LOW_VALUE[] = {0x00,0x00};
static uint8_t DSS_CONFIG__APERTURE_ATTENUATION_VALUE[] = {0x38};

static uint8_t RANGE_CONFIG__SIGMA_THRESH_VALUE[] = {0x01,0x68};
static uint8_t RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS_VALUE[] = {0x00,0xC0};

static uint8_t SYSTEM__GROUPED_PARAMETER_HOLD_0_VALUE[] = {0x01};
static uint8_t SYSTEM__GROUPED_PARAMETER_HOLD_1_VALUE[] = {0x01};
static uint8_t SD_CONFIG__QUANTIFIER_VALUE[] = {2};

static uint8_t SYSTEM__GROUPED_PARAMETER_HOLD_VALUE[] = {0x00};
static uint8_t SYSTEM__SEED_CONFIG_VALUE[] = {1};

static uint8_t SYSTEM__SEQUENCE_CONFIG_VALUE[] = {0x8B};
static uint8_t DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT_VALUE[] = {0xC8,0x00};
static uint8_t DSS_CONFIG__ROI_MODE_CONTROL_VALUE[] = {2};

static const uint32_t TIMING_GUARD = 4528;

/**
 * @brief Distance modes structure
 * 
 */
typedef enum { Short, Medium, Long, Unknown }distanceMode_t;

/**
 * @brief Range status structure to know different kinds states 
 * 
 */
typedef enum{
      RangeValid                =   0,

      // "sigma estimator check is above the internal defined threshold"
      // (sigma = standard deviation of measurement)
      SigmaFail                 =   1,

      // "signal value is below the internal defined threshold"
      SignalFail                =   2,

      // "Target is below minimum detection threshold."
      RangeValidMinRangeClipped =   3,

      // "phase is out of bounds"
      // (nothing detected in range; try a longer distance mode if applicable)
      OutOfBoundsFail           =   4,

      // "HW or VCSEL failure"
      HardwareFail              =   5,

      // "The Range is valid but the wraparound check has not been done."
      RangeValidNoWrapCheckFail =   6,

      // "Wrapped target, not matching phases"
      // "no matching phase in other VCSEL period timing."
      WrapTargetFail            =   7,

      // "Internal algo underflow or overflow in lite ranging."
   // ProcessingFail            =   8: not used in API

      // "Specific to lite ranging."
      // should never occur with this lib (which uses low power auto ranging,
      // as the API does)
      XtalkSignalFail           =   9,

      // "1st interrupt when starting ranging in back to back mode. Ignore
      // data."
      // should never occur with this lib
      SynchronizationInt         =  10, // (the API spells this "syncronisation")

      // "All Range ok but object is result of multiple pulses merging together.
      // Used by RQL for merged pulse detection"
   // RangeValid MergedPulse    =  11: not used in API

      // "Used by RQL as different to phase fail."
   // TargetPresentLackOfSignal =  12:

      // "Target is below minimum detection threshold."
      MinRangeFail              =  13,

      // "The reported range is invalid"
   // RangeInvalid              =  14: can't actually be returned by API (range can never become negative, even after correction)

      // "No Update."
      None                      = 255,
    }rangeStatus_t;

/**
 * @brief Ranging Data structure, store relevant information about ranging measurement
 * 
 */
typedef struct
{
  uint16_t range_mm;
  rangeStatus_t range_status;
  float peak_signal_count_rate_MCPS;
  float ambient_count_rate_MCPS;
}rangingData_t;

/**
 * @brief Result Buffer structure, store the ranging registers information
 * 
 */
typedef struct
{
    uint8_t range_status;
    // uint8_t report_status: not used
    uint8_t stream_count;
    uint16_t dss_actual_effective_spads_sd0;
   // uint16_t peak_signal_count_rate_mcps_sd0: not used
    uint16_t ambient_count_rate_mcps_sd0;
   // uint16_t sigma_sd0: not used
   // uint16_t phase_sd0: not used
    uint16_t final_crosstalk_corrected_range_mm_sd0;
    uint16_t peak_signal_count_rate_crosstalk_corrected_mcps_sd0;
}resultBuffer_t;

/**
 * @brief VL53L1X sensor structure
 * 
 */
typedef struct
{
    // Peripheral handles
    distanceMode_t distance_mode;
    uint16_t fast_osc_frq;
    uint16_t osc_calibrate_val;

    resultBuffer_t results;
    rangingData_t ranging_data;

    i2c_t i2c_handle;
} vl53l1x_t;

static uint16_t timeout_start_ms = 0;
static uint16_t io_timeout = 0;
static bool did_time_out = 0;
static bool calibrated = false;
static uint8_t saved_vhv_init;
static uint8_t saved_vhv_timeout;

/**
 * @brief Set a new i2c address on the sensor
 * 
 * @param vl53l1x pointer to sensor structure
 * @param new_addr
 */
void VL53L1X_setAddress(vl53l1x_t *vl53l1x, uint8_t new_addr);

/**
 * @brief Initialize the VL53L1x driver
 * 
 * @param vl53l1x pointer to sensor structure
 * @param i2c_num 
 * @param gpio_scl 
 * @param gpio_sda 
 * @param io_2v8 1 to 2.8 volts operation mode, sensor uses 1V8 mode for I/O by default
 * 
 * @return true if the sensor is initialized correctly
 * @return false if the sensor initialization failed
 */
bool VL53L1X_init(vl53l1x_t *vl53l1x, i2c_port_t i2c_num, uint8_t scl, uint8_t sda, bool io_2v8);

/**
 * @brief Deinitialize the VL53L1x driver
 * 
* @param vl53l1x pointer to sensor structure
* @return true if the sensor is deinitialized correctly
* @return false if the sensor deinitialization failed
 */
bool VL53L1X_deinit(vl53l1x_t* sensor);

void VL53L1X_initIrq(gpio_num_t gpio_irq);
void gpio_isr_handler(void* arg);

/*!
 * @brief Distance mode is a parameter provided to optimize the internal settings and tunings to get
 * the best ranging performances
 * Modes:
 *    - short     Up to 1.3m  Better ambient immunity.
 *    - Medium    Up to 3m.
 *    - Long      Up to 4m    Maximun distance.
 * 
* @param vl53l1x pointer to sensor structure
* @param distance_mode Short(0) - Medium(1) - Long(2) - Unknown(3) 
* @return true if the sensor distance mode setting correctly
* @return false if the sensor distance mode setting failed
 */
bool VL53L1X_setDistanceMode(vl53l1x_t* vl53l1x,distanceMode_t distance_mode);

/**
 * @brief Get the measurement timing budget in microseconds
 * 
* @param vl53l1x pointer to sensor structure
 */
uint32_t VL53L1X_getMeasurementTimingBudget(vl53l1x_t* vl53l1x);

/**
 * @brief Set the measurement timing budget in microseconds
 * 
* @param vl53l1x pointer to sensor structure
 */
bool VL53L1X_setMeasurementTimingBudget(vl53l1x_t* vl53l1x, uint32_t budget_us);

/**
 * @brief Calculate macro period in microseconds (12.12 format) with given VCSEL period
 * 
 * @param vl53l1x pointer to sensor structure
 * @param vcsel_period 
 */
uint32_t VL53L1X_calcMacroPeriod(vl53l1x_t* vl53l1x, uint8_t vcsel_period);

/**
 * @brief Decode sequence step timeout in MCLKs from register value
 * 
 * @param reg_val
 */
uint32_t VL53L1X_decodeTimeout(uint16_t reg_val);

/**
 * @brief Convert sequence step timeout from macro periods to microseconds with given
 * 
 * @param timeout_mclks
 * @param macro_period_us
 */
uint32_t VL53L1X_timeoutMclksToMicroseconds(uint32_t timeout_mclks, uint32_t macro_period_us);

/**
 * @brief Convert sequence step timeout from microseconds to macro periods with given
 * 
 * @param timeout_us
 * @param macro_period_us
 */
uint32_t VL53L1X_timeoutMicrosecondsToMclks(uint32_t timeout_us, uint32_t macro_period_us);

/**
 * @brief Encode sequence step timeout register value from timeout in MCLKs
 * 
 * @param timeout_mclks
 * @param buff buffer to store operation result {MSB, LSB}
 */
void VL53L1X_encodeTimeout(uint32_t timeout_mclks, uint8_t* buff);

/**
 * @brief Start continuous ranging measurements, with the given inter-measurement
 * 
 * @param vl53l1x
 * @param period_ms period in milliseconds determining how often the sensor takes a measurement.
 */
void VL53L1X_startContinuous(vl53l1x_t* vl53l1x, uint32_t period_ms);

/**
 * @brief  Stop continuous measurements
 * 
 * @param vl53l1x
 */
void VL53L1X_stopContinuos(vl53l1x_t *vl53l1x);

/**
 * @brief Returns a range reading in millimeters when continuous mode is active. If
 *  blocking is true (the default), this function waits for a new measurement to
 *  be available. If blocking is false, it will try to return data immediately.
 *  (readSingle() also calls this function after starting a single-shot range
 *  measurement)
 * 
 * @param vl53l1x pointer to sensor structure
 * @param bloking
 */
uint16_t VL53L1X_readDistance(vl53l1x_t* vl53l1x, bool blocking);

/**
 * @brief read measurement results into buffer
 * 
 * @param vl53l1x pointer to sensor structure
 */
void VL53L1X_readResults(vl53l1x_t* vl53l1x);

/**
 * @brief Setup ranges after the first one in low power auto mode by turning off
 * FW calibration steps and programming static values
 * based on VL53L1_low_power_auto_setup_manual_calibration()
 * 
 * @param vl53l1x pointer to sensor structure
 */
void VL53L1X_setupManualCalibration(vl53l1x_t *vl53l1x);

/**
 * @brief perform Dynamic SPAD Selection calculation/update
 * 
 * @param vl53l1x pointer to sensor structure
 */
void VL53L1X_updateDSS(vl53l1x_t *vl53l1x);

/**
 * @brief get range, status, rates from results buffer
 * 
 * @param vl53l1x pointer to sensor structure
 */
void VL53L1X_getRangingData(vl53l1x_t *vl53l1x);

/**
 * @brief Returns a range reading in millimeters when continuous mode is active. If
 * blocking is true (the default), this function waits for a new measurement to
 * be available. If blocking is false, it will try to return data immediately.
 */
void VL53L1X_startTimeout();

/**
 * @brief check if sensor has new reading available
 * assumes interrupt is active low (GPIO_HV_MUX__CTRL bit 4 is 1)
 */
bool VL53L1X_dataReady(vl53l1x_t *vl53l1x);

/**
 * @brief Check if timeout is enabled (set to nonzero value) and has expired
 */
bool VL53L1X_checkTimeoutExpired();

/**
 * @brief Convert count rate from fixed point 9.7 format to float
 * 
 * @param count_rate_fixed
 */
float VL53L1X_countRateFixedToFloat(uint16_t count_rate_fixed);


//? Calibration functions

//bool VL53L1X_calibrateRefSPAD();

/**
 * @brief estimate the offset due to soldering the device on the customer board or adding a cover glass.
 * 
 * @param vl53l1x pointer to sensor structure
 * @param targetDistanceInMm the distance where the sensor start to "under range", normaly 140mm
 * @param foundOffset
 */

bool VL53L1X_calibrateOffset(vl53l1x_t *vl53l1x, uint16_t targetDistanceInMm, uint16_t *foundOffset);
/**
 * @brief estimate the amount of correction needed to compensate 
 * the effect of a cover glass added on top of the module.
 * due to the influence of the photons reflected back from the cover glass becoming strong. It's also called inflection point
 * 
 * @param vl53l1x pointer to sensor structure
 * @param targetDistanceInMm the distance where the sensor start to "under range", normaly 140mm
 * @param xtalk

 */
bool VL53L1X_calibrateXTalk(vl53l1x_t *vl53l1x, uint16_t targetDistanceInMm, uint16_t *foundXtalk);

/**
 * @brief transform a uint8 list into a uint16 type where data has the shape {MSL,LSB}.
 * 
 * @param data
 */
uint16_t VL53L1X_mergeData(uint8_t *data);

/**
 * @brief divide a given value into a list of bytes going from MSB to LSB 
 * 
 * @param buff
 * @param value
 * @param len
 */
void VL53L1X_divergeData(uint8_t *buff, uint32_t value, uint8_t len);

#endif

