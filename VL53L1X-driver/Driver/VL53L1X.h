/**
 * \file        VL53L1X.h
 * \brief       VL53L1X driver library
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
 * \author      Jose Rivera
 * \version     0.0.1
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

#define I2C_MASTER_FREQ_HZ  400000    /*!< I2C master clock frequency */
#define VL53L1X_SENSOR_ADDR  0x29        /*!< slave address for sensor */

static const char* TAG_VL53L1X = "VL53L1X";

static uint8_t TargetRate[] = {0x0A,0x00};
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

static const uint32_t TimingGuard = 4528;


typedef enum { Short, Medium, Long, Unknown }distanceMode_t;
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
    }RangeStatus_t;

typedef struct
{
  uint16_t range_mm;
  RangeStatus_t range_status;
  float peak_signal_count_rate_MCPS;
  float ambient_count_rate_MCPS;
}RangingData_t;

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

typedef struct
{
    // Peripheral handles
    distanceMode_t distanceMode;
    uint16_t fast_osc_frq;
    uint16_t osc_calibrate_val;

    resultBuffer_t results;
    RangingData_t rangingData;

    i2c_t i2c_handle;
} vl53l1x_t;

static uint16_t timeout_start_ms = 0;
static uint16_t io_timeout = 0;
static bool did_time_out = 0;
static bool calibrated = false;
static uint8_t saved_vhv_init;
static uint8_t saved_vhv_timeout;

bool vl53l1x_init(vl53l1x_t *vl53l1x, i2c_port_t i2c_num, uint8_t scl, uint8_t sda, bool io_2v8);
bool vl53l1x_deinit(vl53l1x_t* sensor);
bool setDistanceMode(vl53l1x_t* vl53l1x,distanceMode_t distanceMode);
uint32_t getMeasurementTimingBudget(vl53l1x_t* vl53l1x);
bool setMeasurementTimingBudget(vl53l1x_t* vl53l1x, uint32_t budget_us);
uint32_t calcMacroPeriod(vl53l1x_t* vl53l1x, uint8_t vcsel_period);
uint32_t decodeTimeout(uint16_t reg_val);
uint32_t timeoutMclksToMicroseconds(uint32_t timeout_mclks, uint32_t macro_period_us);
uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_us, uint32_t macro_period_us);
void encodeTimeout(uint32_t timeout_mclks, uint8_t* buff);
uint16_t read_distance(vl53l1x_t* vl53l1x, bool blocking);
void readResults(vl53l1x_t* vl53l1x);

void setupManualCalibration(vl53l1x_t *vl53l1x);
void updateDSS(vl53l1x_t *vl53l1x);
void getRangingData(vl53l1x_t *vl53l1x);

void startTimeout();
bool dataReady(vl53l1x_t *vl53l1x);
bool checkTimeoutExpired();
float countRateFixedToFloat(uint16_t count_rate_fixed);


uint16_t mergeData(uint8_t *data);
void deMergeData(uint8_t *buff, uint32_t value, uint8_t len);

void startContinuous(vl53l1x_t* vl53l1x, uint32_t period_ms);
#endif

