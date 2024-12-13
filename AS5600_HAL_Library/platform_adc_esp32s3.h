/**
 * \file        platform_adc_esp32s3.h
 * \brief
 * \details     ADC HAL for ESP32-S3
 * 
 * 
 * \author      MaverickST
 * \version     0.0.3
 * \date        05/12/2024
 * \copyright   Unlicensed
 */

#ifndef __HAL_ADC_ESP32S3__
#define __HAL_ADC_ESP32S3__


#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_check.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "sdkconfig.h"
#include "stdatomic.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_heap_caps.h"

#define ADC_CONF_UNIT           ADC_UNIT_1   /*!< ADC unit for ADC1 */
#define ADC_RESOLUTION_12_BIT   4095         /*!< 12-bit resolution for ADC */  

#define ADC_CONV_MODE           ADC_CONV_SINGLE_UNIT_1
#define ADC_OUTPUT_TYPE         ADC_DIGI_OUTPUT_FORMAT_TYPE2
#define ADC_ATTEN               ADC_ATTEN_DB_12
#define ADC_BIT_WIDTH           SOC_ADC_DIGI_MAX_BITWIDTH
#define ADC_CHANNEL_COUNT       1

static const char* TAG_ADC = "adc";

typedef struct
{
    uint8_t gpio_out;
    adc_channel_t chan;
    adc_unit_t unit;
    bool is_calibrated;

    adc_cali_handle_t adc_cali_handle;
    adc_oneshot_unit_handle_t adc_handle;
} adc_t;


/**
 * @brief Initialize the ADC driver
 * 
 * @param adc struct to store the ADC configuration
 * @param gpio_out GPIO pin connected to the ADC output
 * @return true if the ADC is initialized correctly
 * @return false if the ADC initialization failed
 */
bool adc_init(adc_t *adc, uint8_t gpio_out)
{
    esp_err_t ret = ESP_OK;

    adc->gpio_out = gpio_out;

    // From GPIO to ADC channel
    adc->unit = ADC_CONF_UNIT;
    ESP_GOTO_ON_ERROR(adc_oneshot_io_to_channel(adc->gpio_out, &adc->unit, &adc->chan), err, TAG_ADC, "adc io to channel failed");
    ESP_LOGI(TAG_ADC, "ADC channel: %d", adc->chan);

    // ------------- ADC pin OUT configuration ------------- //
    // The DIG ADC2 controller of ESP32-S3 doesn’t work properly (pag. 1444).
    // So we need to use the ADC1 controller (GPIO-1 to GPIO-10, pag. 318).

    // ADC Init
    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_CONF_UNIT,
    };
    ESP_GOTO_ON_ERROR(adc_oneshot_new_unit(&init_config1, &adc_handle), err, TAG_ADC, "adc init failed");

    // ADC Config
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BIT_WIDTH,
    };
    ESP_GOTO_ON_ERROR(adc_oneshot_config_channel(adc_handle, adc->chan, &config), err, TAG_ADC, "adc config failed");

    // ADC calibration 
    adc_cali_handle_t cali_handle = NULL;
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_CONF_UNIT,
        .chan = adc->chan,
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BIT_WIDTH,
    };
    adc->is_calibrated = false;
    ESP_GOTO_ON_ERROR(adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle), err, TAG_ADC, "adc calibration failed");
    
    adc->is_calibrated = true;
    adc->adc_cali_handle = cali_handle;
    adc->adc_handle = adc_handle;

    return true;

err:
    return ret == ESP_OK;
}

/**
 * @brief Deinitialize the ADC driver
 * 
 * @param adc struct to store the ADC configuration
 * @return true if the ADC is deinitialized correctly
 * @return false if the ADC deinitialization failed
 */
bool adc_deinit(adc_t *adc)
{
    esp_err_t ret = ESP_OK;
    ret = adc_oneshot_del_unit(adc->adc_handle);
    ESP_GOTO_ON_ERROR(ret, err, TAG_ADC, "adc oneshot del unit failed");
    ESP_GOTO_ON_ERROR(adc_cali_delete_scheme_curve_fitting(adc->adc_cali_handle), err, TAG_ADC, "adc cali delete scheme curve fitting failed");

    return true;
err:
    return false;
}

/**
 * @brief Read the raw ADC value. Range: 0 - 4095
 * 
 * @param adc 
 * @param raw 
 */
void adc_read_raw(adc_t *adc, int *raw)
{
    ESP_ERROR_CHECK(adc_oneshot_read(adc->adc_handle, adc->chan, raw));
}

/**
 * @brief Read the ADC value and convert it to millivolts
 * 
 * @param adc 
 * @param raw 
 * @param angle 
 */
void adc_read_mvolt(adc_t *adc, uint16_t *mvolt)
{
    int cali_result = 0;
    ESP_ERROR_CHECK(adc_oneshot_get_calibrated_result(adc->adc_handle, adc->adc_cali_handle, adc->chan, &cali_result));
    *mvolt = cali_result;
}



#endif // __HAL_ADC_ESP32S3__