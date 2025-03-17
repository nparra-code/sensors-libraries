/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"

#include "vl6180x_api.h"

void app_main(void)
{
    i2c_t vl6180x_i2c;

    i2c_init(&vl6180x_i2c, I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_20, 100000, I2C_SLAVE_DEVICE_ADDRESS);

    VL6180xDev_t myDev = vl6180x_i2c;
    VL6180x_RangeData_t Range;

    vTaskDelay(10/portTICK_PERIOD_MS); // your code sleep at least 1 msec
    VL6180x_Prepare(myDev);

    while (1) {
        VL6180x_RangePollMeasurement(myDev, &Range);
        if (Range.errorStatus == 0 )
            ESP_LOGI("VL6180x", "Range %ld mm", Range.range_mm); // your code display range in mm
        else
            ESP_LOGE("VL6180x", "Range error %lu", Range.errorStatus);
    }
}
