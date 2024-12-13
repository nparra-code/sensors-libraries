/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"


#include "bno055.h"
#include "platform_esp32s3.h"

// UART configuration structure



void app_main(void)
{
    int8_t success = -1;
    BNO055_t bno055;
    
    success = BNO055_Init(&bno055, 17, 18);

    float ax, ay, az;
    float gx, gy, gz;
    float yaw, pitch, roll;

    while (true)
    {
        // Read Accelerometer data
        // BNO055_GetAcceleration(&bno055, &ax, &ay, &az);

        // printf("Accelerometer data: X: %f, Y: %f, Z: %f\n", ax, ay, az);
        
        // Read Gyroscope data
        // BNO055_GetGyro(&bno055, &gx, &gy, &gz);

        // printf("Gyroscope data: X: %f, Y: %f, Z: %f\n", gx, gy, gz);

        // Read Euler angles
        // Yaw : Rotation around the Z-axis
        // Pitch : Rotation around the Y-axis
        // Roll : Rotation around the X-axis
        BNO055_GetEulerAngles(&bno055, &yaw, &pitch, &roll);

        printf("Euler angles: Roll: %f, Pitch: %f, Yaw: %f\n", roll, pitch, yaw);
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
    
    
}
