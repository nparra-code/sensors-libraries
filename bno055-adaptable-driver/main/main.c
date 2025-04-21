#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "esp_log.h"

// Sensor libraries
#include "bno055.h"
#include "platform_esp32s3.h"
#include "led.h"

#define RAD_TO_DEG 57.2957795 // Conversion factor from radians to degrees

// Max brightness of the LED
#define MAX_BRIGHTNESS 5

BNO055_t bno055; // BNO055 sensor structure
BNO055_CalibProfile_t calib_data; // Calibration data structure
led_t led; // LED sreucture

void init_sensor() {
    
    // Initialize LED
    configure_led(&led);
    led_red(&led, MAX_BRIGHTNESS);

    // Initialize BNO055 sensor
    int8_t success = 0;
    success = BNO055_Init(&bno055, 17, 18);
    while (success != BNO055_SUCCESS) {
        printf("Error: Failed to initialize BNO055 sensor\n");
        vTaskDelay(pdMS_TO_TICKS(1000)); // Wait 1 second
        success = BNO055_Init(&bno055, 17, 18);
    }

    // Led orange while calibrating
    led_set_color(&led, MAX_BRIGHTNESS, MAX_BRIGHTNESS, 0); // Orange

    // Wait for the sensor to calibration routine
    // Get calibration status
    success = BNO055_GetCalibrationStatus(&bno055);
    while (success != BNO055_SUCCESS && bno055.calib_stat != BNO055_CALIB_STAT_OK) {
        printf("Error: Calibration status is not OK\n");
        vTaskDelay(pdMS_TO_TICKS(1000)); // Wait 1 second
        success = BNO055_GetCalibrationStatus(&bno055);
    }
    
    // Change the color of the LED to green if the sensor is initialized
    led_green(&led, MAX_BRIGHTNESS);

    success = BNO055_GetCalibrationProfile(&bno055, &calib_data); // Get calibration profile
    while (success != BNO055_SUCCESS) {
        printf("Error: Failed to get calibration profile\n");
        vTaskDelay(pdMS_TO_TICKS(1000)); // Wait 1 second
        success = BNO055_GetCalibrationProfile(&bno055, &calib_data);
    }
    
}

void print_sensor_data(void *pvParameters) {
    uint32_t timestamp = 1000000; // Start at 1 second
    float roll, pitch, yaw;
    float gx, gy, gz;
    float ax, ay, az;
    float mx, my, mz;
    

    int8_t success = 0; // Variable to check if the sensor read was successful

    while (true) {
        // Read All data from BNO055 sensor
        success = BNO055_ReadAll(&bno055);

        // If the sensor read was not successful, turn the LED red
        if (success != BNO055_SUCCESS)
        {
            led_red(&led, MAX_BRIGHTNESS);
        }
        else
        {
            led_green(&led, MAX_BRIGHTNESS);
        }
        

        // Data from the BNO055 sensor
        ax = bno055.ax;
        ay = bno055.ay;
        az = bno055.az;

        gx = bno055.gx;
        gy = bno055.gy;
        gz = bno055.gz;

        mx = bno055.mx;
        my = bno055.my;
        mz = bno055.mz;

        // Get Euler angles
        roll = bno055.roll;
        pitch = bno055.pitch;
        yaw = bno055.yaw;

        // Convert to degrees
        roll *= RAD_TO_DEG;
        pitch *= RAD_TO_DEG;
        yaw *= RAD_TO_DEG;

        // Invert yaw direction
        yaw = 360.0 - yaw;
      

        // Convert magnetic field au to uT (1au = 50uT)
        mx /= 50.0;
        my /= 50.0;
        mz /= 50.0;

        // accel in m/s^2 to g
        ax /= 9.81;
        ay /= 9.81;
        az /= 9.81;

        // Orientation message
        printf("A,%" PRIu32 ",%.4f,%.4f,%.4f\r\n", timestamp, roll, pitch, yaw);

        // Inertial message (gyroscope and accelerometer)
        printf("I,%" PRIu32 ",%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n", timestamp, gx, gy, gz, ax, ay, az);

        // Magnetometer message
        printf("M,%" PRIu32 ",%.4f,%.4f,%.4f\r\n", timestamp, mx, my, mz);
        
        // Increment timestamp by 1 second (1,000,000 microseconds)
        timestamp += 20000;
        
        vTaskDelay(pdMS_TO_TICKS(20)); // Wait 1 second
    }
}

void app_main() {
    init_sensor(); // Initialize BNO055 sensor
    xTaskCreate(print_sensor_data, "SensorTask", 4096, NULL, 1, NULL); // Create task to print sensor data
}
