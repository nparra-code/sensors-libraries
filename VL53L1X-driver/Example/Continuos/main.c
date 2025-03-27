#include <VL53L1X.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define I2C_PORT 0
#define SDA_GPIO 21
#define SCL_GPIO 22

vl53l1x_t sensor;
void app_main(void){
    io_timeout = 500;
    if(!vl53l1x_init(&sensor,I2C_PORT,SCL_GPIO,SDA_GPIO,1)){
        ESP_LOGE(TAG_VL53L1X, "No se pudo inicializar el sensor");
        while(1){}
    }
    setDistanceMode(&sensor,Long);
    setMeasurementTimingBudget(&sensor,50000);
    startContinuous(&sensor,60);
    
    while(1){
        ESP_LOGI(TAG_VL53L1X, "Distance %dmm",read_distance(&sensor,1));
    }
}