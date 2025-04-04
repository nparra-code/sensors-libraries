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
    if(!VL53L1X_init(&sensor,I2C_PORT,SCL_GPIO,SDA_GPIO,0)){
        ESP_LOGE(TAG_VL53L1X, "No se pudo inicializar el sensor");
        while(1){}
    }
    
    uint16_t offset, xtalk;
    
    // Place a target, 17 % gray, at 140 mm from the sensor
    VL53L1X_calibrateOffset(&sensor,140,&offset);
    ESP_LOGI(TAG_I2C,"Offset: %d",offset);

    VL53L1X_calibrateXTalk(&sensor,140,&xtalk);
    ESP_LOGI(TAG_I2C,"Xtalk: %d",xtalk);
    
    VL53L1X_setDistanceMode(&sensor,Short);
    VL53L1X_setMeasurementTimingBudget(&sensor,20000);
    VL53L1X_startContinuous(&sensor,30);
    while(1){
        ESP_LOGI(TAG_VL53L1X, "Distance %dmm",VL53L1X_readDistance(&sensor,1));
    }
}