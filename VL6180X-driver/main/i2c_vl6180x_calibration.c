#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"

#include <vl6180x_api.h>

#define N_MEASURE_AVG   10
int Sample_InitForOffsetCalib(VL6180xDev_t myDev){
    int status = 1, init_status = 1;
    vTaskDelay(2000/portTICK_PERIOD_MS); // your code sleep at least 1 msec
    init_status = VL6180x_InitData(myDev);
    ESP_LOGE("VL6180x", "int_status %d", init_status);
    if(init_status == 0 || init_status == CALIBRATION_WARNING ){
        status = VL6180x_Prepare(myDev);
        if( !status )
            status=init_status; // if prepare is successfull return potential init warning
    }
    return status; 
}

int Sample_OffsetRunCalibration(VL6180xDev_t myDev)
{
    VL6180x_RangeData_t Range[N_MEASURE_AVG];
    int32_t   RangeSum;
    int status = 1;
    int i;
    int offset;
    int RealTargetDistance;
    /* Real target distance is 50 mm in proximity ranging configuration (scaling x1) or 100 mm in extended-range configuration */
    RealTargetDistance = (VL6180x_UpscaleGetScaling(myDev)==1) ? 50 : 100;
    
    /* Turn off wrap-around filter (to avoid first invalid distances and decrease number of I2C accesses at maximum) */
    VL6180x_FilterSetState(myDev, 0);
    /* Clear all interrupts */
    status = VL6180x_ClearAllInterrupt(myDev);
    if( status ){
        ESP_LOGE("VL6180x", "VL6180x_ClearAllInterrupt  fail");
    }
    /* Ask user to place a white target at know RealTargetDistance */
    ESP_LOGI("VL6180x", "Calibrating : place white target at %dmm", RealTargetDistance);
    
    /* Program a null offset */
    VL6180x_SetOffsetCalibrationData(myDev, 0);
    
    /* Perform several ranging measurement */
    for( i=0; i<N_MEASURE_AVG; i++){
        status = VL6180x_RangePollMeasurement(myDev, &Range[i]);
        if( status ){
            ESP_LOGE("VL6180x", "VL6180x_RangePollMeasurement  fail");
        }
        if( Range[i].errorStatus != 0 ){
            ESP_LOGE("VL6180x", "No target detect");
        }
    }
    
    /* Calculate ranging average (sum) */
    RangeSum=0;
    for( i=0; i<N_MEASURE_AVG; i++){
        RangeSum+= Range[i].range_mm;
    }
    
    /* Calculate part-to-part offset */
    offset = RealTargetDistance - (RangeSum/N_MEASURE_AVG);
    ESP_LOGI("VL6180x", "offset %d", offset);
    return offset;
}

uint8_t Sample_OffsetCalibrate(void) {
    i2c_t vl6180x_i2c;

    i2c_init(&vl6180x_i2c, I2C_NUM_0, 15, 16, 100000, 0x29);

    VL6180xDev_t myDev = vl6180x_i2c;
    VL6180x_RangeData_t Range;
    int offset;
    int status;
    /* init device */
    status = Sample_InitForOffsetCalib(myDev);
    if( status <0 ){
        ESP_LOGE("VL6180x", "Sample_InitForOffsetCalib  fail");
    }
    /* run calibration */
    offset = Sample_OffsetRunCalibration(myDev);
    /* when possible reset re-init device otherwise set back required filter */
    VL6180x_FilterSetState(myDev, 1);  // turn on wrap around filter again
    
    /* program offset */
    VL6180x_SetOffsetCalibrationData(myDev, offset);
    
    /* Perform ranging measurement to check */
    
    VL6180x_RangePollMeasurement(myDev, &Range);
    if (Range.errorStatus == 0 )
        ESP_LOGI("VL6180x", "Range %ld mm", Range.range_mm);
    else
        ESP_LOGE("VL6180x", "Range error %lu", Range.errorStatus);
    
    return offset;
}

void app_main(void)
{
    // i2c_t vl6180x_i2c;

    // i2c_init(&vl6180x_i2c, I2C_NUM_1, 15, 16, 400000, 0x29);

    // uint8_t buff[2] = {0x000, 0x000}, data[1];
    
    // //i2c_write(vl6180x_i2c, buff, 2);
    // // i2c_master_transmit((&vl6180x_i2c)->dev_handle, buff, 2, -1);

    // if (i2c_master_transmit_receive(vl6180x_i2c.dev_handle, buff, 2, data, 1, I2C_TIMEOUT_MS / portTICK_PERIOD_MS) == ESP_OK) {
    //     printf("Read: %x\n", data[0]);
    // }
    Sample_OffsetCalibrate();

}