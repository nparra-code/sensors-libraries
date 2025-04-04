#include "VL53L1X.h"

void setAddress(vl53l1x_t *vl53l1x, uint8_t new_addr)
{
  new_addr = new_addr & 0x7F;
  i2c_write_reg16bits(&vl53l1x->i2c_handle,I2C_SLAVE__DEVICE_ADDRESS, &new_addr, 1);
  vl53l1x->i2c_handle.addr = new_addr;
}

bool VL53L1X_init(vl53l1x_t *vl53l1x, i2c_port_t i2c_num, uint8_t scl, uint8_t sda, bool io_2v8)
{
    //  I2C master configuration 
    if (!i2c_init(&vl53l1x->i2c_handle, i2c_num, scl, sda, I2C_MASTER_FREQ_HZ, VL53L1X_SENSOR_ADDR)) {
        ESP_LOGI(TAG_VL53L1X, "I2C initialization failed");
        return 0;
    }
    //ESP_LOGI(TAG_VL53L1X, "I2C initialization complete");
    // Check status
    uint8_t data[2];

    // Check model ID register and compare with values specified in datasheet
    i2c_read_reg16bits(&vl53l1x->i2c_handle,(IDENTIFICATION__MODEL_ID),data,2); //Check Model register 
    if (VL53L1X_mergeData(data) != 0xEACC) { return false; }
    ESP_LOGI(TAG_VL53L1X, "IDENTIFICATION__MODEL_ID: %x",VL53L1X_mergeData(data));

    // VL53L1_software_reset() begin
    data[0] = 0x0;
    i2c_write_reg16bits(&vl53l1x->i2c_handle,SOFT_RESET,data,1);
    vTaskDelay(1/portTICK_PERIOD_MS);

    data[0] = 0x1;
    i2c_write_reg16bits(&vl53l1x->i2c_handle,SOFT_RESET,data,1);
    ESP_LOGI(TAG_VL53L1X, "SOFT RESERT complete");

    vTaskDelay(1/portTICK_PERIOD_MS);

    VL53L1X_startTimeout();
    // check last_status in case we still get a NACK to try to deal with it correctly
    i2c_read_reg16bits(&vl53l1x->i2c_handle,FIRMWARE__SYSTEM_STATUS,data,1);
    while((data[0] & 0x01) == 0){
      if(VL53L1X_checkTimeoutExpired()){
        did_time_out = true;
        return false;
      }
      ESP_LOGI(TAG_VL53L1X,"FIRMWARE STATUS are 0");
      i2c_read_reg16bits(&vl53l1x->i2c_handle,FIRMWARE__SYSTEM_STATUS,data,1);
    }

    // sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
    if(io_2v8){
      ESP_LOGI(TAG_VL53L1X,"2V8 mode");
      i2c_read_reg16bits(&vl53l1x->i2c_handle,PAD_I2C_HV__EXTSUP_CONFIG,data,1);
      data[0] = data[0] | 0x01;
      i2c_write_reg16bits(&vl53l1x->i2c_handle,PAD_I2C_HV__EXTSUP_CONFIG,data,1);
    }

    // store oscillator info for later use
    i2c_read_reg16bits(&vl53l1x->i2c_handle,OSC_MEASURED__FAST_OSC__FREQUENCY,data,2);
    vl53l1x->fast_osc_frq = VL53L1X_mergeData(data);

    i2c_read_reg16bits(&vl53l1x->i2c_handle,RESULT__OSC_CALIBRATE_VAL,data,2);
    vl53l1x->osc_calibrate_val = VL53L1X_mergeData(data);

    //ESP_LOGI(TAG_VL53L1X,"FO_frq: %x,%x",vl53l1x->fast_osc_frq,vl53l1x->osc_calibrate_val);

    //todo Static Config
    i2c_write_reg16bits(&vl53l1x->i2c_handle,DSS_CONFIG__TARGET_TOTAL_RATE_MCPS,TARGET_RATE,2);
    
    i2c_write_reg16bits(&vl53l1x->i2c_handle,GPIO__TIO_HV_STATUS,GPIO__TIO_HV_STATUS_VALUE,1);
    
    i2c_write_reg16bits(&vl53l1x->i2c_handle,SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS,SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS_VALUE,1);
    i2c_write_reg16bits(&vl53l1x->i2c_handle,SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS,SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS_VALUE,1);
    
    i2c_write_reg16bits(&vl53l1x->i2c_handle,ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM,ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM_VALUE,1);
    
    i2c_write_reg16bits(&vl53l1x->i2c_handle,ALGO__RANGE_IGNORE_VALID_HEIGHT_MM,ALGO__RANGE_IGNORE_VALID_HEIGHT_MM_VALUE,1);
    i2c_write_reg16bits(&vl53l1x->i2c_handle,ALGO__RANGE_MIN_CLIP,ALGO__RANGE_MIN_CLIP_VALUE,1);

    i2c_write_reg16bits(&vl53l1x->i2c_handle,ALGO__CONSISTENCY_CHECK__TOLERANCE,ALGO__CONSISTENCY_CHECK__TOLERANCE_VALUE,1);
    
    //todo General Config
    i2c_write_reg16bits(&vl53l1x->i2c_handle,SYSTEM__THRESH_RATE_HIGH,SYSTEM__THRESH_RATE_HIGH_VALUE,2);
    i2c_write_reg16bits(&vl53l1x->i2c_handle,SYSTEM__THRESH_RATE_LOW,SYSTEM__THRESH_RATE_LOW_VALUE,2);
    i2c_write_reg16bits(&vl53l1x->i2c_handle,DSS_CONFIG__APERTURE_ATTENUATION,DSS_CONFIG__APERTURE_ATTENUATION_VALUE,1);

    //todo Timing Config
    i2c_write_reg16bits(&vl53l1x->i2c_handle,RANGE_CONFIG__SIGMA_THRESH,RANGE_CONFIG__SIGMA_THRESH_VALUE,2);
    i2c_write_reg16bits(&vl53l1x->i2c_handle,RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS,RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS_VALUE,2);
    
    // i2c_read_reg16bits(&vl53l1x->i2c_handle,RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS,data,2);
    // ESP_LOGI(TAG_VL53L1X,"RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS: %d",VL53L1X_mergeData(data));

    //todo Dynamic Config
    i2c_write_reg16bits(&vl53l1x->i2c_handle,SYSTEM__GROUPED_PARAMETER_HOLD_0,SYSTEM__GROUPED_PARAMETER_HOLD_0_VALUE,1);
    i2c_write_reg16bits(&vl53l1x->i2c_handle,SYSTEM__GROUPED_PARAMETER_HOLD_1,SYSTEM__GROUPED_PARAMETER_HOLD_1_VALUE,1);
    i2c_write_reg16bits(&vl53l1x->i2c_handle,SD_CONFIG__QUANTIFIER,SD_CONFIG__QUANTIFIER_VALUE,1);

    ///-------------

    i2c_write_reg16bits(&vl53l1x->i2c_handle,SYSTEM__GROUPED_PARAMETER_HOLD,SYSTEM__GROUPED_PARAMETER_HOLD_VALUE,1);
    i2c_write_reg16bits(&vl53l1x->i2c_handle,SYSTEM__SEED_CONFIG,SYSTEM__SEED_CONFIG_VALUE,1);

    i2c_write_reg16bits(&vl53l1x->i2c_handle,SYSTEM__SEQUENCE_CONFIG,SYSTEM__SEQUENCE_CONFIG_VALUE,1);
    i2c_write_reg16bits(&vl53l1x->i2c_handle,DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT,DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT_VALUE,2);
    i2c_write_reg16bits(&vl53l1x->i2c_handle,DSS_CONFIG__ROI_MODE_CONTROL,DSS_CONFIG__ROI_MODE_CONTROL_VALUE,1);

    VL53L1X_setDistanceMode(vl53l1x,Long);
    VL53L1X_setMeasurementTimingBudget(vl53l1x,50000);

    i2c_read_reg16bits(&vl53l1x->i2c_handle,MM_CONFIG__OUTER_OFFSET_MM,data,2);
    uint8_t write_buff[2];
    VL53L1X_divergeData(write_buff,VL53L1X_mergeData(data)*4,2);
    i2c_write_reg16bits(&vl53l1x->i2c_handle,ALGO__PART_TO_PART_RANGE_OFFSET_MM,write_buff,2);
    
    i2c_read_reg16bits(&vl53l1x->i2c_handle,RESULT__RANGE_STATUS,data,1);
    ESP_LOGE(TAG_VL53L1X,"STATUS INIT: %d",data[0]);
    return 1;
}

bool VL53L1X_deinit(vl53l1x_t *vl53l1x)
{
    i2c_deinit(&vl53l1x->i2c_handle);
    return 0;
}

bool VL53L1X_setDistanceMode(vl53l1x_t* vl53l1x, distanceMode_t distance_mode){
    uint8_t data_range[4];
    uint32_t budget_us = VL53L1X_getMeasurementTimingBudget(vl53l1x);

    switch (distance_mode)
    {
    case Short:
        uint8_t data_short[] = {0x07,0x05,0x38,6};
        memcpy(data_range,data_short,4);
        break;
    
    case Medium:
        uint8_t data_medium[] ={0x0B,0x09,0x78,10};
        memcpy(data_range,data_medium,4);
        break;
    
    case Long:
        uint8_t data_long[] ={0x0F,0x0D,0xB8,14};
        memcpy(data_range,data_long,4);
        break;
    default:
      return 0;
    break;
    }
//todo Timing Config
i2c_write_reg16bits(&vl53l1x->i2c_handle,RANGE_CONFIG__VCSEL_PERIOD_A,data_range,1);
i2c_write_reg16bits(&vl53l1x->i2c_handle,RANGE_CONFIG__VCSEL_PERIOD_B,&data_range[1],1);
i2c_write_reg16bits(&vl53l1x->i2c_handle,RANGE_CONFIG__VALID_PHASE_HIGH,&data_range[2],1);

//todo Dynamic Config
i2c_write_reg16bits(&vl53l1x->i2c_handle,SD_CONFIG__WOI_SD0,data_range,1);
i2c_write_reg16bits(&vl53l1x->i2c_handle,SD_CONFIG__WOI_SD1,&data_range[1],1);
i2c_write_reg16bits(&vl53l1x->i2c_handle,SD_CONFIG__INITIAL_PHASE_SD0,&data_range[3],1);
i2c_write_reg16bits(&vl53l1x->i2c_handle,SD_CONFIG__INITIAL_PHASE_SD1,&data_range[3],1);

vl53l1x->distance_mode = distance_mode;

VL53L1X_setMeasurementTimingBudget(vl53l1x,budget_us);
return 1;
};

uint32_t VL53L1X_getMeasurementTimingBudget(vl53l1x_t* vl53l1x){

    uint8_t data[2];
    i2c_read_reg16bits(&vl53l1x->i2c_handle,RANGE_CONFIG__VCSEL_PERIOD_A,data,1);
    uint32_t macro_period_us = VL53L1X_calcMacroPeriod(vl53l1x,data[0]);

    i2c_read_reg16bits(&vl53l1x->i2c_handle,RANGE_CONFIG__VCSEL_PERIOD_A,data,2);
    uint32_t range_config_timeout_us = VL53L1X_timeoutMclksToMicroseconds(VL53L1X_decodeTimeout(VL53L1X_mergeData(data)),macro_period_us);

    return  2 * range_config_timeout_us + TIMING_GUARD;
}

bool VL53L1X_setMeasurementTimingBudget(vl53l1x_t* vl53l1x,uint32_t budget_us){
    if (budget_us <= TIMING_GUARD) { return false; }

    uint32_t range_config_timeout_us = budget_us -= TIMING_GUARD;
    if (range_config_timeout_us > 1100000) { return false; } // FDA_MAX_TIMING_BUDGET_US * 2
  
    range_config_timeout_us /= 2;
  
    // VL53L1_calc_timeout_register_values() begin
    uint8_t data[2];
    uint32_t macro_period_us;
    i2c_read_reg16bits(&vl53l1x->i2c_handle,RANGE_CONFIG__VCSEL_PERIOD_A,data,1);

    macro_period_us = VL53L1X_calcMacroPeriod(vl53l1x,data[0]);

    uint32_t phasecal_timeout_mclks = VL53L1X_timeoutMicrosecondsToMclks(1000, macro_period_us);
    if (phasecal_timeout_mclks > 0xFF) { phasecal_timeout_mclks = 0xFF; }
    uint8_t phasecal_timeout_mclks8 = (uint8_t)phasecal_timeout_mclks;

    i2c_write_reg16bits(&vl53l1x->i2c_handle,PHASECAL_CONFIG__TIMEOUT_MACROP,&phasecal_timeout_mclks8,1);

    uint8_t write_buff[2];

    VL53L1X_encodeTimeout(VL53L1X_timeoutMicrosecondsToMclks(1,macro_period_us),write_buff);
    i2c_write_reg16bits(&vl53l1x->i2c_handle,MM_CONFIG__TIMEOUT_MACROP_A,write_buff,2);

    VL53L1X_encodeTimeout(VL53L1X_timeoutMicrosecondsToMclks(range_config_timeout_us,macro_period_us),write_buff);
    i2c_write_reg16bits(&vl53l1x->i2c_handle,RANGE_CONFIG__TIMEOUT_MACROP_A,write_buff,2);

    i2c_read_reg16bits(&vl53l1x->i2c_handle,RANGE_CONFIG__VCSEL_PERIOD_B,&data[1],1);
    macro_period_us = VL53L1X_calcMacroPeriod(vl53l1x,data[1]);

    VL53L1X_encodeTimeout(VL53L1X_timeoutMicrosecondsToMclks(1,macro_period_us),write_buff);
    i2c_write_reg16bits(&vl53l1x->i2c_handle,MM_CONFIG__TIMEOUT_MACROP_B,write_buff,2);

    VL53L1X_encodeTimeout(VL53L1X_timeoutMicrosecondsToMclks(range_config_timeout_us,macro_period_us),write_buff);
    i2c_write_reg16bits(&vl53l1x->i2c_handle,RANGE_CONFIG__TIMEOUT_MACROP_B,write_buff,2);

    return true;

}   

uint32_t VL53L1X_calcMacroPeriod(vl53l1x_t* vl53l1x, uint8_t vcsel_period){
    uint32_t pll_period_us = ((uint32_t)0x01 << 30) / vl53l1x->fast_osc_frq;

    uint8_t vcsel_period_pclks = (vcsel_period + 1) << 1;

    uint32_t macro_period_us = (uint32_t)2304 * pll_period_us;
    macro_period_us >>= 6;
    macro_period_us *= vcsel_period_pclks;
    macro_period_us >>= 6;

    return macro_period_us;
}

uint32_t VL53L1X_decodeTimeout(uint16_t reg_val)
{
  return ((uint32_t)(reg_val & 0xFF) << (reg_val >> 8)) + 1;
}

uint32_t VL53L1X_timeoutMclksToMicroseconds(uint32_t timeout_mclks, uint32_t macro_period_us)
{
  return ((uint64_t)timeout_mclks * macro_period_us + 0x800) >> 12;
}

uint32_t VL53L1X_timeoutMicrosecondsToMclks(uint32_t timeout_us, uint32_t macro_period_us)
{
  return (((uint32_t)timeout_us << 12) + (macro_period_us >> 1)) / macro_period_us;
}

void VL53L1X_encodeTimeout(uint32_t timeout_mclks, uint8_t* buff)
{
  // encoded format: "(LSByte * 2^MSByte) + 1"

  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;

  if (timeout_mclks > 0)
  {
    ls_byte = timeout_mclks - 1;

    while ((ls_byte & 0xFFFFFF00) > 0)
    {
      ls_byte >>= 1;
      ms_byte++;
    }

    buff[0] = (uint8_t)ms_byte;
    buff[1] = (ls_byte & 0xFF);
    //return (ms_byte << 8) | (ls_byte & 0xFF);
  }
  else { buff[0] = 0, buff[1] = 0; }
}

void VL53L1X_startContinuous(vl53l1x_t * vl53l1x, uint32_t period_ms){
    uint8_t write_buff[4];
    uint32_t value = period_ms * (uint32_t)vl53l1x->osc_calibrate_val;
    VL53L1X_divergeData(write_buff,value,4);
    i2c_write_reg16bits(&vl53l1x->i2c_handle,SYSTEM__INTERMEASUREMENT_PERIOD,write_buff,4);
    //ESP_LOGE(TAG_VL53L1X,"STATUS SYSTEM__INTERMEASUREMENT_PERIOD_Value: %lx",value);
    //i2c_read_reg16bits(&vl53l1x->i2c_handle,SYSTEM__INTERMEASUREMENT_PERIOD,write_buff,4);
    //ESP_LOGE(TAG_VL53L1X,"STATUS SYSTEM__INTERMEASUREMENT_PERIOD: %x, %x, %x %x",write_buff[0],write_buff[1],write_buff[2],write_buff[3]);

    write_buff[0] = 0x01;
    write_buff[1] = 0x40;

    i2c_write_reg16bits(&vl53l1x->i2c_handle,SYSTEM__INTERRUPT_CLEAR,write_buff,1);
    i2c_write_reg16bits(&vl53l1x->i2c_handle,SYSTEM__MODE_START,&write_buff[1],1);
    //uint8_t status;
    //i2c_read_reg16bits(&vl53l1x->i2c_handle,RESULT__RANGE_STATUS,&status,1);
    //ESP_LOGE(TAG_VL53L1X,"STATUS STconti: %d",status);

}

void VL53L1X_stopContinuos(vl53l1x_t *vl53l1x){
  uint8_t buff[] = {0x80,0x00}; 
  i2c_write_reg16bits(&vl53l1x->i2c_handle, SYSTEM__MODE_START,buff,1);  // mode_range__abort

  calibrated = false;

  // restore vhv configs
  if (saved_vhv_init != 0)
  {
    i2c_write_reg16bits(&vl53l1x->i2c_handle,VHV_CONFIG__INIT,&saved_vhv_init,1);
  }
  if (saved_vhv_timeout != 0)
  {
    i2c_write_reg16bits(&vl53l1x->i2c_handle,VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,&saved_vhv_timeout,1);
  }

  // remove phasecal override
  i2c_write_reg16bits(&vl53l1x->i2c_handle,PHASECAL_CONFIG__OVERRIDE, &buff[1],1);

}

uint16_t VL53L1X_readDistance(vl53l1x_t* vl53l1x, bool blocking)
{
    if(blocking){
        VL53L1X_startTimeout();
        while (!VL53L1X_dataReady(vl53l1x))
        {
            if(VL53L1X_checkTimeoutExpired()){
                ESP_LOGE(TAG_VL53L1X,"VL53L1X_checkTimeoutExpired");
                did_time_out = true;
                return 0;
            }
        }
    }

    VL53L1X_readResults(vl53l1x);
    
    if (!calibrated)
    {
        VL53L1X_setupManualCalibration(vl53l1x);
        calibrated = true;
    }

    VL53L1X_updateDSS(vl53l1x);

    VL53L1X_getRangingData(vl53l1x);

    uint8_t data = 1;
    i2c_write_reg16bits(&vl53l1x->i2c_handle,SYSTEM__INTERRUPT_CLEAR,&data,1);

    return vl53l1x->ranging_data.range_mm;
}

void VL53L1X_readResults(vl53l1x_t* vl53l1x){
    uint8_t read_buff[17];
    i2c_read_reg16bits(&vl53l1x->i2c_handle,RESULT__RANGE_STATUS,read_buff,17);
    
    vl53l1x->results.range_status = read_buff[0];
    vl53l1x->results.stream_count = read_buff[2];
    vl53l1x->results.dss_actual_effective_spads_sd0 = VL53L1X_mergeData(&read_buff[3]);              // [4,5]
    vl53l1x->results.ambient_count_rate_mcps_sd0 = VL53L1X_mergeData(&read_buff[7]);                 // [8,9]
    vl53l1x->results.final_crosstalk_corrected_range_mm_sd0 = VL53L1X_mergeData(&read_buff[13]);     // [2,3]
    vl53l1x->results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0 = VL53L1X_mergeData(&read_buff[15]); // [0, 1]
}

void VL53L1X_startTimeout(){
    timeout_start_ms = (uint16_t)(esp_timer_get_time()/1000);
    //ESP_LOGI(TAG_VL53L1X, "timeout_start_ms: %llu, %d", time, timeout_start_ms);
}

bool VL53L1X_dataReady(vl53l1x_t *vl53l1x){
    uint8_t result;
    i2c_read_reg16bits(&vl53l1x->i2c_handle,GPIO__TIO_HV_STATUS,&result,1);
    //ESP_LOGE(TAG_VL53L1X,"VL53L1X_dataReady %x",result);
    return (result & 0x01) == 0;
}

bool VL53L1X_checkTimeoutExpired(){

    uint16_t time = (uint16_t)(esp_timer_get_time()/1000);
    return (io_timeout > 0) && ((time - timeout_start_ms) > io_timeout);
  
}

void VL53L1X_setupManualCalibration(vl53l1x_t *vl53l1x){
    // "save original vhv configs"
    i2c_read_reg16bits(&vl53l1x->i2c_handle,VHV_CONFIG__INIT,&saved_vhv_init,1);
    i2c_read_reg16bits(&vl53l1x->i2c_handle,VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,&saved_vhv_timeout,1);
    
    // "disable VHV init"
    uint8_t value =  saved_vhv_init & 0x7F;
    i2c_write_reg16bits(&vl53l1x->i2c_handle, VHV_CONFIG__INIT,&value,1);
    
    // "set loop bound to tuning param"
    value = (saved_vhv_timeout & 0x03) + (3 << 2);
    i2c_write_reg16bits(&vl53l1x->i2c_handle, VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,&value,1); // tuning parm default (LOWPOWERAUTO_VHV_LOOP_BOUND_DEFAULT)
    
    // "override phasecal"
    value = 1;
    i2c_write_reg16bits(&vl53l1x->i2c_handle, PHASECAL_CONFIG__OVERRIDE, &value, 1);
    i2c_read_reg16bits(&vl53l1x->i2c_handle,PHASECAL_RESULT__VCSEL_START,&value,1);
    i2c_write_reg16bits(&vl53l1x->i2c_handle, CAL_CONFIG__VCSEL_START, &value, 1);
}

void VL53L1X_updateDSS(vl53l1x_t *vl53l1x){
    uint8_t write_buff[2];
    uint16_t spadCount = vl53l1x->results.dss_actual_effective_spads_sd0;
    
    if (spadCount != 0)
    {
        // "Calc total rate per spad"  
        uint32_t totalRatePerSpad = (uint32_t)vl53l1x->results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0 + vl53l1x->results.ambient_count_rate_mcps_sd0;
        // "clip to 16 bits"
        if (totalRatePerSpad > 0xFFFF) {totalRatePerSpad = 0xFFFF;}

        // "shift up to take advantage of 32 bits"
        totalRatePerSpad <<= 16;
        totalRatePerSpad /= spadCount;
        if (totalRatePerSpad != 0)
        {
            // "get the target rate and shift up by 16"
            uint32_t requiredSpads = ((uint32_t)TARGET_RATE << 16) / totalRatePerSpad;
            
            // "clip to 16 bit"
            if (requiredSpads > 0xFFFF) { requiredSpads = 0xFFFF; }
            
            // "override DSS config"
            VL53L1X_divergeData(write_buff,requiredSpads,2);
            i2c_write_reg16bits(&vl53l1x->i2c_handle, DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, write_buff, 2);
            // DSS_CONFIG__ROI_MODE_CONTROL should already be set to REQUESTED_EFFFECTIVE_SPADS
            
            return;
        }
    }
    // If we reached this point, it means something above would have resulted in a
    // divide by zero.
    // "We want to gracefully set a spad target, not just exit with an error"

    // "set target to mid point"
    VL53L1X_divergeData(write_buff,0x8000,2);
    i2c_write_reg16bits(&vl53l1x->i2c_handle, DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, write_buff, 2);
}

void VL53L1X_getRangingData(vl53l1x_t *vl53l1x){
     // VL53L1_copy_sys_and_core_results_to_range_results() begin

    uint16_t range = vl53l1x->results.final_crosstalk_corrected_range_mm_sd0;

    // "apply correction gain"
    // gain factor of 2011 is tuning parm default (VL53L1_TUNINGPARM_LITE_RANGING_GAIN_FACTOR_DEFAULT)
    // Basically, this appears to scale the result by 2011/2048, or about 98%
    // (with the 1024 added for proper rounding).

    vl53l1x->ranging_data.range_mm = ((uint32_t)range * 2011 + 0x0400) / 0x0800;

    // VL53L1_copy_sys_and_core_results_to_range_results() end

    // set range_status in ranging_data based on value of RESULT__RANGE_STATUS register
    // mostly based on ConvertStatusLite()
    switch(vl53l1x->results.range_status)
    {
      case 17: // MULTCLIPFAIL
      case 2: // VCSELWATCHDOGTESTFAILURE
      case 1: // VCSELCONTINUITYTESTFAILURE
      case 3: // NOVHVVALUEFOUND
        // from SetSimpleData()
        vl53l1x->ranging_data.range_status = HardwareFail;
        break;

      case 13: // USERROICLIP
       // from SetSimpleData()
        vl53l1x->ranging_data.range_status = MinRangeFail;
        break;

      case 18: // GPHSTREAMCOUNT0READY
        vl53l1x->ranging_data.range_status = SynchronizationInt;
        break;

      case 5: // RANGEPHASECHECK
        vl53l1x->ranging_data.range_status =  OutOfBoundsFail;
        break;

      case 4: // MSRCNOTARGET
        vl53l1x->ranging_data.range_status = SignalFail;
        break;

      case 6: // SIGMATHRESHOLDCHECK
        vl53l1x->ranging_data.range_status = SigmaFail;
        break;

      case 7: // PHASECONSISTENCY
        vl53l1x->ranging_data.range_status = WrapTargetFail;
        break;

      case 12: // RANGEIGNORETHRESHOLD
        vl53l1x->ranging_data.range_status = XtalkSignalFail;
        break;

      case 8: // MINCLIP
        vl53l1x->ranging_data.range_status = RangeValidMinRangeClipped;
        break;

      case 9: // RANGECOMPLETE
        // from VL53L1_copy_sys_and_core_results_to_range_results()
        if (vl53l1x->results.stream_count == 0)
        {
            vl53l1x->ranging_data.range_status = RangeValidNoWrapCheckFail;
        }
        else
        {
            vl53l1x->ranging_data.range_status = RangeValid;
        }
        break;

      default:
      vl53l1x->ranging_data.range_status = None;
    }
    //ESP_LOGI(TAG_VL53L1X,"rangingData.range_status : %d",vl53l1x->ranging_data.range_status);

    // from SetSimpleData()
    vl53l1x->ranging_data.peak_signal_count_rate_MCPS = VL53L1X_countRateFixedToFloat(vl53l1x->results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0);
    vl53l1x->ranging_data.ambient_count_rate_MCPS = VL53L1X_countRateFixedToFloat(vl53l1x->results.ambient_count_rate_mcps_sd0);
}

float VL53L1X_countRateFixedToFloat(uint16_t count_rate_fixed){
    return (float)count_rate_fixed / (1 << 7); 
}

bool VL53L1X_calibrateOffset(vl53l1x_t *vl53l1x, uint16_t targetDistanceInMm, uint16_t *foundOffset){
  uint16_t averageDistance = 0;
  uint8_t data[]={0x0,0x0},offset[2];
  //Reset offset values 
  i2c_write_reg16bits(&vl53l1x->i2c_handle,ALGO__PART_TO_PART_RANGE_OFFSET_MM,data,2);
  i2c_write_reg16bits(&vl53l1x->i2c_handle,MM_CONFIG__INNER_OFFSET_MM,data,2);
  i2c_write_reg16bits(&vl53l1x->i2c_handle,MM_CONFIG__OUTER_OFFSET_MM,data,2);

  VL53L1X_setDistanceMode(vl53l1x,Short);
  VL53L1X_setMeasurementTimingBudget(vl53l1x,30000);
  VL53L1X_startContinuous(vl53l1x,40);

  for(uint8_t i=0; i<50; i++){
    averageDistance += VL53L1X_readDistance(vl53l1x,1);
  }
  *foundOffset = (targetDistanceInMm - averageDistance/50);
  VL53L1X_stopContinuos(vl53l1x);
  VL53L1X_divergeData(offset,*foundOffset*4,2);

  i2c_write_reg16bits(&vl53l1x->i2c_handle,ALGO__PART_TO_PART_RANGE_OFFSET_MM,offset,2);
  return 1;
}

bool VL53L1X_calibrateXTalk(vl53l1x_t *vl53l1x, uint16_t targetDistanceInMm, uint16_t *foundXtalk){
  float AverageSignalRate = 0;
	float AverageDistance = 0;
	float AverageSpadNb = 0;
  uint16_t distance = 0;
  uint32_t calXtalk;
  uint8_t data[2];

  VL53L1X_setDistanceMode(vl53l1x,Short);
  VL53L1X_setMeasurementTimingBudget(vl53l1x,30000);
  VL53L1X_startContinuous(vl53l1x,40);

  for(uint8_t i=0; i<50; i++){

    distance = VL53L1X_readDistance(vl53l1x,1);
    AverageDistance += distance;
    AverageSpadNb += vl53l1x->results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0;
		AverageSignalRate += vl53l1x->ranging_data.peak_signal_count_rate_MCPS;
  }
  VL53L1X_stopContinuos(vl53l1x);
  AverageDistance = AverageDistance / 50;
	AverageSpadNb = AverageSpadNb / 50;
	AverageSignalRate = AverageSignalRate / 50;

  /* Calculate Xtalk value */
  calXtalk = (uint16_t)(512*(AverageSignalRate*(1-(AverageDistance/targetDistanceInMm)))/AverageSpadNb);
  if(calXtalk  > 0xffff)
		calXtalk = 0xffff;
  *foundXtalk = (uint16_t)((calXtalk*1000)>>9);
  
  /* Update Xtalk value*/
  VL53L1X_divergeData(data,(uint16_t)calXtalk,2);
  i2c_write_reg16bits(&vl53l1x->i2c_handle,ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS,data,2);

  return 1;
}

uint16_t VL53L1X_mergeData(uint8_t *data){
    return (uint16_t)data[0] << 8 | (uint16_t)data[1];
}

void VL53L1X_divergeData(uint8_t* buff, uint32_t value, uint8_t len){
    for (uint8_t i = 0; i < len; i++)
    {
        buff[i] =  value >> 8*(len-i-1);
    }
}