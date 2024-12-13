#include "i2c_platform.h"

i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;

void i2c0_init(uint8_t address, uint32_t frequency){
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_15,
        .scl_io_num = GPIO_NUM_16,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address = address,
        .scl_speed_hz = frequency
    };
    
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle));
}

uint8_t i2c0_read_byte(uint8_t reg) {
    uint8_t buf[] = {reg};
    uint8_t data;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, buf, 1, &data, 1, -1));
    return data;
}

void i2c0_write_byte(uint8_t reg, uint8_t data) {
    uint8_t buf[] = {reg, data};
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, buf, 2, -1));
}