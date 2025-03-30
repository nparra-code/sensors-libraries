#include "i2c_platform.h"

bool i2c_init(i2c_t *i2c, i2c_port_t i2c_num, uint8_t gpio_scl, uint8_t gpio_sda, uint32_t clk_speed_hz, uint16_t addr)
{
    esp_err_t ret = ESP_OK;

    i2c->addr = addr;
    i2c->clk_speed_hz = clk_speed_hz;
    i2c->i2c_num = i2c_num;
    i2c->gpio_scl = gpio_scl;
    i2c->gpio_sda = gpio_sda;

    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = i2c_num,
        .scl_io_num = gpio_scl,
        .sda_io_num = gpio_sda,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_GOTO_ON_ERROR(i2c_new_master_bus(&i2c_mst_config, &bus_handle), err, "TAG_I2C", "i2c io to channel failed");
    i2c->bus_handle = bus_handle;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = clk_speed_hz,
    };

    static i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
    i2c->dev_handle = dev_handle;

    return true;
    err:
    return ret == ESP_OK;
}

esp_err_t i2c_deinit(i2c_t *i2c)
{
    return i2c_del_master_bus(i2c->bus_handle);
}

esp_err_t i2c_read(i2c_t i2c, uint8_t *data, size_t len)
{
    return i2c_master_receive(i2c.dev_handle, (uint8_t *)data, len, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t i2c_write(i2c_t i2c, uint8_t *data, size_t len)
{
    esp_err_t status = i2c_master_transmit(i2c.dev_handle, data, len, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    if(status)
        ESP_LOGE("VL6180x", "i2c_write status error %x", status);
    return status;
}