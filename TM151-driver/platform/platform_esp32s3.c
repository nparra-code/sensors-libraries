/**
 * @file platform_esp32s3.c
 * @author Cristian David Araujo A. (cristian.araujo@udea.edu.co)
 * @brief 
 * @version 0.1
 * @date 2024-11-08
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "platform_esp32s3.h"

/**
 * @brief Find UART number for the ESP32-S3 microcontroller
 * 
 * @param gpio_tx 
 * @param gpio_rx 
 * @param gpio_rts 
 * @param gpio_cts 
 * @return uart_port_t 
 */
static uart_port_t find_uart_num(uint8_t gpio_tx, uint8_t gpio_rx, uint8_t gpio_rts, uint8_t gpio_cts)
{
    // UART number for the ESP32-S3 microcontroller
    if (gpio_tx == 43 && gpio_rx == 44)
    {
        return UART_NUM_0;
    }
    else if (gpio_tx == 17 && gpio_rx == 18)
    {
        return UART_NUM_1;
    }

    return UART_NUM_2;
}

int uart_init(uart_t *uart_config, uint32_t baud_rate, uint16_t buffer_size, int8_t gpio_tx, int8_t gpio_rx, int8_t gpio_rts, int8_t gpio_cts)
{
    // Check if the UART gpio pins are valid
    if (gpio_tx == UART_PIN_NO_USE || gpio_rx == UART_PIN_NO_USE)
    {
        ESP_LOGE("UART_INIT", "Invalid UART TX or RX pin");
        return -1;
    }
    

    
    // Find UART number for the ESP32-S3 microcontroller
    uart_port_t uart_num = find_uart_num(gpio_tx, gpio_rx, gpio_rts, gpio_cts);

    // Configure UART parameters
    uart_config->baud_rate = baud_rate;
    uart_config->buffer_size = buffer_size;
    uart_config->uart_num = uart_num;
    uart_config->gpio_tx = gpio_tx;
    uart_config->gpio_rx = gpio_rx;
    uart_config->gpio_rts = gpio_rts;
    uart_config->gpio_cts = gpio_cts;

    // Configure UART structure
    uart_config->uart_config = (uart_config_t){
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = (gpio_rts != UART_PIN_NO_USE && gpio_cts != UART_PIN_NO_USE) 
                                    ? UART_HW_FLOWCTRL_CTS_RTS 
                                    : UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    


    int log = 0;
    // Configure UART parameters and check if UART was initialized successfully
    if (uart_param_config(uart_num, &(uart_config->uart_config)) != ESP_OK)
    {
        ESP_LOGE("UART_INIT", "Failed to configure UART parameters");
        log = -1;
    }

    // Configure UART pins
    if (uart_set_pin(uart_num, gpio_tx, gpio_rx, gpio_rts, gpio_cts) != ESP_OK)
    {
        ESP_LOGE("UART_INIT", "Failed to configure UART pins");
        log = -1;
    }

    // Install UART driver
    if (uart_driver_install(uart_num, buffer_size * 2, 0, 0, NULL, 0) != ESP_OK)
    {
        ESP_LOGE("UART_INIT", "Failed to install UART driver");
        log = -1;
    }

    // Clear UART buffer
    if (uart_flush(uart_num) != ESP_OK)
    {
        ESP_LOGE("UART_INIT", "Failed to flush UART buffer");
        log = -1;
    }

    // ESP_LOGI("UART_INIT", "UART initialized successfully");
    return log;
}

int uart_write(uart_t *uart_config, const uint8_t *data, size_t length)
{
    if (uart_config == NULL || data == NULL || length == 0) {
        ESP_LOGE("UART_WRITE", "Invalid parameters.");
        return -1; // Error
    }

    // Attempt to write the data to the UART
    int bytes_written = uart_write_bytes(uart_config->uart_num, (const char *)data, length);

    // Check for errors
    if (bytes_written < 0) {
        ESP_LOGE("UART_WRITE", "Failed to write data to UART.");
        return -1; // Error
    }

    // ESP_LOGI("UART_WRITE", "Successfully wrote %d bytes to UART.", bytes_written);
    return bytes_written;
}

int uart_read(uart_t *uart_config, uint8_t *buffer, size_t length, int timeout_ms)
{
    if (uart_config == NULL || buffer == NULL || length == 0) {
        ESP_LOGE("UART_READ", "Invalid parameters.");
        return -1; // Error
    }
    
    // Attempt to read data from the UART
    int bytes_read = uart_read_bytes(uart_config->uart_num, buffer, length, timeout_ms / portTICK_PERIOD_MS);

    // Check for errors
    if (bytes_read < 0) {
        ESP_LOGE("UART_READ", "Failed to read data from UART.");
        return -1; // Error
    }

    // ESP_LOGDI"UART_READ", "Successfully read %d bytes from UART.", bytes_read);
    return bytes_read;
}

int uart_clear(uart_t *uart_config)
{
    if (uart_config == NULL) {
        ESP_LOGE("UART_FLUSH", "Invalid parameters.");
        return -1; // Error
    }

    // Attempt to flush the UART buffer
    if (uart_flush(uart_config->uart_num) != ESP_OK) {
        ESP_LOGE("UART_FLUSH", "Failed to flush UART buffer.");
        return -1; // Error
    }
    return 0;
}
