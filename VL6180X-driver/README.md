| Supported Targets | ESP32-S3 |

# VL6180X LiDAR ToF sensor on ESP32-S3

This code implements the VL6180X proximity sensor in the ESP32-S3 using I2C. The API provided by STM is used, and adapted to work on FreeRTOS on the ESP using the ESP-IDF extension.

## Overview

There are 3 main codes: ranging polling, sensor calibration, and range interrupt (under development).

The I2C is abstracted, and can be found in the platform files.

#### Ranging Polling
This codes runs in a loop and asks the sensor the range everytime. Not ideal for complex applications because CPU is always busy.

#### Sensor Calibration
It can happen the sensor is misconfigured. This example allows you to calibrate the range measurment.

#### Range Interruption
In this example you can find the configuration for the sensor to work with interruptions. This allows you to set a threshold to detect, then it sends a high voltage through sensor's *GPIO0* pin if detected and do something. Ideal for tasks that act **only** if anything is in a range, you will not need to keep the CPU busy measuring range everytime.

## How to use

### Hardware and Software Required

Hardware: You need to find yourself a VL6180X sensor, and an ESP32-S3.

Software: You must have the [ESP-IDF](https://idf.espressif.com/) framwork.

### I2C platform ESP32-S3 HAL

The following code does the I2C init, where parameters are: a configuration structure named `i2c_t`, followed by the I2C port to use with `i2c_num`, the SCL and SDA pins with `gpio_scl`, `gpio_sda`, the clock frequency with `clk_speed_hz`, and the device-to-connect's slave address `addr`.
```c
bool i2c_init(i2c_t *i2c, i2c_port_t i2c_num, uint8_t gpio_scl, uint8_t gpio_sda, uint32_t clk_speed_hz, uint16_t addr);
```

You can delete your device with:
```c
esp_err_t i2c_deinit(i2c_t *i2c);
```

This function reads the incoming I2C information. Where `data` is where the information is going to be stored, and `len` the size of that information (in bytes; e.g. `0xA5` requires a `len=2`). Note that the availabe space for data must be at least the same as `len`.
```c
esp_err_t i2c_read(i2c_t i2c, uint8_t *data, size_t len);
```

This function writes information. Where `data` is where the information to be written is stored, and `len` the size of that information (in bytes; e.g. `0xA5` requires a `len=2`). Your `data` array must contain everythin you would like to send. If there is any specific register you want to write to, `data` must already contain that address in the format device requires it.
```c
esp_err_t i2c_write(i2c_t i2c, uint8_t *data, size_t len);
```
**E.g.** If you wanted to write to `0x016` VL6180's register to check if device is ready, you must do the following as specified in sensor's datasheet:
```c
buffer[0]=0x016>>8;
buffer[1]=0x016&0xFF;

i2c_write(dev, buffer, 2);
```

### I2C initialization in examples:

Every example inits the I2C as follows:
```c
i2c_init(&vl6180x_i2c, I2C_NUM_1, I2C_SCL, I2C_SDA, 400000, 0x29);
```

`I2C_SCL` and `I2C_SDA` could be replaced by any other GPIO. The slave address is often the same, but sometimes it is needed to check `0x212` register in order to obtain the current one as it can be changed.

### Calibration Example:

When firmware mounted, it is going to require to put an object in a specified range. Usually, that range is *50mm*. It is recommended to wait until the console starts to show **VL6180x: No target detect.**, there is when you put your object at the asked distance.

#### Troubleshoot
If for any reason the program reboots, you do the same: *Wait until the console starts to show **VL6180x: No target detect.**, and then you put your object at the asked distance*.