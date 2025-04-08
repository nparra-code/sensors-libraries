| Supported Targets | ESP32-S3 |

# TM151 IMU TransducerM 9-axis driver

This code implements the TM151 IMU sensor in the ESP32-S3 using UART. The example provided by Syd Dynamics is used, and adapted to work on FreeRTOS on the ESP using the ESP-IDF extension.

## Code

In the example you will find how to read data, and that is done by initializing some protocols used to tell the IMU information needed, and could be changed from macros defined.

## IMU Assistant Software

The company provides an assistant software where you can configure your device. Specs such as baud rate, sensors active, data retrieved, boot options, and more, are configured there.

### Calibration
Calibration is one of the aspects done there. Usually, the IMU calibrates by itself at boot. But, depending on the boot mode, calibration is done different, and precision and quality of service vary.