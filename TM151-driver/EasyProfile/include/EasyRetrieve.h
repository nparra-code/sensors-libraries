#ifndef _EASYRETRIEVE_H_
#define _EASYRETRIEVE_H_

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"

#include "EasyObjectDictionary.h"
#include "EasyProfile.h"

#include "platform_esp32s3.h" // Include the platform-specific header file for ESP32-S3
#include "EasyRetrieve.h"

/*
 * @brief Called every time when new serial data is received
 */
void SerialPort_DataGet(uart_t* myUART);

#endif