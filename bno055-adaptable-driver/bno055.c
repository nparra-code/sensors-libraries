/**
 * @file bno055.c
 * @author Cristian David Araujo A. (cristian.araujo@udea.edu.co)
 * @brief 
 * @version 0.1
 * @date 2024-11-08
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "bno055.h"

int8_t BNO055_Init(BNO055_t *bno055, uint8_t gpio_tx, uint8_t gpio_rx)
{
    int8_t success = BNO055_ERROR;

    printf("Initializing BNO055 sensor...\n");

    // Initialize the UART
    success += uart_init(&bno055->uart_config, 115200, 1024, gpio_tx, gpio_rx, UART_PIN_NO_USE, UART_PIN_NO_USE);

    // Set operation mode to CONFIGMODE
    bno055->operation_mode = INIT;
    success += BNO055_SetOperationMode(bno055, CONFIGMODE);

    // Write the default page as zero
    uint8_t data = BNO055_PAGE_ZERO;
    success += BN055_Write(bno055, BNO055_PAGE_ID_ADDR, &data, BNO055_GEN_READ_WRITE_LENGTH);
    // bno055->page_id = BNO055_PAGE_ZERO;

    // Read the chip ID
    success += BNO055_Read(bno055, BNO055_CHIP_ID_ADDR, &bno055->chip_id, BNO055_GEN_READ_WRITE_LENGTH);

    // Read accelerometer revision ID
    success += BNO055_Read(bno055, BNO055_ACCEL_REV_ID_ADDR, &bno055->accel_rev_id, BNO055_GEN_READ_WRITE_LENGTH);

    // Read magnetometer revision ID
    success += BNO055_Read(bno055, BNO055_MAG_REV_ID_ADDR, &bno055->mag_rev_id, BNO055_GEN_READ_WRITE_LENGTH);

    // Read gyroscope revision ID
    success += BNO055_Read(bno055, BNO055_GYRO_REV_ID_ADDR, &bno055->gyro_rev_id, BNO055_GEN_READ_WRITE_LENGTH);

    // Read bootloader revision ID
    success += BNO055_Read(bno055, BNO055_BL_REV_ID_ADDR, &bno055->bl_rev_id, BNO055_GEN_READ_WRITE_LENGTH);

    // Read software revision ID
    success += BNO055_Read(bno055, BNO055_SW_REV_ID_LSB_ADDR, bno055->sw_rev_id, BNO055_LSB_MSB_READ_LENGTH);

    // Read page ID
    success += BNO055_Read(bno055, BNO055_PAGE_ID_ADDR, &bno055->page_id, BNO055_GEN_READ_WRITE_LENGTH);

    // Set Unit to m/s^2, rad/s, rad, celcius, Windows orientation
    success += BNO055_SetUnit(bno055, BNO055_ACCEL_UNIT_MSQ, BNO055_GYRO_UNIT_RPS, BNO055_EULER_UNIT_RAD, BNO055_TEMP_UNIT_CELSIUS, BNO055_WIN_ORIENTATION);

    // Set Power mode to normal
    bno055->power_mode = LOWPOWER;
    success += BNO055_SetPowerMode(bno055, NORMAL);

    // Set operation mode to IMU
    success += BNO055_SetOperationMode(bno055, IMU);
    

    // Print the data read from the BNO055 sensor
    printf("\n----------------- BNO055 Sensor Data -----------------\n");
    printf("Operation mode: %02X\n", bno055->operation_mode);
    printf("Power mode: %02X\n", bno055->power_mode);
    printf("Chip ID: %02X\n", bno055->chip_id);
    printf("Accelerometer revision ID: %02X\n", bno055->accel_rev_id);
    printf("Magnetometer revision ID: %02X\n", bno055->mag_rev_id);
    printf("Gyroscope revision ID: %02X\n", bno055->gyro_rev_id);
    printf("Bootloader revision ID: %02X\n", bno055->bl_rev_id);
    printf("Software revision ID: %02X%02X\n", bno055->sw_rev_id[0], bno055->sw_rev_id[1]);
    printf("Page ID: %02X\n", bno055->page_id);
    printf("------------------------------------------------------\n");

    // Check if the BNO055 sensor was initialized successfully
    if (success == BNO055_SUCCESS) {
        printf("BNO055 sensor initialized successfully\n");
        return BNO055_SUCCESS;
    } else {
        printf("Error: Failed to initialize BNO055 sensor\n");
        return BNO055_ERROR;
    }

    return BNO055_ERROR; 
}

void BNO055_GetEulerAngles(BNO055_t *bno055, float *yaw, float *pitch, float *roll)
{
    if (bno055 == NULL || yaw == NULL || pitch == NULL || roll == NULL) {
        printf("Error: Null pointer provided\n");
        return; // Error: Null pointer
    }

    // Read the Euler angles from the BNO055 sensor
    uint8_t data[6] = {0};
    BNO055_Read(bno055, BNO055_EULER_H_LSB_ADDR, data, 6);

    // Convert the data to float
    // Convert the data to radians (1 radian = 900 LSB)

    // If data is 0000 0000 0000 0000 give the last value
    if (data[0] == 0 && data[1] == 0 && data[2] == 0 && data[3] == 0 && data[4] == 0 && data[5] == 0)
    {
        *yaw = bno055->yaw;
        *pitch = bno055->pitch;
        *roll = bno055->roll;
        return;
    }

    // Convert the data to float

    *yaw = (float)((int16_t)((data[1] << 8) | data[0])) / 900.0;
    *pitch = (float)((int16_t)((data[3] << 8) | data[2])) / 900.0;
    *roll = (float)((int16_t)((data[5] << 8) | data[4])) / 900.0;

    // Save the data
    bno055->yaw = *yaw;
    bno055->pitch = *pitch;
    bno055->roll = *roll;
}

int8_t BN055_Write(BNO055_t *bno055, uint8_t reg, uint8_t *data, uint8_t len)
{
    if (bno055 == NULL || data == NULL) {
        printf("Error: Null pointer provided\n");
        return -1; // Error: Null pointer
    }

    if (len + 4 > 128) { // Check for buffer overflow
        printf("Error: Data length exceeds buffer size\n");
        return -2; // Error: Buffer overflow
    }

    printf("Write operation\n");
    // The structure for write data is as follows:
    // | Start Byte | Opertation | Register | Length | Data 1 | Data 2 | ... | Data N |

    uint8_t buffer_tx[128];
    buffer_tx[0] = 0xAA; // Start byte
    buffer_tx[1] = 0x00; // Write operation
    buffer_tx[2] = reg;  // Register
    buffer_tx[3] = len;  // Length

    // Copy data into the buffer manually
    for (uint8_t i = 0; i < len; i++) {
        buffer_tx[4 + i] = data[i];
    }

    // Set the last position of the buffer to NULL
    buffer_tx[len + 4] = '\0';

    // Print the buffer
    printf("Buffer TX: ");
    for (uint8_t i = 0; i < len + 4; i++) {
        printf("%02X ", buffer_tx[i]);
    }
    printf("\n");

    uart_write(&bno055->uart_config, buffer_tx, len + 4);

    // Read the data from the BNO055 sensor
    uint8_t buffer_rx[2];
    uart_read(&bno055->uart_config, buffer_rx, 2, 5);

    printf("Buffer RX: ");
    for (uint8_t i = 0; i < 2; i++) {
        printf("%02X ", buffer_rx[i]);
    }
    printf("\n");


    if (BNO055_CheckAck(buffer_rx) == BNO055_SUCCESS)
    {
        printf("Write operation successful\n");
        return BNO055_SUCCESS;
    } else {
        printf("Write operation failed\n");
    }

    return BNO055_ERROR;


}

int8_t BNO055_Read(BNO055_t *bno055, uint8_t reg, uint8_t *data, uint8_t len)
{
    if (bno055 == NULL || data == NULL) {
        printf("Error: Null pointer provided\n");
        return -1; // Error: Null pointer
    }

    if (len + 4 > 128) { // Check for buffer overflow
        printf("Error: Data length exceeds buffer size\n");
        return -2; // Error: Buffer overflow
    }

    // The structure for read data is as follows:
    // | Start Byte | Opertation | Register | Length |

    uint8_t buffer_tx[4];
    buffer_tx[0] = 0xAA; // Start byte
    buffer_tx[1] = 0x01; // Read operation
    buffer_tx[2] = reg; // Register
    buffer_tx[3] = len; // Length

    // // Print the buffer
    // printf("Buffer TX: ");
    // for (uint8_t i = 0; i < 4; i++) {
    //     printf("%02X ", buffer_tx[i]);
    // }
    // printf("\n");

    // Send the read operation to the BNO055 sensor
    uart_write(&bno055->uart_config, buffer_tx, 4);

    // Read the data from the BNO055 sensor
    uint8_t buffer_rx[128] = {0};
    uart_read(&bno055->uart_config, buffer_rx, 128, 6);

    // // Print the buffer
    // printf("Buffer RX: ");
    // for (uint8_t i = 0; i < len + 2; i++) {
    //     printf("%02X ", buffer_rx[i]);
    // }
    // printf("\n");

    uint8_t length = 0;
    // Check if the read operation was successful
    if (buffer_rx[0] == BNO055_READ_SUCCESS) {
        // length = buffer_rx[1];
        // if (length != len) {
        //     printf("Error: Length mismatch\n");
        //     return -4; // Error: Length mismatch
        // }

        for (uint8_t i = 0; i < len; i++) {
            data[i] = buffer_rx[2 + i];
        }
    }
    else if (buffer_rx[0] == BNO055_ACK_VALUE)
    {
        // printf("Error: Read operation failed\n");
        return -3; // Error: Read operation failed
    }
    
    return BNO055_SUCCESS;
}

int8_t BNO055_CheckAck(uint8_t *data)
{
    if (data[0] == BNO055_ACK_VALUE) {
        switch (data[1])
        {
        case BNO055_WRITE_SUCCESS:
            return BNO055_SUCCESS;
            break;
        case BNO055_WRITE_FAIL:
            return BNO055_ERROR;
            break;
        default:
            return BNO055_ERROR;
            break;
        }
    }
    return BNO055_ERROR;

}

int8_t BNO055_ReadAccelX(BNO055_t *bno055)
{
    uint8_t data[2];
    BNO055_Read(bno055, BNO055_ACCEL_DATA_X_LSB_ADDR, data, 2);
    
    bno055->ax = (float)((int16_t)(data[1] << 8 | data[0])) / 100.0f;

    return BNO055_SUCCESS;
}

int8_t BNO055_ReadAccelY(BNO055_t *bno055)
{
    uint8_t data[2];
    BNO055_Read(bno055, BNO055_ACCEL_DATA_Y_LSB_ADDR, data, 2);
    
    bno055->ay = (float)((int16_t)(data[1] << 8 | data[0])) / 100.0f;

    return BNO055_SUCCESS;
}

int8_t BNO055_ReadAccelZ(BNO055_t *bno055)
{
    uint8_t data[2];
    BNO055_Read(bno055, BNO055_ACCEL_DATA_Z_LSB_ADDR, data, 2);
    
    bno055->az = (float)((int16_t)(data[1] << 8 | data[0])) / 100.0f;

    return BNO055_SUCCESS;
}

int8_t BNO055_SetOperationMode(BNO055_t *bno055, BNO055_OperationMode mode)
{
    if (bno055 == NULL) {
        printf("Error: Null pointer provided\n");
        return -1; // Error: Null pointer
    }

    if (mode != bno055->operation_mode) {
        uint8_t data = (uint8_t)mode;
        int8_t success = BN055_Write(bno055, BNO055_OPR_MODE_ADDR, &data, BNO055_GEN_READ_WRITE_LENGTH);
        bno055->operation_mode = mode;
        return success;
    }

    return BNO055_SUCCESS;
}

void BNO055_GetAcceleration(BNO055_t *bno055, float *x, float *y, float *z)
{
    uint8_t data[6];
    BNO055_Read(bno055, BNO055_ACCEL_DATA_X_LSB_ADDR, data, 6);

    *x = (float)((int16_t)(data[1] << 8 | data[0])) / 100.0f;
    *y = (float)((int16_t)(data[3] << 8 | data[2])) / 100.0f;
    *z = (float)((int16_t)(data[5] << 8 | data[4])) / 100.0f;
}

void BNO055_GetGyro(BNO055_t *bno055, float *gx, float *gy, float *gz)
{
    uint8_t data[6];
    BNO055_Read(bno055, BNO055_GYRO_DATA_X_LSB_ADDR, data, 6);

    *gx = (float)((int16_t)(data[1] << 8 | data[0])) / 16.0f;
    *gy = (float)((int16_t)(data[3] << 8 | data[2])) / 16.0f;
    *gz = (float)((int16_t)(data[5] << 8 | data[4])) / 16.0f;
}

uint8_t BNO055_SetUnit(BNO055_t *bno055, uint8_t accel_unit, uint8_t gyro_unit, uint8_t euler_unit, uint8_t temp_unit, uint8_t ori_unit)
{
    if (bno055 == NULL) {
        printf("Error: Null pointer provided\n");
        return -1; // Error: Null pointer
    }

    uint8_t data = 0x00;
    data |= accel_unit << 0;
    data |= gyro_unit << 1;
    data |= euler_unit << 2;
    data |= temp_unit << 4;
    data |= ori_unit << 7;

    int8_t success = BN055_Write(bno055, BNO055_UNIT_SEL_ADDR, &data, BNO055_GEN_READ_WRITE_LENGTH);

    return success;
}

int8_t BNO055_SetPowerMode(BNO055_t *bno055, BNO055_PowerMode mode)
{
    if (bno055 == NULL) {
        printf("Error: Null pointer provided\n");
        return -1; // Error: Null pointer
    }

    if (mode != bno055->power_mode) {
        uint8_t data = (uint8_t)mode;
        int8_t success = BN055_Write(bno055, BNO055_PWR_MODE_ADDR, &data, BNO055_GEN_READ_WRITE_LENGTH);
        bno055->power_mode = mode;
        return success;
    }

    return BNO055_SUCCESS;
}
