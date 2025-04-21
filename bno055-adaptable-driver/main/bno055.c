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

/**
 * @brief Convert the data from the BNO055 sensor to float
 * 
 * @param bno055 Pointer to the BNO055 sensor structure
 * @param data Data in bytes from the BNO055 sensor
 * @param x Float pointer to store the X value
 * @param y Float pointer to store the Y value
 * @param z Float pointer to store the Z value
 */
void BNO055_ConvertData_Accel(BNO055_t *bno055, uint8_t *data, float *x, float *y, float *z)
{
    if (bno055 == NULL || data == NULL) {
        printf("Error: Null pointer provided\n");
        return; // Error: Null pointer
    }

    // Convert the data in bytes to float
    if (bno055->unit_settings.accel_unit == BNO055_ACCEL_UNIT_MSQ) {
        *x = (float)((int16_t)(data[1] << 8 | data[0])) / 100.0f;
        *y = (float)((int16_t)(data[3] << 8 | data[2])) / 100.0f;
        *z = (float)((int16_t)(data[5] << 8 | data[4])) / 100.0f;
    } else {
        *x = (float)((int16_t)(data[1] << 8 | data[0])) / 1.0f;
        *y = (float)((int16_t)(data[3] << 8 | data[2])) / 1.0f;
        *z = (float)((int16_t)(data[5] << 8 | data[4])) / 1.0f;
    }
    
}

/**
 * @brief Convert the data Gyroscope from the BNO055 sensor to float
 * 
 * @param bno055 Pointer to the BNO055 sensor structure
 * @param data Data in bytes from the BNO055 sensor
 * @param x Float pointer to store the X value
 * @param y Float pointer to store the Y value
 * @param z Float pointer to store the Z value
 */
void BNO055_ConvertData_Gyro(BNO055_t *bno055, uint8_t *data, float *x, float *y, float *z)
{
    if (bno055 == NULL || data == NULL) {
        printf("Error: Null pointer provided\n");
        return; // Error: Null pointer
    }

    // Convert the data in bytes to float
    if (bno055->unit_settings.gyro_unit == BNO055_GYRO_UNIT_DPS) {
        *x = (float)((int16_t)(data[1] << 8 | data[0])) / 16.0f;
        *y = (float)((int16_t)(data[3] << 8 | data[2])) / 16.0f;
        *z = (float)((int16_t)(data[5] << 8 | data[4])) / 16.0f;
    } else {
        *x = (float)((int16_t)(data[1] << 8 | data[0])) / 100.0f;
        *y = (float)((int16_t)(data[3] << 8 | data[2])) / 100.0f;
        *z = (float)((int16_t)(data[5] << 8 | data[4])) / 100.0f;
    }
}

/**
 * @brief Convert the data Euler angles from the BNO055 sensor to float
 * 
 * @param bno055 Pointer to the BNO055 sensor structure
 * @param data Data in bytes from the BNO055 sensor
 * @param yaw Float pointer to store the yaw value
 * @param pitch Float pointer to store the pitch value
 * @param roll Float pointer to store the roll value
 */
void BNO055_ConvertData_Euler(BNO055_t *bno055, uint8_t *data, float *yaw, float *pitch, float *roll)
{
    if (bno055 == NULL || data == NULL) {
        printf("Error: Null pointer provided\n");
        return; // Error: Null pointer
    }

    // Convert the data in bytes to float
    if (bno055->unit_settings.euler_unit == BNO055_EULER_UNIT_DEG) {
        *yaw = (float)((int16_t)(data[1] << 8 | data[0])) / 16.0f;
        *pitch = (float)((int16_t)(data[3] << 8 | data[2])) / 16.0f;
        *roll = (float)((int16_t)(data[5] << 8 | data[4])) / 16.0f;
    } else {
        *yaw = (float)((int16_t)(data[1] << 8 | data[0])) / 900.0f;
        *pitch = (float)((int16_t)(data[3] << 8 | data[2])) / 900.0f;
        *roll = (float)((int16_t)(data[5] << 8 | data[4])) / 900.0f;
    }
}

/**
 * @brief Convert the data magnetic field from the BNO055 sensor to float
 * 
 * @param bno055 Pointer to the BNO055 sensor structure
 * @param data Data in bytes from the BNO055 sensor
 * @param x Float pointer to store the X value
 * @param y Float pointer to store the Y value
 * @param z Float pointer to store the Z value
 */
void BNO055_ConvertData_Mag(BNO055_t *bno055, uint8_t *data, float *x, float *y, float *z)
{
    if (bno055 == NULL || data == NULL) {
        printf("Error: Null pointer provided\n");
        return; // Error: Null pointer
    }

    // Convert the data in bytes to float
    if (bno055->unit_settings.accel_unit == BNO055_ACCEL_UNIT_MSQ) {
        *x = (float)((int16_t)(data[1] << 8 | data[0])) / 16.0f;
        *y = (float)((int16_t)(data[3] << 8 | data[2])) / 16.0f;
        *z = (float)((int16_t)(data[5] << 8 | data[4])) / 16.0f;
    } else {
        *x = (float)((int16_t)(data[1] << 8 | data[0])) / 1.0f;
        *y = (float)((int16_t)(data[3] << 8 | data[2])) / 1.0f;
        *z = (float)((int16_t)(data[5] << 8 | data[4])) / 1.0f;
    }
}

int8_t BNO055_Init(BNO055_t *bno055, uint8_t gpio_tx, uint8_t gpio_rx)
{
    int8_t success = BNO055_SUCCESS;

    printf("Initializing BNO055 sensor...\n");

    // Initialize the UART
    uart_init(&bno055->uart_config, 115200, 1024, gpio_tx, gpio_rx, UART_PIN_NO_USE, UART_PIN_NO_USE);


    // Set operation mode to CONFIGMODE
    bno055->operation_mode = INIT;
    success += BNO055_SetOperationMode(bno055, CONFIGMODE);
    
    // Write the default page as zero
    uint8_t data = BNO055_PAGE_ZERO;
    success += BN055_Write(bno055, BNO055_PAGE_ID_ADDR, &data, BNO055_GEN_READ_WRITE_LENGTH);

    // Set Unit to m/s^2, rad/s, rad, celcius, Windows orientation
    success += BNO055_SetUnit(bno055, BNO055_ACCEL_UNIT_MSQ, BNO055_GYRO_UNIT_DPS, BNO055_EULER_UNIT_RAD, BNO055_TEMP_UNIT_CELSIUS, BNO055_ANDROID_ORIENTATION);

    // Set Power mode to normal
    bno055->power_mode = LOWPOWER;
    success += BNO055_SetPowerMode(bno055, NORMAL);

    // Set operation mode to NDOF
    success += BNO055_SetOperationMode(bno055, NDOF);

    success += BNO055_GetInfo(bno055);
    

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

int8_t BNO055_GetCalibrationStatus(BNO055_t *bno055)
{
    if (bno055 == NULL) {
        printf("Error: Null pointer provided\n");
        return -1; // Error: Null pointer
    }

    // Read the calibration status from the BNO055 sensor
    uint8_t calib_status = 0;
    int8_t success = BNO055_Read(bno055, BNO055_CALIB_STAT_ADDR, &calib_status, 1, 10);

    // Print the calibration status
    printf("Calibration Status: %02X\n", calib_status);

    if (success != BNO055_SUCCESS) {
        printf("Error: Failed to read calibration status from the BNO055 sensor\n");
        return BNO055_ERROR;
    }

    // Update the calibration status
    bno055->calib_stat = calib_status;

    if (calib_status == BNO055_CALIB_STAT_OK) {
        // Update the calibration status
        printf("Calibration status is OK\n");
        return BNO055_SUCCESS;
    } else {
        printf("Calibration status is not OK\n");
        return BNO055_ERROR;
    }


    return BNO055_SUCCESS;
}

int8_t BNO055_GetInfo(BNO055_t *bno055)
{

    if (bno055 == NULL) {
        printf("Error: Null pointer provided\n");
        return -1; // Error: Null pointer
    }
    
    // // Get information from the BNO055 sensor
    uint8_t data_read[8] = {0};
    int8_t success = 0;
    success = BNO055_Read(bno055, BNO055_CHIP_ID_ADDR, data_read, 8, 16);

    printf("Data read: ");
    for (uint8_t i = 0; i < 8; i++) {
        printf("%02X ", data_read[i]);
    }
    printf("\n");

    // Read the chip ID
    bno055->chip_id = data_read[0];

    // Read accelerometer revision ID
    bno055->accel_rev_id = data_read[1];

    // Read magnetometer revision ID
    bno055->mag_rev_id = data_read[2];

    // Read gyroscope revision ID
    bno055->gyro_rev_id = data_read[3];

    // Read software revision ID LSB and MSB
    bno055->sw_rev_id[0] = data_read[4];
    bno055->sw_rev_id[1] = data_read[5];

    // Read bootloader revision ID
    bno055->bl_rev_id = data_read[6];

    // Read page ID
    bno055->page_id = data_read[7];

    if (success != BNO055_SUCCESS) {
        printf("Error: Failed to read data from the BNO055 sensor\n");
        return BNO055_ERROR;
    }
    

    return BNO055_SUCCESS;
}

void BNO055_GetEulerAngles(BNO055_t *bno055, float *yaw, float *pitch, float *roll)
{
    if (bno055 == NULL || yaw == NULL || pitch == NULL || roll == NULL) {
        printf("Error: Null pointer provided\n");
        return; // Error: Null pointer
    }

    int8_t success = 0;
    // Read the Euler angles from the BNO055 sensor
    uint8_t data[6] = {0};
    success = BNO055_Read(bno055, BNO055_EULER_H_LSB_ADDR, data, 6, 5);

    // Convert the data to float
    // Convert the data to radians (1 radian = 900 LSB)

    // If data is 0000 0000 0000 0000 give the last value
    if (success != BNO055_SUCCESS) {
        *yaw = bno055->yaw;
        *pitch = bno055->pitch;
        *roll = bno055->roll;
        return;
    }
    // Convert the data to float
    BNO055_ConvertData_Euler(bno055, data, yaw, pitch, roll);

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
    uart_read(&bno055->uart_config, buffer_rx, 2, 20);

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

int8_t BNO055_Read(BNO055_t *bno055, uint8_t reg, uint8_t *data, uint8_t len, uint8_t timeout_ms)
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
    // printf("Read operation\n");
    // printf("Buffer TX: ");
    // for (uint8_t i = 0; i < 4; i++) {
    //     printf("%02X ", buffer_tx[i]);
    // }
    // printf("\n");

    // Send the read operation to the BNO055 sensor
    uart_write(&bno055->uart_config, buffer_tx, 4);

    // // Read the data from the BNO055 sensor
    // uint8_t buffer_rx[128] = {0};
    
    //Clear the buffer of struct of bno055
    for (uint8_t i = 0; i < 128; i++) {
        bno055->buffer[i] = 0;
    }
    
    uart_read(&bno055->uart_config, bno055->buffer, len + 2, 20);

    // // Print the buffer
    printf("Buffer RX: ");
    for (uint8_t i = 0; i < len + 2; i++) {
        printf("%02X ", bno055->buffer[i]);
    }
    printf("\n");

    // Check if the read operation was successful
    if (bno055->buffer[0] == BNO055_READ_SUCCESS && bno055->buffer[1] == len)
    {

        for (uint8_t i = 0; i < len; i++) {
            data[i] = bno055->buffer[2 + i];
        }

        // printf("Data: ");
        // for (uint8_t i = 0; i < len; i++) {
        //     printf("%02X ", data[i]);
        // }

        return BNO055_SUCCESS;
    }
    else if (bno055->buffer[0] == BNO055_ACK_VALUE)
    {
        if (bno055->buffer[1] == BNO055_WRITE_SUCCESS && bno055->buffer[2] == BNO055_READ_SUCCESS && bno055->buffer[3] == len)
        {
           
            uint8_t buffer_rx[2] = {0};
            uart_read(&bno055->uart_config, buffer_rx, 2, 6);
            
             //Extend the length of the buffer for read 2 bytes more
             bno055->buffer[len + 2] = buffer_rx[0];
             bno055->buffer[len + 3] = buffer_rx[1];
             bno055->buffer[len + 4] = '\0';

            // Print the buffer extended
            // printf("Buffer RX Extended: ");
            // for (uint8_t i = 0; i < len; i++) {
            //     printf("%02X ", bno055->buffer[2 + i]);
            // }
            // printf("\n");

            for (uint8_t i = 0; i < len; i++) {
                data[i] = bno055->buffer[4 + i];
            }
            
        }

        else
        {
            return -3; // Error: Read operation failed
        }
    }
    else
    {
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

int8_t BNO055_ReadAll(BNO055_t *bno055)
{
    uint8_t data[24]; // 24 bytes of data (6 bytes for each value, Accelerometer, Magnetometer, Gyroscope, Euler angles)
    int8_t log = BNO055_Read(bno055, BNO055_ACCEL_DATA_X_LSB_ADDR, data, 24, 5);

    // Check if the read operation was successful
    if (log != BNO055_SUCCESS) {
        printf("Error: Failed to read data from the BNO055 sensor\n");
        return BNO055_ERROR;
    }

    // Convert the data to float
    uint8_t data_accel[6] = {0};
    uint8_t data_mag[6] = {0};
    uint8_t data_gyro[6] = {0};
    uint8_t data_euler[6] = {0};

    // Accelerometer data
    for (uint8_t i = 0; i < 6; i++) {
        data_accel[i] = data[i];
    }

    // Magnetometer data
    for (uint8_t i = 0; i < 6; i++) {
        data_mag[i] = data[6 + i];
    }

    // Gyroscope data
    for (uint8_t i = 0; i < 6; i++) {
        data_gyro[i] = data[12 + i];
    }

    // Euler angles data
    for (uint8_t i = 0; i < 6; i++) {
        data_euler[i] = data[18 + i];
    }


    // Convert the data to float
    BNO055_ConvertData_Accel(bno055, data_accel, &bno055->ax, &bno055->ay, &bno055->az);
    BNO055_ConvertData_Gyro(bno055, data_gyro, &bno055->gx, &bno055->gy, &bno055->gz);
    BNO055_ConvertData_Euler(bno055, data_euler, &bno055->yaw, &bno055->pitch, &bno055->roll);
    BNO055_ConvertData_Mag(bno055, data_mag, &bno055->mx, &bno055->my, &bno055->mz);

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
    BNO055_Read(bno055, BNO055_ACCEL_DATA_X_LSB_ADDR, data, 6, 5);

    // Convert the data to float
    BNO055_ConvertData_Accel(bno055, data, x, y, z);
}

void BNO055_GetGyro(BNO055_t *bno055, float *gx, float *gy, float *gz)
{
    uint8_t data[6];
    BNO055_Read(bno055, BNO055_GYRO_DATA_X_LSB_ADDR, data, 6, 5);

    // Convert the data to float
    BNO055_ConvertData_Gyro(bno055, data, gx, gy, gz);
}

int8_t BNO055_SetUnit(BNO055_t *bno055, uint8_t accel_unit, uint8_t gyro_unit, uint8_t euler_unit, uint8_t temp_unit, uint8_t ori_unit)
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

    // Check if the write operation was successful
    if (success == BNO055_SUCCESS) {
        // Update the unit values
        bno055->unit_settings.accel_unit = accel_unit;
        bno055->unit_settings.gyro_unit = gyro_unit;
        bno055->unit_settings.euler_unit = euler_unit;
        bno055->unit_settings.temp_unit = temp_unit;
        bno055->unit_settings.ori_unit = ori_unit;


    } else {
        return BNO055_ERROR;
    }

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

int8_t BNO055_GetCalibrationProfile(BNO055_t *bno055, BNO055_CalibProfile_t *calib_data)
{
    if (bno055 == NULL || calib_data == NULL) {
        printf("Error: Null pointer provided\n");
        return BNO055_ERROR; // Error: Null pointer
    }

    // Get the calibration status
    BNO055_GetCalibrationStatus(bno055);
    if (bno055->calib_stat != BNO055_CALIB_STAT_OK) {
        printf("Error: Calibration status is not OK\n");
        return BNO055_ERROR;
    }

    /* Save the current operation mode */
    BNO055_OperationMode current_mode = bno055->operation_mode;

    // Switch to CONFIG_MODE to read calibration profile
    int8_t success = BNO055_SetOperationMode(bno055, CONFIGMODE);
    if (success != BNO055_SUCCESS) {
        printf("Error: Failed to switch to CONFIG_MODE\n");
        return BNO055_ERROR;
    }
    

    // Read sensor offsets
    uint8_t calib_offsets[22] = {0};
    success = BNO055_Read(bno055, BNO055_ACCEL_OFFSET_X_LSB_ADDR, calib_offsets, 22, 12);
    if (success != BNO055_SUCCESS) {
        printf("Error: Failed to read calibration offsets\n");
        // Switch back to the previous operation mode
        BNO055_SetOperationMode(bno055, current_mode);
        return BNO055_ERROR;
    }

    // Parse accelerometer offsets
    calib_data->accel_offset_x = (int16_t)(calib_offsets[1] << 8 | calib_offsets[0]);
    calib_data->accel_offset_y = (int16_t)(calib_offsets[3] << 8 | calib_offsets[2]);
    calib_data->accel_offset_z = (int16_t)(calib_offsets[5] << 8 | calib_offsets[4]);

    // Parse magnetometer offsets
    calib_data->mag_offset_x = (int16_t)(calib_offsets[7] << 8 | calib_offsets[6]);
    calib_data->mag_offset_y = (int16_t)(calib_offsets[9] << 8 | calib_offsets[8]);
    calib_data->mag_offset_z = (int16_t)(calib_offsets[11] << 8 | calib_offsets[10]);

    // Parse gyroscope offsets
    calib_data->gyro_offset_x = (int16_t)(calib_offsets[13] << 8 | calib_offsets[12]);
    calib_data->gyro_offset_y = (int16_t)(calib_offsets[15] << 8 | calib_offsets[14]);
    calib_data->gyro_offset_z = (int16_t)(calib_offsets[17] << 8 | calib_offsets[16]);

    // Parse sensor radius
    calib_data->accel_radius = (int16_t)(calib_offsets[19] << 8 | calib_offsets[18]);
    calib_data->mag_radius = (int16_t)(calib_offsets[21] << 8 | calib_offsets[20]);

    // Switch back to the previous operation mode
    success = BNO055_SetOperationMode(bno055, current_mode);
    if (success != BNO055_SUCCESS) {
        printf("Error: Failed to switch back to previous operation mode\n");
        return BNO055_ERROR;
    }

    // Print the calibration profile HEX
    printf("------------------------------------------------------\n");
    printf("Calibration Profile:\n");
    printf("Accel Offsets X: %04X\n, y: %04X\n, z: %04X\n", calib_data->accel_offset_x, calib_data->accel_offset_y, calib_data->accel_offset_z);
    printf("Mag Offsets X: %04X\n, y: %04X\n, z: %04X\n", calib_data->mag_offset_x, calib_data->mag_offset_y, calib_data->mag_offset_z);
    printf("Gyro Offsets X: %04X\n, y: %04X\n, z: %04X\n", calib_data->gyro_offset_x, calib_data->gyro_offset_y, calib_data->gyro_offset_z);
    printf("Accel Radius: %04X\n", calib_data->accel_radius);
    printf("Mag Radius: %04X\n", calib_data->mag_radius);
    printf("------------------------------------------------------\n");
    return BNO055_SUCCESS;
}
