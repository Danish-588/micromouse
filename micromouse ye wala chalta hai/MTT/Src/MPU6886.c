#include "MPU6886.h"
#include <stdint.h>

// Function to initialize the MPU6886
HAL_StatusTypeDef MPU6886_Init(MPU6886_Handle *handle) {
    uint8_t data = 0x00; // Reset the device
    HAL_StatusTypeDef status;

    // Write to PWR_MGMT_1 register to wake up the MPU6886
    data = 0x00;
    status = HAL_I2C_Mem_Write(handle->i2cHandle, MPU6886_ADDRESS << 1, MPU6886_PWR_MGMT_1, 1, &data, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    // Configure the gyroscope to ±250 °/s
    data = 0x00;
    status = HAL_I2C_Mem_Write(handle->i2cHandle, MPU6886_ADDRESS << 1, MPU6886_GYRO_CONFIG, 1, &data, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    // Configure the accelerometer to ±2g
    data = 0x00;
    status = HAL_I2C_Mem_Write(handle->i2cHandle, MPU6886_ADDRESS << 1, MPU6886_ACCEL_CONFIG, 1, &data, 1, HAL_MAX_DELAY);
    return status;
}

// Function to read gyroscope data
HAL_StatusTypeDef MPU6886_ReadGyroData(MPU6886_Handle *handle, float *gx_deg, float *gy_deg, float *gz_deg, float gyroBiasX, float gyroBiasY, float gyroBiasZ) {
    uint8_t buffer[6];
    HAL_StatusTypeDef status;

    // Read raw gyroscope data from the MPU6886
    status = HAL_I2C_Mem_Read(handle->i2cHandle, MPU6886_ADDRESS << 1, MPU6886_GYRO_XOUT_H, 1, buffer, 6, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    // Combine high and low bytes to form 16-bit raw data for each axis
    int16_t raw_gx = (int16_t)(buffer[0] << 8 | buffer[1]);
    int16_t raw_gy = (int16_t)(buffer[2] << 8 | buffer[3]);
    int16_t raw_gz = (int16_t)(buffer[4] << 8 | buffer[5]);

    // Convert raw data to degrees per second and subtract the biases
    *gx_deg = (raw_gx / MPU6886_GYRO_SCALE_250DPS) - gyroBiasX;
    *gy_deg = (raw_gy / MPU6886_GYRO_SCALE_250DPS) - gyroBiasY;
    *gz_deg = (raw_gz / MPU6886_GYRO_SCALE_250DPS) - gyroBiasZ;

    return HAL_OK;
}


// Function to read accelerometer data
HAL_StatusTypeDef MPU6886_ReadAccelData(MPU6886_Handle *handle, float *ax, float *ay, float *az) {
    uint8_t buffer[6];
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(handle->i2cHandle, MPU6886_ADDRESS << 1, MPU6886_ACCEL_XOUT_H, 1, buffer, 6, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    int16_t raw_ax = (int16_t)(buffer[0] << 8 | buffer[1]);
    int16_t raw_ay = (int16_t)(buffer[2] << 8 | buffer[3]);
    int16_t raw_az = (int16_t)(buffer[4] << 8 | buffer[5]);

    *ax = raw_ax / MPU6886_ACCEL_SCALE_2G;
    *ay = raw_ay / MPU6886_ACCEL_SCALE_2G;
    *az = raw_az / MPU6886_ACCEL_SCALE_2G;

    return HAL_OK;
}

// Function to read temperature data
HAL_StatusTypeDef MPU6886_ReadTempData(MPU6886_Handle *handle, float *temperature) {
    uint8_t buffer[2];
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(handle->i2cHandle, MPU6886_ADDRESS << 1, MPU6886_TEMP_OUT_H, 1, buffer, 2, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    int16_t raw_temp = (int16_t)(buffer[0] << 8 | buffer[1]);
    *temperature = (raw_temp / 340.0) + 36.53; // Conversion formula for temperature data

    return HAL_OK;
}
