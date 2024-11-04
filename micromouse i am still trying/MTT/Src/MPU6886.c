#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include <math.h> // Include for M_PI
#include "MPU6886.h"

// Function to initialize the MPU6886
HAL_StatusTypeDef MPU6886_Init(MPU6886_Handle *handle)
{
    uint8_t data = 0x00; // Reset the device
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(handle->i2cHandle, MPU6886_ADDRESS << 1, MPU6886_PWR_MGMT_1, 1, &data, 1, HAL_MAX_DELAY);

    // Optionally add other configuration commands here
    // For example, setting accelerometer range, gyroscope range, etc.

    return status; // Return the status of the initialization
}

// Function to get accelerometer data
HAL_StatusTypeDef MPU6886_GetAccelData(MPU6886_Handle *handle, float *ax, float *ay, float *az)
{
    uint8_t buffer[6];
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(handle->i2cHandle, MPU6886_ADDRESS << 1, MPU6886_ACCEL_XOUT_H, 1, buffer, 6, HAL_MAX_DELAY);

    if (status == HAL_OK) {
        // Convert raw data to g (gravity)
        *ax = ((int16_t)(buffer[0] << 8 | buffer[1])) / 16384.0; // Adjust scale as necessary
        *ay = ((int16_t)(buffer[2] << 8 | buffer[3])) / 16384.0;
        *az = ((int16_t)(buffer[4] << 8 | buffer[5])) / 16384.0;
    }

    return status;
}

// Function to get gyroscope data
HAL_StatusTypeDef MPU6886_GetGyroData(MPU6886_Handle *handle, float *gx_rad, float *gy_rad, float *gz_rad, float *gx_deg, float *gy_deg, float *gz_deg)
{
    uint8_t buffer[6];
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(handle->i2cHandle, MPU6886_ADDRESS << 1, MPU6886_GYRO_XOUT_H, 1, buffer, 6, HAL_MAX_DELAY);

    if (status == HAL_OK) {
        // Convert raw data to degrees/sec
        int16_t raw_gx = (int16_t)(buffer[0] << 8 | buffer[1]);
        int16_t raw_gy = (int16_t)(buffer[2] << 8 | buffer[3]);
        int16_t raw_gz = (int16_t)(buffer[4] << 8 | buffer[5]);

        *gx_rad = raw_gx * (M_PI / 180) / 131.0; // Convert to radians
        *gy_rad = raw_gy * (M_PI / 180) / 131.0;
        *gz_rad = raw_gz * (M_PI / 180) / 131.0;

        // Convert to degrees
        *gx_deg = raw_gx / 131.0; // Degrees/sec
        *gy_deg = raw_gy / 131.0;
        *gz_deg = raw_gz / 131.0;
    }

    return status;
}

// Function to get temperature data
HAL_StatusTypeDef MPU6886_GetTempData(MPU6886_Handle *handle, float *t)
{
    uint8_t buffer[2];
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(handle->i2cHandle, MPU6886_ADDRESS << 1, MPU6886_TEMP_OUT_H, 1, buffer, 2, HAL_MAX_DELAY);

    if (status == HAL_OK) {
        int16_t raw_temp = (int16_t)(buffer[0] << 8 | buffer[1]);
        *t = raw_temp / 326.8 + 25.0; // Convert to Celsius
    }

    return status;
}
