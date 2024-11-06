#include "MPU6886.h"
#include <stdint.h>

HAL_StatusTypeDef MPU6886_Init(MPU6886_Handle *handle)
{
    uint8_t data = 0x00; // Reset the device
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(handle->i2cHandle, MPU6886_ADDRESS << 1, MPU6886_PWR_MGMT_1, 1, &data, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    // Set gyroscope full-scale range to ±250 °/s
    data = 0x00;
    status = HAL_I2C_Mem_Write(handle->i2cHandle, MPU6886_ADDRESS << 1, MPU6886_GYRO_CONFIG, 1, &data, 1, HAL_MAX_DELAY);

    return status;
}

HAL_StatusTypeDef MPU6886_GetGyroData(MPU6886_Handle *handle, float *gx_deg, float *gy_deg, float *gz_deg)
{
    uint8_t buffer[6];
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(handle->i2cHandle, MPU6886_ADDRESS << 1, MPU6886_GYRO_XOUT_H, 1, buffer, 6, HAL_MAX_DELAY);

    if (status == HAL_OK) {
        int16_t raw_gx = (int16_t)((buffer[0] << 8) | buffer[1]);
        int16_t raw_gy = (int16_t)((buffer[2] << 8) | buffer[3]);
        int16_t raw_gz = (int16_t)((buffer[4] << 8) | buffer[5]);

        *gx_deg = raw_gx / MPU6886_GYRO_SCALE_250DPS;
        *gy_deg = raw_gy / MPU6886_GYRO_SCALE_250DPS;
        *gz_deg = raw_gz / MPU6886_GYRO_SCALE_250DPS;
    }
    return status;
}
