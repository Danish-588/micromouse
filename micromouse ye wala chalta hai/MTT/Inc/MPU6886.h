#ifndef MPU6886_H
#define MPU6886_H

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"

// Define MPU6886 I2C address
#define MPU6886_ADDRESS 0x68 // Adjust according to your device configuration

// Register Addresses
#define MPU6886_PWR_MGMT_1 0x6B
#define MPU6886_ACCEL_XOUT_H 0x3B
#define MPU6886_GYRO_XOUT_H 0x43
#define MPU6886_TEMP_OUT_H 0x41

typedef struct
{
    I2C_HandleTypeDef *i2cHandle; // Pointer to I2C handle
} MPU6886_Handle;

// Function prototypes
HAL_StatusTypeDef MPU6886_Init(MPU6886_Handle *handle);
HAL_StatusTypeDef MPU6886_GetAccelData(MPU6886_Handle *handle, float *ax, float *ay, float *az);
HAL_StatusTypeDef MPU6886_GetGyroData(MPU6886_Handle *handle, float *gx_rad, float *gy_rad, float *gz_rad, float *gx_deg, float *gy_deg, float *gz_deg);
HAL_StatusTypeDef MPU6886_GetTempData(MPU6886_Handle *handle, float *t);

#endif // MPU6886_H
