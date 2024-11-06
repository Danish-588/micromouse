#ifndef MPU6886_H
#define MPU6886_H

#include "stm32f4xx_hal.h"

// MPU6886 I2C address
#define MPU6886_ADDRESS            0x68

// MPU6886 Register addresses
#define MPU6886_PWR_MGMT_1         0x6B
#define MPU6886_GYRO_CONFIG        0x1B
#define MPU6886_GYRO_XOUT_H        0x43

// Scale factor for ±250 °/s
#define MPU6886_GYRO_SCALE_250DPS  131.0

typedef struct {
    I2C_HandleTypeDef *i2cHandle;
} MPU6886_Handle;

// MPU6886 function prototypes
HAL_StatusTypeDef MPU6886_Init(MPU6886_Handle *handle);
HAL_StatusTypeDef MPU6886_GetGyroData(MPU6886_Handle *handle, float *gx_deg, float *gy_deg, float *gz_deg);

#endif // MPU6886_H
