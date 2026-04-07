#ifndef I2C_UTILS_H
#define I2C_UTILS_H

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"

// Function prototypes
HAL_StatusTypeDef I2C_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef I2C_Write(I2C_HandleTypeDef *hi2c, uint16_t devAddr, uint8_t *pData, uint16_t size);
HAL_StatusTypeDef I2C_Read(I2C_HandleTypeDef *hi2c, uint16_t devAddr, uint8_t *pData, uint16_t size);
HAL_StatusTypeDef I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c, uint16_t devAddr);

#endif // I2C_UTILS_H
