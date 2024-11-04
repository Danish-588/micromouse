#include "i2c.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"



// Function to initialize I2C
HAL_StatusTypeDef I2C_Init(I2C_HandleTypeDef *hi2c)
{
    // I2C configuration can be done here or use the initialization function in main
    if (HAL_I2C_Init(hi2c) != HAL_OK)
    {
        return HAL_ERROR; // Initialization Error
    }
    return HAL_OK;
}

// Function to write data to an I2C device
HAL_StatusTypeDef I2C_Write(I2C_HandleTypeDef *hi2c, uint16_t devAddr, uint8_t *pData, uint16_t size)
{
    return HAL_I2C_Master_Transmit(hi2c, devAddr << 1, pData, size, HAL_MAX_DELAY);
}

// Function to read data from an I2C device
HAL_StatusTypeDef I2C_Read(I2C_HandleTypeDef *hi2c, uint16_t devAddr, uint8_t *pData, uint16_t size)
{
    return HAL_I2C_Master_Receive(hi2c, devAddr << 1, pData, size, HAL_MAX_DELAY);
}

// Function to check if a device is ready on the I2C bus
HAL_StatusTypeDef I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c, uint16_t devAddr)
{
    return HAL_I2C_IsDeviceReady(hi2c, devAddr << 1, 1, 100);
}
