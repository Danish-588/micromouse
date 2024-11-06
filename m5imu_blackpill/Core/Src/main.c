#include "main.h"
#include "i2c.h"
#include "gpio.h"  // Add this to include the MX_GPIO_Init() function


float yaw = 0.0f;
uint32_t prevTime = 0;

// MPU6886 Registers
#define MPU6886_ADDRESS           0x68 << 1
#define MPU6886_PWR_MGMT_1        0x6B
#define MPU6886_ACCEL_XOUT_H      0x3B
#define MPU6886_GYRO_XOUT_H       0x43
#define MPU6886_TEMP_OUT_H        0x41

// Global variables for sensor values
float sensorData[7]; // For accX, accY, accZ, gyroX, gyroY, gyroZ, and temp
HAL_StatusTypeDef init_status;
I2C_HandleTypeDef hi2c1;

// Function prototypes
void SystemClock_Config(void);
void MPU6886_Init(void);
void MPU6886_ReadAccel(float* ax, float* ay, float* az);
void MPU6886_ReadGyro(float* gx, float* gy, float* gz);
void MPU6886_ReadTemp(float* temp);

// Function to initialize GPIOs (for I2C with pull-up resistors enabled)
void MX_GPIO_Init(void)
{
    // Enable the GPIO clocks for the pins you are using (e.g., GPIOB for I2C)
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configure I2C SCL (PB6) and SDA (PB7) as alternate function, open-drain with pull-up resistors
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;  // PB6 -> SCL, PB7 -> SDA
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;         // Alternate function, open-drain
    GPIO_InitStruct.Pull = GPIO_PULLUP;             // Enable pull-up resistors
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;  // Set high speed
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;      // Set alternate function AF4 for I2C1
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);         // Initialize GPIOB with the configuration
}


// Function to initialize I2C1 peripheral
void MX_I2C1_Init(void)
{
    // Set the I2C instance to I2C1
    hi2c1.Instance = I2C1;

    // Initialize the I2C configuration parameters
    hi2c1.Init.ClockSpeed = 100000;                       // Standard mode (100kHz)
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;               // Duty cycle for standard mode
    hi2c1.Init.OwnAddress1 = 0;                           // No specific own address
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;  // Use 7-bit addressing
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE; // Disable dual addressing mode
    hi2c1.Init.OwnAddress2 = 0;                           // No second address
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE; // Disable general call
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;     // Enable clock stretching

    // Initialize the I2C peripheral
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        // If there is an initialization error, log the status and handle the error
        init_status = HAL_ERROR;  // You can monitor this value in the debugger
        Error_Handler();
    }

}



int main(void)
{
    // Initialize the HAL Library
    HAL_Init();

    // Configure the system clock
    SystemClock_Config();

    // Initialize GPIO and I2C
    MX_GPIO_Init();  // Initialize GPIO (regenerated by STM32CubeMX)
    MX_I2C1_Init();  // Initialize I2C (regenerated by STM32CubeMX)

    // Initialize MPU6886
    MPU6886_Init();

    prevTime = HAL_GetTick();

    while (1)
    {
        // Read sensor data
        MPU6886_ReadAccel(&sensorData[0], &sensorData[1], &sensorData[2]);
        MPU6886_ReadGyro(&sensorData[3], &sensorData[4], &sensorData[5]);
        MPU6886_ReadTemp(&sensorData[6]);

        // Calculate time difference in seconds
        uint32_t currentTime = HAL_GetTick();
        float deltaTime = (currentTime - prevTime) / 1000.0f; // Convert ms to s
        prevTime = currentTime;

        // Integrate gyroscope Z-axis data to calculate yaw
        yaw += sensorData[5] * deltaTime; // sensorData[5] is gyroZ

        // Optional: Normalize yaw to 0-360 degrees
        if (yaw >= 360.0f)
            yaw -= 360.0f;
        else if (yaw < 0.0f)
            yaw += 360.0f;

        // Delay to sample periodically
//        HAL_Delay(1000);
    }
}


// MPU6886 Initialization Function
void MPU6886_Init(void)
{
    uint8_t data = 0x00;

    // Check if MPU6886 is ready (I2C address 0x68)
    if (HAL_I2C_IsDeviceReady(&hi2c1, MPU6886_ADDRESS, 3, HAL_MAX_DELAY) != HAL_OK)
    {
        // If the device is not ready, handle the error
        Error_Handler();
    }

    // Initialize MPU6886 by writing to its power management register
    if (HAL_I2C_Mem_Write(&hi2c1, MPU6886_ADDRESS, MPU6886_PWR_MGMT_1, 1, &data, 1, HAL_MAX_DELAY) != HAL_OK)
    {
        // Handle the error if writing fails
        Error_Handler();
    }
}


// Function to read accelerometer data
void MPU6886_ReadAccel(float* ax, float* ay, float* az)
{
    uint8_t buffer[6];
    HAL_StatusTypeDef acc_status = HAL_I2C_Mem_Read(&hi2c1, MPU6886_ADDRESS, MPU6886_ACCEL_XOUT_H, 1, buffer, 6, HAL_MAX_DELAY);

    if (acc_status == HAL_OK)
    {
        *ax = (int16_t)(buffer[0] << 8 | buffer[1]) / 16384.0;
        *ay = (int16_t)(buffer[2] << 8 | buffer[3]) / 16384.0;
        *az = (int16_t)(buffer[4] << 8 | buffer[5]) / 16384.0;
    }
    else
    {
        *ax = *ay = *az = 0; // If reading fails, set to 0 for debugging
    }
}

// Function to read gyroscope data
void MPU6886_ReadGyro(float* gx, float* gy, float* gz)
{
    uint8_t buffer[6];
    HAL_StatusTypeDef gyro_status = HAL_I2C_Mem_Read(&hi2c1, MPU6886_ADDRESS, MPU6886_GYRO_XOUT_H, 1, buffer, 6, HAL_MAX_DELAY);

    if (gyro_status == HAL_OK)
    {
        *gx = (int16_t)(buffer[0] << 8 | buffer[1]) / 131.0;
        *gy = (int16_t)(buffer[2] << 8 | buffer[3]) / 131.0;
        *gz = (int16_t)(buffer[4] << 8 | buffer[5]) / 131.0;
    }
    else
    {
        *gx = *gy = *gz = 0; // If reading fails, set to 0 for debugging
    }
}

// Function to read temperature data
void MPU6886_ReadTemp(float* temp)
{
    uint8_t buffer[2];
    HAL_StatusTypeDef temp_status = HAL_I2C_Mem_Read(&hi2c1, MPU6886_ADDRESS, MPU6886_TEMP_OUT_H, 1, buffer, 2, HAL_MAX_DELAY);

    if (temp_status == HAL_OK)
    {
        int16_t temp_raw = (int16_t)(buffer[0] << 8 | buffer[1]);
        *temp = (temp_raw / 340.0) + 36.53;
    }
    else
    {
        *temp = 0; // If reading fails, set to 0 for debugging
    }
}

// System Clock Configuration (generated by STM32CubeMX)
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        while (1);
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        while (1);
    }
}

void Error_Handler(void)
{
    while (1)
    {
        // Implement some error handling here (LED blinking, etc.)
    }
}
