#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "MPU6886.h"
//#include "vl53l1x.h" // Include the VL53L1X header
#include "i2c.h"


// Define handles for I2C
I2C_HandleTypeDef hi2c1;

// IMU handle
MPU6886_Handle imu6886;

// Global variables to hold sensor data
float accX = 0, accY = 0, accZ = 0;
float gyroX_rad = 0, gyroY_rad = 0, gyroZ_rad = 0; // Gyroscope data in radians
float gyroX_deg = 0, gyroY_deg = 0, gyroZ_deg = 0; // Gyroscope data in degrees
float temp = 0;
uint16_t distance = 0; // Variable to hold distance from VL53L1X

// Global variables for status tracking
HAL_StatusTypeDef acc_status = HAL_OK, gyro_status = HAL_OK, temp_status = HAL_OK, init_status = HAL_OK;

// Function prototypes
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MPU6886_Scan(void); // Function to scan I2C devices

// Function to toggle LED
void LED_Blink(void);

int main(void)
{
    // Initialize the HAL Library
    HAL_Init();

    // Configure the system clock
    SystemClock_Config();

    // Initialize all configured peripherals
    MX_GPIO_Init();
    MX_I2C1_Init();

    // Scan for I2C devices
    MPU6886_Scan();

    // Initialize MPU6886 (IMU)
    imu6886.i2cHandle = &hi2c1;
    init_status = MPU6886_Init(&imu6886);

    // Initialize VL53L1X (Distance sensor)
    // VL53L1X_Init(&hi2c1);

    while (1)
    {
        // Read accelerometer data from MPU6886
        acc_status = MPU6886_GetAccelData(&imu6886, &accX, &accY, &accZ);

        // Read gyroscope data from MPU6886 in radians and degrees
        gyro_status = MPU6886_GetGyroData(&imu6886, &gyroX_rad, &gyroY_rad, &gyroZ_rad, &gyroX_deg, &gyroY_deg, &gyroZ_deg);

        // Read temperature data from MPU6886
        temp_status = MPU6886_GetTempData(&imu6886, &temp);

        // Read distance data from VL53L1X and store in the distance variable
        // VL53L1X_ReadDistance(&hi2c1, &distance);

        // Toggle the LED
        LED_Blink();

        // Add a small delay to control the sampling rate
        HAL_Delay(100); // Delay in milliseconds
    }
}

// Function to toggle LED on C13
void LED_Blink(void)
{
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // Toggle the LED state
    HAL_Delay(500); // Delay for 500 ms (adjust as necessary for blink rate)
}

// Initialize I2C1
static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;                       // 100kHz standard mode
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;  // 7-bit addressing
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        // Initialization Error
        init_status = HAL_ERROR;
        Error_Handler();
    }
}

// Function to scan I2C devices
static void MPU6886_Scan(void)
{
    for (uint8_t i = 0; i < 128; i++)
    {
        if (HAL_I2C_IsDeviceReady(&hi2c1, (i << 1), 1, 100) == HAL_OK)
        {
            // Device found at address i
            // Optionally, you can set a variable here or log it
        }
    }
}

// GPIO Initialization
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();                         // Enable GPIOA clock
    __HAL_RCC_GPIOC_CLK_ENABLE();                         // Enable GPIOC clock

    // Configure GPIO pin C13 (usually the on-board LED)
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_13;                    // Pin 13
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;           // Push-pull mode
    GPIO_InitStruct.Pull = GPIO_NOPULL;                   // No pull-up or pull-down resistors
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;          // Low frequency
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);               // Initialize GPIOC
}

// Error handler function
void Error_Handler(void)
{
    while (1)
    {
        // Stay here to help debug
    }
}
