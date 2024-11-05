#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "MPU6886.h"
#include "i2c.h"
#include "encoder.h"

// Define handles for I2C and timers
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2; // For PWM generation on TIM2
TIM_HandleTypeDef htim4; // For encoder on TIM4

// IMU handle
MPU6886_Handle imu6886;

float Kp = 1.0;  // Proportional gain
float Ki = 0.1;  // Integral gain
float Kd = 0.05; // Derivative gain

uint32_t pwm_value1= 0;
uint32_t pwm_value2 = 0;
float target_velocity = 1000; // Desired velocity in encoder counts per minute
float previous_error = 0;
float integral = 0;

// Global variables to hold sensor data
float accX = 0, accY = 0, accZ = 0;
float gyroX_rad = 0, gyroY_rad = 0, gyroZ_rad = 0; // Gyroscope data in radians
float gyroX_deg = 0, gyroY_deg = 0, gyroZ_deg = 0; // Gyroscope data in degrees
float temp = 0;
volatile int count = 0;
volatile int velocity1 = 0;
volatile int velocity2= 0;
uint16_t distance = 0; // Distance variable for VL53L1X sensor

// PWM parameters for debugging
uint32_t pwm_frequency = 1000;  // 1 kHz default frequency
uint32_t duty_cycle = 69;       // 50% default duty cycle

// Global variables for status tracking
HAL_StatusTypeDef acc_status = HAL_OK, gyro_status = HAL_OK, temp_status = HAL_OK, init_status = HAL_OK;

// Function prototypes
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MPU6886_Scan(void);
static void MX_TIM2_Init(void); // TIM2 for PWM generation
static void MX_TIM1_Init(void); // TIM2 for PWM generation
static void MX_TIM4_Init(void); // TIM4 for encoder
void LED_Blink(void);

// Function to calculate the PID output
uint32_t PID_CalculatePWM1(float current_velocity)
{
    float error = target_velocity - current_velocity;
    integral += error;                     // Accumulate integral
    float derivative = error - previous_error; // Calculate derivative
    previous_error = error;

    // PID formula
    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

    // Calculate PWM duty cycle (clamp output to valid range 0-100%)
    pwm_value1 = (uint32_t)((htim2.Init.Period + 1) * output / 100.0);
    if (pwm_value1 > htim2.Init.Period) pwm_value1 = htim2.Init.Period;
    if (pwm_value1 < 0) pwm_value1 = 0;

    return pwm_value1;
}
uint32_t PID_CalculatePWM2(float current_velocity)
{
    float error = target_velocity - current_velocity;
    integral += error;                     // Accumulate integral
    float derivative = error - previous_error; // Calculate derivative
    previous_error = error;

    // PID formula
    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

    // Calculate PWM duty cycle (clamp output to valid range 0-100%)
    pwm_value2 = (uint32_t)((htim2.Init.Period + 1) * output / 100.0);
//    if (pwm_value > htim2.Init.Period) pwm_value = htim2.Init.Period;
//    if (pwm_value < 0) pwm_value = 0;

    return pwm_value1;
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    // Initialize all configured peripherals
    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM4_Init();
    MX_I2C1_Init();

    Encoder_Init(&htim1, 3);
    Encoder_Init(&htim4, 3);


    // Initialize MPU6886 (IMU)
    imu6886.i2cHandle = &hi2c1;
    init_status = MPU6886_Init(&imu6886);



    // Start PWM on TIM2, Channel 2
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK) {
        // Handle error (e.g., light up an error LED)
        Error_Handler();
    }
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK) {
        // Handle error (e.g., light up an error LED)
        Error_Handler();
    }

    while (1)
    {
    	velocity1 = Encoder_GetVelocity(&htim1);
        // Get encoder velocity
        velocity2 = Encoder_GetVelocity(&htim4); // Assuming velocity is in counts per minute

        // Calculate PWM based on PID control
        uint32_t pwm_value1 = PID_CalculatePWM1(velocity1);

        uint32_t pwm_value2 = PID_CalculatePWM2(velocity2);

        // Update duty cycle with the new PID-calculated value
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_value1);

        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_value2);

        // Optional: Add debugging information, toggle LED, etc.
        LED_Blink();

        // Add a small delay to control the sampling rate
//        HAL_Delay(100); // Delay in milliseconds
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
// Initialize TIM1 for encoder
void MX_TIM1_Init(void)
{
    __HAL_RCC_TIM1_CLK_ENABLE();

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 0xFFFF;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;

    TIM_Encoder_InitTypeDef encoderConfig = {0};
    encoderConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    encoderConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    encoderConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    encoderConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    encoderConfig.IC1Filter = 0;
    encoderConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    encoderConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    encoderConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    encoderConfig.IC2Filter = 0;

    if (HAL_TIM_Encoder_Init(&htim1, &encoderConfig) != HAL_OK)
    {
        // Initialization Error
        Error_Handler();
    }

    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
}


// Initialize TIM2 for PWM generation
void MX_TIM2_Init(void) {
    TIM_OC_InitTypeDef sConfigOC = {0};

    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = (SystemCoreClock / 1000000) - 1; // 1 MHz timer frequency
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = (1000000 / pwm_frequency) - 1; // Set PWM frequency
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim2);

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = (htim2.Init.Period + 1) * duty_cycle / 100; // Set duty cycle
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2); // Use TIM_CHANNEL_2
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
}

// Initialize TIM4 for encoder
void MX_TIM4_Init(void)
{
    TIM_Encoder_InitTypeDef encoderConfig = {0};

    __HAL_RCC_TIM4_CLK_ENABLE();

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 0;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 0xFFFF;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

    encoderConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    encoderConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    encoderConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    encoderConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    encoderConfig.IC1Filter = 0;
    encoderConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    encoderConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    encoderConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    encoderConfig.IC2Filter = 0;

    if (HAL_TIM_Encoder_Init(&htim4, &encoderConfig) != HAL_OK)
    {
        // Initialization Error
        Error_Handler();
    }

    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
}

// Function to scan I2C devices
static void MPU6886_Scan(void)
{
    for (uint8_t i = 0; i < 128; i++)
    {
        if (HAL_I2C_IsDeviceReady(&hi2c1, (i << 1), 1, 100) == HAL_OK)
        {
            // Device found at address i
        }
    }
}

// GPIO Initialization
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configure GPIO pins B6 and B7 for TIM4 encoder input
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Configure GPIO pins A8 and A9 for TIM1 encoder input
       GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
       GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
       GPIO_InitStruct.Pull = GPIO_NOPULL;
       GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
       GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
       HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

       // Configure GPIO pin PA0 for TIM2 Channel 1 (PWM output)
       GPIO_InitStruct.Pin = GPIO_PIN_0;
       GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
       GPIO_InitStruct.Pull = GPIO_NOPULL;
       GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
       GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
       HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Configure GPIO pin A1 for TIM2 Channel 2 (PWM output)
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Configure GPIO pin C13 for LED output
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

// Error Handler
void Error_Handler(void)
{
    while (1)
    {
        // Implement error handling here
    }
}

