#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "MPU6886.h"
#include "i2c.h"
#include "encoder.h"
#include "vl53l0x_api.h"


I2C_HandleTypeDef hi2c2;


// Define handles for I2C and timers
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2; // For PWM generation on TIM2
TIM_HandleTypeDef htim4; // For encoder on TIM4


uint8_t Message[64];
uint8_t MessageLen;

VL53L0X_RangingMeasurementData_t RangingData;
VL53L0X_Dev_t  vl53l0x_c; // center module
VL53L0X_DEV    Dev = &vl53l0x_c;


// IMU handle
MPU6886_Handle imu6886;


int old_vel1=0, old_vel2=0;

float roll = 0.0, pitch = 0.0, yaw = 0.0;
int gyro_calib = 0;
int t1=0,t2=0;
// Global variables for gyroscope bias
float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;

float Kp = 1.0;  // Proportional gain
float Ki = 0.0;  // Integral gain
float Kd = 0.05; // Derivative gain

float target_velocity = 1000; // Desired velocity in encoder counts per minute
float previous_error = 0;
float integral = 0;

// Global variables to hold sensor data
float accX = 0, accY = 0, accZ = 0;
float gyroX_rad = 0, gyroY_rad = 0, gyroZ_rad = 0; // Gyroscope data in radians
   float yaw_angle = 0.0;
   volatile uint32_t prevTime = 0;
float gx_deg = 0, gy_deg = 0, gz_deg = 0; // Gyroscope data in degrees
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
//static void MX_I2C1_Init(void);
static void MPU6886_Scan(void);
static void MX_TIM2_Init(void); // TIM2 for PWM generation
static void MX_TIM1_Init(void); // TIM2 for PWM generation
static void MX_TIM4_Init(void); // TIM4 for encoder
void LED_Blink(void);
void MX_I2C2_Init(void);


void CalibrateGyro(MPU6886_Handle *handle, float *gyroBiasX, float *gyroBiasY, float *gyroBiasZ);
void UpdateGyroBiasIfStationary(void);
HAL_StatusTypeDef MPU6886_ReadGyroData(MPU6886_Handle *handle, float *gx_deg, float *gy_deg, float *gz_deg, float gyroBiasX, float gyroBiasY, float gyroBiasZ);


float sensorData[7]; // Adjust size based on your usage

// PID Control Variables for Motor 1
float Kp1 = 10.0f;  // Proportional gain
float Ki1 = 0.0f;  // Integral gain
float Kd1 = 0.05f; // Derivative gain
float previous_error1 = 0.0f;
float integral1 = 0.0f;
int dir1 = -1;

// PID Control Variables for Motor 2
float Kp2 = 10.0f;  // Proportional gain
float Ki2 = 0.0f;  // Integral gain
float Kd2 = 0.05f; // Derivative gain
float previous_error2 = 0.0f;
float integral2 = 0.0f;
int dir2 = 1;

uint32_t pwm_value1 = 0;
uint32_t pwm_value2 = 0;
float target_rpm1 = 150.0f; // Desired velocity in RPM
int current_rpm1=0;
float target_rpm2 = 150.0f; // Desired velocity in RPM
int current_rpm2=0;
int cpr = 3000;

uint32_t PID_CalculatePWM1(float current_rpm1)
{
    float error = target_rpm1 - current_rpm1;
    float corr = (fabsf(error) / Kp1) + 1.0f; // Calculate correction

    if (error > 0)
    {
        // Need to increase PWM
        pwm_value1 += (uint32_t)corr;
    }
    else if (error < 0)
    {
        // Need to decrease PWM
        if (pwm_value1 > (uint32_t)corr)
            pwm_value1 -= (uint32_t)corr;
        else
            pwm_value1 = 0;
    }
    // If error == 0, do nothing

    // Clamp PWM to valid range
    const uint32_t PWM_MAX = 1023;
    if (pwm_value1 > PWM_MAX)
        pwm_value1 = PWM_MAX;

    return pwm_value1;
}

uint32_t PID_CalculatePWM2(float current_rpm2)
{
    float error = target_rpm2 - current_rpm2;
    float corr = (fabsf(error) / Kp2) + 1.0f; // Calculate correction

    if (error > 0)
    {
        // Need to increase PWM
        pwm_value2 += (uint32_t)corr;
    }
    else if (error < 0)
    {
        // Need to decrease PWM
        if (pwm_value2 > (uint32_t)corr)
            pwm_value2 -= (uint32_t)corr;
        else
            pwm_value2 = 0;
    }
    // If error == 0, do nothing

    // Clamp PWM to valid range
    const uint32_t PWM_MAX = 1023;
    if (pwm_value2 > PWM_MAX)
        pwm_value2 = PWM_MAX;

    return pwm_value2;
}

int main(void)
{


    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;


    HAL_Init();
    SystemClock_Config();

    // Initialize all configured peripherals
    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM4_Init();
    MX_I2C1_Init();
    MX_I2C2_Init();
//    MX_I2C1_Init();

    Encoder_Init(&htim1, 3);
    Encoder_Init(&htim4, 3);

    MessageLen = sprintf((char*)Message, "msalamon.pl VL53L0X test\n\r");
//    HAL_UART_Transmit(&huart2, Message, MessageLen, 100);

    Dev->I2cHandle = &hi2c1;
    Dev->I2cDevAddr = 0x52;

//    HAL_GPIO_WritePin(TOF_XSHUT_GPIO_Port, TOF_XSHUT_Pin, GPIO_PIN_RESET); // Disable XSHUT
    HAL_Delay(20);
//    HAL_GPIO_WritePin(TOF_XSHUT_GPIO_Port, TOF_XSHUT_Pin, GPIO_PIN_SET); // Enable XSHUT
    HAL_Delay(20);


    VL53L0X_WaitDeviceBooted( Dev );
     VL53L0X_DataInit( Dev );
     VL53L0X_StaticInit( Dev );
     VL53L0X_PerformRefCalibration(Dev, &VhvSettings, &PhaseCal);
     VL53L0X_PerformRefSpadManagement(Dev, &refSpadCount, &isApertureSpads);
     VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);

     // Enable/Disable Sigma and Signal check
     VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
     VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
     VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1*65536));
     VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60*65536));
     VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Dev, 33000);
     VL53L0X_SetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
     VL53L0X_SetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
     /* USER CODE END 2 */

//    // Initialize MPU6886 (IMU)
    imu6886.i2cHandle = &hi2c2;
    init_status = MPU6886_Init(&imu6886);

    CalibrateGyro(&imu6886, &gyroBiasX, &gyroBiasY, &gyroBiasZ);
    prevTime = HAL_GetTick();

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

    	if (gyro_calib && !(gyro_calib=0))
    	    CalibrateGyro(&imu6886, &gyroBiasX, &gyroBiasY, &gyroBiasZ);
        // Read sensor data
        MPU6886_ReadAccelData(&imu6886, &sensorData[0], &sensorData[1], &sensorData[2]);
        MPU6886_ReadGyroData(&imu6886, &sensorData[3], &sensorData[4], &sensorData[5], gyroBiasX, gyroBiasY, gyroBiasZ);
        MPU6886_ReadTempData(&imu6886, &sensorData[6]);

        // Calculate time difference in seconds
        uint32_t currentTime = HAL_GetTick();
        float deltaTime = (currentTime - prevTime) / 1000.0f; // Convert ms to s
        prevTime = currentTime;

        // Integrate gyroscope Z-axis data to calculate yaw
        yaw += sensorData[5] * deltaTime; // sensorData[5] is corrected gyroZ
        UpdateYaw(&imu6886, gyroBiasX, gyroBiasY, gyroBiasZ, &roll, &pitch, &yaw);        // Normalize yaw to 0-360 degrees
        if (yaw >= 360.0f)
            yaw -= 360.0f;
        else if (yaw < 0.0f)
            yaw += 360.0f;

        if(velocity1<0)
        	old_vel1=velocity1;
        if (velocity1>0)
        	velocity1 = old_vel1;
        if(velocity2>0)
        	old_vel2=velocity2;
        if (velocity2<0)
        	velocity2 = old_vel2;
    	velocity1 = Encoder_GetVelocity(&htim1);
    	current_rpm1 = 60* old_vel1/cpr;

        // Get encoder velocity
        velocity2 = Encoder_GetVelocity(&htim4); // Assuming velocity is in counts per minute
    	current_rpm2 = 60* old_vel2/cpr;


    	// Corrected Code
    	uint32_t pwm_value1 = PID_CalculatePWM1(current_rpm1*dir1);
    	uint32_t pwm_value2 = PID_CalculatePWM2(current_rpm2*dir2);


        // Update duty cycle with the new PID-calculated value
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_value1);

        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_value2);

        // Optional: Add debugging information, toggle LED, etc.
        LED_Blink();

  	  VL53L0X_PerformSingleRangingMeasurement(Dev, &RangingData);

  	  if(RangingData.RangeStatus == 0)
  	  {
  		  MessageLen = sprintf((char*)Message, "Measured distance: %i\n\r", RangingData.RangeMilliMeter);
//  		  HAL_UART_Transmit(&huart2, Message, MessageLen, 100);
  	  }

        // Delay to sample periodically
        HAL_Delay(10); // Adjust delay as needed
    }
}

// Function to toggle LED on C13
void LED_Blink(void)
{
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // Toggle the LED state
    HAL_Delay(500); // Delay for 500 ms (adjust as necessary for blink rate)
}

// Function for updating yaw using a complementary filter with 6 DOF
void UpdateYaw(MPU6886_Handle *handle, float gyroBiasX, float gyroBiasY, float gyroBiasZ, float *roll, float *pitch, float *yaw) {
    float ax, ay, az;
    float gx, gy, gz;
    static float filteredYaw = 0.0;
    static float filteredRoll = 0.0;
    static float filteredPitch = 0.0;
    float dt = 0.01; // Loop time step in seconds (adjust to actual loop timing)

    // Read sensor data
    MPU6886_ReadAccelData(handle, &ax, &ay, &az);
    MPU6886_ReadGyroData(handle, &gx, &gy, &gz, gyroBiasX, gyroBiasY, gyroBiasZ);

    // Convert gyroscope data to degrees per second
    float gyroRollRate = gx;  // Roll rate from gyroscope (degrees/s)
    float gyroPitchRate = gy; // Pitch rate from gyroscope (degrees/s)
    float gyroYawRate = gz;   // Yaw rate from gyroscope (degrees/s)

    // Integrate gyroscope rates to get angles
    filteredRoll += gyroRollRate * dt;
    filteredPitch += gyroPitchRate * dt;
    filteredYaw += gyroYawRate * dt;

    // Calculate roll and pitch from the accelerometer
    float accelRoll = atan2f(ay, sqrtf(ax * ax + az * az)) * (180.0 / M_PI);
    float accelPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * (180.0 / M_PI);

    // Complementary filter to combine accelerometer and gyroscope data
    filteredRoll = (0.98 * filteredRoll) + (0.02 * accelRoll);
    filteredPitch = (0.98 * filteredPitch) + (0.02 * accelPitch);

    // Yaw correction (optional, as accelerometer is less effective for yaw)
    *yaw = filteredYaw;
    *roll = filteredRoll;
    *pitch = filteredPitch;

    // Print or store the results as needed
    // printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", *roll, *pitch, *yaw);
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

    // Configure GPIO pins PB3 and PB10 for I2C2 with pull-up
    GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

// Initialize I2C2
void MX_I2C2_Init(void)
{
    hi2c2.Instance = I2C2;
    hi2c2.Init.ClockSpeed = 100000;                       // Standard mode (100kHz)
    hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;               // Duty cycle for standard mode
    hi2c2.Init.OwnAddress1 = 0;                           // No specific own address
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;  // Use 7-bit addressing
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE; // Disable dual addressing mode
    hi2c2.Init.OwnAddress2 = 0;                           // No second address
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE; // Disable general call
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;     // Enable clock stretching

    if (HAL_I2C_Init(&hi2c2) != HAL_OK)
    {
        // If there is an initialization error, handle the error
        Error_Handler();
    }
}
void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};




  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB8     ------> I2C1_SCL
    PB9     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
  else if(i2cHandle->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspInit 0 */

  /* USER CODE END I2C2_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C2 GPIO Configuration
    PB10     ------> I2C2_SCL
    PB3     ------> I2C2_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C2_CLK_ENABLE();
  /* USER CODE BEGIN I2C2_MspInit 1 */

  /* USER CODE END I2C2_MspInit 1 */
  }

}



void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();
    __HAL_RCC_I2C2_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB8     ------> I2C1_SCL
    PB9     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3);


  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
}


// Function to calibrate gyroscope bias by averaging readings over 10 seconds
void CalibrateGyro(MPU6886_Handle *handle, float *gyroBiasX, float *gyroBiasY, float *gyroBiasZ) {
    int32_t gyroXSum = 0, gyroYSum = 0, gyroZSum = 0;
    uint32_t sampleCount = 0;
    uint32_t startTime = HAL_GetTick();
    uint32_t calibrationDuration = 30000; // 10 seconds in milliseconds

    // Keep collecting data for 10 seconds
    while ((HAL_GetTick() - startTime) < calibrationDuration) {
        float gx, gy, gz;
        MPU6886_ReadGyroData(&imu6886, &sensorData[3], &sensorData[4], &sensorData[5],0,0,0);

        // Sum up the raw gyro data for averaging
        gyroXSum += gx;
        gyroYSum += gy;
        gyroZSum += gz;
        sampleCount++;

        HAL_Delay(10); // Delay between samples to prevent overwhelming the I2C bus
    }

    // Calculate the average bias
    *gyroBiasX = gyroXSum / (float)sampleCount;
    *gyroBiasY = gyroYSum / (float)sampleCount;
    *gyroBiasZ = gyroZSum / (float)sampleCount;

    // Print or log calibration results if needed
    // printf("Gyro Bias - X: %.2f, Y: %.2f, Z: %.2f\n", *gyroBiasX, *gyroBiasY, *gyroBiasZ);
}



// Function to update gyroscope bias if stationary
void UpdateGyroBiasIfStationary(void)
{
    float ax, ay, az;
    MPU6886_ReadAccelData(&imu6886, &ax, &ay, &az);

    float accelMagnitude = sqrtf(ax * ax + ay * ay + az * az);

    // Check if acceleration magnitude is approximately 1g (stationary)
    if (fabsf(accelMagnitude - 1.0f) < 0.05f)
    {
        // Update gyroscope bias
        CalibrateGyro(&imu6886, &gyroBiasX, &gyroBiasY, &gyroBiasZ);
    }
}



// Error Handler
void Error_Handler(void)
{
    while (1)
    {
        // Implement error handling here
    }
}

