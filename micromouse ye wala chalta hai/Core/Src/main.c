#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "MPU6886.h"
#include "i2c.h"
#include "encoder.h"
#include "vl53l0x_api.h"

enum
{
	rpm_corr,
	angle_corr,
	seedhe,
	posi_corr,

}correction_choice;

I2C_HandleTypeDef hi2c2;

int test_angle=0;
// Define handles for I2C and timers
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2; // For PWM generation on TIM2
TIM_HandleTypeDef htim4; // For encoder on TIM4
UART_HandleTypeDef huart2; // Assuming using USART2 now
TIM_HandleTypeDef htim10;
uint8_t rx_buff2;

uint8_t Message[64];
uint8_t MessageLen;

VL53L0X_RangingMeasurementData_t RangingData;
VL53L0X_Dev_t  vl53l0x_c; // center module
VL53L0X_DEV    Dev = &vl53l0x_c;


long delay_counter = 0;
volatile int retard  = 0;


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

float target_yaw = 0.0f; // Target yaw angle (e.g., to maintain a straight path)
uint32_t pwm_left = 0; // Initial PWM for left wheel
uint32_t pwm_right = 0; // Initial PWM for right wheel
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
static void MX_TIM2_Init(void); // TIM2 for PWM generation
static void MX_TIM1_Init(void); // TIM2 for PWM generation
static void MX_TIM4_Init(void); // TIM4 for encoder
void LED_Blink(void);
void MX_I2C2_Init(void);

uint32_t PID_CalculateStraightWithYawCorrection(float current_rpm1, float current_rpm2, float target_rpm1, float target_rpm2, float current_yaw, float target_yaw, uint32_t *pwm_left, uint32_t *pwm_right);
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

// PID Controller Struct
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float previous_error;
} PIDController;

// General PID Compute Function
float PID_Compute(PIDController *pid, float setpoint, float measurement) {
    float error = setpoint - measurement;
    pid->integral += error; // Accumulate the integral
    float derivative = error - pid->previous_error; // Calculate the derivative
    pid->previous_error = error;

    // Compute the PID output
    float output = (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);
    return output;
}

// Initialize PID Controllers
PIDController pid_motor1 = { .Kp = 10.0f, .Ki = 0.0f, .Kd = 0.05f, .integral = 0.0f, .previous_error = 0.0f };
PIDController pid_motor2 = { .Kp = 10.0f, .Ki = 0.0f, .Kd = 0.05f, .integral = 0.0f, .previous_error = 0.0f };
PIDController pid_yaw = { .Kp = 1.0f, .Ki = 0.01f, .Kd = 0.05f, .integral = 0.0f, .previous_error = 0.0f };



uint32_t PID_CalculatePWM1(float current_rpm1)
{
    float error = target_rpm1 - current_rpm1;
    integral1 += error; // Accumulate integral
    float derivative = error - previous_error1; // Calculate derivative
    previous_error1 = error;

    float corr = (Kp1 * error) + (Ki1 * integral1) + (Kd1 * derivative); // Full PID correction

    if (corr > 0)
    {
        pwm_value1 += (uint32_t)corr;
    }
    else if (corr < 0)
    {
        if (pwm_value1 > (uint32_t)fabsf(corr))
            pwm_value1 -= (uint32_t)fabsf(corr);
        else
            pwm_value1 = 0;
    }

    // Clamp PWM to valid range
    const uint32_t PWM_MAX = 1023;
    if (pwm_value1 > PWM_MAX)
        pwm_value1 = PWM_MAX;

    return pwm_value1;
}

uint32_t PID_CalculatePWM2(float current_rpm2)
{
    float error = target_rpm2 - current_rpm2;
    integral2 += error; // Accumulate integral
    float derivative = error - previous_error2; // Calculate derivative
    previous_error2 = error;

    float corr = (Kp2 * error) + (Ki2 * integral2) + (Kd2 * derivative); // Full PID correction

    if (corr > 0)
    {
        pwm_value2 += (uint32_t)corr;
    }
    else if (corr < 0)
    {
        if (pwm_value2 > (uint32_t)fabsf(corr))
            pwm_value2 -= (uint32_t)fabsf(corr);
        else
            pwm_value2 = 0;
    }

    // Clamp PWM to valid range
    const uint32_t PWM_MAX = 1023;
    if (pwm_value2 > PWM_MAX)
        pwm_value2 = PWM_MAX;

    return pwm_value2;
}

uint32_t PID_CalculateYawPWM(float current_yaw, float target_yaw, uint32_t *pwm_left, uint32_t *pwm_right)
{
    float yaw_error = target_yaw - current_yaw; // Calculate yaw error
    static float yaw_integral = 0.0f;
    static float previous_yaw_error = 0.0f;
    float yaw_Kp = 1.0f; // Proportional gain for yaw control, adjust as needed
    float yaw_Kd = 0.05f; // Derivative gain for yaw control
    float yaw_Ki = 0.01f; // Integral gain for yaw control

    yaw_integral += yaw_error; // Accumulate integral
    float yaw_derivative = yaw_error - previous_yaw_error; // Calculate derivative
    previous_yaw_error = yaw_error;

    // Calculate correction using PID
    float correction = (yaw_Kp * yaw_error) + (yaw_Ki * yaw_integral) + (yaw_Kd * yaw_derivative);

    if (yaw_error > 0)
    {
        *pwm_left += (uint32_t)fabsf(correction);
        if (*pwm_right > (uint32_t)fabsf(correction))
            *pwm_right -= (uint32_t)fabsf(correction);
        else
            *pwm_right = 0;
    }
    else if (yaw_error < 0)
    {
        *pwm_right += (uint32_t)fabsf(correction);
        if (*pwm_left > (uint32_t)fabsf(correction))
            *pwm_left -= (uint32_t)fabsf(correction);
        else
            *pwm_left = 0;
    }

    // Clamp PWM to valid range
    const uint32_t PWM_MAX = 1023;
    if (*pwm_left > PWM_MAX) *pwm_left = PWM_MAX;
    if (*pwm_right > PWM_MAX) *pwm_right = PWM_MAX;

    return 0; // Return value can be modified to indicate status if needed
}

uint32_t PID_CalculateStraightWithYawCorrection(
    float current_rpm1, float current_rpm2,
    float target_rpm1, float target_rpm2,
    float current_yaw, float target_yaw,
    uint32_t *pwm_left, uint32_t *pwm_right)
{
    // Static variables to maintain state between calls
    static float integral1 = 0.0f, integral2 = 0.0f;
    static float previous_error1 = 0.0f, previous_error2 = 0.0f;
    static float yaw_integral = 0.0f, previous_yaw_error = 0.0f;

    // PID constants for speed control (set appropriate values)
    float Kp1 = 1.0, Ki1 = 0, Kd1 = 0;
    float Kp2 = 1.0, Ki2 = 0, Kd2 = 0;

    // PID constants for yaw control
    float yaw_Kp = 1.0, yaw_Ki = 0, yaw_Kd = 0;

    // Speed errors
    float speed_error1 = target_rpm1 - current_rpm1;
    float speed_error2 = target_rpm2 - current_rpm2;

    // Integrate errors
    integral1 += speed_error1;
    integral2 += speed_error2;

    // Derivative of errors
    float derivative1 = speed_error1 - previous_error1;
    float derivative2 = speed_error2 - previous_error2;

    // Update previous errors
    previous_error1 = speed_error1;
    previous_error2 = speed_error2;

    // PID calculations for speed
    float corr1 = (Kp1 * speed_error1) + (Ki1 * integral1) + (Kd1 * derivative1);
    float corr2 = (Kp2 * speed_error2) + (Ki2 * integral2) + (Kd2 * derivative2);

    // Yaw error with wrap-around handling
    float yaw_error = target_yaw - current_yaw;

    // Adjust yaw_error to be within -180 to +180 degrees
    if (yaw_error > 180.0f) {
        yaw_error -= 360.0f;
    } else if (yaw_error < -180.0f) {
        yaw_error += 360.0f;
    }

    // Integrate yaw error
    yaw_integral += yaw_error;

    // Derivative of yaw error
    float yaw_derivative = yaw_error - previous_yaw_error;
    previous_yaw_error = yaw_error;

    // PID calculation for yaw
    float yaw_correction = (yaw_Kp * yaw_error) + (yaw_Ki * yaw_integral) + (yaw_Kd * yaw_derivative);

    // Total corrections
    float total_corr_left = corr1 + yaw_correction;
    float total_corr_right = corr2 - yaw_correction;

//     Clamp corrections to prevent excessive adjustments
//    const float CORR_MAX = /* your value */; // Set maximum correction value
//    if (total_corr_left > CORR_MAX) total_corr_left = CORR_MAX;
//    if (total_corr_left < -CORR_MAX) total_corr_left = -CORR_MAX;
//    if (total_corr_right > CORR_MAX) total_corr_right = CORR_MAX;
//    if (total_corr_right < -CORR_MAX) total_corr_right = -CORR_MAX;

    // Update PWM values with clamped corrections
    // Ensure PWM values do not go negative
    float new_pwm_left = (float)(*pwm_left) + total_corr_left;
    float new_pwm_right = (float)(*pwm_right) + total_corr_right;

    // Clamp PWM values to valid range
    const uint32_t PWM_MAX = 1023;
    if (new_pwm_left > PWM_MAX) new_pwm_left = PWM_MAX;
    if (new_pwm_left < 0.0f) new_pwm_left = 0.0f;
    if (new_pwm_right > PWM_MAX) new_pwm_right = PWM_MAX;
    if (new_pwm_right < 0.0f) new_pwm_right = 0.0f;

    *pwm_left = (uint32_t)new_pwm_left;
    *pwm_right = (uint32_t)new_pwm_right;

    return 0;
}



// Function to initialize UART2 and enable interrupt
void uart_init(UART_HandleTypeDef *huart, uint32_t baudrate, void (*isr_callback)(void)) {
    huart->Instance = USART2;  // Use USART2 instead of USART4
    huart->Init.BaudRate = baudrate;
    huart->Init.WordLength = UART_WORDLENGTH_8B;
    huart->Init.StopBits = UART_STOPBITS_1;
    huart->Init.Parity = UART_PARITY_NONE;
    huart->Init.Mode = UART_MODE_RX;
    huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart->Init.OverSampling = UART_OVERSAMPLING_16;

    HAL_UART_Init(huart);

    // Enable UART interrupt
    HAL_NVIC_EnableIRQ(USART2_IRQn); // Use USART2 IRQ instead of USART4 IRQ
}

void empty(void){}

// Polling function for UART data (if not using interrupts)
void arduimu_poll(void) {
    // Receive 1 byte from UART2
    HAL_UART_Receive(&huart2, &rx_buff2, 1, 5);  // Timeout 5 ms

    uint8_t temp_data = rx_buff2 & 0xFF;  // Extract the byte

    if (temp_data & 0x80) {
        // First byte, shift the value
        temp_angle = (temp_data & 0x7F) << 7;
        next_byte = 1;  // Expect the next byte
    } else if (next_byte) {
        // Second byte, combine with the previous byte
        combined_angle = temp_angle | temp_data;
        next_byte = 0;
        raw_angle = 360.0 - (combined_angle / 10.0);  // Calculate the angle
    } else {
        next_byte = 0;  // Reset in case of an invalid byte
    }
}

// Initialization function to setup the IMU
void arduimu_init(void) {
    // Initialize variables
    temp_angle = 0;
    next_byte = 0;
    combined_angle = 0;

    // Initialize UART2 and the ISR callback function
    uart_init(&huart2, 38400, empty);
}

// USART2 IRQ handler
void USART2_IRQHandler(void) {
    // Call the ISR function to handle UART data
    empty();
}


void ControlLoop()
{
	retard++;
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM10) // Check if the interrupt is from TIM10
    {
        // Call your function here
        ControlLoop();
    }
}

static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 127;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 999;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */
  HAL_TIM_MspPostInit(&htim10);

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
    MX_TIM10_Init();
    HAL_TIM_OC_Start_IT(&htim10, TIM_CHANNEL_1);


//    MX_I2C1_Init();
//    MX_I2C2_Init();
    HAL_Init();
//    MX_I2C1_Init();

    Encoder_Init(&htim1, 3);
    Encoder_Init(&htim4, 3);

    arduimu_init();



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
        if(velocity1<0)
        	old_vel1=velocity1;
        if (velocity1>0)
        	velocity1 = old_vel1;
        if(velocity2>0)
        	old_vel2=velocity2;
        if (velocity2<0)
        	velocity2 = old_vel2;
    	velocity1 = Encoder_GetVelocity(&htim1);
        velocity2 = Encoder_GetVelocity(&htim4); // Assuming velocity is in counts per minute

    	current_rpm1 = 60* old_vel1/cpr;
    	current_rpm2 = 60* old_vel2/cpr;

        switch (correction_choice) {
            case rpm_corr: {
            	pwm_left = PID_Compute(&pid_motor1, target_rpm1, current_rpm1 * dir1);
            	pwm_right = PID_Compute(&pid_motor2, target_rpm2, current_rpm2 * dir2);

                // Update PWM values with clamping
//                UpdatePWM(&pwm_value1, correction1);
//                UpdatePWM(&pwm_value2, correction2);
            } break;

            case angle_corr: {
                float w_req = PID_Compute(&pid_yaw, target_yaw, raw_angle);

                // Apply yaw correction to PWM values
//                ApplyYawCorrection(&pwm_left, &pwm_right, yaw_correction);
            } break;

            case seedhe: {
                PID_CalculateStraightWithYawCorrection(
                    current_rpm1, current_rpm2,
                    target_rpm1, target_rpm2,
                    test_angle, target_yaw,
                    &pwm_left, &pwm_right);
            } break;

            case posi_corr: {
                // Implement position correction as needed
            } break;
        }
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_left);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_right);


	    if(delay_counter%500 == 0)  // Assuming 10 iterations for your required delay
	    {
	        LED_Blink();
	    }
	    if(delay_counter%10 == 0)  // Assuming 10 iterations for your required delay
	    {
	        VL53L0X_PerformSingleRangingMeasurement(Dev, &RangingData);
	    }
	    delay_counter++;
	    arduimu_poll();

    }
}

// Function to toggle LED on C13
void LED_Blink(void)
{
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // Toggle the LED state
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

// Error Handler
void Error_Handler(void)
{
    while (1)
    {
        // Implement error handling here
    }
}

