#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "MPU6886.h"
#include "i2c.h"
#include "encoder.h"
#include "vl53l0x_api.h"
/* ============================================
 *           1. Enumerations
 * ============================================ */
// Correction Modes Enumeration
typedef enum {
    rpm_corr,   // RPM Correction Mode
    angle_corr, // Angle Correction Mode
    seedhe,     // Straight Movement Mode
    posi_corr,  // Position Correction Mode
} CorrectionChoice;

// Global variable to hold the current correction choice
CorrectionChoice correction_choice;

/* ============================================
 *           2. Peripheral Handles
 * ============================================ */
// I2C Handles
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

// Timer Handles
TIM_HandleTypeDef htim1;  // Encoder on TIM1
TIM_HandleTypeDef htim2;  // PWM generation on TIM2
TIM_HandleTypeDef htim4;  // Encoder on TIM4
TIM_HandleTypeDef htim10; // Timer for Control Loop

// UART Handle
UART_HandleTypeDef huart2; // Assuming using USART2 now

/* ============================================
 *           3. Global Variables
 * ============================================ */
// PWM and Control Variables
float Kp = 1.0f;   // Proportional gain
float Ki = 0.0f;   // Integral gain
float Kd = 0.05f;  // Derivative gain

float target_yaw = 0.0f;       // Target yaw angle (e.g., to maintain a straight path)
uint32_t pwm_left = 0;         // Initial PWM for left wheel
uint32_t pwm_right = 0;        // Initial PWM for right wheel
float target_velocity = 1000.0f; // Desired velocity in encoder counts per minute
float previous_error = 0.0f;
float integral = 0.0f;

// Encoder Variables
volatile int velocity1 = 0;
volatile int velocity2 = 0;
int old_vel1 = 0, old_vel2 = 0;
int cpr = 3000; // Counts per revolution

// Control Loop Variables
long delay_counter = 0;
volatile int retard = 0;

// Miscellaneous Variables
uint8_t rx_buff2;
uint8_t Message[64];
uint8_t MessageLen;
uint16_t distance = 0; // Distance variable for VL53L1X sensor
volatile uint32_t prevTime = 0;
volatile int count = 0;

// PWM Parameters for Debugging
uint32_t pwm_frequency = 1000;  // 1 kHz default frequency
uint32_t duty_cycle = 69;       // 69% default duty cycle

// VL53L0X Variables
VL53L0X_RangingMeasurementData_t RangingData;
VL53L0X_Dev_t vl53l0x_c; // Center module
VL53L0X_DEV Dev = &vl53l0x_c;

// Message Array
float sensorData[7]; // Adjust size based on your usage

// Test Angle
int test_angle = 0;

/* ============================================
 *    4. Structures and Typedefs
 * ============================================ */
// PID Controller Struct
typedef struct {
    float Kp;               // Proportional gain
    float Ki;               // Integral gain
    float Kd;               // Derivative gain
    float integral;         // Integral sum
    float previous_error;   // Previous error value
} PIDController;

/* ============================================
 *    5. PID Controllers Initialization
 * ============================================ */
// Initialize PID Controllers for Motor 1, Motor 2, and Yaw Control
PIDController pid_motor1 = {
    .Kp = 10.0f,
    .Ki = 0.0f,
    .Kd = 0.05f,
    .integral = 0.0f,
    .previous_error = 0.0f
};

PIDController pid_motor2 = {
    .Kp = 10.0f,
    .Ki = 0.0f,
    .Kd = 0.05f,
    .integral = 0.0f,
    .previous_error = 0.0f
};

PIDController pid_yaw = {
    .Kp = 1.0f,
    .Ki = 0.01f,
    .Kd = 0.05f,
    .integral = 0.0f,
    .previous_error = 0.0f
};

/* ============================================
 *         6. Function Prototypes
 * ============================================ */
// System and Peripheral Initialization
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
// static void MX_I2C1_Init(void); // Currently commented out
static void MX_TIM2_Init(void); // TIM2 for PWM generation
static void MX_TIM1_Init(void); // TIM1 for Encoder
static void MX_TIM4_Init(void); // TIM4 for Encoder
static void MX_TIM10_Init(void); // TIM10 for Control Loop
void MX_I2C2_Init(void);

// LED Control
void LED_Blink(void);

// UART Functions
void arduimu_init(void);
void arduimu_poll(void);
void uart_init(UART_HandleTypeDef *huart, uint32_t baudrate, void (*isr_callback)(void));

// PID Functions
float PID_Compute(PIDController *pid, float setpoint, float measurement);
void rpm_to_pwm();

// Control Loop
void ControlLoop(void);

// Error Handling
void Error_Handler(void);

// I2C MSP Functions
void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle);
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle);

/* ============================================
 *    7. PID Control Variables for Motors
 * ============================================ */
// PID Control Variables for Motor 1
float Kp1 = 10.0f;  // Proportional gain
float Ki1 = 0.0f;   // Integral gain
float Kd1 = 0.05f;  // Derivative gain
float previous_error1 = 0.0f;
float integral1 = 0.0f;
int dir1 = -1;

// PID Control Variables for Motor 2
float Kp2 = 10.0f;  // Proportional gain
float Ki2 = 0.0f;   // Integral gain
float Kd2 = 0.05f;  // Derivative gain
float previous_error2 = 0.0f;
float integral2 = 0.0f;
int dir2 = 1;

// PWM Values
uint32_t pwm_value1 = 0;
uint32_t pwm_value2 = 0;

// RPM Targets and Current RPMs
float target_rpm1 = 150.0f; // Desired velocity in RPM
int current_rpm1 = 0;
float target_rpm2 = 150.0f; // Desired velocity in RPM
int current_rpm2 = 0;


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

void rpm_to_pwm()
{
	pwm_left = PID_Compute(&pid_motor1, target_rpm1, current_rpm1 * dir1);
	pwm_right = PID_Compute(&pid_motor2, target_rpm2, current_rpm2 * dir2);

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_left);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_right);
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
            	rpm_to_pwm();
            } break;

            case angle_corr: {
                float w_req = PID_Compute(&pid_yaw, target_yaw, raw_angle);

            } break;

            case seedhe: {

            } break;

            case posi_corr: {
                // Implement position correction as needed
            } break;
        }





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

