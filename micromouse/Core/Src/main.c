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
	start,
	straight1,
	turn1,
	straight1_2,
	turn1_2,
	straight1_4,
	turn1_4_2,
	turn1_4_4,
	straight1_6,
	turn1_6,
	straight1_8,
	turn1_8,
	straight2,
	turn2,
	straight3,
	turn3,
	finish,

	delayy,
	astart,
	astraight1,
	aturn1,
	astraight2,
	aturn2,
	astraight3,
	aturn3,
	afinish,
} CorrectionChoice;

// Global variable to hold the current correction choice
CorrectionChoice navigation;


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
 *           3. Global Constants and Macros
 * ============================================ */
#define WHEEL_RADIUS 0.01315    // radius in meters
#define WHEEL_DISTANCE 0.084    // distance in meters
#define RPM_TO_RPS 0.10472      // conversion factor from RPM to radians per second (2 * PI / 60)

#define ONE_SQUARE 0.070
#define HALF_SQUARE 0.065
/* ============================================
 *           4. Motion Control Variables
 * ============================================ */

// Velocity and Position Control Variables
volatile float req_vel_x = 0.0;
volatile float req_vel_w = 0.0;
volatile float current_vel_x = 0.0;
volatile float current_vel_w = 0.0;
volatile float current_vel_left = 0.0;
volatile float current_vel_right = 0.0;
volatile float delta_time = 0.0f;

volatile float pos_x_ros = 0;
volatile float pos_y_ros = 0;
volatile float pos_x_embed = 0;
volatile float pos_y_embed = 0;
volatile float target_pos_x = 0;
volatile float target_pos_y = 0;
volatile float target_yaw = 0.0f;


/* ============================================
 *           5. Encoder and RPM Variables
 * ============================================ */
// Encoder Variables
volatile int encoder_velocity_left = 0;
volatile int encoder_velocity_right = 0;
int cpr = 3666;

// RPM Targets and Current RPMs
volatile float target_rpm_left = 0.0f; // Desired velocity in RPM
volatile float current_rpm_left = 0.0f;
volatile float target_rpm_right = 0.0f; // Desired velocity in RPM
volatile float current_rpm_right = 0.0f;
volatile float pwm_left = 0.0f;
volatile float pwm_right = 0.0f;

// Directional Control
int dir1 = -1;
int dir2 = 1;


/* ============================================
 *           6. Sensor Variables
 * ============================================ */
// Distance Sensors
volatile int front_sensor = 0.0f;
volatile int left_sensor = 0.0f;
volatile int target_left_sensor = 0.0f;

// VL53L0X Variables
VL53L0X_RangingMeasurementData_t RangingData_Center, RangingData_Side;
VL53L0X_Dev_t vl53l0x_c; // Center module
VL53L0X_Dev_t vl53l0x_s; // Side module (second sensor)

// Define pointers to each device instance
VL53L0X_DEV Dev_Center = &vl53l0x_c;
VL53L0X_DEV Dev_Side = &vl53l0x_s;

VL53L0X_Dev_t dev1, dev2;
VL53L0X_RangingMeasurementData_t RangingData1, RangingData2;


/* ============================================
 *           7. Communication and Miscellaneous
 * ============================================ */
// UART Communication Variables
uint8_t rx_buff2;
uint8_t Message[64];
uint8_t MessageLen;
float sensorData[7]; // Adjust size based on your usage

// PWM Parameters for Debugging
uint32_t pwm_frequency = 1000;  // 1 kHz default frequency
uint32_t duty_cycle = 69;       // 69% default duty cycle

// Control Loop Variables
long delay_counter = 0;
volatile int retard = 0;
volatile uint32_t prevTime = 0;
volatile int count = 0;


/* ============================================
 *           8. Control Flags
 * ============================================ */
volatile int theta_correction = 1;
volatile int wall_follow = 0;
volatile int translate = 0;
volatile int yaw_first = 1;

/* ============================================
 *    9. Structures and Typedefs
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
 *    10. PID Controllers Initialization
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
    .Kp = 0.17f,
    .Ki = 0.0f,
    .Kd = 0.005f,
    .integral = 0.0f,
    .previous_error = 0.0f
};



float total_distance_traveled = 0.0;
float ref_total_distance_traveled = 0.0;

/* ============================================
 *         11. Function Prototypes
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
float get_delta_time();
void vel_gen();
void vel_to_rpm();
void rpm_to_pwm();
void update_pos_position(float raw_angle, float delta_time);
void wait_for_yaw();

// Control Loop
void ControlLoop(void);

// Error Handling
void Error_Handler(void);

// I2C MSP Functions
void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle);
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle);






void fix_angle()
{
    while (raw_angle >= 360.0f) {
    	raw_angle -= 360.0f;
    }
    while (raw_angle < 0.0f) {
    	raw_angle += 360.0f;
    }
}


void turn_left()
{
	target_yaw -=90;
}
void turn_right()
{
	target_yaw +=90;
}
void face_north()
{
	target_yaw =0;
}
void face_south()
{
	target_yaw =180;
}
void face_west()
{
	target_yaw =90;
}
void face_east()
{
	target_yaw =270;
}
void move_one()
{
	req_vel_x =0.135;
}
void stop()
{
	req_vel_x =0;
}

bool is_within_angle_threshold(float angle, float target, float threshold) {
    float diff = fmod(fabs(angle - target), 360.0);
    if (diff > 180.0) {
        diff = 360.0 - diff;  // Use the shortest angular distance
    }
    return diff <= threshold;
}

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

void vel_gen()
{
    static bool prev_theta_correction = false;
    static bool prev_wall_follow = false;

    // ANGLE CORRECTION
    if (theta_correction) {
        float error = target_yaw - raw_angle;
        if (error > 180) error -= 360;
        else if (error < -180) error += 360;
        req_vel_w = PID_Compute(&pid_yaw, 0.0, error);
    }
    else if (prev_theta_correction)
        req_vel_w = 0.0;

    //WALL FOLLOWING
    if (wall_follow)
    {
		float error = (target_left_sensor - left_sensor) * -1;
		req_vel_w = PID_Compute(&pid_yaw, 0.0f, error/10);
    }
    else if (prev_wall_follow)
        req_vel_w = 0.0;

    prev_theta_correction = theta_correction;
    prev_wall_follow = wall_follow;
}


void vel_to_rpm()
{
	double v_left=0;
	double v_right=0;
	if (translate)
	{
		v_left = req_vel_x + (req_vel_w * WHEEL_DISTANCE / 2.0);
		v_right = req_vel_x - (req_vel_w * WHEEL_DISTANCE / 2.0);
	}
	else
	{
		v_left = (req_vel_w * WHEEL_DISTANCE / 2.0);
		v_right = -1* (req_vel_w * WHEEL_DISTANCE / 2.0);
	}
	target_rpm_left = (v_left / (2 * 3.141592653589793 * WHEEL_RADIUS)) * 60;
	target_rpm_right = (v_right / (2 * 3.141592653589793 * WHEEL_RADIUS)) * 60;

}


void rpm_to_pwm()
{
	pwm_left = PID_Compute(&pid_motor1, target_rpm_left, current_rpm_left * dir1);
	pwm_right = PID_Compute(&pid_motor2, target_rpm_right, current_rpm_right * dir2);

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_left);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_right);
}

// Function to update pos position using current linear and angular velocities
void update_pos_position(float raw_angle, float delta_time) {
    // Convert raw_angle to radians
    float angle_rad = raw_angle * (M_PI / 180.0);
    current_vel_left = (current_rpm_left * RPM_TO_RPS) * WHEEL_RADIUS;
    current_vel_right = (current_rpm_right * RPM_TO_RPS) * WHEEL_RADIUS;

    // Calculate the forward velocity as the average of left and right velocities
    current_vel_x = ((current_vel_left*dir1) + (current_vel_right*dir2)) / 2.0;

    total_distance_traveled += fabs(current_vel_x * delta_time);
    // Update pos position using current_vel_x and current_vel_w
    pos_x_ros += current_vel_x * cos(angle_rad) * delta_time;
    pos_y_ros += current_vel_x * sin(angle_rad) * delta_time;

    pos_x_embed = -pos_y_ros;
    pos_y_embed = pos_x_ros;

    // Update orientation based on angular velocity (optional if tracking orientation)
    raw_angle += current_vel_w * (180.0 / M_PI) * delta_time;  // Convert rad/s to deg/s for raw_angle update
    if (raw_angle >= 360.0) raw_angle -= 360.0;
    else if (raw_angle < 0.0) raw_angle += 360.0;
}


void wait_for_yaw() {
        float angle_diff = fabs(target_yaw - raw_angle);

        if (angle_diff > 180) {
            angle_diff = 360 - angle_diff;
        }

        if (angle_diff<=5)
        {
        	translate = 1;
        }
        else
        {
        	translate = 0;
        }
}

float get_delta_time() {
    static uint32_t last_time = 0;
    uint32_t current_time = HAL_GetTick();  // Current time in milliseconds
    float delta_time = (current_time - last_time) / 1000.0f;  // Convert ms to seconds
    last_time = current_time;
    return delta_time;
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





void ControlLoop()
{
	retard++;
}



int main(void) {
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
    MX_I2C1_Init();
    MX_I2C2_Init();

    Encoder_Init(&htim1, 3);
    Encoder_Init(&htim4, 3);
    arduimu_init();

    // Initialize first VL53L0X sensor on I2C1
    dev1.I2cHandle = &hi2c1;
    dev1.I2cDevAddr = 0x52;  // Default address; change if needed

    VL53L0X_WaitDeviceBooted(&dev1);
    VL53L0X_DataInit(&dev1);
    VL53L0X_StaticInit(&dev1);
    VL53L0X_PerformRefCalibration(&dev1, &VhvSettings, &PhaseCal);
    VL53L0X_PerformRefSpadManagement(&dev1, &refSpadCount, &isApertureSpads);
    VL53L0X_SetDeviceMode(&dev1, VL53L0X_DEVICEMODE_SINGLE_RANGING);

    VL53L0X_SetLimitCheckEnable(&dev1, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    VL53L0X_SetLimitCheckEnable(&dev1, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    VL53L0X_SetLimitCheckValue(&dev1, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1 * 65536));
    VL53L0X_SetLimitCheckValue(&dev1, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60 * 65536));
    VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&dev1, 33000);
    VL53L0X_SetVcselPulsePeriod(&dev1, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
    VL53L0X_SetVcselPulsePeriod(&dev1, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);

    // Initialize second VL53L0X sensor on I2C2
    dev2.I2cHandle = &hi2c2;
    dev2.I2cDevAddr = 0x52;  // Default address; change if needed

    VL53L0X_WaitDeviceBooted(&dev2);
    VL53L0X_DataInit(&dev2);
    VL53L0X_StaticInit(&dev2);
    VL53L0X_PerformRefCalibration(&dev2, &VhvSettings, &PhaseCal);
    VL53L0X_PerformRefSpadManagement(&dev2, &refSpadCount, &isApertureSpads);
    VL53L0X_SetDeviceMode(&dev2, VL53L0X_DEVICEMODE_SINGLE_RANGING);

    VL53L0X_SetLimitCheckEnable(&dev2, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    VL53L0X_SetLimitCheckEnable(&dev2, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    VL53L0X_SetLimitCheckValue(&dev2, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1 * 65536));
    VL53L0X_SetLimitCheckValue(&dev2, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60 * 65536));
    VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&dev2, 33000);
    VL53L0X_SetVcselPulsePeriod(&dev2, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
    VL53L0X_SetVcselPulsePeriod(&dev2, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);

    // Start PWM on TIM2, Channel 1 and Channel 2
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK) Error_Handler();

    stop();
    vel_gen();
    vel_to_rpm();
    rpm_to_pwm();

    HAL_Delay(10000);

    while (1) {
        encoder_velocity_left = Encoder_GetVelocity_TIM1(&htim1);
        encoder_velocity_right = Encoder_GetVelocity_TIM4(&htim4);

        current_rpm_left = 60 * encoder_velocity_left / cpr;
        current_rpm_right = 60 * encoder_velocity_right / cpr;

        switch (navigation) {

            case start:
            	move_one();
        		ref_total_distance_traveled = total_distance_traveled;
        		target_left_sensor = 44;
        		wall_follow = 1;

            	navigation++;
            break;
            case straight1:
//            	if ((0.95 *3*ONE_SQUARE)< (fabs(total_distance_traveled)-fabs(ref_total_distance_traveled)))
				if (front_sensor <210)
            	{
            		stop();
            		wall_follow = 0;
            		face_west();
                	navigation++;
            	}
            break;
            case turn1:
            	if (is_within_angle_threshold(raw_angle, target_yaw, 3))
				{
            		move_one();
            		target_left_sensor = 44;
            		wall_follow = 1;
            		ref_total_distance_traveled = total_distance_traveled;
            		navigation++;
				}
            break;
            case straight1_2:
            	if (front_sensor < 236)
				{
            		stop();
            		face_north();
                	navigation = straight1_4;
				}
            break;
            case turn1_2:
            	if (is_within_angle_threshold(raw_angle, target_yaw, 5))
				{
            		move_one();
            		ref_total_distance_traveled = total_distance_traveled;
            		navigation++;
				}
            break;
            case straight1_4:
				if (is_within_angle_threshold(raw_angle, target_yaw, 5))

//            	if ((0.1*1*ONE_SQUARE)< (fabs(total_distance_traveled)-fabs(ref_total_distance_traveled)))
				{
            		stop();
            		turn_left();
                	navigation++;
				}
            break;
            case turn1_4_2:
            	if (is_within_angle_threshold(raw_angle, target_yaw, 5))
				{
            		stop();
            		turn_left();
                	navigation = straight1_6;
				}
            break;
            case turn1_4_4:
            	if (is_within_angle_threshold(raw_angle, target_yaw, 5))
				{
            		move_one();
            		ref_total_distance_traveled = total_distance_traveled;
            		navigation++;
				}
            break;
            case straight1_6:
            	if (is_within_angle_threshold(raw_angle, target_yaw, 5))

//            	if ((0.2*1*ONE_SQUARE)< (fabs(total_distance_traveled)-fabs(ref_total_distance_traveled)))
				{
            		stop();
            		face_west();
                	navigation++;
				}
            break;
            case turn1_6:
            	if (is_within_angle_threshold(raw_angle, target_yaw, 5))
				{
            		move_one();
            		ref_total_distance_traveled = total_distance_traveled;
            		navigation = straight2;
				}
            break;
            case straight1_8:
            	if ((0.25*1*ONE_SQUARE)< (fabs(total_distance_traveled)-fabs(ref_total_distance_traveled)))
				{
            		stop();
            		face_east();
                	navigation++;
				}
            break;
            case turn1_8:
            	if (is_within_angle_threshold(raw_angle, target_yaw, 5))
				{
            		move_one();
            		ref_total_distance_traveled = total_distance_traveled;
            		navigation++;
				}
            break;
            case straight2:
            	if ((1.2*1*ONE_SQUARE)< (fabs(total_distance_traveled)-fabs(ref_total_distance_traveled)))
				{
            		stop();
            		face_south();
                	navigation++;
				}
            break;
            case turn2:
            	if (is_within_angle_threshold(raw_angle, target_yaw, 5))
				{
            		move_one();
            		ref_total_distance_traveled = total_distance_traveled;
            		navigation++;
				}
            break;
            case straight3:
            	if (3*ONE_SQUARE< (fabs(total_distance_traveled)-fabs(ref_total_distance_traveled)))
				{
            		stop();
            		face_east();
            		navigation++;
				}
            break;
            case turn3:
            	if (is_within_angle_threshold(raw_angle, target_yaw, 5))
				{
            		move_one();
            		ref_total_distance_traveled = total_distance_traveled;
            		navigation++;
				}
            break;
            case finish:
            	if (2*ONE_SQUARE< (fabs(total_distance_traveled)-fabs(ref_total_distance_traveled)))
				{
            		stop();
            		navigation++;
				}
            break;


//            case delayy:
//            {
//                HAL_Delay(10000);
//            	navigation++;
//            }
//            break;
//            case astart:
//            	move_one();
//            	total_distance_traveled = 0;
//            	ref_total_distance_traveled = 0;
//        		ref_total_distance_traveled = total_distance_traveled;
//            	navigation++;
//            break;
//            case astraight1:
//            	if ((1.05*3*ONE_SQUARE)< (fabs(total_distance_traveled)-fabs(ref_total_distance_traveled)))
//            	{
//            		stop();
//            		face_west();
//                	navigation++;
//            	}
//            break;
//            case aturn1:
//            	if (is_within_angle_threshold(raw_angle, target_yaw, 5))
//				{
//            		move_one();
//            		ref_total_distance_traveled = total_distance_traveled;
//            		navigation++;
//				}
//            break;
//            case astraight2:
//            	if ((0.8*3*ONE_SQUARE)< (fabs(total_distance_traveled)-fabs(ref_total_distance_traveled)))
//				{
//            		stop();
//            		face_south();
//                	navigation++;
//				}
//            break;
//            case aturn2:
//            	if (is_within_angle_threshold(raw_angle, target_yaw, 5))
//				{
//            		move_one();
//            		ref_total_distance_traveled = total_distance_traveled;
//            		navigation++;
//				}
//            break;
//            case astraight3:
//            	if (3*ONE_SQUARE< (fabs(total_distance_traveled)-fabs(ref_total_distance_traveled)))
//				{
//            		stop();
//            		face_east();
//            		navigation++;
//				}
//            break;
//            case aturn3:
//            	if (is_within_angle_threshold(raw_angle, target_yaw, 5))
//				{
//            		move_one();
//            		ref_total_distance_traveled = total_distance_traveled;
//            		navigation++;
//				}
//            break;
//            case afinish:
//            	if (2*ONE_SQUARE< (fabs(total_distance_traveled)-fabs(ref_total_distance_traveled)))
//				{
//            		stop();
//				}
//            break;
        }

        delta_time = get_delta_time();
        update_pos_position(raw_angle, delta_time);

        if (yaw_first)
        	wait_for_yaw();
        fix_angle();
        vel_gen();
        vel_to_rpm();
        rpm_to_pwm();

        if (delay_counter % 500 == 0) LED_Blink();

        if (delay_counter % 10 == 0) {
            VL53L0X_PerformSingleRangingMeasurement(&dev1, &RangingData1);
            left_sensor = RangingData1.RangeMilliMeter;
            VL53L0X_PerformSingleRangingMeasurement(&dev2, &RangingData2);
            front_sensor = RangingData2.RangeMilliMeter;
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

	if(i2cHandle->Instance == I2C1)
	{
		/* I2C1 GPIO Configuration */
		__HAL_RCC_GPIOB_CLK_ENABLE();
		/**I2C1 GPIO Configuration
		PB8     ------> I2C1_SCL
		PB9     ------> I2C1_SDA
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* I2C1 clock enable */
		__HAL_RCC_I2C1_CLK_ENABLE();
	}

	if(i2cHandle->Instance == I2C2)
	{
		/* I2C2 GPIO Configuration */
		__HAL_RCC_GPIOB_CLK_ENABLE();
		/**I2C2 GPIO Configuration
		PB10     ------> I2C2_SCL
		PB3      ------> I2C2_SDA
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_3;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

		// Set alternate function for each pin individually
		GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;  // I2C2 SCL on PB10
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_3;
		GPIO_InitStruct.Alternate = GPIO_AF9_I2C2;  // I2C2 SDA on PB3
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* I2C2 clock enable */
		__HAL_RCC_I2C2_CLK_ENABLE();
	}
}


void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

	  if(i2cHandle->Instance==I2C1)
	  {

	    /* Peripheral clock disable */
	    __HAL_RCC_I2C1_CLK_DISABLE();

	    /**I2C1 GPIO Configuration
	    PB8     ------> I2C1_SCL
	    PB9     ------> I2C1_SDA
	    */
	    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

	  }
	  if(i2cHandle->Instance==I2C2)
	  {

	    /* Peripheral clock disable */
	    __HAL_RCC_I2C2_CLK_DISABLE();

	    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_3);

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
