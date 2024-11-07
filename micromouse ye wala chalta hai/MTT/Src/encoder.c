/*
 * encoder.c
 *
 *  Created on: Nov 5, 2024
 *      Author: hrish
 */
#include "encoder.h"

static uint16_t encoder_ppr = 3000;
float old_value_enc = 0;

/**
 * @brief Initialize the specified timer for encoder mode with specified PPR.
 * @param htim: Timer handle pointer
 * @param ppr: Pulses per revolution for the encoder
 */
void Encoder_Init(TIM_HandleTypeDef *htim, uint16_t ppr)
{
    encoder_ppr = ppr;

    // Enable clock for the specific timer instance
    if (htim->Instance == TIM1) {
        __HAL_RCC_TIM1_CLK_ENABLE();
    } else if (htim->Instance == TIM4) {
        __HAL_RCC_TIM4_CLK_ENABLE();
    }

    htim->Init.Prescaler = 0;
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim->Init.Period = 0xFFFF;
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim->Init.RepetitionCounter = 0;

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

    HAL_TIM_Encoder_Init(htim, &encoderConfig);
    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
}

/**
 * @brief Get the current raw encoder count for a specified timer.
 * @param htim: Timer handle pointer for the encoder
 * @retval Encoder count as an integer
 */
int Encoder_GetCount(TIM_HandleTypeDef *htim)
{
    return __HAL_TIM_GET_COUNTER(htim);
}

/**
 * @brief Calculate the encoder velocity in counts per second for a specified timer.
 * @param htim: Timer handle pointer for the encoder
 * @retval Velocity as a float in counts per second
 */
float Encoder_GetVelocity(TIM_HandleTypeDef *htim)
{

    static int last_count[2] = {0};       // Store previous count for each encoder
    static uint32_t last_time[2] = {0};   // Store previous time for each encoder

    int encoder_index = (htim->Instance == TIM1) ? 0 : 1;  // Use 0 for TIM1, 1 for TIM4

    int current_count = __HAL_TIM_GET_COUNTER(htim);
    uint32_t current_time = HAL_GetTick();

    // Calculate time difference in seconds
    float time_diff = (current_time - last_time[encoder_index]) / 1000.0f; // Convert ms to seconds

    // Calculate count difference
    int32_t count_diff = current_count - last_count[encoder_index];

    // Store current count and time for the next call
    last_count[encoder_index] = current_count;
    last_time[encoder_index] = current_time;


    old_value_enc=count_diff / time_diff;
    if (time_diff > 0)
    {
		if(fabs(count_diff)<1000)
			return count_diff / time_diff;
		else
			return old_value_enc;
    }

    else
        return 0.0f;

}
