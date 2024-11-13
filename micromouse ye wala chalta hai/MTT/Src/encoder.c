/*
 * encoder.c
 *
 *  Created on: Nov 5, 2024
 *      Author: hrish
 */
#include "encoder.h"

static uint16_t encoder_ppr = 3000;
float old_value_enc= 0;
float old_value_enc_TIM1 = 0;
float old_value_enc_TIM4 = 0;


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
float Encoder_GetVelocity(TIM_HandleTypeDef *htim) {
    static int last_count[2] = {0};
    static uint32_t last_time[2] = {0};

    int encoder_index = (htim->Instance == TIM1) ? 0 : 1;
    int current_count = __HAL_TIM_GET_COUNTER(htim);
    uint32_t current_time = HAL_GetTick();

    float time_diff = (current_time - last_time[encoder_index]) / 1000.0f;
    int32_t count_diff = current_count - last_count[encoder_index];

    last_count[encoder_index] = current_count;
    last_time[encoder_index] = current_time;

    float velocity = 0.0f;

    if (time_diff > 0) {
        velocity = count_diff / time_diff;

        // Validate the velocity
        if (isnan(velocity) || fabs(velocity) > 15000) {
            velocity = old_value_enc;  // Use last valid velocity if out of range or NaN
        } else {
            old_value_enc = velocity;  // Update last valid velocity
        }
    }

    return velocity;
}
float Encoder_GetVelocity_TIM1(TIM_HandleTypeDef *htim) {
    static int last_count = 0;
    static uint32_t last_time = 0;

    int current_count = __HAL_TIM_GET_COUNTER(htim);
    uint32_t current_time = HAL_GetTick();

    float time_diff = (current_time - last_time) / 1000.0f;
    int32_t count_diff = current_count - last_count;

    last_count = current_count;
    last_time = current_time;

    float velocity = 0.0f;

    if (time_diff > 0.001f) {  // Small threshold to avoid division by near-zero time
        velocity = count_diff / time_diff;

        // Only filter out NaN and extremely large spikes
        if (isnan(velocity) || fabs(velocity) > 15000) {
            velocity = old_value_enc_TIM1;  // Use last valid velocity if out of range or NaN
        } else {
            old_value_enc_TIM1 = velocity;  // Update last valid velocity for TIM1
        }
    } else {
        velocity = old_value_enc_TIM1;  // Maintain last valid velocity if time_diff is too short
    }

    return velocity;
}

float Encoder_GetVelocity_TIM4(TIM_HandleTypeDef *htim) {
    static int last_count = 0;
    static uint32_t last_time = 0;

    int current_count = __HAL_TIM_GET_COUNTER(htim);
    uint32_t current_time = HAL_GetTick();

    float time_diff = (current_time - last_time) / 1000.0f;
    int32_t count_diff = current_count - last_count;

    last_count = current_count;
    last_time = current_time;

    float velocity = 0.0f;

    if (time_diff > 0.001f) {  // Small threshold to avoid division by near-zero time
        velocity = count_diff / time_diff;

        // Only filter out NaN and extremely large spikes
        if (isnan(velocity) || fabs(velocity) > 15000) {
            velocity = old_value_enc_TIM4;  // Use last valid velocity if out of range or NaN
        } else {
            old_value_enc_TIM4 = velocity;  // Update last valid velocity for TIM4
        }
    } else {
        velocity = old_value_enc_TIM4;  // Maintain last valid velocity if time_diff is too short
    }

    return velocity;
}
