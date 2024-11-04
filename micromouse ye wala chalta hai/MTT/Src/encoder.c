/*
 * encoder.c
 *
 *  Created on: Nov 5, 2024
 *      Author: hrish
 */
#include "encoder.h"
#include "encoder.h"

TIM_HandleTypeDef htim1;
static uint16_t encoder_ppr = 0;
static int32_t last_count = 0;
static uint32_t last_time = 0;

/**
 * @brief Initialize the TIM1 for encoder mode with specified PPR.
 * @param htim: Timer handle pointer
 * @param ppr: Pulses per revolution for the encoder
 */
void Encoder_Init(TIM_HandleTypeDef *htim, uint16_t ppr)
{
    encoder_ppr = ppr;

    __HAL_RCC_TIM1_CLK_ENABLE();

    htim->Instance = TIM1;
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

    // Initialize time and count
    last_count = __HAL_TIM_GET_COUNTER(htim);
    last_time = HAL_GetTick();
}

/**
 * @brief Get the current raw encoder count
 * @retval Encoder count as an integer
 */
int32_t Encoder_GetCount(void)
{
    return __HAL_TIM_GET_COUNTER(&htim1);
}

/**
 * @brief Calculate the encoder velocity in counts per second
 * @retval Velocity as a float in counts per second
 */
float Encoder_GetVelocity(void)
{
    int32_t current_count = __HAL_TIM_GET_COUNTER(&htim1);
    uint32_t current_time = HAL_GetTick();

    // Calculate time difference in seconds
    float time_diff = (current_time - last_time) / 1000.0f; // Convert ms to seconds

    // Calculate count difference
    int32_t count_diff = current_count - last_count;

    // Store current count and time for the next call
    last_count = current_count;
    last_time = current_time;

    // Calculate and return velocity in counts per second
    if (time_diff > 0)
        return count_diff / time_diff;
    else
        return 0.0f;
}
