/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "SysInit.h"
#include "PWM.h"
#include "ADC.h"
#include "UART.h"
#include "GPIO.h"
#include "I2C.h"
#include <stdio.h>

int adc_value = 0;

int main(void)
{

	//SYSINIT
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();

    // Initialize all peripherals
    PWM_Init();                   // Initialize PWM on TIM3 (PA6)
    ADC_Init();                   // Initialize ADC on PA0 (Channel 0)
    UART_Init(9600);              // Initialize UART2 (PA2 TX, PA3 RX) with 9600 baud rate
    GPIO_Init_Output(GPIOA, 5);   // Initialize PA5 as output (for LED control)
    I2C_Init();                   // Initialize I2C1 (PB6 SCL, PB7 SDA)

    // Example: Read ADC and send value over UART
    uint16_t adc_value = ADC_Read();      // Read from ADC (PA0)
    char uart_message[20];
    sprintf(uart_message, "ADC Value: %u\n", adc_value);  // Use sprintf to format the string
    UART_SendString(uart_message);        // Send ADC value over UART

    // Example: Control PWM duty cycle based on ADC value
    uint8_t pwm_duty = (adc_value * 100) / 4095;  // Scale ADC (0-4095) to PWM (0-100%)
    PWM_SetDutyCycle(pwm_duty);          // Set PWM duty cycle

    // Example: Toggle GPIO pin (PA5)
    GPIO_TogglePin(GPIOA, 5);            // Toggle LED (PA5)

    // Example: I2C read/write (assuming a device at address 0xA0)
    uint8_t i2c_data = I2C_Read(0xA0);   // Read data from I2C slave at address 0xA0
    char i2c_message[20];
    sprintf(i2c_message, "I2C Data: %u\n", i2c_data);  // Format the I2C data
    UART_SendString(i2c_message);        // Send the I2C data over UART

    I2C_Write(0xA0, 0x55);               // Write 0x55 to I2C slave at address 0xA0
    UART_SendString("I2C Write Complete\n");



	while (1)
	{
        // Continuously read ADC and adjust PWM duty cycle
        adc_value = ADC_Read();          // Read from ADC (PA0)
        pwm_duty = (adc_value * 100) / 4095;  // Scale ADC value to PWM
        PWM_SetDutyCycle(pwm_duty);      // Adjust PWM based on ADC value

        // Toggle GPIO (LED on PA5) every loop iteration
        GPIO_TogglePin(GPIOA, 5);        // Toggle LED

        // Add a small delay to make the toggling visible
        for (volatile int i = 0; i < 100000; i++);
	}
}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
