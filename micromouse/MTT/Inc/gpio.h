/*
 * gpio.h
 *
 *  Created on: Jun 21, 2021
 *      Author: Soham
 */

#ifndef INC_GPIO_H_
#define INC_GPIO_H_

#define PIN0                 ((uint16_t)0x0001)  /* Pin 0 selected    */
#define PIN1                 ((uint16_t)0x0002)  /* Pin 1 selected    */
#define PIN2                 ((uint16_t)0x0004)  /* Pin 2 selected    */
#define PIN3                 ((uint16_t)0x0008)  /* Pin 3 selected    */
#define PIN4                 ((uint16_t)0x0010)  /* Pin 4 selected    */
#define PIN5                 ((uint16_t)0x0020)  /* Pin 5 selected    */
#define PIN6                 ((uint16_t)0x0040)  /* Pin 6 selected    */
#define PIN7                 ((uint16_t)0x0080)  /* Pin 7 selected    */
#define PIN8                 ((uint16_t)0x0100)  /* Pin 8 selected    */
#define PIN9                 ((uint16_t)0x0200)  /* Pin 9 selected    */
#define PIN10                ((uint16_t)0x0400)  /* Pin 10 selected   */
#define PIN11                ((uint16_t)0x0800)  /* Pin 11 selected   */
#define PIN12                ((uint16_t)0x1000)  /* Pin 12 selected   */
#define PIN13                ((uint16_t)0x2000)  /* Pin 13 selected   */
#define PIN14                ((uint16_t)0x4000)  /* Pin 14 selected   */
#define PIN15                ((uint16_t)0x8000)  /* Pin 15 selected   */
#define PINAll               ((uint16_t)0xFFFF)  /* All pins selected */

#define  nopull        0x00000000U   /*!< No Pull-up or Pull-down activation  */
#define  pullup        0x00000001U   /*!< Pull-up activation                  */
#define  pulldown      0x00000002U   /*!< Pull-down activation                */

#define  rising                    (EXTI_MODE | GPIO_MODE_IT_RISING)                 /*!< External Interrupt Mode with Rising edge trigger detection          */
#define  falling                   (EXTI_MODE | GPIO_MODE_IT_FALLING)  /*!< External Interrupt Mode with Falling edge trigger detection         */
#define  risingfalling            (EXTI_MODE | GPIO_MODE_IT_RISING|GPIO_MODE_IT_FALLING)  /*!< External Interrupt Mode with Rising/Falling edge trigger detection  */

typedef void (*exti_isrfunciton)(void);

struct exti_pins
{
	exti_isrfunciton exti_isrfunction;
	int enable;
}pin0,pin1,pin2,pin3,pin4,pin5,pin6,pin7,pin8,pin9,pin10,pin11,pin12,pin13,pin14,pin15;

volatile int isr_ext0_check;
volatile int isr_ext1_check;
volatile int isr_ext2_check;
volatile int isr_ext3_check;
volatile int isr_ext4_check;
volatile int isr_ext5_check;
volatile int isr_ext6_check;
volatile int isr_ext7_check;
volatile int isr_ext8_check;
volatile int isr_ext9_check;
volatile int isr_ext10_check;
volatile int isr_ext11_check;
volatile int isr_ext12_check;
volatile int isr_ext13_check;
volatile int isr_ext14_check;
volatile int isr_ext15_check;

void exti_isr(void isrhandler(void));
void input(GPIO_TypeDef  *GPIOx, uint16_t pin, uint32_t pull);
void output(GPIO_TypeDef  *GPIOx, uint16_t pin);
void setpin(GPIO_TypeDef  *GPIOx, uint16_t pin);
void resetpin(GPIO_TypeDef  *GPIOx, uint16_t pin);
void togglepin(GPIO_TypeDef  *GPIOx, uint16_t pin);
void sendtoled(int value);
void gpio_interrupt(GPIO_TypeDef  *GPIOx, uint16_t pin, uint32_t pull,uint32_t edge,void (*exti_handler)(void));

#endif /* INC_GPIO_H_ */
