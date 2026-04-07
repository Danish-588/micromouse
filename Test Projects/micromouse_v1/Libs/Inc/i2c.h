/*
 * i2c.h
 *
 *  Created on: Oct 21, 2024
 *      Author: danis
 */

#ifndef INC_I2C_H_
#define INC_I2C_H_

#include "stm32f4xx.h"

void I2C_Init(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_WriteAddress(uint8_t address);
void I2C_WriteData(uint8_t data);
uint8_t I2C_ReadData(void);
void I2C_Ack(void);
void I2C_Nack(void);
uint8_t I2C_Read(uint8_t address);
void I2C_Write(uint8_t address, uint8_t data);


#endif /* INC_I2C_H_ */
