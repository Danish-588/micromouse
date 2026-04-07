/*
 * imu.h
 *
 *  Created on: Jun 27, 2021
 *      Author: Soham
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

struct imu
{
	struct uart_pins *comm_protocol;
	int comm_protocol_speed;
}sparton,arduimu;

void sparton_init();
void sparton_tare();
void sparton_isr();
void sparton2_isr();
void sparton_request_angle();
void arduimu_init();
void arduimu_isr();
void arduimu_poll();
//Sparton
volatile int while_check, can_isr_check , sparton_isr_check ;
volatile int i , k1 ;
volatile float sparton_angle;
volatile int entryflag1 , entryflag2 , flag_float , once , float_data[20], flt ;
volatile int integer ;
volatile float floating ;
volatile int data_u5, data1[100];
volatile float angle1;
volatile int j ;
volatile int next , first_byte, second_byte;
volatile int ascii, toint, isr_check;
volatile int go,final_data,m;
volatile float angle ;
volatile int tare_done ;
volatile int sparton_angle_int;
char flt_data[15];
volatile char data_char;
char* pend;
volatile int checksum_calculated;
volatile int checksum_received;

//ArduIMU
volatile int temp_angle;
volatile bool next_byte;
volatile int combined_angle;
volatile int raw_angle;

#endif /* INC_IMU_H_ */
