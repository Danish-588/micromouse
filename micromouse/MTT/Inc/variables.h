/*
 * variables.h
 *
 *  Created on: Jun 19, 2021
 *      Author: Soham
 */

#ifndef INC_VARIABLES_H_
#define INC_VARIABLES_H_

//SYSCLOCK
#define SYS_CLOCK 180000000

//TIMER
volatile int isr_timer1_check;
volatile int isr_timer2_check;
volatile int isr_timer3_check;
volatile int isr_timer4_check;
volatile int isr_timer5_check;
volatile int isr_timer6_check;
volatile int isr_timer7_check;
volatile int isr_timer8_check;
volatile int isr_timer9_check;
volatile int isr_timer10_check;
volatile int isr_timer11_check;
volatile int isr_timer12_check;
volatile int isr_timer13_check;
volatile int isr_timer14_check;

//----------------------------------IMU----------------------------------//
extern volatile float angle_temp, angle_deadreckon_point, angle;
//------For Error Correction---------//
extern volatile float req_angle, error_angle;
extern volatile bool angle_correct;
extern volatile float theta;
//----------------------------------Distance Sensor----------------------------------//
extern volatile float dist1, dist2, dist3, dist4;
extern volatile int data_dist1, data_dist2, data_dist3, data_dist4;

//------For Error Correction---------//
extern volatile float req_sensor_x, req_sensor_y, req_sensor_angle;
extern volatile float error_sensor_x, error_sensor_y, error_sensor_angle;
extern volatile int error_sensor_x_dir, error_sensor_y_dir, error_sensor_angle_dir;
extern volatile bool y_sensor_correct, x_sensor_correct;

//----------------------------------Velocity----------------------------------//
extern volatile float v1, v2, v3, v4;
extern volatile float vx, vy, vy_temp, w, v_scaling_factor, rpm1, rpm2, rpm3, rpm4;
extern volatile float bot_radius, accs, deaccs, accs_w, deaccs_w, omni_front_angle;
extern volatile int drive;
extern volatile bool holonomic, rpm_ctrl;
extern volatile float vx_limit, vy_limit, w_limit;

//----------------------------------XY----------------------------------//
extern volatile float x, y, x_scale, y_scale, x_final, y_final, x_deadreck, y_deadreck, x_photo, y_photo, velocity_x_xy, velocity_y_xy;

//------For Error Correction---------//
extern volatile float req_x, req_y, error_x, error_y;
extern volatile int error_x_dir, error_y_dir, error_angle_dir;
extern volatile bool x_correct, y_correct;
extern volatile float x_xy_buffer, y_xy_buffer, x_sensor_buffer, y_sensor_buffer;

//----------------------------------Accs----------------------------------//
extern volatile float accs_ka_accs;
extern volatile double accs_vy_limit, accs_vx_limit, accs_vy_limit_neg, accs_vx_limit_neg;
extern volatile bool accs_enable_x, accs_enable_y;

//----------------------------------Stall----------------------------------//
extern volatile bool stall;
extern volatile float stall_deaccs, stall_w_deaccs;

//----------------------------------Navigation----------------------------------//
extern volatile int navi_choice, next_case, direct_next, angle_check;

//----------------------------------Bezier----------------------------------//
extern volatile float t, bezier_buff;


//----------------------------------Navigation----------------------------------//
//enum navi_case{case1, case2};   //Replace with case names for the navigation cases

//----------------------------------Holo-N_Axis----------------------------------//

extern volatile bool r_deadzone, l_deadzone;
extern volatile uint8_t deadzone_limit_l, deadzone_limit_r;
extern volatile int8_t before_sign, after_sign;
extern volatile float v_magnitude, vx_req, vy_req, vy_temp_req, vy_temp, v_limit, error_vx, error_vy, denom_v, denom_error, denom_ps, w_req, error_w;
#endif /* INC_VARIABLES_H_ */
