/*
 * odrivecan.h
 *
 *  Created on: Apr 7, 2022
 *      Author: Mihika Moni
 */

#ifndef INC_ODRIVECAN_H_
#define INC_ODRIVECAN_H_


enum commands
{
    axis_node_id,
    req_state,
    controller_mode,
    input_pos,
    input_vel,
    input_torque,
    vel_limit,
    curr_lim,
    traj_vel_lim,
    traj_accel_lim,
    traj_decel_lim,
    traj_inertia,
    reboot,
    clear_errors,
    linear_count,
    pos_gain,
    vel_gains,
};

enum states
{
  idlestate = 1,
  closedloop = 8,
  lockin = 9,
  homing = 11
};

enum control_modes
{
    volt_control = 0,
    torq_control = 1,
    vel_control = 2,
    pos_control = 3,
};

enum input_modes
{
    passthrough = 1,
    trap_traj = 5,
};

int command_id;

int f2b(float fl);
float b2f(int b);
float u2f(uint32_t u);
void odcansend(int node, int command, float value);
void can_set_controller_mode(int node, uint8_t con_mode, uint8_t input_mode);
void set_limits(int node, float v_lim, float c_lim);
void set_traj_limits(int node, float traj_accel, float traj_decel);
void can_transmit_64(int msg_id, uint64_t data);

#endif /* INC_ODRIVECAN_H_ */
