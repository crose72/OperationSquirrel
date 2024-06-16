#pragma once

/********************************************************************************
 * @file    attitude_controller.h
 * @author  Cameron Rose
 * @date    6/7/2023
 ********************************************************************************/
#ifndef ATTITUDE_CONTROLLER_H
#define ATTITUDE_CONTROLLER_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "mavlink_cmd_handler.h"
#include "mavlink_msg_handler.h"

/********************************************************************************
 * Imported objects
 ********************************************************************************/
extern float mav_veh_q1_target;
extern float mav_veh_q2_target;
extern float mav_veh_q3_target;
extern float mav_veh_q4_target;
extern float mav_veh_roll_rate_target;
extern float mav_veh_pitch_rate_target;
extern float mav_veh_yaw_rate_target;
extern float thrust_target;
extern float mav_veh_q1;
extern float mav_veh_q2;
extern float mav_veh_q3;
extern float mav_veh_q4;
extern float mav_veh_roll_rate;
extern float mav_veh_pitch_rate;
extern float mav_veh_yaw_rate;
extern float mav_veh_thrust;

/********************************************************************************
 * Exported objects
 ********************************************************************************/

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/
bool dtrmn_attitude_target_error(void);
void brake (void);
void move_forward (void);
void attitude_yaw (float yaw_pos, float yaw_rate);

#endif // ATTITUDE_CONTROLLER_H
