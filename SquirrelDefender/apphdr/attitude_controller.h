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
extern float mav_veh_thrust_target;

extern float mav_veh_q1_actual;
extern float mav_veh_q2_actual;
extern float mav_veh_q3_actual;
extern float mav_veh_q4_actual;
extern float mav_veh_roll_rate_actual;
extern float mav_veh_pitch_rate_actual;
extern float mav_veh_yaw_rate_actual;
extern float mav_veh_thrust_actual;

/********************************************************************************
 * Exported objects
 ********************************************************************************/

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/
bool dtrmn_attitude_target_error(void);
void brake(void);
void move_forward(void);
void attitude_yaw(float yaw_pos, float yaw_rate);

#endif // ATTITUDE_CONTROLLER_H
