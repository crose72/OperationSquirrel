#pragma once

/********************************************************************************
 * @file    velocity_controller.h
 * @author  Cameron Rose
 * @date    12/27/2023
 ********************************************************************************/
#ifndef VELOCITY_CONTROLLER_H
#define VELOCITY_CONTROLLER_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "mavlink_cmd_handler.h"
#include "mavlink_msg_handler.h"

/********************************************************************************
 * Imported objects
 ********************************************************************************/

/********************************************************************************
 * Exported objects
 ********************************************************************************/

/********************************************************************************
 * Function prototypes
 ********************************************************************************/
void cmd_position(float position_target[3]);
void cmd_velocity(float velocity_target[3]);
void cmd_velocity_x(float velocity_target);
void cmd_acceleration(float acceleration_target[3]);
float calc_yaw_target(float x, float y);
float calc_yaw_rate_target(float x, float y);

#endif // VELOCITY_CONTROLLER_H