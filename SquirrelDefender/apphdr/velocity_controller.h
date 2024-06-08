#pragma once

/********************************************************************************
 * @file    velocity_controller.h
 * @author  Cameron Rose
 * @date    6/7/2023
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
 * Function prototypes and Class Definitions
 ********************************************************************************/
class VelocityController
{
    public:
        VelocityController(void);
        ~VelocityController(void);

        void cmd_position_NED(float position_target[3]);
        void cmd_velocity_NED(float velocity_target[3]);
        void cmd_acceleration_NED(float acceleration_target[3]);
        void cmd_velocity_x_NED(float velocity_target);
        void cmd_velocity_y_NED(float velocity_target);
        void cmd_velocity_z_NED(float velocity_target);
        void cmd_velocity_xy_NED(float velocity_target[3]);
        
    private:
        float calc_yaw_target(float x, float y);
        float calc_yaw_rate_target(float x, float y);
};


#endif // VELOCITY_CONTROLLER_H