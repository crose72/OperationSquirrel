#pragma once

/********************************************************************************
 * @file    vehicle_controller.h
 * @author  Cameron Rose
 * @date    6/7/2023
 ********************************************************************************/
#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "mavlink_cmd_handler.h"
#include "mavlink_msg_handler.h"
#include "velocity_controller.h"
#include "attitude_controller.h"
#include "follow_target.h"
#include "system_controller.h"

/********************************************************************************
 * Imported objects
 ********************************************************************************/
extern float dt_25ms;

/********************************************************************************
 * Exported objects
 ********************************************************************************/

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/
class VehicleController
{
    VehicleController();
    ~VehicleController();

public:
    static void vehicle_control_init(void);
    static void vehicle_control_loop(void);
    static void vehicle_control_shutdown(void);
    static void cmd_position_NED(float position_target[3]);
    static void cmd_velocity_NED(float velocity_target[3]);
    static void cmd_acceleration_NED(float acceleration_target[3]);
    static void cmd_velocity_x_NED(float velocity_target);
    static void cmd_velocity_y_NED(float velocity_target);
    static void cmd_velocity_z_NED(float velocity_target);
    static void cmd_velocity_xy_NED(float velocity_target[3]);
};

#endif // VEHICLE_CONTROLLER_H
