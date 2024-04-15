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
class VelocityController 
{
public:
    VelocityController();
    ~VelocityController();

    // Public member variables

    // Public member functions
    void cmd_position(float position_target[3]);
    void cmd_velocity(float velocity_target[3]);
    void cmd_acceleration(float acceleration_target[3]);

private:
    // Private member variables
    static mavlink_set_position_target_local_ned_t desired_position_target;
    static mavlink_set_position_target_local_ned_t desired_velocity_target;
    static mavlink_set_position_target_local_ned_t desired_acceleration_target;

    // Create instance of external classes whose functionality is needed
    MavCmd mav;

    // Private member functions
    float calc_yaw_target(float x, float y);
    float calc_yaw_rate_target(float x, float y);
};


#endif // VELOCITY_CONTROLLER_H