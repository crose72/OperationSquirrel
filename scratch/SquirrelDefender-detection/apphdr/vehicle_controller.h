/********************************************************************************
 * @file    vehicle_controller.h
 * @author  Cameron Rose
 * @date    12/27/2023
 ********************************************************************************/
#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "mavlink_cmd_handler.h"
#include "velocity_controller.h"

/********************************************************************************
 * Imported objects
 ********************************************************************************/

/********************************************************************************
 * Exported objects
 ********************************************************************************/

/********************************************************************************
 * Function prototypes
 ********************************************************************************/
class VehicleController 
{
public:
    VehicleController();
    ~VehicleController();

    // Public member variables

    // Public member functions
    void cmd_position(float position_target[3]);
    void cmd_velocity(float velocity_target[3]);
    void cmd_acceleration(float acceleration_target[3]);

private:
    // Private member variables


    // Create instance of external classes whose functionality is needed
    //MavCmd mav;
    //VelocityController vc;

    // Private member functions
    float calc_yaw_target(float x, float y);
    float calc_yaw_rate_target(float x, float y);
};


#endif // VEHICLE_CONTROLLER_H