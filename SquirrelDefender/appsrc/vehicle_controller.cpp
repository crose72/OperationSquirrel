/********************************************************************************
 * @file    vehicle_controller.cpp
 * @author  Cameron Rose
 * @date    6/7/2023
 * @brief   Control the position, velocity, and acceleration of the drone by
 *          sending the following MAVLINK message to the drone.  Control the
 *          vector position, velocity, acceleration, and yaw/yaw rate.
 *
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "vehicle_controller.h"
#include "sim_flight_test_4_VelocityControl.h"

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/
DebugTerm VehStateInfo("");
bool takeoff_dbc;
uint16_t takeoff_dbc_cnt;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
 * Function: VehicleController
 * Description: Constructor of the VehicleController class.
 ********************************************************************************/
VehicleController::VehicleController(void) {}

/********************************************************************************
 * Function: VehicleController
 * Description: Constructor of the VehicleController class.
 ********************************************************************************/
VehicleController::~VehicleController(void) {}

/********************************************************************************
 * Function: cmd_position_NED
 * Description: Move to an x,y,z coordinate in the NED frame.
 ********************************************************************************/
void VehicleController::cmd_position_NED(float position_target[3])
{
    VelocityController::cmd_position_NED(position_target);
}

/********************************************************************************
 * Function: cmd_velocity_NED
 * Description: Move in direction of vector vx,vy,vz in the NED frame.
 ********************************************************************************/
void VehicleController::cmd_velocity_NED(float velocity_target[3])
{
    VelocityController::cmd_velocity_NED(velocity_target);
}

/********************************************************************************
 * Function: cmd_velocity_xy_NED
 * Description: Move in xy plane given a vector vx,vy in the NED frame.
 ********************************************************************************/
void VehicleController::cmd_velocity_xy_NED(float velocity_target[3])
{
    VelocityController::cmd_velocity_xy_NED(velocity_target);
}

/********************************************************************************
 * Function: cmd_velocity_x_NED
 * Description: Move in direction of vector vx in the NED frame.
 ********************************************************************************/
void VehicleController::cmd_velocity_x_NED(float velocity_target)
{
    VelocityController::cmd_velocity_x_NED(velocity_target);
}

/********************************************************************************
 * Function: cmd_velocity_y_NED
 * Description: Move in direction of vector vy in the NED frame.
 ********************************************************************************/
void VehicleController::cmd_velocity_y_NED(float velocity_target)
{
    VelocityController::cmd_velocity_y_NED(velocity_target);
}

/********************************************************************************
 * Function: cmd_acceleration_NED
 * Description: Move in direction of vector ax,ay,az in the NED frame.
 ********************************************************************************/
void VehicleController::cmd_acceleration_NED(float acceleration_target[3])
{
    VelocityController::cmd_acceleration_NED(acceleration_target);
}

#ifdef USE_JETSON

/********************************************************************************
 * Function: follow_target
 * Description: Move in direction of vector ax,ay,az in the NED frame.
 ********************************************************************************/
void VehicleController::follow_mode(void)
{
    float target_velocity[3] = {0.0, 0.0, 0.0};

    Follow::follow_control_loop();

    target_velocity[0] = vx_adjust;
    target_velocity[1] = vy_adjust;
    target_velocity[2] = vz_adjust;

    if (target_too_close)
    {
        VehicleController::cmd_velocity_xy_NED(target_velocity);
    }
    else
    {
        VehicleController::cmd_velocity_NED(target_velocity);
    }
}

#endif // USE_JETSON

/********************************************************************************
 * Function: vehicle_control_init
 * Description: Initial setup of vehicle controller.
 ********************************************************************************/
bool VehicleController::vehicle_control_init(void)
{
    takeoff_dbc = false;
    takeoff_dbc_cnt = 200;

    return true;
}

/********************************************************************************
 * Function: vehicle_control_loop
 * Description: Vehicle controller main loop.  Handles all
 ********************************************************************************/
void VehicleController::vehicle_control_loop(void)
{
    if (system_state == SYSTEM_STATE::INIT)
    {
        MavCmd::set_mode_GUIDED();
    }
    else if (system_state == SYSTEM_STATE::PRE_ARM_GOOD)
    {
        MavCmd::arm_vehicle();
    }
    else if (system_state == SYSTEM_STATE::STANDBY)
    {
        MavCmd::takeoff_GPS_long((float)2.0);
    }
    else if (system_state == SYSTEM_STATE::IN_FLIGHT_GOOD)
    {
#ifdef USE_JETSON

        if (takeoff_dbc_cnt > 0)
        {
            takeoff_dbc_cnt--;
        }
        else
        {
            takeoff_dbc_cnt = 0;
            takeoff_dbc = true;
        }

        if (takeoff_dbc)
        {
            follow_mode();
        }

#elif USE_WSL

        if (takeoff_dbc_cnt > 0)
        {
            takeoff_dbc_cnt--;
        }
        else
        {
            takeoff_dbc_cnt = 0;
            takeoff_dbc = true;
        }

        if (takeoff_dbc)
        {
            test_flight();
        }

#endif // USE_JETSON
    }
}

/********************************************************************************
 * Function: vehicle_control_shutdown
 * Description: Code needed to shutdown the vehicle controller.
 ********************************************************************************/
void VehicleController::vehicle_control_shutdown(void)
{
    MavCmd::set_mode_LAND();
}