/********************************************************************************
 * @file    velocity_controller.cpp
 * @author  Cameron Rose
 * @date    12/27/2023
 * @brief   Control the position, velocity, and acceleration of the drone by 
 *          sending the following MAVLINK message to the drone.  Control the
 *          vector position, velocity, acceleration, and yaw/yaw rate.
 * 
            SET_POSITION_TARGET_LOCAL_NED ( #84 )
            [Message] Sets a desired vehicle position in a local north-east-down coordinate frame. Used by an external controller to command the vehicle (manual controller or other system).

            Field Name	        Type	    Units	Values	                    Description
            time_boot_ms	    uint32_t	ms		                            Timestamp (time since system boot).
            target_system	    uint8_t			                                System ID
            target_component	uint8_t			                                Component ID
            coordinate_frame	uint8_t		        MAV_FRAME	                Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9
            type_mask	        uint16_t		    POSITION_TARGET_TYPEMASK	Bitmap to indicate which dimensions should be ignored by the vehicle.
            x	                float	    m		                            X Position in NED frame
            y	                float	    m		                            Y Position in NED frame
            z	                float	    m		                            Z Position in NED frame (note, altitude is negative in NED)
            vx	                float	    m/s		                            X velocity in NED frame
            vy	                float	    m/s		                            Y velocity in NED frame
            vz	                float	    m/s		                            Z velocity in NED frame
            afx	                float	    m/s/s		                        X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
            afy	                float	    m/s/s		                        Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
            afz	                float	    m/s/s		                        Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
            yaw	                float	    rad		                            yaw setpoint
            yaw_rate	        float	    rad/s		                        yaw rate setpoint
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "velocity_controller.h"

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/
mavlink_set_position_target_local_ned_t desired_position_target;
mavlink_set_position_target_local_ned_t desired_velocity_target;
mavlink_set_position_target_local_ned_t desired_acceleration_target;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
 * Function: 
 * Description: 
 ********************************************************************************/

void cmd_position(float position_target[3])
{
    float yaw_target;

    yaw_target = calc_yaw_target(position_target[0], position_target[1]);

    desired_position_target.x = position_target[0];
    desired_position_target.y = position_target[1];
    desired_position_target.z = position_target[2];
    desired_position_target.yaw = yaw_target;
    desired_position_target.type_mask = 0b0000101111111000; // Ignore velocity and accel and yaw rate
    desired_position_target.coordinate_frame = MAV_FRAME_BODY_OFFSET_NED; // MAV_FRAME_BODY_OFFSET_NED

    MavCmd::send_cmd_set_position_target_local_ned(&desired_position_target);
}

void cmd_velocity(float velocity_target[3])
{
    float yaw_target;
    uint16_t mask = 0;

    yaw_target = calc_yaw_target(velocity_target[0], velocity_target[1]);

    desired_velocity_target.vx = velocity_target[0];
    desired_velocity_target.vy = velocity_target[1];
    desired_velocity_target.vz = velocity_target[2];
    /*
    if (velocity_target[0] < 0)
    {
        desired_velocity_target.type_mask = 0b0000100111000101; // Ignore position and accel and yaw rate
    }
    else
    {
        desired_velocity_target.yaw = yaw_target;
        desired_velocity_target.type_mask = 0b0000100111000111; // Ignore position and accel and yaw rate
    }
    */

    
    desired_velocity_target.vy = velocity_target[1];
    desired_velocity_target.yaw = yaw_target;
    desired_velocity_target.type_mask = 0b0000101111000111; // Ignore position and accel and yaw rate
    
    
    desired_velocity_target.coordinate_frame = MAV_FRAME_BODY_OFFSET_NED; // MAV_FRAME_BODY_OFFSET_NED

    MavCmd::send_cmd_set_position_target_local_ned(&desired_velocity_target);
}

void cmd_velocity_x(float velocity_target)
{
    desired_velocity_target.vx = velocity_target;
    desired_velocity_target.type_mask = 0b0000100111000101; // Ignore position and accel and yaw rate
    desired_velocity_target.coordinate_frame = MAV_FRAME_BODY_OFFSET_NED; // MAV_FRAME_BODY_OFFSET_NED

    MavCmd::send_cmd_set_position_target_local_ned(&desired_velocity_target);
}

void cmd_acceleration(float acceleration_target[3])
{
    float yaw_target;

    yaw_target = calc_yaw_target(acceleration_target[0], acceleration_target[1]);

    desired_acceleration_target.afx = acceleration_target[0];
    desired_acceleration_target.afy = acceleration_target[1];
    desired_acceleration_target.afz = acceleration_target[2];
    desired_acceleration_target.yaw_rate = yaw_target;
    desired_acceleration_target.type_mask = 0b0000100000111111; // Ignore position and vel and yaw
    desired_acceleration_target.coordinate_frame = MAV_FRAME_BODY_OFFSET_NED; // MAV_FRAME_BODY_OFFSET_NED

    MavCmd::send_cmd_set_position_target_local_ned(&desired_acceleration_target);
}

float calc_yaw_target(float x, float y)
{
    return atan2(y,x);
}

float calc_yaw_rate_target(float x, float y)
{
    return atan2(y,x);
}
