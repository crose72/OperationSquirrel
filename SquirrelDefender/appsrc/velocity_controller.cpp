/********************************************************************************
 * @file    velocity_controller.cpp
 * @author  Cameron Rose
 * @date    6/7/2023
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

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
* Function: VelocityController
* Description: Constructor of the VelocityController class.
********************************************************************************/
VelocityController::VelocityController(void){}

/********************************************************************************
* Function: VelocityController
* Description: Constructor of the VelocityController class.
********************************************************************************/
VelocityController::~VelocityController(void){}

/********************************************************************************
 * Function: cmd_position_NED
 * Description: Move to an x,y,z coordinate in the NED frame.
 ********************************************************************************/
void VelocityController::cmd_position_NED(float position_target[3])
{
    float yaw_target = 0.0;
    uint16_t options = 0;

    options |= POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE | POSITION_TARGET_TYPEMASK_VZ_IGNORE | 
        POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;

    yaw_target = calc_yaw_target(position_target[0], position_target[1]);

    MavCmd::send_cmd_velocity_target(SENDER_SYS_ID, SENDER_COMP_ID, TARGET_SYS_ID, TARGET_COMP_ID, 
                                      position_target[0], position_target[1], position_target[2], yaw_target, options, MAV_FRAME_BODY_OFFSET_NED);
}

/********************************************************************************
 * Function: cmd_velocity_NED
 * Description: Move in direction of vector vx,vy,vz in the NED frame.
 ********************************************************************************/
void VelocityController::cmd_velocity_NED(float velocity_target[3])
{
    float yaw_target = 0.0;
    uint16_t options = 0;

    options |= POSITION_TARGET_TYPEMASK_X_IGNORE | POSITION_TARGET_TYPEMASK_Y_IGNORE | POSITION_TARGET_TYPEMASK_Z_IGNORE | 
        POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;

    yaw_target = calc_yaw_target(velocity_target[0], velocity_target[1]);

    MavCmd::send_cmd_velocity_target(SENDER_SYS_ID, SENDER_COMP_ID, TARGET_SYS_ID, TARGET_COMP_ID, 
                                      velocity_target[0], velocity_target[1], velocity_target[2], yaw_target, options, MAV_FRAME_BODY_OFFSET_NED);
}

/********************************************************************************
 * Function: cmd_velocity_xy_NED
 * Description: Move in xy plane given a vector vx,vy in the NED frame.
 ********************************************************************************/
void VelocityController::cmd_velocity_xy_NED(float velocity_target[3])
{
    float yaw_target = 0.0;
    uint16_t options = 0;

    options |= POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE | 
            POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;

    yaw_target = calc_yaw_target(velocity_target[0], velocity_target[1]);

    MavCmd::send_cmd_velocity_target(SENDER_SYS_ID, SENDER_COMP_ID, TARGET_SYS_ID, TARGET_COMP_ID, 
                                      velocity_target[0], velocity_target[1], 0.0, yaw_target, options, MAV_FRAME_BODY_OFFSET_NED);
}

/********************************************************************************
 * Function: cmd_velocity_x_NED
 * Description: Move in direction of vector vx in the NED frame.
 ********************************************************************************/
void VelocityController::cmd_velocity_x_NED(float velocity_target)
{
    uint16_t options = 0;

    options |= POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE | 
            POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;

    MavCmd::send_cmd_velocity_target(SENDER_SYS_ID, SENDER_COMP_ID, TARGET_SYS_ID, TARGET_COMP_ID, 
                                      velocity_target, 0.0, 0.0, 0.0, options, MAV_FRAME_BODY_OFFSET_NED);
}

/********************************************************************************
 * Function: cmd_velocity_y_NED
 * Description: Move in direction of vector vy in the NED frame.
 ********************************************************************************/
void VelocityController::cmd_velocity_y_NED(float velocity_target)
{
    uint16_t options = 0;

    options |= POSITION_TARGET_TYPEMASK_X_IGNORE | POSITION_TARGET_TYPEMASK_Y_IGNORE | POSITION_TARGET_TYPEMASK_Z_IGNORE | 
            POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VZ_IGNORE | 
            POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE | 
            POSITION_TARGET_TYPEMASK_YAW_IGNORE | POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;

    MavCmd::send_cmd_velocity_target(SENDER_SYS_ID, SENDER_COMP_ID, TARGET_SYS_ID, TARGET_COMP_ID, 
                                      0.0, velocity_target, 0.0, 0.0, options, MAV_FRAME_BODY_OFFSET_NED);
}

/********************************************************************************
 * Function: cmd_velocity_y_NED
 * Description: Move in direction of vector vy in the NED frame.
 ********************************************************************************/
void VelocityController::cmd_velocity_z_NED(float velocity_target)
{
    uint16_t options = 0;

    options |= POSITION_TARGET_TYPEMASK_X_IGNORE | POSITION_TARGET_TYPEMASK_Y_IGNORE | POSITION_TARGET_TYPEMASK_Z_IGNORE | 
            POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE | 
            POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE | 
            POSITION_TARGET_TYPEMASK_YAW_IGNORE | POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;

    MavCmd::send_cmd_velocity_target(SENDER_SYS_ID, SENDER_COMP_ID, TARGET_SYS_ID, TARGET_COMP_ID, 
                                      0.0, 0.0, velocity_target, 0.0, options, MAV_FRAME_BODY_OFFSET_NED);
}

/********************************************************************************
 * Function: calc_yaw_target
 * Description: Calculate a yaw target based on the forward and lateral movement
 *              commands.
 ********************************************************************************/
float VelocityController::calc_yaw_target(float x, float y)
{
    return atan2(y,x);
}

/********************************************************************************
 * Function: calc_yaw_rate_target
 * Description: Calculate a yaw rate target based on the forward and lateral movement
 *              commands.
 ********************************************************************************/
float VelocityController::calc_yaw_rate_target(float x, float y)
{
    return atan2(y,x);
}
