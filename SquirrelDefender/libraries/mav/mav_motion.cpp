/********************************************************************************
 * @file    mav_motion_utils.cpp
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   Utilities for controlling the position, velocity, acceleration, or
 *          attitude of the vehicle.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "mav_motion.h"

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
 * Function: MavMotion
 * Description: Constructor of the MavMotion class.
 ********************************************************************************/
MavMotion::MavMotion(void) {}

/********************************************************************************
 * Function: MavMotion
 * Description: Constructor of the MavMotion class.
 ********************************************************************************/
MavMotion::~MavMotion(void) {}

/********************************************************************************
 * Function: cmd_position_NED
 * Description: Move to an x,y,z coordinate in the NED frame.
 ********************************************************************************/
void MavMotion::cmd_position_NED(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, float position_target[3])
{
    float yaw_target = 0.0;
    uint16_t options = 0;

    options |= POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE | POSITION_TARGET_TYPEMASK_VZ_IGNORE |
               POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
               POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;

    yaw_target = calc_yaw_target(position_target[0], position_target[1]);

    send_cmd_velocity_target(sender_sys_id, sender_comp_id, target_sys_id, target_comp_id,
                             position_target[0], position_target[1], position_target[2], yaw_target, options, MAV_FRAME_BODY_OFFSET_NED);
}

/********************************************************************************
 * Function: cmd_velocity_NED
 * Description: Move in direction of vector vx,vy,vz in the NED frame.
 ********************************************************************************/
void MavMotion::cmd_velocity_NED(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, float velocity_target[3], float yaw_target)
{
    uint16_t options = 0;

    options |= POSITION_TARGET_TYPEMASK_X_IGNORE | POSITION_TARGET_TYPEMASK_Y_IGNORE | POSITION_TARGET_TYPEMASK_Z_IGNORE |
               POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
               POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;

    send_cmd_velocity_target(sender_sys_id, sender_comp_id, target_sys_id, target_comp_id,
                             velocity_target[0], velocity_target[1], velocity_target[2], yaw_target, options, MAV_FRAME_BODY_OFFSET_NED);
}

/********************************************************************************
 * Function: cmd_velocity_xy_NED
 * Description: Move in xy plane given a vector vx,vy in the NED frame.
 ********************************************************************************/
void MavMotion::cmd_velocity_xy_NED(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, float velocity_target[3])
{
    float yaw_target = 0.0;
    uint16_t options = 0;

    options |= POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
               POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;

    yaw_target = calc_yaw_target(velocity_target[0], velocity_target[1]);

    send_cmd_velocity_target(sender_sys_id, sender_comp_id, target_sys_id, target_comp_id,
                             velocity_target[0], velocity_target[1], 0.0, yaw_target, options, MAV_FRAME_BODY_OFFSET_NED);
}

/********************************************************************************
 * Function: cmd_velocity_x_NED
 * Description: Move in direction of vector vx in the NED frame.
 ********************************************************************************/
void MavMotion::cmd_velocity_x_NED(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, float velocity_target)
{
    uint16_t options = 0;

    options |= POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
               POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;

    send_cmd_velocity_target(sender_sys_id, sender_comp_id, target_sys_id, target_comp_id,
                             velocity_target, 0.0, 0.0, 0.0, options, MAV_FRAME_BODY_OFFSET_NED);
}

/********************************************************************************
 * Function: cmd_velocity_y_NED
 * Description: Move in direction of vector vy in the NED frame.
 ********************************************************************************/
void MavMotion::cmd_velocity_y_NED(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, float velocity_target)
{
    uint16_t options = 0;

    options |= POSITION_TARGET_TYPEMASK_X_IGNORE | POSITION_TARGET_TYPEMASK_Y_IGNORE | POSITION_TARGET_TYPEMASK_Z_IGNORE |
               POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VZ_IGNORE |
               POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
               POSITION_TARGET_TYPEMASK_YAW_IGNORE | POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;

    send_cmd_velocity_target(sender_sys_id, sender_comp_id, target_sys_id, target_comp_id,
                             0.0, velocity_target, 0.0, 0.0, options, MAV_FRAME_BODY_OFFSET_NED);
}

/********************************************************************************
 * Function: cmd_velocity_y_NED
 * Description: Move in direction of vector vy in the NED frame.
 ********************************************************************************/
void MavMotion::cmd_velocity_z_NED(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, float velocity_target)
{
    uint16_t options = 0;

    options |= POSITION_TARGET_TYPEMASK_X_IGNORE | POSITION_TARGET_TYPEMASK_Y_IGNORE | POSITION_TARGET_TYPEMASK_Z_IGNORE |
               POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE |
               POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
               POSITION_TARGET_TYPEMASK_YAW_IGNORE | POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;

    send_cmd_velocity_target(sender_sys_id, sender_comp_id, target_sys_id, target_comp_id,
                             0.0, 0.0, velocity_target, 0.0, options, MAV_FRAME_BODY_OFFSET_NED);
}

/********************************************************************************
 * Function: calc_yaw_target
 * Description: Calculate a yaw target based on the forward and lateral movement
 *              commands.
 ********************************************************************************/
float MavMotion::calc_yaw_target(float x, float y)
{
    return atan2(y, x);
}

/********************************************************************************
 * Function: calc_yaw_rate_target
 * Description: Calculate a yaw rate target based on the forward and lateral movement
 *              commands.
 ********************************************************************************/
float MavMotion::calc_yaw_rate_target(float x, float y)
{
    return atan2(y, x);
}

/********************************************************************************
 * Function: send_cmd_position_target
 * Description: This function accepts an x, y, z vector target and sends a
 *              request to move to move the drone to that desired position.
 ********************************************************************************/
void MavMotion::send_cmd_position_target(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_system, uint8_t target_component,
                                         float x, float y, float z, float yaw, uint16_t type_mask, uint8_t coordinate_frame)
{
    mavlink_message_t msg;
    mavlink_set_position_target_local_ned_t command_position_target;

    command_position_target.x = x;
    command_position_target.y = y;
    command_position_target.z = z;
    command_position_target.yaw = yaw;
    command_position_target.type_mask = type_mask;
    command_position_target.target_system = target_system;
    command_position_target.target_component = target_component;
    command_position_target.coordinate_frame = coordinate_frame;

    mavlink_msg_set_position_target_local_ned_encode(sender_sys_id, sender_comp_id, &msg, &command_position_target);
    send_mav_cmd(msg);
}

/********************************************************************************
 * Function: send_cmd_position_target
 * Description: This function accepts an x, y, z vector target and sends a
 *              request to move to move the drone to that desired position.
 ********************************************************************************/
void MavMotion::send_cmd_velocity_target(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_system, uint8_t target_component,
                                         float vx, float vy, float vz, float yaw, uint16_t type_mask, uint8_t coordinate_frame)
{
    mavlink_message_t msg;
    mavlink_set_position_target_local_ned_t command_position_target;

    command_position_target.vx = vx;
    command_position_target.vy = vy;
    command_position_target.vz = vz;
    command_position_target.yaw = yaw;
    command_position_target.type_mask = type_mask;
    command_position_target.target_system = target_system;
    command_position_target.target_component = target_component;
    command_position_target.coordinate_frame = coordinate_frame;

    mavlink_msg_set_position_target_local_ned_encode(sender_sys_id, sender_comp_id, &msg, &command_position_target);
    send_mav_cmd(msg);
}

/********************************************************************************
 * Function: send_cmd_position_target
 *
 * Description: This function accepts an x, y, z vector target and sends a
 *              request to move to move the drone to that desired position.
 ********************************************************************************/
void MavMotion::send_cmd_acccel_target(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_system, uint8_t target_component,
                                       float ax, float ay, float az, float yaw, float yaw_rate, uint16_t type_mask, uint8_t coordinate_frame)
{
    // send_cmd_set_position_target_local_ned(sender_sys_id, sender_comp_id, target_system, target_component, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ax, ay, az,
    //                                        yaw, yaw_rate, type_mask, coordinate_frame);
}

/********************************************************************************
 * Function: send_cmd_set_position_target_local_ned
 * Description: This function uses the mavlink message set position target to
 *              allow for manually controlling the position, velocity, or
 *              acceleration of a vehicle using an external controller.
 ********************************************************************************/
void MavMotion::send_cmd_set_position_target_local_ned(uint8_t sender_sys_id, uint8_t sender_comp_id,
                                                       uint8_t target_system, uint8_t target_component,
                                                       float x, float y, float z, float vx, float vy, float vz,
                                                       float afx, float afy, float afz, float yaw, float yaw_rate,
                                                       uint16_t type_mask, uint8_t coordinate_frame)
{
    mavlink_message_t msg;
    mavlink_set_position_target_local_ned_t command_position_target;

    command_position_target.x = x;
    command_position_target.y = y;
    command_position_target.z = z;
    command_position_target.vx = vx;
    command_position_target.vy = vy;
    command_position_target.vz = vz;
    command_position_target.afx = afx;
    command_position_target.afy = afy;
    command_position_target.afz = afz;
    command_position_target.yaw = yaw;
    command_position_target.yaw_rate = yaw_rate;
    command_position_target.type_mask = type_mask;
    command_position_target.target_system = target_system;
    command_position_target.target_component = target_component;
    command_position_target.coordinate_frame = coordinate_frame;

    mavlink_msg_set_position_target_local_ned_encode(sender_sys_id, sender_comp_id, &msg, &command_position_target);
    send_mav_cmd(msg);
}

/********************************************************************************
 * Function: go_to_waypoint
 * Description: Commmand vehicle to move to a specified GPS location with
 *              specific latitude, longtitude, and altitude.
 ********************************************************************************/
void MavMotion::go_to_waypoint(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, int32_t lat, int32_t lon, float alt)
{
    mavlink_set_position_target_global_int_t position_target_glob_int;

    position_target_glob_int.target_system = target_sys_id;
    position_target_glob_int.target_component = target_comp_id;
    position_target_glob_int.lat_int = lat;
    position_target_glob_int.lon_int = lon;
    position_target_glob_int.alt = alt;
    position_target_glob_int.coordinate_frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    position_target_glob_int.type_mask = (uint16_t)0b111111111000;

    // send_cmd_set_position_target_global_int (sender_sys_id, sender_comp_id, &position_target_glob_int); // mavlink_msg_command_int_pack(sender_sys_id, sender_comp_id, &msg, target_sys_id, target_comp_id, MAV_CMD_DO_SET_MODE, 0, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6, 0, 0, -35.3671 * 1e7, 149.1649 * 1e7, 0); is used for AUTO mode?
}

/********************************************************************************
 * Function: send_cmd_set_position_target_global_int
 * Description: This function uses the mavlink message set position target
 *              global int message to send a vehicle to a specific GPS location
 *              using an external controller.
 ********************************************************************************/
void MavMotion::send_cmd_set_position_target_global_int(uint8_t sender_sys_id, uint8_t sender_comp_id, const mavlink_set_position_target_global_int_t *set_position_target_global_int)
{
    mavlink_message_t msg;

    // There should be an _encode function we want to use instead of the pack function
    mavlink_msg_set_position_target_global_int_encode(sender_sys_id, sender_comp_id, &msg, set_position_target_global_int);
    send_mav_cmd(msg);
}
