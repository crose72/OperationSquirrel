/********************************************************************************
 * @file    mav_motion_utils.h
 * @author  Cameron Rose
 * @date    1/22/2025
 ********************************************************************************/
#ifndef MAV_MOTION_H
#define MAV_MOTION_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "mav_serial.h"

/********************************************************************************
 * Typedefs / Enums / Structs
 ********************************************************************************/

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/
class MavMotion
{
    MavMotion();
    ~MavMotion();

public:
    static void cmd_position_NED(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, float position_target[3]);
    static void cmd_velocity_NED(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, float velocity_target[3], float yaw_target);
    static void cmd_velocity_xy_NED(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, float velocity_target[3]);
    static void cmd_velocity_x_NED(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, float velocity_target);
    static void cmd_velocity_y_NED(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, float velocity_target);
    static void cmd_velocity_z_NED(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, float velocity_target);
    static void send_cmd_position_target(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_system, uint8_t target_component,
                                         float x, float y, float z, float yaw, uint16_t type_mask, uint8_t coordinate_frame);
    static void send_cmd_velocity_target(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_system, uint8_t target_component,
                                         float vx, float vy, float vz, float yaw, uint16_t type_mask, uint8_t coordinate_frame);
    static void send_cmd_acccel_target(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_system, uint8_t target_component,
                                       float ax, float ay, float az, float yaw, float yaw_rate, uint16_t type_mask, uint8_t coordinate_frame);
    static void send_cmd_set_position_target_local_ned(uint8_t sender_sys_id, uint8_t sender_comp_id,
                                                       uint8_t target_system, uint8_t target_component,
                                                       float x, float y, float z, float vx, float vy, float vz,
                                                       float afx, float afy, float afz, float yaw, float yaw_rate,
                                                       uint16_t type_mask, uint8_t coordinate_frame);
    static void send_cmd_set_position_target_local_ned(mavlink_set_position_target_local_ned_t *desired_target);
    static void send_cmd_set_position_target_global_int(uint8_t sender_sys_id, uint8_t sender_comp_id, const mavlink_set_position_target_global_int_t *set_position_target_global_int); // coordinate_frame, type_mask, lat_int, lon_int, alt, vx, vy, vz, afx, afy, afz, yaw, yaw_rate
    static void send_cmd_set_attitude_target(
        uint8_t sender_sys_id,
        uint8_t sender_comp_id,
        uint8_t target_system,
        uint8_t target_component,
        uint8_t type_mask,
        const float q[4],
        float body_roll_rate,
        float body_pitch_rate,
        float body_yaw_rate,
        float thrust,
        const float thrust_body[3]);
    static void go_to_waypoint(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, int32_t lat, int32_t lon, float alt);

private:
    static void send_mav_cmd(mavlink_message_t &msg) { MavSerial::write_mav_msg(msg); };
    static float calc_yaw_target(float x, float y);
    static float calc_yaw_rate_target(float x, float y);
};

bool dtrmn_attitude_target_error(void);
void brake(void);
void move_forward(void);
void attitude_yaw(float yaw_pos, float yaw_rate);

#endif // MAV_MOTION_H
