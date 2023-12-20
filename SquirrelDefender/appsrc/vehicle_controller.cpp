#include "vehicle_controller.h"

//  uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
// float x; /*< [m] X Position in NED frame*/
// float y; /*< [m] Y Position in NED frame*/
// float z; /*< [m] Z Position in NED frame (note, altitude is negative in NED)*/
// float vx; /*< [m/s] X velocity in NED frame*/
// float vy; /*< [m/s] Y velocity in NED frame*/
// float vz; /*< [m/s] Z velocity in NED frame*/
// float afx; /*< [m/s/s] X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N*/
// float afy; /*< [m/s/s] Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N*/
// float afz; /*< [m/s/s] Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N*/
// float yaw; /*< [rad] yaw setpoint*/
// float yaw_rate; /*< [rad/s] yaw rate setpoint*/
// uint16_t type_mask; /*<  Bitmap to indicate which dimensions should be ignored by the vehicle.*/
// uint8_t target_system; /*<  System ID*/
// uint8_t target_component; /*<  Component ID*/
// uint8_t coordinate_frame; /*<
// parameters for set_attitude_target command

void cmd_position (float x_target, float y_target, float z_target)
{
    mavlink_set_position_target_local_ned_t desired_position_target;
    float yaw_target;

    yaw_target = calc_yaw(x_target, y_target);

    desired_position_target.x = x_target;
    desired_position_target.y = y_target;
    desired_position_target.z = z_target;
    desired_position_target.yaw = yaw_target;
    desired_position_target.type_mask = 0b0000101111111000; // Ignore velocity and accel and yaw rate
    desired_position_target.coordinate_frame = MAV_FRAME_BODY_OFFSET_NED; // MAV_FRAME_BODY_OFFSET_NED

    send_cmd_set_position_target_local_ned(&desired_position_target);
}

void cmd_velocity (float vx_target, float vy_target, float vz_target)
{
    mavlink_set_position_target_local_ned_t desired_position_target;
    float yaw_target;

    yaw_target = calc_yaw(vx_target, vy_target);

    desired_position_target.vx = vx_target;
    desired_position_target.vy = vy_target;
    desired_position_target.vz = vz_target;
    desired_position_target.yaw = yaw_target;
    desired_position_target.type_mask = 0b0000101111000111; // Ignore position and accel and yaw rate
    desired_position_target.coordinate_frame = MAV_FRAME_BODY_OFFSET_NED; // MAV_FRAME_BODY_OFFSET_NED

    send_cmd_set_position_target_local_ned(&desired_position_target);
}

void cmd_acceleration (float afx_target, float afy_target, float afz_target)
{
    mavlink_set_position_target_local_ned_t desired_position_target;
    float yaw_rate_target;

    yaw_rate_target = calc_yaw_rate(afx_target, afy_target);

    desired_position_target.afx = afx_target;
    desired_position_target.afy = afy_target;
    desired_position_target.afz = afz_target;
    desired_position_target.yaw_rate = yaw_rate_target;
    desired_position_target.type_mask = 0b0000010000111111; // Ignore position and vel and yaw
    desired_position_target.coordinate_frame = MAV_FRAME_BODY_OFFSET_NED; // MAV_FRAME_BODY_OFFSET_NED

    send_cmd_set_position_target_local_ned(&desired_position_target);
}

float calc_yaw (float x, float y)
{
    float yaw_target;
    
    yaw_target = atan(y/x);

    return yaw_target;
}

float calc_yaw_rate (float x, float y)
{
    float yaw_rate_target;
    
    yaw_rate_target = atan(y/x);

    return yaw_rate_target;
}
