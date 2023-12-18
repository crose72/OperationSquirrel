#include "attitude_controller.h"

mavlink_set_position_target_local_ned_t desired_position_target;

void move_to_position (float x_target, float y_target, float z_target)
{
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
    desired_position_target.x = x_target;
    desired_position_target.y = y_target;
    desired_position_target.z = z_target;
    desired_position_target.vx = (float)0.0;
    desired_position_target.vy = (float)0.0;
    desired_position_target.vz = (float)0.0;
    desired_position_target.coordinate_frame = MAV_FRAME_BODY_OFFSET_NED;

    send_cmd_set_position_target_local_ned(&desired_position_target);
}
