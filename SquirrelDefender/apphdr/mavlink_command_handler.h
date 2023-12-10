#ifndef MAVLINK_COMMAND_HANDLER_H
#define MAVLINK_COMMAND_HANDLER_H

#include "standard_libs.h"
#include "serial_port_handler.h"

extern const uint8_t SENDER_SYS_ID;
extern const uint8_t SENDER_COMP_ID;
extern const uint8_t TARGET_SYS_ID;
extern const uint8_t TARGET_COMP_ID;
extern const int32_t MESSAGE_INTERVAL; // microseconds

void startup_sequence(void);
void landing_sequence(void);
void go_to_waypoint(int32_t lat, int32_t lon, float alt);

void send_command_long(uint16_t, uint16_t); // command, msg id
void send_command_long(uint16_t, uint16_t, float); //command, msg id, message rate
void send_command_long(uint16_t, uint8_t, float, float, float, float, float, float, float); // command, confirmation, param1 - param7
void send_cmd_set_position_target_global_int(uint8_t, uint16_t, int32_t, int32_t, float, float, float, float, float, float, float, float, float); // coordinate_frame, type_mask, lat_int, lon_int, alt, vx, vy, vz, afx, afy, afz, yaw, yaw_rate
void send_cmd_set_attitude_target(mavlink_set_attitude_target_t *); //  type_mask, *q, body_roll_rate, body_pitch_rate, body_yaw_rate, thrust, *thrust_body

#endif // MAVLINK_COMMAND_HANDLER_H