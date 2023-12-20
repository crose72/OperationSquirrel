#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include "standard_libs.h"
#include "mavlink_command_handler.h"
#include "mavlink_msg_handler.h"

void cmd_position (float x_target, float y_target, float z_target);
void cmd_velocity (float vx_target, float vy_target, float vz_target);
void cmd_acceleration (float afx_target, float afy_target, float afz_target);
float calc_yaw (float x, float y);
float calc_yaw_rate (float x, float y);

#endif // VEHICLE_CONTROLLER_H
