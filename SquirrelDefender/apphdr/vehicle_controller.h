#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include "standard_libs.h"
#include "mavlink_command_handler.h"
#include "mavlink_msg_handler.h"

void move_to_position (float x_target, float y_target, float z_target);

#endif // VEHICLE_CONTROLLER_H
