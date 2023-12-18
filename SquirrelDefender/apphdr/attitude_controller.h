#ifndef ATTITUDE_CONTROLLER_H
#define ATTITUDE_CONTROLLER_H

#include "standard_libs.h"
#include "mavlink_command_handler.h"
#include "mavlink_msg_handler.h"

extern float q1_target;
extern float q2_target;
extern float q3_target;
extern float q4_target;
extern float roll_rate_target;
extern float pitch_rate_target;
extern float yaw_rate_target;
extern float thrust_target;
extern float q1_actual;
extern float q2_actual;
extern float q3_actual;
extern float q4_actual;
extern float roll_rate_actual;
extern float pitch_rate_actual;
extern float yaw_rate_actual;
extern float thrust_actual;

bool dtrmn_attitude_target_error(void);
void brake (void);
void hover (void);
void move_forward (void);
void turn_left_90 (void);

#endif // ATTITUDE_CONTROLLER_H
