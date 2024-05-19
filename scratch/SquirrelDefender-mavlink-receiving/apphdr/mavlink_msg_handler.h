/********************************************************************************
 * @file    mavlink_msg_handler.h
 * @author  Cameron Rose
 * @date    12/27/2023
 ********************************************************************************/
#ifndef MAVLINK_MSG_HANDLER_H
#define MAVLINK_MSG_HANDLER_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "mavlink_print_info.h"
#include "serial_port_handler.h"
#include "mavlink_cmd_handler.h"

/********************************************************************************
 * Imported objects
 ********************************************************************************/
extern const uint8_t SENDER_SYS_ID;
extern const uint8_t SENDER_COMP_ID;
extern const uint8_t TARGET_SYS_ID;
extern const uint8_t TARGET_COMP_ID;
extern const int32_t MESSAGE_RATE_DEFAULT;
extern const int32_t MESSAGE_RATE_1000us;
extern const int32_t MESSAGE_RATE_25000us;
extern bool stopProgram;

extern int32_t lat;
extern int32_t lon;
extern int32_t alt;
extern int32_t relative_alt;
extern int16_t vx;
extern int16_t vy;
extern int16_t vz;
extern uint16_t hdg;
extern float roll;
extern float pitch;
extern float yaw;
extern float rollspeed;
extern float pitchspeed;
extern float yawspeed;
extern int16_t accel_x;
extern int16_t accel_y;
extern int16_t accel_z;
extern int16_t gyro_x;
extern int16_t gyro_y;
extern int16_t gyro_z;
extern int16_t mag_x;
extern int16_t mag_y;
extern int16_t mag_z;

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
extern uint32_t time_since_boot_ms;
extern uint64_t unix_timestamp_us;

/********************************************************************************
 * Exported objects
 ********************************************************************************/

/********************************************************************************
 * Function prototypes
 ********************************************************************************/
void set_message_rates(void);
void request_messages(void);
void parse_serial_data(void);

#endif // MAVLINK_MSG_HANDLER_H