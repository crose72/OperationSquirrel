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
#include "serial_comm.h"
#include "mavlink_cmd_handler.h"

/********************************************************************************
 * Imported objects
 ********************************************************************************/
extern const uint8_t SENDER_SYS_ID;
extern const uint8_t SENDER_COMP_ID;
extern const uint8_t TARGET_SYS_ID;
extern const uint8_t TARGET_COMP_ID;
extern const int32_t MESSAGE_RATE_DEFAULT;
extern const int32_t MESSAGE_RATE_1Hz;
extern const int32_t MESSAGE_RATE_40Hz;

extern int32_t mav_veh_lat;
extern int32_t mav_veh_lon;
extern int32_t mav_veh_alt;
extern int32_t mav_rel_alt;
extern int16_t mav_veh_gps_vx;
extern int16_t mav_veh_gps_vy;
extern int16_t mav_veh_gps_vz;
extern uint16_t mav_veh_gps_hdg;
extern float mav_veh_roll;
extern float mav_veh_pitch;
extern float mav_veh_yaw;
extern float mav_veh_rollspeed;
extern float mav_veh_pitchspeed;
extern float mav_veh_yawspeed;
extern int16_t mav_veh_imu_ax;
extern int16_t mav_veh_imu_ay;
extern int16_t mav_veh_imu_az;
extern int16_t mav_veh_imu_xgyro;
extern int16_t mav_veh_imu_ygyro;
extern int16_t mav_veh_imu_zgyro;
extern int16_t mav_veh_imu_xmag;
extern int16_t mav_veh_imu_ymag;
extern int16_t mav_veh_imu_zmag;

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

 class MavMsg
 {
     public:
        MavMsg();
        ~MavMsg();

        static void set_mav_msg_rate(uint16_t msg_id, float msg_interval){MavCmd::set_mav_msg_rate(msg_id, msg_interval);};
        static void req_mav_msg(uint16_t msg_id){MavCmd::req_mav_msg(msg_id);};
        static uint8_t read_mav_msg(void){SerialComm::read_uart();};

        static void proc_mav_heartbeat_msg(const mavlink_message_t *msg, bool print = false);
        static void proc_mav_gps_int_msg(const mavlink_message_t *msg, bool print = false);
        static void proc_mav_system_time_msg(const mavlink_message_t *msg, bool print = false);
        static void proc_mav_sys_status_msg(const mavlink_message_t *msg, bool print = false);
        static void proc_mav_statustext_msg(const mavlink_message_t *msg, bool print = false);
        static void proc_mav_param_value_msg(const mavlink_message_t *msg, bool print = false);
        static void proc_mav_autopilot_version_msg(const mavlink_message_t *msg, bool print = false);
        static void proc_mav_scaled_imu_msg(const mavlink_message_t *msg, bool print = false);
        static void proc_mav_local_position_msg(const mavlink_message_t *msg, bool print = false);
        static void proc_mav_position_target_local_ned_msg(const mavlink_message_t *msg, bool print = false);
        static void proc_mav_set_position_target_local_ned_msg(const mavlink_message_t *msg, bool print = false);
        static void proc_mav_attitude_msg(const mavlink_message_t *msg, bool print = false);
        static void proc_mav_attitude_target_msg(const mavlink_message_t *msg, bool print = false);
        static void proc_mav_set_attitude_target_msg(const mavlink_message_t *msg, bool print = false);
        static void proc_mav_attitude_quaternion_msg(const mavlink_message_t *msg, bool print = false);
        static void proc_mav_command_ack_msg(const mavlink_message_t *msg, bool print = false);

     private:

 };

void set_message_rates(void);
void request_messages(void);
void parse_mav_msgs(void);

#endif // MAVLINK_MSG_HANDLER_H