/********************************************************************************
 * @file    mavlink_msg_handler.cpp
 * @author  Cameron Rose
 * @date    12/27/2023
 * @brief   Handles all incoming mavlink messages by setting the desired rate to
 *          receive each, and parsing the serial data to separate out
 *          specific messages.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "mavlink_msg_handler.h"

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/
int32_t mav_veh_lat = 0; /*< [degE7] Latitude, expressed*/
int32_t mav_veh_lon = 0; /*< [degE7] Longitude, expressed*/
int32_t mav_veh_alt = 0; /*< [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.*/
int32_t mav_rel_alt = 0; /*< [mm] Altitude above ground*/
int16_t mav_veh_gps_vx = 0; /*< [cm/s] Ground X Speed (Latitude, positive north)*/
int16_t mav_veh_gps_vy = 0; /*< [cm/s] Ground Y Speed (Longitude, positive east)*/
int16_t mav_veh_gps_vz = 0; /*< [cm/s] Ground Z Speed (Altitude, positive down)*/
uint16_t mav_veh_gps_hdg = 0; /*< [cdeg] Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
float mav_veh_roll = 0.0; /*< [rad] Roll angle (-pi..+pi)*/
float mav_veh_pitch = 0.0; /*< [rad] Pitch angle (-pi..+pi)*/
float mav_veh_yaw = 0.0; /*< [rad] Yaw angle (-pi..+pi)*/
float mav_veh_rollspeed = 0.0; /*< [rad/s] Roll angular speed*/
float mav_veh_pitchspeed = 0.0; /*< [rad/s] Pitch angular speed*/
float mav_veh_yawspeed = 0.0; /*< [rad/s] Yaw angular speed*/
int16_t mav_veh_imu_ax = 0; /*< [mG] X acceleration*/
int16_t mav_veh_imu_ay = 0; /*< [mG] Y acceleration*/
int16_t mav_veh_imu_az = 0; /*< [mG] Z acceleration*/
int16_t mav_veh_imu_xgyro = 0; /*< [mrad/s] Angular speed around X axis*/
int16_t mav_veh_imu_ygyro = 0; /*< [mrad/s] Angular speed around Y axis*/
int16_t mav_veh_imu_zgyro = 0; /*< [mrad/s] Angular speed around Z axis*/
int16_t mav_veh_imu_xmag = 0; /*< [mgauss] X Magnetic field*/
int16_t mav_veh_imu_ymag = 0; /*< [mgauss] Y Magnetic field*/
int16_t mav_veh_imu_zmag = 0; /*< [mgauss] Z Magnetic field*/
float q1_target = 0.0;
float q2_target = 0.0;
float q3_target = 0.0;
float q4_target = 0.0;
float roll_rate_target = 0.0;
float pitch_rate_target = 0.0;
float yaw_rate_target = 0.0;
float thrust_target = 0.0;
float q1_actual = 0.0;
float q2_actual = 0.0;
float q3_actual = 0.0;
float q4_actual = 0.0;
float roll_rate_actual = 0.0;
float pitch_rate_actual = 0.0;
float yaw_rate_actual = 0.0;
float thrust_actual = 0.0;
uint32_t time_since_boot_ms = 0;
uint64_t unix_timestamp_us = 0;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/

 /********************************************************************************
 * Function: MavMsg
 * Description: Class constructor
 ********************************************************************************/
MavMsg::MavMsg(){};

/********************************************************************************
 * Function: ~MavMsg
 * Description: Class destructor
 ********************************************************************************/
MavMsg::~MavMsg(){};

/********************************************************************************
 * Function: set_message_rates
 * Description: Tell the autopilot the frequency to send specific mavlink 
 *              messages.
 ********************************************************************************/
void set_message_rates(void)
{
    MavMsg::set_mav_msg_rate(MAVLINK_MSG_ID_HEARTBEAT, MESSAGE_RATE_DEFAULT);
    MavMsg::set_mav_msg_rate(MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 3125);
    MavMsg::set_mav_msg_rate(MAVLINK_MSG_ID_SYSTEM_TIME, MESSAGE_RATE_DEFAULT);
    MavMsg::set_mav_msg_rate(MAVLINK_MSG_ID_SCALED_IMU, MESSAGE_RATE_25000us);
    MavMsg::set_mav_msg_rate(MAVLINK_MSG_ID_ATTITUDE, MESSAGE_RATE_25000us);
    MavMsg::set_mav_msg_rate(MAVLINK_MSG_ID_ATTITUDE_TARGET, MESSAGE_RATE_25000us);
    MavMsg::set_mav_msg_rate(MAVLINK_MSG_ID_ATTITUDE_QUATERNION, MESSAGE_RATE_25000us);
    MavMsg::set_mav_msg_rate(MAVLINK_MSG_ID_AUTOPILOT_VERSION, MESSAGE_RATE_DEFAULT);
    MavMsg::set_mav_msg_rate(MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED, MESSAGE_RATE_25000us);
}

/********************************************************************************
 * Function: request_messages
 * Description: Tell the autopilot to begin sending specific messages, whose
 *              rate has already been set.
 ********************************************************************************/
void request_messages(void)
{
    MavMsg::req_mav_msg(MAVLINK_MSG_ID_HEARTBEAT);
    MavMsg::req_mav_msg(MAVLINK_MSG_ID_GLOBAL_POSITION_INT);
    MavMsg::req_mav_msg(MAVLINK_MSG_ID_SYSTEM_TIME);
    MavMsg::req_mav_msg(MAVLINK_MSG_ID_SCALED_IMU);
    MavMsg::req_mav_msg(MAVLINK_MSG_ID_ATTITUDE);
    MavMsg::req_mav_msg(MAVLINK_MSG_ID_ATTITUDE_TARGET);
    MavMsg::req_mav_msg(MAVLINK_MSG_ID_ATTITUDE_QUATERNION);
    MavMsg::req_mav_msg(MAVLINK_MSG_ID_AUTOPILOT_VERSION);
    MavMsg::req_mav_msg(MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED);
}

 /********************************************************************************
 * Function: proc_heartbeat
 * Description: Decode heartbeat message and pass along the desired information.
 ********************************************************************************/
 void MavMsg::proc_mav_heartbeat_msg(const mavlink_message_t *msg, bool print)
 {
    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(msg, &heartbeat);

    if (print)
    {
        print_heartbeat(heartbeat);
    }
 }

 /********************************************************************************
 * Function: proc_mav_gps_int
 * Description: Decode gps message and pass along the desired information.
 ********************************************************************************/
 void MavMsg::proc_mav_gps_int_msg(const mavlink_message_t *msg, bool print)
 {
    mavlink_global_position_int_t global_pos_int;
    mavlink_msg_global_position_int_decode(msg, &global_pos_int);

    mav_veh_lat = global_pos_int.lat;
    mav_veh_lon = global_pos_int.lon;
    mav_veh_alt = global_pos_int.alt;
    mav_rel_alt = global_pos_int.relative_alt;
    mav_veh_gps_vx  = global_pos_int.vx;
    mav_veh_gps_vy  = global_pos_int.vy;
    mav_veh_gps_vz  = global_pos_int.vz;
    mav_veh_gps_hdg = global_pos_int.hdg;
    
    if(print)
    {
        print_global_position_int(global_pos_int);
    }
    
 }

/********************************************************************************
 * Function: parse_mav_msgs
 * Description: Read the serial port and unpack specific messages if its 
 *              specific mavlink message ID has been received.
 ********************************************************************************/
void parse_mav_msgs(void)
{
    mavlink_message_t msg; // initialize the Mavlink message buffer
    mavlink_status_t status = {}; // Initialize the Mavlink status
    uint8_t byte;
    
    int n = SerialComm::bytes_available();
    for(int i = n; i > 0; i--)
    {
        byte = MavMsg::read_mav_msg();

        // Parse the byte and check if a message has been received
        if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) 
        {
            // Handle the message based on its type
            // switch case
            switch (msg.msgid) 
            {
                case MAVLINK_MSG_ID_HEARTBEAT:
                    MavMsg::proc_mav_heartbeat_msg(&msg,false);
                    break;
                case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                    MavMsg::proc_mav_gps_int_msg(&msg,true);
                    break;
                case MAVLINK_MSG_ID_SCALED_IMU:
                    mavlink_scaled_imu_t scaled_imu;
                    mavlink_msg_scaled_imu_decode(&msg, &scaled_imu);
                    mav_veh_imu_ax = scaled_imu.xacc;
                    mav_veh_imu_ay = scaled_imu.yacc;
                    mav_veh_imu_az = scaled_imu.zacc;
                    mav_veh_imu_xgyro = scaled_imu.xgyro;
                    mav_veh_imu_ygyro = scaled_imu.ygyro;
                    mav_veh_imu_zgyro = scaled_imu.zgyro;
                    mav_veh_imu_xmag = scaled_imu.xmag;
                    mav_veh_imu_ymag = scaled_imu.ymag;
                    mav_veh_imu_zmag = scaled_imu.zmag;
                    print_scaled_imu(scaled_imu);
                    break;
                case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
                    mavlink_set_position_target_local_ned_t set_position_target_local_ned;
                    mavlink_msg_set_position_target_local_ned_decode(&msg, &set_position_target_local_ned);
                    //print_set_position_target_local_ned(set_position_target_local_ned);
                    break;
                case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
                    mavlink_position_target_local_ned_t position_target_local_ned;
                    mavlink_msg_position_target_local_ned_decode(&msg, &position_target_local_ned);
                    //print_position_target_local_ned(position_target_local_ned);
                    break;
                case MAVLINK_MSG_ID_COMMAND_ACK:
                    mavlink_command_ack_t command_ack;
                    mavlink_msg_command_ack_decode(&msg, &command_ack);
                    //print_command_ack(command_ack);
                    break;
                case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
                    mavlink_param_request_read_t param_request_read;
                    mavlink_msg_param_request_read_decode(&msg, &param_request_read);
                    //print_param_request_read(param_request_read);
                    break;
                case MAVLINK_MSG_ID_STATUSTEXT:
                    mavlink_statustext_t statustext;
                    mavlink_msg_statustext_decode(&msg, &statustext);
                    //print_statustext(statustext);
                    break;
                case MAVLINK_MSG_ID_PARAM_VALUE:
                    mavlink_param_value_t param_value;
                    mavlink_msg_param_value_decode(&msg, &param_value);
                    //print_param_value(param_value);
                    break;
                default:
                    printf("Received message with ID: %d\n", (int)msg.msgid);
            }
        }
    }
}
