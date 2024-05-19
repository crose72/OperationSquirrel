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
int32_t lat = 0; /*< [degE7] Latitude, expressed*/
int32_t lon = 0; /*< [degE7] Longitude, expressed*/
int32_t alt = 0; /*< [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.*/
int32_t relative_alt = 0; /*< [mm] Altitude above ground*/
int16_t vx = 0; /*< [cm/s] Ground X Speed (Latitude, positive north)*/
int16_t vy = 0; /*< [cm/s] Ground Y Speed (Longitude, positive east)*/
int16_t vz = 0; /*< [cm/s] Ground Z Speed (Altitude, positive down)*/
uint16_t hdg = 0; /*< [cdeg] Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
float roll = 0.0; /*< [rad] Roll angle (-pi..+pi)*/
float pitch = 0.0; /*< [rad] Pitch angle (-pi..+pi)*/
float yaw = 0.0; /*< [rad] Yaw angle (-pi..+pi)*/
float rollspeed = 0.0; /*< [rad/s] Roll angular speed*/
float pitchspeed = 0.0; /*< [rad/s] Pitch angular speed*/
float yawspeed = 0.0; /*< [rad/s] Yaw angular speed*/
int16_t accel_x = 0; /*< [mG] X acceleration*/
int16_t accel_y = 0; /*< [mG] Y acceleration*/
int16_t accel_z = 0; /*< [mG] Z acceleration*/
int16_t gyro_x = 0; /*< [mrad/s] Angular speed around X axis*/
int16_t gyro_y = 0; /*< [mrad/s] Angular speed around Y axis*/
int16_t gyro_z = 0; /*< [mrad/s] Angular speed around Z axis*/
int16_t mag_x = 0; /*< [mgauss] X Magnetic field*/
int16_t mag_y = 0; /*< [mgauss] Y Magnetic field*/
int16_t mag_z = 0; /*< [mgauss] Z Magnetic field*/
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
 * Function: set_message_rates
 * Description: Tell the autopilot the frequency to send specific mavlink 
 *              messages.
 ********************************************************************************/
void set_message_rates(void)
{
    set_mav_msg_rate(MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_SCALED_IMU, MESSAGE_RATE_25000us);
    set_mav_msg_rate(MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, MESSAGE_RATE_25000us);
}

/********************************************************************************
 * Function: request_messages
 * Description: Tell the autopilot to begin sending specific messages, whose
 *              rate has already been set.
 ********************************************************************************/
void request_messages(void)
{
    req_mav_msg(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_SCALED_IMU);
    req_mav_msg(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_GLOBAL_POSITION_INT);
}

/********************************************************************************
 * Function: parse_serial_data
 * Description: Read the serial port and unpack specific messages if its 
 *              specific mavlink message ID has been received.
 ********************************************************************************/
void parse_serial_data(void)
{
    uint16_t len = 0; // length of buffer
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN]; // define length of buffer
    mavlink_message_t msg; // initialize the Mavlink message buffer
    mavlink_status_t status = {}; // Initialize the Mavlink status

    uint8_t byte;

    //printf("before while: %d\n", bytes_available());
    while(bytes_available() > 0)
    {
        //printf("inside while\n");
        byte = read_serial_port();
        
        // Parse the byte and check if a message has been received
        if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) 
        {
            // Handle the message based on its type
            // switch case
            switch (msg.msgid) 
            {
                case MAVLINK_MSG_ID_SCALED_IMU:
                    mavlink_scaled_imu_t scaled_imu;
                    mavlink_msg_scaled_imu_decode(&msg, &scaled_imu);
                    accel_x = scaled_imu.xacc;
                    accel_y = scaled_imu.yacc;
                    accel_z = scaled_imu.zacc;
                    gyro_x = scaled_imu.xgyro;
                    gyro_y = scaled_imu.ygyro;
                    gyro_z = scaled_imu.zgyro;
                    mag_x = scaled_imu.xmag;
                    mag_y = scaled_imu.ymag;
                    mag_z = scaled_imu.zmag;
                    print_scaled_imu(scaled_imu);
                    break;
                case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                    mavlink_global_position_int_t global_pos_int;
                    mavlink_msg_global_position_int_decode(&msg, &global_pos_int);
                    lat = global_pos_int.lat;
                    lon = global_pos_int.lon;
                    alt = global_pos_int.alt;
                    relative_alt = global_pos_int.relative_alt;
                    vx  = global_pos_int.vx;
                    vy  = global_pos_int.vy;
                    vz  = global_pos_int.vz;
                    hdg = global_pos_int.hdg;
                    print_global_position_int(global_pos_int);
                    break;
                case MAVLINK_MSG_ID_COMMAND_ACK:
                    mavlink_command_ack_t command_ack;
                    mavlink_msg_command_ack_decode(&msg, &command_ack);
                    print_command_ack(command_ack);
                    break;
                case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
                    mavlink_param_request_read_t param_request_read;
                    mavlink_msg_param_request_read_decode(&msg, &param_request_read);
                    print_param_request_read(param_request_read);
                case MAVLINK_MSG_ID_PARAM_VALUE:
                    mavlink_param_value_t param_value;
                    mavlink_msg_param_value_decode(&msg, &param_value);
                    print_param_value(param_value);
                    break;
                case MAVLINK_MSG_ID_STATUSTEXT:
                    mavlink_statustext_t statustext;
                    mavlink_msg_statustext_decode(&msg, &statustext);
                    print_statustext(statustext);
                    break;
                default:
                    printf("Received message with ID: %d\n", (int)msg.msgid);
            }
        }
    }
    
}
