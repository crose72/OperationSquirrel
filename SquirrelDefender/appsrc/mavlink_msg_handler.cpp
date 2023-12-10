#include "mavlink_msg_handler.h"

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
int16_t xacc = 0; /*< [mG] X acceleration*/
int16_t yacc = 0; /*< [mG] Y acceleration*/
int16_t zacc = 0; /*< [mG] Z acceleration*/
int16_t xgyro = 0; /*< [mrad/s] Angular speed around X axis*/
int16_t ygyro = 0; /*< [mrad/s] Angular speed around Y axis*/
int16_t zgyro = 0; /*< [mrad/s] Angular speed around Z axis*/
int16_t xmag = 0; /*< [mgauss] X Magnetic field*/
int16_t ymag = 0; /*< [mgauss] Y Magnetic field*/
int16_t zmag = 0; /*< [mgauss] Z Magnetic field*/
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


void set_message_rates(void)
{
    send_command_long(MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_HEARTBEAT, MESSAGE_INTERVAL);
    send_command_long(MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_SYSTEM_TIME, MESSAGE_INTERVAL);
    send_command_long(MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_SCALED_IMU, MESSAGE_INTERVAL);
    send_command_long(MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_ATTITUDE, MESSAGE_INTERVAL);
    send_command_long(MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_ATTITUDE_TARGET, MESSAGE_INTERVAL);
    send_command_long(MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_ATTITUDE_QUATERNION, MESSAGE_INTERVAL);
    send_command_long(MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, MESSAGE_INTERVAL);
    send_command_long(MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_AUTOPILOT_VERSION, MESSAGE_INTERVAL);
}

void request_messages(void)
{
    send_command_long(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_HEARTBEAT);
    send_command_long(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_SYSTEM_TIME);
    send_command_long(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_SCALED_IMU);
    send_command_long(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_ATTITUDE);
    send_command_long(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_ATTITUDE_TARGET);
    send_command_long(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_ATTITUDE_QUATERNION);
    send_command_long(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_GLOBAL_POSITION_INT);
    send_command_long(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_AUTOPILOT_VERSION);
}

void parse_serial_data(void)
{
    uint16_t len = 0; // length of buffer
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN]; // define length of buffer
    mavlink_message_t msg; // initialize the Mavlink message buffer
    mavlink_status_t status = {}; // Initialize the Mavlink status

    uint8_t byte;
    
    byte = read_serial_port();

    // Parse the byte and check if a message has been received
    if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) 
    {
        // Handle the message based on its type
        // switch case
        switch (msg.msgid) 
        {
            case MAVLINK_MSG_ID_HEARTBEAT:
                mavlink_heartbeat_t heartbeat;
                mavlink_msg_heartbeat_decode(&msg, &heartbeat);
                //print_heartbeat(heartbeat);
                break;
            case MAVLINK_MSG_ID_SYSTEM_TIME:
                mavlink_system_time_t system_time;
                mavlink_msg_system_time_decode(&msg, &system_time);
                time_since_boot_ms = system_time.time_boot_ms;
                unix_timestamp_us = system_time.time_unix_usec;
                //print_system_time(system_time);
                break;
            case MAVLINK_MSG_ID_ATTITUDE:
                mavlink_attitude_t attitude;
                mavlink_msg_attitude_decode(&msg, &attitude);
                roll = attitude.roll;
                pitch = attitude.pitch;
                yaw = attitude.yaw;
                rollspeed = attitude.rollspeed;
                pitchspeed = attitude.pitchspeed;
                yawspeed = attitude.yawspeed;
                print_attitude(attitude);
                break;
            case MAVLINK_MSG_ID_SCALED_IMU:
                mavlink_scaled_imu_t scaled_imu;
                mavlink_msg_scaled_imu_decode(&msg, &scaled_imu);
                xacc = scaled_imu.xacc;
                yacc = scaled_imu.yacc;
                zacc = scaled_imu.zacc;
                xgyro = scaled_imu.xgyro;
                ygyro = scaled_imu.ygyro;
                zgyro = scaled_imu.zgyro;
                xmag = scaled_imu.xmag;
                ymag = scaled_imu.ymag;
                zmag = scaled_imu.zmag;
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
                //print_param_request_read(param_request_read);
                break;
            case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
                mavlink_request_data_stream_t request_data_stream;
                mavlink_msg_request_data_stream_decode(&msg, &request_data_stream);
                //print_request_data_stream(request_data_stream);
                break;
            case MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN:
                mavlink_gps_global_origin_t gps_global_origin;
                mavlink_msg_gps_global_origin_decode(&msg, &gps_global_origin);
                //print_gps_global_origin(gps_global_origin);
                break;
            case MAVLINK_MSG_ID_HOME_POSITION:
                mavlink_home_position_t home_position;
                mavlink_msg_home_position_decode(&msg, &home_position);
                //print_home_position(home_position);
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
            case MAVLINK_MSG_ID_RAW_IMU:
                mavlink_raw_imu_t raw_imu;
                mavlink_msg_raw_imu_decode(&msg, &raw_imu);
                //print_raw_imu(raw_imu);
                break;
            case MAVLINK_MSG_ID_GPS_RAW_INT:
                mavlink_gps_raw_int_t msg_gps_raw_int;
                mavlink_msg_gps_raw_int_decode(&msg, &msg_gps_raw_int);
                //print_gps_raw_int(msg_gps_raw_int);
                break;
            case MAVLINK_MSG_ID_AUTOPILOT_VERSION:
                mavlink_autopilot_version_t autopilot_version;
                mavlink_msg_autopilot_version_decode(&msg, &autopilot_version);
                //print_autopilot_version(autopilot_version);
                break;
            case MAVLINK_MSG_ID_ATTITUDE_TARGET:
                mavlink_attitude_target_t attitude_target;
                mavlink_msg_attitude_target_decode(&msg, &attitude_target);
                q1_target = attitude_target.q[0];
                q2_target = attitude_target.q[1];
                q3_target = attitude_target.q[2];
                q4_target = attitude_target.q[3];
                roll_rate_target = attitude_target.body_roll_rate;
                pitch_rate_target = attitude_target.body_pitch_rate;
                yaw_rate_target = attitude_target.body_yaw_rate;
                thrust_target = attitude_target.thrust;
                //print_attitude_target(attitude_target);
                break;
            case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
                mavlink_attitude_quaternion_t attitude_quaternion;
                mavlink_msg_attitude_quaternion_decode(&msg, &attitude_quaternion);
                q1_actual = attitude_quaternion.q1;
                q1_actual = attitude_quaternion.q2;
                q3_actual = attitude_quaternion.q3;
                q4_actual = attitude_quaternion.q4;
                roll_rate_actual = attitude_quaternion.rollspeed;
                pitch_rate_actual = attitude_quaternion.pitchspeed;
                yaw_rate_actual = attitude_quaternion.yawspeed;
                print_attitude_quaternion(attitude_quaternion);
                break;
            //case MAVLINK_MSG_ID_PROTOCOL_CAPABILITY: {
                //mavlink_protocol_version_t protocol_capability;
                //mavlink_msg_protocol_capability_decode(&msg, &protocol_capability);
                //print_protocol_capability(protocol_capability);
                //break;
            default:
                printf("Received message with ID: %d\n", (int)msg.msgid);
        }
    }
}
