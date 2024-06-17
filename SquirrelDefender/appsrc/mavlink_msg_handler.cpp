/********************************************************************************
 * @file    mavlink_msg_handler.cpp
 * @author  Cameron Rose
 * @date    6/7/2023
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
DebugTerm MavCmdAck("/dev/pts/3");
DebugTerm SysStatInfo("/dev/pts/3");
DebugTerm HeartbeatInfo("/dev/pts/3");

uint16_t mav_veh_command_id = 0;              /*<  Command ID (of acknowledged command).*/
uint8_t mav_veh_command_result = 0;           /*<  Result of command.*/
uint8_t mav_veh_command_progress = 0;         /*< [%] The progress percentage when result is MAV_RESULT_IN_PROGRESS. Values: [0-100],
                                            or UINT8_MAX if the progress is unknown.*/
int32_t mav_veh_command_result_param2 = 0;    /*<  Additional result information. Can be set with a command-specific enum containing
                                           command-specific error reasons for why the command might be denied. If used, the associated
                                           enum must be documented in the corresponding MAV_CMD (this enum should have a 0 value to indicate
                                           "unused" or "unknown").*/
uint8_t mav_veh_command_target_system = 0;    /*<  System ID of the target recipient. This is the ID of the system that sent the command for
                                               which this COMMAND_ACK is an acknowledgement.*/
uint8_t mav_veh_command_target_component = 0; /*<  Component ID of the target recipient. This is the ID of the system that sent the command for
                                            which this COMMAND_ACK is an acknowledgement.*/

uint32_t mav_veh_sys_stat_onbrd_cntrl_snsrs_present = 0;      /*<  Bitmap showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present.*/
uint32_t mav_veh_sys_stat_onbrd_cntrl_snsrs_enabled = 0;      /*<  Bitmap showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled.*/
uint32_t mav_veh_sys_stat_onbrd_cntrl_snsrs_health = 0;       /*<  Bitmap showing which onboard controllers and sensors have an error (or are operational). Value of 0: error.
                                                              Value of 1: healthy.*/
uint16_t mav_veh_sys_stat_load = 0;                           /*< [d%] Maximum usage in percent of the mainloop time. Values: [0-1000] - should always be below 1000*/
uint16_t mav_veh_sys_stat_voltage_battery = 0;                /*< [mV] Battery voltage, UINT16_MAX: Voltage not sent by autopilot*/
int16_t mav_veh_sys_stat_current_battery = 0;                 /*< [cA] Battery current, -1: Current not sent by autopilot*/
uint16_t mav_veh_sys_stat_drop_rate_comm = 0;                 /*< [c%] Communication drop rate, (UART, I2C, SPI, CAN), dropped packets on all links
                                                            (packets that were corrupted on reception on the MAV)*/
uint16_t mav_veh_sys_stat_errors_comm = 0;                    /*<  Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)*/
uint16_t mav_veh_sys_stat_errors_count1 = 0;                  /*<  Autopilot-specific errors*/
uint16_t mav_veh_sys_stat_errors_count2 = 0;                  /*<  Autopilot-specific errors*/
uint16_t mav_veh_sys_stat_errors_count3 = 0;                  /*<  Autopilot-specific errors*/
uint16_t mav_veh_sys_stat_errors_count4 = 0;                  /*<  Autopilot-specific errors*/
int8_t mav_veh_sys_stat_battery_remaining = 0;                /*< [%] Battery energy remaining, -1: Battery remaining energy not sent by autopilot*/
uint32_t mav_veh_sys_stat_onbrd_cntrl_snsrs_prsnt_extnd = 0;  /*<  Bitmap showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present.*/
uint32_t mav_veh_sys_stat_onbrd_cntrl_snsrs_enbld_extnd = 0;  /*<  Bitmap showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled.*/
uint32_t mav_veh_sys_stat_onbrd_cntrl_snsrs_health_extnd = 0; /*<  Bitmap showing which onboard controllers and sensors have an error (or are operational). Value of 0: error.
                                                            Value of 1: healthy.*/
uint32_t mav_veh_custom_mode = 0;                             /*<  A bitfield for use for autopilot-specific flags*/
uint8_t mav_veh_type = 0;                                     /*<  Vehicle or component type. For a flight controller component the vehicle type (quadrotor, helicopter, etc.). For other components the
                                                            component type (e.g. camera, gimbal, etc.). This should be used in preference to component id for identifying the component type.*/
uint8_t mav_veh_autopilot_type = 0;                           /*<  Autopilot type / class. Use MAV_AUTOPILOT_INVALID for components that are not flight controllers.*/
uint8_t mav_veh_base_mode = 0;                                /*<  System mode bitmap.*/
uint8_t mav_veh_state = 0;                                    /*<  System status flag.*/
uint8_t mav_veh_mavlink_version = 0;                          /*<  MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version*/

int32_t mav_veh_lat = 0;        /*< [degE7] Latitude, expressed*/
int32_t mav_veh_lon = 0;        /*< [degE7] Longitude, expressed*/
int32_t mav_veh_alt = 0;        /*< [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.*/
int32_t mav_veh_rel_alt = 0;    /*< [mm] Altitude above ground*/
int16_t mav_veh_gps_vx = 0;     /*< [cm/s] Ground X Speed (Latitude, positive north)*/
int16_t mav_veh_gps_vy = 0;     /*< [cm/s] Ground Y Speed (Longitude, positive east)*/
int16_t mav_veh_gps_vz = 0;     /*< [cm/s] Ground Z Speed (Altitude, positive down)*/
uint16_t mav_veh_gps_hdg = 0;   /*< [cdeg] Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
float mav_veh_roll = 0.0;       /*< [rad] Roll angle (-pi..+pi)*/
float mav_veh_pitch = 0.0;      /*< [rad] Pitch angle (-pi..+pi)*/
float mav_veh_yaw = 0.0;        /*< [rad] Yaw angle (-pi..+pi)*/
float mav_veh_rollspeed = 0.0;  /*< [rad/s] Roll angular speed*/
float mav_veh_pitchspeed = 0.0; /*< [rad/s] Pitch angular speed*/
float mav_veh_yawspeed = 0.0;   /*< [rad/s] Yaw angular speed*/
int16_t mav_veh_imu_ax = 0;     /*< [mG] X acceleration*/
int16_t mav_veh_imu_ay = 0;     /*< [mG] Y acceleration*/
int16_t mav_veh_imu_az = 0;     /*< [mG] Z acceleration*/
int16_t mav_veh_imu_xgyro = 0;  /*< [mrad/s] Angular speed around X axis*/
int16_t mav_veh_imu_ygyro = 0;  /*< [mrad/s] Angular speed around Y axis*/
int16_t mav_veh_imu_zgyro = 0;  /*< [mrad/s] Angular speed around Z axis*/
int16_t mav_veh_imu_xmag = 0;   /*< [mgauss] X Magnetic field*/
int16_t mav_veh_imu_ymag = 0;   /*< [mgauss] Y Magnetic field*/
int16_t mav_veh_imu_zmag = 0;   /*< [mgauss] Z Magnetic field*/
float mav_veh_q1_target = 0.0;
float mav_veh_q2_target = 0.0;
float mav_veh_q3_target = 0.0;
float mav_veh_q4_target = 0.0;
float mav_veh_roll_rate_target = 0.0;
float mav_veh_pitch_rate_target = 0.0;
float mav_veh_yaw_rate_target = 0.0;
float thrust_target = 0.0;
float mav_veh_q1 = 0.0;
float mav_veh_q2 = 0.0;
float mav_veh_q3 = 0.0;
float mav_veh_q4 = 0.0;
float mav_veh_roll_rate = 0.0;
float mav_veh_pitch_rate = 0.0;
float mav_veh_yaw_rate = 0.0;
float mav_veh_thrust = 0.0;

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
 * Function: subscribe
 * Description: Subscribe to mavlink messages and specify
 *              the desired to receive the message at.
 ********************************************************************************/
void MavMsg::subscribe(uint16_t msg_id, float msg_interval)
{
    set_mav_msg_rate(msg_id, msg_interval);
    req_mav_msg(msg_id);
}

/********************************************************************************
 * Function: proc_heartbeat
 * Description: Decode heartbeat message and pass along the desired information.
 ********************************************************************************/
void MavMsg::proc_mav_heartbeat_msg(const mavlink_message_t *msg, bool print)
{
    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(msg, &heartbeat);

    mav_veh_type = heartbeat.type;
    mav_veh_autopilot_type = heartbeat.autopilot;
    mav_veh_base_mode = heartbeat.base_mode;
    mav_veh_custom_mode = heartbeat.custom_mode;
    mav_veh_state = heartbeat.system_status;
    mav_veh_mavlink_version = heartbeat.mavlink_version;

    HeartbeatInfo.cpp_cout("Heartbeat type:" + std::to_string(mav_veh_type));
    HeartbeatInfo.cpp_cout("Autopilot type:" + std::to_string(mav_veh_autopilot_type));
    HeartbeatInfo.cpp_cout("Base mode:" + std::to_string(mav_veh_base_mode));
    HeartbeatInfo.cpp_cout("Custom mode:" + std::to_string(mav_veh_custom_mode));
    HeartbeatInfo.cpp_cout("Mav state:" + std::to_string(mav_veh_state));
    HeartbeatInfo.cpp_cout("Mavlink version:" + std::to_string(mav_veh_mavlink_version));

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
    mav_veh_rel_alt = global_pos_int.relative_alt;
    mav_veh_gps_vx = global_pos_int.vx;
    mav_veh_gps_vy = global_pos_int.vy;
    mav_veh_gps_vz = global_pos_int.vz;
    mav_veh_gps_hdg = global_pos_int.hdg;

    if (print)
    {
        print_global_position_int(global_pos_int);
    }
}

/********************************************************************************
 * Function: proc_mav_system_time_msg
 * Description: Decode system time message and optionally print the information.
 ********************************************************************************/
void MavMsg::proc_mav_system_time_msg(const mavlink_message_t *msg, bool print)
{
    mavlink_system_time_t system_time;
    mavlink_msg_system_time_decode(msg, &system_time);

    if (print)
    {
        print_system_time(system_time);
    }
}

/********************************************************************************
 * Function: proc_mav_sys_status_msg
 * Description: Decode system status message and optionally print the information.
 ********************************************************************************/
void MavMsg::proc_mav_sys_status_msg(const mavlink_message_t *msg, bool print)
{
    mavlink_sys_status_t sys_status;
    mavlink_msg_sys_status_decode(msg, &sys_status);

    mav_veh_sys_stat_onbrd_cntrl_snsrs_present = sys_status.onboard_control_sensors_present;
    mav_veh_sys_stat_onbrd_cntrl_snsrs_enabled = sys_status.onboard_control_sensors_enabled;
    mav_veh_sys_stat_onbrd_cntrl_snsrs_health = sys_status.onboard_control_sensors_health;
    mav_veh_sys_stat_load = sys_status.load;
    mav_veh_sys_stat_voltage_battery = sys_status.voltage_battery;
    mav_veh_sys_stat_current_battery = sys_status.current_battery;
    mav_veh_sys_stat_drop_rate_comm = sys_status.drop_rate_comm;
    mav_veh_sys_stat_errors_comm = sys_status.errors_comm;
    mav_veh_sys_stat_errors_count1 = sys_status.errors_count1;
    mav_veh_sys_stat_errors_count2 = sys_status.errors_count2;
    mav_veh_sys_stat_errors_count3 = sys_status.errors_count3;
    mav_veh_sys_stat_errors_count4 = sys_status.errors_count4;
    mav_veh_sys_stat_battery_remaining = sys_status.battery_remaining;
    mav_veh_sys_stat_onbrd_cntrl_snsrs_prsnt_extnd = sys_status.onboard_control_sensors_present_extended;
    mav_veh_sys_stat_onbrd_cntrl_snsrs_enbld_extnd = sys_status.onboard_control_sensors_enabled_extended;
    mav_veh_sys_stat_onbrd_cntrl_snsrs_health_extnd = sys_status.onboard_control_sensors_health_extended;

    SysStatInfo.cpp_cout("Control sensors present:" + std::to_string(mav_veh_sys_stat_onbrd_cntrl_snsrs_present));
    SysStatInfo.cpp_cout("Control sensors enabled:\t" + std::to_string(mav_veh_sys_stat_onbrd_cntrl_snsrs_enabled));
    SysStatInfo.cpp_cout("Control sensors health:\t" + std::to_string(mav_veh_sys_stat_onbrd_cntrl_snsrs_health));
    SysStatInfo.cpp_cout("Load:\t" + std::to_string(mav_veh_sys_stat_load));
    SysStatInfo.cpp_cout("Batt V:\t" + std::to_string(mav_veh_sys_stat_voltage_battery));
    SysStatInfo.cpp_cout("Batt I:\t" + std::to_string(mav_veh_sys_stat_current_battery));
    SysStatInfo.cpp_cout("Comm drop rate: " + std::to_string(mav_veh_sys_stat_drop_rate_comm));
    SysStatInfo.cpp_cout("Comm errors:\t" + std::to_string(mav_veh_sys_stat_errors_comm));
    SysStatInfo.cpp_cout("Errors count 1:\t" + std::to_string(mav_veh_sys_stat_errors_count1));
    SysStatInfo.cpp_cout("Errors count 2:\t" + std::to_string(mav_veh_sys_stat_errors_count2));
    SysStatInfo.cpp_cout("Errors count 3:\t" + std::to_string(mav_veh_sys_stat_errors_count3));
    SysStatInfo.cpp_cout("Errors count 4:\t" + std::to_string(mav_veh_sys_stat_errors_count4));
    SysStatInfo.cpp_cout("Batt remaining:\t" + std::to_string(mav_veh_sys_stat_battery_remaining));
    SysStatInfo.cpp_cout("Sensors present extended:\t" + std::to_string(mav_veh_sys_stat_onbrd_cntrl_snsrs_prsnt_extnd));
    SysStatInfo.cpp_cout("Sensors enabled extended:\t" + std::to_string(mav_veh_sys_stat_onbrd_cntrl_snsrs_enbld_extnd));
    SysStatInfo.cpp_cout("Sensors health extended:\t" + std::to_string(mav_veh_sys_stat_onbrd_cntrl_snsrs_health_extnd));

    SysStatInfo.cpp_cout("PRE ARM GOOD:" + std::to_string(mav_veh_sys_stat_onbrd_cntrl_snsrs_present & MAV_SYS_STATUS_PREARM_CHECK));

    if (print)
    {
        print_sys_status(sys_status);
    }
}

/********************************************************************************
 * Function: proc_mav_statustext_msg
 * Description: Decode status text message and optionally print the information.
 ********************************************************************************/
void MavMsg::proc_mav_statustext_msg(const mavlink_message_t *msg, bool print)
{
    mavlink_statustext_t statustext;
    mavlink_msg_statustext_decode(msg, &statustext);

    if (print)
    {
        print_statustext(statustext);
    }
}

/********************************************************************************
 * Function: proc_mav_param_value_msg
 * Description: Decode parameter value message and optionally print the information.
 ********************************************************************************/
void MavMsg::proc_mav_param_value_msg(const mavlink_message_t *msg, bool print)
{
    mavlink_param_value_t param_value;
    mavlink_msg_param_value_decode(msg, &param_value);

    if (print)
    {
        print_param_value(param_value);
    }
}

/********************************************************************************
 * Function: proc_mav_autopilot_version_msg
 * Description: Decode autopilot version message and optionally print the information.
 ********************************************************************************/
void MavMsg::proc_mav_autopilot_version_msg(const mavlink_message_t *msg, bool print)
{
    mavlink_autopilot_version_t autopilot_version;
    mavlink_msg_autopilot_version_decode(msg, &autopilot_version);

    if (print)
    {
        print_autopilot_version(autopilot_version);
    }
}

/********************************************************************************
 * Function: proc_mav_scaled_imu_msg
 * Description: Decode scaled IMU message and optionally print the information.
 ********************************************************************************/
void MavMsg::proc_mav_scaled_imu_msg(const mavlink_message_t *msg, bool print)
{
    mavlink_scaled_imu_t scaled_imu;
    mavlink_msg_scaled_imu_decode(msg, &scaled_imu);

    mav_veh_imu_ax = scaled_imu.xacc;
    mav_veh_imu_ay = scaled_imu.yacc;
    mav_veh_imu_az = scaled_imu.zacc;
    mav_veh_imu_xgyro = scaled_imu.xgyro;
    mav_veh_imu_ygyro = scaled_imu.ygyro;
    mav_veh_imu_zgyro = scaled_imu.zgyro;
    mav_veh_imu_xmag = scaled_imu.xmag;
    mav_veh_imu_ymag = scaled_imu.ymag;
    mav_veh_imu_zmag = scaled_imu.zmag;

    if (print)
    {
        print_scaled_imu(scaled_imu);
    }
}

/********************************************************************************
 * Function: proc_mav_local_position_msg
 * Description: Decode local position message and optionally print the information.
 ********************************************************************************/
void MavMsg::proc_mav_local_position_msg(const mavlink_message_t *msg, bool print)
{
    mavlink_local_position_ned_t local_position;
    mavlink_msg_local_position_ned_decode(msg, &local_position);

    if (print)
    {
        print_local_position(local_position);
    }
}

/********************************************************************************
 * Function: proc_mav_position_target_local_ned_msg
 * Description: Decode position target local NED message and optionally print the information.
 ********************************************************************************/
void MavMsg::proc_mav_position_target_local_ned_msg(const mavlink_message_t *msg, bool print)
{
    mavlink_position_target_local_ned_t position_target_local_ned;
    mavlink_msg_position_target_local_ned_decode(msg, &position_target_local_ned);

    if (print)
    {
        print_position_target_local_ned(position_target_local_ned);
    }
}

/********************************************************************************
 * Function: proc_mav_set_position_target_local_ned_msg
 * Description: Decode set position target local NED message and optionally print the information.
 ********************************************************************************/
void MavMsg::proc_mav_set_position_target_local_ned_msg(const mavlink_message_t *msg, bool print)
{
    mavlink_set_position_target_local_ned_t set_position_target_local_ned;
    mavlink_msg_set_position_target_local_ned_decode(msg, &set_position_target_local_ned);

    if (print)
    {
        print_set_position_target_local_ned(set_position_target_local_ned);
    }
}

/********************************************************************************
 * Function: proc_mav_attitude_msg
 * Description: Decode attitude message and optionally print the information.
 ********************************************************************************/
void MavMsg::proc_mav_attitude_msg(const mavlink_message_t *msg, bool print)
{
    mavlink_attitude_t attitude;
    mavlink_msg_attitude_decode(msg, &attitude);

    if (print)
    {
        print_attitude(attitude);
    }
}

/********************************************************************************
 * Function: proc_mav_attitude_target_msg
 * Description: Decode attitude target message and optionally print the information.
 ********************************************************************************/
void MavMsg::proc_mav_attitude_target_msg(const mavlink_message_t *msg, bool print)
{
    mavlink_attitude_target_t attitude_target;
    mavlink_msg_attitude_target_decode(msg, &attitude_target);

    if (print)
    {
        print_attitude_target(attitude_target);
    }
}

/********************************************************************************
 * Function: proc_mav_set_attitude_target_msg
 * Description: Decode set attitude target message and optionally print the information.
 ********************************************************************************/
void MavMsg::proc_mav_set_attitude_target_msg(const mavlink_message_t *msg, bool print)
{
    mavlink_set_attitude_target_t set_attitude_target;
    mavlink_msg_set_attitude_target_decode(msg, &set_attitude_target);

    if (print)
    {
        print_set_attitude_target(set_attitude_target);
    }
}

/********************************************************************************
 * Function: proc_mav_attitude_quaternion_msg
 * Description: Decode attitude quaternion message and optionally print the information.
 ********************************************************************************/
void MavMsg::proc_mav_attitude_quaternion_msg(const mavlink_message_t *msg, bool print)
{
    mavlink_attitude_quaternion_t attitude_quaternion;
    mavlink_msg_attitude_quaternion_decode(msg, &attitude_quaternion);

    if (print)
    {
        print_attitude_quaternion(attitude_quaternion);
    }
}

/********************************************************************************
 * Function: proc_mav_command_ack_msg
 * Description: Decode command acknowledgment message and optionally print the information.
 ********************************************************************************/
void MavMsg::proc_mav_command_ack_msg(const mavlink_message_t *msg, bool print)
{
    mavlink_command_ack_t command_ack;
    mavlink_msg_command_ack_decode(msg, &command_ack);

    mav_veh_command_id = command_ack.command;
    mav_veh_command_result = command_ack.result;
    mav_veh_command_progress = command_ack.progress;
    mav_veh_command_result_param2 = command_ack.result_param2;
    mav_veh_command_target_system = command_ack.target_system;
    mav_veh_command_target_component = command_ack.target_component;
    /*
    MavCmdAck.cpp_cout("Command ID: " + std::to_string(mav_veh_command_id));
    MavCmdAck.cpp_cout("Result:\t" + std::to_string(mav_veh_command_result));
    MavCmdAck.cpp_cout("Progress:\t" + std::to_string(mav_veh_command_progress));
    MavCmdAck.cpp_cout("Result param 2:\t" + std::to_string(mav_veh_command_result_param2));
    MavCmdAck.cpp_cout("Target sys:\t" + std::to_string(mav_veh_command_target_system));
    MavCmdAck.cpp_cout("Target comp:\t" + std::to_string(mav_veh_command_target_component));
    */
    if (print)
    {
        print_command_ack(command_ack);
    }
}

/********************************************************************************
 * Function: proc_mav_optical_flow_msg
 * Description: Decode optical flow message and optionally print the information.
 ********************************************************************************/
void MavMsg::proc_mav_optical_flow_msg(const mavlink_message_t *msg, bool print)
{
    mavlink_optical_flow_t optical_flow;
    mavlink_msg_optical_flow_decode(msg, &optical_flow);

    if (print)
    {
        print_optical_flow(optical_flow);
    }
}

/********************************************************************************
 * Function: proc_mav_distance_sensor_msg
 * Description: Decode distance sensor message and optionally print the information.
 ********************************************************************************/
void MavMsg::proc_mav_distance_sensor_msg(const mavlink_message_t *msg, bool print)
{
    mavlink_distance_sensor_t distance_sensor;
    mavlink_msg_distance_sensor_decode(msg, &distance_sensor);

    if (print)
    {
        print_distance_sensor(distance_sensor);
    }
}

/********************************************************************************
 * Function: start_message_subscriptions
 * Description: Handle all message subscriptions.  Any messages subscribed to
 *              is requested by the companion computer from the autopilot.
 ********************************************************************************/
bool MavMsg::start_message_subscriptions(void)
{
    // What happens when we subscribe to a message and the request is denied?  How do we handle that?
    MavMsg::subscribe(MAVLINK_MSG_ID_HEARTBEAT, MESSAGE_RATE_1Hz);
    MavMsg::subscribe(MAVLINK_MSG_ID_GLOBAL_POSITION_INT, MESSAGE_RATE_40Hz);
    MavMsg::subscribe(MAVLINK_MSG_ID_SYSTEM_TIME, MESSAGE_RATE_DEFAULT);
    MavMsg::subscribe(MAVLINK_MSG_ID_SCALED_IMU, MESSAGE_RATE_40Hz);
    MavMsg::subscribe(MAVLINK_MSG_ID_ATTITUDE, MESSAGE_RATE_40Hz);
    MavMsg::subscribe(MAVLINK_MSG_ID_OPTICAL_FLOW, MESSAGE_RATE_40Hz);
    MavMsg::subscribe(MAVLINK_MSG_ID_DISTANCE_SENSOR, MESSAGE_RATE_40Hz);
    MavMsg::subscribe(MAVLINK_MSG_ID_ATTITUDE_TARGET, MESSAGE_RATE_1Hz);
    MavMsg::subscribe(MAVLINK_MSG_ID_ATTITUDE_QUATERNION, MESSAGE_RATE_1Hz);
    MavMsg::subscribe(MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED, MESSAGE_RATE_1Hz);
    MavMsg::subscribe(MAVLINK_MSG_ID_LOCAL_POSITION_NED, MESSAGE_RATE_1Hz);
    MavMsg::subscribe(MAVLINK_MSG_ID_SYS_STATUS, MESSAGE_RATE_1Hz);
    MavMsg::subscribe(MAVLINK_MSG_ID_PARAM_VALUE, MESSAGE_RATE_DEFAULT);
    MavMsg::subscribe(MAVLINK_MSG_ID_AUTOPILOT_VERSION, MESSAGE_RATE_DEFAULT);

    return true;
}

/********************************************************************************
 * Function: parse_mav_msgs
 * Description: Read the serial port and unpack specific messages if its
 *              specific mavlink message ID has been received.
 ********************************************************************************/
void MavMsg::parse_mav_msgs(void)
{
    mavlink_message_t msg;        // initialize the Mavlink message buffer
    mavlink_status_t status = {}; // Initialize the Mavlink status
    uint8_t byte;

    int n = SerialComm::bytes_available();
    for (int i = n; i > 0; i--)
    {
        byte = MavMsg::read_mav_msg();

        // Parse the byte and check if a message has been received
        if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status))
        {
            switch (msg.msgid)
            {
            case MAVLINK_MSG_ID_HEARTBEAT:
                MavMsg::proc_mav_heartbeat_msg(&msg, true);
                break;
            case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                MavMsg::proc_mav_gps_int_msg(&msg, true);
                break;
            case MAVLINK_MSG_ID_SYSTEM_TIME:
                MavMsg::proc_mav_system_time_msg(&msg, true);
                break;
            case MAVLINK_MSG_ID_SCALED_IMU:
                MavMsg::proc_mav_scaled_imu_msg(&msg, true);
                break;
            case MAVLINK_MSG_ID_OPTICAL_FLOW:
                MavMsg::proc_mav_optical_flow_msg(&msg, true);
                break;
            case MAVLINK_MSG_ID_DISTANCE_SENSOR:
                MavMsg::proc_mav_distance_sensor_msg(&msg, true);
                break;
            case MAVLINK_MSG_ID_ATTITUDE:
                MavMsg::proc_mav_attitude_msg(&msg, true);
                break;
            case MAVLINK_MSG_ID_ATTITUDE_TARGET:
                MavMsg::proc_mav_attitude_target_msg(&msg, true);
                break;
            case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
                MavMsg::proc_mav_attitude_quaternion_msg(&msg, true);
                break;
            case MAVLINK_MSG_ID_AUTOPILOT_VERSION:
                MavMsg::proc_mav_autopilot_version_msg(&msg, true);
                break;
            case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
                MavMsg::proc_mav_position_target_local_ned_msg(&msg, true);
                break;
            case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
                MavMsg::proc_mav_set_position_target_local_ned_msg(&msg, true);
                break;
            case MAVLINK_MSG_ID_SYS_STATUS:
                MavMsg::proc_mav_sys_status_msg(&msg, true);
                break;
            case MAVLINK_MSG_ID_STATUSTEXT:
                MavMsg::proc_mav_statustext_msg(&msg, true);
                break;
            case MAVLINK_MSG_ID_PARAM_VALUE:
                MavMsg::proc_mav_param_value_msg(&msg, true);
                break;
            case MAVLINK_MSG_ID_COMMAND_ACK:
                MavMsg::proc_mav_command_ack_msg(&msg, true);
                break;
            default:
                PrintPass::c_printf("Received message with ID: %d\n", (int)msg.msgid);
            }
        }
    }
}

/********************************************************************************
 * Function: mav_comm_init
 * Description: Code to run once at the beginning of the program.
 ********************************************************************************/
bool MavMsg::mav_comm_init(void)
{
    if (!start_mav_comm() ||
        !start_message_subscriptions())
    {
        PrintPass::c_fprintf("Failed to initialize MAVLink communication");
        return false;
    }

    return true;
}

/********************************************************************************
 * Function: mav_comm_loop
 * Description: Code to to execute each loop of the program.
 ********************************************************************************/
void MavMsg::mav_comm_loop(void)
{
    parse_mav_msgs();
}

/********************************************************************************
 * Function: mav_comm_shutdown
 * Description: Code run at the end of the program.
 ********************************************************************************/
void MavMsg::mav_comm_shutdown(void)
{
    stop_mav_comm();
}