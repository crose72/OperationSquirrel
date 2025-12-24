/********************************************************************************
 * @file    mav_data_hub.cpp
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   Central hub for handling incoming MAVLink communication.
 *          Responsible for configuring message rates, reading and parsing
 *          MAVLink bytes from the serial port, decoding messages, updating
 *          global MAVLink state variables, and forwarding processed data
 *          to the rest of the system. This module also manages opening and
 *          closing the MAVLink serial connection.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include <spdlog/spdlog.h>

#include "common_inc.h"
#include "mav_data_hub.h"
#include "mav_utils.h"

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/
uint16_t g_mav_cmd_id;           /*<  Command ID (of acknowledged command).*/
uint8_t g_mav_cmd_result;        /*<  Result of command.*/
uint8_t g_mav_cmd_progress;      /*< [%] The progress percentage when result is MAV_RESULT_IN_PROGRESS. Values: [0-100],
                                         or UINT8_MAX if the progress is unknown.*/
int32_t g_mav_cmd_result_param2; /*<  Additional result information. Can be set with a command-specific enum containing
                                        command-specific error reasons for why the command might be denied. If used, the associated
                                        enum must be documented in the corresponding MAV_CMD (this enum should have a 0 value to indicate
                                        "unused" or "unknown").*/
uint8_t g_mav_cmd_tgt_sys;       /*<  System ID of the target recipient. This is the ID of the system that sent the command for
                                                  which this COMMAND_ACK is an acknowledgement.*/
uint8_t g_mav_cmd_tgt_comp;      /*<  Component ID of the target recipient. This is the ID of the system that sent the command for
                                                 which this COMMAND_ACK is an acknowledgement.*/

uint32_t g_mav_sys_sensors_present;     /*<  Bitmap showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present.*/
uint32_t g_mav_sys_sensors_enabled;     /*<  Bitmap showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled.*/
uint32_t g_mav_sys_sensors_health;      /*<  Bitmap showing which onboard controllers and sensors have an error (or are operational). Value of 0: error.
                                                             Value of 1: healthy.*/
uint16_t g_mav_sys_load;                /*< [d%] Maximum usage in percent of the mainloop time. Values: [0-1000] - should always be below 1000*/
uint16_t g_mav_batt_voltage_mv;         /*< [mV] Battery voltage, UINT16_MAX: Voltage not sent by autopilot*/
int16_t g_mav_batt_current_ma;          /*< [cA] Battery current, -1: Current not sent by autopilot*/
uint16_t g_mav_comm_drop_rate;          /*< [c%] Communication drop rate, (UART, I2C, SPI, CAN), dropped packets on all links
                                                     (packets that were corrupted on reception on the MAV)*/
uint16_t g_mav_comm_errors;             /*<  Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)*/
uint16_t g_mav_err_count1;              /*<  Autopilot-specific errors*/
uint16_t g_mav_err_count2;              /*<  Autopilot-specific errors*/
uint16_t g_mav_err_count3;              /*<  Autopilot-specific errors*/
uint16_t g_mav_err_count4;              /*<  Autopilot-specific errors*/
int8_t g_mav_batt_remaining_pct;        /*< [%] Battery energy remaining, -1: Battery remaining energy not sent by autopilot*/
uint32_t g_mav_sys_sensors_present_ext; /*<  Bitmap showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present.*/
uint32_t g_mav_sys_sensors_enabled_ext; /*<  Bitmap showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled.*/
uint32_t g_mav_sys_sensors_health_ext;  /*<  Bitmap showing which onboard controllers and sensors have an error (or are operational). Value of 0: error.
                                                             Value of 1: healthy.*/
uint32_t g_mav_mode_custom;             /*<  A bitfield for use for autopilot-specific flags*/
bool g_ctrl_land_override;
uint8_t g_mav_type;           /*<  Vehicle or component type. For a flight controller component the vehicle type (quadrotor, helicopter, etc.). For other components the
                                  component type (e.g. camera, gimbal, etc.). This should be used in preference to component id for identifying the component type.*/
uint8_t g_mav_autopilot_type; /*<  Autopilot type / class. Use MAV_AUTOPILOT_INVALID for components that are not flight controllers.*/
uint8_t g_mav_mode_base;      /*<  System mode bitmap.*/
uint8_t g_mav_state;          /*<  System status flag.*/
uint8_t g_mav_version;        /*<  MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version*/

int32_t g_mav_gps_lat;           /*< [degE7] Latitude, expressed*/
int32_t g_mav_gps_lon;           /*< [degE7] Longitude, expressed*/
int32_t g_mav_gps_alt_msl;       /*< [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.*/
int32_t g_mav_gps_alt_rel;       /*< [mm] Altitude above ground*/
int16_t g_mav_gps_vel_x;         /*< [cm/s] Ground X Speed (Latitude, positive north)*/
int16_t g_mav_gps_vel_y;         /*< [cm/s] Ground Y Speed (Longitude, positive east)*/
int16_t g_mav_gps_vel_z;         /*< [cm/s] Ground Z Speed (Altitude, positive down)*/
uint16_t g_mav_gps_heading_cdeg; /*< [cdeg] Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/

float g_mav_veh_roll_rad;   /*< [rad] Roll angle (-pi..+pi)*/
float g_mav_veh_pitch_rad;  /*< [rad] Pitch angle (-pi..+pi)*/
float g_mav_veh_yaw_rad;    /*< [rad] Yaw angle (-pi..+pi)*/
float g_mav_veh_roll_rate;  /*< [rad/s] Roll angular speed*/
float g_mav_veh_pitch_rate; /*< [rad/s] Pitch angular speed*/
float g_mav_veh_yaw_rate;   /*< [rad/s] Yaw angular speed*/

int16_t g_mav_imu_accel_x; /*< [mG] X acceleration*/
int16_t g_mav_imu_accel_y; /*< [mG] Y acceleration*/
int16_t g_mav_imu_accel_z; /*< [mG] Z acceleration*/
int16_t g_mav_imu_gyro_x;  /*< [mrad/s] Angular speed around X axis*/
int16_t g_mav_imu_gyro_y;  /*< [mrad/s] Angular speed around Y axis*/
int16_t g_mav_imu_gyro_z;  /*< [mrad/s] Angular speed around Z axis*/
int16_t g_mav_imu_mag_x;   /*< [mgauss] X Magnetic field*/
int16_t g_mav_imu_mag_y;   /*< [mgauss] Y Magnetic field*/
int16_t g_mav_imu_mag_z;   /*< [mgauss] Z Magnetic field*/

float g_mav_att_target_q1;         /*<  Quaternion component 1, w (1 in null-rotation)*/
float g_mav_att_target_q2;         /*<  Quaternion component 2, x (0 in null-rotation)*/
float g_mav_att_target_q3;         /*<  Quaternion component 3, y (0 in null-rotation)*/
float g_mav_att_target_q4;         /*<  Quaternion component 4, z (0 in null-rotation)*/
float g_mav_att_target_roll_rate;  /*< [rad/s] Roll angular speed*/
float g_mav_att_target_pitch_rate; /*< [rad/s] Pitch angular speed*/
float g_mav_att_target_yaw_rate;   /*< [rad/s] Yaw angular speed*/
float g_mav_att_target_thrust;     /*<  Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)*/
uint8_t attitude_target_mask;      /*<  Bitmap to indicate which dimensions should be ignored by the vehicle.*/

float g_mav_att_actual_q1;         /*<  Quaternion component 1, w (1 in null-rotation)*/
float g_mav_att_actual_q2;         /*<  Quaternion component 2, x (0 in null-rotation)*/
float g_mav_att_actual_q3;         /*<  Quaternion component 3, y (0 in null-rotation)*/
float g_mav_att_actual_q4;         /*<  Quaternion component 4, z (0 in null-rotation)*/
float g_mav_att_actual_roll_rate;  /*< [rad/s] Roll angular speed*/
float g_mav_att_actual_pitch_rate; /*< [rad/s] Pitch angular speed*/
float g_mav_att_actual_yaw_rate;   /*< [rad/s] Yaw angular speed*/
float g_mav_att_repr_offset_q[4];  /*<  Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)*/

uint16_t g_mav_rngfndr_min_cm;     /*< [cm] Minimum distance the sensor can measure*/
uint16_t g_mav_rngfndr_max_cm;     /*< [cm] Maximum distance the sensor can measure*/
uint16_t g_mav_rngfndr_dist_cm;    /*< [cm] Current distance reading*/
uint8_t g_mav_rngfndr_type;        /*<  Type of distance sensor.*/
uint8_t g_mav_rngfndr_id;          /*<  Onboard ID of the sensor*/
uint8_t g_mav_rngfndr_orient;      /*<  Direction the sensor faces. downward-facing: ROTATION_PITCH_270, upward-facing: ROTATION_PITCH_90, backward-facing: ROTATION_PITCH_180, forward-facing: ROTATION_NONE, left-facing: ROTATION_YAW_90, right-facing: ROTATION_YAW_270*/
uint8_t g_mav_rngfndr_cov;         /*< [cm^2] Measurement variance. Max standard deviation is 6cm. UINT8_MAX if unknown.*/
float g_mav_rngfndr_fov_horiz_rad; /*< [rad] Horizontal Field of View (angle) where the distance measurement is valid and the field of view is known. Otherwise this is set to 0.*/
float g_mav_rngfndr_fov_vert_rad;  /*< [rad] Vertical Field of View (angle) where the distance measurement is valid and the field of view is known. Otherwise this is set to 0.*/
float g_mav_rngfndr_quat[4];       /*<  Quaternion of the sensor orientation in vehicle body frame (w, x, y, z order, zero-rotation is 1, 0, 0, 0). Zero-rotation is along the vehicle body x-axis. This field is required if the orientation is set to MAV_SENSOR_ROTATION_CUSTOM. Set it to 0 if invalid."*/
uint8_t g_mav_rngfndr_quality;     /*< [%] Signal quality of the sensor. Specific to each sensor type, representing the relation of the signal strength with the target reflectivity, distance, size or aspect, but normalised as a percentage. 0 = unknown/unset signal quality, 1 = invalid signal, 100 = perfect signal.*/

float g_mav_flow_vel_x;         /*< [m/s] Flow in x-sensor direction, angular-speed compensated*/
float g_mav_flow_vel_y;         /*< [m/s] Flow in y-sensor direction, angular-speed compensated*/
float g_mav_flow_ground_dist_m; /*< [m] Ground distance. Positive value: distance known. Negative value: Unknown distance*/
int16_t g_mav_flow_px_x;        /*< [dpix] Flow in x-sensor direction*/
int16_t g_mav_flow_px_y;        /*< [dpix] Flow in y-sensor direction*/
uint8_t g_mav_flow_sensor_id;   /*<  Sensor ID*/
uint8_t g_mav_flow_quality;     /*<  Optical flow quality / confidence. 0: bad, 255: maximum quality*/
float g_mav_flow_rate_x;        /*< [rad/s] Flow rate about X axis*/
float g_mav_flow_rate_y;        /*< [rad/s] Flow rate about Y axis*/

float g_mav_veh_pos_ned_x; /*< [m] X Position*/
float g_mav_veh_pos_ned_y; /*< [m] Y Position*/
float g_mav_veh_pos_ned_z; /*< [m] Z Position*/
float g_mav_veh_vel_ned_x; /*< [m/s] X Speed*/
float g_mav_veh_vel_ned_y; /*< [m/s] Y Speed*/
float g_mav_veh_vel_ned_z; /*< [m/s] Z Speed*/

// Parameter read

float g_mav_param_val;      /*<  Onboard parameter value*/
uint16_t g_mav_param_count; /*<  Total number of onboard parameters*/
uint16_t g_mav_param_index; /*<  Index of this onboard parameter*/
char g_mav_param_id[17];    /*<  Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string*/
uint8_t g_mav_param_type;   /*<  Onboard parameter type.*/
std::string g_mav_param_name;
bool g_mav_param_read;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/
const int32_t MESSAGE_RATE_DEFAULT = 0;
const int32_t MESSAGE_RATE_40Hz = 25000;
const int32_t MESSAGE_RATE_1Hz = 1000000;

/********************************************************************************
 * Function definitions
 ********************************************************************************/
bool start_mav_comm(void) { return MavSerial::start_mav_comm(); }; // Open up uart port for mavlink messages
void stop_mav_comm(void) { MavSerial::stop_mav_comm(); };          // Stop mavlink comms on uart port
uint8_t read_mav_msg(void) { return MavSerial::read_mav_msg(); };  // Read a byte
void subscribe(uint16_t msg_id, float msg_interval);               // Subscribe to a mavlink message at desired rate
void set_msg_rate(uint16_t msg_id, float msg_interval) { MavCmd::set_mav_msg_rate(companion_sys_id, companion_comp_id, autopilot_sys_id, autopilot_comp_id, msg_id, msg_interval); };
void req_msg(uint16_t msg_id) { MavCmd::req_mav_msg(companion_sys_id, companion_comp_id, autopilot_sys_id, autopilot_comp_id, msg_id); };
bool start_message_subscriptions(void);
void parse_mav_msgs(void);

void proc_mav_heartbeat_msg(const mavlink_message_t *msg);
void proc_mav_gps_int_msg(const mavlink_message_t *msg);
void proc_mav_system_time_msg(const mavlink_message_t *msg);
void proc_mav_sys_status_msg(const mavlink_message_t *msg);
void proc_mav_statustext_msg(const mavlink_message_t *msg);
void proc_mav_param_value_msg(const mavlink_message_t *msg);
void proc_mav_autopilot_version_msg(const mavlink_message_t *msg);
void proc_mav_scaled_imu_msg(const mavlink_message_t *msg);
void proc_mav_local_position_msg(const mavlink_message_t *msg);
void proc_mav_position_target_local_ned_msg(const mavlink_message_t *msg);
void proc_mav_set_position_target_local_ned_msg(const mavlink_message_t *msg);
void proc_mav_attitude_msg(const mavlink_message_t *msg);
void proc_mav_attitude_target_msg(const mavlink_message_t *msg);
void proc_mav_set_attitude_target_msg(const mavlink_message_t *msg);
void proc_mav_attitude_quaternion_msg(const mavlink_message_t *msg);
void proc_mav_command_ack_msg(const mavlink_message_t *msg);
void proc_mav_optical_flow_msg(const mavlink_message_t *msg);
void proc_mav_distance_sensor_msg(const mavlink_message_t *msg);
void proc_mav_position_local_ned_msg(const mavlink_message_t *msg);
void proc_mav_unknown(const mavlink_message_t *msg);

/********************************************************************************
 * Function: subscribe
 * Description: Subscribe to mavlink messages and specify
 *              the desired to receive the message at.
 ********************************************************************************/
void subscribe(uint16_t msg_id, float msg_interval)
{
    set_msg_rate(msg_id, msg_interval);
    req_msg(msg_id);
}

/********************************************************************************
 * Function: proc_heartbeat
 * Description: Decode heartbeat message and pass along the desired information.
 ********************************************************************************/
void proc_mav_heartbeat_msg(const mavlink_message_t *msg)
{
    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(msg, &heartbeat);

    /* Filter out non-quadrotor heartbeats - e.g. a GCS */
    if (heartbeat.type == 2)
    {
        g_mav_type = heartbeat.type;
        g_mav_autopilot_type = heartbeat.autopilot;
        g_mav_mode_base = heartbeat.base_mode;
        g_mav_mode_custom = heartbeat.custom_mode;
        g_mav_state = heartbeat.system_status;
        g_mav_version = heartbeat.mavlink_version;
    }

#ifdef DEBUG_BUILD

    print_heartbeat(heartbeat);

#endif // DEBUG_BUILD
}

/********************************************************************************
 * Function: proc_mav_gps_int
 * Description: Decode gps message and pass along the desired information.
 ********************************************************************************/
void proc_mav_gps_int_msg(const mavlink_message_t *msg)
{
    mavlink_global_position_int_t global_pos_int;
    mavlink_msg_global_position_int_decode(msg, &global_pos_int);

    g_mav_gps_lat = global_pos_int.lat;
    g_mav_gps_lon = global_pos_int.lon;
    g_mav_gps_alt_msl = global_pos_int.alt;
    g_mav_gps_alt_rel = global_pos_int.relative_alt;
    g_mav_gps_vel_x = global_pos_int.vx;
    g_mav_gps_vel_y = global_pos_int.vy;
    g_mav_gps_vel_z = global_pos_int.vz;
    g_mav_gps_heading_cdeg = global_pos_int.hdg;

#ifdef DEBUG_BUILD

    print_global_position_int(global_pos_int);

#endif // DEBUG_BUILD
}

/********************************************************************************
 * Function: proc_mav_system_time_msg
 * Description: Decode system time message and optionally print the
 *              information.
 ********************************************************************************/
void proc_mav_system_time_msg(const mavlink_message_t *msg)
{
    mavlink_system_time_t system_time;
    mavlink_msg_system_time_decode(msg, &system_time);

#ifdef DEBUG_BUILD

    print_system_time(system_time);

#endif // DEBUG_BUILD
}

/********************************************************************************
 * Function: proc_mav_sys_status_msg
 * Description: Decode system status message and optionally print the
 *              information.
 ********************************************************************************/
void proc_mav_sys_status_msg(const mavlink_message_t *msg)
{
    mavlink_sys_status_t sys_status;
    mavlink_msg_sys_status_decode(msg, &sys_status);

    g_mav_sys_sensors_present = sys_status.onboard_control_sensors_present;
    g_mav_sys_sensors_enabled = sys_status.onboard_control_sensors_enabled;
    g_mav_sys_sensors_health = sys_status.onboard_control_sensors_health;
    g_mav_sys_load = sys_status.load;
    g_mav_batt_voltage_mv = sys_status.voltage_battery;
    g_mav_batt_current_ma = sys_status.current_battery;
    g_mav_comm_drop_rate = sys_status.drop_rate_comm;
    g_mav_comm_errors = sys_status.errors_comm;
    g_mav_err_count1 = sys_status.errors_count1;
    g_mav_err_count2 = sys_status.errors_count2;
    g_mav_err_count3 = sys_status.errors_count3;
    g_mav_err_count4 = sys_status.errors_count4;
    g_mav_batt_remaining_pct = sys_status.battery_remaining;
    g_mav_sys_sensors_present_ext = sys_status.onboard_control_sensors_present_extended;
    g_mav_sys_sensors_enabled_ext = sys_status.onboard_control_sensors_enabled_extended;
    g_mav_sys_sensors_health_ext = sys_status.onboard_control_sensors_health_extended;

#ifdef DEBUG_BUILD

    print_sys_status(sys_status);

#endif // DEBUG_BUILD
}

/********************************************************************************
 * Function: proc_mav_statustext_msg
 * Description: Decode status text message and optionally print the
 *              information.
 ********************************************************************************/
void proc_mav_statustext_msg(const mavlink_message_t *msg)
{
    mavlink_statustext_t statustext;
    mavlink_msg_statustext_decode(msg, &statustext);

#ifdef DEBUG_BUILD

    print_statustext(statustext);

#endif // DEBUG_BUILD
}

/********************************************************************************
 * Function: proc_mav_param_value_msg
 * Description: Decode parameter value message and optionally print the
 *              information.
 ********************************************************************************/
void proc_mav_param_value_msg(const mavlink_message_t *msg)
{
    mavlink_param_value_t param;
    mavlink_msg_param_value_decode(msg, &param);

    // Copy param_id safely for internal use
    memcpy(g_mav_param_id, param.param_id, 16);
    g_mav_param_id[16] = '\0';

    g_mav_param_val = param.param_value;
    g_mav_param_count = param.param_count;
    g_mav_param_index = param.param_index;
    g_mav_param_type = param.param_type;
    g_mav_param_name = g_mav_param_id;
    g_mav_param_read = true;

    // // --- Debug: print all raw MAVLink values ---
    // char name_buf[17];
    // memcpy(name_buf, param.param_id, 16);
    // name_buf[16] = '\0'; // ensure null termination

    // spdlog::info("[MAVLINK] PARAM_VALUE message received:");
    // spdlog::info("  param_id:     {}", name_buf);
    // spdlog::info("  param_value:  {}", param.param_value);
    // spdlog::info("  param_type:   {}", static_cast<int>(param.param_type));
    // spdlog::info("  param_index:  {}", param.param_index);
    // spdlog::info("  param_count:  {}", param.param_count);
}

/********************************************************************************
 * Function: proc_mav_autopilot_version_msg
 * Description: Decode autopilot version message and optionally print the
 *              information.
 ********************************************************************************/
void proc_mav_autopilot_version_msg(const mavlink_message_t *msg)
{
    mavlink_autopilot_version_t autopilot_version;
    mavlink_msg_autopilot_version_decode(msg, &autopilot_version);

#ifdef DEBUG_BUILD

    print_autopilot_version(autopilot_version);

#endif // DEBUG_BUILD
}

/********************************************************************************
 * Function: proc_mav_scaled_imu_msg
 * Description: Decode scaled IMU message and optionally print the information.
 ********************************************************************************/
void proc_mav_scaled_imu_msg(const mavlink_message_t *msg)
{
    mavlink_scaled_imu_t scaled_imu;
    mavlink_msg_scaled_imu_decode(msg, &scaled_imu);

    g_mav_imu_accel_x = scaled_imu.xacc;
    g_mav_imu_accel_y = scaled_imu.yacc;
    g_mav_imu_accel_z = scaled_imu.zacc;
    g_mav_imu_gyro_x = scaled_imu.xgyro;
    g_mav_imu_gyro_y = scaled_imu.ygyro;
    g_mav_imu_gyro_z = scaled_imu.zgyro;
    g_mav_imu_mag_x = scaled_imu.xmag;
    g_mav_imu_mag_y = scaled_imu.ymag;
    g_mav_imu_mag_z = scaled_imu.zmag;

#ifdef DEBUG_BUILD

    print_scaled_imu(scaled_imu);

#endif // DEBUG_BUILD
}

/********************************************************************************
 * Function: proc_mav_local_position_msg
 * Description: Decode local position message and optionally print the
 *              information.
 ********************************************************************************/
void proc_mav_local_position_msg(const mavlink_message_t *msg)
{
    mavlink_local_position_ned_t local_position;
    mavlink_msg_local_position_ned_decode(msg, &local_position);

#ifdef DEBUG_BUILD

    print_local_position(local_position);

#endif // DEBUG_BUILD
}

/********************************************************************************
 * Function: proc_mav_position_target_local_ned_msg
 * Description: Decode position target local NED message and optionally print the
 *              information.
 ********************************************************************************/
void proc_mav_position_target_local_ned_msg(const mavlink_message_t *msg)
{
    mavlink_position_target_local_ned_t position_target_local_ned;
    mavlink_msg_position_target_local_ned_decode(msg, &position_target_local_ned);

#ifdef DEBUG_BUILD

    print_position_target_local_ned(position_target_local_ned);

#endif // DEBUG_BUILD
}

/********************************************************************************
 * Function: proc_mav_set_position_target_local_ned_msg
 * Description: Decode set position target local NED message and optionally print
 *              the information.
 ********************************************************************************/
void proc_mav_set_position_target_local_ned_msg(const mavlink_message_t *msg)
{
    mavlink_set_position_target_local_ned_t set_position_target_local_ned;
    mavlink_msg_set_position_target_local_ned_decode(msg, &set_position_target_local_ned);

#ifdef DEBUG_BUILD

    print_set_position_target_local_ned(set_position_target_local_ned);

#endif // DEBUG_BUILD
}

/********************************************************************************
 * Function: proc_mav_position_local_ned_msg
 * Description: Decode local position NED message and optionally print the
 *              information.
 ********************************************************************************/
void proc_mav_position_local_ned_msg(const mavlink_message_t *msg)
{
    mavlink_local_position_ned_t position_local_ned;
    mavlink_msg_local_position_ned_decode(msg, &position_local_ned);

    g_mav_veh_pos_ned_x = position_local_ned.x;
    g_mav_veh_pos_ned_y = position_local_ned.y;
    g_mav_veh_pos_ned_z = position_local_ned.z;
    // TODO - figure out why
    // Data showed Forward and right were -vx & -vy
    // Forward and right should be +vx & +vy
    g_mav_veh_vel_ned_x = position_local_ned.vx;
    g_mav_veh_vel_ned_y = position_local_ned.vy;
    g_mav_veh_vel_ned_z = position_local_ned.vz;

#ifdef DEBUG_BUILD

    print_position_local_ned(position_local_ned);

#endif // DEBUG_BUILD
}

/********************************************************************************
 * Function: proc_mav_attitude_msg
 * Description: Decode attitude message and optionally print the information.
 ********************************************************************************/
void proc_mav_attitude_msg(const mavlink_message_t *msg)
{
    mavlink_attitude_t attitude;
    mavlink_msg_attitude_decode(msg, &attitude);

    g_mav_veh_roll_rad = attitude.roll;
    g_mav_veh_pitch_rad = attitude.pitch;
    g_mav_veh_yaw_rad = attitude.yaw;
    g_mav_veh_roll_rate = attitude.rollspeed;
    g_mav_veh_pitch_rate = attitude.pitchspeed;
    g_mav_veh_yaw_rate = attitude.yawspeed;

#ifdef DEBUG_BUILD

    print_attitude(attitude);

#endif // DEBUG_BUILD
}

/********************************************************************************
 * Function: proc_mav_attitude_target_msg
 * Description: Decode attitude target message and optionally print the
 *              information.
 ********************************************************************************/
void proc_mav_attitude_target_msg(const mavlink_message_t *msg)
{
    mavlink_attitude_target_t attitude_target;
    mavlink_msg_attitude_target_decode(msg, &attitude_target);

#ifdef DEBUG_BUILD

    print_attitude_target(attitude_target);

#endif // DEBUG_BUILD
}

/********************************************************************************
 * Function: proc_mav_set_attitude_target_msg
 * Description: Decode set attitude target message and optionally print the
 *              information.
 ********************************************************************************/
void proc_mav_set_attitude_target_msg(const mavlink_message_t *msg)
{
    mavlink_set_attitude_target_t set_attitude_target;
    mavlink_msg_set_attitude_target_decode(msg, &set_attitude_target);

#ifdef DEBUG_BUILD

    print_set_attitude_target(set_attitude_target);

#endif // DEBUG_BUILD
}

/********************************************************************************
 * Function: proc_mav_attitude_quaternion_msg
 * Description: Decode attitude quaternion message and optionally print the
 *              information.
 ********************************************************************************/
void proc_mav_attitude_quaternion_msg(const mavlink_message_t *msg)
{
    mavlink_attitude_quaternion_t attitude_quaternion;
    mavlink_msg_attitude_quaternion_decode(msg, &attitude_quaternion);

    g_mav_att_actual_q1 = attitude_quaternion.q1;
    g_mav_att_actual_q2 = attitude_quaternion.q2;
    g_mav_att_actual_q3 = attitude_quaternion.q3;
    g_mav_att_actual_q4 = attitude_quaternion.q4;
    g_mav_att_actual_roll_rate = attitude_quaternion.rollspeed;
    g_mav_att_actual_pitch_rate = attitude_quaternion.pitchspeed;
    g_mav_att_actual_yaw_rate = attitude_quaternion.yawspeed;

    for (int i = 0; i < 4; i++)
    {
        g_mav_att_repr_offset_q[i] = attitude_quaternion.repr_offset_q[i];
    }

#ifdef DEBUG_BUILD

    print_attitude_quaternion(attitude_quaternion);

#endif // DEBUG_BUILD
}

/********************************************************************************
 * Function: proc_mav_command_ack_msg
 * Description: Decode command acknowledgment message and optionally print the
 *              information.
 ********************************************************************************/
void proc_mav_command_ack_msg(const mavlink_message_t *msg)
{
    mavlink_command_ack_t command_ack;
    mavlink_msg_command_ack_decode(msg, &command_ack);

    g_mav_cmd_id = command_ack.command;
    g_mav_cmd_result = command_ack.result;
    g_mav_cmd_progress = command_ack.progress;
    g_mav_cmd_result_param2 = command_ack.result_param2;
    g_mav_cmd_tgt_sys = command_ack.target_system;
    g_mav_cmd_tgt_comp = command_ack.target_component;

#ifdef DEBUG_BUILD

    print_command_ack(command_ack);

#endif // DEBUG_BUILD
}

/********************************************************************************
 * Function: proc_mav_optical_flow_msg
 * Description: Decode optical flow message and optionally print the information.
 ********************************************************************************/
void proc_mav_optical_flow_msg(const mavlink_message_t *msg)
{
    mavlink_optical_flow_t optical_flow;
    mavlink_msg_optical_flow_decode(msg, &optical_flow);

    g_mav_flow_vel_x = optical_flow.flow_comp_m_x;
    g_mav_flow_vel_y = optical_flow.flow_comp_m_y;
    g_mav_flow_ground_dist_m = optical_flow.ground_distance;
    g_mav_flow_px_x = optical_flow.flow_x;
    g_mav_flow_px_y = optical_flow.flow_y;
    g_mav_flow_sensor_id = optical_flow.sensor_id;
    g_mav_flow_quality = optical_flow.quality;
    g_mav_flow_rate_x = optical_flow.flow_rate_x;
    g_mav_flow_rate_y = optical_flow.flow_rate_y;

#ifdef DEBUG_BUILD

    print_optical_flow(optical_flow);

#endif // DEBUG_BUILD
}

/********************************************************************************
 * Function: proc_mav_distance_sensor_msg
 * Description: Decode distance sensor message and optionally print the
 *              information.
 ********************************************************************************/
void proc_mav_distance_sensor_msg(const mavlink_message_t *msg)
{
    mavlink_distance_sensor_t distance_sensor;
    mavlink_msg_distance_sensor_decode(msg, &distance_sensor);

    g_mav_rngfndr_min_cm = distance_sensor.min_distance;
    g_mav_rngfndr_max_cm = distance_sensor.max_distance;
    g_mav_rngfndr_dist_cm = distance_sensor.current_distance;
    g_mav_rngfndr_type = distance_sensor.type;
    g_mav_rngfndr_id = distance_sensor.id;
    g_mav_rngfndr_orient = distance_sensor.orientation;
    g_mav_rngfndr_cov = distance_sensor.covariance;
    g_mav_rngfndr_fov_horiz_rad = distance_sensor.horizontal_fov;
    g_mav_rngfndr_fov_vert_rad = distance_sensor.vertical_fov;
    g_mav_rngfndr_quality = distance_sensor.signal_quality;

    for (int i = 0; i < 4; i++)
    {
        g_mav_rngfndr_quat[i] = distance_sensor.quaternion[i];
    }

#ifdef DEBUG_BUILD

    print_distance_sensor(distance_sensor);

#endif // DEBUG_BUILD
}

/********************************************************************************
 * Function: proc_mav_unknown
 * Description: Print message ID of mavlink messages that haven't been processed.
 ********************************************************************************/
void proc_mav_unknown(const mavlink_message_t *msg)
{
#ifdef DEBUG_BUILD

    printf("Received message with ID: %u\n", msg->msgid);

#endif // DEBUG_BUILD
}

/********************************************************************************
 * Function: start_message_subscriptions
 * Description: Handle all message subscriptions.  Any messages subscribed to
 *              is requested by the companion computer from the autopilot.
 ********************************************************************************/
bool start_message_subscriptions(void)
{
    // What happens when we subscribe to a message and the request is denied?  How do we handle that?
    subscribe(MAVLINK_MSG_ID_HEARTBEAT, MESSAGE_RATE_DEFAULT);
    subscribe(MAVLINK_MSG_ID_GLOBAL_POSITION_INT, MESSAGE_RATE_40Hz);
    subscribe(MAVLINK_MSG_ID_SYSTEM_TIME, MESSAGE_RATE_DEFAULT);
    subscribe(MAVLINK_MSG_ID_SCALED_IMU, MESSAGE_RATE_40Hz);
    subscribe(MAVLINK_MSG_ID_ATTITUDE, MESSAGE_RATE_40Hz);
    subscribe(MAVLINK_MSG_ID_OPTICAL_FLOW, MESSAGE_RATE_40Hz);
    subscribe(MAVLINK_MSG_ID_DISTANCE_SENSOR, MESSAGE_RATE_40Hz);
    subscribe(MAVLINK_MSG_ID_ATTITUDE_TARGET, MESSAGE_RATE_DEFAULT);
    subscribe(MAVLINK_MSG_ID_ATTITUDE_QUATERNION, MESSAGE_RATE_DEFAULT);
    subscribe(MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED, MESSAGE_RATE_40Hz);
    subscribe(MAVLINK_MSG_ID_LOCAL_POSITION_NED, MESSAGE_RATE_40Hz);
    subscribe(MAVLINK_MSG_ID_SYS_STATUS, MESSAGE_RATE_DEFAULT);
    subscribe(MAVLINK_MSG_ID_PARAM_VALUE, MESSAGE_RATE_DEFAULT);
    subscribe(MAVLINK_MSG_ID_AUTOPILOT_VERSION, MESSAGE_RATE_DEFAULT);

    return true;
}

/********************************************************************************
 * Function: parse_mav_msgs
 * Description: Read the serial port and unpack specific messages if its
 *              specific mavlink message ID has been received.
 ********************************************************************************/
void parse_mav_msgs(void)
{
    mavlink_message_t msg;        // initialize the Mavlink message buffer
    mavlink_status_t status = {}; // Initialize the Mavlink status
    uint8_t byte;

    // reset param read status
    g_mav_param_read = false;

    int n = MavSerial::bytes_available();

    for (int i = n; i > 0; i--)
    {
        byte = read_mav_msg();

        // Parse the byte and check if a message has been received
        if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status))
        {
            switch (msg.msgid)
            {
            case MAVLINK_MSG_ID_HEARTBEAT:
                proc_mav_heartbeat_msg(&msg);
                break;
            case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                proc_mav_gps_int_msg(&msg);
                break;
            case MAVLINK_MSG_ID_SYSTEM_TIME:
                proc_mav_system_time_msg(&msg);
                break;
            case MAVLINK_MSG_ID_SCALED_IMU:
                proc_mav_scaled_imu_msg(&msg);
                break;
            case MAVLINK_MSG_ID_OPTICAL_FLOW:
                proc_mav_optical_flow_msg(&msg);
                break;
            case MAVLINK_MSG_ID_DISTANCE_SENSOR:
                proc_mav_distance_sensor_msg(&msg);
                break;
            case MAVLINK_MSG_ID_ATTITUDE:
                proc_mav_attitude_msg(&msg);
                break;
            case MAVLINK_MSG_ID_ATTITUDE_TARGET:
                proc_mav_attitude_target_msg(&msg);
                break;
            case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
                proc_mav_attitude_quaternion_msg(&msg);
                break;
            case MAVLINK_MSG_ID_AUTOPILOT_VERSION:
                proc_mav_autopilot_version_msg(&msg);
                break;
            case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
                proc_mav_position_target_local_ned_msg(&msg);
                break;
            case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
                proc_mav_set_position_target_local_ned_msg(&msg);
                break;
            case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
                proc_mav_position_local_ned_msg(&msg);
                break;
            case MAVLINK_MSG_ID_SYS_STATUS:
                proc_mav_sys_status_msg(&msg);
                break;
            case MAVLINK_MSG_ID_STATUSTEXT:
                proc_mav_statustext_msg(&msg);
                break;
            case MAVLINK_MSG_ID_PARAM_VALUE:
                proc_mav_param_value_msg(&msg);
                break;
            case MAVLINK_MSG_ID_COMMAND_ACK:
                proc_mav_command_ack_msg(&msg);
                break;
            default:
                proc_mav_unknown(&msg);
            }
        }
    }
}

/********************************************************************************
 * Function: MavMsg
 * Description: Class constructor
 ********************************************************************************/
MavMsg::MavMsg() {};

/********************************************************************************
 * Function: ~MavMsg
 * Description: Class destructor
 ********************************************************************************/
MavMsg::~MavMsg() {};

/********************************************************************************
 * Function: init
 * Description: Code to run once at the beginning of the program.
 ********************************************************************************/
bool MavMsg::init(void)
{
    if (!start_mav_comm() ||
        !start_message_subscriptions())
    {
        spdlog::error("Failed to initialize MAVLink communication\n");
        return false;
    }

    return true;
}

/********************************************************************************
 * Function: loop
 * Description: Code to to execute each loop of the program.
 ********************************************************************************/
void MavMsg::loop(void)
{
    parse_mav_msgs();
}

/********************************************************************************
 * Function: shutdown
 * Description: Code run at the end of the program.
 ********************************************************************************/
void MavMsg::shutdown(void)
{
    stop_mav_comm();
}
