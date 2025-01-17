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
uint16_t mav_veh_command_id;              /*<  Command ID (of acknowledged command).*/
uint8_t mav_veh_command_result;           /*<  Result of command.*/
uint8_t mav_veh_command_progress;         /*< [%] The progress percentage when result is MAV_RESULT_IN_PROGRESS. Values: [0-100],
                                            or UINT8_MAX if the progress is unknown.*/
int32_t mav_veh_command_result_param2;    /*<  Additional result information. Can be set with a command-specific enum containing
                                           command-specific error reasons for why the command might be denied. If used, the associated
                                           enum must be documented in the corresponding MAV_CMD (this enum should have a 0 value to indicate
                                           "unused" or "unknown").*/
uint8_t mav_veh_command_target_system;    /*<  System ID of the target recipient. This is the ID of the system that sent the command for
                                               which this COMMAND_ACK is an acknowledgement.*/
uint8_t mav_veh_command_target_component; /*<  Component ID of the target recipient. This is the ID of the system that sent the command for
                                            which this COMMAND_ACK is an acknowledgement.*/

uint32_t mav_veh_sys_stat_onbrd_cntrl_snsrs_present;      /*<  Bitmap showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present.*/
uint32_t mav_veh_sys_stat_onbrd_cntrl_snsrs_enabled;      /*<  Bitmap showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled.*/
uint32_t mav_veh_sys_stat_onbrd_cntrl_snsrs_health;       /*<  Bitmap showing which onboard controllers and sensors have an error (or are operational). Value of 0: error.
                                                              Value of 1: healthy.*/
uint16_t mav_veh_sys_stat_load;                           /*< [d%] Maximum usage in percent of the mainloop time. Values: [0-1000] - should always be below 1000*/
uint16_t mav_veh_sys_stat_voltage_battery;                /*< [mV] Battery voltage, UINT16_MAX: Voltage not sent by autopilot*/
int16_t mav_veh_sys_stat_current_battery;                 /*< [cA] Battery current, -1: Current not sent by autopilot*/
uint16_t mav_veh_sys_stat_drop_rate_comm;                 /*< [c%] Communication drop rate, (UART, I2C, SPI, CAN), dropped packets on all links
                                                            (packets that were corrupted on reception on the MAV)*/
uint16_t mav_veh_sys_stat_errors_comm;                    /*<  Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)*/
uint16_t mav_veh_sys_stat_errors_count1;                  /*<  Autopilot-specific errors*/
uint16_t mav_veh_sys_stat_errors_count2;                  /*<  Autopilot-specific errors*/
uint16_t mav_veh_sys_stat_errors_count3;                  /*<  Autopilot-specific errors*/
uint16_t mav_veh_sys_stat_errors_count4;                  /*<  Autopilot-specific errors*/
int8_t mav_veh_sys_stat_battery_remaining;                /*< [%] Battery energy remaining, -1: Battery remaining energy not sent by autopilot*/
uint32_t mav_veh_sys_stat_onbrd_cntrl_snsrs_prsnt_extnd;  /*<  Bitmap showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present.*/
uint32_t mav_veh_sys_stat_onbrd_cntrl_snsrs_enbld_extnd;  /*<  Bitmap showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled.*/
uint32_t mav_veh_sys_stat_onbrd_cntrl_snsrs_health_extnd; /*<  Bitmap showing which onboard controllers and sensors have an error (or are operational). Value of 0: error.
                                                            Value of 1: healthy.*/
uint32_t mav_veh_custom_mode;                             /*<  A bitfield for use for autopilot-specific flags*/
uint8_t mav_veh_type;                                     /*<  Vehicle or component type. For a flight controller component the vehicle type (quadrotor, helicopter, etc.). For other components the
                                                            component type (e.g. camera, gimbal, etc.). This should be used in preference to component id for identifying the component type.*/
uint8_t mav_veh_autopilot_type;                           /*<  Autopilot type / class. Use MAV_AUTOPILOT_INVALID for components that are not flight controllers.*/
uint8_t mav_veh_base_mode;                                /*<  System mode bitmap.*/
uint8_t mav_veh_state;                                    /*<  System status flag.*/
uint8_t mav_veh_mavlink_version;                          /*<  MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version*/

int32_t mav_veh_lat;      /*< [degE7] Latitude, expressed*/
int32_t mav_veh_lon;      /*< [degE7] Longitude, expressed*/
int32_t mav_veh_alt;      /*< [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.*/
int32_t g_mav_veh_rel_alt;  /*< [mm] Altitude above ground*/
int16_t mav_veh_gps_vx;   /*< [cm/s] Ground X Speed (Latitude, positive north)*/
int16_t mav_veh_gps_vy;   /*< [cm/s] Ground Y Speed (Longitude, positive east)*/
int16_t mav_veh_gps_vz;   /*< [cm/s] Ground Z Speed (Altitude, positive down)*/
uint16_t mav_veh_gps_hdg; /*< [cdeg] Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/

float mav_veh_roll;       /*< [rad] Roll angle (-pi..+pi)*/
float mav_veh_pitch;      /*< [rad] Pitch angle (-pi..+pi)*/
float mav_veh_yaw;        /*< [rad] Yaw angle (-pi..+pi)*/
float mav_veh_rollspeed;  /*< [rad/s] Roll angular speed*/
float mav_veh_pitchspeed; /*< [rad/s] Pitch angular speed*/
float mav_veh_yawspeed;   /*< [rad/s] Yaw angular speed*/

int16_t mav_veh_imu_ax;    /*< [mG] X acceleration*/
int16_t mav_veh_imu_ay;    /*< [mG] Y acceleration*/
int16_t mav_veh_imu_az;    /*< [mG] Z acceleration*/
int16_t mav_veh_imu_xgyro; /*< [mrad/s] Angular speed around X axis*/
int16_t mav_veh_imu_ygyro; /*< [mrad/s] Angular speed around Y axis*/
int16_t mav_veh_imu_zgyro; /*< [mrad/s] Angular speed around Z axis*/
int16_t mav_veh_imu_xmag;  /*< [mgauss] X Magnetic field*/
int16_t mav_veh_imu_ymag;  /*< [mgauss] Y Magnetic field*/
int16_t mav_veh_imu_zmag;  /*< [mgauss] Z Magnetic field*/

float mav_veh_q1_target;         /*<  Quaternion component 1, w (1 in null-rotation)*/
float mav_veh_q2_target;         /*<  Quaternion component 2, x (0 in null-rotation)*/
float mav_veh_q3_target;         /*<  Quaternion component 3, y (0 in null-rotation)*/
float mav_veh_q4_target;         /*<  Quaternion component 4, z (0 in null-rotation)*/
float mav_veh_roll_rate_target;  /*< [rad/s] Roll angular speed*/
float mav_veh_pitch_rate_target; /*< [rad/s] Pitch angular speed*/
float mav_veh_yaw_rate_target;   /*< [rad/s] Yaw angular speed*/
float mav_veh_thrust_target;     /*<  Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)*/
uint8_t attitude_target_mask;    /*<  Bitmap to indicate which dimensions should be ignored by the vehicle.*/

float mav_veh_q1_actual;         /*<  Quaternion component 1, w (1 in null-rotation)*/
float mav_veh_q2_actual;         /*<  Quaternion component 2, x (0 in null-rotation)*/
float mav_veh_q3_actual;         /*<  Quaternion component 3, y (0 in null-rotation)*/
float mav_veh_q4_actual;         /*<  Quaternion component 4, z (0 in null-rotation)*/
float mav_veh_roll_rate_actual;  /*< [rad/s] Roll angular speed*/
float mav_veh_pitch_rate_actual; /*< [rad/s] Pitch angular speed*/
float mav_veh_yaw_rate_actual;   /*< [rad/s] Yaw angular speed*/
float mav_veh_repr_offset_q[4];  /*<  Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)*/

uint16_t mav_veh_rngfdr_min_distance;     /*< [cm] Minimum distance the sensor can measure*/
uint16_t mav_veh_rngfdr_max_distance;     /*< [cm] Maximum distance the sensor can measure*/
uint16_t g_mav_veh_rngfdr_current_distance; /*< [cm] Current distance reading*/
uint8_t mav_veh_rngfdr_type;              /*<  Type of distance sensor.*/
uint8_t mav_veh_rngfdr_id;                /*<  Onboard ID of the sensor*/
uint8_t mav_veh_rngfdr_orientation;       /*<  Direction the sensor faces. downward-facing: ROTATION_PITCH_270, upward-facing: ROTATION_PITCH_90, backward-facing: ROTATION_PITCH_180, forward-facing: ROTATION_NONE, left-facing: ROTATION_YAW_90, right-facing: ROTATION_YAW_270*/
uint8_t mav_veh_rngfdr_covariance;        /*< [cm^2] Measurement variance. Max standard deviation is 6cm. UINT8_MAX if unknown.*/
float mav_veh_rngfdr_horizontal_fov;      /*< [rad] Horizontal Field of View (angle) where the distance measurement is valid and the field of view is known. Otherwise this is set to 0.*/
float mav_veh_rngfdr_vertical_fov;        /*< [rad] Vertical Field of View (angle) where the distance measurement is valid and the field of view is known. Otherwise this is set to 0.*/
float mav_veh_rngfdr_quaternion[4];       /*<  Quaternion of the sensor orientation in vehicle body frame (w, x, y, z order, zero-rotation is 1, 0, 0, 0). Zero-rotation is along the vehicle body x-axis. This field is required if the orientation is set to MAV_SENSOR_ROTATION_CUSTOM. Set it to 0 if invalid."*/
uint8_t mav_veh_rngfdr_signal_quality;    /*< [%] Signal quality of the sensor. Specific to each sensor type, representing the relation of the signal strength with the target reflectivity, distance, size or aspect, but normalised as a percentage. 0 = unknown/unset signal quality, 1 = invalid signal, 100 = perfect signal.*/

float mav_veh_flow_comp_m_x;   /*< [m/s] Flow in x-sensor direction, angular-speed compensated*/
float mav_veh_flow_comp_m_y;   /*< [m/s] Flow in y-sensor direction, angular-speed compensated*/
float mav_veh_ground_distance; /*< [m] Ground distance. Positive value: distance known. Negative value: Unknown distance*/
int16_t mav_veh_flow_x;        /*< [dpix] Flow in x-sensor direction*/
int16_t mav_veh_flow_y;        /*< [dpix] Flow in y-sensor direction*/
uint8_t mav_veh_sensor_id;     /*<  Sensor ID*/
uint8_t mav_veh_flow_quality;  /*<  Optical flow quality / confidence. 0: bad, 255: maximum quality*/
float mav_veh_flow_rate_x;     /*< [rad/s] Flow rate about X axis*/
float mav_veh_flow_rate_y;     /*< [rad/s] Flow rate about Y axis*/

float mav_veh_local_ned_x;  /*< [m] X Position*/
float mav_veh_local_ned_y;  /*< [m] Y Position*/
float mav_veh_local_ned_z;  /*< [m] Z Position*/
float mav_veh_local_ned_vx; /*< [m/s] X Speed*/
float mav_veh_local_ned_vy; /*< [m/s] Y Speed*/
float mav_veh_local_ned_vz; /*< [m/s] Z Speed*/

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/
bool start_mav_comm(void) { return Serial::start_uart_comm(); }; // Open up uart port for mavlink messages
void stop_mav_comm(void) { Serial::stop_uart_comm(); };          // Stop mavlink comms on uart port
uint8_t read_mav_msg(void) { return Serial::read_uart(); };      // Read a byte
void subscribe(uint16_t msg_id, float msg_interval);                 // Subscribe to a mavlink message at desired rate
void set_mav_msg_rate(uint16_t msg_id, float msg_interval) { MavCmd::set_mav_msg_rate(SENDER_SYS_ID, SENDER_COMP_ID, TARGET_SYS_ID, TARGET_COMP_ID, msg_id, msg_interval); };
void req_mav_msg(uint16_t msg_id) { MavCmd::req_mav_msg(SENDER_SYS_ID, SENDER_COMP_ID, TARGET_SYS_ID, TARGET_COMP_ID, msg_id); };
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

/********************************************************************************
 * Function: subscribe
 * Description: Subscribe to mavlink messages and specify
 *              the desired to receive the message at.
 ********************************************************************************/
void subscribe(uint16_t msg_id, float msg_interval)
{
    set_mav_msg_rate(msg_id, msg_interval);
    req_mav_msg(msg_id);
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
        mav_veh_type = heartbeat.type;
        mav_veh_autopilot_type = heartbeat.autopilot;
        mav_veh_base_mode = heartbeat.base_mode;
        mav_veh_custom_mode = heartbeat.custom_mode;
        mav_veh_state = heartbeat.system_status;
        mav_veh_mavlink_version = heartbeat.mavlink_version;
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

    mav_veh_lat = global_pos_int.lat;
    mav_veh_lon = global_pos_int.lon;
    mav_veh_alt = global_pos_int.alt;
    g_mav_veh_rel_alt = global_pos_int.relative_alt;
    mav_veh_gps_vx = global_pos_int.vx;
    mav_veh_gps_vy = global_pos_int.vy;
    mav_veh_gps_vz = global_pos_int.vz;
    mav_veh_gps_hdg = global_pos_int.hdg;

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
    mavlink_param_value_t param_value;
    mavlink_msg_param_value_decode(msg, &param_value);

#ifdef DEBUG_BUILD

    print_param_value(param_value);

#endif // DEBUG_BUILD
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

    mav_veh_imu_ax = scaled_imu.xacc;
    mav_veh_imu_ay = scaled_imu.yacc;
    mav_veh_imu_az = scaled_imu.zacc;
    mav_veh_imu_xgyro = scaled_imu.xgyro;
    mav_veh_imu_ygyro = scaled_imu.ygyro;
    mav_veh_imu_zgyro = scaled_imu.zgyro;
    mav_veh_imu_xmag = scaled_imu.xmag;
    mav_veh_imu_ymag = scaled_imu.ymag;
    mav_veh_imu_zmag = scaled_imu.zmag;

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

    mav_veh_local_ned_x = position_local_ned.x;
    mav_veh_local_ned_y = position_local_ned.y;
    mav_veh_local_ned_z = position_local_ned.z;
    mav_veh_local_ned_vx = position_local_ned.vx;
    mav_veh_local_ned_vy = position_local_ned.vy;
    mav_veh_local_ned_vz = position_local_ned.vz;

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

    mav_veh_roll = attitude.roll;
    mav_veh_pitch = attitude.pitch;
    mav_veh_yaw = attitude.yaw;
    mav_veh_rollspeed = attitude.rollspeed;
    mav_veh_pitchspeed = attitude.pitchspeed;
    mav_veh_yawspeed = attitude.yawspeed;

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

    mav_veh_q1_actual = attitude_quaternion.q1;
    mav_veh_q2_actual = attitude_quaternion.q2;
    mav_veh_q3_actual = attitude_quaternion.q3;
    mav_veh_q4_actual = attitude_quaternion.q4;
    mav_veh_roll_rate_actual = attitude_quaternion.rollspeed;
    mav_veh_pitch_rate_actual = attitude_quaternion.pitchspeed;
    mav_veh_yaw_rate_actual = attitude_quaternion.yawspeed;
    mav_veh_repr_offset_q[4] = attitude_quaternion.repr_offset_q[4];

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

    mav_veh_command_id = command_ack.command;
    mav_veh_command_result = command_ack.result;
    mav_veh_command_progress = command_ack.progress;
    mav_veh_command_result_param2 = command_ack.result_param2;
    mav_veh_command_target_system = command_ack.target_system;
    mav_veh_command_target_component = command_ack.target_component;

    print_command_ack(command_ack);
}

/********************************************************************************
 * Function: proc_mav_optical_flow_msg
 * Description: Decode optical flow message and optionally print the information.
 ********************************************************************************/
void proc_mav_optical_flow_msg(const mavlink_message_t *msg)
{
    mavlink_optical_flow_t optical_flow;
    mavlink_msg_optical_flow_decode(msg, &optical_flow);

    mav_veh_flow_comp_m_x = optical_flow.flow_comp_m_x;
    mav_veh_flow_comp_m_y = optical_flow.flow_comp_m_y;
    mav_veh_ground_distance = optical_flow.ground_distance;
    mav_veh_flow_x = optical_flow.flow_x;
    mav_veh_flow_y = optical_flow.flow_y;
    mav_veh_sensor_id = optical_flow.sensor_id;
    mav_veh_flow_quality = optical_flow.quality;
    mav_veh_flow_rate_x = optical_flow.flow_rate_x;
    mav_veh_flow_rate_y = optical_flow.flow_rate_y;

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

    mav_veh_rngfdr_min_distance = distance_sensor.min_distance;
    mav_veh_rngfdr_max_distance = distance_sensor.max_distance;
    g_mav_veh_rngfdr_current_distance = distance_sensor.current_distance;
    mav_veh_rngfdr_type = distance_sensor.type;
    mav_veh_rngfdr_id = distance_sensor.id;
    mav_veh_rngfdr_orientation = distance_sensor.orientation;
    mav_veh_rngfdr_covariance = distance_sensor.covariance;
    mav_veh_rngfdr_horizontal_fov = distance_sensor.horizontal_fov;
    mav_veh_rngfdr_vertical_fov = distance_sensor.vertical_fov;
    mav_veh_rngfdr_quaternion[4] = distance_sensor.quaternion[4];
    mav_veh_rngfdr_signal_quality = distance_sensor.signal_quality;

#ifdef DEBUG_BUILD

    print_distance_sensor(distance_sensor);

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

    int n = Serial::bytes_available();

    const char *term = "/dev/pts/0";

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
                printf("Received message with ID: %u\n",msg.msgid);
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
    mav_veh_command_id = 0;
    mav_veh_command_result = 0;
    mav_veh_command_progress = 0;
    mav_veh_command_result_param2 = 0;
    mav_veh_command_target_system = 0;
    mav_veh_command_target_component = 0;
    mav_veh_sys_stat_onbrd_cntrl_snsrs_present = 0;
    mav_veh_sys_stat_onbrd_cntrl_snsrs_enabled = 0;
    mav_veh_sys_stat_onbrd_cntrl_snsrs_health = 0;

    mav_veh_sys_stat_load = 0;
    mav_veh_sys_stat_voltage_battery = 0;
    mav_veh_sys_stat_current_battery = 0;
    mav_veh_sys_stat_drop_rate_comm = 0;

    mav_veh_sys_stat_errors_comm = 0;
    mav_veh_sys_stat_errors_count1 = 0;
    mav_veh_sys_stat_errors_count2 = 0;
    mav_veh_sys_stat_errors_count3 = 0;
    mav_veh_sys_stat_errors_count4 = 0;
    mav_veh_sys_stat_battery_remaining = 0;
    mav_veh_sys_stat_onbrd_cntrl_snsrs_prsnt_extnd = 0;
    mav_veh_sys_stat_onbrd_cntrl_snsrs_enbld_extnd = 0;
    mav_veh_sys_stat_onbrd_cntrl_snsrs_health_extnd = 0;

    mav_veh_custom_mode = 0;
    mav_veh_type = 0;
    mav_veh_autopilot_type = 0;
    mav_veh_base_mode = 0;
    mav_veh_state = 0;
    mav_veh_mavlink_version = 0;

    mav_veh_lat = 0;
    mav_veh_lon = 0;
    mav_veh_alt = 0;
    g_mav_veh_rel_alt = 0;
    mav_veh_gps_vx = 0;
    mav_veh_gps_vy = 0;
    mav_veh_gps_vz = 0;
    mav_veh_gps_hdg = 0;

    mav_veh_roll = 0.0;
    mav_veh_pitch = 0.0;
    mav_veh_yaw = 0.0;
    mav_veh_rollspeed = 0.0;
    mav_veh_pitchspeed = 0.0;
    mav_veh_yawspeed = 0.0;

    mav_veh_imu_ax = 0;
    mav_veh_imu_ay = 0;
    mav_veh_imu_az = 0;
    mav_veh_imu_xgyro = 0;
    mav_veh_imu_ygyro = 0;
    mav_veh_imu_zgyro = 0;
    mav_veh_imu_xmag = 0;
    mav_veh_imu_ymag = 0;
    mav_veh_imu_zmag = 0;

    mav_veh_q1_target = 0.0;
    mav_veh_q2_target = 0.0;
    mav_veh_q3_target = 0.0;
    mav_veh_q4_target = 0.0;
    mav_veh_roll_rate_target = 0.0;
    mav_veh_pitch_rate_target = 0.0;
    mav_veh_yaw_rate_target = 0.0;
    mav_veh_thrust_target = 0.0;
    attitude_target_mask = 0;

    mav_veh_q1_actual = 0.0;
    mav_veh_q2_actual = 0.0;
    mav_veh_q3_actual = 0.0;
    mav_veh_q4_actual = 0.0;
    mav_veh_roll_rate_actual = 0.0;
    mav_veh_pitch_rate_actual = 0.0;
    mav_veh_yaw_rate_actual = 0.0;
    mav_veh_repr_offset_q[0] = 0.0;
    mav_veh_repr_offset_q[1] = 0.0;
    mav_veh_repr_offset_q[2] = 0.0;
    mav_veh_repr_offset_q[3] = 0.0;

    mav_veh_rngfdr_min_distance = 0;
    mav_veh_rngfdr_max_distance = 0;
    g_mav_veh_rngfdr_current_distance = 0;
    mav_veh_rngfdr_type = 0;
    mav_veh_rngfdr_id = 0;
    mav_veh_rngfdr_orientation = 0;
    mav_veh_rngfdr_covariance = 0;
    mav_veh_rngfdr_horizontal_fov = 0.0;
    mav_veh_rngfdr_vertical_fov = 0.0;
    mav_veh_rngfdr_quaternion[0] = 0.0;
    mav_veh_rngfdr_quaternion[1] = 0.0;
    mav_veh_rngfdr_quaternion[2] = 0.0;
    mav_veh_rngfdr_quaternion[3] = 0.0;
    mav_veh_rngfdr_signal_quality = 0;

    if (!start_mav_comm() ||
        !start_message_subscriptions())
    {
        Print::c_fprintf("Failed to initialize MAVLink communication");
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
