/********************************************************************************
 * @file    mavlink_print_info.cpp
 * @author  Cameron Rose
 * @date    12/27/2023
 * @brief   Formatted print statements for mavlink messages.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "mavlink_print_info.h"

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
 * Function: print_heartbeat
 * Description: Print the heartbeat mavlink message.
 ********************************************************************************/
void print_heartbeat(mavlink_heartbeat_t &heartbeat) 
{
    printf("Heartbeat:\n");
    printf("\tType: %d\n", heartbeat.type);
    printf("\tAutopilot: %d\n", heartbeat.autopilot);
    printf("\tBase mode: %d\n", heartbeat.base_mode);
    printf("\tCustom mode: %d\n", heartbeat.custom_mode);
    printf("\tSystem status: %d\n", heartbeat.system_status);
    printf("\tMavlink version: %d\n", heartbeat.mavlink_version);
}

/********************************************************************************
 * Function: print_system_time
 * Description: Print the system time mavlink message.
 ********************************************************************************/
void print_system_time(mavlink_system_time_t &system_time)
{
    printf("System Time:\n");
    printf("\tUnix timestamp (us): %lu\n", system_time.time_unix_usec);
    printf("\tTime since boot (ms): %u\n", system_time.time_boot_ms);
}

/********************************************************************************
 * Function: print_sys_status
 * Description: Print the system status mavlink message.
 ********************************************************************************/
void print_sys_status(mavlink_sys_status_t &sys_status) 
{
    printf("System status:\n");
    printf("\tOnboard control sensors present: %u\n", sys_status.onboard_control_sensors_present);
    printf("\tOnboard control sensors enabled: %u\n", sys_status.onboard_control_sensors_enabled);
    printf("\tOnboard control sensors health: %u\n", sys_status.onboard_control_sensors_health);
    printf("\tLoad: %u%%\n", sys_status.load);
    printf("\tVoltage battery: %u mV\n", sys_status.voltage_battery);
    printf("\tCurrent battery: %d mA\n", sys_status.current_battery);
    printf("\tBattery remaining: %d%%\n", sys_status.battery_remaining);
    printf("\tDrop rate communication: %u%%\n", sys_status.drop_rate_comm);
    printf("\tErrors comm: %u\n", sys_status.errors_comm);
    printf("\tErrors count 1: %u\n", sys_status.errors_count1);
    printf("\tErrors count 2: %u\n", sys_status.errors_count2);
    printf("\tErrors count 3: %u\n", sys_status.errors_count3);
    printf("\tErrors count 4: %u\n", sys_status.errors_count4);
}

/********************************************************************************
 * Function: print_statustext
 * Description: Print the status text mavlink message.
 ********************************************************************************/
void print_statustext(mavlink_statustext_t &statustext) 
{
    printf("STATUSTEXT message:\n");
    printf("\tSeverity: %u\n", statustext.severity);
    printf("\tText: %s\n", statustext.text);
}

/********************************************************************************
 * Function: print_param_value
 * Description: Print the value of a parameter.
 ********************************************************************************/
void print_param_value(mavlink_param_value_t &param_value) 
{
    printf("Parameter Name: %s\n", param_value.param_id);
    printf("\tValue: %.4f\n", param_value.param_value);
    printf("\tType: %d\n", param_value.param_type);
    printf("\tCount: %d\n", param_value.param_count);
    printf("\tIndex: %d\n", param_value.param_index);
}

/********************************************************************************
 * Function: print_autopilot_version
 * Description: Print the autopilot version mavlink message.
 ********************************************************************************/
void print_autopilot_version(mavlink_autopilot_version_t &autopilot_version) 
{
    printf("Autopilot Version:\n");
    printf("\tCapabilities: %lu\n", autopilot_version.capabilities);
    printf("\tFlight Software Version: %lu\n", autopilot_version.flight_sw_version);
    printf("\tMiddleware Software Version: %lu\n", autopilot_version.middleware_sw_version);
    printf("\tOS Software Version: %lu\n", autopilot_version.os_sw_version);
    printf("\tBoard Version: %lu\n", autopilot_version.board_version);
    printf("\tFlight Custom Version: %lu\n", autopilot_version.flight_custom_version);
    printf("\tMiddleware Custom Version: %lu\n", autopilot_version.middleware_custom_version);
    printf("\tOS Custom Version: %lu\n", autopilot_version.os_custom_version);
}

/********************************************************************************
 * Function: print_gps_global_origin
 * Description: Print the gps global origin mavlink message.
 ********************************************************************************/
void print_gps_global_origin(mavlink_gps_global_origin_t &gps_global_origin) 
{
    printf("GPS_GLOBAL_ORIGIN message:\n");
    printf("\tLatitude: %f\n", (double)gps_global_origin.latitude);
    printf("\tLongitude: %f\n", (double)gps_global_origin.longitude);
    printf("\tAltitude: %f\n", (double)gps_global_origin.altitude);
}

/********************************************************************************
 * Function: print_home_position
 * Description: Print the home position mavlink message.
 ********************************************************************************/
void print_home_position(mavlink_home_position_t &home_position) 
{
    printf("HOME_POSITION message:\n");
    printf("\tLatitude: %f\n", (double)home_position.latitude);
    printf("\tLongitude: %f\n", (double)home_position.longitude);
    printf("\tAltitude: %f\n", (double)home_position.altitude);
}

/********************************************************************************
 * Function: print_global_position_int
 * Description: Print the global position integer mavlink message.
 ********************************************************************************/
void print_global_position_int(mavlink_global_position_int_t &global_pos_int) 
{
    printf("Global position:\n");
    printf("\tLatitude: %d degrees (1e-7)\n", global_pos_int.lat);
    printf("\tLongitude: %d degrees (1e-7)\n", global_pos_int.lon);
    printf("\tAltitude: %d mm\n", global_pos_int.alt);
    printf("\tRelative Altitude: %d mm\n", global_pos_int.relative_alt);
    printf("\tVelocity X: %d cm/s\n", global_pos_int.vx);
    printf("\tVelocity Y: %d cm/s\n", global_pos_int.vy);
    printf("\tVelocity Z: %d cm/s\n", global_pos_int.vz);
    printf("\tHeading: %d centi-degrees\n", global_pos_int.hdg);
}

/********************************************************************************
 * Function: print_scaled_imu
 * Description: Print the scaled imu mavlink message.
 ********************************************************************************/
void print_scaled_imu(mavlink_scaled_imu_t &scaled_imu) 
{
    printf("Scaled IMU data:\n");
    printf("\tX acceleration: %d\n", scaled_imu.xacc);
    printf("\tY acceleration: %d\n", scaled_imu.yacc);
    printf("\tZ acceleration: %d\n", scaled_imu.zacc);
    printf("\tX gyro: %d\n", scaled_imu.xgyro);
    printf("\tY gyro: %d\n", scaled_imu.ygyro);
    printf("\tZ gyro: %d\n", scaled_imu.zgyro);
    printf("\tX magnetometer: %d\n", scaled_imu.xmag);
    printf("\tY magnetometer: %d\n", scaled_imu.ymag);
    printf("\tZ magnetometer: %d\n", scaled_imu.zmag);
}

/********************************************************************************
 * Function: print_optical_flow
 * Description: Print the optical flow mavlink message.
 ********************************************************************************/
void print_optical_flow(mavlink_optical_flow_t &optical_flow) 
{
    printf("Optical Flow data:\n");
    printf("\tTimestamp: %llu us\n", optical_flow.time_usec);
    printf("\tSensor ID: %d\n", optical_flow.sensor_id);
    printf("\tFlow in x-sensor direction: %d dpix\n", optical_flow.flow_x);
    printf("\tFlow in y-sensor direction: %d dpix\n", optical_flow.flow_y);
    printf("\tFlow in x-sensor direction (angular-speed compensated): %.2f m/s\n", optical_flow.flow_comp_m_x);
    printf("\tFlow in y-sensor direction (angular-speed compensated): %.2f m/s\n", optical_flow.flow_comp_m_y);
    printf("\tOptical flow quality / confidence: %d\n", optical_flow.quality);
    printf("\tGround distance: %.2f m\n", optical_flow.ground_distance);
    printf("\tFlow rate about X axis: %.2f rad/s\n", optical_flow.flow_rate_x);
    printf("\tFlow rate about Y axis: %.2f rad/s\n", optical_flow.flow_rate_y);
}

/********************************************************************************
 * Function: print_distance_sensor
 * Description: Print the distance sensor mavlink message.
 ********************************************************************************/
void print_distance_sensor(mavlink_distance_sensor_t &distance_sensor) 
{
    printf("Distance Sensor data:\n");
    printf("\tTimestamp: %u ms\n", distance_sensor.time_boot_ms);
    printf("\tMinimum distance: %u cm\n", distance_sensor.min_distance);
    printf("\tMaximum distance: %u cm\n", distance_sensor.max_distance);
    printf("\tCurrent distance reading: %u cm\n", distance_sensor.current_distance);
    printf("\tType of distance sensor: %u\n", distance_sensor.type);
    printf("\tOnboard ID of the sensor: %u\n", distance_sensor.id);
    printf("\tOrientation of the sensor: %u\n", distance_sensor.orientation);
    printf("\tMeasurement variance: %u cm^2\n", distance_sensor.covariance);
    printf("\tHorizontal Field of View: %.2f rad\n", distance_sensor.horizontal_fov);
    printf("\tVertical Field of View: %.2f rad\n", distance_sensor.vertical_fov);
    printf("\tSensor orientation quaternion: [%.2f, %.2f, %.2f, %.2f]\n", 
            distance_sensor.quaternion[0], 
            distance_sensor.quaternion[1], 
            distance_sensor.quaternion[2], 
            distance_sensor.quaternion[3]);
    printf("\tSignal quality: %u%%\n", distance_sensor.signal_quality);
}

/********************************************************************************
 * Function: print_local_position
 * Description: Print the local position mavlink message.
 ********************************************************************************/
void print_local_position(mavlink_local_position_ned_t& local_position) 
{
    printf("Local Position:\n");
    printf("\tTimestamp: %u\n", local_position.time_boot_ms);
    printf("\tPosition (meters):\n");
    printf("\t\tX: %f\n", local_position.x);
    printf("\t\tY: %f\n", local_position.y);
    printf("\t\tZ: %f\n", local_position.z);
    printf("\tVelocity (m/s):\n");
    printf("\t\tVX: %f\n", local_position.vx);
    printf("\t\tVY: %f\n", local_position.vy);
    printf("\t\tVZ: %f\n", local_position.vz);
}

/********************************************************************************
 * Function: print_position_target_local_ned
 * Description: Print the position_target_local_ned mavlink message.
 ********************************************************************************/
void print_position_target_local_ned(mavlink_position_target_local_ned_t& position_target_local_ned) 
{
    printf("Position target local NED: %u\n", position_target_local_ned.time_boot_ms);
    printf("\tTime since boot (ms): %u\n", position_target_local_ned.time_boot_ms);
    printf("\tPosition (X, Y, Z): %.3f, %.3f, %.3f\n", position_target_local_ned.x, position_target_local_ned.y, position_target_local_ned.z);
    printf("\tVelocity (X, Y, Z): %.3f, %.3f, %.3f\n", position_target_local_ned.vx, position_target_local_ned.vy, position_target_local_ned.vz);
    printf("\tAcceleration/Force (X, Y, Z): %.3f, %.3f, %.3f\n", position_target_local_ned.afx, position_target_local_ned.afy, position_target_local_ned.afz);
    printf("\tYaw setpoint: %.3f radians\n", position_target_local_ned.yaw);
    printf("\tYaw rate setpoint: %.3f radians/second\n", position_target_local_ned.yaw_rate);
    printf("\tType mask: %u\n", position_target_local_ned.type_mask);
    printf("\tCoordinate frame: %d\n", position_target_local_ned.coordinate_frame);
}

/********************************************************************************
 * Function: print_set_position_target_local_ned
 * Description: Print the set_position_target_local_ned mavlink message.
 ********************************************************************************/
void print_set_position_target_local_ned(mavlink_set_position_target_local_ned_t& set_position_target_local_ned)
{
    printf("\tTime since boot (ms): %u\n", set_position_target_local_ned.time_boot_ms);
    printf("\tPosition (X, Y, Z): %.3f, %.3f, %.3f\n", set_position_target_local_ned.x, set_position_target_local_ned.y, set_position_target_local_ned.z);
    printf("\tVelocity (X, Y, Z): %.3f, %.3f, %.3f\n", set_position_target_local_ned.vx, set_position_target_local_ned.vy, set_position_target_local_ned.vz);
    printf("\tAcceleration/Force (X, Y, Z): %.3f, %.3f, %.3f\n", set_position_target_local_ned.afx, set_position_target_local_ned.afy, set_position_target_local_ned.afz);
    printf("\tYaw setpoint: %.3f radians\n", set_position_target_local_ned.yaw);
    printf("\tYaw rate setpoint: %.3f radians/second\n", set_position_target_local_ned.yaw_rate);
    printf("\tType mask: %u\n", set_position_target_local_ned.type_mask);
    printf("\tTarget system ID: %d\n", set_position_target_local_ned.target_system);
    printf("\tTarget component ID: %d\n", set_position_target_local_ned.target_component);
    printf("\tCoordinate frame: %d\n", set_position_target_local_ned.coordinate_frame);
}

/********************************************************************************
 * Function: print_attitude
 * Description: Print the attitude mavlink message.
 ********************************************************************************/
void print_attitude(mavlink_attitude_t &attitude) 
{
    printf("Attitude:\n");
    printf("\tRoll: %.4f radians\n", attitude.roll);
    printf("\tPitch: %.4f radians\n", attitude.pitch);
    printf("\tYaw: %.4f radians\n", attitude.yaw);
    printf("\tRoll speed: %.4f rad/s\n", attitude.rollspeed);
    printf("\tPitch speed: %.4f rad/s\n", attitude.pitchspeed);
    printf("\tYaw speed: %.4f rad/s\n", attitude.yawspeed);
}

/********************************************************************************
 * Function: print_attitude_target
 * Description: Print the attitude target mavlink message.
 ********************************************************************************/
void print_attitude_target(mavlink_attitude_target_t &attitude_target) 
{
    printf("Attitude Target:\n");
    printf("\tTime Boot (ms): %lu\n", attitude_target.time_boot_ms);
    printf("\tType Mask: %u\n", attitude_target.type_mask);
    printf("\tQuaternion (q): [%f, %f, %f, %f]\n", attitude_target.q[0], attitude_target.q[1], attitude_target.q[2], attitude_target.q[3]);
    printf("\tBody Roll Rate: %f\n", attitude_target.body_roll_rate);
    printf("\tBody Pitch Rate: %f\n", attitude_target.body_pitch_rate);
    printf("\tBody Yaw Rate: %f\n", attitude_target.body_yaw_rate);
    printf("\tThrust: %f\n", attitude_target.thrust);
}

/********************************************************************************
 * Function: print_set_attitude_target
 * Description: Print the set_attitude_target mavlink message.
 ********************************************************************************/
void print_set_attitude_target(mavlink_set_attitude_target_t &set_attitude_target) 
{
    printf("Set Attitude Target:\n");
    printf("\tTime Boot (ms): %lu\n", set_attitude_target.time_boot_ms);
    printf("\tTarget System: %u\n", set_attitude_target.target_system);
    printf("\tTarget Component: %u\n", set_attitude_target.target_component);
    printf("\tType Mask: %u\n", set_attitude_target.type_mask);
    printf("\tQuaternion (q): [%f, %f, %f, %f]\n", set_attitude_target.q[0], set_attitude_target.q[1], set_attitude_target.q[2], set_attitude_target.q[3]);
    printf("\tBody Roll Rate: %f\n", set_attitude_target.body_roll_rate);
    printf("\tBody Pitch Rate: %f\n", set_attitude_target.body_pitch_rate);
    printf("\tBody Yaw Rate: %f\n", set_attitude_target.body_yaw_rate);
    printf("\tThrust: %f\n", set_attitude_target.thrust);
}

/********************************************************************************
 * Function: print_attitude_quaternion
 * Description: Print the attitude quaternion mavlink message.
 ********************************************************************************/
void print_attitude_quaternion(mavlink_attitude_quaternion_t &attitude_quaternion) 
{
    printf("Attitude Quaternion:\n");
    printf("\tTime Boot (ms): %ul\n", attitude_quaternion.time_boot_ms);
    printf("\tQuaternion (q): [%f, %f, %f, %f]\n", attitude_quaternion.q1, attitude_quaternion.q2, attitude_quaternion.q3, attitude_quaternion.q4);
    printf("\tBody Roll Rate: %f\n", attitude_quaternion.rollspeed);
    printf("\tBody Pitch Rate: %f\n", attitude_quaternion.pitchspeed);
    printf("\tBody Yaw Rate: %f\n", attitude_quaternion.yawspeed);
    printf("\tRotation offset: [%f, %f, %f, %f]\n", attitude_quaternion.repr_offset_q[0], attitude_quaternion.repr_offset_q[1], attitude_quaternion.repr_offset_q[2], attitude_quaternion.repr_offset_q[3]);
}

/********************************************************************************
 * Function: print_command_ack
 * Description: Print the mavlink command received by the autopilot.
 ********************************************************************************/
void print_command_ack(mavlink_command_ack_t &command_ack) 
{
    printf("Command ACK received:\n");
    printf("\tCommand: ");

    switch (command_ack.command)
    {
        case MAV_CMD_NAV_WAYPOINT:
            printf("MAV_CMD_NAV_WAYPOINT\n");
            break;
        case MAV_CMD_NAV_LOITER_UNLIM:
            printf("MAV_CMD_NAV_LOITER_UNLIM\n");
            break;
        case MAV_CMD_NAV_LOITER_TURNS:
            printf("MAV_CMD_NAV_LOITER_TURNS\n");
            break;
        case MAV_CMD_NAV_LOITER_TIME:
            printf("MAV_CMD_NAV_LOITER_TIME\n");
            break;
        case MAV_CMD_NAV_RETURN_TO_LAUNCH:
            printf("MAV_CMD_NAV_RETURN_TO_LAUNCH\n");
            break;
        case MAV_CMD_NAV_LAND:
            printf("MAV_CMD_NAV_LAND\n");
            break;
        case MAV_CMD_NAV_TAKEOFF:
            printf("MAV_CMD_NAV_TAKEOFF\n");
            break;
        case MAV_CMD_NAV_LAND_LOCAL:
            printf("MAV_CMD_NAV_LAND_LOCAL\n");
            break;
        case MAV_CMD_NAV_TAKEOFF_LOCAL:
            printf("MAV_CMD_NAV_TAKEOFF_LOCAL\n");
            break;
        case MAV_CMD_NAV_FOLLOW:
            printf("MAV_CMD_NAV_FOLLOW\n");
            break;
        case MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
            printf("MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT\n");
            break;
        case MAV_CMD_NAV_LOITER_TO_ALT:
            printf("MAV_CMD_NAV_LOITER_TO_ALT\n");
            break;
        case MAV_CMD_DO_FOLLOW:
            printf("MAV_CMD_DO_FOLLOW\n");
            break;
        case MAV_CMD_DO_FOLLOW_REPOSITION:
            printf("MAV_CMD_DO_FOLLOW_REPOSITION\n");
            break;
        case MAV_CMD_DO_ORBIT:
            printf("MAV_CMD_DO_ORBIT\n");
            break;
        case MAV_CMD_NAV_ROI:
            printf("MAV_CMD_NAV_ROI\n");
            break;
        case MAV_CMD_NAV_PATHPLANNING:
            printf("MAV_CMD_NAV_PATHPLANNING\n");
            break;
        case MAV_CMD_NAV_SPLINE_WAYPOINT:
            printf("MAV_CMD_NAV_SPLINE_WAYPOINT\n");
            break;
        case MAV_CMD_NAV_VTOL_TAKEOFF:
            printf("MAV_CMD_NAV_VTOL_TAKEOFF\n");
            break;
        case MAV_CMD_NAV_VTOL_LAND:
            printf("MAV_CMD_NAV_VTOL_LAND\n");
            break;
        case MAV_CMD_NAV_GUIDED_ENABLE:
            printf("MAV_CMD_NAV_GUIDED_ENABLE\n");
            break;
        case MAV_CMD_NAV_DELAY:
            printf("MAV_CMD_NAV_DELAY\n");
            break;
        case MAV_CMD_NAV_PAYLOAD_PLACE:
            printf("MAV_CMD_NAV_PAYLOAD_PLACE\n");
            break;
        case MAV_CMD_NAV_LAST:
            printf("MAV_CMD_NAV_LAST\n");
            break;
        case MAV_CMD_CONDITION_DELAY:
            printf("MAV_CMD_CONDITION_DELAY\n");
            break;
        case MAV_CMD_CONDITION_CHANGE_ALT:
            printf("MAV_CMD_CONDITION_CHANGE_ALT\n");
            break;
        case MAV_CMD_CONDITION_DISTANCE:
            printf("MAV_CMD_CONDITION_DISTANCE\n");
            break;
        case MAV_CMD_CONDITION_YAW:
            printf("MAV_CMD_CONDITION_YAW\n");
            break;
        case MAV_CMD_CONDITION_LAST:
            printf("MAV_CMD_CONDITION_LAST\n");
            break;
        case MAV_CMD_DO_SET_MODE:
            printf("MAV_CMD_DO_SET_MODE\n");
            break;
        case MAV_CMD_DO_JUMP:
            printf("MAV_CMD_DO_JUMP\n");
            break;
        case MAV_CMD_DO_CHANGE_SPEED:
            printf("MAV_CMD_DO_CHANGE_SPEED\n");
            break;
        case MAV_CMD_DO_SET_HOME:
            printf("MAV_CMD_DO_SET_HOME\n");
            break;
        case MAV_CMD_DO_SET_PARAMETER:
            printf("MAV_CMD_DO_SET_PARAMETER\n");
            break;
        case MAV_CMD_DO_SET_RELAY:
            printf("MAV_CMD_DO_SET_RELAY\n");
            break;
        case MAV_CMD_DO_REPEAT_RELAY:
            printf("MAV_CMD_DO_REPEAT_RELAY\n");
            break;
        case MAV_CMD_DO_SET_SERVO:
            printf("MAV_CMD_DO_SET_SERVO\n");
            break;
        case MAV_CMD_DO_REPEAT_SERVO:
            printf("MAV_CMD_DO_REPEAT_SERVO\n");
            break;
        case MAV_CMD_DO_FLIGHTTERMINATION:
            printf("MAV_CMD_DO_FLIGHTTERMINATION\n");
            break;
        case MAV_CMD_DO_CHANGE_ALTITUDE:
            printf("MAV_CMD_DO_CHANGE_ALTITUDE\n");
            break;
        case MAV_CMD_DO_SET_ACTUATOR:
            printf("MAV_CMD_DO_SET_ACTUATOR\n");
            break;
        case MAV_CMD_DO_LAND_START:
            printf("MAV_CMD_DO_LAND_START\n");
            break;
        case MAV_CMD_DO_RALLY_LAND:
            printf("MAV_CMD_DO_RALLY_LAND\n");
            break;
        case MAV_CMD_DO_GO_AROUND:
            printf("MAV_CMD_DO_GO_AROUND\n");
            break;
        case MAV_CMD_DO_REPOSITION:
            printf("MAV_CMD_DO_REPOSITION\n");
            break;
        case MAV_CMD_DO_PAUSE_CONTINUE:
            printf("MAV_CMD_DO_PAUSE_CONTINUE\n");
            break;
        case MAV_CMD_DO_SET_REVERSE:
            printf("MAV_CMD_DO_SET_REVERSE\n");
            break;
        case MAV_CMD_DO_SET_ROI_LOCATION:
            printf("MAV_CMD_DO_SET_ROI_LOCATION\n");
            break;
        case MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET:
            printf("MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET\n");
            break;
        case MAV_CMD_DO_SET_ROI_NONE:
            printf("MAV_CMD_DO_SET_ROI_NONE\n");
            break;
        case MAV_CMD_DO_SET_ROI_SYSID:
            printf("MAV_CMD_DO_SET_ROI_SYSID\n");
            break;
        case MAV_CMD_DO_CONTROL_VIDEO:
            printf("MAV_CMD_DO_CONTROL_VIDEO\n");
            break;
        case MAV_CMD_DO_SET_ROI:
            printf("MAV_CMD_DO_SET_ROI\n");
            break;
        case MAV_CMD_DO_DIGICAM_CONFIGURE:
            printf("MAV_CMD_DO_DIGICAM_CONFIGURE\n");
            break;
        case MAV_CMD_DO_DIGICAM_CONTROL:
            printf("MAV_CMD_DO_DIGICAM_CONTROL\n");
            break;
        case MAV_CMD_DO_MOUNT_CONFIGURE:
            printf("MAV_CMD_DO_MOUNT_CONFIGURE\n");
            break;
        case MAV_CMD_DO_MOUNT_CONTROL:
            printf("MAV_CMD_DO_MOUNT_CONTROL\n");
            break;
        case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
            printf("MAV_CMD_DO_SET_CAM_TRIGG_DIST\n");
            break;
        case MAV_CMD_DO_FENCE_ENABLE:
            printf("MAV_CMD_DO_FENCE_ENABLE\n");
            break;
        case MAV_CMD_DO_PARACHUTE:
            printf("MAV_CMD_DO_PARACHUTE\n");
            break;
        case MAV_CMD_DO_MOTOR_TEST:
            printf("MAV_CMD_DO_MOTOR_TEST\n");
            break;
        case MAV_CMD_DO_INVERTED_FLIGHT:
            printf("MAV_CMD_DO_INVERTED_FLIGHT\n");
            break;
        case MAV_CMD_DO_GRIPPER:
            printf("MAV_CMD_DO_GRIPPER\n");
            break;
        case MAV_CMD_DO_AUTOTUNE_ENABLE:
            printf("MAV_CMD_DO_AUTOTUNE_ENABLE\n");
            break;
        case MAV_CMD_NAV_SET_YAW_SPEED:
            printf("MAV_CMD_NAV_SET_YAW_SPEED\n");
            break;
        case MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
            printf("MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL\n");
            break;
        case MAV_CMD_DO_MOUNT_CONTROL_QUAT:
            printf("MAV_CMD_DO_MOUNT_CONTROL_QUAT\n");
            break;
        case MAV_CMD_DO_GUIDED_MASTER:
            printf("MAV_CMD_DO_GUIDED_MASTER\n");
            break;
        case MAV_CMD_DO_GUIDED_LIMITS:
            printf("MAV_CMD_DO_GUIDED_LIMITS\n");
            break;
        case MAV_CMD_DO_ENGINE_CONTROL:
            printf("MAV_CMD_DO_ENGINE_CONTROL\n");
            break;
        case MAV_CMD_DO_SET_MISSION_CURRENT:
            printf("MAV_CMD_DO_SET_MISSION_CURRENT\n");
            break;
        case MAV_CMD_DO_LAST:
            printf("MAV_CMD_DO_LAST\n");
            break;
        case MAV_CMD_PREFLIGHT_CALIBRATION:
            printf("MAV_CMD_PREFLIGHT_CALIBRATION\n");
            break;
        case MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
            printf("MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS\n");
            break;
        case MAV_CMD_PREFLIGHT_UAVCAN:
            printf("MAV_CMD_PREFLIGHT_UAVCAN\n");
            break;
        case MAV_CMD_PREFLIGHT_STORAGE:
            printf("MAV_CMD_PREFLIGHT_STORAGE\n");
            break;
        case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            printf("MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN\n");
            break;
        case MAV_CMD_OVERRIDE_GOTO:
            printf("MAV_CMD_OVERRIDE_GOTO\n");
            break;
        case MAV_CMD_OBLIQUE_SURVEY:
            printf("MAV_CMD_OBLIQUE_SURVEY\n");
            break;
        case MAV_CMD_MISSION_START:
            printf("MAV_CMD_MISSION_START\n");
            break;
        case MAV_CMD_ACTUATOR_TEST:
            printf("MAV_CMD_ACTUATOR_TEST\n");
            break;
        case MAV_CMD_CONFIGURE_ACTUATOR:
            printf("MAV_CMD_CONFIGURE_ACTUATOR\n");
            break;
        case MAV_CMD_COMPONENT_ARM_DISARM:
            printf("MAV_CMD_COMPONENT_ARM_DISARM\n");
            break;
        case MAV_CMD_RUN_PREARM_CHECKS:
            printf("MAV_CMD_RUN_PREARM_CHECKS\n");
            break;
        case MAV_CMD_ILLUMINATOR_ON_OFF:
            printf("MAV_CMD_ILLUMINATOR_ON_OFF\n");
            break;
        case MAV_CMD_GET_HOME_POSITION:
            printf("MAV_CMD_GET_HOME_POSITION\n");
            break;
        case MAV_CMD_INJECT_FAILURE:
            printf("MAV_CMD_INJECT_FAILURE\n");
            break;
        case MAV_CMD_START_RX_PAIR:
            printf("MAV_CMD_START_RX_PAIR\n");
            break;
        case MAV_CMD_GET_MESSAGE_INTERVAL:
            printf("MAV_CMD_GET_MESSAGE_INTERVAL\n");
            break;
        case MAV_CMD_SET_MESSAGE_INTERVAL:
            printf("MAV_CMD_SET_MESSAGE_INTERVAL\n");
            break;
        case MAV_CMD_REQUEST_MESSAGE:
            printf("MAV_CMD_REQUEST_MESSAGE\n");
            break;
        case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
            printf("MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES\n");
            break;
        case MAV_CMD_SET_CAMERA_MODE:
            printf("MAV_CMD_SET_CAMERA_MODE\n");
            break;
        default:
            printf("Unknown command\n");
            break;
    }

    printf("\tResult: ");

    switch (command_ack.result)
    {
        case 0:
            printf("MAV_RESULT_ACCEPTED\n");
            break;
        case 1:
            printf("MAV_RESULT_TEMPORARILY_REJECTED\n");
            break;
        case 2:
            printf("MAV_RESULT_DENIED\n");
            break;
        case 3:
            printf("MAV_RESULT_UNSUPPORTED\n");
            break;
        case 4:
            printf("MAV_RESULT_FAILED\n");
            break;
        case 5:
            printf("MAV_RESULT_IN_PROGRESS\n");
            break;
        case 6:
            printf("MAV_RESULT_CANCELLED\n");
            break;
        case 7:
            printf("MAV_RESULT_COMMAND_LONG_ONLY\n");
            break;
        case 8:
            printf("MAV_RESULT_COMMAND_INT_ONLY\n");
            break;
        case 9:
            printf("MAV_RESULT_COMMAND_COMMAND_UNSUPPORTED_MAV_FRAME\n");
            break;
        default:
            printf("Unkown result\n");
    }
}
