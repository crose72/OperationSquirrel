/********************************************************************************
 * @file    mavlink_cmd_handler.cpp
 * @author  Cameron Rose
 * @date    6/7/2023
 * @brief   Abstracts and simplifies the mavlink commands for use throughout the
 *          program.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "mavlink_cmd_handler.h"

/********************************************************************************
 * Typedefs
 ********************************************************************************/
// Auto Pilot Modes enumeration - same as in ArduPilot firmware
enum class FlightMode : uint8_t
{
    STABILIZE = 0,     // manual airframe angle with manual throttle
    ACRO = 1,          // manual body-frame angular rate with manual throttle
    ALT_HOLD = 2,      // manual airframe angle with automatic throttle
    AUTO = 3,          // fully automatic waypoint control using mission commands
    GUIDED = 4,        // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
    LOITER = 5,        // automatic horizontal acceleration with automatic throttle
    RTL = 6,           // automatic return to launching point
    CIRCLE = 7,        // automatic circular flight with automatic throttle
    LAND = 9,          // automatic landing with horizontal position control
    DRIFT = 11,        // semi-autonomous position, yaw and throttle control
    SPORT = 13,        // manual earth-frame angular rate control with manual throttle
    FLIP = 14,         // automatically flip the vehicle on the roll axis
    AUTOTUNE = 15,     // automatically tune the vehicle's roll and pitch gains
    POSHOLD = 16,      // automatic position hold with manual override, with automatic throttle
    BRAKE = 17,        // full-brake using inertial/GPS system, no pilot input
    THROW = 18,        // throw to launch mode using inertial/GPS system, no pilot input
    AVOID_ADSB = 19,   // automatic avoidance of obstacles in the macro scale - e.g. full-sized aircraft
    GUIDED_NOGPS = 20, // guided mode but only accepts attitude and altitude
    SMART_RTL = 21,    // SMART_RTL returns to home by retracing its steps
    FLOWHOLD = 22,     // FLOWHOLD holds position with optical flow without rangefinder
    FOLLOW = 23,       // follow attempts to follow another vehicle or ground station
    ZIGZAG = 24,       // ZIGZAG mode is able to fly in a zigzag manner with predefined point A and point B
    SYSTEMID = 25,     // System ID mode produces automated system identification signals in the controllers
    AUTOROTATE = 26,   // Autonomous autorotation
    AUTO_RTL = 27,     // Auto RTL, this is not a true mode, AUTO will report as this mode if entered to perform a DO_LAND_START Landing sequence
    TURTLE = 28,       // Flip over after crash

    // Mode number 127 reserved for the "drone show mode" in the Skybrush
    // fork at https://github.com/skybrush-io/ardupilot
};

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/
mavlink_command_long_t arm_command;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
 * Function: MavCmd
 * Description: Class constructor
 ********************************************************************************/
MavCmd::MavCmd() {};

/********************************************************************************
 * Function: ~MavCmd
 * Description: Class destructor
 ********************************************************************************/
MavCmd::~MavCmd() {};

/********************************************************************************
 * Function: takeoff_LOCAL
 * Description: Takekoff to specified height.
 ********************************************************************************/
void MavCmd::takeoff_LOCAL(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, int32_t x, int32_t y, float z)
{
    mavlink_command_int_t command_int;

    command_int.target_system = target_sys_id;
    command_int.target_component = target_sys_id;
    command_int.frame = MAV_FRAME_BODY_OFFSET_NED;
    command_int.command = MAV_CMD_NAV_TAKEOFF_LOCAL;
    command_int.x = x;
    command_int.y = y;
    command_int.z = z;

    send_cmd_int(sender_sys_id, sender_comp_id, command_int);
}

/********************************************************************************
 * Function: takeoff_GPS_long
 * Description: Takekoff to specified height using command_long->
 ********************************************************************************/
void MavCmd::takeoff_GPS_long(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, float alt)
{
    send_cmd_long(sender_sys_id, sender_comp_id, target_sys_id, target_comp_id, MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt);
}

/********************************************************************************
 * Function: arm_vehicle
 * Description: Arm vehicle.
 ********************************************************************************/
void MavCmd::arm_vehicle(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id)
{
    // Arm throttle - param2 = 0 requires safety checks, param2 = 21196 allows
    // arming to override preflight checks and disarming in flight
    send_cmd_long(sender_sys_id, sender_comp_id, target_sys_id, target_comp_id, MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0);
}

/********************************************************************************
 * Function: disarm_vehicle
 * Description: Disarm vehicle.
 ********************************************************************************/
void MavCmd::disarm_vehicle(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id)
{
    // Disarm throttle - param2 = 0 requires safety checks, param2 = 21196 allows
    // arming to override preflight checks and disarming in flight
    send_cmd_long(sender_sys_id, sender_comp_id, target_sys_id, target_comp_id, MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 1, 0, 0, 0, 0, 0);
}

/********************************************************************************
 * Function: set_mode_LAND
 * Description: Change mode to LAND.
 ********************************************************************************/
void MavCmd::set_mode_LAND(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id)
{
    set_flight_mode(sender_sys_id, sender_comp_id, target_sys_id, target_comp_id, 
                             0, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, (float)FlightMode::LAND, 0);
}

/********************************************************************************
 * Function: set_mode_GUIDED
 * Description: Change mode to GUIDED to allow control from a companion computer.
 ********************************************************************************/
void MavCmd::set_mode_GUIDED(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id)
{
    set_flight_mode(sender_sys_id, sender_comp_id, target_sys_id, target_comp_id, 
                            0, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, (float)FlightMode::GUIDED, 0);
}

/********************************************************************************
 * Function: set_mode_GUIDED_NOGPS
 * Description: Change mode to GUIDED_NOGPS to allow control from a companion computer.
 ********************************************************************************/
void MavCmd::set_mode_GUIDED_NOGPS(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id)
{
    set_flight_mode(sender_sys_id, sender_comp_id, target_sys_id, target_comp_id, 
                            0, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, (float)FlightMode::GUIDED_NOGPS, 0);
}

/********************************************************************************
 * Function: set_mode_RTL
 * Description: Change mode to RTL and vehicle will return to launch location.
 ********************************************************************************/
void MavCmd::set_mode_RTL(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id)
{
    set_flight_mode(sender_sys_id, sender_comp_id, target_sys_id, target_comp_id, 
                            0, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, (float)FlightMode::RTL, 0);
}

/********************************************************************************
 * Function: go_to_waypoint
 * Description: Commmand vehicle to move to a specified GPS location with
 *              specific latitude, longtitude, and altitude.
 ********************************************************************************/
void MavCmd::go_to_waypoint(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, int32_t lat, int32_t lon, float alt)
{
    mavlink_set_position_target_global_int_t position_target_glob_int;

    position_target_glob_int.target_system = target_sys_id;
    position_target_glob_int.target_component = target_comp_id;
    position_target_glob_int.lat_int = lat;
    position_target_glob_int.lon_int = lon;
    position_target_glob_int.alt = alt;
    position_target_glob_int.coordinate_frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    position_target_glob_int.type_mask = (uint16_t)0b111111111000;

   
    send_cmd_set_position_target_global_int (sender_sys_id, sender_comp_id, &position_target_glob_int); // mavlink_msg_command_int_pack(SENDER_SYS_ID, SENDER_COMP_ID, &msg, TARGET_SYS_ID, TARGET_COMP_ID, MAV_CMD_DO_SET_MODE, 0, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6, 0, 0, -35.3671 * 1e7, 149.1649 * 1e7, 0); is used for AUTO mode?
}

/********************************************************************************
 * Function: set_flight_mode
 * Description: Set the vehicle flight mode
 ********************************************************************************/
void MavCmd::set_flight_mode(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, 
                             uint8_t confirmation, float mode, float custom_mode, float custom_submode)
{
    send_cmd_long(sender_sys_id, sender_comp_id, target_sys_id, target_comp_id, MAV_CMD_DO_SET_MODE, confirmation, mode, custom_mode, custom_submode, 0, 0, 0, 0);
}

/********************************************************************************
 * Function: set_mav_msg_rate
 * Description: This function is specifically designed to send a
 *              mavlink command long to request that a mavlink message ID ims
 *              sent at a desired rate.
 ********************************************************************************/
void MavCmd::set_mav_msg_rate(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, uint16_t msg_id, float msg_interval)
{
    send_cmd_long(sender_sys_id, sender_comp_id, target_sys_id, target_comp_id, MAV_CMD_SET_MESSAGE_INTERVAL, 0, msg_id, msg_interval, 0, 0, 0, 0, 1);
}

/********************************************************************************
 * Function: req_mav_msg
 * Description: This overloaded function is specifically designed to send a
 *              mavlink command long torequest that a specific mavlink message
 *              ID is sent.
 ********************************************************************************/
void MavCmd::req_mav_msg(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, uint16_t msg_id)
{
    mavlink_command_long_t* command_long;

    command_long->target_system = target_sys_id;
    command_long->target_component = target_comp_id;
    command_long->command = MAV_CMD_REQUEST_MESSAGE;
    command_long->param1 = msg_id;
    command_long->param7 = 1;

    send_cmd_long(sender_sys_id, sender_comp_id, target_sys_id, target_comp_id, MAV_CMD_REQUEST_MESSAGE, 0, msg_id, 0, 0, 0, 0, 0, 1);
}

/********************************************************************************
 * Function: send_cmd_int
 * Description: This overloaded function allows for various mavlink command
 *              longs to be sent, including a mode change, takeoff, ARM throttle,
 *              and more.
 ********************************************************************************/
void MavCmd::send_cmd_int(uint8_t sender_sys_id, uint8_t sender_comp_id, const mavlink_command_int_t& command_int)
{
    mavlink_message_t msg;
    uint8_t command_int_current = 0;      // no used according to documentation, set 0
    uint8_t command_int_autocontinue = 0; // no used according to documentation, set 0

    mavlink_msg_command_int_encode(sender_sys_id, sender_comp_id, &msg, &command_int);
    send_mav_cmd(msg);
}

/********************************************************************************
 * Function: send_cmd_long
 * Description: This function sends mavlink command
 *              long which can do a mode change, takeoff, ARM throttle,
 *              and more.
 ********************************************************************************/
void MavCmd::send_cmd_long(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, 
                           uint16_t mavlink_command, uint8_t confirmation,
                           float cmd_long_param1, float cmd_long_param2, float cmd_long_param3, float cmd_long_param4, 
                           float cmd_long_param5, float cmd_long_param6, float cmd_long_param7)
{
    mavlink_message_t msg;
    mavlink_command_long_t command_long;

    command_long.target_system = TARGET_SYS_ID;
    command_long.target_component = TARGET_COMP_ID;
    command_long.command = mavlink_command;
    command_long.confirmation = confirmation;
    command_long.param1 = cmd_long_param1;
    command_long.param2 = cmd_long_param2;
    command_long.param3 = cmd_long_param3;
    command_long.param4 = cmd_long_param4;
    command_long.param5 = cmd_long_param5;
    command_long.param6 = cmd_long_param6;
    command_long.param7 = cmd_long_param7;

    mavlink_msg_command_long_encode(SENDER_SYS_ID, SENDER_COMP_ID, &msg, &command_long);
    send_mav_cmd(msg);
}

/********************************************************************************
 * Function: send_cmd_set_position_target_global_int
 * Description: This function uses the mavlink message set position target
 *              global int message to send a vehicle to a specific GPS location
 *              using an external controller.
 ********************************************************************************/
void MavCmd::send_cmd_set_position_target_global_int(uint8_t sender_sys_id, uint8_t sender_comp_id, const mavlink_set_position_target_global_int_t* set_position_target_global_int)
{
    mavlink_message_t msg;

    // There should be an _encode function we want to use instead of the pack function
    mavlink_msg_set_position_target_global_int_encode(sender_sys_id, sender_comp_id, &msg, set_position_target_global_int);
    send_mav_cmd(msg);
}

/********************************************************************************
 * Function: send_cmd_set_attitude_target
 * Description: This function uses the mavlink message set attitude target to
 *              allow for manually controlling the roll, pitch, yaw, and thrust
 *              of a vehicle using an external controller.
 ********************************************************************************/
void MavCmd::send_cmd_set_attitude_target(mavlink_set_attitude_target_t *desired_attitude_target)
{
    mavlink_message_t msg;

    mavlink_msg_set_attitude_target_encode(SENDER_SYS_ID, SENDER_COMP_ID, &msg, desired_attitude_target);
    send_mav_cmd(msg);
}

/********************************************************************************
 * Function: send_cmd_position_target
 * Description: This function accepts an x, y, z vector target and sends a
 *              request to move to move the drone to that desired position.
 ********************************************************************************/
void MavCmd::send_cmd_position_target(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_system, uint8_t target_component, 
                                      float x, float y, float z, float yaw, uint16_t type_mask, uint8_t coordinate_frame)
{
    mavlink_message_t msg;
    mavlink_set_position_target_local_ned_t command_position_target;

    command_position_target.x = x;
    command_position_target.y = y;
    command_position_target.z = z;
    command_position_target.yaw = yaw;
    command_position_target.type_mask = type_mask;
    command_position_target.target_system = target_system;
    command_position_target.target_component = target_component;
    command_position_target.coordinate_frame = coordinate_frame;

    mavlink_msg_set_position_target_local_ned_encode(sender_sys_id, sender_comp_id, &msg, &command_position_target);
    send_mav_cmd(msg);
}

/********************************************************************************
 * Function: send_cmd_position_target
 * Description: This function accepts an x, y, z vector target and sends a
 *              request to move to move the drone to that desired position.
 ********************************************************************************/
void MavCmd::send_cmd_velocity_target(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_system, uint8_t target_component, 
                                      float vx, float vy, float vz, float yaw, uint16_t type_mask, uint8_t coordinate_frame)
{
    mavlink_message_t msg;
    mavlink_set_position_target_local_ned_t command_position_target;

    command_position_target.vx = vx;
    command_position_target.vy = vy;
    command_position_target.vz = vz;
    command_position_target.yaw = yaw;
    command_position_target.type_mask = type_mask;
    command_position_target.target_system = target_system;
    command_position_target.target_component = target_component;
    command_position_target.coordinate_frame = coordinate_frame;

    mavlink_msg_set_position_target_local_ned_encode(sender_sys_id, sender_comp_id, &msg, &command_position_target);
    send_mav_cmd(msg);
}

/********************************************************************************
 * Function: send_cmd_position_target
 * 
 * Description: This function accepts an x, y, z vector target and sends a
 *              request to move to move the drone to that desired position.
 ********************************************************************************/
void MavCmd::send_cmd_acccel_target(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_system, uint8_t target_component, 
                                      float ax, float ay, float az, float yaw, float yaw_rate, uint16_t type_mask, uint8_t coordinate_frame)
{
    send_cmd_set_position_target_local_ned(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ax, ay, az, 
                                           yaw, yaw_rate,type_mask, target_system, target_component, coordinate_frame);
}

/********************************************************************************
 * Function: send_cmd_set_position_target_local_ned
 * Description: This function uses the mavlink message set position target to
 *              allow for manually controlling the position, velocity, or
 *              acceleration of a vehicle using an external controller.
 ********************************************************************************/
void MavCmd::send_cmd_set_position_target_local_ned(float x, float y, float z, float vx, float vy, float vz, 
                                                    float afx, float afy, float afz, float yaw, float yaw_rate,
                                                    uint16_t type_mask, uint8_t target_system, uint8_t target_component, 
                                                    uint8_t coordinate_frame)
{
    mavlink_message_t msg;
    mavlink_set_position_target_local_ned_t command_position_target;

    command_position_target.x = x;
    command_position_target.y = y;
    command_position_target.z = z;
    command_position_target.vx = vx;
    command_position_target.vy = vy;
    command_position_target.vz = vz;
    command_position_target.afx = afx;
    command_position_target.afy = afy;
    command_position_target.afz = afz;
    command_position_target.yaw = yaw;
    command_position_target.yaw_rate = yaw_rate;
    command_position_target.type_mask = type_mask;
    command_position_target.target_system = target_system;
    command_position_target.target_component = target_component;
    command_position_target.coordinate_frame = coordinate_frame;

    mavlink_msg_set_position_target_local_ned_encode(SENDER_SYS_ID, SENDER_COMP_ID, &msg, &command_position_target);
    send_mav_cmd(msg);
}
