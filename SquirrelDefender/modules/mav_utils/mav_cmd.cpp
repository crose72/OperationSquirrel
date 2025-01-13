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
#include "mav_cmd.h"

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
 * Function: mav_cmd
 * Description: Class constructor
 ********************************************************************************/
mav_cmd::mav_cmd() {};

/********************************************************************************
 * Function: ~mav_cmd
 * Description: Class destructor
 ********************************************************************************/
mav_cmd::~mav_cmd() {};

/********************************************************************************
 * Function: takeoff_local
 * Description: Takekoff to specified height.
 ********************************************************************************/
void mav_cmd::takeoff_local(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, int32_t x, int32_t y, float z)
{
    mavlink_command_int_t command_int;

    command_int.target_system = target_sys_id;
    command_int.target_component = target_sys_id;
    command_int.frame = MAV_FRAME_BODY_OFFSET_NED;
    command_int.command = MAV_CMD_NAV_takeoff_local;
    command_int.x = x;
    command_int.y = y;
    command_int.z = z;

    send_cmd_int(sender_sys_id, sender_comp_id, command_int);
}

/********************************************************************************
 * Function: takeoff_gps
 * Description: Takekoff to specified height using command_long->
 ********************************************************************************/
void mav_cmd::takeoff_gps(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, float alt)
{
    send_cmd_long(sender_sys_id, sender_comp_id, target_sys_id, target_comp_id, MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt);
}

/********************************************************************************
 * Function: arm_vehicle
 * Description: Arm vehicle.
 ********************************************************************************/
void mav_cmd::arm_vehicle(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id)
{
    // Arm throttle - param2 = 0 requires safety checks, param2 = 21196 allows
    // arming to override preflight checks and disarming in flight
    send_cmd_long(sender_sys_id, sender_comp_id, target_sys_id, target_comp_id, MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0);
}

/********************************************************************************
 * Function: disarm_vehicle
 * Description: Disarm vehicle.
 ********************************************************************************/
void mav_cmd::disarm_vehicle(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id)
{
    // Disarm throttle - param2 = 0 requires safety checks, param2 = 21196 allows
    // arming to override preflight checks and disarming in flight
    send_cmd_long(sender_sys_id, sender_comp_id, target_sys_id, target_comp_id, MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 1, 0, 0, 0, 0, 0);
}

/********************************************************************************
 * Function: set_mode_land
 * Description: Change mode to LAND.
 ********************************************************************************/
void mav_cmd::set_mode_land(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id)
{
    set_flight_mode(sender_sys_id, sender_comp_id, target_sys_id, target_comp_id, 
                             0, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, (float)FlightMode::LAND, 0);
}

/********************************************************************************
 * Function: set_mode_guided
 * Description: Change mode to GUIDED to allow control from a companion computer.
 ********************************************************************************/
void mav_cmd::set_mode_guided(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id)
{
    set_flight_mode(sender_sys_id, sender_comp_id, target_sys_id, target_comp_id, 
                            0, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, (float)FlightMode::GUIDED, 0);
}

/********************************************************************************
 * Function: set_mode_guided_nogps
 * Description: Change mode to GUIDED_NOGPS to allow control from a companion computer.
 ********************************************************************************/
void mav_cmd::set_mode_guided_nogps(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id)
{
    set_flight_mode(sender_sys_id, sender_comp_id, target_sys_id, target_comp_id, 
                            0, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, (float)FlightMode::GUIDED_NOGPS, 0);
}

/********************************************************************************
 * Function: set_mode_rtl
 * Description: Change mode to RTL and vehicle will return to launch location.
 ********************************************************************************/
void mav_cmd::set_mode_rtl(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id)
{
    set_flight_mode(sender_sys_id, sender_comp_id, target_sys_id, target_comp_id, 
                            0, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, (float)FlightMode::RTL, 0);
}

/********************************************************************************
 * Function: set_flight_mode
 * Description: Set the vehicle flight mode
 ********************************************************************************/
void mav_cmd::set_flight_mode(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, 
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
void mav_cmd::set_mav_msg_rate(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, uint16_t msg_id, float msg_interval)
{
    send_cmd_long(sender_sys_id, sender_comp_id, target_sys_id, target_comp_id, MAV_CMD_SET_MESSAGE_INTERVAL, 0, msg_id, msg_interval, 0, 0, 0, 0, 1);
}

/********************************************************************************
 * Function: req_mav_msg
 * Description: This overloaded function is specifically designed to send a
 *              mavlink command long torequest that a specific mavlink message
 *              ID is sent.
 ********************************************************************************/
void mav_cmd::req_mav_msg(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, uint16_t msg_id)
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
void mav_cmd::send_cmd_int(uint8_t sender_sys_id, uint8_t sender_comp_id, const mavlink_command_int_t& command_int)
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
void mav_cmd::send_cmd_long(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, 
                           uint16_t mavlink_command, uint8_t confirmation,
                           float cmd_long_param1, float cmd_long_param2, float cmd_long_param3, float cmd_long_param4, 
                           float cmd_long_param5, float cmd_long_param6, float cmd_long_param7)
{
    mavlink_message_t msg;
    mavlink_command_long_t command_long;

    command_long.target_system = target_sys_id;
    command_long.target_component = target_comp_id;
    command_long.command = mavlink_command;
    command_long.confirmation = confirmation;
    command_long.param1 = cmd_long_param1;
    command_long.param2 = cmd_long_param2;
    command_long.param3 = cmd_long_param3;
    command_long.param4 = cmd_long_param4;
    command_long.param5 = cmd_long_param5;
    command_long.param6 = cmd_long_param6;
    command_long.param7 = cmd_long_param7;

    mavlink_msg_command_long_encode(sender_sys_id, sender_comp_id, &msg, &command_long);
    send_mav_cmd(msg);
}

