/********************************************************************************
 * @file    mav_cmd.h
 * @author  Cameron Rose
 * @date    1/22/2025
 ********************************************************************************/
#ifndef MAV_CMD_H
#define MAV_CMD_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "mav_serial.h"
#include <mavlink.h>
#include <common.h>

/********************************************************************************
 * Typedefs / Enums / Structs
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
 * Function prototypes and Class Definitions
 ********************************************************************************/
class MavCmd
{
public:
    MavCmd();
    ~MavCmd();

    // Public member functions
    static void arm_vehicle(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id);
    static void disarm_vehicle(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id);
    static void takeoff_local(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, int32_t x, int32_t y, float z);
    static void takeoff_gps(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, float alt);
    static void set_mode_land(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id);
    static void set_mode_guided(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id);
    static void set_mode_guided_nogps(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id);
    static void set_mode_rtl(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id);
    static void set_flight_mode(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id,
                                uint8_t confirmation, float mode, float custom_mode, float custom_submode);
    static void set_mav_msg_rate(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, uint16_t msg_id, float msg_interval); // For setting a message rate - command, msg id, message rate
    static void req_mav_msg(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id, uint16_t msg_id);                          // For requesting a mavlink message - command, msg id
    static void send_cmd_long(uint8_t sender_sys_id, uint8_t sender_comp_id, uint8_t target_sys_id, uint8_t target_comp_id,
                              uint16_t mavlink_command, uint8_t confirmation,
                              float cmd_long_param1, float cmd_long_param2, float cmd_long_param3, float cmd_long_param4,
                              float cmd_long_param5, float cmd_long_param6, float cmd_long_param7); // command, confirmation, param1 - param7
    static void send_cmd_int(uint8_t sender_sys_id, uint8_t sender_comp_id, const mavlink_command_int_t &command_int);
    static void read_param(uint8_t sender_sys_id,
                           uint8_t sender_comp_id,
                           uint8_t target_sys_id,
                           uint8_t target_comp_id,
                           const char *param_id,
                           int16_t param_index);

private:
    static void send_mav_cmd(mavlink_message_t &msg) { MavSerial::write_mav_msg(msg); };
    static bool start_mav_comm(void) { return MavSerial::start_mav_comm(); }; // Open up uart port for mavlink messages
    static void stop_mav_comm(void) { MavSerial::stop_mav_comm(); };          // Stop mavlink comms on uart port
    static uint8_t read_mav_msg(void) { return MavSerial::read_mav_msg(); };  // Read a byte
    static void subscribe(uint16_t msg_id, float msg_interval);               // Subscribe to a mavlink message at desired rate
};

#endif // MAV_CMD_H
