/********************************************************************************
 * @file    mavlink_cmd_handler.h
 * @author  Cameron Rose
 * @date    12/27/2023
 * @brief   Mavlink command handler header
 ********************************************************************************/
#ifndef MAVLINK_COMMAND_HANDLER_H
#define MAVLINK_COMMAND_HANDLER_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "serial_port_handler.h"
#include "velocity_controller.h"

/********************************************************************************
 * Imported objects
 ********************************************************************************/
extern const uint8_t SENDER_SYS_ID;
extern const uint8_t SENDER_COMP_ID;
extern const uint8_t TARGET_SYS_ID;
extern const uint8_t TARGET_COMP_ID;
extern const int32_t MESSAGE_RATE_DEFAULT;
extern const int32_t MESSAGE_RATE_1000us;

/********************************************************************************
 * Exported objects
 ********************************************************************************/

/********************************************************************************
 * Function prototypes
 ********************************************************************************/
// Auto Pilot Modes enumeration - same as in ArduPilot firmware
enum class FlightMode : uint8_t 
{
    STABILIZE =     0,  // manual airframe angle with manual throttle
    ACRO =          1,  // manual body-frame angular rate with manual throttle
    ALT_HOLD =      2,  // manual airframe angle with automatic throttle
    AUTO =          3,  // fully automatic waypoint control using mission commands
    GUIDED =        4,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
    LOITER =        5,  // automatic horizontal acceleration with automatic throttle
    RTL =           6,  // automatic return to launching point
    CIRCLE =        7,  // automatic circular flight with automatic throttle
    LAND =          9,  // automatic landing with horizontal position control
    DRIFT =        11,  // semi-autonomous position, yaw and throttle control
    SPORT =        13,  // manual earth-frame angular rate control with manual throttle
    FLIP =         14,  // automatically flip the vehicle on the roll axis
    AUTOTUNE =     15,  // automatically tune the vehicle's roll and pitch gains
    POSHOLD =      16,  // automatic position hold with manual override, with automatic throttle
    BRAKE =        17,  // full-brake using inertial/GPS system, no pilot input
    THROW =        18,  // throw to launch mode using inertial/GPS system, no pilot input
    AVOID_ADSB =   19,  // automatic avoidance of obstacles in the macro scale - e.g. full-sized aircraft
    GUIDED_NOGPS = 20,  // guided mode but only accepts attitude and altitude
    SMART_RTL =    21,  // SMART_RTL returns to home by retracing its steps
    FLOWHOLD  =    22,  // FLOWHOLD holds position with optical flow without rangefinder
    FOLLOW    =    23,  // follow attempts to follow another vehicle or ground station
    ZIGZAG    =    24,  // ZIGZAG mode is able to fly in a zigzag manner with predefined point A and point B
    SYSTEMID  =    25,  // System ID mode produces automated system identification signals in the controllers
    AUTOROTATE =   26,  // Autonomous autorotation
    AUTO_RTL =     27,  // Auto RTL, this is not a true mode, AUTO will report as this mode if entered to perform a DO_LAND_START Landing sequence
    TURTLE =       28,  // Flip over after crash

    // Mode number 127 reserved for the "drone show mode" in the Skybrush
    // fork at https://github.com/skybrush-io/ardupilot
};

// Public member functions                                          
void takeoff_sequence(float takeoff_alt);
void arm_vehicle(void);
void disarm_vehicle(void);
void takeoff_LOCAL(float pitch, float ascend_rate, float yaw,int32_t x, int32_t y, float z);
void takeoff_GPS_long(float alt);
void takeoff_GPS(float alt);
void set_mode_GUIDED(void);
void set_mode_RTL(void);
void go_to_waypoint(int32_t lat, int32_t lon, float alt);
void set_flight_mode(uint8_t confirmation, float mode, float custom_mode, float custom_submode);
void set_mav_msg_rate(uint16_t mavlink_command, uint16_t msg_id, float msg_interval);                                           // For setting a message rate - command, msg id, message rate
void req_mav_msg(uint16_t mavlink_command, uint16_t msg_id);                                                                    // For requesting a mavlink message - command, msg id
void send_cmd_long(uint16_t mavlink_command, uint8_t confirmation, 
                float param1, float param2, float param3, float param4, float param5, float param6, float param7);              // command, confirmation, param1 - param7
void send_cmd_int(uint8_t frame, uint16_t command, 
                float param1, float param2, float param3, float param4, int32_t x, int32_t y, float z);
void send_cmd_set_position_target_global_int(uint8_t coordinate_frame, uint16_t type_mask, int32_t lat_int, int32_t lon_int, 
                float alt, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float yaw_rate);           // coordinate_frame, type_mask, lat_int, lon_int, alt, vx, vy, vz, afx, afy, afz, yaw, yaw_rate
void send_cmd_set_attitude_target(mavlink_set_attitude_target_t *desired_attitude_target);                                      // type_mask, *q, body_roll_rate, body_pitch_rate, body_yaw_rate, thrust, *thrust_body
void send_cmd_set_position_target_local_ned(mavlink_set_position_target_local_ned_t *desired_target);                           // coordinate_frame, type_mask, x, y, z, vx, vy, vz, afx, afy, afz, yaw, yaw_rate

class MavCmd 
{
public:
    MavCmd();
    ~MavCmd();

    // Public member variables 

private:
    // Private member variables
    
    // Create instance of external classes whose functionality is needed
    //SerialPort serial;

    // Private member functions
};


#endif // MAVLINK_COMMAND_HANDLER_H