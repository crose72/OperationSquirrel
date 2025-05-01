#pragma once

/********************************************************************************
 * @file    sim_drone.h
 * @author  Cameron Rose
 * @date    4/1/2025
 ********************************************************************************/
#ifndef SIM_DRONE_H
#define SIM_DRONE_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "mav_data_hub.h"

/********************************************************************************
 * Imported objects
 ********************************************************************************/
extern int32_t g_mav_veh_lat;      /*< [degE7] Latitude, expressed*/
extern int32_t g_mav_veh_lon;      /*< [degE7] Longitude, expressed*/
extern int32_t g_mav_veh_alt;      /*< [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.*/
extern int32_t g_mav_veh_rel_alt;  /*< [mm] Altitude above ground*/
extern int16_t g_mav_veh_gps_vx;   /*< [cm/s] Ground X Speed (Latitude, positive north)*/
extern int16_t g_mav_veh_gps_vy;   /*< [cm/s] Ground Y Speed (Longitude, positive east)*/
extern int16_t g_mav_veh_gps_vz;   /*< [cm/s] Ground Z Speed (Altitude, positive down)*/
extern uint16_t g_mav_veh_gps_hdg; /*< [cdeg] Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/

extern float g_mav_veh_roll;       /*< [rad] Roll angle (-pi..+pi)*/
extern float g_mav_veh_pitch;      /*< [rad] Pitch angle (-pi..+pi)*/
extern float g_mav_veh_yaw;        /*< [rad] Yaw angle (-pi..+pi)*/
extern float g_mav_veh_rollspeed;  /*< [rad/s] Roll angular speed*/
extern float g_mav_veh_pitchspeed; /*< [rad/s] Pitch angular speed*/
extern float g_mav_veh_yawspeed;   /*< [rad/s] Yaw angular speed*/

extern int16_t g_mav_veh_imu_ax;    /*< [mG] X acceleration*/
extern int16_t g_mav_veh_imu_ay;    /*< [mG] Y acceleration*/
extern int16_t g_mav_veh_imu_az;    /*< [mG] Z acceleration*/
extern int16_t g_mav_veh_imu_xgyro; /*< [mrad/s] Angular speed around X axis*/
extern int16_t g_mav_veh_imu_ygyro; /*< [mrad/s] Angular speed around Y axis*/
extern int16_t g_mav_veh_imu_zgyro; /*< [mrad/s] Angular speed around Z axis*/
extern int16_t g_mav_veh_imu_xmag;  /*< [mgauss] X Magnetic field*/
extern int16_t g_mav_veh_imu_ymag;  /*< [mgauss] Y Magnetic field*/
extern int16_t g_mav_veh_imu_zmag;  /*< [mgauss] Z Magnetic field*/

extern uint16_t g_mav_veh_rngfdr_min_distance;     /*< [cm] Minimum distance the sensor can measure*/
extern uint16_t g_mav_veh_rngfdr_max_distance;     /*< [cm] Maximum distance the sensor can measure*/
extern uint16_t g_mav_veh_rngfdr_current_distance; /*< [cm] Current distance reading*/
extern uint8_t g_mav_veh_rngfdr_type;              /*<  Type of distance sensor.*/
extern uint8_t g_mav_veh_rngfdr_id;                /*<  Onboard ID of the sensor*/
extern uint8_t g_mav_veh_rngfdr_orientation;       /*<  Direction the sensor faces. downward-facing: ROTATION_PITCH_270, upward-facing: ROTATION_PITCH_90, backward-facing: ROTATION_PITCH_180, forward-facing: ROTATION_NONE, left-facing: ROTATION_YAW_90, right-facing: ROTATION_YAW_270*/
extern uint8_t g_mav_veh_rngfdr_covariance;        /*< [cm^2] Measurement variance. Max standard deviation is 6cm. UINT8_MAX if unknown.*/
extern float g_mav_veh_rngfdr_horizontal_fov;      /*< [rad] Horizontal Field of View (angle) where the distance measurement is valid and the field of view is known. Otherwise this is set to 0.*/
extern float g_mav_veh_rngfdr_vertical_fov;        /*< [rad] Vertical Field of View (angle) where the distance measurement is valid and the field of view is known. Otherwise this is set to 0.*/
extern float g_mav_veh_rngfdr_quaternion[4];       /*<  Quaternion of the sensor orientation in vehicle body frame (w, x, y, z order, zero-rotation is 1, 0, 0, 0). Zero-rotation is along the vehicle body x-axis. This field is required if the orientation is set to MAV_SENSOR_ROTATION_CUSTOM. Set it to 0 if invalid."*/
extern uint8_t g_mav_veh_rngfdr_signal_quality;    /*< [%] Signal quality of the sensor. Specific to each sensor type, representing the relation of the signal strength with the target reflectivity, distance, size or aspect, but normalised as a percentage. 0 = unknown/unset signal quality, 1 = invalid signal, 100 = perfect signal.*/

extern float g_mav_veh_flow_comp_m_x;   /*< [m/s] Flow in x-sensor direction, angular-speed compensated*/
extern float g_mav_veh_flow_comp_m_y;   /*< [m/s] Flow in y-sensor direction, angular-speed compensated*/
extern float g_mav_veh_ground_distance; /*< [m] Ground distance. Positive value: distance known. Negative value: Unknown distance*/
extern int16_t g_mav_veh_flow_x;        /*< [dpix] Flow in x-sensor direction*/
extern int16_t g_mav_veh_flow_y;        /*< [dpix] Flow in y-sensor direction*/
extern uint8_t g_mav_veh_sensor_id;     /*<  Sensor ID*/
extern uint8_t g_mav_veh_flow_quality;  /*<  Optical flow quality / confidence. 0: bad, 255: maximum quality*/
extern float g_mav_veh_flow_rate_x;     /*< [rad/s] Flow rate about X axis*/
extern float g_mav_veh_flow_rate_y;     /*< [rad/s] Flow rate about Y axis*/

extern float g_mav_veh_local_ned_x;  /*< [m] X Position*/
extern float g_mav_veh_local_ned_y;  /*< [m] Y Position*/
extern float g_mav_veh_local_ned_z;  /*< [m] Z Position*/
extern float g_mav_veh_local_ned_vx; /*< [m/s] X Speed*/
extern float g_mav_veh_local_ned_vy; /*< [m/s] Y Speed*/
extern float g_mav_veh_local_ned_vz; /*< [m/s] Z Speed*/

/********************************************************************************
 * Exported objects
 ********************************************************************************/


/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/

class SimDrone
{
public:
    SimDrone(void);
    ~SimDrone(void);

    static bool init(void);
    static void loop(void);
    static void shutdown(void);

private:
};

#endif // SIM_DRONE_H
