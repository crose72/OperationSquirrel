#pragma once

/********************************************************************************
 * @file    vehicle_controller.h
 * @author  Cameron Rose
 * @date    6/7/2023
 ********************************************************************************/
#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "mavlink_cmd_handler.h"
#include "mavlink_msg_handler.h"
#include "velocity_controller.h"
#include "attitude_controller.h"
#include "follow_target.h"
#include "system_controller.h"

/********************************************************************************
 * Imported objects
 ********************************************************************************/
extern bool target_too_close;
extern bool target_valid;
extern float vx_adjust;
extern float vy_adjust;
extern float vz_adjust;
extern float dt_25ms;
extern uint16_t mav_veh_rngfdr_current_distance;
extern int32_t mav_veh_rel_alt;

/********************************************************************************
 * Exported objects
 ********************************************************************************/

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/
class VehicleController
{
    VehicleController();
    ~VehicleController();

public:
    static bool init(void);
    static void loop(void);
    static void shutdown(void);

private:
};

#endif // VEHICLE_CONTROLLER_H
