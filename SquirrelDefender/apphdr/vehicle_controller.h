#pragma once

/********************************************************************************
 * @file    vehicle_controller.h
 * @author  Cameron Rose
 * @date    1/22/2025
 ********************************************************************************/
#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "mav_data_hub.h"
#include "mav_utils.h"
#include "follow_target.h"
#include "system_controller.h"

/********************************************************************************
 * Imported objects
 ********************************************************************************/
extern bool g_target_too_close;
extern bool g_target_valid;
extern float g_vx_adjust;
extern float g_vy_adjust;
extern float g_vz_adjust;
extern uint16_t g_mav_veh_rngfdr_current_distance;
extern int32_t g_mav_veh_rel_alt;

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
