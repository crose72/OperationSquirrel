#pragma once

/********************************************************************************
 * @file    system_controller.h
 * @author  Cameron Rose
 * @date    1/22/2025
 ********************************************************************************/
#ifndef SYSTEM_CONTROLLER_H
#define SYSTEM_CONTROLLER_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "mav_data_hub.h"
#include "mav_utils.h"
#include "datalog.h"
#include "video_io.h"
#include "status_io.h"
#include "vehicle_controller.h"
#include "detect_target.h"
#include "track_target.h"
#include "localize_target.h"
#include "follow_target.h"
#include "time_calc.h"

/********************************************************************************
 * Imported objects
 ********************************************************************************/
extern bool g_valid_image_rcvd;
extern uint16_t g_mav_veh_rngfdr_current_distance;
extern int32_t g_mav_veh_rel_alt;
extern bool g_save_button_press;

/********************************************************************************
 * Exported objects
 ********************************************************************************/
extern SystemState g_system_state;

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/
class SystemController
{
public:
    SystemController();
    ~SystemController();

    static int init(void);
    static void loop(void);
    static void shutdown(void);

private:
};

#endif // SYSTEM_CONTROLLER_H
