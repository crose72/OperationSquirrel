#pragma once

/********************************************************************************
 * @file    system_controller.h
 * @author  Cameron Rose
 * @date    6/7/2023
 ********************************************************************************/
#ifndef SYSTEM_CONTROLLER_H
#define SYSTEM_CONTROLLER_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "mavlink_msg_handler.h"
#include "video_IO.h"
#include "jetson_IO.h"

/********************************************************************************
 * Imported objects
 ********************************************************************************/
extern bool valid_image_rcvd;
extern uint16_t mav_veh_rngfdr_current_distance;
extern int32_t mav_veh_rel_alt;
extern float dt_25ms;

/********************************************************************************
 * Exported objects
 ********************************************************************************/
extern SYSTEM_STATE system_state;

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
    static int system_state_machine(void);
    static void led_system_indicators(void);

private:
};

#endif // SYSTEM_CONTROLLER_H
