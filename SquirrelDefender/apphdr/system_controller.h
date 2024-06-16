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

/********************************************************************************
 * Imported objects
 ********************************************************************************/
extern bool valid_image_rcvd;
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

    static int system_init(void);
    static int system_state_machine(void);
    static void system_shutdown(void);

private:
};

#endif // SYSTEM_CONTROLLER_H
