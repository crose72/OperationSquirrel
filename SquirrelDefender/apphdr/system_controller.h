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

/********************************************************************************
 * Exported objects
 ********************************************************************************/
extern SystemState g_system_state;
extern bool g_manual_override_land;

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/
class SystemController
{
public:
    SystemController();
    ~SystemController();

    static bool init(void);
    static void loop(void);
    static void shutdown(void);

private:
};

#endif // SYSTEM_CONTROLLER_H
