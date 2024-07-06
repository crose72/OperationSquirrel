#pragma once

#ifdef USE_JETSON

/********************************************************************************
 * @file    jetson_IO.h
 * @author  Cameron Rose
 * @date    6/7/2023
 ********************************************************************************/
#ifndef JETSON_IO_H
#define JETSON_IO_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include <JetsonGPIO.h>
#include <thread>
#include <chrono>
#include <unistd.h>

/********************************************************************************
 * Imported objects
 ********************************************************************************/

/********************************************************************************
 * Exported objects
 ********************************************************************************/
extern bool save_button_press;

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/
class StatusIndicators
{
public:
    StatusIndicators();
    ~StatusIndicators();

    static bool gpio_init(void);
    static void io_loop(void);
    static void gpio_shutdown(void);
    static void status_initializing(void);
    static void status_good(void);
    static void status_bad(void);
    static void status_bad_blink(void);
    static void clear_all_leds(void);
    static void save_video_button_state(void);

private:
};

#endif // JETSON_IO_H

#endif // USE_JETSON