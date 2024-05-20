/********************************************************************************
 * @file    sim_flight_test_4_VelocityControl.h
 * @author  Cameron Rose
 * @date    12/27/2023
 * @brief   Command the drone to follow a NED velocity vector where +x is 
 *          forward, +y is right, and +z is down.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "mavlink_msg_handler.h"
#include "mavlink_cmd_handler.h"
#include "velocity_controller.h"

/********************************************************************************
 * Imported objects
 ********************************************************************************/
extern int32_t mav_veh_alt;

/********************************************************************************
 * Test flight object definitions
 ********************************************************************************/
float target_velocity[3] = {0.0,0.0,0.0};
float timerVal = 0;
int stage = 0;

const float dt = 0.025;
const float error_cal = 0.1;

/********************************************************************************
 * Additional functions
 ********************************************************************************/
void countupTimer(void)
{
    timerVal = timerVal + dt; // printf("Timer: %.3f\n", timerVal);
}

void resetTimer(void)
{
    timerVal = 0;
}

/********************************************************************************
 * Test flight definition
 ********************************************************************************/
void test_flight(void)
{

    countupTimer();

    if (timerVal > 6.0 && stage == 0)
    {
        target_velocity[0] = 87.0;
        target_velocity[1] = 0;
        target_velocity[2] = 0;
        cmd_position(target_velocity);
        stage = 1;
    }

    if (timerVal > 12 && stage == 1)
    {
        target_velocity[0] = 0.0;
        target_velocity[1] = -87.0;
        target_velocity[2] = 0.0;
        cmd_position(target_velocity);
        stage = 2;
    }

    if (timerVal > 16.0 && stage == 2)
    {
        target_velocity[0] = -6.0;
        target_velocity[1] = 4.0;
        target_velocity[2] = -11.0;
        cmd_position(target_velocity);
        stage = 3;
    }

    if (timerVal > 23.0 && stage == 3)
    {
        target_velocity[0] = -6.0;
        target_velocity[1] = 4.0;
        target_velocity[2] = -11.0;
        cmd_position(target_velocity);
        stage = 4;
    }

    if (timerVal > 29 && stage == 4)
    {
        MavCmd::set_mode_RTL();
    }
}
