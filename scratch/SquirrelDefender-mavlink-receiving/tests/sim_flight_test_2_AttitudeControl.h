/********************************************************************************
 * @file    sim_flight_test_2_AttitudeControl.h
 * @author  Cameron Rose
 * @date    12/27/2023
 * @brief   A test to understand the set attitude target message for controlling
 *          the drone.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "mavlink_msg_handler.h"
#include "mavlink_cmd_handler.h"
#include "attitude_controller.h"

/********************************************************************************
 * Imported objects
 ********************************************************************************/
extern int32_t alt;

/********************************************************************************
 * Test flight object definitions
 ********************************************************************************/
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

    if (alt > 5.0 && stage == 0)
    {
        move_forward();
        stage = 1;
    }

    // if (timerVal > 8 && stage == 1)
    // {
    //     brake();
    //     stage = 2;
    //     //return_to_launch();
    // }

    /*if (timerVal > 14.0)
    {
        return_to_launch();
    }*/
}