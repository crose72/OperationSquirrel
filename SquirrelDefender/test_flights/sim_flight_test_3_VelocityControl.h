/********************************************************************************
 * @file    sim_flight_test_3_VelocityControl.h
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   Command the drone to follow a NED velocity vector where +x is 
 *          forward, +y is right, and +z is down.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "mav_data_hub.h"
#include "mav_utils.h"

/********************************************************************************
 * Imported objects
 ********************************************************************************/
extern int32_t alt;

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
    VelocityController VelController;
    countupTimer();

    if (timerVal > 6.0 && stage == 0)
    {
        target_velocity[0] = 10.0;
        target_velocity[1] = -4.0;
        target_velocity[2] = -4.0;
        VelController.cmd_position_NED(SENDER_SYS_ID, SENDER_COMP_ID, TARGET_SYS_ID, TARGET_COMP_ID, target_velocity);
        stage = 1;
    }

    if (timerVal > 12 && stage == 1)
    {
        target_velocity[0] = 5.0;
        target_velocity[1] = 7.0;
        target_velocity[2] = 1.0;
        VelController.cmd_position_NED(SENDER_SYS_ID, SENDER_COMP_ID, TARGET_SYS_ID, TARGET_COMP_ID, target_velocity);
        stage = 2;
    }

    if (timerVal > 16.0 && stage == 2)
    {
        target_velocity[0] = -6.0;
        target_velocity[1] = 4.0;
        target_velocity[2] = -2.0;
        VelController.cmd_position_NED(SENDER_SYS_ID, SENDER_COMP_ID, TARGET_SYS_ID, TARGET_COMP_ID, target_velocity);
        stage = 3;
    }

    if (timerVal > 23 && stage == 3)
    {
        return_to_launch();
    }
}
