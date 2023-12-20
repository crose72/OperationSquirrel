#include "standard_libs.h"
#include "mavlink_msg_handler.h"
#include "mavlink_command_handler.h"
#include "vehicle_controller.h"

extern int32_t alt;

const float dt = 0.025;
const float error_cal = 0.1;

float timerVal = 0;
int stage = 0;

void countupTimer(void)
{
    timerVal = timerVal + dt; // printf("Timer: %.3f\n", timerVal);
}

void resetTimer(void)
{
    timerVal = 0;
}

void test_flight(void)
{
    countupTimer();

    if (timerVal > 6.0 && stage == 0)
    {
        cmd_position((float)10, (float)-4, (float)-4);
        stage = 1;
    }

    if (timerVal > 12 && stage == 1)
    {
        cmd_position((float)5, (float)7, (float)1);
        stage = 2;
    }

    // if (timerVal > 16.0 && stage == 2)
    // {
    //     cmd_position((float)-6, (float)4, (float)-2);
    //     stage = 3;
    // }

    if (timerVal > 23 && stage == 3)
    {
        landing_sequence();
    }
}