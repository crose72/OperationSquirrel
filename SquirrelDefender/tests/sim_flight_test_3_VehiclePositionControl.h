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

    if (timerVal > 5.0 && stage == 0)
    {
        move_to_position((float)2, (float)-10, (float)-4);
        stage = 1;
    }

    if (timerVal > 10 && stage == 1)
    {
        move_to_position((float)20, (float)-4, (float)-11);
        stage = 2;
    }

    /*if (timerVal > 14.0)
    {
        landing_sequence();
    }*/
}