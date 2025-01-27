/********************************************************************************
 * @file    sim_flight_test_1_GPSWaypoints.h
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   Command the drone to follow a move to specified GPS lat, lon, and 
 *          alt.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "mavlink_msg_handler.h"
#include "mav_utils.h"

/********************************************************************************
 * Imported objects
 ********************************************************************************/
extern int32_t alt;

/********************************************************************************
 * Test flight object definitions
 ********************************************************************************/
float timerVal = 0;
int stage = 0;
int32_t A_lat = -353632362; 
int32_t A_lon = 1491648312;
float A_alt = 5;
int32_t B_lat = -353637101;  
int32_t B_lon = 1491653676;
float B_alt = 6;
int32_t C_lat = -353637206;
int32_t C_lon = 1491660477;
float C_alt = 5;
int32_t D_lat = -353633059;
int32_t D_lon = 1491656980;
float D_alt = 12;

const float dt = 0.025;

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
    if (stage <= 4)
    {
        countupTimer();
    }

    if (timerVal > 7.0 && stage == 0)
    {
        printf("Moving to A\n");
        stage = stage + 1;
        resetTimer();
        go_to_waypoint(A_lat, A_lon, A_alt);

    }

    if (timerVal > 6.0 && stage == 1)
    {
        printf("Moving to B\n");
        stage = stage + 1;
        resetTimer();
        go_to_waypoint(B_lat, B_lon, B_alt);
    }

    if (timerVal > 6.0 && stage == 2)
    {
        printf("Moving to C\n");
        stage = stage + 1;
        resetTimer();
        go_to_waypoint(C_lat, C_lon, C_alt);
    }

    if (timerVal > 5.0 && stage == 3)
    {
        printf("Moving to D\n");
        stage = stage + 1;
        resetTimer();
        go_to_waypoint(D_lat, D_lon, D_alt);
    }

    if (timerVal > 7.0 && stage == 4)
    {
        printf("Returning to base\n");
        stage = stage + 1;
        resetTimer();
        return_to_launch();
    }
}
