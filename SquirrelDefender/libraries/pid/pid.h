#pragma once

/********************************************************************************
 * @file    pid.h
 * @author  Cameron Rose
 * @date    1/22/2025
 ********************************************************************************/
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"

/********************************************************************************
 * Imported objects
 ********************************************************************************/
extern float g_dt_25ms;

/********************************************************************************
 * Exported objects
 ********************************************************************************/

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/
class PID
{
public:
    PID();
    ~PID();

    float pid3(float Kp, float Ki, float Kd,
               float err1, float err2, float err3,
               float w1, float w2, float w3, int dim, 
               float dt);

private:
    float err_sum[6];                  // Array to hold integral sums for x, y, z, roll, pitch, yaw
    float err_prv[6];                  // Array to hold previous errors for x, y, z, roll, pitch, yaw
    const float max_integral = 10.0;   // Max value for integral term to prevent windup
    const float integral_decay = 0.95; // Decay factor for integral term
};

#endif // PID_CONTROLLER_H