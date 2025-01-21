/********************************************************************************
 * @file    pid.cpp
 * @author  Cameron Rose
 * @date    6/7/2023
 * @brief   Provide methods for implementing PID controllers from basic to
 *          complex.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "pid.h"

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
 * Function: PID
 * Description: Class constructor
 ********************************************************************************/
PID::PID()
{
    for (int i = 0; i < 3; ++i)
    {
        err_sum[i] = 0.0;
        err_prv[i] = 0.0;
    }
}

/********************************************************************************
 * Function: ~PID
 * Description: Class destructor
 ********************************************************************************/
PID::~PID(void) {}

/********************************************************************************
 * Function: pid3
 * Description: PID controller with up to 3 parameters to control.
 ********************************************************************************/
float PID::pid3(float Kp, float Ki, float Kd,
                             float err1, float err2, float err3,
                             float w1, float w2, float w3, int dim, float dt)
{
    float err = (err1 * w1) + (err2 * w2) + (err3 * w3);
    float err_sum_local = 0.0;
    float err_prv_local = 0.0;

    float proportional_term = Kp * err;
    float integral_term = Ki * (err_sum[dim] + err * dt);
    float derivative_term = Kd * (err - err_prv[dim]) / dt;
    float control = proportional_term + integral_term + derivative_term;

    // Apply clamping and decay on integrall sum
    err_sum[dim] = integral_decay * (err_sum[dim] + err * dt);

    if (err_sum[dim] > max_integral)
    {
        err_sum[dim] = max_integral;
    }
    else if (err_sum[dim] < -max_integral)
    {
        err_sum[dim] = -max_integral;
    }

    err_prv[dim] = err;

    return control;
}
