/********************************************************************************
 * @file    pid.cpp
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   PID controller implementation and variations.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "pid.h"

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
 * Function: PID
 * Description: Class constructor.
 ********************************************************************************/
PID::PID()
{
    max_integral = 1.0f;
    integral_decay = 1.0f;
    deriv_tau = 0.05f; // 50 ms default filter
    reset();           // handles clearing arrays
}

/********************************************************************************
 * Function: ~PID
 * Description: Class destructor.
 ********************************************************************************/
PID::~PID() {}

/********************************************************************************
 * Function: pid
 * Description: PID controller to control a specified dimension.
 ********************************************************************************/
float PID::pid(float Kp, float Ki, float Kd,
               float err, int dim, float dt)
{
    // --- Integrator update with decay ---
    err_sum[dim] = integral_decay * err_sum[dim] + err * dt;

    // Clamp integral (anti-windup)
    if (err_sum[dim] > max_integral)
    {
        err_sum[dim] = max_integral;
    }
    else if (err_sum[dim] < -max_integral)
    {
        err_sum[dim] = -max_integral;
    }

    float derr = (float)0.0;

    // --- Derivative with low-pass filter ---
    if (dt > (float)0.0)
    {
        derr = (err - err_prv[dim]) / dt;
    }

    const float alpha = dt / (deriv_tau + dt);
    d_filt[dim] = d_filt[dim] + alpha * (derr - d_filt[dim]);

    // --- PID output ---
    const float control = Kp * err + Ki * err_sum[dim] + Kd * d_filt[dim];

    // Store previous error
    err_prv[dim] = err;

    return control;
}

/********************************************************************************
 * Function: reset
 * Description: Reset the PID internal state.
 ********************************************************************************/
void PID::reset(void)
{
    for (int i = 0; i < MAX_CONTROL_AXES; i++)
    {
        err_sum[i] = 0.0f;
        err_prv[i] = 0.0f;
        d_filt[i] = 0.0f;
    }
}

/********************************************************************************
 * Function: PID3
 * Description: Class constructor
 ********************************************************************************/
PID3::PID3()
{
    max_integral = 1.0f;
    integral_decay = 1.0f;
    deriv_tau = 0.05f; // 50 ms default filter
    reset();           // handles clearing arrays
}

/********************************************************************************
 * Function: ~PID3
 * Description: Class destructor
 ********************************************************************************/
PID3::~PID3(void) {}

/********************************************************************************
 * Function: pid3
 * Description: PID3 controller with up to 3 errors to combine into a single
 *              control signal for a specified dimension.
 ********************************************************************************/
float PID3::pid3(float Kp, float Ki, float Kd,
                 float err1, float err2, float err3,
                 float w1, float w2, float w3, int dim, float dt)
{
    // Weighted error
    const float err = err1 * w1 + err2 * w2 + err3 * w3;

    // --- Update integral (apply decay first, then add) ---
    err_sum[dim] = integral_decay * err_sum[dim] + err * dt;

    // Clamp integral (anti-windup)
    if (err_sum[dim] > max_integral)
    {
        err_sum[dim] = max_integral;
    }
    else if (err_sum[dim] < -max_integral)
    {
        err_sum[dim] = -max_integral;
    }

    // --- Derivative with 1st-order low-pass (to tame noise) ---
    float derr = (float)0.0;

    // --- Derivative with low-pass filter ---
    if (dt > (float)0.0)
    {
        derr = (err - err_prv[dim]) / dt;
    }
    const float alpha = dt / (deriv_tau + dt); // e.g., deriv_tau = 0.05..0.15 s
    d_filt[dim] = d_filt[dim] + alpha * (derr - d_filt[dim]);

    // --- PID3 output ---
    const float control = Kp * err + Ki * err_sum[dim] + Kd * d_filt[dim];

    // Store previous error
    err_prv[dim] = err;

    return control; // Clamp the final actuator elsewhere and optionally back-calc AW if needed
}

/********************************************************************************
 * Function: reset
 * Description: Reset the PID internal state.
 ********************************************************************************/
void PID3::reset(void)
{
    for (int i = 0; i < MAX_CONTROL_AXES; i++)
    {
        err_sum[i] = 0.0f;
        err_prv[i] = 0.0f;
        d_filt[i] = 0.0f;
    }
}