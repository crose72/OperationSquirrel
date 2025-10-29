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

/********************************************************************************
 * Typedefs / Enums / Structs
 ********************************************************************************/

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/
class PID
{
public:
    PID();
    ~PID();

    // --- Core PID function ---
    float pid(float Kp, float Ki, float Kd,
              float err, int dim, float dt);

    // --- Publicly tunable parameters (modifiable at runtime) ---
    float max_integral;   // anti-windup clamp
    float integral_decay; // integral decay factor
    float deriv_tau;      // derivative low-pass time constant (s)

    // --- Reset integrator, derivative, and previous error ---
    void reset(void);

private:
    float err_sum[6]; // integral sum for x, y, z, roll, pitch, yaw
    float err_prv[6]; // previous errors
    float d_filt[6];  // filtered derivative term
};

class PID3
{
public:
    PID3();
    ~PID3();

    float pid3(float Kp, float Ki, float Kd,
               float err1, float err2, float err3,
               float w1, float w2, float w3, int dim,
               float dt);

private:
    float err_sum[6];                  // integral sum for x, y, z, roll, pitch, yaw
    float err_prv[6];                  // previous error
    float d_filt[6];                   // filtered derivative term
    const float max_integral = 10.0;   // clamp to prevent windup
    const float integral_decay = 0.95; // decay factor
    const float deriv_tau = 0.10;      // derivative low-pass time constant (s)
};

#endif // PID_CONTROLLER_H