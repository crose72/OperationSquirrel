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
#define MAX_CONTROL_AXES 7

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/
class PID
{
public:
    PID();
    ~PID();

    float max_integral;   // anti-windup clamp
    float integral_decay; // integral decay factor
    float deriv_tau;      // derivative low-pass time constant (s)

    float pid(float Kp, float Ki, float Kd,
              float err, int dim, float dt);
    void reset(void);

private:
    float err_sum[MAX_CONTROL_AXES]; // integral sum for x, y, z, roll, pitch, yaw
    float err_prv[MAX_CONTROL_AXES]; // previous errors
    float d_filt[MAX_CONTROL_AXES];  // filtered derivative term
};

class PID3
{
public:
    PID3();
    ~PID3();

    float max_integral = 10.0;   // clamp to prevent windup
    float integral_decay = 0.95; // decay factor
    float deriv_tau = 0.10;      // derivative low-pass time constant (s)

    float pid3(float Kp, float Ki, float Kd,
               float err1, float err2, float err3,
               float w1, float w2, float w3, int dim,
               float dt);
    void reset(void);

private:
    float err_sum[MAX_CONTROL_AXES]; // integral sum for x, y, z, roll, pitch, yaw
    float err_prv[MAX_CONTROL_AXES]; // previous error
    float d_filt[MAX_CONTROL_AXES];  // filtered derivative term
};

#endif // PID_CONTROLLER_H