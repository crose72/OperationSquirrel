#include "standard_libs.h"
#include "mavlink_msg_handler.h"
#include "mavlink_command_handler.h"

extern float q1_target;
extern float q2_target;
extern float q3_target;
extern float q4_target;
extern float roll_rate_target;
extern float pitch_rate_target;
extern float yaw_rate_target;
extern float thrust_target;
extern float q1_actual;
extern float q2_actual;
extern float q3_actual;
extern float q4_actual;
extern float roll_rate_actual;
extern float pitch_rate_actual;
extern float yaw_rate_actual;
extern float thrust_actual;
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

    mavlink_set_attitude_target_t desired_attitude_target;
    // parameters for set_attitude_target command
    desired_attitude_target.time_boot_ms = 0;
    desired_attitude_target.body_roll_rate = 0.1;
    desired_attitude_target.body_pitch_rate = 0.1;
    desired_attitude_target.body_yaw_rate = 0.1; 
    desired_attitude_target.thrust = (float)1.0;  // Define the desired thrust magnitude (adjust as needed), Range: 0.0 (no thrust) to 1.0 (full thrust)
    desired_attitude_target.q[0] = (float)1.0; // real part, i, j, k
    desired_attitude_target.q[1] = (float)0.0;
    desired_attitude_target.q[2] = (float)0.0;
    desired_attitude_target.q[3] = (float)0.0;
    desired_attitude_target.target_system = TARGET_SYS_ID;
    desired_attitude_target.target_component = TARGET_COMP_ID;
    desired_attitude_target.type_mask = 0b00000000;
    desired_attitude_target.thrust_body[0] = (float)1.0; // Define the desired thrust direction, Positive X-axis (forward)
    desired_attitude_target.thrust_body[1] = (float)0.0;
    desired_attitude_target.thrust_body[2] = (float)0.0;



    if (alt > 12.0)
    {
        if (std::fabs(q1_actual - q1_target) > error_cal || std::fabs(q2_actual - q2_target) > error_cal || std::fabs(q3_actual - q3_target) > error_cal || std::fabs(q4_actual - q4_target) > error_cal 
        || std::fabs(roll_rate_actual - roll_rate_target) > error_cal || std::fabs(pitch_rate_actual - pitch_rate_target) > error_cal || std::fabs(yaw_rate_actual - yaw_rate_target) > error_cal)
        {
            send_cmd_set_attitude_target(&desired_attitude_target);
        }
        else
        {
            // Do nothing
        }
    }

    /*if (timerVal > 14.0)
    {
        landing_sequence();
    }*/
}
