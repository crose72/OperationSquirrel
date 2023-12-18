#include "attitude_controller.h"

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

mavlink_set_attitude_target_t desired_attitude_target;
bool attitude_target_error = false;

const float error_cal = 0.1;

// SET_ATTITUDE_TARGET ( #82 )
// [Message] Sets a desired vehicle attitude. Used by an external controller to command the vehicle (manual controller or other system).

// Field Name	    Type	    Units	Values	Description
// time_boot_ms	    uint32_t	ms		        Timestamp (time since system boot).
// target_system	uint8_t			            System ID
// target_component	uint8_t			            Component ID
// type_mask	    uint8_t		                ATTITUDE_TARGET_TYPEMASK	Bitmap to indicate which dimensions should be ignored by the vehicle.
// q	            float[4]			        Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0) from MAV_FRAME_LOCAL_NED to MAV_FRAME_BODY_FRD
// body_roll_rate	float	    rad/s		    Body roll rate
// body_pitch_rate	float	    rad/s		    Body pitch rate
// body_yaw_rate	float	    rad/s		    Body yaw rate
// thrust	float			                    Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
// thrust_body **	float[3]			        3D thrust setpoint in the body NED frame, normalized to -1 .. 1

// ATTITUDE_TARGET_TYPEMASK
// [Enum] Bitmap to indicate which dimensions should be ignored by the vehicle: a value of 0b00000000 indicates that none of the setpoint dimensions should be ignored.

// Value	Field Name	                                    Description
// 1	    ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE	Ignore body roll rate
// 2	    ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE	Ignore body pitch rate
// 4	    ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE	Ignore body yaw rate
// 32	    ATTITUDE_TARGET_TYPEMASK_THRUST_BODY_SET	    Use 3D body thrust setpoint instead of throttle
// 64	    ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE	    Ignore throttle
// 128	    ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE	    Ignore attitude

// True if the difference between the desired attitude and the actual attitude is large
bool dtrmn_attitude_target_error(void)
{
    if (std::fabs(q1_actual - q1_target) > error_cal 
    || std::fabs(q2_actual - q2_target) > error_cal 
    || std::fabs(q3_actual - q3_target) > error_cal 
    || std::fabs(q4_actual - q4_target) > error_cal 
    || std::fabs(roll_rate_actual - roll_rate_target) > error_cal 
    || std::fabs(pitch_rate_actual - pitch_rate_target) > error_cal 
    || std::fabs(yaw_rate_actual - yaw_rate_target) > error_cal)
    {
        attitude_target_error = true;
    }
    else
    {
        attitude_target_error = false;
    }
}

void move_forward (void)
{
    // parameters for set_attitude_target command
    desired_attitude_target.time_boot_ms = 0;
    desired_attitude_target.body_roll_rate = 0.0;
    desired_attitude_target.body_pitch_rate = -2.0;
    desired_attitude_target.body_yaw_rate = 0.0; 
    desired_attitude_target.thrust = (float)0.5;  // Define the desired thrust magnitude (adjust as needed), Range: 0.0 (no thrust) to 1.0 (full thrust)
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

    for (int i = 0; i < 1; i++)
    {
        send_cmd_set_attitude_target(&desired_attitude_target);
    }
}

void brake (void)
{
    // parameters for set_attitude_target command
    desired_attitude_target.time_boot_ms = 0;
    desired_attitude_target.body_roll_rate = 0.0;
    desired_attitude_target.body_pitch_rate = 2.0;
    desired_attitude_target.body_yaw_rate = 0.0; 
    desired_attitude_target.thrust = (float)0.5;  // Define the desired thrust magnitude (adjust as needed), Range: 0.0 (no thrust) to 1.0 (full thrust)
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

    for (int i = 0; i < 2; i++)
    {
        send_cmd_set_attitude_target(&desired_attitude_target);
    }
}

void turn_left_90 (void)
{
    // parameters for set_attitude_target command
    desired_attitude_target.time_boot_ms = 0;
    desired_attitude_target.body_roll_rate = 0.0;
    desired_attitude_target.body_pitch_rate = 0.0;
    desired_attitude_target.body_yaw_rate = -1.57; 
    desired_attitude_target.thrust = (float)0.0;  // Define the desired thrust magnitude (adjust as needed), Range: 0.0 (no thrust) to 1.0 (full thrust)
    desired_attitude_target.q[0] = (float)1.0; // real part, i, j, k
    desired_attitude_target.q[1] = (float)0.0;
    desired_attitude_target.q[2] = (float)0.0;
    desired_attitude_target.q[3] = (float)0.0;
    desired_attitude_target.target_system = TARGET_SYS_ID;
    desired_attitude_target.target_component = TARGET_COMP_ID;
    desired_attitude_target.type_mask = 0b00000000;
    desired_attitude_target.thrust_body[0] = (float)0.0; // Define the desired thrust direction, Positive X-axis (forward)
    desired_attitude_target.thrust_body[1] = (float)0.0;
    desired_attitude_target.thrust_body[2] = (float)0.0;

    if (true)
    {
        send_cmd_set_attitude_target(&desired_attitude_target);
    }
}
