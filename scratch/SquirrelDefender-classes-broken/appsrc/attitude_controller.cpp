/********************************************************************************
 * @file    attitude_controller.cpp
 * @author  Cameron Rose
 * @date    12/27/2023
 * @brief   Control the yaw, pitch, roll, thrust of the drone by sending the 
 *          following MAVLINK message to the drone.  Useful for trajectory
 *          planning.
 * 
            SET_ATTITUDE_TARGET ( #82 )
            [Message] Sets a desired vehicle attitude. Used by an external controller to command the vehicle (manual controller or other system).

            Field Name	        Type	    Units	Values	Description
            time_boot_ms	    uint32_t	ms		        Timestamp (time since system boot).
            target_system	    uint8_t			            System ID
            target_component    uint8_t			            Component ID
            type_mask	        uint8_t		                ATTITUDE_TARGET_TYPEMASK	Bitmap to indicate which dimensions should be ignored by the vehicle.
            q	                float[4]			        Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0) from MAV_FRAME_LOCAL_NED to MAV_FRAME_BODY_FRD
            body_roll_rate	    float	    rad/s		    Body roll rate
            body_pitch_rate	    float	    rad/s		    Body pitch rate
            body_yaw_rate	    float	    rad/s		    Body yaw rate
            thrust	float	    		                    Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
            thrust_body **	    float[3]			        3D thrust setpoint in the body NED frame, normalized to -1 .. 1

            ATTITUDE_TARGET_TYPEMASK
            [Enum] Bitmap to indicate which dimensions should be ignored by the vehicle: a value of 0b00000000 indicates that none of the setpoint dimensions should be ignored.

            Value	Field Name	                                    Description
            1	    ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE	Ignore body roll rate
            2	    ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE	Ignore body pitch rate
            4	    ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE	Ignore body yaw rate
            32	    ATTITUDE_TARGET_TYPEMASK_THRUST_BODY_SET	    Use 3D body thrust setpoint instead of throttle
            64	    ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE	    Ignore throttle
            128	    ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE	    Ignore attitude
 ********************************************************************************/


/********************************************************************************
 * Includes
 ********************************************************************************/
#include "attitude_controller.h"

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/
mavlink_set_attitude_target_t AttitudeController::desired_attitude_target;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/
const float AttitudeController::att_error_target = 0.1;

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
 * Function: dtrmn_att_target_achvd
 * Description: Determine if error between desired and actual attitude is
 *              sufficiently small.
 ********************************************************************************/
bool AttitudeController::dtrmn_att_target_achvd(void)
{
    bool att_target_achvd = false;
    
    if (std::fabs(q1_actual - q1_target) < att_error_target 
    || std::fabs(q2_actual - q2_target) < att_error_target 
    || std::fabs(q3_actual - q3_target) < att_error_target 
    || std::fabs(q4_actual - q4_target) < att_error_target 
    || std::fabs(roll_rate_actual - roll_rate_target) < att_error_target 
    || std::fabs(pitch_rate_actual - pitch_rate_target) < att_error_target 
    || std::fabs(yaw_rate_actual - yaw_rate_target) < att_error_target
    || std::fabs(thrust_actual - thrust_target) < att_error_target)
    {
        att_target_achvd = true;
    }
    else
    {
        att_target_achvd = false;
    }

    return att_target_achvd;
}

/********************************************************************************
 * Function: move_forward
 * Description: Command drone to move forward until otherwise directed
 ********************************************************************************/
void AttitudeController::move_forward (void)
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

    mav.send_cmd_set_attitude_target(&desired_attitude_target);
}

/********************************************************************************
 * Function: brake
 * Description: Command drone to air brake
 ********************************************************************************/
void AttitudeController::brake (void)
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

    mav.send_cmd_set_attitude_target(&desired_attitude_target);
}
