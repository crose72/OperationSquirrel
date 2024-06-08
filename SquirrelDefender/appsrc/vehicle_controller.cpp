/********************************************************************************
 * @file    vehicle_controller.cpp
 * @author  Cameron Rose
 * @date    6/7/2023
 * @brief   Control the position, velocity, and acceleration of the drone by 
 *          sending the following MAVLINK message to the drone.  Control the
 *          vector position, velocity, acceleration, and yaw/yaw rate.
 * 
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "vehicle_controller.h"

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
 * Function: VehicleController
 * Description: Class constructor
 ********************************************************************************/
VehicleController::VehicleController() 
{
    for (int i = 0; i < control_dimension::num_dims; ++i) 
	{
        err_sum[i] = 0.0;
        err_prv[i] = 0.0;
    }
}

/********************************************************************************
 * Function: ~VehicleController
 * Description: Class destructor
 ********************************************************************************/
VehicleController::~VehicleController(void){}

/********************************************************************************
 * Function: pid_controller_2d
 * Description: PID controller with up to 2 parameters to control.
 ********************************************************************************/
float VehicleController::pid_controller_3d(float Kp, float Ki, float Kd, 
                                        float err1, float err2, float err3, 
                                        float w1, float w2, float w3, control_dimension dim)
{
	float err = (err1 * w1) + (err2 * w2) + (err3 * w3);
	float err_sum_local = 0.0;
	float err_prv_local = 0.0;
	
	float proportional_term = Kp * err;
	float integral_term =  Ki * (err_sum[dim] + err * dt_25ms);
	float derivative_term = Kd * (err - err_prv[dim]) / dt_25ms;
	float control = proportional_term + integral_term + derivative_term;
	
	err_sum[dim] = err_sum[dim] + err * dt_25ms;
	err_prv[dim] = err;
	
	return control;
}
