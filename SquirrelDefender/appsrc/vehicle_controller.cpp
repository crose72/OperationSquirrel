/********************************************************************************
 * @file    vehicle_controller.cpp
 * @author  Cameron Rose
 * @date    12/27/2023
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
float err_x = 0.0;
float err_y = 0.0;
float err_x_prv = 0.0;
float err_y_prv = 0.0;
float err_x_sum = 0.0;
float err_y_sum = 0.0;
float Kp_x = 0.0;
float Ki_x = 0.0;
float Kd_x = 0.0;
float Kp_y = 0.0;
float Ki_y = 0.0;
float Kd_y = 0.0;

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
VehicleController::VehicleController(void){}

/********************************************************************************
 * Function: ~VehicleController
 * Description: Class destructor
 ********************************************************************************/
VehicleController::~VehicleController(void){}

/********************************************************************************
 * Function: pid_controller_2d
 * Description: PID controller with up to 2 parameters to control.
 ********************************************************************************/
float VehicleController::pid_controller_2d(float Kp, float Ki, float Kd, 
                                        float desired, float actual, float desired2, 
                                        float actual2, float w1, float w2, int dim)
{
	float dt = 0.025;
	float error = (desired - actual) * w1 + (desired2 - actual2) * w2;
	float err_sum = 0.0;
	float err_prv = 0.0;
	
	if (dim == 0)
	{
		err_sum = err_x_sum;
		err_prv = err_x_prv;
	}
	else if (dim == 1)
	{
		err_sum = err_y_sum;	
		err_prv = err_y_prv;
	}
	
	float integral = (err_sum+error*dt)*Ki;
	float derivative = (error-err_prv)/dt*Kd;
	float control = error*Kp+integral*Ki+derivative*Kd;
	
	if (dim == 0)
	{
		err_x_sum = err_x_sum + error;
		err_x_prv = error;
	}
	else if (dim == 1)
	{
		err_y_sum = err_y_sum + error;
		err_y_prv = error;	
	}
	
	return control;
}

/********************************************************************************
 * Function: pid_controller_2d
 * Description: PID controller with up to 2 parameters to control.
 ********************************************************************************/
// Function to read PID parameters from a JSON file
void VehicleController::readPIDParametersFromJSON(const std::string& filename, float& Kp_x, float& Ki_x, float& Kd_x, float& Kp_y, float& Ki_y, float& Kd_y) 
{
    #ifdef USE_JETSON
    std::ifstream configFile(filename);
    Json::Value root;
    configFile >> root;

    Kp_x = root["Kp_x"].asFloat();
    Ki_x = root["Ki_x"].asFloat();
    Kd_x = root["Kd_x"].asFloat();
    Kp_y = root["Kp_y"].asFloat();
    Ki_y = root["Ki_y"].asFloat();
    Kd_y = root["Kd_y"].asFloat();
    #else
	//#error "Please define USE_JETSON to enable use of this code."
    #endif
}
