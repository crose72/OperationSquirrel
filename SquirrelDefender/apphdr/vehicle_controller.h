/********************************************************************************
 * @file    vehicle_controller.h
 * @author  Cameron Rose
 * @date    12/27/2023
 ********************************************************************************/
#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "mavlink_cmd_handler.h"
#include "velocity_controller.h"
#include "attitude_controller.h"

#ifdef USE_JETSON
	#include <jsoncpp/json/json.h> //sudo apt-get install libjsoncpp-dev THEN target_link_libraries(your_executable_name jsoncpp)
#else
	//#error "Please define USE_JETSON to enable use of this code."
#endif

/********************************************************************************
 * Imported objects
 ********************************************************************************/

/********************************************************************************
 * Exported objects
 ********************************************************************************/
extern float err_x;
extern float err_y;
extern float err_x_prv;
extern float err_y_prv;
extern float err_x_sum;
extern float err_y_sum;
extern float Kp_x;
extern float Ki_x;
extern float Kd_x;
extern float Kp_y;
extern float Ki_y;
extern float Kd_y;

/********************************************************************************
 * Function prototypes
 ********************************************************************************/
class VehicleController 
{
public:
    VehicleController();
    ~VehicleController();

    static float pid_controller_2d(float Kp, float Ki, float Kd, 
                                            float desired, float actual, float desired2, 
                                            float actual2, float w1, float w2, int dim);
    static void readPIDParametersFromJSON(const std::string& filename, float& Kp_x, float& Ki_x, float& Kd_x, 
                                          float& Kp_y, float& Ki_y, float& Kd_y);

private:
    
};


#endif // VEHICLE_CONTROLLER_H
