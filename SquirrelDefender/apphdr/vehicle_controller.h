#pragma once

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

/********************************************************************************
 * Function prototypes
 ********************************************************************************/
class VehicleController 
{
    public:
        VehicleController();
        ~VehicleController();

        enum control_dimension
        {
            x = 0,
            y = 1,
            z = 2,
            num_dims = 3
        };

        float pid_controller_3d(float Kp, float Ki, float Kd, 
                                float err1, float err2, float err3,  
                                float w1, float w2, float w3, control_dimension dim);

    private:
        float err_sum[3]; // Array to hold integral sums for x, y, z
        float err_prv[3]; // Array to hold previous errors for x, y, z
};


#endif // VEHICLE_CONTROLLER_H
