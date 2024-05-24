/********************************************************************************
 * @file    follow_target.cpp
 * @author  Cameron Rose
 * @date    12/27/2023
 * @brief   Follow the target and maintain a specified x, y, z offset.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "follow_target.h"

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
 * Function: 
 * Description: 
 ********************************************************************************/
void follow_target(void)
{
    #ifdef USE_JETSON
        VehicleController::readPIDParametersFromJSON("../params.json", Kp_x, Ki_x, Kd_x, Kp_y, Ki_y, Kd_y);
        if (!Video::capture_image())
        {
            //break;
        }

        detect_objects();
        get_object_info();
        print_object_info();

        if (!Video::render_output())
        {
            //break;
        }
        //std::cout << "Vx: " << mav_veh_gps_vx << std::endl;
        
        float target_velocity[3] = {0.0,0.0,0.0};
        if( numDetections > 0 )
        {
            for( int n=0; n < numDetections; n++ )
            {
                if( detections[n].TrackID >= 0 ) // is this a tracked object?
                {			
                    if (detections[n].ClassID == 1 && detections[n].Confidence > 0.5)
                    {
                        float x_desired = 360.0;
                        float x_actual = detections[n].Height()/2.0 + detections[n].Top;
                        float height_desired = 650;
                        float height_actual = detections[n].Height();
                        float w1 = 0.5;
                        float w2 = 0.5;
                        float vx_adjust = VehicleController::pid_controller_2d(Kp_x, Ki_x, Kd_x, x_desired, x_actual, 											height_desired, height_actual, w1, w2, 0);
                        //std::cout << "Control signal x: " << vx_adjust << std::endl;
                        if (vx_adjust < 0)
                        {
                            vx_adjust = -vx_adjust;
                        }
                        if (height_actual >= height_desired)
                        {
                            vx_adjust = 0.0;
                        }
                        target_velocity[0] = vx_adjust;

                        float y_desired = 640.0;
                        float y_actual = detections[n].Width()/2.0 + detections[n].Left;
                        float vy_adjust = -VehicleController::pid_controller_2d(Kp_y, Ki_y, Kd_y, y_desired, y_actual, 										0.0, 0.0, 1.0, 0.0, 1);
                        //std::cout << "Control signal y: " << vy_adjust << std::endl;
                        target_velocity[1] = vy_adjust;

                        cmd_velocity(target_velocity);	
                    }
                }
                
            }
        }

        //print_performance_stats();
    #else
        //#error "Please define USE_JETSON to enable use of this code."
        //test_flight();
    #endif
}
