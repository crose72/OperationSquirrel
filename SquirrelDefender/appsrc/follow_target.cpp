#ifdef USE_JETSON

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
float w1_x = 0.0;
float w2_x = 0.0;
float w3_x = 0.0;
float w1_y = 0.0;
float w2_y = 0.0;
float w3_y = 0.0;
float w1_z = 0.0;
float w2_z = 0.0;
float w3_z = 0.0;
float x_desired = 0;
float x_actual = 0.0;
float height_desired = 0.0;
float height_actual = 0.0;
float y_desired = 0;
float y_actual = 0.0;
float width_desired = 0.0;
float width_actual = 0.0;
float err_x_1 = 0.0;
float err_x_2 = 0.0;
float err_x_3 = 0.0;
float err_y_1 = 0.0;
float err_y_2 = 0.0;
float err_y_3 = 0.0;
float err_z_1 = 0.0;
float err_z_2 = 0.0;
float err_z_3 = 0.0;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
 * Function: Follow
 * Description: Follow class constructor.
 ********************************************************************************/
Follow::Follow(void){};

/********************************************************************************
 * Function: ~Follow
 * Description: Follow class destructor.
 ********************************************************************************/
Follow::~Follow(void){};

/********************************************************************************
 * Function: get_control_params
 * Description: Read follow control parameters from a json or other file type.
 ********************************************************************************/
void Follow::get_control_params(void)
{
    Parameters veh_params("../params.json");
    // Accessing Vel_PID_x parameters
    Kp_x = veh_params.get_float_param("Vel_PID_x", "Kp");
    Ki_x = veh_params.get_float_param("Vel_PID_x", "Ki");
    Kd_x = veh_params.get_float_param("Vel_PID_x", "Kd");
    w1_x = veh_params.get_float_param("Vel_PID_x", "w1");
    w2_x = veh_params.get_float_param("Vel_PID_x", "w2");
    w3_x = veh_params.get_float_param("Vel_PID_x", "w3");

    // Accessing Vel_PID_y parameters
    Kp_y = veh_params.get_float_param("Vel_PID_y", "Kp");
    Ki_y = veh_params.get_float_param("Vel_PID_y", "Ki");
    Kd_y = veh_params.get_float_param("Vel_PID_y", "Kd");
    w1_y = veh_params.get_float_param("Vel_PID_y", "w1");
    w2_y = veh_params.get_float_param("Vel_PID_y", "w2");
    w3_y = veh_params.get_float_param("Vel_PID_y", "w3");
}

/********************************************************************************
 * Function: get_target_desired_params
 * Description: Read follow target parameters from a json or other file type.
 ********************************************************************************/
void Follow::get_target_desired_params(void)
{
    Parameters target_params("../params.json");

    x_desired = static_cast<float>(input_video_height)/2.0;
    y_desired = static_cast<float>(input_video_width)/2.0;
    height_desired = target_params.get_float_param("Target", "Desired_Height");
    width_desired = target_params.get_float_param("Target", "Desired_Width");
}

/********************************************************************************
 * Function: calc_target_actual_params
 * Description: Calculate the parameters of a target.
 ********************************************************************************/
void Follow::calc_target_actual_params(int n)
{
    x_actual = detections[n].Height()/2.0 + detections[n].Top;
    height_actual = detections[n].Height();
    y_actual = detections[n].Width()/2.0 + detections[n].Left;
    width_actual = detections[n].Width();
}

/********************************************************************************
 * Function: calc_target_error
 * Description: Calculate the error of a target's position.
 ********************************************************************************/
void Follow::calc_target_error(void)
{
    err_x_1 = x_desired - x_actual;
    err_x_2 = height_desired - height_actual;
    err_x_3 = 0.0;
    err_y_1 = y_actual - y_desired;
    err_y_2 = 0.0;
    err_y_3 = 0.0;
    err_z_1 = 0.0;
    err_z_2 = 0.0;
    err_z_3 = 0.0;
}

/********************************************************************************
 * Function: follow_target
 * Description: Control vehicle to follow a designated target at a specific
 *               distance.
 ********************************************************************************/
void Follow::follow_target_loop(void)
{
    VehicleController vc;
    float target_velocity[3] = {0.0,0.0,0.0};

    get_control_params();
    get_target_desired_params();

    if( numDetections > 0 )
    {
        for( int n=0; n < numDetections; n++ )
        {
            if( detections[n].TrackID >= 0 ) // is this a tracked object?
            {			
                if (detections[n].ClassID == 1 && detections[n].Confidence > 0.5)
                {
                    calc_target_actual_params(n);
                    calc_target_error();
                    
                    float vx_adjust = vc.pid_controller_3d(Kp_x, Ki_x, Kd_x, 
                                                           err_x_1, err_x_2, err_x_3, 
                                                           w1_x, w2_x, w3_x, VehicleController::control_dimension::x);
                    float vy_adjust = -vc.pid_controller_3d(Kp_y, Ki_y, Kd_y, 
                                                           err_y_1, err_y_2, err_y_3, 
                                                           w1_y, w2_y, w3_y, VehicleController::control_dimension::y);
                    
                    if (vx_adjust < 0)
                    {
                        vx_adjust = -vx_adjust;
                    }
                    if (height_actual >= height_desired)
                    {
                        vx_adjust = 0.0;
                    }

                    std::cout << "Control signal x: " << vx_adjust << std::endl;
                    std::cout << "Control signal y: " << vy_adjust << std::endl;

                    target_velocity[0] = vx_adjust;
                    target_velocity[1] = vy_adjust;

                    cmd_velocity(target_velocity);	
                }
            }
            
        }
    }
}


#endif // USE_JETSON