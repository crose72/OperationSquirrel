/********************************************************************************
 * @file    attitude_controller.h
 * @author  Cameron Rose
 * @date    12/27/2023
 ********************************************************************************/
#ifndef ATTITUDE_CONTROLLER_H
#define ATTITUDE_CONTROLLER_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "mavlink_cmd_handler.h"
#include "mavlink_msg_handler.h"

/********************************************************************************
 * Imported objects
 ********************************************************************************/
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

/********************************************************************************
 * Exported objects
 ********************************************************************************/

/********************************************************************************
 * Function prototypes
 ********************************************************************************/

class AttitudeController 
{
public:
    AttitudeController();
    ~AttitudeController();

    // Public member variables
    static const float att_error_target;
    
    // Public member functions
    void brake (void);
    void move_forward (void);

private:
    // Private member variables
    static mavlink_set_attitude_target_t desired_attitude_target;

    // Create instance of external classes whose functionality is needed
    MavCmd mav;

    // Private member functions
    bool dtrmn_att_target_achvd(void);
};


#endif // ATTITUDE_CONTROLLER_H