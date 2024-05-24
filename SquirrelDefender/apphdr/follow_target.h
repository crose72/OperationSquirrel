/********************************************************************************
 * @file    follow_target.h
 * @author  Cameron Rose
 * @date    12/27/2023
 ********************************************************************************/
#ifndef FOLLOW_TARGET_H
#define FOLLOW_TARGET_H

/************************************
 * Includes
 ************************************/
#include "common_inc.h"
#include "videoIO.h"
#include "object_detection.h"
#include "vehicle_controller.h"
#include "velocity_controller.h"

/************************************
 * Imported objects
 ************************************/
extern detectNet* net;
extern detectNet::Detection* detections;
extern videoSource* input;
extern uchar3* image;
extern int numDetections;

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

/************************************
 * Exported objects
 ************************************/

/************************************
 * Function prototypes
 ************************************/
class Follow
{
    public:
        Follow();
        ~Follow();

        static void follow_target(void);
        static void overtake_target(void);
};

#endif // FOLLOW_TARGET_H
