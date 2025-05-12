/********************************************************************************
 * @file    path_planner
 * @author  Cameron Rose
 * @date    3/12/2025
 ********************************************************************************/
#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include "param_reader.h"
#include "pid.h"
#include "localize_target.h"
#include "time_calc.h"
#include "track_target.h"
#include "mav_data_hub.h"
#include "signal_processing.h"

/********************************************************************************
 * Imported objects
 ********************************************************************************/
extern uint16_t g_mav_veh_rngfdr_min_distance;
extern uint16_t g_mav_veh_rngfdr_max_distance;
extern uint16_t g_mav_veh_rngfdr_current_distance;
extern float g_mav_veh_pitch;
extern bool g_target_valid;
extern float g_x_target;
extern float g_y_target;
extern float g_z_target;
extern float g_dt;
extern float g_x_target_ekf;
extern float g_y_target_ekf;
extern bool g_first_loop_after_start;
extern float g_mav_veh_yaw;
extern float g_mav_veh_yawspeed;
extern bool g_use_video_playback;
extern float g_target_cntr_offset_y;
extern float g_target_cntr_offset_x;
extern bool g_target_loc_data_ok;

/********************************************************************************
 * Exported objects
 ********************************************************************************/
extern bool g_target_too_close;
extern float g_x_error;
extern float g_y_error;
extern float g_vx_adjust;
extern float g_vy_adjust;
extern float g_vz_adjust;
extern float g_yaw_target;
extern float g_yaw_adjust;
extern float g_mav_veh_yaw_adjusted;
extern float g_yaw_target_error;
extern float g_target_cntr_offset_x_filt;

/********************************************************************************
 * Function prototypes
 ********************************************************************************/
class PathPlanner
{
public:
    PathPlanner();
    ~PathPlanner();

    static bool init(void);
    static void loop(void);
    static void shutdown(void);

private:
};

#endif // PATH_PLANNER_H
