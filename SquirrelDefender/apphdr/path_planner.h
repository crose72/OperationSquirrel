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
extern float g_mav_veh_yaw_adjusted_for_playback;
extern float g_yaw_target_error;
extern float g_veh_vx_est;
extern float g_veh_vy_est;
extern float g_x_error_dot;

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
