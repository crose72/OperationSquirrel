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
extern bool g_tgt_too_close;
extern float g_pos_err_x;
extern float g_pos_err_y;
extern float g_ctrl_vel_x_cmd;
extern float g_ctrl_vel_y_cmd;
extern float g_ctrl_vel_z_cmd;
extern float g_ctrl_yaw_tgt;
extern float g_ctrl_yaw_cmd;
extern float g_veh_yaw_playback_adj;
extern float g_yaw_err;
extern float g_veh_vel_x_est;
extern float g_veh_vel_y_est;
extern float g_pos_err_x_dot;

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
