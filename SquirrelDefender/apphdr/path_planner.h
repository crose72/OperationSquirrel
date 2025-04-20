/********************************************************************************
 * @file    path_planner
 * @author  Shaun Bowman
 * @date    2025/04/19
 ********************************************************************************/
#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include <vector>

/********************************************************************************
 * Imported objects
 ********************************************************************************/

/********************************************************************************
 * Exported objects
 ********************************************************************************/
extern float g_vx_cmd;
extern float g_vy_cmd;
extern float g_vz_cmd;
extern float g_yaw_cmd;

/**
 * @brief Enum class for path planner types
 */
enum class PathPlannerType {
    DELIVERY, // payload delivery
    FOLLOWER // follow at an offset
};

/********************************************************************************
 * Function prototypes
 ********************************************************************************/
class PathPlannerBase
{
public:
    PathPlannerBase() = default;
    virtual ~PathPlanner() = default;

    virtual bool init(void) const = 0;
    virtual void loop(void) const = 0;
    virtual void shutdown(void) const = 0;

};

class PathPlanner : public PathPlannerBase
{
public:
    PathPlanner();
    ~PathPlanner() override;

    bool init(void) const override;
    void loop(void) const override;
    void shutdown(void) const override;
}

#endif // PATH_PLANNER_H
