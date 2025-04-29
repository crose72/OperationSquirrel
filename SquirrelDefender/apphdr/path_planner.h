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
#include <vector>
#include <memory>

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
    virtual ~PathPlannerBase() = default;

    virtual bool init(void) const = 0;
    virtual void loop(void) const = 0;
    virtual void shutdown(void) const = 0;

};

class PathPlanner : public PathPlannerBase
{
public:
    PathPlanner(PathPlannerType type);
    virtual ~PathPlanner();

    bool init(void) const override;
    void loop(void) const override;
    void shutdown(void) const override;

    // Type of the path planner
    static PathPlannerType m_type;

    // Factory method - get static instace of specific planner type
    static std::shared_ptr<const PathPlannerBase> getPlanner(PathPlannerType type);


private:
    // Pointer to the active planner (points to a static instance)
    mutable std::shared_ptr<PathPlannerBase> m_activePlanner;

};

#endif // PATH_PLANNER_H
