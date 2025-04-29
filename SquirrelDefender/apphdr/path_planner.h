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
#include "path_planner_types.h"
#include <vector>

/********************************************************************************
 * Imported objects
 ********************************************************************************/

/********************************************************************************
 * Exported objects
 ********************************************************************************/

/********************************************************************************
 * Function prototypes
 ********************************************************************************/
namespace path_planner {

class PathPlannerBase {
public:
    PathPlannerBase() = default;
    virtual ~PathPlannerBase() = default;

    // Initialize the planner
    virtual bool init() = 0;

    // Get the current path as a sequence of waypoints
    virtual std::vector<Waypoint> getPath() const = 0;

    // Set the target waypoint
    virtual void setTarget(const Waypoint& target) = 0;

    // Clean up resources
    virtual void shutdown() = 0;

    // Get the type of this planner
    virtual Type getType() const = 0;
};

// Factory function to create a planner of the specified type
// Returns nullptr if type is invalid
PathPlannerBase* createPlanner(Type type);

// Destroy a planner created by createPlanner
void destroyPlanner(PathPlannerBase* planner);

} // namespace path_planner

#endif // PATH_PLANNER_H
