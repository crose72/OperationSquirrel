#pragma once

#include "path_planner_types.h"
#include <array>
#include <cstddef>
#include <vector>

namespace path_planner {

using Path = std::vector<Waypoint>;

/**
 * @brief Abstract base class for path planning algorithms
 */
class PathPlannerBase {
public:
    virtual ~PathPlannerBase() = default;

    /**
     * @brief Initialize the planner
     * @return true if initialization successful
     */
    virtual bool init() = 0;

    /**
     * @brief Get the current path
     * @return vector of waypoints representing the path
     */
    virtual std::vector<Waypoint> getPath() const = 0;

    /**
     * @brief Set the target waypoint
     * @param target Target waypoint
     */
    virtual void setTarget(const Waypoint& target) = 0;

    /**
     * @brief Shutdown the planner
     */
    virtual void shutdown() = 0;

    /**
     * @brief Get the planner type
     * @return Type The type of the planner
     */
    virtual Type getType() const = 0;
};

/**
 * @brief Create a new path planner instance
 * @param type The type of planner to create
 * @return Pointer to the new planner instance
 */
PathPlannerBase* createPlanner(Type type);

/**
 * @brief Destroy a path planner instance
 * @param planner Pointer to the planner to destroy
 */
void destroyPlanner(PathPlannerBase* planner);

} // namespace path_planner
