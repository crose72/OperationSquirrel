#pragma once

#include "types.h"
#include <array>
#include <cstddef>

namespace path_planner {

/**
 * @brief Abstract base class for path planning algorithms
 * 
 * @tparam Derived The derived class type (CRTP pattern)
 */
template<typename Derived>
class PathPlannerBase {
public:
    virtual ~PathPlannerBase() = default;

    /**
     * @brief Initialize the planner with an obstacle map
     * 
     * @tparam WIDTH Grid width
     * @tparam HEIGHT Grid height
     * @param obstacle_map 2D array representing obstacles (true = obstacle)
     * @return true if initialization successful
     */
    template<std::size_t WIDTH, std::size_t HEIGHT>
    bool initialize(const std::array<std::array<bool, HEIGHT>, WIDTH>& obstacle_map) {
        return static_cast<Derived*>(this)->doInitialize(obstacle_map);
    }

    /**
     * @brief Plan a path from start to goal
     * 
     * @param start Starting point
     * @param goal Goal point
     * @return Path object containing the planned path
     */
    virtual Path planPath(const Point& start, const Point& goal) = 0;
};

/**
 * @brief Interface class for path planning algorithms
 */
class PathPlanner : public PathPlannerBase<PathPlanner> {
public:
    virtual ~PathPlanner() = default;

    /**
     * @brief Plan a path from start to goal
     * 
     * @param start Starting point
     * @param goal Goal point
     * @return Path object containing the planned path
     */
    virtual Path planPath(const Point& start, const Point& goal) = 0;

protected:
    /**
     * @brief Internal implementation of initialize
     */
    template<std::size_t WIDTH, std::size_t HEIGHT>
    bool doInitialize(const std::array<std::array<bool, HEIGHT>, WIDTH>& obstacle_map) {
        return true;  // Default implementation
    }
};

} // namespace path_planner 