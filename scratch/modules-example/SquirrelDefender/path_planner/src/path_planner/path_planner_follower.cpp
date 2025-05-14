/********************************************************************************
 * @file    path_planner_follower.cpp
 * @author  Shaun Bowman
 * @date    2025/04/19
 ********************************************************************************/

#include "path_planner/path_planner_follower.h"
#include "path_planner/path_planner_types.h"
#include <cmath>

namespace path_planner {

FollowerPlanner::FollowerPlanner() : m_target{}, m_waypoints{}, m_initialized{false} {}

bool FollowerPlanner::init() {
    if (m_initialized) {
        return true;
    }
    
    // Initialize any required resources here
    m_initialized = true;
    return true;
}

std::vector<Waypoint> FollowerPlanner::getPath() const {
    return m_waypoints;
}

void FollowerPlanner::setTarget(const Waypoint& target) {
    m_target = target;
    
    // Generate a simple path to the target
    m_waypoints.clear();
    
    // Add the target as the only waypoint
    m_waypoints.push_back(target);
}

void FollowerPlanner::shutdown() {
    m_initialized = false;
    m_waypoints.clear();
}

} // namespace path_planner 