/********************************************************************************
 * @file    path_planner_delivery.cpp
 * @author  Shaun Bowman
 * @date    2025/04/19
 ********************************************************************************/

#include "path_planner/path_planner_delivery.h"
#include "path_planner/path_planner_types.h"
#include <cmath>

namespace path_planner {

DeliveryPlanner::DeliveryPlanner() : m_target{}, m_waypoints{}, m_initialized{false} {}

bool DeliveryPlanner::init() {
    if (m_initialized) {
        return true;
    }
    
    // Initialize any required resources here
    m_initialized = true;
    return true;
}

std::vector<Waypoint> DeliveryPlanner::getPath() const {
    return m_waypoints;
}

void DeliveryPlanner::setTarget(const Waypoint& target) {
    m_target = target;
    
    // Generate a simple path to the target
    m_waypoints.clear();
    
    // Add the target as the only waypoint
    m_waypoints.push_back(target);
}

void DeliveryPlanner::shutdown() {
    m_initialized = false;
    m_waypoints.clear();
}

} // namespace path_planner 