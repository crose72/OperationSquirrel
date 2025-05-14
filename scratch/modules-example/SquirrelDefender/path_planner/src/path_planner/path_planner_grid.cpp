/********************************************************************************
 * @file    path_planner_grid.cpp
 * @author  Shaun Bowman
 * @date    2025/04/19
 ********************************************************************************/

#include "path_planner/path_planner_grid.h"
#include "path_planner/path_planner_types.h"
#include <cmath>
#include <algorithm>

namespace path_planner {

GridPlanner::GridPlanner() : m_config{}, m_waypoints{}, m_current_waypoint_idx{0}, 
                            m_initialized{false}, m_start_position{}, m_reverse_direction{false} {}

bool GridPlanner::init() {
    if (m_initialized) {
        return true;
    }
    
    // Initialize with default grid configuration
    m_initialized = true;
    return true;
}

void GridPlanner::setGridConfig(const GridConfig& config) {
    m_config = config;
    if (m_initialized) {
        generateGridWaypoints(m_start_position);
    }
}

void GridPlanner::setTarget(const Waypoint& target) {
    m_start_position = target.position;
    generateGridWaypoints(m_start_position);
}

void GridPlanner::generateGridWaypoints(const Position& start_pos) {
    m_waypoints.clear();
    m_current_waypoint_idx = 0;
    m_reverse_direction = false;

    // Generate waypoints in a grid pattern
    for (uint8_t row = 0; row < m_config.rows; ++row) {
        // Determine direction for this row (alternate for snake pattern)
        bool reverse = m_config.snake_pattern && (row % 2 == 1);
        
        for (uint8_t col = 0; col < m_config.cols; ++col) {
            uint8_t actual_col = reverse ? (m_config.cols - 1 - col) : col;
            
            Waypoint wp;
            wp.position.x = start_pos.x + (actual_col * m_config.cell_size);
            wp.position.y = start_pos.y + (row * m_config.cell_size);
            wp.position.z = m_config.altitude;
            wp.yaw = 0.0f;  // Keep yaw constant for now
            wp.tolerance = m_config.cell_size * 0.2f;  // 20% of cell size
            
            m_waypoints.push_back(wp);
        }
    }
}

std::vector<Waypoint> GridPlanner::getPath() const {
    return m_waypoints;
}

void GridPlanner::shutdown() {
    m_initialized = false;
    m_waypoints.clear();
    m_current_waypoint_idx = 0;
}

} // namespace path_planner 