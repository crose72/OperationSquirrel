/********************************************************************************
 * @file    path_planner_example.cpp
 * @author  Shaun Bowman
 * @date    2025/04/19
 * @brief   Example implementation showing how to use the path planners
 ********************************************************************************/

#include "path_planner/path_planner.h"
#include "path_planner/path_planner_types.h"
#include "path_planner/path_planner_grid.h"
#include "path_planner/path_planner_delivery.h"

#include <iostream>
#include <memory>

namespace path_planner {

class PathPlannerExample {
public:
    PathPlannerExample() = default;
    ~PathPlannerExample() = default;

    void runExample() {
        // Example 1: Using the Grid Planner
        std::cout << "\n=== Grid Planner Example ===" << std::endl;
        auto grid_planner = std::unique_ptr<PathPlannerBase>(path_planner::createPlanner(Type::GRID));
        
        // Configure grid
        GridConfig grid_config;
        grid_config.rows = 3;
        grid_config.cols = 3;
        grid_config.cell_size = 5.0f;
        grid_config.altitude = 10.0f;
        grid_config.speed = 2.0f;
        grid_config.snake_pattern = true;
        
        // Cast to GridPlanner to set configuration
        auto* grid_planner_ptr = dynamic_cast<GridPlanner*>(grid_planner.get());
        if (grid_planner_ptr) {
            grid_planner_ptr->setGridConfig(grid_config);
            
            // Set start position
            Waypoint start_pos;
            start_pos.position.x = 0.0f;
            start_pos.position.y = 0.0f;
            start_pos.position.z = 0.0f;
            grid_planner_ptr->setTarget(start_pos);
            
            // Get and print the path
            auto grid_path = grid_planner_ptr->getPath();
            std::cout << "Grid path has " << grid_path.size() << " waypoints:" << std::endl;
            for (const auto& wp : grid_path) {
                std::cout << "  Waypoint: (" << wp.position.x << ", " 
                          << wp.position.y << ", " << wp.position.z << ")" << std::endl;
            }
        }

        // Example 2: Using the Delivery Planner
        std::cout << "\n=== Delivery Planner Example ===" << std::endl;
        auto delivery_planner = std::unique_ptr<PathPlannerBase>(path_planner::createPlanner(Type::DELIVERY));
        
        // Set target position
        Waypoint target;
        target.position.x = 10.0f;
        target.position.y = 20.0f;
        target.position.z = 5.0f;
        target.yaw = 0.0f;
        target.tolerance = 0.5f;
        
        delivery_planner->setTarget(target);
        
        // Get and print the path
        auto delivery_path = delivery_planner->getPath();
        std::cout << "Delivery path has " << delivery_path.size() << " waypoints:" << std::endl;
        for (const auto& wp : delivery_path) {
            std::cout << "  Waypoint: (" << wp.position.x << ", " 
                      << wp.position.y << ", " << wp.position.z << ")" << std::endl;
        }
    }
};

} // namespace path_planner

// Example usage
int main() {
    path_planner::PathPlannerExample example;
    example.runExample();
    return 0;
} 