#include <planner/planner_astar.h>
#include <iostream>
#include <array>

int main() {
    // Define grid dimensions
    constexpr size_t GRID_WIDTH = 10;
    constexpr size_t GRID_HEIGHT = 10;

    // Create planner instance
    planner::PlannerAStar<GRID_WIDTH, GRID_HEIGHT> planner(100);

    // Create obstacle map (simple example with a wall in the middle)
    std::array<std::array<bool, GRID_HEIGHT>, GRID_WIDTH> obstacle_map{};
    for (size_t i = 0; i < GRID_WIDTH; ++i) {
        for (size_t j = 0; j < GRID_HEIGHT; ++j) {
            obstacle_map[i][j] = false;
        }
    }
    // Add a wall in the middle
    for (size_t i = 2; i < 8; ++i) {
        obstacle_map[i][5] = true;
    }

    // Initialize planner
    if (!planner.initialize(obstacle_map)) {
        std::cerr << "Failed to initialize planner" << std::endl;
        return 1;
    }

    // Define start and goal points
    planner::Point start{0, 0};
    planner::Point goal{9, 9};

    // Plan path
    planner::Path path = planner.planPath(start, goal);

    // Print results
    if (path.isValid()) {
        std::cout << "Path found!" << std::endl;
        for (const auto& point : path.getPoints()) {
            std::cout << "(" << point.x << ", " << point.y << ")" << std::endl;
        }
    } else {
        std::cout << "No valid path found" << std::endl;
    }

    return 0;
} 