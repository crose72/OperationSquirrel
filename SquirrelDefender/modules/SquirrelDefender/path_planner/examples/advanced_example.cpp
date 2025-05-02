#include <planner/planner_astar.h>
#include <iostream>
#include <array>
#include <random>
#include <chrono>

int main() {
    // Define grid dimensions
    constexpr size_t GRID_WIDTH = 50;
    constexpr size_t GRID_HEIGHT = 50;

    // Create planner instance with larger path length
    planner::PlannerAStar<GRID_WIDTH, GRID_HEIGHT> planner(1000);

    // Create obstacle map with random obstacles
    std::array<std::array<bool, GRID_HEIGHT>, GRID_WIDTH> obstacle_map{};
    
    // Initialize random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);

    // Fill map with random obstacles (30% chance)
    for (size_t i = 0; i < GRID_WIDTH; ++i) {
        for (size_t j = 0; j < GRID_HEIGHT; ++j) {
            obstacle_map[i][j] = (dis(gen) < 0.3);
        }
    }

    // Ensure start and goal are not obstacles
    obstacle_map[0][0] = false;
    obstacle_map[GRID_WIDTH-1][GRID_HEIGHT-1] = false;

    // Initialize planner
    if (!planner.initialize(obstacle_map)) {
        std::cerr << "Failed to initialize planner" << std::endl;
        return 1;
    }

    // Define start and goal points
    planner::Point start{0, 0};
    planner::Point goal{static_cast<int32_t>(GRID_WIDTH-1), static_cast<int32_t>(GRID_HEIGHT-1)};

    // Time the path planning
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Plan path
    planner::Path path = planner.planPath(start, goal);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    // Print results
    if (path.isValid()) {
        std::cout << "Path found in " << duration.count() << "ms!" << std::endl;
        std::cout << "Path length: " << path.getPoints().size() << " points" << std::endl;
        
        // Print the first 10 and last 10 points
        const auto& points = path.getPoints();
        for (size_t i = 0; i < std::min(size_t(10), points.size()); ++i) {
            std::cout << "(" << points[i].x << ", " << points[i].y << ")" << std::endl;
        }
        if (points.size() > 20) {
            std::cout << "..." << std::endl;
            for (size_t i = points.size() - 10; i < points.size(); ++i) {
                std::cout << "(" << points[i].x << ", " << points[i].y << ")" << std::endl;
            }
        }
    } else {
        std::cout << "No valid path found" << std::endl;
    }

    return 0;
} 