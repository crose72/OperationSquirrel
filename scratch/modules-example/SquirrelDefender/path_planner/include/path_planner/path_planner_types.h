/********************************************************************************
 * @file    path_planner_types.h
 * @author  Shaun Bowman
 * @date    2025/04/19
 ********************************************************************************/
#ifndef PATH_PLANNER_TYPES_H
#define PATH_PLANNER_TYPES_H

#include <cstdint>
#include <array>

namespace path_planner {

// Strongly typed enums for path planner types
enum class Type : uint8_t {
    DELIVERY,  // payload delivery
    FOLLOWER,  // follow at an offset
    GRID       // snake-like grid pattern
};

// Strongly typed velocity commands
struct VelocityCommands {
    float vx{0.0f};  // m/s
    float vy{0.0f};  // m/s
    float vz{0.0f};  // m/s
    float yaw{0.0f}; // rad
};

// Strongly typed position
struct Position {
    float x{0.0f};  // m
    float y{0.0f};  // m
    float z{0.0f};  // m
};

// Strongly typed waypoint
struct Waypoint {
    Position position;
    float yaw{0.0f};  // rad
    float tolerance{0.1f};  // m
};

// Grid configuration for grid planner
struct GridConfig {
    float cell_size{5.0f};      // m, size of each grid cell
    uint8_t rows{3};            // number of rows in grid
    uint8_t cols{3};            // number of columns in grid
    float altitude{10.0f};      // m, fixed altitude for grid flight
    float speed{2.0f};          // m/s, speed for grid traversal
    bool snake_pattern{true};   // true for snake pattern, false for row-by-row
};

} // namespace path_planner

#endif // PATH_PLANNER_TYPES_H 