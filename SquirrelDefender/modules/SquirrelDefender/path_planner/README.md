# Path Planner Module

The Path Planner module provides efficient path planning capabilities for the SquirrelDefender system, with a focus on embedded system deployment considerations.

## Class Structure

The module is organized into the following key components:

### Core Classes

1. `PathPlanner` (Base Class)
   - Abstract base class defining the interface for path planning
   - Handles memory management and resource allocation
   - Provides common utilities for path planning operations

2. `AStarPlanner` (Concrete Implementation)
   - Implements A* algorithm for path finding
   - Optimized for embedded systems with fixed-size memory allocation
   - Uses template parameters for grid size and node type

3. `Path` (Data Structure)
   - Represents a planned path
   - Uses fixed-size arrays to avoid dynamic memory allocation
   - Includes path metadata and validation

### Memory Management

The module is designed with embedded systems in mind:

- All memory is pre-allocated at initialization
- No dynamic memory allocation during runtime
- Fixed-size containers using `std::array` instead of `std::vector`
- Template parameters for grid dimensions to enable compile-time optimization

## Example Usage

```cpp
#include "path_planner/astar_planner.h"

// Define grid dimensions at compile time
constexpr size_t GRID_WIDTH = 100;
constexpr size_t GRID_HEIGHT = 100;

// Create planner instance
AStarPlanner<GRID_WIDTH, GRID_HEIGHT> planner;

// Initialize with obstacle map
std::array<std::array<bool, GRID_HEIGHT>, GRID_WIDTH> obstacle_map;
// ... populate obstacle_map ...

planner.initialize(obstacle_map);

// Plan path from start to goal
Point start{0, 0};
Point goal{99, 99};
Path path = planner.planPath(start, goal);

// Use the path
if (path.isValid()) {
    for (const auto& point : path.getPoints()) {
        // Process path points
    }
}
```

## Build Integration

### CMake Configuration

The path_planner module is integrated into the main SquirrelDefender build system. To use it:

1. Add the following to your module's CMakeLists.txt:

```cmake
add_subdirectory(path_planner)
target_link_libraries(your_target PRIVATE path_planner)
```

2. Include the headers in your source files:

```cpp
#include "path_planner/astar_planner.h"
```

### Build Options

The module supports the following CMake options:

- `PATH_PLANNER_ENABLE_DEBUG`: Enable debug logging (default: OFF)
- `PATH_PLANNER_ENABLE_TESTS`: Build unit tests (default: OFF)

### Dependencies

- C++17 or later
- Eigen3 (for matrix operations)
- Armadillo (for numerical computations)

## Memory Considerations

1. **Static Allocation**
   - Grid dimensions must be known at compile time
   - Memory usage is predictable and fixed
   - No heap fragmentation issues

2. **Cache Optimization**
   - Data structures are aligned for optimal cache usage
   - Contiguous memory layout for path points
   - Minimized pointer chasing

3. **Stack Usage**
   - Limited recursion depth in path planning algorithms
   - Stack-allocated temporary variables
   - Configurable maximum path length

## Performance Guidelines

1. **Grid Size**
   - Keep grid dimensions as small as possible
   - Consider using hierarchical planning for large areas
   - Balance between resolution and memory usage

2. **Update Frequency**
   - Plan paths at a fixed interval
   - Cache results when possible
   - Use incremental updates for small changes

3. **Real-time Considerations**
   - Worst-case execution time is bounded
   - No dynamic memory allocation during planning
   - Configurable timeout for path planning

## Integration Notes

1. **Thread Safety**
   - The planner is not thread-safe by default
   - Use mutex protection if accessing from multiple threads
   - Consider creating separate planner instances per thread

2. **Error Handling**
   - All operations return status codes
   - Invalid paths are clearly marked
   - Memory allocation failures are handled gracefully

3. **Testing**
   - Unit tests available in the `tests` directory
   - Memory usage can be verified using the test harness
   - Performance benchmarks included
