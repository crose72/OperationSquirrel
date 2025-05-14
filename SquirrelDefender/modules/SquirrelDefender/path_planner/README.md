# Path Planner Module

The Path Planner module provides efficient path planning capabilities for the SquirrelDefender system, with a focus on embedded system deployment considerations.

## Class Structure

The module is organized into the following key components:

### Core Classes

1. `PathPlanner` (Base Class)
   - Abstract base class defining the interface for path planning
   - Handles memory management and resource allocation
   - Provides common utilities for path planning operations

2. `GridPlanner` (Concrete Implementation)
   - Plans a "space filling" grid pattern
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

see examples/src/path_planner_example.cpp


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
Here are all the essential commands for building and running the path planner project:

1. Clean Build (from project root):
```code
rm -rf build
mkdir build
cd build
```

2. Configure with CMake (from build directory):
```code
# Debug build
cmake -DCMAKE_BUILD_TYPE=Debug ..

# Release build
cmake -DCMAKE_BUILD_TYPE=Release ..
```

3. Build (from build directory):
```
make -j6       # Build all targets
```

4. Run executables (from build directory):
```
# Run the example program
./path_planner_example

# Run the tests
./path_planner_test
```

## VSCode integration

Using the VSCode "Run and Debug" tool, you can select from the following

1. Debug Example : build debug target `/examples/src/path_planner_example.cpp`

2. Debug Tests : build debug target `/tests/src/test_path_planner.cpp`