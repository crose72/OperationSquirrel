/********************************************************************************
 * @file    path_planner
 * @author  Shaun Bowman
 * @date    2025/04/19
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "path_planner.h"
#include <iostream>

/********************************************************************************
 * Imported objects
 ********************************************************************************/

/********************************************************************************
 * Exported objects
 ********************************************************************************/
// Define the static member variable
PathPlannerType PathPlanner::m_type;

/********************************************************************************
 * Function defnitions
 ********************************************************************************/
PathPlanner::PathPlanner(PathPlannerType type) : m_activePlanner(nullptr) {
    // Init 
    this->m_type = type;
}

PathPlanner::~PathPlanner() {
    // Cleanup
}

bool PathPlanner::init(void) const {
    // Implementation
    std::cout << "init PathPlanner" << std::endl;
    return true;
}

void PathPlanner::loop(void) const {
    // Implementation
    std::cout << "Looping PathPlanner" << std::endl;
}

void PathPlanner::shutdown(void) const {
    // Implementation
    std::cout << "Shutdown PathPlanner" << std::endl;
}