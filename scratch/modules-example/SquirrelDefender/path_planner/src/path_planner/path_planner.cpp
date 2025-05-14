/********************************************************************************
 * @file    path_planner.cpp
 * @author  Cameron Rose
 * @date    3/12/2025
 * @brief   The path planner contains the MPC setup and solving for the optimal
 *          flight path to deliver a payload to a moving target.  An external
 *          MPC library will be used to solve the MPC problem statement.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "path_planner/path_planner.h"
#include "path_planner/path_planner_delivery.h"
#include "path_planner/path_planner_grid.h"
#include "path_planner/path_planner_follower.h"
#include <stdexcept>

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/

namespace path_planner {

PathPlannerBase* createPlanner(Type type) {
    switch (type) {
        case Type::DELIVERY:
            return new DeliveryPlanner();
        case Type::GRID:
            return new GridPlanner();
        case Type::FOLLOWER:
            // TODO: Implement follower planner
            throw std::runtime_error("Follower planner not implemented");
        default:
            throw std::runtime_error("Invalid planner type");
    }
}

void destroyPlanner(PathPlannerBase* planner) {
    if (planner) {
        delete planner;
    }
}

} // namespace path_planner