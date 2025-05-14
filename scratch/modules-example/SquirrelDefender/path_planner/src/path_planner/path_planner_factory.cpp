/********************************************************************************
 * @file    path_planner_factory.cpp
 * @author  Shaun Bowman
 * @date    2025/04/19
 ********************************************************************************/

#include "path_planner/path_planner.h"
#include "path_planner/path_planner_delivery.h"
#include "path_planner/path_planner_grid.h"
#include "path_planner/path_planner_types.h"
#include <stdexcept>

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