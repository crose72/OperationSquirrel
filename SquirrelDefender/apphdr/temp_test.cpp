// temp_test.cpp
// for testing headers compile

#include "path_planner.h"
#include <iostream>

int main() {
    std::cout << "Starting" << std::endl;
    PathPlanner pp = PathPlanner(PathPlannerType::DELIVERY);
    pp.init();
    pp.loop();
    std::cout << "Active planner type: " << static_cast<int>(pp.m_type) << std::endl;
    pp.shutdown();

    return 0;
}