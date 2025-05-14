/********************************************************************************
 * @file    path_planner_delivery.h
 * @author  Shaun Bowman
 * @date    2025/04/19
 ********************************************************************************/
#ifndef PATH_PLANNER_DELIVERY_H
#define PATH_PLANNER_DELIVERY_H

#include "path_planner.h"
#include <vector>

namespace path_planner {

class DeliveryPlanner : public PathPlannerBase {
public:
    DeliveryPlanner();
    ~DeliveryPlanner() override = default;

    bool init() override;
    std::vector<Waypoint> getPath() const override;
    void setTarget(const Waypoint& target) override;
    void shutdown() override;
    Type getType() const override { return Type::DELIVERY; }

private:
    Waypoint m_target;
    std::vector<Waypoint> m_waypoints;
    bool m_initialized{false};
};

} // namespace path_planner

#endif // PATH_PLANNER_DELIVERY_H
