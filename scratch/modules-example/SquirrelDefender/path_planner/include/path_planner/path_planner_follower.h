
/********************************************************************************
 * @file    path_planner_follower.h
 * @author  Shaun Bowman
 * @date    2025/04/19
 ********************************************************************************/
#ifndef PATH_PLANNER_FOLLOWER_H
#define PATH_PLANNER_FOLLOWER_H

#include "path_planner.h"
#include <vector>

namespace path_planner {

class FollowerPlanner : public PathPlannerBase {
public:
    FollowerPlanner();
    ~FollowerPlanner() override = default;

    bool init() override;
    std::vector<Waypoint> getPath() const override;
    void setTarget(const Waypoint& target) override;
    void shutdown() override;
    Type getType() const override { return Type::FOLLOWER; }

private:
    Waypoint m_target;
    std::vector<Waypoint> m_waypoints;
    bool m_initialized{false};
};

} // namespace path_planner

#endif // PATH_PLANNER_FOLLOWER_H
