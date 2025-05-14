/********************************************************************************
 * @file    path_planner_grid.h
 * @author  Shaun Bowman
 * @date    2025/04/19
 ********************************************************************************/
#ifndef PATH_PLANNER_GRID_H
#define PATH_PLANNER_GRID_H

#include "path_planner.h"
#include <vector>

namespace path_planner {

class GridPlanner : public PathPlannerBase {
public:
    GridPlanner();
    ~GridPlanner() override = default;

    bool init() override;
    std::vector<Waypoint> getPath() const override;
    void setTarget(const Waypoint& target) override;
    void shutdown() override;
    Type getType() const override { return Type::GRID; }

    // Set grid configuration
    void setGridConfig(const GridConfig& config);

private:
    // Generate grid waypoints based on start position and configuration
    void generateGridWaypoints(const Position& start_pos);

    GridConfig m_config;
    std::vector<Waypoint> m_waypoints;
    std::size_t m_current_waypoint_idx{0};
    bool m_initialized{false};
    Position m_start_position;
    bool m_reverse_direction{false};
};

} // namespace path_planner

#endif // PATH_PLANNER_GRID_H 