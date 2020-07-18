// based on: nav2_costmap_2d/footprint_collision_checker.hpp

#ifndef SMAC_PLANNER__FOOTPRINT_COLLISION_CHECKER_HPP_
#define SMAC_PLANNER__FOOTPRINT_COLLISION_CHECKER_HPP_

// C++
#include <vector>

// ROS
#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>


namespace smac_planner
{
typedef std::vector<geometry_msgs::msg::Point> Footprint;

class FootprintCollisionChecker {
public:
    static double lineCost(const nav2_costmap_2d::Costmap2D *costmap, int x0, int x1, int y0, int y1);

    static double footprintCost(const nav2_costmap_2d::Costmap2D *costmap, const Footprint & footprint);

    static double
    footprintCostAtPose(const nav2_costmap_2d::Costmap2D *costmap, std::vector<geometry_msgs::msg::Point> footprint, double x,
                        double y, double theta);
};

}  // namespace smac_planner

#endif  // SMAC_PLANNER__FOOTPRINT_COLLISION_CHECKER_HPP_
