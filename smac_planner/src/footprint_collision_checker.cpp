// based on: nav2_costmap_2d/footprint_collision_checker.hpp

#include "smac_planner/footprint_collision_checker.hpp"

// ROS
#include <nav2_util/line_iterator.hpp>
#include <nav2_costmap_2d/cost_values.hpp>


namespace smac_planner
{
double
FootprintCollisionChecker::lineCost(const nav2_costmap_2d::Costmap2D *costmap, int x0, int x1,
                                    int y0,
                                    int y1)
{
  unsigned char line_cost = nav2_costmap_2d::FREE_SPACE;

  for (nav2_util::LineIterator line(x0, y0, x1, y1); line.isValid(); line.advance()) {
    line_cost = std::max(line_cost, costmap->getCost(line.getX(), line.getY()));
  }

  return static_cast<double>(line_cost);
}

double smac_planner::FootprintCollisionChecker::footprintCost(
  const nav2_costmap_2d::Costmap2D *costmap,
  const Footprint & footprint)
{
  // now we really have to lay down the footprint in the costmap grid
  unsigned int x0, x1, y0, y1;
  double footprint_cost = 0.0;

  // we need to rasterize each line in the footprint
  for (unsigned int i = 0; i < footprint.size() - 1; ++i) {
    // get the cell coord of the first point
    if (!costmap->worldToMap(footprint[i].x, footprint[i].y, x0, y0)) {
      return static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE);
    }

    // get the cell coord of the second point
    if (!costmap->worldToMap(footprint[i + 1].x, footprint[i + 1].y, x1, y1)) {
      return static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE);
    }

    footprint_cost = std::max(lineCost(costmap, x0, x1, y0, y1), footprint_cost);

    if (footprint_cost >= static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE)) {
      return footprint_cost;
    }
  }

  // we also need to connect the first point in the footprint to the last point
  // get the cell coord of the last point
  if (!costmap->worldToMap(footprint.back().x, footprint.back().y, x0, y0)) {
    return static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE);
  }

  // get the cell coord of the first point
  if (!costmap->worldToMap(footprint.front().x, footprint.front().y, x1, y1)) {
    return static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE);
  }

  footprint_cost = std::max(lineCost(costmap, x0, x1, y0, y1), footprint_cost);

  return footprint_cost;
}

double FootprintCollisionChecker::footprintCostAtPose(const nav2_costmap_2d::Costmap2D *costmap,
                                                      std::vector<geometry_msgs::msg::Point> footprint,
                                                      double x, double y,
                                                      double theta)
{
  const double cos_th = std::cos(theta);
  const double sin_th = std::sin(theta);
  Footprint oriented_footprint;

  for (const auto & i : footprint) {
    geometry_msgs::msg::Point new_pt;
    new_pt.x = x + (i.x * cos_th - i.y * sin_th);
    new_pt.y = y + (i.x * sin_th + i.y * cos_th);

    oriented_footprint.push_back(new_pt);
  }

  return footprintCost(costmap, oriented_footprint);
}

}  // namespace smac_planner
