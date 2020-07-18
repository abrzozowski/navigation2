#ifndef SMAC_PLANNER__OMPL_PLANNER_HPP_
#define SMAC_PLANNER__OMPL_PLANNER_HPP_

// OMPL
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include <ompl/geometric/SimpleSetup.h>

// ROS
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/cost_values.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <tf2/utils.h>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <nav2_util/lifecycle_node.hpp>

// Project
#include "smac_planner/footprint_collision_checker.hpp"


namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace smac_planner
{

class OMPLPlanner {
public:

    void configure(rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
                   nav2_costmap_2d::Costmap2D *costmap,
                   const std::vector<geometry_msgs::msg::Point> robot_footprint);

    bool isStateValid(const nav2_costmap_2d::Costmap2D *constmap,
                      const std::vector<geometry_msgs::msg::Point> & robot_footprint,
                      const geometry_msgs::msg::PoseStamped & goal,
                      const ompl::base::State *state);

    nav_msgs::msg::Path run(const geometry_msgs::msg::PoseStamped & start,
                            const geometry_msgs::msg::PoseStamped & goal);

    static void poseStampedToScopedState(const geometry_msgs::msg::PoseStamped & pose_stamped,
                                         ob::ScopedState<ob::SE2StateSpace> & scoped_state);

    static void
    getXYThetaFromState(const ob::StateSpacePtr state_space, const ob::State *s, double *x,
                        double *y, double *th);

protected:
    nav2_util::LifecycleNode::SharedPtr _node;

    nav2_costmap_2d::Costmap2D *_costmap;
    std::vector<geometry_msgs::msg::Point> _robot_footprint;

    ob::StateSpacePtr _state_space;
    og::SimpleSetupPtr _ss;
};

}  // namespace smac_planner

#endif  // SMAC_PLANNER__OMPL_PLANNER_HPP_
