#include "smac_planner/ompl_planner.hpp"

// OMPL
#include <ompl/geometric/planners/rrt/RRTstar.h>

// ROS
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>


namespace smac_planner
{
void OMPLPlanner::configure(rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
                            nav2_costmap_2d::Costmap2D *costmap,
                            std::vector<geometry_msgs::msg::Point> robot_footprint)
{
  _node = parent;

  _costmap = costmap;
  _robot_footprint = robot_footprint;

  ob::RealVectorBounds bounds(2);
  {
    bounds.setLow(0, _costmap->getOriginX());
    bounds.setHigh(0, _costmap->getSizeInMetersX() +
                      _costmap->getOriginX());
    bounds.setLow(1, _costmap->getOriginY());
    bounds.setHigh(1, _costmap->getSizeInMetersY() +
                      _costmap->getOriginY());
  }

  _state_space = std::make_shared<ob::ReedsSheppStateSpace>();
  _state_space->as<ob::SE2StateSpace>()->setBounds(bounds);

  _ss = std::make_shared<og::SimpleSetup>(_state_space);

  _ss->getSpaceInformation()->setStateValidityCheckingResolution(0.1);

  ob::OptimizationObjectivePtr lengthObj(
    new ob::PathLengthOptimizationObjective(_ss->getSpaceInformation()));
  //  lengthObj->setCostThreshold(ob::Cost(500.0));

  _ss->setOptimizationObjective(lengthObj);
}

nav_msgs::msg::Path OMPLPlanner::run(const geometry_msgs::msg::PoseStamped & start,
                                     const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path plan;
  plan.header.stamp = _node->now();
  plan.header.frame_id = start.header.frame_id;
  geometry_msgs::msg::PoseStamped pose;

  _ss->setStateValidityChecker(
    [this, goal](const ob::State *s) {
        return this->isStateValid(_costmap, _robot_footprint, goal, s);
    });

  // set the start and goal states
  ob::ScopedState<ob::SE2StateSpace> ss_start(_state_space), ss_goal(_state_space);
  {
    poseStampedToScopedState(start, ss_start);
    poseStampedToScopedState(goal, ss_goal);

    if (!ss_start.satisfiesBounds()) {
      RCLCPP_WARN(_node->get_logger(), "Start state doesn't satisfied bounds");

      return plan;
    }

    if (!ss_goal.satisfiesBounds()) {
      RCLCPP_WARN(_node->get_logger(), "Goal state doesn't satisfied bounds");
      return plan;
    }

    _ss->setStartAndGoalStates(ss_start, ss_goal);
  }

  auto *planner = new og::RRTstar(_ss->getSpaceInformation());

  _ss->setPlanner(ob::PlannerPtr(planner));

//  ss.print();
  _ss->setup();

  ob::PlannerStatus solved = _ss->solve(0.5);

  if (solved) {
    og::PathGeometric path = _ss->getSolutionPath();

    RCLCPP_INFO(_node->get_logger(), "Found solution with %i states", path.getStateCount());

    path.interpolate(100);

    RCLCPP_DEBUG(_node->get_logger(), "After interpolation path has %i states",
                 path.getStateCount());

    //    path.printAsMatrix(std::cout);

    for (auto it = std::begin(path.getStates()); it != std::end(path.getStates()); ++it) {
      double x, y, th;
      getXYThetaFromState(_state_space, (*it), &x, &y, &th);
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = start.header.frame_id;
      ps.header.stamp = plan.header.stamp;
      ps.pose.position.x = x;
      ps.pose.position.y = y;
      ps.pose.position.z = 0.0;
      tf2::Quaternion quaternion;
      quaternion.setRPY(0, 0, th);
      ps.pose.orientation = tf2::toMsg(quaternion);

      plan.poses.push_back(ps);
    }
  } else {
    RCLCPP_WARN(_node->get_logger(), "Solution not found");
  }

  return plan;
}

bool OMPLPlanner::isStateValid(const nav2_costmap_2d::Costmap2D *constmap,
                               const std::vector<geometry_msgs::msg::Point> & robot_footprint,
                               const geometry_msgs::msg::PoseStamped & goal,
                               const ompl::base::State *state)
{
  // Check if the state is valid in the OMPL state space bounds
  if (!_ss->getSpaceInformation()->satisfiesBounds(state)) {
    return false;
  }

  const double wx(state->as<ob::SE2StateSpace::StateType>()->getX());
  const double wy(state->as<ob::SE2StateSpace::StateType>()->getY());

  if (std::hypot(goal.pose.position.x - wx, goal.pose.position.y - wy) > 3.0) {
    return false;
  }

  int mx, my;
  constmap->worldToMapEnforceBounds(wx, wy, mx, my);

  const auto cost = constmap->getCost(mx, my);

  if (cost < 0 ||
      cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
      cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
      cost == nav2_costmap_2d::NO_INFORMATION) {
    return false;
  }

  const auto footprint_cost = FootprintCollisionChecker::footprintCostAtPose(
    constmap,
    robot_footprint,
    state->as<ob::SE2StateSpace::StateType>()->getX(),
    state->as<ob::SE2StateSpace::StateType>()->getY(),
    state->as<ob::SE2StateSpace::StateType>()->getYaw());

  return footprint_cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
}

void OMPLPlanner::poseStampedToScopedState(const geometry_msgs::msg::PoseStamped & pose_stamped,
                                           ob::ScopedState<ob::SE2StateSpace> & scoped_state)
{
  scoped_state->setX(pose_stamped.pose.position.x);
  scoped_state->setY(pose_stamped.pose.position.y);
  scoped_state->setYaw(tf2::getYaw(pose_stamped.pose.orientation));

  scoped_state.enforceBounds();
}

void
OMPLPlanner::getXYThetaFromState(const ob::StateSpacePtr state_space, const ob::State *s, double *x,
                                 double *y,
                                 double *th)
{
  ob::ScopedState<> ss(state_space);
  ss = s;
  *x = ss[0];
  *y = ss[1];
  *th = ss[2];
}

}  // namespace smac_planner
