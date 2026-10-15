#pragma once

#include "nav2_controller/plugins/simple_goal_checker.hpp"
#include "dwb_core/dwb_local_planner.hpp"

namespace unitree_social_costmap
{
// Humble's ControllerServer can pass a default Pose after transformPose fails.
// Pair this checker with ValidatedDWBLocalPlanner: a default Pose is valid ROS
// geometry, so validity alone cannot establish successful goal transformation.
class ValidatedGoalChecker : public nav2_controller::SimpleGoalChecker
{
public:
  void reset() override;
  bool setGoalFromPath(
    const geometry_msgs::msg::PoseStamped & goal,
    const std::shared_ptr<tf2_ros::Buffer> & tf, const std::string & frame,
    rclcpp::Duration tolerance);
  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose,
    const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) override;
private:
  geometry_msgs::msg::Pose checked_goal_;
  bool have_checked_goal_{false};
};

class ValidatedDWBLocalPlanner : public dwb_core::DWBLocalPlanner
{
public:
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
private:
  geometry_msgs::msg::PoseStamped path_goal_;
};
}  // namespace unitree_social_costmap
