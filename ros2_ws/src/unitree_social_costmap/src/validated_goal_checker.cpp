#include "unitree_social_costmap/validated_goal_checker.hpp"

#include <cmath>
#include "pluginlib/class_list_macros.hpp"
#include "nav_2d_utils/tf_help.hpp"
#include "nav2_core/exceptions.hpp"

namespace unitree_social_costmap
{
namespace
{
bool validPose(const geometry_msgs::msg::Pose & pose)
{
  const auto & p = pose.position;
  const auto & q = pose.orientation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
         std::isfinite(norm) && std::abs(norm - 1.0) <= 1e-3;
}
}  // namespace

void ValidatedGoalChecker::reset()
{
  have_checked_goal_ = false;
  nav2_controller::SimpleGoalChecker::reset();
}

bool ValidatedGoalChecker::setGoalFromPath(
  const geometry_msgs::msg::PoseStamped & goal,
  const std::shared_ptr<tf2_ros::Buffer> & tf, const std::string & frame,
  rclcpp::Duration tolerance)
{
  geometry_msgs::msg::PoseStamped transformed;
  if (goal.header.frame_id.empty() || frame.empty() ||
    (goal.header.frame_id != frame && !tf) || !validPose(goal.pose) ||
    !nav_2d_utils::transformPose(tf, frame, goal, transformed, tolerance) ||
    !validPose(transformed.pose))
  {
    reset();
    return false;
  }
  checked_goal_ = transformed.pose;
  have_checked_goal_ = true;
  return true;
}

bool ValidatedGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose,
  const geometry_msgs::msg::Pose & /*goal_pose*/,
  const geometry_msgs::msg::Twist & velocity)
{
  if (!have_checked_goal_ || !validPose(query_pose)) {
    // Invalid transform output is never arrival evidence, even after the
    // underlying checker's XY latch has been satisfied by an earlier sample.
    reset();
    return false;
  }
  return nav2_controller::SimpleGoalChecker::isGoalReached(query_pose, checked_goal_, velocity);
}

void ValidatedDWBLocalPlanner::setPlan(const nav_msgs::msg::Path & path)
{
  path_goal_ = geometry_msgs::msg::PoseStamped();
  if (path.poses.empty()) {
    throw nav2_core::PlannerException("Cannot validate an empty path endpoint");
  }
  dwb_core::DWBLocalPlanner::setPlan(path);
  path_goal_ = path.poses.back();
  path_goal_.header.frame_id = path.header.frame_id;
}

geometry_msgs::msg::TwistStamped ValidatedDWBLocalPlanner::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  auto * checked = dynamic_cast<ValidatedGoalChecker *>(goal_checker);
  if (!checked) {
    throw nav2_core::PlannerException("ValidatedDWBLocalPlanner requires ValidatedGoalChecker");
  }
  if (pose.header.frame_id != costmap_ros_->getGlobalFrameID()) {
    checked->reset();
    throw nav2_core::PlannerException("Robot pose frame differs from checked goal frame");
  }
  if (!checked->setGoalFromPath(path_goal_, tf_, costmap_ros_->getGlobalFrameID(),
      rclcpp::Duration::from_seconds(costmap_ros_->getTransformTolerance())))
  {
    // ControllerServer's configured failure tolerance publishes zero velocity
    // and retries; isGoalReached must remain false during that interval.
    throw nav2_core::PlannerException("Path endpoint transform unavailable; arrival withheld");
  }
  return dwb_core::DWBLocalPlanner::computeVelocityCommands(pose, velocity, goal_checker);
}
}  // namespace unitree_social_costmap

PLUGINLIB_EXPORT_CLASS(unitree_social_costmap::ValidatedGoalChecker, nav2_core::GoalChecker)
PLUGINLIB_EXPORT_CLASS(unitree_social_costmap::ValidatedDWBLocalPlanner, nav2_core::Controller)
