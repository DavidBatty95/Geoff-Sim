#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "nav_2d_utils/tf_help.hpp"
#include "pluginlib/class_loader.hpp"
#include "unitree_social_costmap/validated_goal_checker.hpp"

class ValidatedGoalCheckerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("validated_goal_checker_test");
    node->declare_parameter("guard.xy_goal_tolerance", 0.45);
    node->declare_parameter("guard.yaw_goal_tolerance", 0.75);
    node->declare_parameter("original.xy_goal_tolerance", 0.45);
    node->declare_parameter("original.yaw_goal_tolerance", 0.75);
    guard.initialize(node, "guard", nullptr);
    original.initialize(node, "original", nullptr);
  }

  void TearDown() override
  {
    node.reset();
    rclcpp::shutdown();
  }

  geometry_msgs::msg::Pose pose(double x = 0., double y = 0.)
  {
    geometry_msgs::msg::Pose value;
    value.position.x = x;
    value.position.y = y;
    value.orientation.w = 1.;
    return value;
  }

  bool reached(const geometry_msgs::msg::Pose & query, const geometry_msgs::msg::Pose & target)
  {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "odom";
    goal.pose = target;
    guard.setGoalFromPath(goal, nullptr, "odom", rclcpp::Duration::from_seconds(.1));
    return guard.isGoalReached(query, target, velocity);
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  unitree_social_costmap::ValidatedGoalChecker guard;
  nav2_controller::SimpleGoalChecker original;
  geometry_msgs::msg::Twist velocity;
};

TEST_F(ValidatedGoalCheckerTest, FailedHumbleTransformCannotBecomeOriginArrival)
{
  auto buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  buffer->setUsingDedicatedThread(true);  // This test pre-populates all TF samples.
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "odom";
  transform.child_frame_id = "map";
  transform.transform.rotation.w = 1.;
  transform.header.stamp.sec = 10;
  ASSERT_TRUE(buffer->setTransform(transform, "test"));
  transform.header.stamp.nanosec = 100000000;
  ASSERT_TRUE(buffer->setTransform(transform, "test"));
  geometry_msgs::msg::PoseStamped goal, transformed;
  goal.header.frame_id = "map";
  goal.header.stamp.sec = 11;
  goal.pose = pose(-.75, -.19);
  auto tolerance = rclcpp::Duration::from_seconds(.1);
  ASSERT_FALSE(nav_2d_utils::transformPose(buffer, "odom", goal, transformed, tolerance));
  ASSERT_DOUBLE_EQ(transformed.pose.orientation.w, 1.);  // ROS defaults look valid!
  // Match the retained incident: robot is near odom origin but not the goal.
  auto robot = pose(.37, .13);
  EXPECT_TRUE(original.isGoalReached(robot, transformed.pose, velocity));
  ASSERT_FALSE(guard.setGoalFromPath(goal, buffer, "odom", tolerance));
  EXPECT_FALSE(guard.isGoalReached(robot, transformed.pose, velocity));
  EXPECT_FALSE(guard.isGoalReached(robot, goal.pose, velocity));
  // A newly available transform permits the unchanged real goal, never the
  // previous default output. This exercises recovery with the real TF buffer.
  transform.header.stamp = goal.header.stamp;
  ASSERT_TRUE(buffer->setTransform(transform, "test"));
  ASSERT_TRUE(guard.setGoalFromPath(goal, buffer, "odom", tolerance));
  EXPECT_FALSE(guard.isGoalReached(robot, transformed.pose, velocity));
  EXPECT_TRUE(guard.isGoalReached(goal.pose, transformed.pose, velocity));
}

TEST_F(ValidatedGoalCheckerTest, LegitimateOriginGoalStillWorks)
{
  EXPECT_TRUE(reached(pose(.1, .1), pose()));
}

TEST_F(ValidatedGoalCheckerTest, DistanceAndYawTolerancesAreUnchanged)
{
  EXPECT_FALSE(reached(pose(.46), pose()));
  auto rotated = pose(.2);
  rotated.orientation.z = std::sin(.8 / 2.);
  rotated.orientation.w = std::cos(.8 / 2.);
  EXPECT_FALSE(reached(rotated, pose()));
  EXPECT_TRUE(reached(pose(.2), pose()));
}

TEST_F(ValidatedGoalCheckerTest, InvalidStateResetsPriorPositionLatch)
{
  auto rotated = pose(.1);
  rotated.orientation.z = 1.;
  rotated.orientation.w = 0.;
  EXPECT_FALSE(reached(rotated, pose()));
  geometry_msgs::msg::Pose invalid;
  invalid.orientation.w = 0.;
  EXPECT_FALSE(reached(pose(), invalid));
  EXPECT_FALSE(reached(pose(2.), pose()));
}

TEST_F(ValidatedGoalCheckerTest, NonfiniteAndNonunitGeometryIsRejected)
{
  for (double invalid : {std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()})
  {
    auto bad = pose();
    bad.position.x = invalid;
    EXPECT_FALSE(reached(pose(), bad));
    EXPECT_FALSE(reached(bad, pose()));
    bad = pose();
    bad.orientation.z = invalid;
    EXPECT_FALSE(reached(pose(), bad));
  }
  auto bad = pose();
  bad.orientation.w = 2.;
  EXPECT_FALSE(reached(pose(), bad));
  EXPECT_FALSE(reached(bad, pose()));
}

TEST_F(ValidatedGoalCheckerTest, ValidTransformCanRecoverWithoutChangingGoal)
{
  geometry_msgs::msg::Pose invalid;
  invalid.orientation.w = 0.;
  EXPECT_FALSE(reached(pose(), invalid));
  EXPECT_FALSE(reached(pose(), pose(2.)));
  EXPECT_TRUE(reached(pose(2.), pose(2.)));
}

TEST_F(ValidatedGoalCheckerTest, PluginLoadsThroughNav2Interface)
{
  pluginlib::ClassLoader<nav2_core::GoalChecker> loader("nav2_core", "nav2_core::GoalChecker");
  auto checker = loader.createSharedInstance("unitree_social_costmap::ValidatedGoalChecker");
  checker->initialize(node, "loaded", nullptr);
  geometry_msgs::msg::Pose invalid;
  EXPECT_FALSE(checker->isGoalReached(pose(), invalid, velocity));
  auto * typed = dynamic_cast<unitree_social_costmap::ValidatedGoalChecker *>(checker.get());
  ASSERT_NE(typed, nullptr);
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "odom";
  goal.pose = pose();
  ASSERT_TRUE(typed->setGoalFromPath(goal, nullptr, "odom", rclcpp::Duration::from_seconds(.1)));
  EXPECT_TRUE(checker->isGoalReached(pose(), pose(), velocity));
}

TEST_F(ValidatedGoalCheckerTest, UncheckedArgumentCannotReplaceActualPathEndpoint)
{
  EXPECT_FALSE(reached(pose(), pose(2.)));
  EXPECT_FALSE(guard.isGoalReached(pose(), pose(), velocity));
  EXPECT_TRUE(guard.isGoalReached(pose(2.), pose(), velocity));
  guard.reset();
  EXPECT_FALSE(guard.isGoalReached(pose(2.), pose(2.), velocity));
}

TEST_F(ValidatedGoalCheckerTest, ControllerPluginRequiresMatchingCheckerAndNonemptyPath)
{
  pluginlib::ClassLoader<nav2_core::Controller> loader("nav2_core", "nav2_core::Controller");
  auto controller = loader.createSharedInstance("unitree_social_costmap::ValidatedDWBLocalPlanner");
  geometry_msgs::msg::PoseStamped query;
  EXPECT_THROW(controller->computeVelocityCommands(query, velocity, &original), std::exception);
  EXPECT_THROW(controller->setPlan(nav_msgs::msg::Path()), std::exception);
}
