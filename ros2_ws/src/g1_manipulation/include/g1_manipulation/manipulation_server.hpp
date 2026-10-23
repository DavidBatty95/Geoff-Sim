#ifndef G1_MANIPULATION__MANIPULATION_SERVER_HPP_
#define G1_MANIPULATION__MANIPULATION_SERVER_HPP_

#include <g1_msgs/action/pick.hpp>
#include <g1_msgs/action/place.hpp>
#include <g1_msgs/action/set_arm_posture.hpp>
#include <g1_msgs/srv/grasp_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <moveit_msgs/msg/collision_object.hpp>

#include "g1_manipulation/geometry.hpp"

namespace g1_manipulation
{

std::optional<vision_msgs::msg::Detection3D> findDetection(
  const vision_msgs::msg::Detection3DArray & objects, const std::string & object_id);

class ManipulationServer : public rclcpp::Node
{
public:
  explicit ManipulationServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ManipulationServer() override;

  // Must be called while this node is already spinning. Servers are advertised only on success.
  void initialize();

private:
  using Pick = g1_msgs::action::Pick;
  using Place = g1_msgs::action::Place;
  using SetArmPosture = g1_msgs::action::SetArmPosture;
  using MoveGroup = moveit::planning_interface::MoveGroupInterface;
  template<typename ActionT>
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;

  bool acquire();
  MoveGroup * groupFor(const std::string & name);
  void executePick(const std::shared_ptr<GoalHandle<Pick>> & handle);
  void executePlace(const std::shared_ptr<GoalHandle<Place>> & handle);
  void executePosture(const std::shared_ptr<GoalHandle<SetArmPosture>> & handle);
  void onObjects(vision_msgs::msg::Detection3DArray::ConstSharedPtr message);
  std::optional<vision_msgs::msg::Detection3D> freshDetection(const std::string & object_id);
  std::optional<geometry_msgs::msg::Pose> toPlanningFrame(
    const geometry_msgs::msg::Pose & pose, const std::string & frame_id);
  moveit_msgs::msg::CollisionObject makeCollisionObject(
    const vision_msgs::msg::Detection3D & detection,
    const geometry_msgs::msg::Pose & planning_pose) const;
  bool moveToPose(
    MoveGroup & group, const geometry_msgs::msg::Pose & pose, const std::string & tip,
    const std::string & phase);
  bool moveToNamed(MoveGroup & group, const std::string & target);
  bool verifyGripper(const std::string & side, std::string & reason);
  bool callGripper(const std::string & side, bool grasp, std::string & reason,
    const std::string & object_id = "");
  bool attachObject(
    const ArmContext & arm, const moveit_msgs::msg::CollisionObject & object);
  bool detachObject(const std::string & object_id);
  geometry_msgs::msg::Pose tipGoal(const geometry_msgs::msg::Pose & object, bool is_left) const;
  static void setStartStateInBounds(MoveGroup & group);

  template<typename ActionT, typename WorkT>
  void runGoal(const std::shared_ptr<GoalHandle<ActionT>> & handle, WorkT work)
  {
    active_goals_.fetch_add(1);
    try {
      work();
    } catch (const std::exception & error) {
      auto result = std::make_shared<typename ActionT::Result>();
      result->success = false;
      result->message = std::string("internal error: ") + error.what();
      RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
      if (handle->is_executing() || handle->is_canceling()) {
        handle->abort(result);
      }
    }
    busy_.store(false);
    active_goals_.fetch_sub(1);
  }

  std::map<std::string, std::shared_ptr<MoveGroup>> groups_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr objects_sub_;
  std::mutex objects_mutex_;
  vision_msgs::msg::Detection3DArray objects_;
  rclcpp_action::Server<Pick>::SharedPtr pick_server_;
  rclcpp_action::Server<Place>::SharedPtr place_server_;
  rclcpp_action::Server<SetArmPosture>::SharedPtr posture_server_;
  std::map<std::string, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> gripper_clients_;
  rclcpp::Client<g1_msgs::srv::GraspObject>::SharedPtr object_grasp_client_;
  bool use_object_grasp_service_{true};

  std::string planning_frame_;
  std::string left_tip_link_;
  std::string right_tip_link_;
  std::string left_object_id_;
  std::string right_object_id_;
  Vector3 tip_to_object_offset_;
  std::array<double, 3> grasp_rpy_{};
  double object_timeout_s_{1.0};
  double approach_height_m_{0.15};
  double lift_height_m_{0.15};
  double collision_padding_m_{0.01};
  double place_tolerance_m_{0.08};
  double planning_time_s_{5.0};
  int planning_attempts_{3};
  double velocity_scaling_{0.25};
  std::chrono::milliseconds service_timeout_{2000};
  std::atomic<bool> busy_{false};
  std::atomic<int> active_goals_{0};
};

}  // namespace g1_manipulation

#endif  // G1_MANIPULATION__MANIPULATION_SERVER_HPP_
