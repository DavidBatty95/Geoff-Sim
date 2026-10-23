#include "g1_manipulation/manipulation_server.hpp"

#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <functional>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

namespace g1_manipulation
{
namespace
{
constexpr double kMinimumExtent = 0.005;

std::vector<std::string> handTouchLinks(bool is_left)
{
  const std::string side = is_left ? "left" : "right";
  std::vector<std::string> links{side+"_wrist_yaw_link",side+"_hand_palm_link"};
  for (const auto *name : {"thumb_0","thumb_1","thumb_2","middle_0","middle_1","index_0","index_1"}) {
    links.push_back(side+"_hand_"+name+"_link");
  }
  return links;
}

Pose toPure(const geometry_msgs::msg::Pose & pose)
{
  return {{pose.position.x, pose.position.y, pose.position.z},
    {pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w}};
}

geometry_msgs::msg::Pose toRos(const Pose & pose)
{
  geometry_msgs::msg::Pose result;
  result.position.x = pose.position.x;
  result.position.y = pose.position.y;
  result.position.z = pose.position.z;
  result.orientation.x = pose.orientation.x;
  result.orientation.y = pose.orientation.y;
  result.orientation.z = pose.orientation.z;
  result.orientation.w = pose.orientation.w;
  return result;
}
}  // namespace

std::optional<vision_msgs::msg::Detection3D> findDetection(
  const vision_msgs::msg::Detection3DArray & objects, const std::string & object_id)
{
  if (object_id.empty()) {
    return std::nullopt;
  }
  for (const auto & detection : objects.detections) {
    if (!detection.results.empty() &&
      detection.results.front().hypothesis.class_id == object_id)
    {
      return detection;
    }
  }
  return std::nullopt;
}

ManipulationServer::ManipulationServer(const rclcpp::NodeOptions & options)
: Node("g1_manipulation_server", options)
{
  object_timeout_s_ = declare_parameter<double>("object_timeout_s", 1.0);
  approach_height_m_ = declare_parameter<double>("approach_height_m", 0.15);
  lift_height_m_ = declare_parameter<double>("lift_height_m", 0.15);
  collision_padding_m_ = declare_parameter<double>("collision_padding_m", 0.01);
  place_tolerance_m_ = declare_parameter<double>("place_tolerance_m", 0.08);
  planning_time_s_ = declare_parameter<double>("planning_time_s", 5.0);
  planning_attempts_ = declare_parameter<int>("planning_attempts", 3);
  velocity_scaling_ = declare_parameter<double>("velocity_scaling", 0.25);
  service_timeout_ = std::chrono::milliseconds(
    declare_parameter<int>("gripper_service_timeout_ms", 8000));
  left_tip_link_ = declare_parameter<std::string>("left_tip_link", "left_wrist_yaw_link");
  right_tip_link_ = declare_parameter<std::string>("right_tip_link", "right_wrist_yaw_link");
  left_object_id_ = declare_parameter<std::string>("left_object_id", "red_cube");
  right_object_id_ = declare_parameter<std::string>("right_object_id", "red_cube");
  use_object_grasp_service_ = declare_parameter<bool>("use_object_grasp_service", true);
  object_grasp_client_ = create_client<g1_msgs::srv::GraspObject>("/g1_sim_manipulation_io/grasp_object");

  const auto offset = declare_parameter<std::vector<double>>(
    "tip_to_object_offset", {-0.135, -0.04, 0.0});
  const auto rpy = declare_parameter<std::vector<double>>(
    "grasp_rpy", {0.0, 0.0, 0.0});
  if (offset.size() != 3 || rpy.size() != 3) {
    throw std::invalid_argument("tip_to_object_offset and grasp_rpy require exactly 3 values");
  }
  tip_to_object_offset_ = {offset[0], offset[1], offset[2]};
  grasp_rpy_ = {rpy[0], rpy[1], rpy[2]};
  if (!validVector(tip_to_object_offset_) || !validVector({rpy[0], rpy[1], rpy[2]}) ||
    object_timeout_s_ <= 0.0 || approach_height_m_ <= 0.0 || lift_height_m_ <= 0.0 ||
    collision_padding_m_ < 0.0 || place_tolerance_m_ < 0.0 || planning_time_s_ <= 0.0 ||
    planning_attempts_ <= 0 || velocity_scaling_ <= 0.0 || velocity_scaling_ > 1.0 ||
    service_timeout_.count() <= 0 ||
    left_tip_link_.empty() || right_tip_link_.empty() ||
    left_object_id_.empty() || right_object_id_.empty())
  {
    throw std::invalid_argument("invalid manipulation parameter");
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  objects_sub_ = create_subscription<vision_msgs::msg::Detection3DArray>(
    "/objects", rclcpp::QoS(1).reliable(),
    std::bind(&ManipulationServer::onObjects, this, std::placeholders::_1));

  for (const auto & side : {std::string("left"), std::string("right")}) {
    gripper_clients_["grasp_" + side] = create_client<std_srvs::srv::Trigger>(
      "/g1_sim_manipulation_io/grasp_" + side);
    gripper_clients_["release_" + side] = create_client<std_srvs::srv::Trigger>(
      "/g1_sim_manipulation_io/release_" + side);
    gripper_clients_["verify_" + side] = create_client<std_srvs::srv::Trigger>(
      "/g1_sim_manipulation_io/verify_" + side);
  }
}

ManipulationServer::~ManipulationServer()
{
  while (active_goals_.load() != 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

void ManipulationServer::initialize()
{
  for (const auto & name : {"left_arm", "right_arm", "both_arms"}) {
    auto group = std::make_shared<MoveGroup>(shared_from_this(), name);
    group->setPlanningTime(planning_time_s_);
    group->setNumPlanningAttempts(planning_attempts_);
    group->setMaxVelocityScalingFactor(velocity_scaling_);
    group->setMaxAccelerationScalingFactor(velocity_scaling_);
    groups_.emplace(name, std::move(group));
  }
  planning_frame_ = groups_.at("left_arm")->getPlanningFrame();
  if (planning_frame_.empty()) {
    throw std::runtime_error("MoveIt returned an empty planning frame");
  }

  const auto accept_goal = [this](const rclcpp_action::GoalUUID &, const auto &) {
      return acquire() ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
             rclcpp_action::GoalResponse::REJECT;
    };
  const auto accept_cancel = [](const auto &) {return rclcpp_action::CancelResponse::ACCEPT;};

  pick_server_ = rclcpp_action::create_server<Pick>(
    this, "~/pick", accept_goal, accept_cancel,
    [this](const std::shared_ptr<GoalHandle<Pick>> & handle) {
      std::thread([this, handle]() {
        runGoal<Pick>(handle, [this, handle]() {executePick(handle);});
      }).detach();
    });
  place_server_ = rclcpp_action::create_server<Place>(
    this, "~/place", accept_goal, accept_cancel,
    [this](const std::shared_ptr<GoalHandle<Place>> & handle) {
      std::thread([this, handle]() {
        runGoal<Place>(handle, [this, handle]() {executePlace(handle);});
      }).detach();
    });
  posture_server_ = rclcpp_action::create_server<SetArmPosture>(
    this, "~/set_arm_posture", accept_goal, accept_cancel,
    [this](const std::shared_ptr<GoalHandle<SetArmPosture>> & handle) {
      std::thread([this, handle]() {
        runGoal<SetArmPosture>(handle, [this, handle]() {executePosture(handle);});
      }).detach();
    });

  RCLCPP_WARN(
    get_logger(),
    "Simulation-first manipulation ready in '%s'. This node does not acquire controller "
    "authority and makes no hardware-operation claim.", planning_frame_.c_str());
}

bool ManipulationServer::acquire()
{
  bool expected = false;
  return busy_.compare_exchange_strong(expected, true);
}

ManipulationServer::MoveGroup * ManipulationServer::groupFor(const std::string & name)
{
  const auto found = groups_.find(name);
  return found == groups_.end() ? nullptr : found->second.get();
}

void ManipulationServer::onObjects(
  vision_msgs::msg::Detection3DArray::ConstSharedPtr message)
{
  std::lock_guard<std::mutex> lock(objects_mutex_);
  objects_ = *message;
}

std::optional<vision_msgs::msg::Detection3D> ManipulationServer::freshDetection(
  const std::string & object_id)
{
  vision_msgs::msg::Detection3DArray snapshot;
  {
    std::lock_guard<std::mutex> lock(objects_mutex_);
    snapshot = objects_;
  }
  const rclcpp::Time stamp(snapshot.header.stamp);
  if (stamp.nanoseconds() == 0) {
    return std::nullopt;
  }
  const double age = (now() - stamp).seconds();
  if (!std::isfinite(age) || age < -0.1 || age > object_timeout_s_) {
    return std::nullopt;
  }
  return findDetection(snapshot, object_id);
}

std::optional<geometry_msgs::msg::Pose> ManipulationServer::toPlanningFrame(
  const geometry_msgs::msg::Pose & pose, const std::string & frame_id)
{
  if (!validPose(toPure(pose)) || frame_id.empty()) {
    return std::nullopt;
  }
  if (frame_id == planning_frame_) {
    return pose;
  }
  geometry_msgs::msg::PoseStamped input;
  input.header.frame_id = frame_id;
  input.header.stamp = rclcpp::Time(0);
  input.pose = pose;
  try {
    return tf_buffer_->transform(input, planning_frame_, std::chrono::milliseconds(500)).pose;
  } catch (const tf2::TransformException & error) {
    RCLCPP_ERROR(get_logger(), "pose transform failed: %s", error.what());
    return std::nullopt;
  }
}

moveit_msgs::msg::CollisionObject ManipulationServer::makeCollisionObject(
  const vision_msgs::msg::Detection3D & detection,
  const geometry_msgs::msg::Pose & planning_pose) const
{
  moveit_msgs::msg::CollisionObject object;
  object.id = detection.results.front().hypothesis.class_id;
  object.header.frame_id = planning_frame_;
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {
    std::max(kMinimumExtent, detection.bbox.size.x + 2.0 * collision_padding_m_),
    std::max(kMinimumExtent, detection.bbox.size.y + 2.0 * collision_padding_m_),
    std::max(kMinimumExtent, detection.bbox.size.z + 2.0 * collision_padding_m_)};
  object.primitives.push_back(box);
  object.primitive_poses.push_back(planning_pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return object;
}

geometry_msgs::msg::Pose ManipulationServer::tipGoal(
  const geometry_msgs::msg::Pose & object, bool is_left) const
{
  auto offset = tip_to_object_offset_;
  if (is_left) {offset.y = -offset.y;}
  return toRos(graspTipPose(toPure(object), offset, grasp_rpy_, is_left));
}

void ManipulationServer::setStartStateInBounds(MoveGroup & group)
{
  const auto current = group.getCurrentState();
  if (!current) {
    group.setStartStateToCurrentState();
    return;
  }
  moveit::core::RobotState bounded(*current);
  bounded.enforceBounds(bounded.getRobotModel()->getJointModelGroup(group.getName()));
  group.setStartState(bounded);
}

bool ManipulationServer::moveToPose(
  MoveGroup & group, const geometry_msgs::msg::Pose & pose, const std::string & tip,
  const std::string & phase)
{
  setStartStateInBounds(group);
  group.clearPoseTargets();
  if (!group.setPoseTarget(pose, tip)) {
    RCLCPP_ERROR(get_logger(), "%s: invalid pose target", phase.c_str());
    return false;
  }
  MoveGroup::Plan plan;
  const auto planned = group.plan(plan);
  if (planned != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "%s: planning failed (%d)", phase.c_str(), planned.val);
    return false;
  }
  const auto executed = group.execute(plan);
  if (executed != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(
      get_logger(), "%s: execution failed (%d); controller authority must already exist",
      phase.c_str(), executed.val);
    return false;
  }
  return true;
}

bool ManipulationServer::moveToNamed(MoveGroup & group, const std::string & target)
{
  if (target.empty()) {
    return false;
  }
  setStartStateInBounds(group);
  group.clearPoseTargets();
  if (!group.setNamedTarget(target)) {
    return false;
  }
  MoveGroup::Plan plan;
  if (group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    return false;
  }
  return group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
}

bool ManipulationServer::callGripper(
  const std::string & side, bool grasp, std::string & reason, const std::string & object_id)
{
  if (grasp && use_object_grasp_service_) {
    if (object_id.empty() || !object_grasp_client_->wait_for_service(service_timeout_)) {
      reason = "object-selectable grasp service unavailable or object identifier empty";
      return false;
    }
    auto request = std::make_shared<g1_msgs::srv::GraspObject::Request>();
    request->arm = side;
    request->object_id = object_id;
    auto future = object_grasp_client_->async_send_request(request);
    if (future.wait_for(service_timeout_) != std::future_status::ready) {
      reason = "object-selectable grasp service timed out";
      return false;
    }
    const auto response = future.get();
    reason = response->message;
    return response->success;
  }
  const std::string key = std::string(grasp ? "grasp_" : "release_") + side;
  const auto client = gripper_clients_.at(key);
  if (!client->wait_for_service(service_timeout_)) {
    reason = key + " service unavailable";
    return false;
  }
  auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  if (future.wait_for(service_timeout_) != std::future_status::ready) {
    reason = key + " service timed out";
    return false;
  }
  const auto response = future.get();
  reason = response->message;
  return response->success;
}

bool ManipulationServer::verifyGripper(const std::string & side, std::string & reason)
{
  const auto client = gripper_clients_.at("verify_" + side);
  if (!client->wait_for_service(service_timeout_)) {
    reason = "physical grasp feedback service unavailable";
    return false;
  }
  auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  if (future.wait_for(service_timeout_) != std::future_status::ready) {
    reason = "physical grasp verification timed out";
    return false;
  }
  const auto response = future.get();
  reason = response->message;
  return response->success;
}

bool ManipulationServer::attachObject(
  const ArmContext & arm, const moveit_msgs::msg::CollisionObject & object)
{
  moveit_msgs::msg::AttachedCollisionObject attached;
  attached.link_name = arm.tip_link;
  attached.object = object;
  attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;
  attached.touch_links = handTouchLinks(arm.is_left);
  return planning_scene_.applyAttachedCollisionObject(attached);
}

bool ManipulationServer::detachObject(const std::string & object_id)
{
  moveit_msgs::msg::AttachedCollisionObject remove;
  remove.object.id = object_id;
  remove.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  return planning_scene_.applyAttachedCollisionObject(remove);
}

void ManipulationServer::executePick(const std::shared_ptr<GoalHandle<Pick>> & handle)
{
  const auto goal = handle->get_goal();
  auto result = std::make_shared<Pick::Result>();
  auto feedback = std::make_shared<Pick::Feedback>();
  const auto arm = resolveArm(goal->arm, left_tip_link_, right_tip_link_);
  if (!arm || goal->object_id.empty()) {
    result->message = "arm must be left/right and object_id must not be empty";
    handle->abort(result);
    return;
  }
  const auto & supported_object = arm->is_left ? left_object_id_ : right_object_id_;
  if (!use_object_grasp_service_ && goal->object_id != supported_object) {
    result->message = arm->side + " gripper is configured for object " + supported_object;
    handle->abort(result);
    return;
  }
  MoveGroup & group = *groupFor(arm->group);
  bool grasp_attempted = false;
  bool attached = false;
  bool world_removed = false;
  std::optional<moveit_msgs::msg::CollisionObject> collision;

  const auto finish_bad = [&](const std::string & phase, const std::string & why, bool canceled) {
      std::string cleanup_error;
      if (grasp_attempted && !callGripper(arm->side, false, cleanup_error)) {
        cleanup_error = "; release cleanup failed: " + cleanup_error;
      } else {
        cleanup_error.clear();
      }
      const bool was_attached = attached;
      if (was_attached && !detachObject(goal->object_id)) {
        cleanup_error += "; detach cleanup failed";
      }
      // Before attachment, restore the measured world copy removed for the final approach.
      // After attachment, cleanup removes it from the robot; /objects supplies a fresh world
      // pose on a retry rather than inventing a stale pose after a partial lift.
      if (collision && world_removed && !was_attached) {
        planning_scene_.applyCollisionObject(*collision);
      }
      result->success = false;
      result->message = phase + ": " + why + cleanup_error;
      canceled ? handle->canceled(result) : handle->abort(result);
    };
  const auto check_cancel = [&](const std::string & phase) {
      if (!handle->is_canceling()) {
        return false;
      }
      finish_bad(phase, "cancelled", true);
      return true;
    };

  feedback->phase = Pick::Feedback::PHASE_LOCATING;
  handle->publish_feedback(feedback);
  const auto detection = freshDetection(goal->object_id);
  if (!detection || detection->results.empty() || detection->bbox.size.x <= 0.0 ||
    detection->bbox.size.y <= 0.0 || detection->bbox.size.z <= 0.0)
  {
    finish_bad(feedback->phase, "no fresh valid detection", false);
    return;
  }
  std::string frame = detection->header.frame_id;
  if (frame.empty()) {
    std::lock_guard<std::mutex> lock(objects_mutex_);
    frame = objects_.header.frame_id;
  }
  const auto object_pose = toPlanningFrame(detection->results.front().pose.pose, frame);
  if (!object_pose) {
    finish_bad(feedback->phase, "object pose cannot be transformed", false);
    return;
  }
  collision = makeCollisionObject(*detection, *object_pose);
  if (!planning_scene_.applyCollisionObject(*collision)) {
    finish_bad(feedback->phase, "could not add collision object", false);
    return;
  }
  const auto grasp = tipGoal(*object_pose, arm->is_left);
  const auto pregrasp = toRos(above(toPure(grasp), approach_height_m_));

  if (check_cancel(Pick::Feedback::PHASE_PREGRASP)) {return;}
  feedback->phase = Pick::Feedback::PHASE_PREGRASP;
  handle->publish_feedback(feedback);
  if (!moveToPose(group, pregrasp, arm->tip_link, feedback->phase)) {
    finish_bad(feedback->phase, "motion failed", false);
    return;
  }

  if (check_cancel(Pick::Feedback::PHASE_APPROACH)) {return;}
  feedback->phase = Pick::Feedback::PHASE_APPROACH;
  handle->publish_feedback(feedback);
  moveit_msgs::msg::CollisionObject remove;
  remove.id = goal->object_id;
  remove.header.frame_id = planning_frame_;
  remove.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  if (!planning_scene_.applyCollisionObject(remove)) {
    finish_bad(feedback->phase, "could not remove target collision object", false);
    return;
  }
  world_removed = true;
  if (!moveToPose(group, grasp, arm->tip_link, feedback->phase)) {
    finish_bad(feedback->phase, "motion failed", false);
    return;
  }

  if (check_cancel(Pick::Feedback::PHASE_GRASP)) {return;}
  feedback->phase = Pick::Feedback::PHASE_GRASP;
  handle->publish_feedback(feedback);
  grasp_attempted = true;
  std::string service_reason;
  if (!callGripper(arm->side, true, service_reason, goal->object_id)) {
    finish_bad(feedback->phase, "grasp service failed: " + service_reason, false);
    return;
  }
  if (!attachObject(*arm, *collision)) {
    finish_bad(feedback->phase, "planning-scene attach failed", false);
    return;
  }
  attached = true;

  if (check_cancel(Pick::Feedback::PHASE_LIFT)) {return;}
  feedback->phase = Pick::Feedback::PHASE_LIFT;
  handle->publish_feedback(feedback);
  const auto lifted = toRos(above(toPure(grasp), lift_height_m_));
  if (!moveToPose(group, lifted, arm->tip_link, feedback->phase)) {
    finish_bad(feedback->phase, "motion failed", false);
    return;
  }

  if (!verifyGripper(arm->side, service_reason)) {
    finish_bad(feedback->phase, "physical lift verification failed: " + service_reason, false);
    return;
  }

  result->success = true;
  result->message = "picked " + goal->object_id + " with " + arm->side + " arm";
  handle->succeed(result);
}

void ManipulationServer::executePlace(const std::shared_ptr<GoalHandle<Place>> & handle)
{
  const auto goal = handle->get_goal();
  auto result = std::make_shared<Place::Result>();
  auto feedback = std::make_shared<Place::Feedback>();
  const auto arm = resolveArm(goal->arm, left_tip_link_, right_tip_link_);
  if (!arm) {
    result->message = "arm must be left or right";
    handle->abort(result);
    return;
  }
  MoveGroup & group = *groupFor(arm->group);
  const auto attached_objects = planning_scene_.getAttachedObjects();
  std::string held_id;
  double held_height = 0.0;
  std::optional<moveit_msgs::msg::CollisionObject> held_collision;
  for (const auto & entry : attached_objects) {
    const auto & attached = entry.second;
    if (attached.link_name == arm->tip_link && !attached.object.primitives.empty() &&
      attached.object.primitives.front().dimensions.size() == 3)
    {
      held_id = entry.first;
      held_height = std::max(
        kMinimumExtent,
        attached.object.primitives.front().dimensions[2] - 2.0 * collision_padding_m_);
      held_collision = attached.object;
      break;
    }
  }
  if (held_id.empty() || held_height <= 0.0) {
    result->message = "the selected arm has no valid attached object";
    handle->abort(result);
    return;
  }

  std::optional<vision_msgs::msg::Detection3D> surface;
  std::optional<geometry_msgs::msg::Pose> target;
  if (!goal->surface_object_id.empty()) {
    surface = freshDetection(goal->surface_object_id);
    if (surface && !surface->results.empty() && surface->bbox.size.z > 0.0) {
      std::string frame = surface->header.frame_id;
      if (frame.empty()) {
        std::lock_guard<std::mutex> lock(objects_mutex_);
        frame = objects_.header.frame_id;
      }
      target = toPlanningFrame(surface->results.front().pose.pose, frame);
      if (target) {
        target->position.z = placementCenterHeight(
          target->position.z, surface->bbox.size.z, held_height);
      }
    }
  } else {
    target = toPlanningFrame(goal->pose.pose, goal->pose.header.frame_id);
  }
  if (!target || !validPose(toPure(*target))) {
    result->message = "preplace: invalid, stale, or untransformable place target";
    handle->abort(result);
    return;
  }

  bool released = false;
  bool detached = false;
  const auto finish_bad = [&](const std::string & phase, const std::string & why, bool canceled) {
      std::string cleanup;
      if (released && !detached && !detachObject(held_id)) {
        cleanup = "; detach cleanup failed";
      }
      result->success = false;
      result->message = phase + ": " + why + cleanup;
      canceled ? handle->canceled(result) : handle->abort(result);
    };
  const auto check_cancel = [&](const std::string & phase) {
      if (!handle->is_canceling()) {return false;}
      finish_bad(phase, "cancelled", true);
      return true;
    };

  const auto lower = tipGoal(*target, arm->is_left);
  const auto preplace = toRos(above(toPure(lower), approach_height_m_));
  if (check_cancel(Place::Feedback::PHASE_PREPLACE)) {return;}
  feedback->phase = Place::Feedback::PHASE_PREPLACE;
  handle->publish_feedback(feedback);
  if (!moveToPose(group, preplace, arm->tip_link, feedback->phase)) {
    finish_bad(feedback->phase, "motion failed", false);
    return;
  }

  if (check_cancel(Place::Feedback::PHASE_LOWER)) {return;}
  feedback->phase = Place::Feedback::PHASE_LOWER;
  handle->publish_feedback(feedback);
  if (!moveToPose(group, lower, arm->tip_link, feedback->phase)) {
    finish_bad(feedback->phase, "motion failed", false);
    return;
  }

  if (check_cancel(Place::Feedback::PHASE_RELEASE)) {return;}
  feedback->phase = Place::Feedback::PHASE_RELEASE;
  handle->publish_feedback(feedback);
  std::string service_reason;
  if (!callGripper(arm->side, false, service_reason)) {
    finish_bad(feedback->phase, "release service failed: " + service_reason, false);
    return;
  }
  released = true;
  if (!detachObject(held_id)) {
    finish_bad(feedback->phase, "planning-scene detach failed", false);
    return;
  }
  detached = true;
  held_collision->header.frame_id = planning_frame_;
  held_collision->primitive_poses = {*target};
  held_collision->operation = moveit_msgs::msg::CollisionObject::ADD;
  if (!planning_scene_.applyCollisionObject(*held_collision)) {
    finish_bad(feedback->phase, "could not add released object to the planning scene", false);
    return;
  }

  if (check_cancel(Place::Feedback::PHASE_RETREAT)) {return;}
  feedback->phase = Place::Feedback::PHASE_RETREAT;
  handle->publish_feedback(feedback);
  if (!moveToPose(group, preplace, arm->tip_link, feedback->phase)) {
    finish_bad(feedback->phase, "motion failed", false);
    return;
  }

  if (const auto landed = freshDetection(held_id)) {
    std::string frame = landed->header.frame_id;
    if (frame.empty()) {
      std::lock_guard<std::mutex> lock(objects_mutex_);
      frame = objects_.header.frame_id;
    }
    const auto final_pose = toPlanningFrame(landed->results.front().pose.pose, frame);
    if (!final_pose) {
      finish_bad(feedback->phase, "fresh final pose could not be transformed", false);
      return;
    }
    const double dx = final_pose->position.x - target->position.x;
    const double dy = final_pose->position.y - target->position.y;
    const double dz = final_pose->position.z - target->position.z;
    const double error = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (error > place_tolerance_m_) {
      finish_bad(feedback->phase, "final object position outside tolerance", false);
      return;
    }
  }

  result->success = true;
  result->message = "placed " + held_id + " with " + arm->side + " arm";
  handle->succeed(result);
}

void ManipulationServer::executePosture(
  const std::shared_ptr<GoalHandle<SetArmPosture>> & handle)
{
  const auto goal = handle->get_goal();
  auto result = std::make_shared<SetArmPosture::Result>();
  if (!validPostureGroup(goal->group) || goal->named_target.empty()) {
    result->message = "group must be left_arm, right_arm, or both_arms and target must not be empty";
    handle->abort(result);
    return;
  }
  if (handle->is_canceling()) {
    result->message = "cancelled";
    handle->canceled(result);
    return;
  }
  MoveGroup & group = *groupFor(goal->group);
  const auto names = group.getNamedTargets();
  if (std::find(names.begin(), names.end(), goal->named_target) == names.end()) {
    result->message = "named target is not defined for " + goal->group;
    handle->abort(result);
    return;
  }
  if (!moveToNamed(group, goal->named_target)) {
    result->message = "planning or execution failed for " + goal->named_target;
    handle->abort(result);
    return;
  }
  result->success = true;
  result->message = goal->group + " reached " + goal->named_target;
  handle->succeed(result);
}

}  // namespace g1_manipulation
