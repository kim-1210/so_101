#include <array>
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace
{
template <typename T>
T param(const rclcpp::Node::SharedPtr& node, const std::string& name, const T& default_value)
{
  if (!node->has_parameter(name))
  {
    node->declare_parameter<T>(name, default_value);
  }
  return node->get_parameter(name).get_value<T>();
}

moveit_msgs::msg::CollisionObject makeBox(
  const std::string& id,
  const std::string& frame_id,
  const std::array<double, 3>& size,
  double x,
  double y,
  double z)
{
  moveit_msgs::msg::CollisionObject object;
  object.id = id;
  object.header.frame_id = frame_id;

  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions.resize(3);
  box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = size[0];
  box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = size[1];
  box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = size[2];

  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;

  object.primitives.push_back(box);
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return object;
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("so101_planning_scene_obstacle_demo", options);
  auto logger = node->get_logger();

  const std::string group_name = param<std::string>(node, "group_name", "arm");
  const std::string ee_link = param<std::string>(node, "end_effector_link", "gripper_tcp");
  const std::string planning_frame = param<std::string>(node, "planning_frame", "base");
  const std::string target_type = param<std::string>(node, "target_type", "joint");
  const bool execute = param<bool>(node, "execute", false);
  const bool publish_display_trajectory = param<bool>(node, "publish_display_trajectory", true);
  const double hold_seconds = param<double>(node, "hold_seconds", 8.0);

  const double goal_x = param<double>(node, "goal_x", 0.15);
  const double goal_y = param<double>(node, "goal_y", -0.30);
  const double goal_z = param<double>(node, "goal_z", 0.18);
  const double joint_rotation = param<double>(node, "joint_rotation", -0.90);
  const double joint_pitch = param<double>(node, "joint_pitch", 0.75);
  const double joint_elbow = param<double>(node, "joint_elbow", -0.95);
  const double joint_wrist_pitch = param<double>(node, "joint_wrist_pitch", 0.55);
  const double joint_wrist_roll = param<double>(node, "joint_wrist_roll", 0.0);

  const double obstacle_x = param<double>(node, "obstacle_x", 0.20);
  const double obstacle_y = param<double>(node, "obstacle_y", -0.18);
  const double obstacle_z = param<double>(node, "obstacle_z", 0.10);
  const double obstacle_size_x = param<double>(node, "obstacle_size_x", 0.08);
  const double obstacle_size_y = param<double>(node, "obstacle_size_y", 0.14);
  const double obstacle_size_z = param<double>(node, "obstacle_size_z", 0.16);

  const double planning_time = param<double>(node, "planning_time", 10.0);
  const int planning_attempts = param<int>(node, "planning_attempts", 10);
  const double velocity_scale = param<double>(node, "velocity_scale", 0.15);
  const double acceleration_scale = param<double>(node, "acceleration_scale", 0.15);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  moveit::planning_interface::MoveGroupInterface move_group(node, group_name);
  moveit::planning_interface::PlanningSceneInterface planning_scene;
  auto display_publisher = node->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
    "/display_planned_path", rclcpp::QoS(1).transient_local().reliable());

  move_group.setEndEffectorLink(ee_link);
  move_group.setPoseReferenceFrame(planning_frame);
  move_group.setPlanningTime(planning_time);
  move_group.setNumPlanningAttempts(planning_attempts);
  move_group.setMaxVelocityScalingFactor(velocity_scale);
  move_group.setMaxAccelerationScalingFactor(acceleration_scale);

  const std::string obstacle_id = "so101_test_box_obstacle";
  planning_scene.removeCollisionObjects({ obstacle_id });
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  const auto obstacle = makeBox(
    obstacle_id,
    planning_frame,
    { obstacle_size_x, obstacle_size_y, obstacle_size_z },
    obstacle_x,
    obstacle_y,
    obstacle_z);
  planning_scene.applyCollisionObject(obstacle);
  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_INFO(
    logger,
    "Added obstacle '%s' in frame '%s': center=(%.3f, %.3f, %.3f), size=(%.3f, %.3f, %.3f)",
    obstacle_id.c_str(), planning_frame.c_str(), obstacle_x, obstacle_y, obstacle_z,
    obstacle_size_x, obstacle_size_y, obstacle_size_z);

  move_group.setStartStateToCurrentState();

  if (target_type == "pose")
  {
    move_group.setPositionTarget(goal_x, goal_y, goal_z, ee_link);
    RCLCPP_INFO(
      logger,
      "Planning for %s to position target in %s: (%.3f, %.3f, %.3f)",
      ee_link.c_str(), planning_frame.c_str(), goal_x, goal_y, goal_z);
  }
  else
  {
    std::map<std::string, double> joint_target = {
      { "Rotation", joint_rotation },
      { "Pitch", joint_pitch },
      { "Elbow", joint_elbow },
      { "Wrist_Pitch", joint_wrist_pitch },
      { "Wrist_Roll", joint_wrist_roll },
    };
    move_group.setJointValueTarget(joint_target);
    RCLCPP_INFO(
      logger,
      "Planning to joint target: Rotation=%.3f, Pitch=%.3f, Elbow=%.3f, Wrist_Pitch=%.3f, Wrist_Roll=%.3f",
      joint_rotation, joint_pitch, joint_elbow, joint_wrist_pitch, joint_wrist_roll);
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto result = move_group.plan(plan);

  if (result != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(
      logger,
      "Planning failed. Try moving the obstacle, changing joint_* values, or use target_type:=pose with reachable goal_x/goal_y/goal_z.");
    executor.cancel();
    if (spinner.joinable())
    {
      spinner.join();
    }
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(logger, "Planning succeeded with %zu trajectory points.", plan.trajectory_.joint_trajectory.points.size());

  if (publish_display_trajectory)
  {
    moveit_msgs::msg::DisplayTrajectory display_trajectory;
    display_trajectory.model_id = "so_arm";
    display_trajectory.trajectory_start = plan.start_state_;
    display_trajectory.trajectory.push_back(plan.trajectory_);
    display_publisher->publish(display_trajectory);
    RCLCPP_INFO(logger, "Published the planned path to /display_planned_path for RViz animation.");
  }

  if (execute)
  {
    RCLCPP_INFO(logger, "Executing planned trajectory.");
    const auto execute_result = move_group.execute(plan);
    if (execute_result != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(logger, "Execution failed.");
      executor.cancel();
      if (spinner.joinable())
      {
        spinner.join();
      }
      rclcpp::shutdown();
      return 2;
    }
    RCLCPP_INFO(logger, "Execution complete.");
  }
  else
  {
    RCLCPP_INFO(logger, "execute:=false, so the robot will not move. The planned path is displayed in RViz.");
  }

  if (hold_seconds > 0.0)
  {
    RCLCPP_INFO(logger, "Keeping the node alive for %.1f seconds so RViz can show the result.", hold_seconds);
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(hold_seconds)));
  }

  executor.cancel();
  if (spinner.joinable())
  {
    spinner.join();
  }
  rclcpp::shutdown();
  return 0;
}
