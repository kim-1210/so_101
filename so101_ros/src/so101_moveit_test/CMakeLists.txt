cmake_minimum_required(VERSION 3.22)
project(so101_moveit_test)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(shape_msgs REQUIRED)

add_executable(planning_scene_obstacle_demo src/planning_scene_obstacle_demo.cpp)
target_compile_features(planning_scene_obstacle_demo PUBLIC cxx_std_17)
ament_target_dependencies(
  planning_scene_obstacle_demo
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  shape_msgs
)

install(TARGETS planning_scene_obstacle_demo
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
