cmake_minimum_required(VERSION 3.8)
project(so101_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(nlohmann_json REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_executable(hardware_receive
  src/hardware_receive.cpp
)

add_executable(hardware_send
  src/hardware_send.cpp
)

add_executable(hardware_node
  src/hardware_node.cpp
)

# plugin이름 정의
add_library(so101_hardware SHARED
  src/so101_hardware_interface.cpp
)

target_include_directories(hardware_receive PRIVATE
  include
)

target_include_directories(hardware_send PRIVATE
  include
)

target_include_directories(hardware_node PRIVATE
  include
)

target_include_directories(so101_hardware PUBLIC
  include
)

ament_target_dependencies(hardware_receive
  rclcpp
)

ament_target_dependencies(hardware_send
  rclcpp
)

ament_target_dependencies(hardware_node
  rclcpp
  sensor_msgs
  std_msgs
)

ament_target_dependencies(so101_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

target_link_libraries(hardware_receive
  Boost::system
  nlohmann_json::nlohmann_json
)

target_link_libraries(hardware_send
  Boost::system
  nlohmann_json::nlohmann_json
)

target_link_libraries(hardware_node
  Boost::system
  nlohmann_json::nlohmann_json
)

target_link_libraries(so101_hardware
  Boost::system
  nlohmann_json::nlohmann_json
)

# ros2에게 plugin이 존재한다고 알리는 부분
pluginlib_export_plugin_description_file(
  hardware_interface
  so101_hardware.xml
)

install(
  TARGETS so101_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  hardware_receive
  hardware_send
  hardware_node
  DESTINATION lib/${PROJECT_NAME}
)

# 빌드 후 XML 파일을 설치 폴더로 복사
install(
  FILES
    so101_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
