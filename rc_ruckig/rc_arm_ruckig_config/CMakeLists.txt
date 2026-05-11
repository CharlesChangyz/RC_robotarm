cmake_minimum_required(VERSION 3.8)
project(rc_arm_ruckig_config)

find_package(ament_cmake REQUIRED)
find_package(arm_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ruckig REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(ruckig_trajectory_server src/ruckig_trajectory_server.cpp)
ament_target_dependencies(ruckig_trajectory_server
  arm_msgs
  rclcpp
  trajectory_msgs
)
target_link_libraries(ruckig_trajectory_server ruckig::ruckig)

install(
  TARGETS ruckig_trajectory_server
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
